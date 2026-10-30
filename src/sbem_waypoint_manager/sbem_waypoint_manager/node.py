"""ROS services for persistent map-aware named locations."""
import math
import sqlite3
import threading

from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from sbem_waypoint_interfaces.msg import Location as LocationMessage
from sbem_waypoint_interfaces.srv import (
    DeleteLocation, GetLocation, GetMap, GotoLocation, ListLocations, SaveLocation,
)
from .navigation import GoalDispatcher
from .storage import MapSnapshot, WaypointStore


def pose_values(pose):
    return (pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)


def make_pose(values):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = values[:3]
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = values[3:]
    return pose


class WaypointManager(Node):
    def __init__(self, **kwargs):
        super().__init__('waypoint_manager', **kwargs)
        defaults = {
            'database_path': '~/.ros/sbem_waypoints.sqlite3',
            'map_topic': 'map', 'global_frame': 'map', 'robot_frame': 'base_link',
            'nav_action_name': 'navigate_to_pose',
            'max_transform_age': 1.0, 'server_timeout': 2.0, 'acceptance_timeout': 5.0,
        }
        self.settings = {name: self.declare_parameter(name, default).value
                         for name, default in defaults.items()}
        for name in ('max_transform_age', 'server_timeout', 'acceptance_timeout'):
            value = self.settings[name]
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        for name in ('database_path', 'map_topic', 'global_frame', 'robot_frame',
                     'nav_action_name'):
            if not self.settings[name].strip():
                raise ValueError(f'{name} must not be empty')
        self._state_lock = threading.RLock()
        self._map = None
        self._map_id = ''
        self.store = WaypointStore(self.settings['database_path'])
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(
            self, NavigateToPose, self.settings['nav_action_name'],
            callback_group=ReentrantCallbackGroup())
        self.dispatcher = GoalDispatcher(self.nav_client, self.get_logger().info)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, self.settings['map_topic'], self._on_map, qos)
        self._waypoint_services = []
        for service_type, name, callback in (
            (SaveLocation, 'save_location', self._save),
            (GetLocation, 'get_location', self._get),
            (ListLocations, 'list_locations', self._list),
            (DeleteLocation, 'delete_location', self._delete),
            (GetMap, 'get_map', self._get_map),
        ):
            self._waypoint_services.append(self.create_service(
                service_type, '~/' + name, self._guard(callback)))
        # Waiting for acceptance must not block map updates, TF or action callbacks.
        self._waypoint_services.append(self.create_service(
            GotoLocation, '~/goto_location', self._guard(self._goto),
            callback_group=ReentrantCallbackGroup()))
        self.get_logger().info(f'Waypoint database: {self.settings["database_path"]}')

    @property
    def current_map_id(self):
        with self._state_lock:
            return self._map_id

    def _guard(self, callback):
        def invoke(request, response):
            try:
                return callback(request, response)
            except (ValueError, KeyError, sqlite3.Error, TransformException) as error:
                response.success = False
                response.message = str(error)
                self.get_logger().warning(response.message)
                return response
        return invoke

    def _on_map(self, message):
        try:
            grid = MapSnapshot(
                message.header.frame_id, message.info.width, message.info.height,
                message.info.resolution, pose_values(message.info.origin), tuple(message.data))
            if grid.frame_id != self.settings['global_frame']:
                raise ValueError('Received map frame does not match global_frame')
            map_id = grid.map_id
        except ValueError as error:
            with self._state_lock:
                self._map = None
                self._map_id = ''
                self.dispatcher.map_changed('')
            self.get_logger().warning(f'Cannot use received map: {error}')
            return
        with self._state_lock:
            changed = map_id != self._map_id
            self._map, self._map_id = grid, map_id
            self.dispatcher.map_changed(map_id)
        if changed:
            self.get_logger().info(f'Current map: {map_id}')

    def _resolve_map(self, requested):
        map_id = requested or self._map_id
        if not map_id:
            raise ValueError('No current map received; publish the Nav2 static map first')
        return map_id

    def _location_message(self, location):
        message = LocationMessage()
        message.name, message.map_id = location.name, location.map_id
        message.pose = PoseStamped()
        message.pose.header.frame_id = location.frame_id
        message.pose.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose = make_pose(location.pose)
        seconds, nanoseconds = divmod(location.captured_ns, 10**9)
        message.captured_at = TimeMessage(sec=seconds, nanosec=nanoseconds)
        message.map_matches_current = location.map_id == self._map_id
        return message

    def _save(self, request, response):
        with self._state_lock:
            self._resolve_map('')
            transform = self.tf_buffer.lookup_transform(
                self.settings['global_frame'], self.settings['robot_frame'], Time())
            stamp = transform.header.stamp
            captured_ns = stamp.sec * 10**9 + stamp.nanosec
            age = (self.get_clock().now().nanoseconds - captured_ns) / 1e9
            if age < 0 or age > self.settings['max_transform_age']:
                raise ValueError(f'Robot transform is stale or in the future (age={age:.3f}s)')
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            pose = (translation.x, translation.y, translation.z,
                    rotation.x, rotation.y, rotation.z, rotation.w)
            saved = self.store.save(request.name, self._map, pose, captured_ns, request.overwrite)
            response.location = self._location_message(saved)
        response.success, response.message = True, 'Location saved'
        return response

    def _get(self, request, response):
        with self._state_lock:
            location = self.store.get(request.name, self._resolve_map(request.map_id))
            response.location = self._location_message(location)
        response.success, response.message = True, 'Location retrieved'
        return response

    def _list(self, request, response):
        with self._state_lock:
            response.map_id = self._resolve_map(request.map_id)
            response.locations = [self._location_message(item)
                                  for item in self.store.list(response.map_id)]
        response.success, response.message = True, 'Locations listed'
        return response

    def _delete(self, request, response):
        with self._state_lock:
            self.store.delete(request.name, self._resolve_map(request.map_id))
        response.success, response.message = True, 'Location deleted'
        return response

    def _get_map(self, request, response):
        with self._state_lock:
            map_id = self._resolve_map(request.map_id)
            # The current map is available even before its first location is saved.
            grid = self._map if map_id == self._map_id else self.store.get_map(map_id)
            response.map_id = map_id
            response.map.header.frame_id = grid.frame_id
            response.map.header.stamp = self.get_clock().now().to_msg()
            response.map.info.width = grid.width
            response.map.info.height = grid.height
            response.map.info.resolution = float(grid.resolution)
            response.map.info.origin = make_pose(grid.origin)
            response.map.data = list(grid.data)
        response.success, response.message = True, 'Map retrieved'
        return response

    def _goto(self, request, response):
        with self._state_lock:
            map_id = self._resolve_map(request.map_id)
            if not self._map_id or map_id != self._map_id:
                raise ValueError('Location map does not match the current map; '
                                 'map switching is manual')
            location = self.store.get(request.name, map_id)
            if location.frame_id != self.settings['global_frame']:
                raise ValueError('Location frame does not match global_frame')
            if self.dispatcher.busy:
                raise ValueError('Navigation is busy with a pending or active goal')
        if not self.nav_client.wait_for_server(timeout_sec=self.settings['server_timeout']):
            raise ValueError('Nav2 NavigateToPose action server is unavailable')
        with self._state_lock:
            if map_id != self._map_id:
                raise ValueError('Map changed before navigation could be dispatched')
            goal = NavigateToPose.Goal()
            goal.pose = self._location_message(location).pose
            ticket = self.dispatcher.start(goal, map_id)
        if not ticket.event.wait(timeout=self.settings['acceptance_timeout']):
            self.dispatcher.timeout(ticket)
        response.success = ticket.success
        response.message = ticket.message
        response.goal_id = ticket.goal_id
        return response

    def destroy_node(self):
        # Best-effort cancellation; normal operation tracks goals until terminal results.
        self.dispatcher.map_changed('')
        self.store.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = WaypointManager()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.dispatcher.map_changed('')
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
