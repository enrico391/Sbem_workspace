"""ROS integration against real services, TF and a controlled Nav2 action server."""
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs', reason='Nav2 messages are required for ROS integration')
pytest.importorskip('sbem_waypoint_interfaces.srv', reason='Build waypoint interfaces first')
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster

from sbem_waypoint_interfaces.srv import (
    DeleteLocation, GetLocation, GetMap, GotoLocation, ListLocations, SaveLocation,
)
from sbem_waypoint_manager.node import WaypointManager


class Harness:
    def __init__(self, tmp_path, *, max_age=1.0):
        self.manager = WaypointManager(parameter_overrides=[
            Parameter('database_path', value=str(tmp_path / 'locations.db')),
            Parameter('max_transform_age', value=max_age),
            Parameter('server_timeout', value=0.15),
            Parameter('acceptance_timeout', value=0.5),
        ])
        self.peer = Node('waypoint_test_peer')
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.manager)
        self.executor.add_node(self.peer)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.pool = ThreadPoolExecutor(max_workers=3)
        self.publisher = self.peer.create_publisher(
            OccupancyGrid, 'map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf = TransformBroadcaster(self.peer)
        self.server = None
        self.seen_goals = []
        self.completed = []
        self.accept = True
        self.accept_delay = 0.0
        self.finish = threading.Event()

    def wait(self, predicate, timeout=4.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        raise AssertionError('Timed out waiting for ROS condition')

    def call(self, service_type, endpoint, **kwargs):
        client = self.peer.create_client(service_type, '/waypoint_manager/' + endpoint,
                                         callback_group=ReentrantCallbackGroup())
        try:
            assert client.wait_for_service(timeout_sec=3.)
            future = client.call_async(service_type.Request(**kwargs))
            self.wait(future.done, 5.)
            return future.result()
        finally:
            self.peer.destroy_client(client)

    def publish_map(self, changed=False):
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.info.width = 2
        message.info.height = 2
        message.info.resolution = 0.05
        message.info.origin.orientation.w = 1.
        message.data = [0, 0, 100, 50 if changed else -1]
        before = self.manager.current_map_id
        self.publisher.publish(message)
        self.wait(lambda: self.manager.current_map_id and
                  (not changed or self.manager.current_map_id != before))

    def broadcast(self, age=0.):
        stamp = self.peer.get_clock().now().nanoseconds - int(age * 1e9)
        message = TransformStamped()
        message.header.frame_id = 'map'
        message.child_frame_id = 'base_link'
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
        message.transform.translation.x = 3.
        message.transform.translation.y = 4.
        message.transform.rotation.z = 0.6
        message.transform.rotation.w = 0.8
        self.tf.sendTransform(message)
        self.wait(lambda: self.manager.tf_buffer.can_transform(
            'map', 'base_link', rclpy.time.Time()))

    def save(self):
        self.publish_map()
        self.broadcast()
        result = self.call(SaveLocation, 'save_location', name='home')
        assert result.success, result.message
        return result

    def start_action_server(self):
        def goal(request):
            self.seen_goals.append(request)
            time.sleep(self.accept_delay)
            return GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT

        def execute(handle):
            deadline = time.monotonic() + 8.
            while time.monotonic() < deadline and not self.finish.is_set():
                if handle.is_cancel_requested:
                    handle.canceled()
                    self.completed.append(GoalStatus.STATUS_CANCELED)
                    return NavigateToPose.Result()
                time.sleep(0.01)
            handle.succeed()
            self.completed.append(GoalStatus.STATUS_SUCCEEDED)
            return NavigateToPose.Result()

        self.server = ActionServer(
            self.peer, NavigateToPose, 'navigate_to_pose', execute,
            goal_callback=goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.wait(self.manager.nav_client.server_is_ready)

    def close(self):
        self.finish.set()
        self.pool.shutdown(wait=True)
        self.wait(lambda: not self.manager.dispatcher.busy, timeout=5.)
        self.executor.shutdown(timeout_sec=5.)
        self.thread.join(timeout=5.)
        if self.server is not None:
            self.server.destroy()
        self.manager.destroy_node()
        self.peer.destroy_node()


@pytest.fixture
def harness(tmp_path):
    rclpy.init()
    instance = None
    try:
        instance = Harness(tmp_path)
        yield instance
    finally:
        if instance is not None:
            instance.close()
        rclpy.shutdown()


def test_services_capture_retrieve_delete_and_map_roundtrip(harness):
    missing = harness.call(SaveLocation, 'save_location', name='home')
    assert not missing.success
    harness.publish_map()
    no_tf = harness.call(SaveLocation, 'save_location', name='home')
    assert not no_tf.success
    harness.broadcast()
    saved = harness.call(SaveLocation, 'save_location', name='home')
    assert saved.success, saved.message
    assert saved.location.pose.pose.position.x == 3.
    assert saved.location.pose.pose.position.y == 4.
    assert saved.location.map_matches_current
    assert not harness.call(SaveLocation, 'save_location', name='home').success
    assert harness.call(SaveLocation, 'save_location', name='home', overwrite=True).success
    retrieved = harness.call(GetLocation, 'get_location', name='home')
    assert retrieved.success
    assert retrieved.location.pose.header.stamp.sec > 0
    listed = harness.call(ListLocations, 'list_locations')
    assert [entry.name for entry in listed.locations] == ['home']
    grid = harness.call(GetMap, 'get_map')
    assert grid.success
    assert list(grid.map.data) == [0, 0, 100, -1]
    original_id = grid.map_id
    harness.publish_map(changed=True)
    assert not harness.call(GetLocation, 'get_location', name='home').success
    old = harness.call(GetLocation, 'get_location', name='home', map_id=original_id)
    assert old.success and not old.location.map_matches_current
    assert harness.call(DeleteLocation, 'delete_location', name='home', map_id=original_id).success
    assert not harness.call(GetLocation, 'get_location', name='home', map_id=original_id).success


def test_stale_and_future_transforms_are_rejected(harness):
    harness.publish_map()
    harness.broadcast(age=5.)
    assert not harness.call(SaveLocation, 'save_location', name='old').success
    harness.broadcast(age=-5.)
    assert not harness.call(SaveLocation, 'save_location', name='future').success


def test_goto_acceptance_busy_and_map_change_cancel(harness):
    saved = harness.save()
    harness.start_action_server()
    result = harness.call(GotoLocation, 'goto_location', name='home')
    assert result.success, result.message
    assert len(result.goal_id) == 32
    goal = harness.seen_goals[0]
    assert goal.pose.header.frame_id == 'map'
    assert goal.pose.pose.position.x == 3.
    assert goal.pose.pose.orientation.z == 0.6
    assert goal.pose.pose.orientation.w == 0.8
    assert not harness.call(GotoLocation, 'goto_location', name='home').success
    harness.publish_map(changed=True)
    harness.wait(lambda: GoalStatus.STATUS_CANCELED in harness.completed)
    wrong_map = harness.call(GotoLocation, 'goto_location', name='home',
                             map_id=saved.location.map_id)
    assert not wrong_map.success


def test_goto_missing_server_and_rejected_goal(harness):
    harness.save()
    assert not harness.call(GotoLocation, 'goto_location', name='home').success
    harness.accept = False
    harness.start_action_server()
    result = harness.call(GotoLocation, 'goto_location', name='home')
    assert not result.success
    assert 'rejected' in result.message


def test_goto_timeout_cancels_late_accepted_goal(harness):
    harness.save()
    harness.accept_delay = 1.0
    harness.start_action_server()
    result = harness.call(GotoLocation, 'goto_location', name='home')
    assert not result.success
    assert 'unknown' in result.message
    harness.wait(lambda: GoalStatus.STATUS_CANCELED in harness.completed)
    harness.wait(lambda: not harness.manager.dispatcher.busy)


def test_map_change_during_acceptance_cancels_late_goal(harness):
    harness.save()
    harness.accept_delay = 1.0
    harness.start_action_server()
    future = harness.pool.submit(harness.call, GotoLocation, 'goto_location', name='home')
    harness.wait(lambda: len(harness.seen_goals) == 1)
    harness.publish_map(changed=True)
    assert not future.result(timeout=3.).success
    harness.wait(lambda: GoalStatus.STATUS_CANCELED in harness.completed)


def test_persistence_after_ros_node_restart(harness):
    saved = harness.save()
    settings = [Parameter(name, value=value) for name, value in harness.manager.settings.items()]
    harness.executor.remove_node(harness.manager)
    harness.manager.destroy_node()
    harness.manager = WaypointManager(parameter_overrides=settings)
    harness.executor.add_node(harness.manager)
    harness.wait(lambda: harness.manager.current_map_id == saved.location.map_id)
    restored = harness.call(GetLocation, 'get_location', name='home')
    assert restored.success, restored.message
    assert restored.location.pose.pose.position.x == 3.
    assert restored.location.pose.pose.orientation.z == 0.6
    assert restored.location.captured_at == saved.location.captured_at
    grid = harness.call(GetMap, 'get_map', map_id=saved.location.map_id)
    assert grid.success and list(grid.map.data) == [0, 0, 100, -1]


def test_invalid_map_clears_current_map(harness):
    harness.save()
    invalid = OccupancyGrid()
    invalid.header.frame_id = 'odom'
    harness.publisher.publish(invalid)
    harness.wait(lambda: not harness.manager.current_map_id)
    assert not harness.call(SaveLocation, 'save_location', name='bad').success
    assert not harness.call(GotoLocation, 'goto_location', name='home').success


def test_simultaneous_goto_rejects_second_pending_request(harness):
    harness.save()
    harness.accept_delay = 0.2
    harness.start_action_server()
    first = harness.pool.submit(harness.call, GotoLocation, 'goto_location', name='home')
    harness.wait(lambda: len(harness.seen_goals) == 1)
    second = harness.call(GotoLocation, 'goto_location', name='home')
    assert not second.success and 'busy' in second.message
    assert first.result(timeout=3.).success
    assert len(harness.seen_goals) == 1
