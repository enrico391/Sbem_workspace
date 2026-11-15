"""Explicit pose saving with real ROS messages and SQLite, without a Nav2 server."""
import importlib.util
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from sbem_waypoint_interfaces.srv import SaveLocationPose
from sbem_waypoint_manager.storage import MapSnapshot, WaypointStore


@pytest.fixture
def manager(tmp_path, monkeypatch):
    # Nav2 is unrelated to pose saving. Only replace its import when unavailable;
    # load a separate module so this does not affect the ROS integration tests.
    try:
        import nav2_msgs.action  # noqa: F401
    except ImportError:
        nav2 = ModuleType('nav2_msgs')
        action = ModuleType('nav2_msgs.action')
        action.NavigateToPose = object
        monkeypatch.setitem(sys.modules, 'nav2_msgs', nav2)
        monkeypatch.setitem(sys.modules, 'nav2_msgs.action', action)
    path = Path(__file__).parents[1] / 'sbem_waypoint_manager' / 'node.py'
    spec = importlib.util.spec_from_file_location('sbem_waypoint_manager.pose_test_node', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    instance = module.WaypointManager.__new__(module.WaypointManager)
    instance.settings = {'global_frame': 'map'}
    instance._state_lock = threading.RLock()
    instance._map = MapSnapshot('map', 1, 1, 0.05, (0., 0., 0., 0., 0., 0., 1.), (0,))
    instance._map_id = instance._map.map_id
    instance.store = WaypointStore(tmp_path / 'locations.db')
    monkeypatch.setattr(instance, 'get_clock', lambda: SimpleNamespace(
        now=lambda: Time(nanoseconds=10_000_000_000)))
    monkeypatch.setattr(instance, 'get_logger', lambda: SimpleNamespace(warning=lambda _: None))
    yield instance
    instance.store.close()


def save(manager, **kwargs):
    return manager._guard(manager._save_pose)(
        SaveLocationPose.Request(**kwargs), SaveLocationPose.Response())


@pytest.fixture
def pose():
    message = PoseStamped()
    message.header.frame_id = 'map'
    message.header.stamp.sec = 2
    message.header.stamp.nanosec = 123
    message.pose.position.x = 3.
    message.pose.position.y = 4.
    message.pose.position.z = 0.5
    message.pose.orientation.z = 0.6
    message.pose.orientation.w = 0.8
    return message


@pytest.mark.parametrize('explicit_map', [False, True])
def test_save_explicit_pose_without_tf_and_reopen_database(manager, pose, explicit_map, tmp_path):
    result = save(manager, name='desk', pose=pose,
                  map_id=manager.current_map_id if explicit_map else '')
    assert result.success, result.message
    assert result.location.name == 'desk'
    assert result.location.map_id == manager.current_map_id
    assert result.location.map_matches_current
    assert result.location.pose.header.frame_id == 'map'
    assert result.location.pose.header.stamp.sec == 10
    assert result.location.captured_at.sec == 2
    assert result.location.captured_at.nanosec == 123
    reopened = WaypointStore(tmp_path / 'locations.db')
    try:
        stored = reopened.get('desk', manager.current_map_id)
        assert stored.pose == (3., 4., 0.5, 0., 0., 0.6, 0.8)
        assert stored.captured_ns == 2_000_000_123
    finally:
        reopened.close()


def test_zero_timestamp_uses_current_time(manager, pose):
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0
    result = save(manager, name='desk', pose=pose)
    assert result.success, result.message
    assert result.location.captured_at.sec == 10
    assert result.location.captured_at.nanosec == 0


def test_duplicate_requires_overwrite_and_preserves_original(manager, pose):
    assert save(manager, name='desk', pose=pose).success
    pose.pose.position.x = 8.
    rejected = save(manager, name='desk', pose=pose)
    assert not rejected.success and 'already exists' in rejected.message
    assert manager.store.get('desk', manager.current_map_id).pose[0] == 3.
    assert save(manager, name='desk', pose=pose, overwrite=True).success
    assert manager.store.get('desk', manager.current_map_id).pose[0] == 8.


@pytest.mark.parametrize('invalid', [
    'missing_map', 'other_map', 'other_frame', 'empty_frame', 'empty_name',
    'nan_position', 'infinite_orientation', 'zero_quaternion', 'unnormalized_quaternion',
    'negative_timestamp',
])
def test_invalid_requests_do_not_write_locations(manager, pose, invalid):
    kwargs = {'name': 'desk', 'pose': pose}
    if invalid == 'missing_map':
        manager._map, manager._map_id = None, ''
    elif invalid == 'other_map':
        kwargs['map_id'] = 'another-map'
    elif invalid in ('other_frame', 'empty_frame'):
        pose.header.frame_id = 'odom' if invalid == 'other_frame' else ''
    elif invalid == 'empty_name':
        kwargs['name'] = ''
    elif invalid == 'nan_position':
        pose.pose.position.x = float('nan')
    elif invalid == 'infinite_orientation':
        pose.pose.orientation.w = float('inf')
    elif invalid == 'zero_quaternion':
        pose.pose.orientation.z = pose.pose.orientation.w = 0.
    elif invalid == 'unnormalized_quaternion':
        pose.pose.orientation.w = 2.
    elif invalid == 'negative_timestamp':
        pose.header.stamp.sec = -1
    result = save(manager, **kwargs)
    assert not result.success
    assert result.message
    assert manager.store.list(manager.current_map_id) == []
