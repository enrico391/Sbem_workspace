# Copyright 2026 Enrico Moro
# SPDX-License-Identifier: Apache-2.0
"""Exercise the actual runner and OpenNav plugin against isolated ROS test servers."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import unittest

import rclpy
from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_prefix
from opennav_docking_msgs.action import DockRobot
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import SetBool


class AutonomyIntegrationTest(unittest.TestCase):
    """No physical robot nodes are used; each test launches the production executable."""

    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '177':
            raise RuntimeError('Integration tests require isolated ROS_DOMAIN_ID=177')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('autonomy_test_servers')
        self.level = 0.6
        self.charging = False
        self.publish_battery = True
        self.nav_active = False
        self.nav_terminal = False
        self.cancel_requests = 0
        self.dock_requests = []
        self.canceled_docks = 0
        self.dock_mode = 'success'
        self.release_dock = False
        self.state = None
        self.process = None
        self.output = tempfile.TemporaryFile(mode='w+')
        self.battery_pub = self.node.create_publisher(BatteryState, '/autonomy_test/battery', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.node.create_publisher(
            GoalStatusArray, '/autonomy_test/nav_pose/_action/status', qos)
        self.node.create_subscription(String, '/sbem_autonomy/state', self.on_state, qos)
        self.node.create_service(
            CancelGoal, '/autonomy_test/nav_pose/_action/cancel_goal', self.cancel_pose)
        self.node.create_service(
            CancelGoal, '/autonomy_test/nav_poses/_action/cancel_goal', self.cancel_poses)
        self.server = ActionServer(
            self.node, DockRobot, '/autonomy_test/dock', execute_callback=self.dock,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.node.create_timer(0.03, self.publish)
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def on_state(self, message):
        self.state = message.data

    def publish(self):
        if self.publish_battery:
            message = BatteryState()
            message.percentage = self.level
            message.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.charging else
                BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
            self.battery_pub.publish(message)
        if self.nav_active:
            status = GoalStatus()
            status.goal_info.goal_id.uuid = [1] * 16
            status.status = GoalStatus.STATUS_CANCELED if self.nav_terminal else GoalStatus.STATUS_CANCELING
            message = GoalStatusArray()
            message.status_list = [status]
            self.status_pub.publish(message)

    def cancel_pose(self, _, response):
        self.cancel_requests += 1
        if self.nav_active:
            goal = GoalInfo()
            goal.goal_id.uuid = [1] * 16
            response.goals_canceling = [goal]
        return response

    def cancel_poses(self, _, response):
        return response

    def dock(self, handle):
        self.dock_requests.append(handle.request)
        if self.dock_mode == 'hold':
            while not handle.is_cancel_requested and not self.release_dock:
                time.sleep(0.01)
            if handle.is_cancel_requested:
                self.canceled_docks += 1
                handle.canceled()
                return DockRobot.Result(success=False)
        if self.dock_mode == 'fail':
            handle.abort()
            return DockRobot.Result(success=False, error_code=906)
        handle.succeed()
        return DockRobot.Result(success=True)

    def start(self, enabled=True):
        executable = str(Path(get_package_prefix('sbem_autonomy')) /
                         'lib/sbem_autonomy/autonomy_node')
        self.process = subprocess.Popen([
            executable, '--ros-args',
            '-p', 'battery_topic:=/autonomy_test/battery',
            '-p', 'docking_action:=/autonomy_test/dock',
            '-p', 'navigation_actions:=[/autonomy_test/nav_pose,/autonomy_test/nav_poses]',
            '-p', 'low_battery_duration:=0.1', '-p', 'battery_timeout:=0.3',
            '-p', 'tick_period_ms:=20', '-p', 'retry_delay:=0.1',
            '-p', 'cancel_timeout:=1.5', '-p', 'charging_loss_timeout:=3.0',
            '-p', 'max_attempts:=2', '-p', f'enabled:={str(enabled).lower()}',
        ], stdout=self.output, stderr=subprocess.STDOUT, env=os.environ.copy())
        self.wait(lambda: self.state is not None)

    def wait(self, condition, timeout=6.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return
            if self.process is not None and self.process.poll() is not None:
                self.output.seek(0)
                self.fail('Runner exited:\n' + self.output.read())
            time.sleep(0.02)
        self.output.seek(0)
        self.fail(f'Timed out; state={self.state}\n' + self.output.read())

    def enable(self, enabled):
        client = self.node.create_client(SetBool, '/sbem_autonomy/set_enabled')
        self.assertTrue(client.wait_for_service(timeout_sec=3.0))
        future = client.call_async(SetBool.Request(data=enabled))
        self.wait(future.done)
        self.assertTrue(future.result().success)

    def tearDown(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.release_dock = True
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        self.server.destroy()
        self.node.destroy_node()
        self.output.close()

    def test_waits_for_navigation_then_charges_and_stays_docked(self):
        self.nav_active = True
        self.start()
        self.wait(lambda: self.state == 'idle')
        self.assertEqual(len(self.dock_requests), 0)
        self.level = 0.1
        self.wait(lambda: self.cancel_requests > 0)
        time.sleep(0.15)
        self.assertEqual(len(self.dock_requests), 0)  # CANCELING is not stopped.
        self.nav_terminal = True
        self.wait(lambda: len(self.dock_requests) == 1)
        request = self.dock_requests[0]
        self.assertEqual(request.dock_id, 'charging_dock')
        self.assertTrue(request.use_dock_id)
        self.assertTrue(request.navigate_to_staging_pose)
        self.level = 0.85
        time.sleep(0.15)
        self.assertEqual(self.state, 'recharging')
        self.charging = True
        self.wait(lambda: self.state == 'charged')
        time.sleep(0.2)
        self.assertEqual(len(self.dock_requests), 1)

    def test_stale_battery_cancels_an_active_dock(self):
        self.dock_mode = 'hold'
        self.level = 0.1
        self.start()
        self.wait(lambda: len(self.dock_requests) == 1)
        self.publish_battery = False
        self.wait(lambda: self.canceled_docks == 1)
        self.wait(lambda: self.state == 'waiting_for_battery')
        time.sleep(0.2)
        self.assertEqual(len(self.dock_requests), 1)

    def test_failed_docking_stops_after_budget_and_can_be_reset(self):
        self.dock_mode = 'fail'
        self.level = 0.1
        self.start()
        self.wait(lambda: self.state == 'failed')
        self.assertEqual(len(self.dock_requests), 2)
        time.sleep(0.2)
        self.assertEqual(len(self.dock_requests), 2)
        self.enable(True)
        self.wait(lambda: len(self.dock_requests) == 4 and self.state == 'failed')

    def test_disabled_and_invalid_battery_never_dock(self):
        self.level = 0.1
        self.start(enabled=False)
        self.wait(lambda: self.state == 'disabled')
        time.sleep(0.2)
        self.assertEqual(len(self.dock_requests), 0)
        self.level = float('nan')
        time.sleep(0.1)
        self.enable(True)
        self.wait(lambda: self.state == 'waiting_for_battery')
        time.sleep(0.2)
        self.assertEqual(len(self.dock_requests), 0)


if __name__ == '__main__':
    unittest.main()
