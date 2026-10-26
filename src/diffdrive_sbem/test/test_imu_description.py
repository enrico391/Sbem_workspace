"""Validate the real robot's IMU interfaces and broadcaster configuration."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml


def test_imu_interfaces_and_broadcaster():
    description = Path(get_package_share_directory('sbem_description'))
    model = description / 'description' / 'robot_sbem.urdf.xacro'
    robot = ET.fromstring(xacro.process_file(str(model)).toxml())
    sensor = robot.find("ros2_control/sensor[@name='imu']")
    assert sensor is not None
    expected = {
        'orientation.x', 'orientation.y', 'orientation.z', 'orientation.w',
        'angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z',
        'linear_acceleration.x', 'linear_acceleration.y', 'linear_acceleration.z',
    }
    assert {item.attrib['name'] for item in sensor.findall('state_interface')} == expected
    assert robot.find("ros2_control/hardware/param[@name='imu_name']").text == 'imu'
    assert robot.find("link[@name='imu_1']") is not None

    config = Path(get_package_share_directory('diffdrive_sbem')) / 'config'
    controllers = yaml.safe_load((config / 'diffbot_controllers.yaml').read_text())
    params = controllers['imu_sensor_broadcaster']['ros__parameters']
    assert params['sensor_name'] == sensor.attrib['name']
    assert params['frame_id'] == 'imu_1'
    assert params['static_covariance_orientation'][0] == -1.0


def test_simulation_does_not_claim_the_esp32_imu():
    description = Path(get_package_share_directory('sbem_description'))
    model = description / 'description' / 'robot_sbem.urdf.xacro'
    robot = ET.fromstring(xacro.process_file(str(model), mappings={'sim_mode': 'true'}).toxml())
    assert robot.find("ros2_control/sensor[@name='imu']") is None
