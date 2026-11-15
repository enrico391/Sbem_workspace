"""Launch motor supervision independently of the charging behavior tree."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sbem_autonomy'), 'config', 'lidar_motor.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('lidar_params_file', default_value=config),
        Node(package='sbem_autonomy', executable='lidar_motor_node', name='lidar_motor',
             output='screen', parameters=[LaunchConfiguration('lidar_params_file')]),
    ])
