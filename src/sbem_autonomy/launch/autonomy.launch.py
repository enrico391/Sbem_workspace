# Copyright 2026 Enrico Moro
# SPDX-License-Identifier: Apache-2.0
"""Launch battery supervision alongside an existing Nav2 and docking bringup."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sbem_autonomy'), 'config', 'autonomy.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enabled', default_value='true'),
        Node(
            package='sbem_autonomy', executable='autonomy_node', name='sbem_autonomy',
            output='screen', parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool)},
            ]),
    ])
