import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():



    dock_pose_publisher = Node(
            package='robot_sbem',
            executable='dock_pose_publisher',
            name='dock_pose_publisher',
            parameters=[{'use_first_detection': True, 'dock_tag_id': 0}],
        )
    

    return launch.LaunchDescription([
        dock_pose_publisher,
    ])