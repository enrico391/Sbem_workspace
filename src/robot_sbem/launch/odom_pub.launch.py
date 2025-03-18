from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    odom_node = Node(
            package='robot_sbem',
            executable='pub_odom_sbem.py',
         )



    return LaunchDescription([
        odom_node    
    ])