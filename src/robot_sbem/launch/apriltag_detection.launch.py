import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource




def generate_launch_description():


    apriltag_node = Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[{
                'tag_family': '36h11',
                'publish_tf': True,
                'tf_prefix': 'tag_detections',
                'camera_frame_id': 'camera_link',
                'tag_size': 0.075,  # Size of the tag in meters
            }],
            remappings=[
                ('image_rect', '/image'),
                ('camera_info', '/camera_info'),
            ],
        )
    


    return LaunchDescription([
        apriltag_node,
    ])