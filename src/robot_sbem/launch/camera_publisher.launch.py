from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_sbem',
            executable='camera_publisher.py',
            name='camera_publisher',
            output='screen'
        ),
        
    ])