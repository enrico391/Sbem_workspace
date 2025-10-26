from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    wake_record_mic_node = Node(
            package="sbem_speaking",
            executable="wake_stt_sbem_direct_mic.py",
            name="wake_record_mic"
        )
    
    
    
    
    return LaunchDescription([
        wake_record_mic_node
    ])