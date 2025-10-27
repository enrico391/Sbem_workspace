from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    wake_record_mic_node = Node(
            package="sbem_speaking",
            executable="wake_stt_sbem_direct_mic.py",
            name="wake_record_mic",
            output="screen"
        )

    stt_server_node = Node(
            package="sbem_speaking",
            executable="stt_server.py",
            name="stt_server",
            output="screen"
        )

    tts_server_node = Node(
            package="sbem_speaking",
            executable="tts_server.py",
            name="tts_server",
            output="screen"
    )
    
    
    return LaunchDescription([
        wake_record_mic_node,
        stt_server_node,
        #tts_server_node
    ])