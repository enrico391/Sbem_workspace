from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stt_server = Node(
            package="wakeword_stt_tts",
            executable="stt_server",
            name="stt_server",
            output="screen",
    )
    
    return LaunchDescription([
        stt_server
    ])