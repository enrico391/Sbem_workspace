from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    wake_record_mic_node = Node(
            package="wakeword_stt_tts",
            executable="wakeword_stt_whisper_server",
            name="wakeword_stt_whisper_server",
            output="screen",
            parameters=[{
                "channels": 1,
                "rate": 44100,
                "device": 0,
                "on_device": False                
            }]
        )
    
    return LaunchDescription([
        wake_record_mic_node
    ])