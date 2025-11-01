from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    wake_record_mic_node = Node(
            package="wakeword_stt_tts",
            executable="wakeword_stt",
            name="wake_record_mic",
            output="screen",
            parameters=[{
                "index_mic": -1,
                "use_wake_word": False
            }]
        )

    tts_server_node = Node(
            package="wakeword_stt_tts",
            executable="tts_server",
            name="tts_server",
            output="screen",
            parameters=[{
                "channels": 1,
                "device": 24,
                "useLocalTTS": False,
                "typeLocalTTS": "coqui"
            }]
    )
    
    return LaunchDescription([
        wake_record_mic_node,
        tts_server_node
    ])