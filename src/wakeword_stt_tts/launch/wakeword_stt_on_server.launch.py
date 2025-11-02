from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    wake_record_mic_node = Node(
            package="wakeword_stt_tts",
            executable="wakeword_stt_whisper_server",
            name="wakeword_stt_whisper_server",
            output="screen",
            parameters=[{
                "channels": LaunchConfiguration('channels'),
                "rate": LaunchConfiguration('rate'),
                "device": LaunchConfiguration('device'),
                "on_device": False                
            }]
        )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'channels',
            default_value='1',
            description='Number of audio channels'),
        DeclareLaunchArgument(
            'rate',
            default_value='44100',
            description='Audio sampling rate'),
        DeclareLaunchArgument(
            'device',
            default_value='0',
            description='Audio input device index'),
        wake_record_mic_node
    ])