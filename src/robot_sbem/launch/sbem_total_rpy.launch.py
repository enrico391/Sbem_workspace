import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    
    lidar = Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard',
                'serial_baudrate':115200
            }]
    )

    

    imu_node = Node(
        package='robot_sbem',
        executable='imu_publisher.py',
        name='imu_publisher',
        output='screen',
    )

    camera_publisher = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            parameters=[{
                'image_size': [640,480],
                'camera_frame_id': 'camera_link_optical',
            }]
    )

    #audio_listener_node = Node(
    #    package='audio_common',
    #    executable='audio_capturer_node',
    #    output='screen',
    #)

    audio_player_node = Node(
        package='sbem_speaking',
        executable='tts_sbem.py',
        output='screen',
    )

    audio_listener_node = Node(
        package='sbem_speaking',
        executable='wake_stt_sbem_direct_mic.py',
        output='screen',
    )



    return LaunchDescription([
        lidar,
        imu_node,
        camera_publisher,
        audio_listener_node,
        audio_player_node
    ])
