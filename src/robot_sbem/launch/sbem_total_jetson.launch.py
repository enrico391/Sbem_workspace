import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    lidar_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                FindPackageShare('robot_sbem').find('robot_sbem'),
                'launch',
                'footprint_filter_laser.launch.py'
            )
        ])
    )


    ros2_control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                FindPackageShare('diffdrive_sbem').find('diffdrive_sbem'),
                'launch',
                'diffbot.launch.py'
            )
        ])
    )

    # imu_node = Node(
    #     package='robot_sbem',
    #     executable='imu_publisher.py',
    #     name='imu_publisher',
    #     output='screen',
    # )

    camera_publisher = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            parameters=[{
                'image_size': [640,480],
                'camera_frame_id': 'camera_link_optical',
            }]
    )

    node_republisher_camera = Node(
        package='image_transport',
        executable='republish',
        name='republish_camera',
        output='screen',
        arguments=['compressed', 'raw'],
        remappings=[
            ('/in/compressed', '/image_raw/compressed'),
            ('/out', '/image')
        ]
    )

    #audio_listener_node = Node(
    #    package='audio_common',
    #    executable='audio_capturer_node',
    #    output='screen',
    #)

    #audio_player_node = Node(
    #    package='sbem_speaking',
    #    executable='tts_sbem.py',
    #    output='screen',
    #)

    audio_listener_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                FindPackageShare('wakeword_stt_tts').find('wakeword_stt_tts'),
                'launch',
                'wakeword_stt_on_device.launch.py'
            )
        ])
    )



    return LaunchDescription([
        lidar,
        #imu_node,
        lidar_filter,
        ros2_control_node,
        camera_publisher,
        audio_listener_node,
        #node_republisher_camera,
        #audio_player_node
    ])
