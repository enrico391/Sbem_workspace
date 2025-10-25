import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_sbem'), 
            'launch', 
            'localization.launch.py'))
    )

    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_sbem'), 
            'launch', 
            'navigation.launch.py'))
    )
    

    docking_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_sbem'), 
            'launch', 
            'docking_sbem.launch.py'))
    )

    ros_bridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
    )

    # Create a node image republisher node
    node_image_republisher = Node(
        package='image_transport',
        executable='republish',
        output='screen',
        name='image_republisher_compressed',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/image_raw/compressed'),
            ('out', '/image')
        ],
    )

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

    server_stt = Node(
        package='sbem_speaking',
        executable='stt_socket_server.py',
        name='audio_transcription_server',
        output='screen',
    )
    
    # Launch!
    return LaunchDescription([
        localization_node,
        navigation_node,
        docking_node,
        node_image_republisher,
        apriltag_node,
        ros_bridge_server,
        server_stt,
    ])
