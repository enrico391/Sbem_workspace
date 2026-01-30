import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    # Declare use_sim_time for all child launches
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True')
    
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='The name of container that nodes will load in if use composition')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')  

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

    dock_pose_publisher = Node(
            package='robot_sbem',
            executable='dock_pose_publisher',
            name='dock_pose_publisher',
            parameters=[{'use_first_detection': True, 'dock_tag_id': 0}],
    )

    # ros_bridge_server = Node(
    #     package='rosbridge_server',
    #     executable='rosbridge_websocket',
    #     name='rosbridge_websocket',
    #     output='screen',
    # )

    # # Create a node image republisher node
    # node_image_republisher = Node(
    #     package='image_transport',
    #     executable='republish',
    #     output='screen',
    #     name='image_republisher_compressed',
    #     arguments=['compressed', 'raw'],
    #     remappings=[
    #         ('in/compressed', '/image_raw/compressed'),
    #         ('out', '/image')
    #     ],
    # )

    # apriltag_node = Node(
    #         package='apriltag_ros',
    #         executable='apriltag_node',
    #         name='apriltag_node',
    #         output='screen',
    #         parameters=[{
    #             'tag_family': '36h11',
    #             'publish_tf': True,
    #             'tf_prefix': 'tag_detections',
    #             'camera_frame_id': 'camera_link',
    #             'tag_size': 0.075,  # Size of the tag in meters
    #         }],
    #         remappings=[
    #             ('image_rect', '/image'),
    #             ('camera_info', '/camera_info'),
    #         ],
    #     )

    # server_stt = Node(
    #     package='sbem_speaking',
    #     executable='stt_socket_server.py',
    #     name='audio_transcription_server',
    #     output='screen',
    # )
    
    # Launch!
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_use_composition_cmd,
        declare_container_name_cmd,
        dock_pose_publisher,
        localization_node,
        navigation_node,
        docking_node
        # node_image_republisher,
        # apriltag_node,
        # ros_bridge_server,
        # server_stt,
    ])
