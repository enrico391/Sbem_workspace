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
            get_package_share_directory('sbem_docking'), 
            'launch', 
            'docking_sbem.launch.py'))
    )

    dock_pose_publisher = Node(
            package='robot_sbem',
            executable='dock_pose_publisher',
            name='dock_pose_publisher',
            parameters=[{'use_first_detection': True, 'dock_tag_id': 0}],
    )

    apriltag_detector_realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('isaac_ros_apriltag'), 
            'launch', 
            'isaac_ros_apriltag_realsense.launch.py'))
    )

    # ros_bridge_server = Node(
    #     package='rosbridge_server',
    #     executable='rosbridge_websocket',
    #     name='rosbridge_websocket',
    #     output='screen',
    # )

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
        apriltag_detector_realsense_node,
        dock_pose_publisher,
        localization_node,
        navigation_node,
        docking_node,
        # ros_bridge_server,
        # server_stt,
    ])
