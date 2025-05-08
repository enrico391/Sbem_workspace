import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Check if we're told to use ros2_control
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='false')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('robot_sbem'))
    xacro_file = os.path.join(pkg_path,'description','robot_sbem.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' use_sim_time:=', use_sim_time])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Create a node for the odometry and publish transform
    # odom_node = Node(
    #         package='robot_sbem',
    #         executable='diff_tf.py',
    #      )
    
    # publish only odometry
    odom_node = Node(
            package='robot_sbem',
            executable='pub_odom_sbem.py',
         )
    
    # node for robot localization with kalman filter
    fusing_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("robot_sbem"),
                    "config", "ekf.yaml",
                ]),  
                {'use_sim_time': use_sim_time}],
    )
    
    # Create a laser filter node to filter the laser scan from the robot structure
    laser_filter = Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("robot_sbem"),
                    "config", "footprint_filter_laser.yaml",
                ])],
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

    # Create a node for the TTS local server
    tts_local_server = Node(
        package='sbem_speaking',
        executable='stt_socketServer.py',  
    )
    
    # Create an RViz node to visualize the robot with a custom config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'config', 'config_rviz.rviz')],
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        odom_node,
        laser_filter,
        fusing_node,
        node_image_republisher,
        ros_bridge_server,
        tts_local_server,
        rviz_node
    ])
