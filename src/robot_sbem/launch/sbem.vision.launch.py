import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import xacro
import logging


def generate_launch_description():

    config_tag = os.path.join(
        get_package_share_directory("apriltag_ros"), "cfg", "tags_36h11.yaml"
    )
    
    # Setup logging
    logger = logging.getLogger('launch')
    logger.info(f"AprilTag config file: {config_tag}")

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

    # # node for apriltag detection
    # node_apriltag_detector = Node(
    #     package="apriltag_ros",
    #     executable="apriltag_node",
    #     name="apriltag",
    #     namespace="apriltag",
    #     output="screen",
    #     parameters=[{'from' : config_tag}],
    #     # parameters=[
    #     #         PathJoinSubstitution([
    #     #             FindPackageShare('apriltag_ros'),
    #     #             'cfg',
    #     #             'tags_36h11.yaml'
    #     #         ])
    #     #     ],
            
        
    # )


    # Launch!
    return LaunchDescription([
        node_image_republisher,
        #node_apriltag_detector
    ])
