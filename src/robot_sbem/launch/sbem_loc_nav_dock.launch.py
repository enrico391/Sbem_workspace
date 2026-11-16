import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # Declare use_sim_time for all child launches
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Use composed bringup if True')
    
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='The name of container that nodes will load in if use composition')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_enable_autonomy_cmd = DeclareLaunchArgument(
        'enable_autonomy', default_value='false',
        description='Start automatic low-battery docking supervision')
    autonomy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sbem_autonomy'), 'launch', 'autonomy.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('enable_autonomy')))
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name') 


    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('robot_sbem'),
                'config',
                'nav2_params.yaml',
            ),
        ),
        root_key=LaunchConfiguration('namespace', default=''),
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True,
    )

    # container initialization if composition is True
    nav2_container = Node(
        condition=IfCondition(use_composition),
        package='rclcpp_components',
        executable='component_container_isolated',
        name=container_name,
        namespace=LaunchConfiguration('namespace', default=''),
        output='screen',
        parameters=[configured_params],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    ) 

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
    
    # Launch!
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_enable_autonomy_cmd,
        declare_use_composition_cmd,
        declare_container_name_cmd,
        nav2_container,
        dock_pose_publisher,
        localization_node,
        navigation_node,
        docking_node,
        autonomy_node,
    ])
