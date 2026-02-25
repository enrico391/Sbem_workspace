import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument
import launch
from launch_ros.actions import Node
#from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Omniverse Isaac Sim) clock if true'),
        DeclareLaunchArgument(
            'params_file_dock',
            default_value=os.path.join(
                 get_package_share_directory(
                     'sbem_docking'),
                 'params', 'docking_simulation.yaml'
             ),
            description='Full path to the docking param file to load'),
    ]
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    params_file_dock = LaunchConfiguration('params_file_dock', default=os.path.join(get_package_share_directory('sbem_docking'),'params', 'docking_simulation.yaml'))
    
    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[params_file_dock,
                    {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': ['docking_server']}],
    )

    return LaunchDescription(launch_args + [
        docking_server,
        lifecycle_manager
    ])
