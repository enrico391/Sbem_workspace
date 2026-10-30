"""Run the named-location manager alongside an existing Nav2 bringup."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'database_path': '~/.ros/sbem_waypoints.sqlite3',
        'map_topic': 'map',
        'global_frame': 'map',
        'robot_frame': 'base_link',
        'nav_action_name': 'navigate_to_pose',
        'max_transform_age': '1.0',
        'server_timeout': '2.0',
        'acceptance_timeout': '5.0',
        'use_sim_time': 'false',
    }
    types = {'max_transform_age': float, 'server_timeout': float,
             'acceptance_timeout': float, 'use_sim_time': bool}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        Node(package='sbem_waypoint_manager', executable='waypoint_manager',
             name='waypoint_manager', output='screen', parameters=[{
                 name: ParameterValue(LaunchConfiguration(name),
                                      value_type=types.get(name, str))
                 for name in defaults
             }]),
    ])
