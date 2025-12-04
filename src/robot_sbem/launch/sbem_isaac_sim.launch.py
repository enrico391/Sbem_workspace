import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command


def generate_launch_description():
    package_name='robot_sbem'

    pkg_dir = get_package_share_directory(package_name)

    
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    

    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('robot_sbem'),
            'rviz',
            'rviz_config_isaac.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    
    joy_params = os.path.join(get_package_share_directory('robot_sbem'),'config','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}]
            #,remappings=[('/cmd_vel','/diff_cont/cmd_vel_unstamped')]
         )
    

    # Launch the footprint filter node to filter laser scans
    foot_print_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_sbem'), 
            'launch', 
            'footprint_filter_laser.launch.py'))
    )


    
    return LaunchDescription([
        #foot_print_filter,
        rviz_node,
        joy_node,
        teleop_node
    ])