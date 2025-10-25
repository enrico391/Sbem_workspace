import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():
    package_name='robot_sbem'

    pkg_dir = get_package_share_directory(package_name)

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','sbem.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
                                        'world': '/home/morolinux/Projects/Sbem/sbem_project_ws/src/robot_sbem/worlds/new_world.world'}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot',
                                    '-x', '1.0',       # X position
                                    '-y', '-10.0',     # Y position
                                    '-z', '0.1',       # Z position
                                    '-R', '0.0',       # Roll (in radians)
                                    '-P', '0.0',       # Pitch (in radians)
                                    '-Y', '1.57'       # Yaw (in radians)
                                   ],
                        output='screen')


    

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
        #remappings=[('/cmd_vel_unstamped','/cmd_vel')]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Launch the footprint filter node to filter laser scans
    foot_print_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_sbem'), 
            'launch', 
            'footprint_filter_laser.launch.py'))
    )


    # start localization and navigation nodes

    #map_dir = '/home/morolinux/Projects/Sbem/sbem_project_ws/src/robot_sbem/maps/new_virtual_map/new_map_save.yaml'

    # location_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('robot_sbem'), 
    #         'launch', 
    #         'localization.launch.py'))
    #         # launch_arguments={'use_sim_time': 'true',
    #         #            'map': map_dir }.items()
    # )

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        foot_print_filter
    ])