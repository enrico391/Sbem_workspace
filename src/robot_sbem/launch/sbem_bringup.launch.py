import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('robot_sbem'), 'launch'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_autonomy',
            default_value='true',
            description='Enable automatic low-battery docking',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'sbem_periphericals.launch.py')
            ),
        ),

        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'sbem_loc_nav_dock.launch.py')
                    ),
                    launch_arguments={
                        'enable_autonomy': LaunchConfiguration('enable_autonomy'),
                    }.items(),
                ),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('sbem_waypoint_manager'),
                    'launch',
                    'waypoint_manager.launch.py'
                )
            ),
        ),

        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('sbem_nvblox_bringup'),
                            'launch',
                            'sensors',
                            'realsense.launch.py',
                        )
                    ),
                    launch_arguments={
                        'num_cameras': '1',
                        'run_standalone': 'True',
                        'container_name': 'nvblox_container',
                    }.items(),
                ),
            ],
        ),

        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('sbem_docking'),
                            'launch',
                            'apriltag_detection_realsense.launch.py',
                        )
                    ),
                    launch_arguments={'use_composition': 'True'}.items(),
                ),
            ],
        ),

        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('rosbridge_server'),
                            'launch',
                            'rosbridge_websocket_launch.xml',
                        )
                    ),
                    launch_arguments={
                        'port': '9090',
                    }.items(),
                ),
            ],
        ),
    ])
