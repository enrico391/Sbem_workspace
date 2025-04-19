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
            'namespace',
            default_value='',
            description='Namespace for ROS nodes in this launch script'),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='False',
            description='Whether to apply a namespace to the navigation stack'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Whether to use composed Nav2 bringup'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Omniverse Isaac Sim) clock if true'),
        DeclareLaunchArgument(
            'init_pose_x',
            default_value='0.0',
            description='Initial position X coordinate'),
        DeclareLaunchArgument(
            'init_pose_y',
            default_value='0.0',
            description='Initial position Y coordinate'),
        DeclareLaunchArgument(
            'init_pose_yaw',
            default_value='0.0',
            description='Initial yaw orientation'),
        DeclareLaunchArgument(
            'params_file_dock',
            default_value=os.path.join(
                 get_package_share_directory(
                     'sbem_docking'),
                 'params', 'nova_carter_docking.yaml'
             ),
            description='Full path to the docking param file to load'),
        # DeclareLaunchArgument(
        #     'map',
        #     default_value=os.path.join(
        #         get_package_share_directory(
        #             'isaac_ros_vda5050_nav2_client_bringup'),
        #         'maps', 'carter_warehouse_navigation.yaml'
        #     ),
        #     description='Full path to map file to load'),
        # DeclareLaunchArgument(
        #     'nav_params_file',
        #     default_value=os.path.join(
        #         get_package_share_directory(
        #             'isaac_ros_vda5050_nav2_client_bringup'),
        #         'config', 'carter_navigation_params.yaml'
        #     ),
        #     description='Full path to navigation param file to load'),
        
        DeclareLaunchArgument(
            'image',
            default_value='/image_raw',
            description='Image topic'),
        DeclareLaunchArgument(
            'camera_info',
            default_value='/camera_info',
            description='Camera info topic'),
    ]
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_composition = LaunchConfiguration('use_composition')
    use_sim_time = LaunchConfiguration('use_sim_time')
    init_pose_x = LaunchConfiguration('init_pose_x', default=0.0)
    init_pose_y = LaunchConfiguration('init_pose_y', default=0.0)
    init_pose_yaw = LaunchConfiguration('init_pose_yaw', default=0.0)
    map_dir = LaunchConfiguration('map')
    params_file_dock = LaunchConfiguration('params_file_dock', default=os.path.join(get_package_share_directory('sbem_docking'),'params', 'nova_carter_docking.yaml'))
    launch_rviz = LaunchConfiguration('launch_rviz')
    image = LaunchConfiguration('image')
    camera_info = LaunchConfiguration('camera_info')
    nova_carter_dock_launch_dir = os.path.join(
        get_package_share_directory('sbem_docking'), 'launch')

    param_substitutions = {
        'x': init_pose_x,
        'y': init_pose_y,
        'yaw': init_pose_yaw
    }

    # configured_params = RewrittenYaml(
    #     source_file=nav_params_file,
    #     param_rewrites=param_substitutions,
    #     convert_types=True)

    nova_carter_dock_params_dir = os.path.join(
        get_package_share_directory('sbem_docking'), 'params')
    
    # nav2_bringup_launch_dir = os.path.join(
    #     get_package_share_directory('nav2_bringup'), 'launch')


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
        #dock_detection_launch,
        #nav2_bringup_launch,
        docking_server,
        lifecycle_manager
    ])
