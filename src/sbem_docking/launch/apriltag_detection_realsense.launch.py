# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    launch_args = [
        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='Use composed bringup with NVBLOX_CONTAINER_NAME if True'),
    ]

    use_composition = LaunchConfiguration('use_composition',default='True')

    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='rectify',
        namespace='',
        parameters=[{
            'output_width': 1920,
            'output_height': 1080,
        }]
    )

    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace='',
        condition=UnlessCondition(use_composition)
    )

    apriltag_node_nvblox = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace='',
        remappings=[
            ('image', '/camera0/camera/color/image_raw'),
            ('camera_info', '/camera0/camera/color/camera_info')
        ],
        condition=IfCondition(use_composition)
    )

    # use only if not using composition, otherwise these nodes will be loaded in the NVBLOX_CONTAINER_NAME container
    realsense_camera_node = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name='realsense2_camera',
        namespace='',
        parameters=[{
            'color_height': 1080,
            'color_width': 1920,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_depth': False,
            'camera_name': 'camera0'
        }],
        remappings=[('/realsense2_camera/color/image_raw', '/image'),
                    ('/realsense2_camera/color/camera_info', '/camera_info'),],
        condition=UnlessCondition(use_composition)
    )

    # if not using composition, load nodes in this container, otherwise load them in the NVBLOX_CONTAINER_NAME container
    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            rectify_node,
            apriltag_node,
            realsense_camera_node
        ],
        output='screen',
        condition=UnlessCondition(use_composition)
    )

    # load nodes in the NVBLOX_CONTAINER_NAME container if using composition, otherwise they will be loaded in the apriltag_container defined above
    loaded_container_nvblox = LoadComposableNodes(
        target_container=NVBLOX_CONTAINER_NAME,
        composable_node_descriptions=[
            rectify_node,
            apriltag_node_nvblox,
        ],
        condition=IfCondition(use_composition)
    )

    return launch.LaunchDescription(launch_args + [
        apriltag_container,
        loaded_container_nvblox
    ])