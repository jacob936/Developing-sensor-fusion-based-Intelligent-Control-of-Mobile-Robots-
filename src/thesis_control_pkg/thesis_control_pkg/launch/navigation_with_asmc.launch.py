#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    thesis_pkg_dir = get_package_share_directory('thesis_control_pkg')
    
    return LaunchDescription([
        # 1. Start SLAM Toolbox (for mapping/localization)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nav2_bringup_dir, 'launch', 'slam_toolbox.launch.py')
            ]),
            launch_arguments={'use_sim_time': 'True'}.items()
        ),
        
        # 2. Start NAV2 Navigation Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'True',
                'params_file': os.path.join(thesis_pkg_dir, 'nav2_config', 'nav2_params.yaml')
            }.items()
        ),
        
        # 3. YOUR Adaptive EKF (provides state to NAV2)
        Node(
            package='thesis_control_pkg',
            executable='adaptive_ekf_node',
            name='adaptive_ekf',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/state_estimated', '/pose')  # NAV2 expects /pose
            ]
        ),
        
        # 4. YOUR ASMC Controller (receives NAV2 commands)
        Node(
            package='thesis_control_pkg',
            executable='asmc_controller_node',
            name='asmc_controller',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # 5. Safety Layer (additional obstacle avoidance)
        Node(
            package='thesis_control_pkg',
            executable='safety_layer_node',
            name='safety_layer',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])

