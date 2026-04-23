#!/usr/bin/env python3
"""
Launch file for Adaptive Sliding Mode Control with Sensor Fusion
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    
    return LaunchDescription([
        # Adaptive EKF Node
        Node(
            package='thesis_control_pkg',
            executable='adaptive_ekf_node',
            name='adaptive_ekf',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        
        # ASMC Controller Node
        Node(
            package='thesis_control_pkg',
            executable='asmc_controller_node',
            name='asmc_controller',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        
        # Safety Layer Node (Potential Field Obstacle Avoidance)
        #Node(
            #package='thesis_control_pkg',
            #executable='safety_layer_node',
            #name='safety_layer',
            #output='screen',
            #parameters=[{'use_sim_time': True}],
        #),
        
        # Trajectory Generator
        Node(
            package='thesis_control_pkg',
            executable='trajectory_generator_node',
            name='trajectory_generator',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])

