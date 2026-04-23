#!/usr/bin/env python3
"""
Launch file for PID Controller Experiment
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Adaptive EKF Node (State Estimation)
        Node(
            package='thesis_control_pkg',
            executable='adaptive_ekf_node',
            name='adaptive_ekf',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # PID Controller Node (instead of ASMC)
        Node(
            package='thesis_control_pkg',
            executable='pid_controller_node',
            name='pid_controller',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Trajectory Generator Node
        Node(
            package='thesis_control_pkg',
            executable='trajectory_generator_node',
            name='trajectory_generator',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()

