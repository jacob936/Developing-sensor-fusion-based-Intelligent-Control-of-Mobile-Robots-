#!/usr/bin/env python3
"""
Trajectory Generator - Path Through Obstacles
For testing obstacle avoidance
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator_node')
        
        self.path_pub = self.create_publisher(Path, '/reference_path', 10)
        
        # Robot spawns at ~(1.5, -0.1)
        self.spawn_x = 1.5
        self.spawn_y = -0.1
        
        # Waypoints that CROSS the pillar field (pillars are at ~±1.5m from center)
        # This forces the robot to encounter obstacles!
        self.waypoints_relative = [
            [0.0, 0.0],       # Start at spawn
            [2.5, 0.5],       # Cross through pillar area
            [1.0, 1.5],       # Near pillars
            [-0.5, 0.5],      # Cross back through pillars
            [0.0, 0.0],       # Return to start
        ]
        
        # Apply spawn offset
        self.waypoints = [
            [wp[0] + self.spawn_x, wp[1] + self.spawn_y]
            for wp in self.waypoints_relative
        ]
        
        self.timer = self.create_timer(1.0, self.publish_path)
        
        self.get_logger().info('===========================================')
        self.get_logger().info('Trajectory Generator - OBSTACLE TEST')
        self.get_logger().info(f'Spawn: ({self.spawn_x:.2f}, {self.spawn_y:.2f})')
        self.get_logger().info(f'Waypoints: {len(self.waypoints)}')
        self.get_logger().info('Path CROSSES pillar field!')
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'  WP {i}: ({wp[0]:.2f}m, {wp[1]:.2f}m)')
        self.get_logger().info('===========================================')
    
    def publish_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    generator = TrajectoryGenerator()
    rclpy.spin(generator)
    generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

