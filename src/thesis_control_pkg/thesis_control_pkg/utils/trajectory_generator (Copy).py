#!/usr/bin/env python3
"""
Waypoint Generator - FIXED for Actual Robot Spawn Position
Robot spawns at (1.5, -0.1) in turtlebot3_world, so waypoints are adjusted
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator_node')
        
        # ===== ROBOT SPAWN OFFSET =====
        # TurtleBot3 spawns at this position in turtlebot3_world.launch.py
        # NOT at (0, 0)! This was causing the trajectory mismatch.
        self.spawn_x = 1.5
        self.spawn_y = -0.1
        
        # ===== WAYPOINTS RELATIVE TO SPAWN =====
        # Define waypoints relative to where robot actually spawns
        self.waypoints_relative = [
            [0.0, 0.0],       # Start at spawn position
            [2.0, 0.0],       # Forward 2m from spawn
            [2.0, 2.0],       # Left 2m
            [0.0, 2.0],       # Back 2m  
            [0.0, 0.0],       # Return to spawn
        ]
        
        # Apply spawn offset to get actual world coordinates
        self.waypoints = [
            [wp[0] + self.spawn_x, wp[1] + self.spawn_y]
            for wp in self.waypoints_relative
        ]
        
        self.path_pub = self.create_publisher(Path, '/reference_path', 10)
        
        self.state_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/state_estimated',
            self.state_callback,
            10)
        
        self.current_state = None
        self.timer = self.create_timer(0.5, self.publish_path)
        
        # Log spawn position and waypoints for verification
        self.get_logger().info('===========================================')
        self.get_logger().info('Waypoint Generator - FIXED')
        self.get_logger().info(f'Robot spawn offset: ({self.spawn_x:.2f}, {self.spawn_y:.2f})')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'  WP {i}: ({wp[0]:.2f}m, {wp[1]:.2f}m)')
        self.get_logger().info('===========================================')
    
    def state_callback(self, msg):
        self.current_state = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ]
    
    def publish_path(self):
        path = Path()
        path.header.frame_id = 'odom'  # Must match state_estimated frame!
        path.header.stamp = self.get_clock().now().to_msg()
        
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(wp[0])  # Already includes spawn offset
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        self.path_pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

