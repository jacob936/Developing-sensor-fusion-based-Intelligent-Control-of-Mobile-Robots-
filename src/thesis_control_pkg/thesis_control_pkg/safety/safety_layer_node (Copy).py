#!/usr/bin/env python3
"""
Safety Layer for TurtleBot3
Monitors LiDAR data and applies potential field for obstacle avoidance
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

class SafetyLayer(Node):
    def __init__(self):
        super().__init__('safety_layer_node')
        
        # Safety parameters
        self.safety_distance = 0.3  # meters
        self.max_velocity = 0.22  # TurtleBot3 max
        self.min_velocity = 0.0
        
        # State
        self.current_scan = None
        self.input_path = None
        self.safe_path = None
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.path_sub = self.create_subscription(
            Path,
            '/reference_path',
            self.path_callback,
            10)
        
        # Publisher
        self.safe_path_pub = self.create_publisher(
            Path,
            '/safe_path',
            10)
        
        # Timer
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('Safety Layer initialized')
    
    def scan_callback(self, msg):
        """Receive LiDAR scan data"""
        self.current_scan = msg
    
    def path_callback(self, msg):
        """Receive reference path"""
        self.input_path = msg
        # Apply potential field immediately
        safe_path = self.apply_potential_field(msg)
        if safe_path is not None:
            self.safe_path = safe_path
            self.safe_path_pub.publish(safe_path)
    
    def apply_potential_field(self, msg):
        """Apply potential field for obstacle avoidance"""
        if self.current_scan is None or msg.poses is None:
            return msg
        
        # Convert scan to numpy array
        ranges = np.array(self.current_scan.ranges)
        
        # Filter invalid ranges (inf, nan, out of range)
        valid_ranges = np.isfinite(ranges)
        close_ranges = ranges[valid_ranges]
        
        # FIX: Check if array is empty before calling np.min()
        if len(close_ranges) == 0:
            self.get_logger().debug('No valid scan data')
            return msg
        
        # Find minimum range within 2m
        ranges_within_2m = close_ranges[close_ranges < 2.0]
        
        # FIX: Check if filtered array is empty
        if len(ranges_within_2m) == 0:
            min_range = 2.0  # No obstacles within 2m
        else:
            min_range = np.min(ranges_within_2m)
        
        # Calculate repulsive potential
        if min_range < self.safety_distance:
            # Obstacle too close - stop
            self.get_logger().warn(f'Obstacle too close: {min_range:.2f}m')
            # Return empty path (stop robot)
            safe_msg = Path()
            safe_msg.header = msg.header
            safe_msg.poses = []
            return safe_msg
        elif min_range < 2.0:
            # Obstacle nearby - scale down path
            self.get_logger().debug(f'Obstacle nearby: {min_range:.2f}m')
            # Scale path points based on distance
            safe_msg = Path()
            safe_msg.header = msg.header
            scale_factor = (min_range - self.safety_distance) / (2.0 - self.safety_distance)
            scale_factor = np.clip(scale_factor, 0.0, 1.0)
            
            for pose in msg.poses:
                new_pose = PoseStamped()
                new_pose.header = pose.header
                new_pose.pose.position.x = pose.pose.position.x * scale_factor
                new_pose.pose.position.y = pose.pose.position.y * scale_factor
                new_pose.pose.orientation = pose.pose.orientation
                safe_msg.poses.append(new_pose)
            
            return safe_msg
        else:
            # No obstacles - path is safe
            return msg
    
    def safety_check(self):
        """Periodic safety check"""
        if self.current_scan is not None:
            ranges = np.array(self.current_scan.ranges)
            valid_ranges = np.isfinite(ranges)
            close_ranges = ranges[valid_ranges]
            
            # FIX: Check if array is empty
            if len(close_ranges) > 0:
                ranges_within_2m = close_ranges[close_ranges < 2.0]
                
                if len(ranges_within_2m) > 0:
                    min_range = np.min(ranges_within_2m)
                    if min_range < self.safety_distance:
                        self.get_logger().warn(f'Safety stop: {min_range:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyLayer()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
