#!/usr/bin/env python3
"""
Safety Layer for TurtleBot3
Uses Potential Field for Obstacle Avoidance
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import numpy as np
import math

class SafetyLayer(Node):
    def __init__(self):
        super().__init__('safety_layer_node')
        
        # Safety parameters
        self.safety_distance = 0.5      # Stop if obstacle closer than this
        self.avoidance_distance = 1.0   # Start avoiding if obstacle closer than this
        self.max_velocity = 0.22        # TurtleBot3 max linear velocity
        self.min_velocity = 0.0
        
        # State
        self.current_scan = None
        self.input_path = None
        self.desired_velocity = [0.1, 0.0]  # [v, omega] default
        
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
        
        # Publisher - Output safe velocity commands
        self.safe_vel_pub = self.create_publisher(
            TwistStamped,
            '/safe_velocity',
            10)
        
        # Timer - Safety check at 20 Hz
        self.safety_timer = self.create_timer(0.05, self.safety_loop)
        
        self.get_logger().info('Safety Layer initialized (Potential Field)')
    
    def scan_callback(self, msg):
        """Receive LiDAR scan data"""
        self.current_scan = msg
    
    def path_callback(self, msg):
        """Receive reference path"""
        self.input_path = msg
        
        # Extract desired velocity from path (simplified - use default)
        # In advanced version, compute velocity from path curvature
        self.desired_velocity = [0.15, 0.0]  # Default forward velocity
    
    def compute_potential_field_velocity(self, desired_v, desired_omega):
        """Compute velocity using potential field obstacle avoidance"""
        
        if self.current_scan is None:
            return desired_v, desired_omega
        
        # Convert scan to numpy array
        ranges = np.array(self.current_scan.ranges)
        angles = np.array([
            self.current_scan.angle_min + i * self.current_scan.angle_increment
            for i in range(len(ranges))
        ])
        
        # Filter invalid ranges
        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]
        
        if len(ranges) == 0:
            return desired_v, desired_omega
        
        # Initialize repulsion velocity
        v_repulsion = 0.0
        omega_repulsion = 0.0
        
        # Process each scan point
        for i, (r, theta) in enumerate(zip(ranges, angles)):
            if r < self.avoidance_distance:
                # Calculate repulsion strength (stronger when closer)
                repulsion_strength = (self.avoidance_distance - r) / self.avoidance_distance
                repulsion_strength = np.clip(repulsion_strength, 0.0, 1.0)
                
                # Front obstacles (reduce forward velocity)
                if abs(theta) < math.pi / 4:  # Front 90 degrees
                    v_repulsion -= repulsion_strength * desired_v * 0.8
                
                # Left obstacles (turn right)
                if 0 < theta < math.pi / 2:
                    omega_repulsion -= repulsion_strength * 0.5
                
                # Right obstacles (turn left)
                if -math.pi / 2 < theta < 0:
                    omega_repulsion += repulsion_strength * 0.5
        
        # Apply repulsion to desired velocity
        safe_v = desired_v + v_repulsion
        safe_omega = desired_omega + omega_repulsion
        
        # Apply limits
        safe_v = np.clip(safe_v, self.min_velocity, self.max_velocity)
        safe_omega = np.clip(safe_omega, -2.84, 2.84)
        
        # Emergency stop if obstacle too close
        if len(ranges) > 0 and np.min(ranges) < self.safety_distance:
            self.get_logger().warn(f'EMERGENCY STOP: Obstacle at {np.min(ranges):.2f}m')
            return 0.0, 0.0
        
        return safe_v, safe_omega
    
    def safety_loop(self):
        """Main safety loop - publish safe velocity"""
        
        if self.current_scan is None:
            return
        
        # Compute safe velocity
        safe_v, safe_omega = self.compute_potential_field_velocity(
            self.desired_velocity[0],
            self.desired_velocity[1]
        )
        
        # Publish safe velocity
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(safe_v)
        msg.twist.angular.z = float(safe_omega)
        
        self.safe_vel_pub.publish(msg)
        
        # Log occasionally
        if abs(safe_v - self.desired_velocity[0]) > 0.01:
            self.get_logger().debug(f'Velocity modified: {self.desired_velocity[0]:.2f} → {safe_v:.2f}')

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyLayer()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

