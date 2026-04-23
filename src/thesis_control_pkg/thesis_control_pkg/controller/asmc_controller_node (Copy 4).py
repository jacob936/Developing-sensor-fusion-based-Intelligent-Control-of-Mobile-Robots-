#!/usr/bin/env python3
"""
ASMC Controller - FIXED Integral Action
Prevents spinning and going off-course
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Path
import math
import numpy as np
import csv
import time
import os

class AdaptiveSMCController(Node):
    def __init__(self):
        super().__init__('asmc_controller_node')
        
        # Controller Gains
        self.k_linear = 0.8
        self.k_angular = 1.5
        self.max_v = 0.22
        self.max_omega = 1.0
        
        # Integral Action - LINEAR ONLY (angular causes spinning!)
        self.ki_linear = 0.02      # Very small integral
        self.integral_linear = 0.0
        self.integral_limit = 2.0   # Smaller limit
        
        # Waypoint tracking
        self.current_waypoint_idx = 0
        self.waypoint_threshold = 0.15
        
        # Data logging
        self.output_dir = '/root/sensor_fusion_ws/experiment_data'
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = int(time.time())
        self.csv_file = f'{self.output_dir}/baseline_{timestamp}.csv'
        
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time_sec', 'state_x', 'state_y', 'state_theta',
                'ref_x', 'ref_y', 'error_x', 'error_y', 'error_dist',
                'cmd_v', 'cmd_omega'
            ])
        
        self.start_time = time.time()
        self.log_count = 0
        
        self.get_logger().info('===========================================')
        self.get_logger().info('ASMC Controller - FIXED')
        self.get_logger().info('Linear integral ONLY (prevents spinning)')
        self.get_logger().info(f'kp_v={self.k_linear}, ki_v={self.ki_linear}')
        self.get_logger().info(f'kp_w={self.k_angular}')
        self.get_logger().info(f'Output: {self.csv_file}')
        self.get_logger().info('===========================================')
        
        self.current_state = None
        self.reference_path = None
        
        self.state_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/state_estimated', self.state_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/reference_path', self.path_callback, 10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)
    
    def state_callback(self, msg):
        self.current_state = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quaternion_to_yaw(msg.pose.pose.orientation)
        }
    
    def path_callback(self, msg):
        self.reference_path = msg.poses
        if len(self.reference_path) > 0:
            self.get_logger().info(f'Received path with {len(self.reference_path)} waypoints')
    
    def quaternion_to_yaw(self, quat):
        return math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                         1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def advance_waypoint(self, distance):
        if distance < self.waypoint_threshold:
            self.current_waypoint_idx += 1
            
            # ===== CRITICAL FIX: Reset integral on waypoint change =====
            self.integral_linear = 0.0
            # ============================================================
            
            if self.current_waypoint_idx >= len(self.reference_path):
                self.current_waypoint_idx = 0
                self.get_logger().info('*** Completed loop! ***')
            else:
                wp = self.reference_path[self.current_waypoint_idx].pose.position
                self.get_logger().warn(f'*** WP reached! Next: ({wp.x:.2f}, {wp.y:.2f}) ***')
            return True
        return False
    
    def compute_control(self):
        if self.current_state is None:
            return 0.0, 0.0
        if self.reference_path is None or len(self.reference_path) == 0:
            return 0.0, 0.0
        
        # Get waypoint
        goal_x = self.reference_path[self.current_waypoint_idx].pose.position.x
        goal_y = self.reference_path[self.current_waypoint_idx].pose.position.y
        
        x = self.current_state['x']
        y = self.current_state['y']
        theta = self.current_state['theta']
        
        # Compute error
        dx = goal_x - x
        dy = goal_y - y
        distance = math.sqrt(dx**2 + dy**2)
        
        self.advance_waypoint(distance)
        
        # Recompute after advance
        goal_x = self.reference_path[self.current_waypoint_idx].pose.position.x
        goal_y = self.reference_path[self.current_waypoint_idx].pose.position.y
        dx = goal_x - x
        dy = goal_y - y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Heading error
        desired_theta = math.atan2(dy, dx)
        theta_error = self.normalize_angle(desired_theta - theta)
        
        # ===== INTEGRAL: Linear ONLY =====
        # Update linear integral
        self.integral_linear = self.integral_linear + distance * 0.05
        self.integral_linear = np.clip(self.integral_linear, -self.integral_limit, self.integral_limit)
        
        # Control: Linear has P+I, Angular has P ONLY
        v = self.k_linear * distance + self.ki_linear * self.integral_linear
        omega = self.k_angular * theta_error  # NO integral for angular!
        
        # Forward motion only
        v = max(0.0, v)
        
        # Limit
        v = np.clip(v, 0.0, self.max_v)
        omega = np.clip(omega, -self.max_omega, self.max_omega)
        
        return v, omega
    
    def control_loop(self):
        v, omega = self.compute_control()
        
        # Log every 0.5s
        self.log_count += 1
        if self.log_count % 10 == 0 and self.current_state and self.reference_path:
            t = time.time() - self.start_time
            goal_x = self.reference_path[self.current_waypoint_idx].pose.position.x
            goal_y = self.reference_path[self.current_waypoint_idx].pose.position.y
            error_x = self.current_state['x'] - goal_x
            error_y = self.current_state['y'] - goal_y
            error_dist = math.sqrt(error_x**2 + error_y**2)
            
            with open(self.csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    f'{t:.2f}',
                    f'{self.current_state["x"]:.4f}',
                    f'{self.current_state["y"]:.4f}',
                    f'{self.current_state["theta"]:.4f}',
                    f'{goal_x:.4f}', f'{goal_y:.4f}',
                    f'{error_x:.4f}', f'{error_y:.4f}',
                    f'{error_dist:.4f}', f'{v:.4f}', f'{omega:.4f}'
                ])
            
            if self.log_count % 50 == 0:
                self.get_logger().info(f'Logged {self.log_count} | Error: {error_dist:.3f}m | v={v:.3f}, w={omega:.3f}')
        
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(omega)
        self.cmd_vel_pub.publish(cmd)
    
    def destroy_node(self):
        self.get_logger().info(f'Complete! {self.log_count} samples saved to {self.csv_file}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = AdaptiveSMCController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

