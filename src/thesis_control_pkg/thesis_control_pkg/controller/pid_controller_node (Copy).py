#!/usr/bin/env python3
"""
PID Controller with Built-in Data Logging
For Thesis Experiment: ASMC vs PID Comparison
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

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        
        # ================================
        # PID Gains (TUNED for TurtleBot3)
        # ================================
        self.kp_linear = 1.0      # Increased from 0.8
        self.ki_linear = 0.02     # Small integral
        self.kd_linear = 0.1      # Small derivative
        
        self.kp_angular = 2.0     # Increased from 1.5
        self.ki_angular = 0.03    # Small integral
        self.kd_angular = 0.2     # Small derivative
        
        self.max_v = 0.22
        self.max_omega = 1.0
        
        # Integral anti-windup
        self.integral_limit = 5.0
        self.prev_error_linear = 0.0
        self.prev_error_angular = 0.0
        self.integral_linear = 0.0
        self.integral_angular = 0.0
        
        # Waypoint tracking
        self.current_waypoint_idx = 0
        self.waypoint_threshold = 0.15
        
        # ================================
        # DATA LOGGING (Same as ASMC)
        # ================================
        self.output_dir = '/root/sensor_fusion_ws/experiment_data'
        os.makedirs(self.output_dir, exist_ok=True)
        
        timestamp = int(time.time())
        self.csv_file = f'{self.output_dir}/pid_{timestamp}.csv'
        
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time_sec',
                'state_x', 'state_y', 'state_theta',
                'ref_x', 'ref_y',
                'error_x', 'error_y', 'error_dist',
                'cmd_v', 'cmd_omega'
            ])
        
        self.start_time = time.time()
        self.log_count = 0
        self.get_logger().info('===========================================')
        self.get_logger().info('PID CONTROLLER WITH LOGGING')
        self.get_logger().info(f'Output: {self.csv_file}')
        self.get_logger().info('For Thesis: ASMC vs PID Comparison')
        self.get_logger().info('===========================================')
        # ================================
        
        self.current_state = None
        self.reference_path = None
        
        self.state_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/state_estimated', self.state_callback, 10)
        
        self.path_sub = self.create_subscription(
            Path, '/reference_path', self.path_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('PID Controller initialized (for comparison)')
    
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
    
    def advance_waypoint(self, distance):
        if distance < self.waypoint_threshold:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.reference_path):
                self.current_waypoint_idx = 0
                self.get_logger().info('*** Completed waypoint loop! ***')
            else:
                wp = self.reference_path[self.current_waypoint_idx].pose.position
                self.get_logger().warn(f'*** WP reached! Next: ({wp.x:.2f}, {wp.y:.2f}) ***')
            return True
        return False
    
    def compute_pid_control(self, error, error_type='linear'):
        """PID control law with anti-windup"""
        dt = 0.05  # 20 Hz control loop
        
        if error_type == 'linear':
            kp, ki, kd = self.kp_linear, self.ki_linear, self.kd_linear
            prev_err = self.prev_error_linear
            integral = self.integral_linear
        else:
            kp, ki, kd = self.kp_angular, self.ki_angular, self.kd_angular
            prev_err = self.prev_error_angular
            integral = self.integral_angular
        
        # Proportional
        p_term = kp * error
        
        # Integral (with anti-windup)
        integral = integral + error * dt
        integral = np.clip(integral, -self.integral_limit, self.integral_limit)
        i_term = ki * integral
        
        # Derivative
        d_term = kd * (error - prev_err) / dt
        
        # Total
        output = p_term + i_term + d_term
        
        # Update state
        if error_type == 'linear':
            self.prev_error_linear = error
            self.integral_linear = integral
        else:
            self.prev_error_angular = error
            self.integral_angular = integral
        
        return output
    
    def compute_control(self):
        if self.current_state is None:
            self.get_logger().warn('NO STATE!')
            return 0.0, 0.0
        
        if self.reference_path is None or len(self.reference_path) == 0:
            self.get_logger().warn('NO PATH!')
            return 0.0, 0.0
        
        # Get target waypoint
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
        
        # Recompute after potential advance
        goal_x = self.reference_path[self.current_waypoint_idx].pose.position.x
        goal_y = self.reference_path[self.current_waypoint_idx].pose.position.y
        dx = goal_x - x
        dy = goal_y - y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Desired heading
        desired_theta = math.atan2(dy, dx)
        theta_error = self.normalize_angle(desired_theta - theta)
        
        # PID Control
        v = self.compute_pid_control(distance, 'linear')
        omega = self.compute_pid_control(theta_error, 'angular')
        
        # Ensure forward motion only
        v = max(0.0, v)
        
        # Limit
        v = np.clip(v, 0.0, self.max_v)
        omega = np.clip(omega, -self.max_omega, self.max_omega)
        
        self.get_logger().debug(f'PID: v={v:.3f}, ω={omega:.3f}, dist={distance:.2f}, err={theta_error:.2f}')
        
        return v, omega
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def control_loop(self):
        v, omega = self.compute_control()
        
        # ===== LOG DATA (Same as ASMC) =====
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
                    f'{goal_x:.4f}',
                    f'{goal_y:.4f}',
                    f'{error_x:.4f}',
                    f'{error_y:.4f}',
                    f'{error_dist:.4f}',
                    f'{v:.4f}',
                    f'{omega:.4f}'
                ])
            
            if self.log_count % 50 == 0:
                self.get_logger().info(f'Logged {self.log_count} samples | Error: {error_dist:.3f}m | v={v:.3f}')
        # ===================================
        
        # Publish command
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(omega)
        self.cmd_vel_pub.publish(cmd)
        
        # Debug: verify publishing
        if self.log_count % 100 == 0:
            self.get_logger().info(f'Publishing cmd_vel: v={v:.3f}, ω={omega:.3f}')
    
    def destroy_node(self):
        self.get_logger().info('===========================================')
        self.get_logger().info(f'PID EXPERIMENT COMPLETE!')
        self.get_logger().info(f'Total samples: {self.log_count}')
        self.get_logger().info(f'Duration: {time.time() - self.start_time:.1f}s')
        self.get_logger().info(f'File: {self.csv_file}')
        self.get_logger().info('===========================================')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

