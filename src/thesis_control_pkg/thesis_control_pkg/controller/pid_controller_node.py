#!/usr/bin/env python3
"""
PID Controller with Simple Obstacle Avoidance
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import csv
import time
import os

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        
        # ===== PID GAINS =====
        self.kp_linear = 0.8
        self.ki_linear = 0.01
        self.kd_linear = 0.05
        self.kp_angular = 1.5
        self.ki_angular = 0.02
        self.kd_angular = 0.10
        
        self.max_v = 0.15
        self.max_omega = 0.7
        self.integral_limit = 3.0
        
        # ===== OBSTACLE AVOIDANCE =====
        self.avoidance_distance = 0.6
        self.avoidance_speed = 0.08
        self.escape_distance = 0.25
        self.in_avoidance = False
        self.avoidance_start_time = None
        
        # ===== WAYPOINT TRACKING =====
        self.current_waypoint_idx = 0
        self.waypoint_threshold = 0.20
        
        # ===== PID STATE =====
        self.integral_linear = 0.0
        self.integral_angular = 0.0
        self.prev_error_linear = 0.0
        self.prev_error_angular = 0.0
        
        # ===== LOGGING =====
        self.output_dir = '/root/sensor_fusion_ws/experiment_data'
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = int(time.time())
        self.csv_file = f'{self.output_dir}/pid_obstacle_{timestamp}.csv'
        
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time_sec', 'state_x', 'state_y', 'state_theta',
                'ref_x', 'ref_y', 'error_dist',
                'cmd_v', 'cmd_omega',
                'obstacle_detected', 'min_distance', 'mode'
            ])
        
        self.start_time = time.time()
        self.log_count = 0
        
        self.get_logger().info('===========================================')
        self.get_logger().info('PID Controller + Obstacle Avoidance')
        self.get_logger().info(f'Avoidance distance: {self.avoidance_distance}m')
        self.get_logger().info(f'Output: {self.csv_file}')
        self.get_logger().info('===========================================')
        
        self.current_state = None
        self.reference_path = None
        self.lidar_scan = None
        
        self.state_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/state_estimated', self.state_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/reference_path', self.path_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        
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
        if len(self.reference_path) > 0 and self.log_count == 0:
            self.get_logger().info(f'Path: {len(self.reference_path)} waypoints')
    
    def lidar_callback(self, msg):
        self.lidar_scan = msg.ranges
    
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
            self.integral_linear = 0.0
            self.integral_angular = 0.0
            
            if self.reference_path is None or len(self.reference_path) == 0:
                return False
            
            if self.current_waypoint_idx >= len(self.reference_path):
                self.current_waypoint_idx = 0
                self.get_logger().info('*** Loop complete! ***')
            return True
        return False
    
    def get_current_waypoint(self):
        if self.reference_path is None or len(self.reference_path) == 0:
            return None
        if self.current_waypoint_idx >= len(self.reference_path):
            self.current_waypoint_idx = 0
        return self.reference_path[self.current_waypoint_idx]
    
    def check_obstacles(self):
        if self.lidar_scan is None or len(self.lidar_scan) == 0:
            return False, 10.0
        
        front_start = 270
        front_end = 450
        front_distances = [d for d in self.lidar_scan[front_start:front_end] if 0.1 < d < 10.0]
        
        if len(front_distances) == 0:
            return False, 10.0
        
        min_dist = min(front_distances)
        
        if min_dist < self.avoidance_distance:
            left_distances = [d for d in self.lidar_scan[180:270] if 0.1 < d < 10.0]
            right_distances = [d for d in self.lidar_scan[450:540] if 0.1 < d < 10.0]
            left_clear = min(left_distances) if left_distances else 10.0
            right_clear = min(right_distances) if right_distances else 10.0
            direction = 1.0 if right_clear > left_clear else -1.0
            return True, min_dist, direction
        
        return False, min_dist, 1.0
    
    def compute_avoidance_command(self, min_dist, direction):
        v = self.avoidance_speed * (min_dist / self.avoidance_distance)
        v = np.clip(v, 0.02, self.avoidance_speed)
        omega = direction * 0.6 * (1.0 - min_dist / self.avoidance_distance)
        omega = np.clip(omega, -self.max_omega, self.max_omega)
        return v, omega
    
    def compute_control(self):
        if self.current_state is None or self.reference_path is None or len(self.reference_path) == 0:
            return 0.0, 0.0, False, 10.0
        
        # ===== OBSTACLE AVOIDANCE =====
        obstacle_result = self.check_obstacles()
        obstacle_detected = obstacle_result[0]
        min_dist = obstacle_result[1]
        direction = obstacle_result[2] if len(obstacle_result) > 2 else 1.0
        
        # Escape mode
        if self.lidar_scan is not None:
            all_distances = [d for d in self.lidar_scan if 0.1 < d < 10.0]
            if all_distances and min(all_distances) < self.escape_distance:
                if not self.in_avoidance:
                    self.in_avoidance = True
                    self.avoidance_start_time = time.time()
                    self.get_logger().warn('*** ESCAPE MODE! ***')
                
                elapsed = time.time() - self.avoidance_start_time
                if elapsed < 1.0:
                    return -0.1, 0.0, True, min_dist
                elif elapsed < 2.0:
                    return 0.0, 0.8, True, min_dist
                else:
                    self.in_avoidance = False
        
        if obstacle_detected and not self.in_avoidance:
            self.in_avoidance = True
            self.avoidance_start_time = time.time()
            v, omega = self.compute_avoidance_command(min_dist, direction)
            self.get_logger().info(f'Avoiding at {min_dist:.2f}m')
            return v, omega, True, min_dist
        else:
            self.in_avoidance = False
        
        # ===== NORMAL TRACKING =====
        current_wp = self.get_current_waypoint()
        if current_wp is None:
            return 0.0, 0.0, False, 10.0
        
        goal_x = current_wp.pose.position.x
        goal_y = current_wp.pose.position.y
        
        x = self.current_state['x']
        y = self.current_state['y']
        theta = self.current_state['theta']
        
        dx = goal_x - x
        dy = goal_y - y
        distance = math.sqrt(dx**2 + dy**2)
        
        self.advance_waypoint(distance)
        
        current_wp = self.get_current_waypoint()
        if current_wp is None:
            return 0.0, 0.0, False, 10.0
        
        goal_x = current_wp.pose.position.x
        goal_y = current_wp.pose.position.y
        dx = goal_x - x
        dy = goal_y - y
        distance = math.sqrt(dx**2 + dy**2)
        
        desired_theta = math.atan2(dy, dx)
        theta_error = self.normalize_angle(desired_theta - theta)
        
        # ===== PID CONTROL =====
        dt = 0.05
        
        self.integral_linear = np.clip(self.integral_linear + distance * dt, -self.integral_limit, self.integral_limit)
        d_error_linear = (distance - self.prev_error_linear) / dt
        v = self.kp_linear * distance + self.ki_linear * self.integral_linear + self.kd_linear * d_error_linear
        self.prev_error_linear = distance
        
        self.integral_angular = np.clip(self.integral_angular + theta_error * dt, -self.integral_limit, self.integral_limit)
        d_error_angular = (theta_error - self.prev_error_angular) / dt
        omega = self.kp_angular * theta_error + self.ki_angular * self.integral_angular + self.kd_angular * d_error_angular
        self.prev_error_angular = theta_error
        
        v = max(0.0, v)
        v = np.clip(v, 0.0, self.max_v)
        omega = np.clip(omega, -self.max_omega, self.max_omega)
        
        return v, omega, False, min_dist
    
    def control_loop(self):
        v, omega, obstacle_active, min_dist = self.compute_control()
        
        self.log_count += 1
        if self.log_count % 10 == 0 and self.current_state and self.reference_path:
            t = time.time() - self.start_time
            
            current_wp = self.get_current_waypoint()
            goal_x = current_wp.pose.position.x if current_wp else 0.0
            goal_y = current_wp.pose.position.y if current_wp else 0.0
            
            error_dist = math.sqrt((self.current_state['x'] - goal_x)**2 + 
                                  (self.current_state['y'] - goal_y)**2)
            
            mode = 'avoidance' if obstacle_active else 'tracking'
            
            with open(self.csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    f'{t:.2f}',
                    f'{self.current_state["x"]:.4f}',
                    f'{self.current_state["y"]:.4f}',
                    f'{self.current_state["theta"]:.4f}',
                    f'{goal_x:.4f}', f'{goal_y:.4f}',
                    f'{error_dist:.4f}',
                    f'{v:.4f}', f'{omega:.4f}',
                    f'{obstacle_active}', f'{min_dist:.4f}', mode
                ])
            
            if self.log_count % 50 == 0:
                self.get_logger().info(f'Logged {self.log_count} | Error: {error_dist:.3f}m | Mode: {mode}')
        
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(omega)
        self.cmd_vel_pub.publish(cmd)
    
    def destroy_node(self):
        self.get_logger().info(f'Done! {self.log_count} samples -> {self.csv_file}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

