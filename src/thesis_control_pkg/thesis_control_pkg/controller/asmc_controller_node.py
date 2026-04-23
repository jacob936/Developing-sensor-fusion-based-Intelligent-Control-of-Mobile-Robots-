#!/usr/bin/env python3
"""
ASMC Controller with IMPROVED Obstacle Avoidance
Fixed: Earlier detection + Smoother turning to maintain orientation
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

class AdaptiveSMCController(Node):
    def __init__(self):
        super().__init__('asmc_controller_node')
        
        # ===== CONTROLLER GAINS =====
        self.kp_linear = 0.5
        self.kp_angular = 1.0
        self.max_v = 0.12
        self.max_omega = 0.6  # Reduced from 0.8 for smoother turns
        
        # ===== OBSTACLE AVOIDANCE (FIXED) =====
        self.detect_distance = 0.8      # DETECT EARLIER (was 0.5m)
        self.avoidance_distance = 0.6   # Start avoidance at 0.6m
        self.safe_distance = 0.9        # Must be clear to 0.9m before resuming
        self.avoidance_speed = 0.10     # Slightly faster (was 0.08)
        self.avoidance_duration = 3.0   # Reduced from 4.0s
        
        # Smoother turning
        self.turn_rate_factor = 0.4     # Reduced from 0.7 (gentler turns)
        
        self.in_avoidance = False
        self.avoidance_start_time = 0.0
        self.avoidance_direction = 1.0
        self.avoidance_distance_traveled = 0.0
        self.last_position = None
        self.clear_counter = 0          # Hysteresis counter
        
        # ===== WAYPOINT TRACKING =====
        self.current_waypoint_idx = 0
        self.waypoint_threshold = 0.20
        
        # ===== DATA LOGGING =====
        self.output_dir = '/root/sensor_fusion_ws/experiment_data'
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = int(time.time())
        self.csv_file = f'{self.output_dir}/asmc_obstacle_{timestamp}.csv'
        
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
        self.get_logger().info('ASMC + IMPROVED Obstacle Avoidance')
        self.get_logger().info(f'Detect distance: {self.detect_distance}m (EARLIER)')
        self.get_logger().info(f'Avoidance distance: {self.avoidance_distance}m')
        self.get_logger().info(f'Safe distance: {self.safe_distance}m')
        self.get_logger().info(f'Turn rate factor: {self.turn_rate_factor} (SMOOTHER)')
        self.get_logger().info(f'Output: {self.csv_file}')
        self.get_logger().info('===========================================')
        
        self.current_state = None
        self.reference_path = None
        self.lidar_scan = None
        self.front_start = 0
        self.front_end = 100
        self.left_start = 0
        self.left_end = 50
        self.right_start = 50
        self.right_end = 100
        
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
            self.get_logger().info(f'Path received: {len(self.reference_path)} waypoints')
    
    def lidar_callback(self, msg):
        """Store LiDAR scan and calculate sectors"""
        self.lidar_scan = msg.ranges
        total_ranges = len(msg.ranges)
        
        # Calculate sectors on first scan
        if self.front_end == 0:
            front_center = total_ranges // 2
            sector_45deg = int(total_ranges * 45 / 360)
            
            self.front_start = max(0, front_center - sector_45deg)
            self.front_end = min(total_ranges, front_center + sector_45deg)
            self.left_start = max(0, front_center - 2 * sector_45deg)
            self.left_end = front_center
            self.right_start = front_center
            self.right_end = min(total_ranges, front_center + 2 * sector_45deg)
            
            self.get_logger().info(f'LiDAR sectors: front[{self.front_start}:{self.front_end}], left[{self.left_start}:{self.left_end}], right[{self.right_start}:{self.right_end}]')
    
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
            
            if self.reference_path is None or len(self.reference_path) == 0:
                return False
            
            if self.current_waypoint_idx >= len(self.reference_path):
                self.current_waypoint_idx = 0
                self.get_logger().info('*** Completed loop! ***')
            else:
                wp = self.reference_path[self.current_waypoint_idx].pose.position
                self.get_logger().info(f'WP {self.current_waypoint_idx}: ({wp.x:.2f}, {wp.y:.2f})')
            return True
        return False
    
    def get_current_waypoint(self):
        if self.reference_path is None or len(self.reference_path) == 0:
            return None
        if self.current_waypoint_idx >= len(self.reference_path):
            self.current_waypoint_idx = 0
        return self.reference_path[self.current_waypoint_idx]
    
    def scan_sector(self, start_idx, end_idx, max_range=10.0):
        """Get minimum distance in a sector"""
        if self.lidar_scan is None or start_idx >= end_idx:
            return max_range
        
        distances = [d for d in self.lidar_scan[start_idx:end_idx] if 0.1 < d < max_range]
        return min(distances) if distances else max_range
    
    def check_obstacles(self):
        """Check for obstacles with EARLIER detection"""
        if self.lidar_scan is None:
            return False, 10.0, 1.0
        
        front_min = self.scan_sector(self.front_start, self.front_end, self.safe_distance)
        left_min = self.scan_sector(self.left_start, self.left_end, self.safe_distance)
        right_min = self.scan_sector(self.right_start, self.right_end, self.safe_distance)
        
        # Log every 50 cycles
        if self.log_count % 50 == 0:
            self.get_logger().info(f'LiDAR: front={front_min:.2f}m, left={left_min:.2f}m, right={right_min:.2f}m')
        
        # DETECT EARLIER (at detect_distance instead of avoidance_distance)
        if front_min < self.detect_distance:
            direction = 1.0 if right_min > left_min else -1.0
            
            # Only warn if actually close
            if front_min < self.avoidance_distance:
                self.get_logger().warn(f'*** OBSTACLE! front={front_min:.2f}, left={left_min:.2f}, right={right_min:.2f}, turn={direction} ***')
            
            return True, front_min, direction
        
        return False, front_min, 1.0
    
    def compute_avoidance_command(self, direction):
        """Compute SMOOTHER velocity commands"""
        v = self.avoidance_speed
        # REDUCED TURN RATE for better orientation maintenance
        omega = direction * self.max_omega * self.turn_rate_factor
        return v, omega
    
    def compute_control(self):
        if self.current_state is None or self.reference_path is None or len(self.reference_path) == 0:
            return 0.0, 0.0, False, 10.0, 'error'
        
        current_time = time.time()
        
        # ===== OBSTACLE AVOIDANCE =====
        obstacle_detected, min_dist, direction = self.check_obstacles()
        
        if obstacle_detected and min_dist < self.avoidance_distance:
            if not self.in_avoidance:
                self.in_avoidance = True
                self.avoidance_start_time = current_time
                self.avoidance_direction = direction
                self.avoidance_distance_traveled = 0.0
                self.last_position = (self.current_state['x'], self.current_state['y'])
                self.clear_counter = 0
                self.get_logger().warn(f'*** AVOIDANCE STARTED at {min_dist:.2f}m! Turn {self.avoidance_direction} ***')
        else:
            # No obstacle or far enough
            if self.in_avoidance:
                self.clear_counter += 1
            else:
                self.clear_counter = 0
        
        if self.in_avoidance:
            elapsed = current_time - self.avoidance_start_time
            
            if self.last_position:
                dx = self.current_state['x'] - self.last_position[0]
                dy = self.current_state['y'] - self.last_position[1]
                self.avoidance_distance_traveled += math.sqrt(dx**2 + dy**2)
                self.last_position = (self.current_state['x'], self.current_state['y'])
            
            # Check if path is clear with HYSTERESIS
            is_clear = (
                not obstacle_detected or 
                (obstacle_detected and min_dist >= self.safe_distance)
            )
            
            can_resume = (
                elapsed >= self.avoidance_duration and
                self.avoidance_distance_traveled >= 0.8 and  # Reduced from 1.0m
                self.clear_counter >= 10  # Must be clear for 10 cycles (0.5s)
            )
            
            if can_resume:
                self.in_avoidance = False
                self.clear_counter = 0
                self.get_logger().info(f'*** RESUMING (traveled {self.avoidance_distance_traveled:.2f}m, clear {self.clear_counter} cycles) ***')
            else:
                v, omega = self.compute_avoidance_command(self.avoidance_direction)
                mode = 'avoidance_nav'
                if elapsed < 0.5:
                    mode = 'avoidance_turn'
                    omega = self.avoidance_direction * self.max_omega * 0.5
                    v = 0.05
                return v, omega, True, min_dist, mode
        
        # ===== NORMAL WAYPOINT TRACKING =====
        current_wp = self.get_current_waypoint()
        if current_wp is None:
            return 0.0, 0.0, False, 10.0, 'error'
        
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
            return 0.0, 0.0, False, 10.0, 'error'
        
        goal_x = current_wp.pose.position.x
        goal_y = current_wp.pose.position.y
        dx = goal_x - x
        dy = goal_y - y
        distance = math.sqrt(dx**2 + dy**2)
        
        desired_theta = math.atan2(dy, dx)
        theta_error = self.normalize_angle(desired_theta - theta)
        
        # ===== P CONTROL =====
        v = self.kp_linear * distance
        omega = self.kp_angular * theta_error
        
        v = max(0.0, v)
        v = np.clip(v, 0.0, self.max_v)
        omega = np.clip(omega, -self.max_omega, self.max_omega)
        
        return v, omega, False, min_dist, 'tracking'
    
    def control_loop(self):
        v, omega, obstacle_active, min_dist, mode = self.compute_control()
        
        self.log_count += 1
        if self.log_count % 10 == 0 and self.current_state and self.reference_path:
            t = time.time() - self.start_time
            
            current_wp = self.get_current_waypoint()
            if current_wp is not None:
                goal_x = current_wp.pose.position.x
                goal_y = current_wp.pose.position.y
            else:
                goal_x = 0.0
                goal_y = 0.0
            
            error_dist = math.sqrt((self.current_state['x'] - goal_x)**2 + 
                                  (self.current_state['y'] - goal_y)**2)
            
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
    controller = AdaptiveSMCController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

