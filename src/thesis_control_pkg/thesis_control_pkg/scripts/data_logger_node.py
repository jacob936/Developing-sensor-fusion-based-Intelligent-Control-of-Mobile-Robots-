#!/usr/bin/env python3
"""
Data Logger for Thesis Experiments
Records state, control, error, and performance metrics to CSV
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Path, Odometry
import csv
import time
import math
from datetime import datetime
import os

class DataLogger(Node):
    def __init__(self, experiment_name="baseline"):
        super().__init__('data_logger_node')
        
        # Create output directory
        self.output_dir = '/root/sensor_fusion_ws/experiment_data'
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Create CSV file with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file = f'{self.output_dir}/{experiment_name}_{timestamp}.csv'
        
        # Initialize CSV with headers
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp_sec',
                'state_x', 'state_y', 'state_theta',
                'state_cov_xx', 'state_cov_yy',  # EKF uncertainty
                'reference_x', 'reference_y',    # Current waypoint
                'tracking_error_x', 'tracking_error_y', 'tracking_error_dist',
                'cmd_v', 'cmd_omega',             # Control commands
                'actual_v', 'actual_omega',       # From odometry
                'waypoint_idx',
                'experiment_phase'               # tracking/avoiding/escape
            ])
        
        # State variables
        self.current_state = None
        self.current_reference = None
        self.current_cmd = None
        self.current_odom = None
        self.current_waypoint_idx = 0
        
        # Subscribers
        self.state_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/state_estimated', self.state_callback, 10)
        
        self.path_sub = self.create_subscription(
            Path, '/reference_path', self.path_callback, 10)
        
        self.cmd_sub = self.create_subscription(
            TwistStamped, '/cmd_vel', self.cmd_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Timer - log at 10 Hz
        self.timer = self.create_timer(0.1, self.log_data)
        
        self.start_time = time.time()
        self.get_logger().info(f'Data Logger started. Saving to: {self.csv_file}')
    
    def state_callback(self, msg):
        self.current_state = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quaternion_to_yaw(msg.pose.pose.orientation),
            'cov_xx': msg.pose.covariance[0],
            'cov_yy': msg.pose.covariance[7]
        }
    
    def path_callback(self, msg):
        if len(msg.poses) > 0:
            # Track current waypoint index (simplified)
            self.current_reference = {
                'x': msg.poses[0].pose.position.x,
                'y': msg.poses[0].pose.position.y
            }
    
    def cmd_callback(self, msg):
        self.current_cmd = {
            'v': msg.twist.linear.x,
            'omega': msg.twist.angular.z
        }
    
    def odom_callback(self, msg):
        self.current_odom = {
            'v': msg.twist.linear.x,
            'omega': msg.twist.angular.z
        }
    
    def quaternion_to_yaw(self, quat):
        return math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                         1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
    
    def log_data(self):
        if self.current_state is None or self.current_reference is None:
            return
        
        # Compute tracking error
        error_x = self.current_state['x'] - self.current_reference['x']
        error_y = self.current_state['y'] - self.current_reference['y']
        error_dist = math.sqrt(error_x**2 + error_y**2)
        
        # Get control values (default to 0 if not received)
        cmd_v = self.current_cmd['v'] if self.current_cmd else 0.0
        cmd_omega = self.current_cmd['omega'] if self.current_cmd else 0.0
        actual_v = self.current_odom['v'] if self.current_odom else 0.0
        actual_omega = self.current_odom['omega'] if self.current_odom else 0.0
        
        # Determine experiment phase (simplified)
        phase = 'tracking'
        if error_dist > 0.5:
            phase = 'large_error'
        
        # Write to CSV
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                time.time() - self.start_time,
                self.current_state['x'],
                self.current_state['y'],
                self.current_state['theta'],
                self.current_state['cov_xx'],
                self.current_state['cov_yy'],
                self.current_reference['x'],
                self.current_reference['y'],
                error_x,
                error_y,
                error_dist,
                cmd_v,
                cmd_omega,
                actual_v,
                actual_omega,
                self.current_waypoint_idx,
                phase
            ])
    
    def get_summary(self):
        """Compute summary statistics from logged data"""
        import pandas as pd
        try:
            df = pd.read_csv(self.csv_file)
            summary = {
                'duration_sec': df['timestamp_sec'].max(),
                'mean_tracking_error': df['tracking_error_dist'].mean(),
                'max_tracking_error': df['tracking_error_dist'].max(),
                'rmse_tracking_error': math.sqrt((df['tracking_error_dist']**2).mean()),
                'mean_cmd_v': df['cmd_v'].abs().mean(),
                'mean_cmd_omega': df['cmd_omega'].abs().mean(),
                'total_samples': len(df)
            }
            return summary
        except Exception as e:
            self.get_logger().error(f'Could not compute summary: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    
    # Get experiment name from command line
    import sys
    experiment_name = sys.argv[1] if len(sys.argv) > 1 else 'baseline'
    
    logger = DataLogger(experiment_name)
    rclpy.spin(logger)
    logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

