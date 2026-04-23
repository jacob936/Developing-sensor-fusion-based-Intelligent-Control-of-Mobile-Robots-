
#!/usr/bin/env python3
"""
Ultra-Simple Data Logger - Guaranteed to Work
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Path, Odometry
import csv
import time
import os
from datetime import datetime

class UltraSimpleLogger(Node):
    def __init__(self):
        super().__init__('ultra_simple_logger')
        
        # Create output file
        self.output_dir = '/root/sensor_fusion_ws/experiment_data'
        os.makedirs(self.output_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file = os.path.join(self.output_dir, f'baseline_{timestamp}.csv')
        
        # Write headers
        with open(self.csv_file, 'w') as f:
            f.write('time_sec,state_x,state_y,ref_x,ref_y,cmd_v,cmd_omega\n')
        
        self.get_logger().info(f'Logging to: {self.csv_file}')
        
        # Minimal subscriptions
        self.create_subscription(PoseWithCovarianceStamped, '/state_estimated', self.log_state, 10)
        self.create_subscription(Path, '/reference_path', self.log_path, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.log_cmd, 10)
        
        self.state = None
        self.ref = None
        self.cmd = None
        self.start_time = time.time()
        self.count = 0
        
        # Log every 0.5 seconds (not every callback)
        self.timer = self.create_timer(0.5, self.write_row)
    
    def log_state(self, msg):
        self.state = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    
    def log_path(self, msg):
        if msg.poses:
            self.ref = (msg.poses[0].pose.position.x, msg.poses[0].pose.position.y)
    
    def log_cmd(self, msg):
        self.cmd = (msg.twist.linear.x, msg.twist.angular.z)
    
    def write_row(self):
        if self.state and self.ref and self.cmd:
            t = time.time() - self.start_time
            with open(self.csv_file, 'a') as f:
                f.write(f'{t:.2f},{self.state[0]:.4f},{self.state[1]:.4f},'
                       f'{self.ref[0]:.4f},{self.ref[1]:.4f},'
                       f'{self.cmd[0]:.4f},{self.cmd[1]:.4f}\n')
            self.count += 1
            if self.count % 10 == 0:
                self.get_logger().info(f'Logged {self.count} rows')
    
    def destroy_node(self):
        self.get_logger().info(f'Done! Logged {self.count} rows to {self.csv_file}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UltraSimpleLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



