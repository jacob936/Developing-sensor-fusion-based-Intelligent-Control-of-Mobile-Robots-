#!/usr/bin/env python3
"""
ASMC Controller with Simple Obstacle Avoidance
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from sensor_msgs.msg import LaserScan  # FIXED: Was geometry_msgs!
from nav_msgs.msg import Path
import math
import numpy as np

class AdaptiveSMCController(Node):
    def __init__(self):
        super().__init__('asmc_controller_node')
        
        # Controller gains
        self.k_linear = 0.5
        self.k_angular = 1.0
        self.max_v = 0.22
        self.max_omega = 1.0
        
        # Obstacle avoidance parameters
        self.obstacle_threshold = 1.0  # meters
        self.avoidance_strength = 0.8
        
        # State
        self.current_state = None
        self.reference_path = None
        self.scan_data = None
        
        # Subscribers
        self.state_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/state_estimated', self.state_callback, 10)
        
        self.path_sub = self.create_subscription(
            Path, '/reference_path', self.path_callback, 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('ASMC Controller with Obstacle Avoidance initialized')
    
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
    
    def scan_callback(self, msg):
        self.scan_data = msg
    
    def quaternion_to_yaw(self, quat):
        return math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                         1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
    
    def get_obstacle_direction(self):
        """Check if obstacle is in front and return avoidance direction"""
        if self.scan_data is None:
            return 0.0
        
        ranges = np.array(self.scan_data.ranges)
        valid = np.isfinite(ranges)
        
        # Check front 90 degrees
        center_idx = len(ranges) // 2
        front_range = int(center_idx * 0.25)
        front_ranges = ranges[center_idx - front_range : center_idx + front_range]
        front_valid = front_ranges[valid[center_idx - front_range : center_idx + front_range]]
        
        if len(front_valid) == 0:
            return 0.0
        
        min_dist = np.min(front_valid)
        
        if min_dist < self.obstacle_threshold:
            avoidance = self.avoidance_strength * (1.0 - min_dist / self.obstacle_threshold)
            self.get_logger().info(f'Obstacle at {min_dist:.2f}m - avoiding!')
            return avoidance
        else:
            return 0.0
    
    def compute_control(self):
        if self.current_state is None or self.reference_path is None:
            return 0.0, 0.0
        
        if len(self.reference_path) == 0:
            return 0.0, 0.0
        
        # Get target waypoint
        goal_x = self.reference_path[0].pose.position.x
        goal_y = self.reference_path[0].pose.position.y
        
        # Current state
        x = self.current_state['x']
        y = self.current_state['y']
        theta = self.current_state['theta']
        
        # Compute error to goal
        dx = goal_x - x
        dy = goal_y - y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Desired heading
        desired_theta = math.atan2(dy, dx)
        theta_error = self.normalize_angle(desired_theta - theta)
        
        # Base velocities
        v = self.k_linear * distance
        omega = self.k_angular * theta_error
        
        # Add obstacle avoidance
        avoidance = self.get_obstacle_direction()
        if avoidance != 0.0:
            omega += avoidance  # Turn to avoid
            v *= 0.5  # Slow down
            self.get_logger().info(f'Avoiding! v={v:.2f}, omega={omega:.2f}')
        
        # Limit velocities
        v = np.clip(v, 0.0, self.max_v)
        omega = np.clip(omega, -self.max_omega, self.max_omega)
        
        return v, omega
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def control_loop(self):
        v, omega = self.compute_control()
        
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(omega)
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = AdaptiveSMCController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

