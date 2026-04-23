#!/usr/bin/env python3
"""
ASMC Controller - Fixed Waypoint Navigation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Path
import math
import numpy as np

class AdaptiveSMCController(Node):
    def __init__(self):
        super().__init__('asmc_controller_node')
        
        # Controller gains
        self.k_linear = 0.8
        self.k_angular = 1.5
        self.max_v = 0.22
        self.max_omega = 1.0
        
        # Waypoint tracking - REDUCED threshold!
        self.current_waypoint_idx = 0
        self.waypoint_threshold = 0.15  # REDUCED from 0.5!
        
        self.current_state = None
        self.reference_path = None
        
        self.state_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/state_estimated', self.state_callback, 10)
        
        self.path_sub = self.create_subscription(
            Path, '/reference_path', self.path_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('ASMC Controller initialized')
        self.get_logger().info(f'Waypoint threshold: {self.waypoint_threshold}m')
    
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
            for i, wp in enumerate(self.reference_path[:3]):  # Log first 3 waypoints
                self.get_logger().info(f'  WP{i}: ({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f})')
    
    def quaternion_to_yaw(self, quat):
        return math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                         1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
    
    def advance_waypoint(self, distance):
        if distance < self.waypoint_threshold:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.reference_path):
                self.current_waypoint_idx = 0
                self.get_logger().info('*** Completed loop! ***')
            else:
                wp = self.reference_path[self.current_waypoint_idx].pose.position
                self.get_logger().warn(f'*** WP{self.current_waypoint_idx-1} reached! Next: ({wp.x:.2f}, {wp.y:.2f}) ***')
            return True
        return False
    
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
        
        # Current state
        x = self.current_state['x']
        y = self.current_state['y']
        theta = self.current_state['theta']
        
        # Compute error BEFORE advancing
        dx = goal_x - x
        dy = goal_y - y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Log BEFORE advancing
        self.get_logger().info(f'WP{self.current_waypoint_idx}: Dist={distance:.2f}m, Robot=({x:.2f},{y:.2f}), Goal=({goal_x:.2f},{goal_y:.2f})')
        
        # Advance waypoint if close
        self.advance_waypoint(distance)
        
        # Get NEW target after potential advance
        goal_x = self.reference_path[self.current_waypoint_idx].pose.position.x
        goal_y = self.reference_path[self.current_waypoint_idx].pose.position.y
        
        # Recompute error
        dx = goal_x - x
        dy = goal_y - y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Desired heading
        desired_theta = math.atan2(dy, dx)
        theta_error = self.normalize_angle(desired_theta - theta)
        
        # Compute velocities
        v = self.k_linear * distance
        omega = self.k_angular * theta_error
        
        # Limit
        v = np.clip(v, 0.0, self.max_v)
        omega = np.clip(omega, -self.max_omega, self.max_omega)
        
        self.get_logger().warn(f'CMD: v={v:.3f}, omega={omega:.3f}, theta_err={theta_error:.2f}rad')
        
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

