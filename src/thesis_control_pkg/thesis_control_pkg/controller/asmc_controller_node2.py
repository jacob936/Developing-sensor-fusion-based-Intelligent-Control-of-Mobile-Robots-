
#!/usr/bin/env python3
"""
Adaptive Sliding Mode Controller for TurtleBot3
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
        
        # Controller gains (TUNED for waypoint tracking)
        self.k_linear = 0.5    # Linear velocity gain
        self.k_angular = 1.0   # Angular velocity gain
        self.max_v = 0.22      # Max linear velocity (TurtleBot3 limit)
        self.max_omega = 1.0   # Max angular velocity
        
        # Waypoint tracking
        self.current_waypoint_idx = 0
        self.waypoint_threshold = 0.3  # meters
        
        # State
        self.current_state = None
        self.reference_path = None
        
        # Subscribers
        self.state_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/state_estimated',
            self.state_callback,
            10)
        
        self.path_sub = self.create_subscription(
            Path,
            '/reference_path',
            self.path_callback,
            10)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Timer (20 Hz control loop)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('ASMC Controller initialized')
    
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
    
    def compute_control(self):
        if self.current_state is None or self.reference_path is None:
            return 0.0, 0.0
        
        if len(self.reference_path) == 0:
            return 0.0, 0.0
        
        # Get current target waypoint
        if self.current_waypoint_idx >= len(self.reference_path):
            self.current_waypoint_idx = 0  # Loop
        
        target = self.reference_path[self.current_waypoint_idx].pose
        goal_x = target.position.x
        goal_y = target.position.y
        
        # Current state
        x = self.current_state['x']
        y = self.current_state['y']
        theta = self.current_state['theta']
        
        # Compute error to goal
        dx = goal_x - x
        dy = goal_y - y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if waypoint reached
        if distance < self.waypoint_threshold:
            self.current_waypoint_idx += 1
            self.get_logger().info(f'Waypoint {self.current_waypoint_idx-1} reached!')
            if self.current_waypoint_idx >= len(self.reference_path):
                self.current_waypoint_idx = 0
                self.get_logger().info('Completed waypoint loop!')
            return 0.0, 0.0  # Stop at waypoint
        
        # Compute desired heading to goal
        desired_theta = math.atan2(dy, dx)
        
        # Heading error
        theta_error = self.normalize_angle(desired_theta - theta)
        
        # Compute velocities
        v = self.k_linear * distance
        omega = self.k_angular * theta_error
        
        # Limit velocities
        v = np.clip(v, -self.max_v, self.max_v)
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
        
        # Publish command
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

