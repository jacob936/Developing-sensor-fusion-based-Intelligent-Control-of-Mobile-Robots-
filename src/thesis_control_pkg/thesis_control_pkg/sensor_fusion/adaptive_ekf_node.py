#!/usr/bin/env python3
"""
Adaptive Extended Kalman Filter for Sensor Fusion
Fuses IMU and Odometry data with adaptive noise covariance
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import os

os.environ['ROS_DISTRO'] = 'jazzy'

class AdaptiveEKF(Node):
    def __init__(self):
        super().__init__('adaptive_ekf_node')
 
        # State vector: [x, y, theta, v, omega]
        self.state = np.zeros(5)
        self.P = np.eye(5) * 0.1
        
        # Process noise covariance
        self.Q = np.diag([0.01, 0.01, 0.001, 0.01, 0.01])
        
        # Measurement noise covariance
        self.R_odom = np.diag([0.01, 0.01, 0.001])
        self.R_imu = np.diag([0.001])
        
        # Adaptive parameters
        self.innovation_threshold = 3.0
        self.adaptation_rate = 0.1
        
        # First measurement flag
        self.first_odom = True
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        
        # Publisher
        self.state_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/state_estimated', 10)
        
        # Store last control input
        self.last_v = 0.0
        self.last_omega = 0.0
        self.last_time = None
        
        self.get_logger().info('Adaptive EKF initialized')
    
    def predict(self, dt):
        """Prediction step using kinematic model"""
        if self.last_time is None:
            return
        
        v = self.last_v
        omega = self.last_omega
        theta = self.state[2]
        
        # State prediction
        self.state[0] += v * np.cos(theta) * dt
        self.state[1] += v * np.sin(theta) * dt
        self.state[2] += omega * dt
        
        # Normalize theta
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))
        
        # Jacobian of state transition
        F = np.eye(5)
        F[0, 2] = -v * np.sin(theta) * dt
        F[1, 2] = v * np.cos(theta) * dt
        
        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q
    
    def update_odometry(self, z_odom):
        """Update step with odometry measurement"""
        # First odometry initializes state
        if self.first_odom:
            self.state[:3] = z_odom
            self.first_odom = False
            self.get_logger().info(f'First odometry: ({z_odom[0]:.3f}, {z_odom[1]:.3f}, {z_odom[2]:.3f})')
            return
        
        # Measurement function
        H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0]
        ])
        
        # Innovation
        y = z_odom - self.state[:3]
        y[2] = np.arctan2(np.sin(y[2]), np.cos(y[2]))
        
        # Kalman gain
        S = H @ self.P @ H.T + self.R_odom
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.state += K @ y
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))
        
        # Covariance update
        I = np.eye(len(self.state))
        self.P = (I - K @ H) @ self.P
    
    def update_imu(self, z_imu):
        """Update step with IMU measurement"""
        H = np.array([[0, 0, 1, 0, 0]])
        y = np.array([z_imu - self.state[2]])
        y[0] = np.arctan2(np.sin(y[0]), np.cos(y[0]))
        
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.state[2] += K[0, 0] * y[0]
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))
        
        I = np.eye(len(self.state))
        self.P = (I - K @ H) @ self.P
    
    def odom_callback(self, msg):
        """Odometry callback"""
        current_time = self.get_clock().now()
        
        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds * 1e-9
            if dt > 0:
                self.predict(dt)
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        z_odom = np.array([x, y, theta])
        self.update_odometry(z_odom)
        
        self.last_v = msg.twist.twist.linear.x
        self.last_omega = msg.twist.twist.angular.z
        self.last_time = current_time
        self.publish_state()
    
    def imu_callback(self, msg):
        """IMU callback"""
        quat = msg.orientation
        _, _, theta = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.update_imu(theta)
        self.publish_state()
    
    def publish_state(self):
        """Publish estimated state with covariance"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        
        msg.pose.pose.position.x = self.state[0]
        msg.pose.pose.position.y = self.state[1]
        
        quat = quaternion_from_euler(0, 0, self.state[2])
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        
        msg.pose.covariance[0] = self.P[0, 0]
        msg.pose.covariance[7] = self.P[1, 1]
        msg.pose.covariance[35] = self.P[2, 2]
        
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    ekf_node = AdaptiveEKF()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

