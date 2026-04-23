#!/usr/bin/env python3
"""
Waypoint Trajectory Generator for TurtleBot3
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import os

os.environ['ROS_DISTRO'] = 'jazzy'

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator_node')
        
        # Define waypoints (X, Y in meters)
        # Adjust based on your warehouse map
        self.waypoints = [
            [0.0, 0.0],      # Start (center)
            [1.0, 0.0],      # Waypoint 1
            [1.0, 1.0],      # Waypoint 2
            [0.0, 1.0],      # Waypoint 3
            [-1.0, 1.0],     # Waypoint 4
            [-1.0, 0.0],     # Waypoint 5
            [-1.0, -1.0],    # Waypoint 6
            [0.0, -1.0],     # Waypoint 7
            [1.0, -1.0],     # Waypoint 8
            [0.0, 0.0],      # Return to start
        ]
        
        self.current_waypoint_idx = 0
        self.waypoint_threshold = 0.3  # meters (distance to consider waypoint reached)
        
        # Publisher
        self.path_pub = self.create_publisher(Path, '/reference_path', 10)
        
        # Timer - publish path at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_trajectory)
        
        # Subscriber - get robot state to track progress
        self.state_sub = self.create_subscription(
            PoseStamped,
            '/state_estimated',
            self.state_callback,
            10)
        
        self.current_state = None
        
        self.get_logger().info('Waypoint Trajectory Generator initialized')
    
    def state_callback(self, msg):
        """Receive robot state"""
        self.current_state = [
            msg.pose.position.x,
            msg.pose.position.y
        ]
    
    def generate_waypoint_path(self):
        """Generate path from current position to next waypoint"""
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        
        if self.current_waypoint_idx >= len(self.waypoints):
            self.current_waypoint_idx = 0  # Loop back
        
        # Add current waypoint and next few waypoints
        for i in range(3):
            idx = (self.current_waypoint_idx + i) % len(self.waypoints)
            wp = self.waypoints[idx]
            
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.0
            
            # Orientation (face next waypoint)
            next_idx = (idx + 1) % len(self.waypoints)
            next_wp = self.waypoints[next_idx]
            yaw = math.atan2(next_wp[1] - wp[1], next_wp[0] - wp[0])
            
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            
            path.poses.append(pose)
        
        return path
    
    def check_waypoint_reached(self):
        """Check if current waypoint is reached"""
        if self.current_state is None:
            return False
        
        target = self.waypoints[self.current_waypoint_idx]
        dx = self.current_state[0] - target[0]
        dy = self.current_state[1] - target[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < self.waypoint_threshold:
            self.get_logger().info(f'Waypoint {self.current_waypoint_idx} reached!')
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self.current_waypoint_idx = 0
                self.get_logger().info('Completed full waypoint loop!')
            return True
        
        return False
    
    def publish_trajectory(self):
        """Publish reference trajectory"""
        self.check_waypoint_reached()
        path = self.generate_waypoint_path()
        self.path_pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    generator = TrajectoryGenerator()
    rclpy.spin(generator)
    generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

