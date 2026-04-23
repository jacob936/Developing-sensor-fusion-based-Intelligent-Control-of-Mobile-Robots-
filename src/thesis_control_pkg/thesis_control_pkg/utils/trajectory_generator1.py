#!/usr/bin/env python3
"""
Simple Waypoint Trajectory Generator
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator_node')
        
        # Simple square path (adjust for your warehouse)
        self.waypoints = [
            [0.0, 0.0],
            [0.5, 0.0],
            [0.5, 0.5],
            [0.0, 0.5],
            [0.0, 0.0],
        ]
        
        self.path_pub = self.create_publisher(Path, '/reference_path', 10)
        
        # Subscribe to correct message type
        self.state_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/state_estimated',
            self.state_callback,
            10)
        
        self.current_state = None
        self.timer = self.create_timer(0.5, self.publish_path)
        
        self.get_logger().info('Waypoint Generator initialized')
    
    def state_callback(self, msg):
        self.current_state = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ]
    
    def publish_path(self):
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        self.path_pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
