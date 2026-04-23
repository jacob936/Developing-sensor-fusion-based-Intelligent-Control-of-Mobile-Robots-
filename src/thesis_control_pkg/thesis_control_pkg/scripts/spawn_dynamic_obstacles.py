#!/usr/bin/env python3
"""
Spawn Dynamic Obstacles in Gazebo
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SpawnEntity
from geometry_msgs.msg import Pose, Twist
import math
import time

class DynamicObstacleSpawner(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_spawner')
        
        # Define obstacle positions and motion patterns
        self.obstacles = [
            {'name': 'obstacle_1', 'x': 2.0, 'y': 0.0, 'pattern': 'linear'},
            {'name': 'obstacle_2', 'x': -2.0, 'y': 1.0, 'pattern': 'circular'},
            {'name': 'obstacle_3', 'x': 0.0, 'y': -2.0, 'pattern': 'linear'},
        ]
        
        # Service clients
        self.set_state_client = self.create_client(SetModelState, '/set_entity_state')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # Timer - update obstacle positions at 10 Hz
        self.timer = self.create_timer(0.1, self.update_obstacles)
        
        self.start_time = self.get_clock().now()
        
        self.get_logger().info('Dynamic Obstacle Spawner initialized')
    
    def spawn_obstacle(self, name, x, y):
        """Spawn a box obstacle in Gazebo"""
        # Wait for service
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        # Box model XML
        box_xml = """
        <sdf version="1.6">
          <model name="{}">
            <static>false</static>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <box>
                    <size>0.5 0.5 0.5</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>0.5 0.5 0.5</size>
                  </box>
                </geometry>
                <material>
                  <ambient>1 0 0 1</ambient>
                  <diffuse>1 0 0 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """.format(name)
        
        request = SpawnEntity.Request()
        request.name = name
        request.xml = box_xml
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = 0.25
        
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Spawned {name} at ({x}, {y})')
        else:
            self.get_logger().error(f'Failed to spawn {name}')
    
    def update_obstacles(self):
        """Update obstacle positions"""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        
        for obs in self.obstacles:
            # Wait for service
            while not self.set_state_client.wait_for_service(timeout_sec=1.0):
                pass
            
            # Calculate new position based on pattern
            if obs['pattern'] == 'linear':
                x = obs['x'] + 0.2 * math.sin(elapsed * 0.5)
                y = obs['y']
            elif obs['pattern'] == 'circular':
                radius = 1.0
                x = obs['x'] + radius * math.cos(elapsed * 0.3)
                y = obs['y'] + radius * math.sin(elapsed * 0.3)
            else:
                x = obs['x']
                y = obs['y']
            
            # Set new state
            request = SetModelState.Request()
            request.model_state.model_name = obs['name']
            request.model_state.pose.position.x = x
            request.model_state.pose.position.y = y
            request.model_state.pose.position.z = 0.25
            request.model_state.reference_frame = 'world'
            
            future = self.set_state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
    
    def start(self):
        """Spawn all obstacles"""
        for obs in self.obstacles:
            self.spawn_obstacle(obs['name'], obs['x'], obs['y'])
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    spawner = DynamicObstacleSpawner()
    spawner.start()
    rclpy.spin(spawner)
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

