#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')
        
        # Create publisher for forward position controller
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/forward_position_controller/commands', 
            10
        )
        
        # Timer to publish positions periodically
        self.timer = self.create_timer(3.0, self.publish_positions)
        self.position_set = 0
        
        self.get_logger().info('Joint position publisher started')
    
    def publish_positions(self):
        msg = Float64MultiArray()
        
        # Define different joint position sets
        position_sets = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Home position
            [math.pi/4, -math.pi/6, math.pi/3, -math.pi/4, math.pi/6, 0.0],
            [-math.pi/4, math.pi/6, -math.pi/3, math.pi/4, -math.pi/6, math.pi/2],
            [math.pi/2, -math.pi/4, math.pi/6, -math.pi/2, math.pi/4, -math.pi/2]
        ]
        
        msg.data = position_sets[self.position_set]
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Published joint positions: {msg.data}')
        
        # Cycle through position sets
        self.position_set = (self.position_set + 1) % len(position_sets)

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()