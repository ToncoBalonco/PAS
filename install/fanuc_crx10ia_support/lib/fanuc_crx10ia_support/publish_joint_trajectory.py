#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        
        # Create publisher for joint trajectory controller
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Timer to publish trajectories periodically
        self.timer = self.create_timer(8.0, self.publish_trajectory)
        self.trajectory_set = 0
        
        self.get_logger().info('Joint trajectory publisher started')
    
    def publish_trajectory(self):
        msg = JointTrajectory()
        
        # Set joint names
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # Define different trajectory points
        if self.trajectory_set == 0:
            # Trajectory 1: Smooth movement
            points = []
            for i in range(5):
                point = JointTrajectoryPoint()
                positions = [
                    math.sin(i * 0.5) * math.pi/4,
                    math.cos(i * 0.5) * math.pi/6,
                    math.sin(i * 0.3) * math.pi/3,
                    math.cos(i * 0.4) * math.pi/4,
                    math.sin(i * 0.6) * math.pi/6,
                    i * math.pi/8
                ]
                point.positions = positions
                point.time_from_start = rclpy.duration.Duration(seconds=i+1).to_msg()
                points.append(point)
            msg.points = points
            
        else:
            # Trajectory 2: Different pattern
            points = []
            for i in range(4):
                point = JointTrajectoryPoint()
                positions = [
                    math.cos(i * 0.8) * math.pi/3,
                    math.sin(i * 0.7) * math.pi/4,
                    math.cos(i * 0.5) * math.pi/6,
                    math.sin(i * 0.9) * math.pi/3,
                    math.cos(i * 0.6) * math.pi/4,
                    -i * math.pi/6
                ]
                point.positions = positions
                point.time_from_start = rclpy.duration.Duration(seconds=i*2+1).to_msg()
                points.append(point)
            msg.points = points
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published joint trajectory with {len(msg.points)} points')
        
        # Alternate between trajectory sets
        self.trajectory_set = 1 - self.trajectory_set

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()