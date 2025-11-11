

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Joint trajectory publisher node
    joint_trajectory_publisher = Node(
        package='fanuc_crx10ia_support',
        executable='publish_joint_trajectory',
        name='publish_joint_trajectory',
        output='screen'
    )
    
    return LaunchDescription([
        joint_trajectory_publisher
    ])