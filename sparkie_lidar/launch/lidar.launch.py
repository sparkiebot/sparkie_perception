#!/usr/bin/env python3
# Got from https://github.com/Slamtec/sllidar_ros2/blob/main/launch/sllidar_c1_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    # Path to the configuration file
    pkg_share = FindPackageShare(package='sparkie_lidar').find('sparkie_lidar')
    
    return LaunchDescription([
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[
                os.path.join(pkg_share, 'params', 'sllidar.yaml')
            ],
            remappings=[('/scan', '/sparkie/scan')],
            output='screen'),
    ])