#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('tilt_lidar_node')
    rviz_config = os.path.join(pkg_share, 'rviz', 'view_tilt_lidar.rviz')
    # bag_path    = os.path.join(pkg_share, 'bag', 'lidar_servo_imu_pcl_slow_10hz_5hz_10hz')
    # bag_path    = os.path.join(pkg_share, 'bag', 'subset')
    # bag_path    = os.path.join(pkg_share, 'bag', 'lidar_servo_pcl_slow_10hz_15hz')
    bag_path = os.path.join(pkg_share, 'bag', 'lidarServo')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-l', bag_path, '--clock'],
            output='screen'
        ),
        
        Node(
            package='tilt_lidar_node',
            executable='tilt_lidar_node',
            name='tilt_lidar_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
    ])