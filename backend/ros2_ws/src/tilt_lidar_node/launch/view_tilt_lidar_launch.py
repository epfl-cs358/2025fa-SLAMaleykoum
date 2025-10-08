#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('tilt_lidar_node')
    rviz_config = os.path.join(pkg_share, 'rviz', 'view_tilt_lidar.rviz')

    return LaunchDescription([
        Node(
            package='tilt_lidar_node',
            executable='tilt_lidar_node',
            name='tilt_lidar_node',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
    ])