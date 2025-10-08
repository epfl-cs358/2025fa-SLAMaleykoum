# ros 1 launch
# <launch>

#   <node pkg="loam_back_and_forth" type="scanRegistration_bf" name="scanRegistration_bf" output="screen"/>
#   <node pkg="loam_back_and_forth" type="laserOdometry_bf" name="laserOdometry_bf" output="screen"/>
#   <node pkg="loam_back_and_forth" type="laserMapping_bf" name="laserMapping_bf" output="screen"/>
#   <node pkg="loam_back_and_forth" type="transformMaintenance_bf" name="transformMaintenance_bf" output="screen"/>

#   <node launch-prefix="nice" pkg="rviz" type="rviz" name="rviz" args="-d $(find loam_back_and_forth)/rviz_cfg/loam_back_and_forth.rviz"/>

# </launch>

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

pkg_share = get_package_share_directory('loam_back_and_forth')
rviz_config = os.path.join(pkg_share, 'rviz_cfg', 'full_scan.rviz')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='loam_back_and_forth',
            executable='scanRegistration_bf',
            name='scanRegistration_bf',
            output='screen'
        ),
        Node(
            package='loam_back_and_forth',
            executable='laserOdometry_bf',
            name='laserOdometry_bf',
            output='screen'
        ),
        Node(
            package='loam_back_and_forth',
            executable='laserMapping_bf',
            name='laserMapping_bf',
            output='screen'
        ),
        Node(
            package='loam_back_and_forth',
            executable='transformMaintenance_bf',
            name='transformMaintenance_bf',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])

