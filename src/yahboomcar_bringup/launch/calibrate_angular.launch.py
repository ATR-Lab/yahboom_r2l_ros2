#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Angular velocity calibration node
    calibrate_angular_node = Node(
        package='yahboomcar_bringup',
        executable='calibrate_angular.py',
        name='calibrate_angular',
        output='screen'
    )

    return LaunchDescription([
        calibrate_angular_node
    ])