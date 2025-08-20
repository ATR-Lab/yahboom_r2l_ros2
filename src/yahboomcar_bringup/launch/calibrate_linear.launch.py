#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Linear speed calibration node
    calibrate_linear_node = Node(
        package='yahboomcar_bringup',
        executable='calibrate_linear.py',
        name='calibrate_linear',
        output='screen'
    )

    return LaunchDescription([
        calibrate_linear_node
    ])