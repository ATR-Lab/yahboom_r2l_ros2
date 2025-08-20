#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Yahboom keyboard teleop node
        Node(
            package='yahboomcar_ctrl',
            executable='yahboom_keyboard',
            name='yahboom_keyboard',
            output='screen',
            parameters=[{
                'linear_speed_limit': 1.0,
                'angular_speed_limit': 5.0
            }]
        )
    ])