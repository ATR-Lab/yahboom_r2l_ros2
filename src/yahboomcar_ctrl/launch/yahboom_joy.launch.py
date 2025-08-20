#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Joy node (from joy package)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Yahboom joystick teleop node
        Node(
            package='yahboomcar_ctrl',
            executable='yahboom_joy',
            name='yahboom_joy',
            output='screen',
            parameters=[{
                'linear_speed_limit': 1.0,
                'angular_speed_limit': 5.0
            }]
        )
    ])