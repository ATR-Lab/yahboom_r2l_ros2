#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    yahboomcar_ctrl_dir = get_package_share_directory('yahboomcar_ctrl')

    # Hardware driver node
    mcnamu_driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver.py',
        name='driver_node',
        output='screen',
        parameters=[{
            'xlinear_speed_limit': 1.0,
            'ylinear_speed_limit': 1.0,
            'angular_speed_limit': 5.0,
            'imu_link': 'imu_link'
        }],
        remappings=[
            ('/pub_vel', '/vel_raw'),
            ('/pub_imu', '/imu/imu_raw'),
            ('/pub_mag', '/mag/mag_raw')
        ]
    )

    # Include joystick control launch
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yahboomcar_ctrl_dir, 'launch', 'yahboom_joy.launch.py')
        )
    )

    return LaunchDescription([
        mcnamu_driver_node,
        joy_launch
    ])