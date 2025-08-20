#!/usr/bin/env python3
"""
Simple launch file for testing just the hardware driver
No EKF, no IMU filtering - just the basic driver functionality
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='R2L',
        description='Robot type [X3, R2, R2L]'
    )
    
    nav_use_rotvel_arg = DeclareLaunchArgument(
        'nav_use_rotvel',
        default_value='false',
        description='Use rotational velocity for navigation'
    )
    
    # Get launch configurations
    robot_type = LaunchConfiguration('robot_type')
    nav_use_rotvel = LaunchConfiguration('nav_use_rotvel')
    
    # Hardware driver node (main driver)
    hardware_driver_node = Node(
        package='yahboomcar_bringup',
        executable='mcnamu_driver.py',
        name='yahboomcar_driver',
        output='screen',
        parameters=[{
            'car_type': robot_type,
            'xlinear_speed_limit': 1.0,
            'ylinear_speed_limit': 1.0, 
            'angular_speed_limit': 5.0,
            'nav_use_rotvel': nav_use_rotvel,
            'imu_link': 'imu_link'
        }],
        remappings=[
            ('/pub_vel', '/vel_raw'),
            ('/pub_imu', '/imu/imu_raw'),
            ('/pub_mag', '/mag/mag_raw')
        ]
    )

    return LaunchDescription([
        robot_type_arg,
        nav_use_rotvel_arg,
        hardware_driver_node
    ])