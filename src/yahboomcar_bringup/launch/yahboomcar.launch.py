#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value=EnvironmentVariable('ROBOT_TYPE', default_value='R2L'),
        description='Robot type [X1,X3,X3plus,R2,X7,R2L]'
    )

    # Get launch configuration
    robot_type = LaunchConfiguration('robot_type')

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

    # Note: Warning node was commented out in original launch file
    # warning_node = Node(
    #     package='yahboomcar_bringup',
    #     executable='warning.py',
    #     name='warning',
    #     output='screen'
    #     # TODO: Add condition for robot_type != 'R2'
    # )

    return LaunchDescription([
        robot_type_arg,
        mcnamu_driver_node
    ])