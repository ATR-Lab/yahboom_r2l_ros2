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
    
    car_id_arg = DeclareLaunchArgument(
        'car_id',
        default_value='1',
        description='Unique car identifier for multiplayer racing (1-4)'
    )

    # Get launch configurations
    robot_type = LaunchConfiguration('robot_type')
    car_id = LaunchConfiguration('car_id')
    
    # Create namespace from car_id
    namespace = ['/car_', car_id]

    # Hardware driver node
    mcnamu_driver_node = Node(
        package='yahboomcar_bringup',
        executable='mcnamu_driver.py',
        name='driver_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            'car_id': car_id,
            'xlinear_speed_limit': 1.0,
            'ylinear_speed_limit': 1.0,
            'angular_speed_limit': 5.0,
            'imu_link': 'imu_link'  # Keep as simple string - namespace handled by node namespace
        }],
        remappings=[
            ('pub_vel', 'vel_raw'),         # /car_X/vel_raw (raw velocity data)
            ('pub_imu', 'imu/imu_raw'),     # /car_X/imu/imu_raw (raw IMU data)
            ('pub_mag', 'mag/mag_raw'),     # /car_X/mag/mag_raw (raw magnetometer data)
            ('voltage', 'voltage'),         # /car_X/voltage (battery data)
            ('joint_states', 'joint_states'), # /car_X/joint_states (joint position data)
            ('edition', 'edition')          # /car_X/edition (robot version info)
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
        car_id_arg,
        mcnamu_driver_node
    ])