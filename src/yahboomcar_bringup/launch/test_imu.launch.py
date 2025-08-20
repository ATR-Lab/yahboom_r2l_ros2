#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, EqualsSubstitution, NotSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value=EnvironmentVariable('ROBOT_TYPE', default_value='R2L'),
        description='Robot type [X1,X3,X3plus,R2,X7,R2L]'
    )

    # Get launch configurations
    debug = LaunchConfiguration('debug')
    use_rviz = LaunchConfiguration('use_rviz')
    robot_type = LaunchConfiguration('robot_type')

    # Create robot type condition (equivalent to ROS1's $(eval arg('robot_type') == 'R2L'))
    robot_type_is_r2l = EqualsSubstitution(robot_type, 'R2L')

    # Get package directory
    yahboomcar_bringup_dir = get_package_share_directory('yahboomcar_bringup')

    # Hardware driver node (only for R2L robot type)
    mcnamu_driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver.py',
        name='driver_node',
        output='screen',
        condition=IfCondition(robot_type_is_r2l),
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

    # Warning node (for non-R2L robot types)
    warning_node = Node(
        package='yahboomcar_bringup',
        executable='warning.py',
        name='warning',
        output='screen',
        condition=IfCondition(NotSubstitution(robot_type_is_r2l))
    )

    # Base node (odometry publisher)
    base_node = Node(
        package='yahboomcar_bringup',
        executable='base_node',
        name='odometry_publisher',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_footprint_frame': 'base_footprint',
            'linear_scale_x': 1.1,
            'linear_scale_y': 1.0
        }],
        remappings=[
            ('/sub_vel', '/vel_raw'),
            ('/pub_odom', '/odom_raw')
        ]
    )

    # IMU filter node
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'fixed_frame': 'base_link',
            'use_mag': False,
            'publish_tf': False,
            'use_magnetic_field_msg': False,
            'world_frame': 'enu',
            'orientation_stddev': 0.05,
            'angular_scale': 1.0
        }],
        remappings=[
            ('/sub_imu', '/imu/imu_raw'),
            ('/sub_mag', '/mag/mag_raw'),
            ('/pub_imu', '/imu/imu_data'),
            ('/pub_mag', '/mag/mag_field')
        ]
    )

    # Static transform publisher (only for R2L robot type)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        condition=IfCondition(robot_type_is_r2l),
        arguments=['-0.1', '0.01', '0.01', '0', '0', '0', '/base_link', '/imu_link']
    )

    # RViz node
    rviz_config_file = os.path.join(yahboomcar_bringup_dir, 'rviz', 'test_imu.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='odom_rviz',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        # Arguments
        debug_arg,
        use_rviz_arg,
        robot_type_arg,

        # Nodes
        mcnamu_driver_node,
        warning_node,
        base_node,
        imu_filter_node,
        static_tf_node,
        rviz_node
    ])