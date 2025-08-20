#!/usr/bin/env python3
"""
Simple launch file for testing robot state publisher only
No GUI, no RViz - just the core robot description functionality
"""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Path to R2 URDF file (used for R2L robot)
    urdf_file = PathJoinSubstitution([
        FindPackageShare('yahboomcar_description'),
        'urdf',
        'yahboomcar_R2.urdf.xacro'
    ])
    
    # Robot description parameter
    robot_description = Command(['xacro ', urdf_file])
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
        output='screen'
    )
    
    # Joint state publisher (no GUI)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])