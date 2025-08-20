#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable, EqualsSubstitution, AndSubstitution, NotSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Use joint_state_publisher_gui'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz'
    )
    
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Use EKF localization'
    )
    
    nav_use_rotvel_arg = DeclareLaunchArgument(
        'nav_use_rotvel',
        default_value='false',
        description='Navigation use rotational velocity'
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value=EnvironmentVariable('ROBOT_TYPE', default_value='R2L'),
        description='Robot type from environment variable'
    )

    # Get launch configurations
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz') 
    use_ekf = LaunchConfiguration('use_ekf')
    nav_use_rotvel = LaunchConfiguration('nav_use_rotvel')
    robot_type = LaunchConfiguration('robot_type')

    # Create robot type condition (equivalent to ROS1's $(eval arg('robot_type') == 'R2L'))
    robot_type_is_r2l = EqualsSubstitution(robot_type, 'R2L')

    # Get package directories
    yahboomcar_description_dir = get_package_share_directory('yahboomcar_description')
    yahboomcar_bringup_dir = get_package_share_directory('yahboomcar_bringup')
    
    # Robot description (only for R2L robot type)
    xacro_file = os.path.join(yahboomcar_description_dir, 'urdf', 'yahboomcar_R2.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Robot state publisher (only for R2L robot type)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(robot_type_is_r2l),
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # Joint state publisher (GUI version) - only for R2L robot type
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(AndSubstitution(use_gui, robot_type_is_r2l))
    )

    # Joint state publisher (non-GUI version) - only for R2L robot type
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(AndSubstitution(
            NotSubstitution(use_gui),  # NOT use_gui
            robot_type_is_r2l
        ))
    )

    # Hardware driver node (only for R2L robot type)
    mcnamu_driver_node = Node(
        package='yahboomcar_bringup',
        executable='mcnamu_driver.py',
        name='driver_node',
        output='screen',
        condition=IfCondition(robot_type_is_r2l),
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

    # Base node (odometry publisher) - EKF mode
    base_node_ekf = Node(
        package='yahboomcar_bringup',
        executable='base_node',
        name='odometry_publisher',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[{
            'odom_frame': 'odom',
            'base_footprint_frame': 'base_footprint',
            'linear_scale_x': 1.0,
            'linear_scale_y': 1.0,
            'wheelbase': 0.25,
            'pub_odom_tf': False
        }],
        remappings=[
            ('/sub_vel', '/vel_raw'),
            ('/pub_odom', '/odom_raw')
        ]
    )

    # Base node (odometry publisher) - non-EKF mode
    base_node_no_ekf = Node(
        package='yahboomcar_bringup',
        executable='base_node',
        name='odometry_publisher',
        output='screen',
        condition=UnlessCondition(use_ekf),
        parameters=[{
            'odom_frame': 'odom',
            'base_footprint_frame': 'base_footprint',
            'linear_scale_x': 1.0,
            'linear_scale_y': 1.0,
            'wheelbase': 0.25,
            'pub_odom_tf': True
        }],
        remappings=[
            ('/sub_vel', '/vel_raw'),
            ('/pub_odom', '/odom')
        ]
    )

    # IMU filter node
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[{
            'fixed_frame': 'base_link',
            'use_mag': True,
            'publish_tf': False,
            'use_magnetic_field_msg': True,
            'world_frame': 'enu',
            'orientation_stddev': 0.05,
            'angular_scale': 1.05
        }],
        remappings=[
            ('/sub_imu', '/imu/imu_raw'),
            ('/sub_mag', '/mag/mag_raw'),
            ('/pub_imu', '/imu/imu_data'),
            ('/pub_mag', '/mag/mag_field')
        ]
    )

    # EKF localization node
    robot_localization_file_path = os.path.join(yahboomcar_bringup_dir, 'param', 'robot_localization.yaml')
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[robot_localization_file_path, {
            'odom_frame': '/odom',
            'world_frame': '/odom',
            'base_link_frame': '/base_footprint'
        }],
        remappings=[
            ('odometry/filtered', 'odom'),
            ('/imu0', '/imu/imu_data'),
            ('/odom0', 'odom_raw')
        ]
    )

    # Include joystick control launch
    yahboomcar_ctrl_dir = get_package_share_directory('yahboomcar_ctrl')
    joy_launch = IncludeLaunchDescription(
        os.path.join(yahboomcar_ctrl_dir, 'launch', 'yahboom_joy.launch.py')
    )

    # RViz node
    rviz_config_file = os.path.join(yahboomcar_bringup_dir, 'rviz', 'odom.rviz')
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
        use_gui_arg,
        use_rviz_arg,
        use_ekf_arg,
        nav_use_rotvel_arg,
        robot_type_arg,

        # Core nodes
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        
        # Hardware and odometry
        mcnamu_driver_node,
        base_node_ekf,
        base_node_no_ekf,
        
        # Sensor processing
        imu_filter_node,
        ekf_node,
        
        # Control and visualization
        joy_launch,
        rviz_node
    ])