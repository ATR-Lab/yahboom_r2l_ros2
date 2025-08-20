#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable, EqualsSubstitution, NotSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
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
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value=EnvironmentVariable('ROBOT_TYPE', default_value='R2'),
        description='Robot type [X1,X3,X3plus,R2,X7]'
    )

    # Get launch configurations
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    robot_type = LaunchConfiguration('robot_type')

    # Create robot type condition (equivalent to ROS1's $(eval arg('robot_type') == 'R2'))
    robot_type_is_r2 = EqualsSubstitution(robot_type, 'R2')

    # Get package directories
    yahboomcar_description_dir = get_package_share_directory('yahboomcar_description')
    yahboomcar_bringup_dir = get_package_share_directory('yahboomcar_bringup')
    yahboomcar_ctrl_dir = get_package_share_directory('yahboomcar_ctrl')

    # Hardware driver node (only for R2 robot type)
    mcnamu_driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver.py',
        name='driver_node',
        output='screen',
        condition=IfCondition(robot_type_is_r2),
        parameters=[{
            'xlinear_speed_limit': 1.0,
            'ylinear_speed_limit': 1.0,
            'angular_speed_limit': 5.0,
            'imu_link': 'imu_link'
        }],
        remappings=[
            ('/pub_vel', '/imu/vel_raw'),  # Note: Different from other launch files
            ('/pub_imu', '/imu/imu_raw'),
            ('/pub_mag', '/mag/mag_raw')
        ]
    )

    # Warning node (for non-R2 robot types)
    warning_node = Node(
        package='yahboomcar_bringup',
        executable='warning.py',
        name='warning',
        output='screen',
        condition=IfCondition(NotSubstitution(robot_type_is_r2))
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

    # IMU calibration node
    imu_calib_file = os.path.join(yahboomcar_bringup_dir, 'param', 'imu_calib.yaml')
    imu_calib_node = Node(
        package='imu_calib',
        executable='apply_calib',
        name='apply_calib',
        output='screen',
        parameters=[{
            'calib_file': imu_calib_file,
            'calibrate_gyros': True
        }],
        remappings=[
            ('/sub_imu', '/imu/imu_raw'),
            ('/sub_mag', '/mag/mag_raw'),
            ('/pub_imu', '/imu/imu_calib'),
            ('/pub_mag', '/mag/mag_calib')
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
            'use_mag': True,
            'publish_tf': False,
            'use_magnetic_field_msg': True,
            'world_frame': 'enu',
            'orientation_stddev': 0.05,
            'angular_scale': 1.08
        }],
        remappings=[
            ('/sub_imu', '/imu/imu_calib'),
            ('/sub_mag', '/mag/mag_calib'),
            ('/pub_imu', '/imu/imu_data'),
            ('/pub_mag', '/mag/mag_field')
        ]
    )

    # Static transform publisher
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['-0.1', '0.01', '0.01', '1.5707', '3.1415', '0', '/base_link', '/imu_link']
    )

    # EKF localization node
    robot_localization_file_path = os.path.join(yahboomcar_bringup_dir, 'param', 'robot_localization.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization',
        output='screen',
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

    # Robot description
    xacro_file = os.path.join(yahboomcar_description_dir, 'urdf', 'yahboomcar_R2.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # Joint state publisher (GUI version)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(use_gui)
    )

    # Joint state publisher (non-GUI version)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_gui)
    )

    # Include joystick control launch
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yahboomcar_ctrl_dir, 'launch', 'yahboom_joy.launch.py')
        )
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
        robot_type_arg,

        # Hardware and odometry
        mcnamu_driver_node,
        warning_node,
        base_node,

        # IMU calibration and filtering
        imu_calib_node,
        imu_filter_node,
        static_tf_node,
        ekf_node,

        # Robot model
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,

        # Control and visualization
        joy_launch,
        rviz_node
    ])