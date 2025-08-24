cmake_minimum_required(VERSION 3.8)
project(yahboomcar_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(imu_filter_madgwick REQUIRED)

# Build C++ executables
add_executable(base_node src/base_node.cpp src/base.cpp)

# Add include directories
target_include_directories(base_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(base_node 
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  nav_msgs 
  tf2_ros
  tf2
  tf2_geometry_msgs
)

# Install C++ executables
install(TARGETS
  base_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install Python scripts
install(PROGRAMS
  scripts/mcnamu_driver.py
  scripts/mcnamu_driver_sim.py
  scripts/calibrate_angular.py
  scripts/calibrate_linear.py
  scripts/transform_utils.py
  scripts/patrol_4ROS.py
  scripts/patrol_a1.py
  scripts/Turn_Right.py
  scripts/test.py
  scripts/warning.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  param
  config
  rviz
  cfg
  DESTINATION share/${PROJECT_NAME}/
)

# Export include directories and dependencies
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
