# Yahboom R2L ROS2 Migration

ROS2 Humble port of the Yahboom R2L robot platform, migrated from ROS1 Noetic.

## Overview

This workspace contains the ROS2 implementation of the Yahboom R2L robot system, including:
- **Hardware drivers** for robot control and sensor data
- **Robot description** (URDF/meshes) for visualization and simulation  
- **Custom message definitions** for robot-specific data types
- **Control interfaces** for joystick/keyboard teleop
- **Launch files** for system startup and testing

## Prerequisites

### System Requirements
- **Ubuntu 22.04 LTS** 
- **ROS2 Humble** (installed and configured)
- **Python 3.10+**

### ROS2 Dependencies
Install the following ROS2 packages:
```bash
sudo apt update
sudo apt install -y \
    ros-humble-xacro \
    ros-humble-robot-localization \
    ros-humble-imu-filter-madgwick \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui
```

### Python Dependencies
Install required Python packages:
```bash
# Serial communication for hardware interface
pip3 install pyserial
```

### Hardware Interface Library
Install the Yahboom hardware library:
```bash
# Navigate to the hardware library directory
cd /path/to/YahboomR2L/software/py_install

# Install in development mode
pip3 install -e .
```

## Package Structure

```
src/
├── yahboomcar_bringup/     # Hardware drivers and system startup
│   ├── scripts/            # Python driver nodes
│   ├── launch/             # Launch files
│   ├── param/              # Parameter configurations
│   └── config/             # Additional configurations
│
├── yahboomcar_ctrl/        # Control interfaces (joystick/keyboard)
│   └── [TODO: To be migrated]
│
├── yahboomcar_description/ # Robot model and visualization
│   ├── urdf/               # Robot description files
│   ├── meshes/             # 3D model meshes  
│   ├── launch/             # Visualization launch files
│   └── rviz/               # RViz configuration files
│
└── yahboomcar_msgs/        # Custom message definitions
    └── msg/                # Message type definitions
```

## Building the Workspace

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

## Usage

### Robot Visualization
Display the robot model in RViz2:
```bash
# Launch robot description with RViz2
ros2 launch yahboomcar_description display_r2l.launch.py

# Launch without GUI (headless)
ros2 launch yahboomcar_description display_r2l.launch.py use_gui:=false rviz:=false
```

### Hardware Driver Testing
Test the hardware driver (requires robot hardware):
```bash
# Basic driver test
ros2 launch yahboomcar_bringup driver_test.launch.py

# Full system bringup with EKF and sensor fusion
ros2 launch yahboomcar_bringup bringup.launch.py
```

### Message Interfaces
View custom message types:
```bash
# List all yahboom messages
ros2 interface list | grep yahboomcar

# Show message structure
ros2 interface show yahboomcar_msgs/msg/Target
ros2 interface show yahboomcar_msgs/msg/TargetArray
```

## Custom Messages

The workspace includes custom message definitions:
- `yahboomcar_msgs/msg/ImageMsg` - Custom image data structure
- `yahboomcar_msgs/msg/Position` - Target position data
- `yahboomcar_msgs/msg/Target` - Detection target information  
- `yahboomcar_msgs/msg/PointArray` - Collection of 3D points
- `yahboomcar_msgs/msg/TargetArray` - Array of detected targets

## Hardware Interface

The system uses the `Rosmaster_Lib` hardware abstraction layer to communicate with the robot via serial interface:
- **Serial Port**: `/dev/myserial` (configurable)
- **Baud Rate**: 115200
- **Protocol**: Custom binary protocol with checksums
- **Features**: Motion control, sensor reading, LED/buzzer control

### Supported Hardware
- Yahboom X1, X3, X3Plus, R2, R2L robots
- IMU sensors (MPU9250, ICM20948)
- Encoder feedback
- RGB LED strips
- Buzzer/beeper
- Servo control

## Migration Status

### ✅ Completed
- [x] ROS2 workspace setup
- [x] Package structure creation
- [x] Custom message definitions
- [x] Robot description (URDF/meshes)
- [x] Hardware driver (basic functionality)
- [x] Launch file infrastructure

### 🚧 In Progress
- [ ] Control interfaces (joystick/keyboard)
- [ ] Complete launch file ecosystem
- [ ] System integration testing

### 📋 Planned
- [ ] Navigation integration
- [ ] Sensor fusion optimization
- [ ] Documentation completion

## Troubleshooting

### Common Issues

**ModuleNotFoundError: No module named 'serial'**
```bash
pip3 install pyserial
```

**ModuleNotFoundError: No module name 'Rosmaster_Lib'**
```bash
cd /path/to/YahboomR2L/software/py_install
pip3 install -e .
```

**Serial port permission denied**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

**No hardware connected (testing)**
- The driver will fail to connect to `/dev/myserial` when no robot is connected
- This is expected behavior and indicates the software is working correctly

## Robot Control (Teleoperation)

### Joystick Control
Launch the joystick teleop system (requires physical joystick):
```bash
# Launch both joy_node and yahboom_joy teleop
ros2 launch yahboomcar_ctrl yahboom_joy.launch.py

# Or run individually
ros2 run joy joy_node
ros2 run yahboomcar_ctrl yahboom_joy
```

### Keyboard Control
**Important:** Keyboard teleop must be run directly from terminal (cannot use launch files):
```bash
# Run keyboard teleop directly
ros2 run yahboomcar_ctrl yahboom_keyboard
```

**Note:** Due to ROS2 architectural differences, keyboard teleop requires direct terminal stdin access and cannot be launched through launch files. This is a known limitation in ROS2 for interactive terminal-based nodes.

## Contributing

This project follows the ROS2 coding standards and conventions. Key migration principles:
- **Minimal changes** to proven hardware interface logic
- **Preserve existing functionality** wherever possible  
- **Follow ROS2 best practices** for new implementations
- **Maintain compatibility** with existing Yahboom hardware

## License

MIT License - see original Yahboom documentation for hardware-specific licensing.

## Related Documentation

- [Rosmaster_Lib Hardware Interface Documentation](software/py_install/Rosmaster_Lib/README.md)
- [ROS1 Original Implementation](../rosmaster_r2l/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)