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

### Single Robot Operation

#### **Hardware Mode** (Requires physical robot):
```bash
# Full system with all sensors and EKF
export ROBOT_TYPE=R2L
ros2 launch yahboomcar_bringup bringup.launch.py car_id:=1

# Basic driver only
ros2 launch yahboomcar_bringup yahboomcar.launch.py car_id:=1
```

#### **Simulation Mode** (No hardware required):
```bash
# Launch simulation driver with car ID
ros2 launch yahboomcar_bringup yahboomcar_sim.launch.py car_id:=1

# Test multiple cars in simulation
ros2 launch yahboomcar_bringup yahboomcar_sim.launch.py car_id:=2
ros2 launch yahboomcar_bringup yahboomcar_sim.launch.py car_id:=3
```

**Simulation Features:**
- 🔄 **No Hardware Required**: Runs without physical robot or serial connection
- 📊 **Realistic Sensor Data**: Publishes simulated battery (12.6V), IMU, and motion feedback
- 🎮 **Full Command Interface**: Complete `cmd_vel`, `manual_cmd_vel`, emergency stop support
- ⚡ **Identical Behavior**: Same priority system and safety features as hardware mode
- 🧪 **Perfect for Testing**: Ideal for developing multiplayer features and AR integration

### Multiplayer Racing System
For Mario Kart Live-style multiplayer racing:

#### **Individual Robot Launch** (Run on each robot's computer):

**Hardware Mode:**
```bash
# Robot Car 1
export ROBOT_TYPE=R2L
ros2 launch yahboomcar_bringup bringup.launch.py car_id:=1

# Robot Car 2  
export ROBOT_TYPE=R2L
ros2 launch yahboomcar_bringup bringup.launch.py car_id:=2

# Robot Car 3
export ROBOT_TYPE=R2L
ros2 launch yahboomcar_bringup bringup.launch.py car_id:=3

# Robot Car 4
export ROBOT_TYPE=R2L
ros2 launch yahboomcar_bringup bringup.launch.py car_id:=4
```

**Simulation Mode** (for development/testing):
```bash
# Test Car 1 (Terminal 1)
ros2 launch yahboomcar_bringup yahboomcar_sim.launch.py car_id:=1

# Test Car 2 (Terminal 2) 
ros2 launch yahboomcar_bringup yahboomcar_sim.launch.py car_id:=2

# Test Car 3 (Terminal 3)
ros2 launch yahboomcar_bringup yahboomcar_sim.launch.py car_id:=3

# Test Car 4 (Terminal 4)
ros2 launch yahboomcar_bringup yahboomcar_sim.launch.py car_id:=4
```

#### **Master Control Center** (Run on control station):
```bash
ros2 run yahboomcar_master_ui master_ui
```

#### **Car ID System**:
- **Valid Range**: `car_id` must be 1-4 (supports up to 4 simultaneous racers)
- **Namespace Generation**: `car_id:=2` creates `/car_2/` namespace for all topics
- **Unique Identity**: Each car gets isolated topics, nodes, and TF frames
- **No Conflicts**: Multiple cars can run simultaneously without interference

### Robot Visualization
Display the robot model in RViz2:
```bash
# Launch robot description with RViz2
ros2 launch yahboomcar_description display_r2l.launch.py

# Launch without GUI (headless)
ros2 launch yahboomcar_description display_r2l.launch.py use_gui:=false rviz:=false
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
- [x] **Multiplayer racing foundation** (Phase 1 complete)
- [ ] **Bluetooth-ROS bridge** for AR app integration (Phase 2)
- [ ] **Game effects system** for power-ups and collisions (Phase 3)
- [ ] **Race management system** (Phase 4)

### 📋 Planned
- [ ] **AR app communication protocol** definition
- [ ] **Web interface integration** with AR app
- [ ] **Advanced racing features** (checkpoints, leaderboards)
- [ ] **Performance optimization** for real-time racing

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

## Robot Control System

### Multiplayer Racing Control Architecture

#### **Command Priority System** (Safety-First Design):
```
Priority 1: Emergency Stop       # Instant stop all motion
Priority 2: Manual Override      # Master control joystick 
Priority 3: Normal Commands      # AR app/game commands
```

#### **Topic Structure per Robot**:
```bash
# Car 1 Topics
/car_1/cmd_vel           # Primary command interface (AR app → Bluetooth → ROS)
/car_1/manual_cmd_vel    # Manual override from master control
/car_1/emergency_stop    # Individual emergency stop
/car_1/vel_raw          # Raw velocity feedback
/car_1/voltage          # Battery voltage
/car_1/joint_states     # Joint positions
/car_1/imu/imu_raw      # Raw IMU data
/car_1/odom             # Processed odometry

# System-Wide Control
/system/emergency_stop_all  # Master emergency stop for all cars
```

#### **Manual Control Behavior**:
- **Automatic Activation**: Manual commands automatically enable override mode
- **Timeout Protection**: Override mode auto-deactivates after 2 seconds of no manual commands
- **AR Command Blocking**: While manual override is active, AR app commands are ignored
- **Seamless Handback**: When override expires, AR app regains control instantly

### Testing Priority System (Real-Time Verification)

#### **Step 1: Launch Simulation and Monitor Output**
```bash
# Terminal 1: Launch simulation
ros2 launch yahboomcar_bringup yahboomcar_sim.launch.py car_id:=1

# Terminal 2: Continuously monitor robot's actual velocity output
ros2 topic echo /car_1/vel_raw
```

#### **Step 2: Test Normal Commands (Priority 3)**
```bash
# Terminal 3: Send normal command (should change vel_raw in Terminal 2)
ros2 topic pub /car_1/cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}" --once
```
**Expected**: `/car_1/vel_raw` shows non-zero values (speed limited to ~0.21 for safety)

#### **Step 3: Test Manual Override (Priority 2)**
```bash
# Terminal 3: Send manual override (should override normal commands)
ros2 topic pub /car_1/manual_cmd_vel geometry_msgs/Twist "{linear: {x: -0.2}, angular: {z: -1.0}}" --once

# Try normal command during override (should be ignored)
ros2 topic pub /car_1/cmd_vel geometry_msgs/Twist "{linear: {x: 0.8}, angular: {z: 2.0}}" --once
```
**Expected**: `/car_1/vel_raw` shows manual command values, ignores subsequent `cmd_vel`

#### **Step 4: Test Emergency Stop (Priority 1)**
```bash
# Terminal 3: Emergency stop (should override everything)
ros2 topic pub /car_1/emergency_stop std_msgs/Bool "{data: true}" --once

# Try any command during emergency (should be blocked)
ros2 topic pub /car_1/manual_cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}" --once
```
**Expected**: `/car_1/vel_raw` immediately shows all zeros, ignores all motion commands

#### **Step 5: Resume Normal Operation**
```bash
# Terminal 3: Release emergency stop
ros2 topic pub /car_1/emergency_stop std_msgs/Bool "{data: false}" --once

# Test normal operation resumed
ros2 topic pub /car_1/cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once  
```
**Expected**: `/car_1/vel_raw` responds to commands again

### Traditional Single Robot Control

#### **Joystick Control**
Launch the joystick teleop system (requires physical joystick):
```bash
# For single robot (traditional mode)
ros2 launch yahboomcar_ctrl yahboom_joy.launch.py

# Or run individually
ros2 run joy joy_node
ros2 run yahboomcar_ctrl yahboom_joy
```

#### **Keyboard Control**
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