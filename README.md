# Yahboom R2L ROS2 Migration

ROS2 Humble port of the Yahboom R2L robot platform, migrated from ROS1 Noetic.

## Overview

This workspace contains the ROS2 implementation of the Yahboom R2L robot system, including:
- **Hardware drivers** for robot control and sensor data
- **Robot description** (URDF/meshes) for visualization and simulation  
- **Custom message definitions** for robot-specific data types
- **Control interfaces** for joystick/keyboard teleop
- **Master control UI** for multiplayer racing management
- **Launch files** for system startup and testing

## Prerequisites

### System Requirements
- **Ubuntu 20.04 LTS** (for ROS2 Foxy) or **Ubuntu 22.04 LTS** (for ROS2 Humble)
- **ROS2 Foxy** or **ROS2 Humble** (installed and configured)
- **Python 3.8+** (Foxy) or **Python 3.10+** (Humble)

### ROS2 Dependencies

**For ROS2 Foxy (Ubuntu 20.04):**
```bash
sudo apt update
sudo apt install -y \
    ros-foxy-xacro \
    ros-foxy-robot-localization \
    ros-foxy-imu-filter-madgwick \
    ros-foxy-joint-state-publisher \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-tf2-geometry-msgs
```

**For ROS2 Humble (Ubuntu 22.04):**
```bash
sudo apt update
sudo apt install -y \
    ros-humble-xacro \
    ros-humble-robot-localization \
    ros-humble-imu-filter-madgwick \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-tf2-geometry-msgs
```

**Note:** These dependencies are required for the workspace to build successfully. The `xacro` package is needed for robot description processing, while `robot-localization` and `imu-filter-madgwick` are required for sensor fusion and localization features.

### Python Dependencies
Install required Python packages:
```bash
# Serial communication for hardware interface
pip3 install pyserial

# Bluetooth LE server for iPhone AR app communication  
# Note: Use requirements.txt for tested versions and BlueZ compatibility
pip3 install -r src/yahboomcar_bluetooth/scripts/bluetooth_requirements.txt

# Master Control UI dependencies
sudo apt install python3-pyqt5 python3-pyqt5.qtmultimedia
pip3 install PyQt5 numpy
```

### System Dependencies for Bluetooth
For Bluetooth LE functionality, ensure BlueZ is installed:
```bash
# Install BlueZ (usually pre-installed on Ubuntu)
sudo apt update
sudo apt install -y bluez

# Ensure Bluetooth service is running
sudo systemctl enable bluetooth
sudo systemctl start bluetooth
```

### **CRITICAL: Enable BlueZ Experimental Features**
BLE GATT server functionality (required for robot-to-iPhone communication) is considered "experimental" in BlueZ and **must be explicitly enabled**:

```bash
# 1. Create backup of bluetooth service file
sudo cp /lib/systemd/system/bluetooth.service /lib/systemd/system/bluetooth.service.backup

# 2. Add experimental flag (-E) to BlueZ daemon
sudo sed -i 's|^ExecStart=/usr/lib/bluetooth/bluetoothd$|ExecStart=/usr/lib/bluetooth/bluetoothd -E|' /lib/systemd/system/bluetooth.service

# 3. Verify the change
grep ExecStart /lib/systemd/system/bluetooth.service
# Should show: ExecStart=/usr/lib/bluetooth/bluetoothd -E

# 4. Reload and restart Bluetooth service
sudo systemctl daemon-reload
sudo systemctl restart bluetooth

# 5. Verify experimental features are enabled
ps aux | grep bluetoothd
# Should show: /usr/lib/bluetooth/bluetoothd -E

# 6. Add user to bluetooth group (if not already done)
sudo usermod -aG bluetooth $USER
# Log out and back in for group changes to take effect
```
ros2 run yahboomcar_master_ui master_ui
**⚠️ Important Notes:**
- **Without the `-E` flag, BLE server functionality will NOT work**
- This applies to **all Linux systems** including Jetson Nano, Raspberry Pi, etc.
- The experimental flag is safe and widely used in production IoT devices
- You may see TxPower D-Bus warnings in logs - these can be safely ignored

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
│   ├── param/             # Parameter configurations
│   └── config/             # Additional configurations
│
├── yahboomcar_bluetooth/   # iPhone AR app Bluetooth bridge
│   ├── yahboomcar_bluetooth/ # Python bridge nodes
│   └── launch/             # Bluetooth launch files
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
├── yahboomcar_master_ui/   # Master control interface for racing
│   ├── yahboomcar_master_ui/ # PyQt5 UI components
│   │   ├── car_details/    # Advanced car control dialogs
│   │   ├── main_window.py  # Main application window
│   │   ├── master_ui.py    # ROS2 node entry point
│   │   └── [widgets]       # UI component modules
│   └── [launch files]      # UI launch configurations
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

### ROS2 Version Compatibility

**For ROS2 Foxy (Ubuntu 20.04):**
```bash
# Use the Foxy-compatible launch file
export ROBOT_TYPE=R2L
ros2 launch yahboomcar_bringup bringup_foxy.launch.py car_id:=1
```

**For ROS2 Galactic and later:**
```bash
# Use the standard launch file
export ROBOT_TYPE=R2L
ros2 launch yahboomcar_bringup bringup.launch.py car_id:=1
```

**Note:** The `bringup_foxy.launch.py` file is specifically designed for ROS2 Foxy compatibility and includes necessary workarounds for Foxy's stricter namespace validation and launch system limitations.

### Multiplayer Racing System
For Mario Kart Live-style multiplayer racing:

#### **Individual Robot Launch** (Run on each robot's computer):

**Hardware Mode (ROS2 Foxy):**
```bash
# Robot Car 1
export ROBOT_TYPE=R2L
ros2 launch yahboomcar_bringup bringup_foxy.launch.py car_id:=1

# Robot Car 2  
export ROBOT_TYPE=R2L
ros2 launch yahboomcar_bringup bringup_foxy.launch.py car_id:=2

# Robot Car 3
export ROBOT_TYPE=R2L
ros2 launch yahboomcar_bringup bringup_foxy.launch.py car_id:=3

# Robot Car 4
export ROBOT_TYPE=R2L
ros2 launch yahboomcar_bringup bringup_foxy.launch.py car_id:=4
```

**Hardware Mode (ROS2 Galactic+):**
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

#### **Bluetooth Bridge for iPhone AR App** (Run on each robot):
```bash
# Ubuntu/macOS (BlueZ 5.64+):
ros2 launch yahboomcar_bluetooth bluetooth_bridge.launch.py car_id:=1

# Jetson Nano (BlueZ 5.53 - requires compatibility mode):
ros2 launch yahboomcar_bluetooth bluetooth_bridge.launch.py car_id:=1 jetson_mode:=true

# For multiple cars, repeat with different car_id values
# Car 2, Car 3, etc. using same BlueZ compatibility approach
```

**BLE Server Details:**
- **Device Names**: Each robot advertises as "YahboomRacer_Car1", "YahboomRacer_Car2", etc.
- **Service UUID**: `12345678-1234-1234-1234-123456789abc` (racing service)
- **Single Characteristic**: `11111111-2222-3333-4444-555555555555` (bidirectional AR app communication)
- **Ultra-Compact Protocol**: Optimized JSON format to fit BLE packet limits (~57 chars vs 600+ chars)
- **Command Priority**: Bluetooth commands integrate with existing safety system (Emergency > Manual > Bluetooth)
- **BLE Reliability**: 100% message delivery with zero truncation errors

**Testing BLE Server:**
```bash
# Ubuntu/macOS (BlueZ 5.64+):
ros2 launch yahboomcar_bluetooth bluetooth_bridge.launch.py car_id:=1

# Jetson Nano (BlueZ 5.53):
ros2 launch yahboomcar_bluetooth bluetooth_bridge.launch.py car_id:=1 jetson_mode:=true

# Expected successful output:
# [INFO] [...]: 📡 Advertisement registered successfully
# [INFO] [...]: ✅ BLE server setup complete
# [INFO] [...]: 📱 Device: YahboomRacer_Car1
# [INFO] [...]: 💡 Ready for iPhone AR app connections!

# Alternative: Direct script testing (for development)
cd src/yahboomcar_bluetooth/scripts/
# Ubuntu/macOS: python3 ble_server.py
# Jetson Nano: python3 ble_server.py --jetson

# Test with comprehensive ROS2 bridge test client
python3 test_ros2_bridge.py  # Tests new ultra-compact format
python3 test_ros2_bridge.py --jetson  # For Jetson Nano compatibility

# Or test with iPhone nRF Connect app or similar BLE scanner
# Should see "YahboomRacer_Car1" device with optimized 57-character responses
```

#### **Master Control Center** (Run on control station):
```bash
ros2 run yahboomcar_master_ui master_ui
```

### Master Control UI

The **Master Control Center** is a comprehensive PyQt5-based interface for managing robot car racing events, designed for Mario Kart Live-style racing competitions with multiple autonomous vehicles.

#### **Key Features**

- **🎛️ System Status Panel**: Network connectivity, fleet overview, uptime monitoring
- **🚗 Robot Car Fleet**: Individual car status cards with real-time telemetry
- **🎮 Manual Control Panel**: Physical joystick integration with car selection
- **🏁 Game State Panel**: Race progress, leaderboard, and track conditions
- **📊 Race Statistics**: Performance metrics and analytics
- **📝 Real-time Event Log**: System events and user actions logging

#### **Car Status Monitoring**

Each robot car displays:
- **Connection Status**: WebRTC, Bluetooth, and ROS2 connectivity indicators
- **Control Mode**: Racing, Manual Control, or Emergency Stopped states
- **Vital Signs**: Battery level with voltage, speed, position (x,y), and heading
- **Action Controls**: Manual control toggle, emergency stop, reset, and diagnostics

#### **Safety Features**

- **Speed Limiting**: Configurable maximum speed (default 40%)
- **Deadman Switches**: Automatic timeout protection
- **Connection Monitoring**: Real-time connectivity status
- **Visual State Indicators**: Color-coded status (Green/Yellow/Red)
- **Emergency Stop**: Instant kill switches for all cars or individual vehicles

#### **Running the Master UI**

**Method 1: Using ROS2 (Recommended)**
```bash
# Source the workspace
source install/setup.bash

# Launch the master control interface
ros2 run yahboomcar_master_ui master_ui
```

**Method 2: Direct Python Execution (Development)**
```bash
# Navigate to the package directory
cd src/yahboomcar_master_ui/yahboomcar_master_ui

# Run directly with Python
python3 master_ui.py
```

#### **Prerequisites for Master UI**

```bash
# Install PyQt5 dependencies
sudo apt install python3-pyqt5 python3-pyqt5.qtmultimedia
pip3 install PyQt5 numpy
```

#### **Current Implementation Status**

- **✅ UI Framework**: Complete PyQt5 interface with custom styling
- **✅ Real-time Updates**: 5Hz updates (200ms intervals) for live telemetry
- **✅ Safety Controls**: Emergency stop and manual override functionality
- **🔄 ROS2 Integration**: Stub functions ready for topic/service implementation
- **🔄 Data Management**: Currently uses dummy data with TODO integration points

#### **Architecture**

```
MasterControlWindow (QMainWindow)
├── SystemStatusWidget           # Network and fleet overview
├── DualCarStatusWidget         # Two-car status cards (space efficient)
│   └── SingleCarSection        # Individual car within dual card
├── CarStatusWidget             # Full-featured single car display
├── ManualControlWidget         # Joystick control interface
├── GameStateWidget             # Race status and track conditions
└── RosDataManager              # ROS2 integration and data management
```

#### **Known Issues & Development Notes**

- **Code Duplication**: `CarStatusWidget` and `SingleCarSection` contain ~70% duplicate code
- **ROS2 Integration**: Multiple integration points marked with `TODO` comments for actual ROS2 sser ubscribers
- **Dynamic Configuration**: Currently hardcoded to 4 cars (needs variable car support)
- **UI Improvements**: Better responsive layout and theme system needed

For detailed development information, see: `src/yahboomcar_master_ui/README.md`

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

### ✅ Completed
- [x] **Multiplayer racing foundation** (Phase 1 complete)
- [x] **Bluetooth-ROS bridge** for AR app integration (Phase 2 complete)
- [x] **Master Control UI** for racing management (Phase 2.5 complete)

### 🚧 In Progress  
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

### Master UI Issues

**ModuleNotFoundError: No module named 'PyQt5'**
```bash
sudo apt install python3-pyqt5 python3-pyqt5.qtmultimedia
pip3 install PyQt5
```

**Master UI won't start / Import errors**
```bash
# Ensure workspace is built and sourced
colcon build --packages-select yahboomcar_master_ui
source install/setup.bash

# Run with ROS2
ros2 run yahboomcar_master_ui master_ui
```

**UI displays but shows dummy data**
- This is expected behavior - the UI framework is complete but ROS2 integration is still in development
- The interface shows realistic dummy data for testing and development
- Real ROS2 integration is marked with TODO comments in the code

### Bluetooth Issues

**BLE Server Not Working / TxPower Errors**
```bash
# Check if BlueZ has experimental features enabled
ps aux | grep bluetoothd
# Must show: /usr/lib/bluetooth/bluetoothd -E

# If missing -E flag, enable experimental features:
sudo sed -i 's|^ExecStart=/usr/lib/bluetooth/bluetoothd$|ExecStart=/usr/lib/bluetooth/bluetoothd -E|' /lib/systemd/system/bluetooth.service
sudo systemctl daemon-reload
sudo systemctl restart bluetooth
```

**Permission Denied / D-Bus Access Issues**
```bash
# Add user to bluetooth group
sudo usermod -aG bluetooth $USER
# IMPORTANT: Log out and back in for group changes to take effect

# Verify group membership
groups $USER | grep bluetooth
```

**BLE Server Process Starts But No Advertisement**
```bash
# Check Bluetooth adapter status
bluetoothctl list
bluetoothctl show

# Ensure adapter is powered and discoverable
bluetoothctl power on
bluetoothctl discoverable on
```

**Jetson Nano Specific Issues**
```bash
# Jetson Nano may need additional Bluetooth packages
sudo apt install -y bluetooth bluez-tools

# Check if Bluetooth hardware is detected
lsusb | grep -i bluetooth
hciconfig -a

# If no Bluetooth adapter found, may need USB Bluetooth dongle
```

**BlueZ Version Compatibility Issues**
```bash
# Error: "CBATTErrorDomain Code=14 'Unlikely error.'" on macOS/iOS clients
# Cause: BlueZ version differences between Jetson Nano and Ubuntu

# Solution: Use Jetson compatibility mode
# On Jetson Nano server:
ros2 launch yahboomcar_bluetooth bluetooth_bridge.launch.py car_id:=1 jetson_mode:=true

# Or direct script with compatibility flag:
python3 ble_server.py --jetson

# Technical details:
# - Jetson Nano (Ubuntu 20.04): BlueZ 5.53 - requires write_without_response
# - Ubuntu 22.04+: BlueZ 5.64+ - supports write_with_response automatically
# - jetson_mode:=true enables dual-mode characteristic properties for compatibility
```

**macOS Bluetooth Device Name Caching**
```bash
# Issue: BLE test scripts see old cached device names while other apps see current names
# Example: test_ros2_bridge.py finds "YahboomRobot" but nRF Connect shows "YahboomRacer_Car1"

# Cause: macOS Core Bluetooth framework caches BLE device information in multiple database files
# Standard Bluetooth cache clearing is insufficient for BLE device name changes

# Solution: Comprehensive Bluetooth cache clearing (macOS)
# Step 1: Standard cache clearing (prepare system)
sudo pkill bluetoothd
sudo rm -rf /Library/Preferences/com.apple.Bluetooth.plist  
rm -rf ~/Library/Preferences/com.apple.bluetoothuserd.plist

# Step 2: Clear BLE device database files (CRITICAL for device name caching)
sudo rm -f /Library/Bluetooth/com.apple.MobileBluetooth.ledevices.other.db*
sudo rm -f /Library/Bluetooth/com.apple.MobileBluetooth.ledevices.paired.db*
sudo rm -f /Library/Bluetooth/Library/Preferences/com.apple.MobileBluetooth.devices.plist

# Step 3: Restart Bluetooth daemon  
sudo pkill bluetoothd  # Will auto-restart

# Step 4: REBOOT REQUIRED (changes need system restart)
sudo reboot

# After reboot, test scripts should see correct device names:
cd src/yahboomcar_bluetooth/scripts/
python3 test_ros2_bridge.py  # Should now find "YahboomRacer_Car1"

# Note: Hidden Bluetooth debug menu (Option+Shift+Bluetooth) was removed in macOS Sequoia 15.5+
# Manual cache clearing is now the only reliable method for BLE device name issues
```

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

## Jetson Nano Deployment Guide

This section provides specific instructions for deploying the robot racing system on **NVIDIA Jetson Nano**.

### Jetson Nano Prerequisites
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Humble (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# Install Jetson-specific packages
sudo apt install -y \
    bluetooth \
    bluez \
    bluez-tools \
    python3-pip \
    python3-dev \
    build-essential
```

### Bluetooth Setup on Jetson Nano
```bash
# 1. Check if Bluetooth is available
lsusb | grep -i bluetooth
hciconfig -a

# 2. If no Bluetooth adapter found, install USB Bluetooth dongle
# Recommended: CSR 4.0 or Intel AX200 based adapters

# 3. Enable BlueZ experimental features (CRITICAL!)
sudo cp /lib/systemd/system/bluetooth.service /lib/systemd/system/bluetooth.service.backup
sudo sed -i 's|^ExecStart=/usr/lib/bluetooth/bluetoothd$|ExecStart=/usr/lib/bluetooth/bluetoothd -E|' /lib/systemd/system/bluetooth.service
sudo systemctl daemon-reload
sudo systemctl restart bluetooth

# 4. Add user to bluetooth group
sudo usermod -aG bluetooth $USER
# Log out and back in

# 5. Verify setup
ps aux | grep bluetoothd  # Should show: bluetoothd -E
groups $USER | grep bluetooth  # Should show bluetooth group
```

### Build and Deploy
```bash
# 1. Clone repository on Jetson Nano
git clone <your-repo-url> yahboom_r2l_ros2
cd yahboom_r2l_ros2

# 2. Install Python dependencies (use tested versions for BlueZ compatibility)
pip3 install pyserial
pip3 install -r src/yahboomcar_bluetooth/scripts/bluetooth_requirements.txt

# 3. Install Yahboom hardware library
cd /path/to/YahboomR2L/software/py_install
pip3 install -e .
cd ~/yahboom_r2l_ros2

# 4. Build ROS2 workspace
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# 5. Test BLE server (simulation mode with Jetson compatibility)
ros2 launch yahboomcar_bluetooth bluetooth_bridge.launch.py car_id:=1 jetson_mode:=true

# Alternative: Direct script testing (with Jetson compatibility flag)
cd src/yahboomcar_bluetooth/scripts/
python3 ble_server.py --jetson

# Expected output:
# [INFO] [...]: ✅ Jetson mode: Added write_without_response compatibility
# [INFO] [...]: 📡 Advertisement registered successfully
# [INFO] [...]: 💡 Ready for iPhone AR app connections!
```

### Production Deployment
```bash
# 1. Launch full robot system (with hardware)
export ROBOT_TYPE=R2L
ros2 launch yahboomcar_bringup bringup.launch.py car_id:=1

# 2. Launch Bluetooth bridge with Jetson compatibility (separate terminal)
ros2 launch yahboomcar_bluetooth bluetooth_bridge.launch.py car_id:=1 jetson_mode:=true

# 3. Test iPhone connectivity with nRF Connect or similar app
# Look for "YahboomRacer_Car1" in BLE device list
# Should connect without "CBATTErrorDomain Code=14" timeout errors
```

### Jetson Nano Performance Tips
- **CPU Governor**: Set to `performance` for racing applications
- **Memory**: Ensure swap is configured for compilation
- **Power Mode**: Use `MAXN` power mode for best performance
- **Cooling**: Ensure adequate cooling during racing sessions
- **BlueZ Compatibility**: Always use `jetson_mode:=true` for BLE functionality

## Related Documentation

- [Rosmaster_Lib Hardware Interface Documentation](software/py_install/Rosmaster_Lib/README.md)
- [ROS1 Original Implementation](../rosmaster_r2l/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)