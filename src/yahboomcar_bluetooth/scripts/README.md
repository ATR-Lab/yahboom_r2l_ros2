# Bluetooth Detection Test Scripts

This directory contains scripts to test and debug the YahboomRacer Bluetooth LE server functionality.

## 🔵 test_bluetooth_detection.py

A comprehensive standalone script that tests whether the YahboomRacer BLE server is properly advertising and accessible from client devices.

### 🎯 Purpose

This script helps diagnose Bluetooth connection issues by:
- Scanning for BLE devices in range
- Checking if YahboomRacer devices are detectable
- Testing actual service discovery and communication
- Providing detailed diagnostic information

### 📋 Prerequisites

- Python 3.7+
- `bleak` Python library for BLE operations
- Bluetooth adapter with BLE support
- The `bluetooth_bridge_node` should be running

### 🚀 Installation

1. **Install the required Python library:**
   ```bash
   pip3 install bleak
   ```

2. **Verify Bluetooth is available:**
   ```bash
   # Check if Bluetooth service is running
   systemctl is-active bluetooth
   
   # Check Bluetooth adapter status  
   hciconfig
   ```

### 🔧 Usage

1. **Start the YahboomRacer Bluetooth bridge node** in one terminal:
   ```bash
   # Navigate to your ROS2 workspace
   cd /path/to/yahboom_r2l_ros2
   source install/setup.bash
   
   # Launch the bluetooth bridge (replace car_id as needed)
   ros2 run yahboomcar_bluetooth bluetooth_bridge_node --ros-args -p car_id:=1
   ```

2. **Run the detection test** in another terminal:
   ```bash
   # Navigate to the scripts directory
   cd /home/atr-lab/development/yahboom_r2l_ros2/src/yahboomcar_bluetooth/scripts
   
   # Run the test script
   python3 test_bluetooth_detection.py
   ```

### 📊 Expected Output

The script will run through 3 phases:

#### Phase 1: BLE Device Scanning
```
📡 Phase 1: BLE Device Scanning
🔍 Scanning for BLE devices (15 seconds)...
✅ Found 5 BLE devices total
🎯 FOUND YahboomRacer device: YahboomRacer_Car1 (14:4F:8A:CA:F2:37)
```

#### Phase 2: Service Discovery  
```
🔍 Phase 2: Service Discovery
🔗 Connecting to YahboomRacer_Car1 (14:4F:8A:CA:F2:37)
✅ Connected to YahboomRacer_Car1
📋 Found 3 services:
  🔧 Service: 12345678-1234-1234-1234-123456789abc
      🎯 THIS IS THE RACING SERVICE!
```

#### Phase 3: Communication Testing
```
🔗 Phase 3: Communication Testing
📡 Testing communication with YahboomRacer_Car1
  📊 Testing sensor data read...
  ✅ Sensor read successful: {'car_id': 1, 'battery_voltage': 12.4}
  📝 Testing command write...
  ✅ Command write successful
```

### 📋 Results Summary

At the end, you'll get a comprehensive summary:

```
📋 Test Results Summary
📡 BLE Scan Results:
   Total devices found: 5
   YahboomRacer devices: 1
   ✅ YahboomRacer devices detected successfully!

🔍 Service Discovery:
   Device 14:4F:8A:CA:F2:37: Connected ✅ Racing Service

🔗 Communication Tests:
   Device 14:4F:8A:CA:F2:37: Sensor ✅ Command ✅

🎯 Overall Status:
   ✅ BLE server is advertising and detectable!
   💡 Your Bluetooth server appears to be working correctly.
```

### ❌ Troubleshooting

#### No YahboomRacer devices found
```
📡 BLE Scan Results:
   Total devices found: 3
   YahboomRacer devices: 0
   ❌ NO YAHBOOM DEVICES DETECTED
```

**Possible causes:**
1. **Bluetooth bridge node not running** - Start it first
2. **Permission issues** - Try running with `sudo`
3. **BLE not properly initialized** - Check node logs for errors
4. **Wrong device name** - Check if advertising with different name

#### Connection failures
```
❌ Failed to connect to YahboomRacer_Car1: [Errno 16] Device or resource busy
```

**Possible causes:**
1. **Device already connected** - Disconnect other clients first  
2. **Bluetooth stack issues** - Restart Bluetooth service
3. **Permission issues** - Ensure user is in `bluetooth` group

#### Service discovery failures
```
🔍 Service Discovery:
   Device 14:4F:8A:CA:F2:37: Connected ❌ Racing Service
```

**Possible causes:**
1. **Service not properly registered** - Check node logs
2. **Wrong service UUID** - Verify UUID matches in code
3. **D-Bus permission issues** - May need elevated permissions

### 📁 Output Files

The script saves detailed results to:
- **`ble_test_results.json`** - Complete test results in JSON format

### 🔧 Advanced Usage

#### Test with specific timeout
```python
# Edit the script to change scan duration
devices = await BleakScanner.discover(timeout=30.0, return_adv=True)  # 30 seconds
```

#### Test from different location
```bash
# Copy script to test from another machine
scp test_bluetooth_detection.py user@othermachine:/tmp/
ssh user@othermachine "cd /tmp && python3 test_bluetooth_detection.py"
```

### 📚 Understanding the Results

- **✅ Green checkmarks** = Working correctly
- **❌ Red X marks** = Issues found  
- **⚠️ Yellow warnings** = Partial functionality or skipped tests

The script tests the complete BLE communication stack:
1. **Advertisement visibility** (can other devices see it?)
2. **Service accessibility** (can clients connect and discover services?)
3. **Data communication** (can clients read/write data?)

### 🆘 Getting Help

If you encounter issues:

1. **Check the ROS2 node logs** for errors
2. **Verify Bluetooth permissions**: `groups $USER` should include `bluetooth`
3. **Test Bluetooth functionality**: `bluetoothctl scan on`
4. **Check system Bluetooth status**: `systemctl status bluetooth`

For additional debugging, you can also check:
- **D-Bus permissions**: `/etc/dbus-1/system.d/bluetooth.conf`
- **Bluetooth adapter capabilities**: `bluetoothctl show`
- **HCI interface status**: `hciconfig -a`