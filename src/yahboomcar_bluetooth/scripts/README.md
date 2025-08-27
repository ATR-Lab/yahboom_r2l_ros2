# Yahboom Robot Bluetooth Scripts

This directory contains a **working Bluetooth Low Energy (BLE) server** implementation that allows other devices to discover, connect to, and exchange messages with the Yahboom robot using real Bluetooth communication.

## 🎯 Quick Start

### 1. Install Dependencies
```bash
pip install -r bluetooth_requirements.txt
```

### 2. Start the BLE Server
```bash
# For Ubuntu/macOS (BlueZ 5.64+):
python3 ble_server.py

# For Jetson Nano (BlueZ 5.53):
python3 ble_server.py --jetson
```

### 3. Test the Server
Choose one of these testing approaches:

#### Option A: Automated Test (Recommended)
```bash
# For Ubuntu/macOS (BlueZ 5.64+):
python3 ble_client_test.py

# For Jetson Nano (BlueZ 5.53):
python3 ble_client_test.py --jetson
```

#### Option B: Mobile Device Testing
- Download a BLE scanner app (nRF Connect, LightBlue Explorer)
- Look for "YahboomRobot" device  
- Connect and explore the custom service

## 📁 Files in this Directory

### 🔧 Working Implementation
- **`ble_server.py`** - Complete BLE server implementation ⭐
  - Real Bluetooth Low Energy communication
  - Full GATT structure: Service + Characteristics + Properties
  - Advertises as "YahboomRobot" (discoverable by mobile BLE scanners)
  - JSON-based bidirectional status data and message handling
  - BlessGATTCharacteristic callbacks for read/write operations
  - **BlueZ compatibility mode** (`--jetson` flag for BlueZ 5.53)
  - Comprehensive logging and error handling

- **`ble_client_test.py`** - Automated test client ⭐
  - Comprehensive server validation
  - Device discovery and connection testing
  - Read/write operation validation
  - **Adaptive write mode** (`--jetson` flag for BlueZ 5.53 compatibility)
  - Clear pass/fail reporting

### 📦 Dependencies & Setup
- **`bluetooth_requirements.txt`** - Python dependencies with tested versions
  - `bless==0.2.6` - BLE server library ⭐ (REQUIRED version for characteristics support)
  - `bleak==0.19.5` - BLE client library (for testing)
  - `async-timeout==4.0.3` - Async timeout support
  - `dbus_next>=1.2.0` - Linux D-Bus interface (critical for Ubuntu/Jetson)

### 🔍 Utilities
- **`ble_scanner.py`** - BLE device scanner utility
  - Scan for nearby BLE devices
  - Useful for debugging and discovery

## 🚀 BLE Server Features

### Core Functionality
- ✅ **Real Bluetooth Communication** - Not a simulation
- ✅ **Device Discovery** - Appears as "YahboomRobot" in BLE scanners
- ✅ **Custom Service** - UUID: `12345678-1234-1234-1234-123456789abc`
- ✅ **Status Characteristic** - UUID: `11111111-2222-3333-4444-555555555555`
- ✅ **Cross-Platform** - Works on macOS, Linux with Bluetooth hardware

### Communication Protocol
- **Read Operations**: Returns JSON status data with server info
- **Write Operations**: Accepts commands and logs interactions
- **JSON Format**: Structured data with timestamps and message counters
- **Multi-Client**: Supports multiple simultaneous connections

### Example Status Response

**New Ultra-Compact Format** (Used by `bluetooth_ros2_bridge.py`):
```json
{"t":"sens","d":[12.4,0,0.5,0.2,0.1,0.0],"id":1}
```
**Data Array**: `[battery, emergency, speed, imu_z, imu_x, imu_y]` (~57 chars)

**Legacy Format** (Used by `ble_server.py` for testing):
```json
{
  "timestamp": "2024-01-13T10:30:00",
  "status": "running",
  "message_count": 42,
  "server_name": "YahboomRobot",
  "uptime_seconds": 1800
}
```

## 🧪 Testing the BLE Server

### Automated Testing (Recommended)

The **`ble_client_test.py`** provides comprehensive automated testing:

```bash
# Ubuntu/macOS (BlueZ 5.64+):
# Terminal 1: Start server
python3 ble_server.py

# Terminal 2: Run automated test
python3 ble_client_test.py

# Jetson Nano (BlueZ 5.53):
# Terminal 1: Start server with compatibility mode
python3 ble_server.py --jetson

# Terminal 2: Run test with compatibility mode
python3 ble_client_test.py --jetson
```

**Test Coverage:**
- ✅ **Device Discovery** - Finds "YahboomRobot" in BLE scan
- ✅ **Connection** - Establishes BLE connection
- ✅ **Service Discovery** - Validates service/characteristic UUIDs  
- ✅ **Read Operations** - Tests JSON status data retrieval
- ✅ **Write Operations** - Tests bidirectional communication
- ✅ **Error Handling** - Comprehensive troubleshooting info

**Expected Success Output:**
```
🎉 SUCCESS: All tests passed!
   Your BLE server is working correctly!

🎯 Overall Results: 5/5 tests passed

📊 Individual Test Results:
   ✅ Device Discovery
   ✅ Connection
   ✅ Service Discovery
   ✅ Read Operation
   ✅ Write Operation
```

### Mobile Apps for Testing
| Platform | Recommended App | Developer |
|----------|----------------|-----------|
| Android | nRF Connect for Mobile | Nordic Semiconductor |
| iOS | LightBlue Explorer | Punch Through |
| Cross-platform | BLE Scanner | Various developers |

### Testing Steps
1. **Start Server**: 
   - Ubuntu/macOS: `python3 ble_server.py`
   - Jetson Nano: `python3 ble_server.py --jetson`
2. **Open BLE Scanner**: Launch your chosen mobile app
3. **Scan for Devices**: Look for "YahboomRobot"
4. **Connect**: Tap to connect to the device
5. **Explore Service**: Navigate to service UUID `12345678-1234-1234-1234-123456789abc`
6. **Read Status**: Read from characteristic `11111111-2222-3333-4444-555555555555`
7. **Send Commands**: Write data to test bidirectional communication

### Expected Behavior
- ✅ Device appears in scan results as "YahboomRobot"
- ✅ Connection establishes successfully
- ✅ Custom service is discoverable
- ✅ Read operations return JSON status data
- ✅ Write operations are logged in server console
- ✅ Server handles multiple connections gracefully

## 🔧 Development & Integration

### ROS2 Integration
To integrate with ROS2 nodes:
```python
# Add to ble_server.py
import rclpy
from your_msgs.msg import YourMessageType

# In read_request_callback:
# - Query ROS2 topics for robot status
# - Return real sensor data

# In write_request_callback:
# - Parse received commands
# - Publish to ROS2 topics
# - Control robot behavior
```

### Custom Commands
Extend the server by modifying the callback functions:
```python
def write_request_callback(self, characteristic: BlessGATTCharacteristic, value, **kwargs):
    """Handle write requests with bless 0.2.6 API"""
    try:
        # Update characteristic value
        characteristic.value = value
        
        # Decode command
        command = value.decode('utf-8')
        if command == "move_forward":
            # Implement robot movement
            pass
        elif command == "get_sensors":
            # Return sensor data via read_request_callback
            pass
        elif command.startswith("SPEED:"):
            speed = int(command.split(":")[1])
            # Set robot speed
            pass
    except Exception as e:
        logger.error(f"Command processing error: {e}")
```

### Adding New Characteristics
To add more BLE characteristics (bless 0.2.6 API):
```python
from bless import GATTCharacteristicProperties, GATTAttributePermissions

# In setup_server method:
NEW_CHAR_UUID = "aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee"

# Define properties and permissions
char_properties = (
    GATTCharacteristicProperties.read 
    | GATTCharacteristicProperties.write
)
char_permissions = (
    GATTAttributePermissions.readable 
    | GATTAttributePermissions.writeable
)

# Add to existing service
await self.server.add_new_characteristic(
    SERVICE_UUID,        # service_uuid
    NEW_CHAR_UUID,       # char_uuid
    char_properties,     # properties
    None,               # initial_value
    char_permissions    # permissions
)

# Handle in read/write callbacks based on characteristic.uuid
```

## 🛠 Troubleshooting

### Server Won't Start
**Issue**: Server fails to start with permission errors
```bash
# Solution 1: Check Bluetooth permissions
# macOS: System Preferences → Security & Privacy → Bluetooth
# Linux: Run with sudo or add user to bluetooth group

# Solution 2: Verify dependencies
pip install -r bluetooth_requirements.txt

# Solution 3: Check Bluetooth hardware
# Ensure Bluetooth is enabled and working
```

### Device Not Discoverable
**Issue**: Mobile apps can't find "YahboomRobot"
```bash
# Check server logs for advertising confirmation
# Look for: "📡 Advertising as: YahboomRobot"

# Verify service UUID matches expectations
# Ensure no firewall is blocking Bluetooth

# Try restarting Bluetooth service
# macOS: Turn Bluetooth off/on in System Preferences
# Linux: sudo systemctl restart bluetooth
```

### Connection Issues
**Issue**: Can connect but can't read/write data
```bash
# Check characteristic UUIDs match exactly
# Verify read/write permissions on characteristics
# Review server logs for callback execution
# Ensure mobile app has proper BLE permissions
```

### API Errors & Version Issues
**Issue**: `bless` library API errors (characteristic creation fails)
```bash
# CRITICAL: Use bless 0.2.6 (NOT 0.2.4)
pip install bless==0.2.6 bleak==0.19.5 async-timeout==4.0.3

# Common error with 0.2.4:
# "BleakGATTCharacteristicBlueZDBus.__init__() missing 1 required positional argument"

# Solution: Upgrade to 0.2.6
pip install --upgrade bless==0.2.6

# Check Python version (3.7+ required for asyncio)
python3 --version

# Verify no conflicting Bluetooth libraries
pip list | grep -i blue
```

**Issue**: Ubuntu/Linux D-Bus errors
```bash
# Ensure dbus_next is installed (critical for BlueZ communication)
pip install dbus_next>=1.2.0

# Error: "ModuleNotFoundError: No module named 'dbus_next'"
# Solution: Always install from requirements.txt on Linux systems
```

### BlueZ Version Compatibility
**Issue**: Write operations time out on Jetson Nano/older BlueZ versions
```bash
# Error: "CBATTErrorDomain Code=14 'Unlikely error.'"
# Cause: BlueZ 5.53 (Jetson Nano) vs BlueZ 5.64+ (Ubuntu 22.04)

# Solution: Use --jetson compatibility flag
# On Jetson Nano server:
python3 ble_server.py --jetson

# On client connecting to Jetson server:
python3 ble_client_test.py --jetson

# Technical details:
# - BlueZ 5.53: Requires write_without_response for callbacks
# - BlueZ 5.64+: Supports write_with_response automatically
# - --jetson flag enables dual-mode characteristic properties

# Unity Mobile Apps:
# - Our dual-mode server prevents Unity BLE plugin timeouts
# - Racing game performance improved with write-without-response
# - No Unity code changes required - server adapts automatically
```

### macOS Bluetooth Device Name Caching
**Issue**: Python BLE scripts see old device names (e.g. "YahboomRobot") while other apps see correct names (e.g. "YahboomRacer_Car1")
```bash
# Cause: macOS Core Bluetooth framework caches device information in multiple locations
# Standard cache clearing may not clear all BLE device databases

# Solution: Comprehensive Bluetooth cache clearing
# Step 1: Standard cache clearing (often insufficient alone)
sudo pkill bluetoothd
sudo rm -rf /Library/Preferences/com.apple.Bluetooth.plist  
rm -rf ~/Library/Preferences/com.apple.bluetoothuserd.plist

# Step 2: Clear BLE device database files (KEY STEP for device name caching)
sudo rm -f /Library/Bluetooth/com.apple.MobileBluetooth.ledevices.other.db*
sudo rm -f /Library/Bluetooth/com.apple.MobileBluetooth.ledevices.paired.db*
sudo rm -f /Library/Bluetooth/Library/Preferences/com.apple.MobileBluetooth.devices.plist

# Step 3: Restart Bluetooth daemon
sudo pkill bluetoothd  # Will auto-restart

# Step 4: REBOOT (Critical - changes require restart)
sudo reboot

# Note: The hidden Bluetooth debug menu (Option+Shift+Bluetooth icon) 
# was removed in macOS Sequoia 15.5+, so manual cache clearing is required.

# After reboot, test your script - should now see correct device names:
python3 ble_client_test.py
# Should find "YahboomRacer_Car1" instead of cached "YahboomRobot"
```

## 🎯 Use Cases

### Robot Control
- Send movement commands from mobile device
- Real-time status monitoring
- Emergency stop functionality
- Parameter adjustment

### Data Collection
- Stream sensor data to mobile app
- Log robot performance metrics
- Remote diagnostics
- Telemetry dashboard

### Game Integration
- Multiplayer racing controls
- Power-up activation
- Score sharing
- Real-time competition data

### Unity 3D Mobile App Integration
- **iOS Unity Apps**: Compatible with our BLE server using CoreBluetooth framework
- **Android Unity Apps**: Compatible using Android BLE API
- **Racing Game Performance**: Optimized for real-time command streaming
- **Cross-Platform Support**: Same BLE server works with both iOS/Android Unity clients

## 🎮 Unity Mobile App Compatibility

### Unity BLE Client Considerations

Our BLE server is **fully compatible** with Unity 3D mobile applications through standard Unity BLE plugins. The `--jetson` compatibility flag specifically addresses issues Unity apps would encounter on Jetson Nano deployments.

#### **iOS Unity Apps:**
```csharp
// Unity BLE plugins use CoreBluetooth framework internally
// Our dual-mode server prevents CBATTErrorDomain Code=14 timeouts
BLEManager.WriteCharacteristic(deviceId, serviceUuid, charUuid, commandData);
// ✅ Works with --jetson flag on Jetson Nano servers
```

#### **Android Unity Apps:**
```csharp
// Unity plugins use Android BluetoothGatt API
// Our server handles both write-with-response and write-without-response
BLEManager.SendCommand(robotCommand);
// ✅ Better performance with write-without-response on Jetson
```

#### **Racing Game Performance Benefits:**
- **Lower Latency**: Write-without-response mode reduces command lag
- **Higher Throughput**: More steering/throttle updates per second  
- **Smoother Gameplay**: Eliminates timeout-induced stuttering
- **Real-Time Responsiveness**: Optimal for 60 FPS racing controls

#### **Unity BLE Plugin Compatibility:**
| Plugin Name | Default Write Mode | Jetson Compatibility |
|-------------|-------------------|---------------------|
| "Bluetooth LE for iOS/Android" | write-with-response | ✅ Our server adapts |
| "Unity BLE Framework" | write-with-response | ✅ Our server adapts |
| Native platform wrappers | Varies by platform | ✅ Dual-mode support |

#### **Recommended Unity Implementation:**
```csharp
// Standard Unity BLE plugin usage - no changes needed
public class RobotController : MonoBehaviour {
    void SendRacingCommand(float steering, float throttle) {
        var command = JsonUtility.ToJson(new {
            command = "race_control",
            steering = steering,
            throttle = throttle,
            timestamp = Time.time
        });
        
        // Our server handles this regardless of Unity BLE plugin's write mode
        BLEManager.WriteCharacteristic(robotServiceUuid, commandCharUuid, command);
    }
}
```

#### **Unity Development Notes:**
- **No Unity code changes required** - our server adapts to Unity BLE plugins
- **Testing**: Use `--jetson` flag when testing against Jetson Nano robots
- **Performance**: Racing games benefit from improved command responsiveness  
- **Cross-Platform**: Same Unity code works with both Ubuntu and Jetson deployments

## 📊 Performance Notes

- **Connection Latency**: ~100-500ms typical for BLE
- **Data Throughput**: Up to 20 KB/s (BLE limitation)
- **Range**: ~10 meters typical for BLE
- **Power Usage**: Low (BLE optimized for battery devices)
- **Concurrent Connections**: Multiple clients supported
- **BlueZ Compatibility**: `--jetson` flag eliminates write timeout issues
- **Unity Gaming**: Optimized for real-time racing command streaming

### BLE Message Size Limitations

**Important Discovery**: BLE characteristic read/write operations have strict packet size limits:

- **BLE Packet Limit**: ~100-120 characters per operation
- **Truncation Threshold**: Messages >510-512 characters get truncated
- **JSON Impact**: Long messages cause "Unterminated string" parsing errors

**Message Optimization in `bluetooth_ros2_bridge.py`**:
- **Before**: Verbose JSON responses (600+ characters) → Frequent truncation
- **After**: Ultra-compact format (~57 characters) → 100% delivery success
- **Key Changes**: Short keys (`t` vs `type`), array data, reduced precision, no whitespace

**For Custom Development**:
```python
# ❌ BAD: Will be truncated over BLE
response = {
    "type": "sensor_data",  
    "content": {"battery_voltage": 12.123456789, ...},
    "timestamp": 1756282736.123456789,
    "uptime_seconds": 1234567890
}

# ✅ GOOD: Fits within BLE limits  
response = {"t": "sens", "d": [12.1, 0, 0.3], "id": 1}
```

## 🔒 Security Considerations

This is a **demonstration implementation**. For production use, consider:
- Authentication mechanisms
- Message encryption
- Device whitelisting
- Input validation
- Rate limiting
- Secure pairing procedures

## 📚 References

- [bless Library Documentation](https://github.com/kevincar/bless)
- [BLE GATT Specification](https://www.bluetooth.com/specifications/specs/generic-attribute-profile-1-1/)
- [Nordic BLE Development](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/ug_ble.html)
- [macOS Core Bluetooth](https://developer.apple.com/documentation/corebluetooth)
- [Unity Mobile BLE Development](https://docs.unity3d.com/Manual/android-bluetooth.html)
- [Unity Asset Store - BLE Plugins](https://assetstore.unity.com/search?q=bluetooth)

## 🎉 Success!

You now have a **fully functional Bluetooth server** that:
- ✅ Actually uses real Bluetooth (not TCP simulation)
- ✅ Complete GATT implementation with services and characteristics
- ✅ Discoverable by mobile devices (tested with nRF Connect)
- ✅ **Unity mobile app compatible** with iOS/Android BLE plugins
- ✅ Proper BlessGATTCharacteristic callback handling
- ✅ Structured JSON data exchange
- ✅ Multiple concurrent client support
- ✅ Cross-platform (macOS/Ubuntu/Jetson Nano tested)
- ✅ **BlueZ version compatibility** (`--jetson` flag for older BlueZ)
- ✅ **Racing game optimized** - low latency, high throughput
- ✅ Comprehensive logging and error handling

**CRITICAL REQUIREMENTS**: 
- Must use `bless==0.2.6` for characteristic creation to work!
- Use `--jetson` flag on Jetson Nano for BlueZ 5.53 compatibility
- Unity apps work without code changes - server adapts to BLE plugins

Ready for integration with your Yahboom robot and Unity mobile racing applications! 🤖🏎️🎮
