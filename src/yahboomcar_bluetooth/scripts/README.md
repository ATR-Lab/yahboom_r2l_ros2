# Yahboom Robot Bluetooth Scripts

This directory contains a **working Bluetooth Low Energy (BLE) server** implementation that allows other devices to discover, connect to, and exchange messages with the Yahboom robot using real Bluetooth communication.

## 🎯 Quick Start

### 1. Install Dependencies
```bash
pip install -r bluetooth_requirements.txt
```

### 2. Start the BLE Server
```bash
python3 ble_server.py
```

### 3. Test the Server
Choose one of these testing approaches:

#### Option A: Automated Test (Recommended)
```bash
python3 ble_client_test.py
```

#### Option B: Mobile Device Testing
- Download a BLE scanner app (nRF Connect, LightBlue Explorer)
- Look for "YahboomRobot" device  
- Connect and explore the custom service

## 📁 Files in this Directory

### 🔧 Working Implementation
- **`ble_server.py`** - Complete BLE server implementation ⭐
  - Real Bluetooth Low Energy communication
  - Advertises as "YahboomRobot"
  - JSON-based status data and message handling
  - Comprehensive logging and error handling

- **`ble_client_test.py`** - Automated test client ⭐
  - Comprehensive server validation
  - Device discovery and connection testing
  - Read/write operation validation  
  - Clear pass/fail reporting

### 📦 Dependencies & Setup
- **`bluetooth_requirements.txt`** - Python dependencies with tested versions
  - `bless==0.2.4` - BLE server library
  - `bleak==0.19.5` - BLE client library (for testing)
  - `async-timeout==4.0.3` - Async timeout support

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
# Terminal 1: Start server
python3 ble_server.py

# Terminal 2: Run automated test
python3 ble_client_test.py
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
1. **Start Server**: `python3 ble_server.py`
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
def write_request_callback(self, characteristic, value, offset, without_response):
    try:
        command = value.decode('utf-8')
        if command == "move_forward":
            # Implement robot movement
            pass
        elif command == "get_sensors":
            # Return sensor data
            pass
    except Exception as e:
        logger.error(f"Command processing error: {e}")
```

### Adding New Characteristics
To add more BLE characteristics:
```python
# In setup_server method:
NEW_CHAR_UUID = "aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee"
await self.server.add_new_service(NEW_CHAR_UUID)

# Handle in read/write callbacks
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

### API Errors
**Issue**: `bless` library API errors
```bash
# Ensure exact versions from requirements.txt
pip install bless==0.2.4 bleak==0.19.5 async-timeout==4.0.3

# Check Python version (3.7+ required for asyncio)
python3 --version

# Verify no conflicting Bluetooth libraries
pip list | grep -i blue
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

## 📊 Performance Notes

- **Connection Latency**: ~100-500ms typical for BLE
- **Data Throughput**: Up to 20 KB/s (BLE limitation)
- **Range**: ~10 meters typical for BLE
- **Power Usage**: Low (BLE optimized for battery devices)
- **Concurrent Connections**: Multiple clients supported

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

## 🎉 Success!

You now have a **working Bluetooth server** that:
- ✅ Actually uses real Bluetooth (not TCP simulation)
- ✅ Is discoverable by mobile devices
- ✅ Handles bidirectional communication
- ✅ Provides structured JSON data
- ✅ Supports multiple clients
- ✅ Includes comprehensive logging

Ready for integration with your Yahboom robot! 🤖
