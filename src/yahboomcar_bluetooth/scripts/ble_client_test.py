#!/usr/bin/env python3

"""
BLE Client Test for Yahboom Robot BLE Server
============================================

A comprehensive test client that validates the Yahboom BLE server functionality.
This script discovers, connects to, and tests communication with the BLE server.

Requirements:
    pip install -r bluetooth_requirements.txt

Usage:
    python3 ble_client_test.py

Features:
    - Automatic device discovery
    - Service and characteristic validation
    - Read/write operation testing
    - JSON data parsing and validation
    - Comprehensive error handling
    - Clear pass/fail reporting

Author: Yahboom Robot Bluetooth Team
"""

import asyncio
import json
import logging
import time
from datetime import datetime
from typing import Optional, Dict, Any

try:
    from bleak import BleakScanner, BleakClient
    from bleak.backends.device import BLEDevice
    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False
    print("❌ bleak library not available. Install with: pip install -r bluetooth_requirements.txt")

# Server Configuration (must match ble_server.py)
TARGET_DEVICE_NAME = "YahboomRobot"
SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
STATUS_CHAR_UUID = "11111111-2222-3333-4444-555555555555"

# Configure logging
logging.basicConfig(level=logging.WARNING)  # Reduce noise, focus on our output
logger = logging.getLogger(__name__)


class YahboomBLEClientTest:
    """Comprehensive BLE server test client."""
    
    def __init__(self):
        self.device: Optional[BLEDevice] = None
        self.client: Optional[BleakClient] = None
        self.test_results = {
            "device_discovery": False,
            "connection": False,
            "service_discovery": False,
            "read_test": False,
            "write_test": False,
            "json_parsing": False,
            "status_data": None,
            "errors": []
        }
        
    async def run_full_test(self) -> bool:
        """Run comprehensive BLE server test suite."""
        print("🔵 Yahboom BLE Server Test Client")
        print("=" * 50)
        print()
        
        if not BLEAK_AVAILABLE:
            print("❌ Cannot run test - bleak library required")
            print("Install with: pip install -r bluetooth_requirements.txt")
            return False
        
        success = True
        
        try:
            # Phase 1: Device Discovery
            success &= await self._test_device_discovery()
            if not success:
                return False
                
            # Phase 2: Connection
            success &= await self._test_connection()
            if not success:
                return False
                
            # Phase 3: Service Discovery
            success &= await self._test_service_discovery()
            if not success:
                return False
                
            # Phase 4: Communication Testing
            success &= await self._test_read_operation()
            success &= await self._test_write_operation()
            
        except Exception as e:
            print(f"❌ Test suite failed with unexpected error: {e}")
            self.test_results["errors"].append(f"Unexpected error: {e}")
            success = False
            
        finally:
            # Always cleanup
            await self._cleanup()
            
        # Phase 5: Results Summary
        self._print_test_summary()
        
        return success
        
    async def _test_device_discovery(self) -> bool:
        """Phase 1: Discover YahboomRobot BLE device."""
        print("🔍 Phase 1: Device Discovery")
        print("-" * 30)
        
        try:
            print(f"Scanning for '{TARGET_DEVICE_NAME}' BLE device...")
            
            # Scan for devices with timeout
            devices = await BleakScanner.discover(timeout=10.0, return_adv=False)
            
            print(f"Found {len(devices)} BLE devices")
            
            # Look for our target device
            target_device = None
            for device in devices:
                if device.name == TARGET_DEVICE_NAME:
                    target_device = device
                    break
                    
            if target_device:
                self.device = target_device
                print(f"✅ Found {TARGET_DEVICE_NAME} at {target_device.address}")
                print(f"   RSSI: {target_device.rssi} dBm")
                self.test_results["device_discovery"] = True
                return True
            else:
                print(f"❌ {TARGET_DEVICE_NAME} device not found")
                print("\n🔧 Troubleshooting:")
                print(f"   • Ensure 'python3 ble_server.py' is running")
                print(f"   • Check that server is advertising as '{TARGET_DEVICE_NAME}'")
                print(f"   • Verify Bluetooth is enabled on both devices")
                print(f"   • Try running server with sudo on Linux")
                
                # Show other devices for debugging
                if devices:
                    print(f"\n📱 Other BLE devices found:")
                    for device in devices[:5]:  # Show first 5
                        name = device.name or "Unknown"
                        print(f"   • {name} ({device.address})")
                        
                self.test_results["errors"].append("Target device not found")
                return False
                
        except Exception as e:
            print(f"❌ Device discovery failed: {e}")
            self.test_results["errors"].append(f"Discovery error: {e}")
            return False
            
    async def _test_connection(self) -> bool:
        """Phase 2: Test connection to device."""
        print(f"\n🔌 Phase 2: Connection Test")
        print("-" * 30)
        
        try:
            print(f"Connecting to {self.device.name}...")
            
            self.client = BleakClient(self.device)
            await self.client.connect()
            
            if self.client.is_connected:
                print("✅ Connected successfully!")
                self.test_results["connection"] = True
                return True
            else:
                print("❌ Connection failed")
                self.test_results["errors"].append("Connection failed")
                return False
                
        except Exception as e:
            print(f"❌ Connection error: {e}")
            self.test_results["errors"].append(f"Connection error: {e}")
            return False
            
    async def _test_service_discovery(self) -> bool:
        """Phase 3: Discover and validate services."""
        print(f"\n🔍 Phase 3: Service Discovery")
        print("-" * 30)
        
        try:
            print("Discovering services and characteristics...")
            
            services = self.client.services
            print(f"Found {len(services)} services")
            
            # Look for our service
            target_service = None
            for service in services:
                if service.uuid.lower() == SERVICE_UUID.lower():
                    target_service = service
                    break
                    
            if not target_service:
                print(f"❌ Target service not found: {SERVICE_UUID}")
                print("\n🔧 Available services:")
                for service in services:
                    print(f"   • {service.uuid}")
                self.test_results["errors"].append("Target service not found")
                return False
                
            print(f"✅ Found target service: {SERVICE_UUID}")
            
            # Look for status characteristic
            status_char = None
            for char in target_service.characteristics:
                if char.uuid.lower() == STATUS_CHAR_UUID.lower():
                    status_char = char
                    break
                    
            if not status_char:
                print(f"❌ Status characteristic not found: {STATUS_CHAR_UUID}")
                print("\n🔧 Available characteristics:")
                for char in target_service.characteristics:
                    print(f"   • {char.uuid} (Properties: {char.properties})")
                self.test_results["errors"].append("Status characteristic not found")
                return False
                
            print(f"✅ Found status characteristic: {STATUS_CHAR_UUID}")
            print(f"   Properties: {status_char.properties}")
            
            self.test_results["service_discovery"] = True
            return True
            
        except Exception as e:
            print(f"❌ Service discovery error: {e}")
            self.test_results["errors"].append(f"Service discovery error: {e}")
            return False
            
    async def _test_read_operation(self) -> bool:
        """Phase 4a: Test reading from status characteristic."""
        print(f"\n📖 Phase 4a: Read Operation Test")
        print("-" * 30)
        
        try:
            print("Reading status data from server...")
            
            # Read from status characteristic
            data = await self.client.read_gatt_char(STATUS_CHAR_UUID)
            
            if not data:
                print("❌ No data received from server")
                self.test_results["errors"].append("No data received")
                return False
                
            print(f"✅ Received {len(data)} bytes of data")
            
            # Try to decode as JSON
            try:
                json_str = data.decode('utf-8')
                status_data = json.loads(json_str)
                
                print("✅ JSON parsing successful")
                print("📊 Status Data:")
                for key, value in status_data.items():
                    print(f"   {key}: {value}")
                    
                # Validate expected fields
                expected_fields = ["timestamp", "status", "message_count", "server_name", "uptime_seconds"]
                missing_fields = [field for field in expected_fields if field not in status_data]
                
                if missing_fields:
                    print(f"⚠️  Missing expected fields: {missing_fields}")
                else:
                    print("✅ All expected fields present")
                    
                # Validate server name
                if status_data.get("server_name") == TARGET_DEVICE_NAME:
                    print("✅ Server name matches expected value")
                else:
                    print(f"⚠️  Server name mismatch: expected '{TARGET_DEVICE_NAME}', got '{status_data.get('server_name')}'")
                    
                self.test_results["read_test"] = True
                self.test_results["json_parsing"] = True
                self.test_results["status_data"] = status_data
                return True
                
            except json.JSONDecodeError as e:
                print(f"❌ JSON parsing failed: {e}")
                print(f"Raw data: {data}")
                self.test_results["errors"].append(f"JSON parsing failed: {e}")
                return False
                
        except Exception as e:
            print(f"❌ Read operation failed: {e}")
            self.test_results["errors"].append(f"Read operation failed: {e}")
            return False
            
    async def _test_write_operation(self) -> bool:
        """Phase 4b: Test writing to server."""
        print(f"\n✍️  Phase 4b: Write Operation Test")
        print("-" * 30)
        
        try:
            # Create test message
            test_message = f"Hello from BLE test client at {datetime.now().strftime('%H:%M:%S')}"
            print(f"Sending test message: '{test_message}'")
            
            # Write to status characteristic (server should log this)
            message_bytes = test_message.encode('utf-8')
            await self.client.write_gatt_char(STATUS_CHAR_UUID, message_bytes, response=False)
            
            print("✅ Write operation completed")
            print("📝 Check server logs to verify message was received")
            
            # Small delay to let server process
            await asyncio.sleep(0.5)
            
            self.test_results["write_test"] = True
            return True
            
        except Exception as e:
            print(f"❌ Write operation failed: {e}")
            self.test_results["errors"].append(f"Write operation failed: {e}")
            return False
            
    async def _cleanup(self):
        """Clean up connection resources."""
        if self.client and self.client.is_connected:
            try:
                await self.client.disconnect()
                print("\n🔌 Disconnected from device")
            except:
                pass
                
    def _print_test_summary(self):
        """Print comprehensive test results."""
        print("\n" + "=" * 50)
        print("📋 Test Results Summary")
        print("=" * 50)
        
        results = self.test_results
        
        # Overall status
        total_tests = 5  # discovery, connection, service discovery, read, write
        passed_tests = sum([
            results["device_discovery"],
            results["connection"], 
            results["service_discovery"],
            results["read_test"],
            results["write_test"]
        ])
        
        print(f"\n🎯 Overall Results: {passed_tests}/{total_tests} tests passed")
        
        # Individual test results
        print(f"\n📊 Individual Test Results:")
        tests = [
            ("Device Discovery", results["device_discovery"]),
            ("Connection", results["connection"]),
            ("Service Discovery", results["service_discovery"]),
            ("Read Operation", results["read_test"]),
            ("Write Operation", results["write_test"]),
            ("JSON Parsing", results["json_parsing"])
        ]
        
        for test_name, passed in tests:
            status = "✅" if passed else "❌"
            print(f"   {status} {test_name}")
            
        # Status data summary
        if results["status_data"]:
            data = results["status_data"]
            print(f"\n📊 Server Status Summary:")
            print(f"   Server Name: {data.get('server_name')}")
            print(f"   Status: {data.get('status')}")
            print(f"   Uptime: {data.get('uptime_seconds')} seconds")
            print(f"   Message Count: {data.get('message_count')}")
            
        # Errors
        if results["errors"]:
            print(f"\n❌ Errors Encountered:")
            for error in results["errors"]:
                print(f"   • {error}")
                
        # Final verdict
        if passed_tests == total_tests:
            print(f"\n🎉 SUCCESS: All tests passed!")
            print(f"   Your BLE server is working correctly!")
        elif passed_tests >= 3:
            print(f"\n⚠️  PARTIAL SUCCESS: Basic functionality working")
            print(f"   Server is discoverable and connectable")
            print(f"   Some advanced features may need attention")
        else:
            print(f"\n❌ FAILURE: Major issues detected")
            print(f"   Check server configuration and try again")
            
        print("\n💡 Next Steps:")
        if passed_tests == total_tests:
            print("   • Try testing with mobile BLE scanner apps")
            print("   • Integrate with your ROS2 robot application")
            print("   • Add custom commands and data handling")
        else:
            print("   • Review server logs for error messages")
            print("   • Ensure server is running: python3 ble_server.py")
            print("   • Check Bluetooth permissions and hardware")


async def main():
    """Main test function."""
    print("Starting BLE Server Test...")
    print("Make sure 'python3 ble_server.py' is running in another terminal!")
    print()
    
    # Brief delay to let user read the message
    await asyncio.sleep(1)
    
    tester = YahboomBLEClientTest()
    success = await tester.run_full_test()
    
    return success


if __name__ == '__main__':
    if BLEAK_AVAILABLE:
        result = asyncio.run(main())
        exit_code = 0 if result else 1
        exit(exit_code)
    else:
        print("\n❌ Cannot run test without bleak library")
        print("Install with: pip install -r bluetooth_requirements.txt")
        exit(1)
