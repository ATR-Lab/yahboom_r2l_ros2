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
from typing import Optional, Dict, Any, List

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
        """Phase 1: Discover Yahboom BLE device using redesigned multi-stage approach."""
        print("🔍 Phase 1: Device Discovery (Redesigned Multi-Stage)")
        print("-" * 50)
        
        try:
            # Stage 1: Broad name pattern matching (handles truncation)
            print(f"🎯 Stage 1: Scanning for Yahboom devices (pattern matching)...")
            candidates = await self._discover_by_name_patterns()
            
            if not candidates:
                print(f"❌ No Yahboom-like devices found")
                await self._show_comprehensive_debug_info()
                self.test_results["errors"].append("No candidate devices found")
                return False
                
            print(f"✅ Found {len(candidates)} candidate device(s)")
            for i, candidate in enumerate(candidates):
                print(f"   {i+1}. {candidate.name or 'Unknown'} ({candidate.address}) RSSI: {candidate.rssi}dBm")
            
            # Stage 2: Post-connection service validation
            print(f"\n🎯 Stage 2: Validating service UUIDs (post-connection)...")
            validated_device = await self._validate_device_services(candidates)
            
            if validated_device:
                self.device = validated_device
                print(f"✅ Device validated successfully!")
                print(f"   Device Name: {validated_device.name or 'Not Advertised'}")
                print(f"   Actual Name: '{validated_device.name}' (expected: '{TARGET_DEVICE_NAME}')")
                print(f"   Address: {validated_device.address}")
                print(f"   RSSI: {validated_device.rssi} dBm")
                print(f"   Service UUID: {SERVICE_UUID} ✓")
                
                self.test_results["device_discovery"] = True
                return True
            else:
                print(f"❌ No candidates passed service validation")
                await self._show_comprehensive_debug_info()
                self.test_results["errors"].append("Service validation failed")
                return False
                
        except Exception as e:
            print(f"❌ Device discovery failed: {e}")
            self.test_results["errors"].append(f"Discovery error: {e}")
            return False

    async def _discover_by_name_patterns(self) -> List[BLEDevice]:
        """Stage 1: Discover devices using flexible name pattern matching."""
        try:
            # Standard device discovery without service UUID filtering
            # (Service UUID not in advertising data, only available after connection)
            devices = await BleakScanner.discover(timeout=10.0, return_adv=True)
            
            print(f"   Scanning {len(devices)} total BLE devices...")
            
            candidates = []
            
            # Multiple name matching strategies to handle truncation
            name_patterns = [
                TARGET_DEVICE_NAME,           # Exact: "YahboomRobot"
                "YahboomRob",                 # Truncated (nRF Connect showed this)
                "Yahboom",                    # Prefix match
                lambda name: "yahboom" in name.lower(),  # Case insensitive substring
                lambda name: "robot" in name.lower(),    # Generic robot pattern
                lambda name: name.lower().startswith("yah"),  # Very flexible prefix
            ]
            
            for device_info in devices.values():
                device = device_info[0] if isinstance(device_info, tuple) else device_info
                device_name = device.name
                
                if not device_name:  # Skip devices without names
                    continue
                    
                # Check against all patterns
                is_match = False
                matched_pattern = None
                
                for pattern in name_patterns:
                    if callable(pattern):
                        # Pattern is a function
                        if pattern(device_name):
                            is_match = True
                            matched_pattern = f"function({pattern.__name__ if hasattr(pattern, '__name__') else 'lambda'})"
                            break
                    else:
                        # Pattern is a string
                        if device_name == pattern:
                            is_match = True
                            matched_pattern = f"exact('{pattern}')"
                            break
                            
                if is_match:
                    candidates.append(device)
                    print(f"   ✅ Candidate: '{device_name}' matched by {matched_pattern}")
                    
            if candidates:
                print(f"   ✅ Pattern matching successful: {len(candidates)} candidate(s) found")
            else:
                print(f"   ❌ No devices matched any Yahboom name patterns")
                
            return candidates
            
        except Exception as e:
            print(f"   ❌ Pattern matching error: {e}")
            return []

    async def _validate_device_services(self, candidates: List[BLEDevice]) -> Optional[BLEDevice]:
        """Stage 2: Post-connection service validation to confirm the correct device."""
        for i, candidate in enumerate(candidates):
            print(f"   Validating candidate {i+1}/{len(candidates)}: {candidate.name} ({candidate.address})")
            
            try:
                # Attempt connection
                test_client = BleakClient(candidate)
                await test_client.connect()
                
                if not test_client.is_connected:
                    print(f"   ❌ Failed to connect to {candidate.name}")
                    continue
                    
                print(f"   ✅ Connected to {candidate.name}")
                
                # Check if our target service exists
                services = test_client.services
                target_service = None
                
                for service in services:
                    if service.uuid.lower() == SERVICE_UUID.lower():
                        target_service = service
                        break
                        
                if target_service:
                    # Check if our target characteristic exists
                    target_characteristic = None
                    for char in target_service.characteristics:
                        if char.uuid.lower() == STATUS_CHAR_UUID.lower():
                            target_characteristic = char
                            break
                            
                    if target_characteristic:
                        print(f"   ✅ Service validation passed!")
                        print(f"      Service: {SERVICE_UUID} ✓")
                        print(f"      Characteristic: {STATUS_CHAR_UUID} ✓")
                        
                        # Disconnect the test client
                        await test_client.disconnect()
                        
                        # Return this device as the validated candidate
                        return candidate
                    else:
                        print(f"   ❌ Service exists but missing target characteristic")
                else:
                    print(f"   ❌ Target service not found")
                    print(f"      Available services:")
                    for service in services:
                        print(f"         • {service.uuid}")
                        
                # Clean up connection
                await test_client.disconnect()
                
            except Exception as e:
                print(f"   ❌ Validation error for {candidate.name}: {e}")
                try:
                    if 'test_client' in locals() and test_client.is_connected:
                        await test_client.disconnect()
                except:
                    pass
                continue
                
        print(f"   ❌ No candidates passed service validation")
        return None

    async def _show_comprehensive_debug_info(self):
        """Show detailed debugging information for troubleshooting."""
        print(f"\n🔧 Comprehensive Troubleshooting Information (Redesigned Discovery)")
        print("-" * 60)
        
        try:
            # Get all devices with full advertising data for analysis
            print("📊 Analyzing all nearby BLE devices...")
            devices = await BleakScanner.discover(timeout=8.0, return_adv=True)
            
            if not devices:
                print("❌ No BLE devices found at all")
                print("\n💡 Basic Troubleshooting:")
                print("   • Check if Bluetooth is enabled on this device")
                print("   • Ensure you have Bluetooth permissions")
                print("   • Try running with sudo on Linux")
                return
                
            print(f"📱 Found {len(devices)} BLE devices total:")
            
            # Detailed device analysis with pattern matching info
            yahboom_candidates = []
            potential_matches = []
            
            # Pattern matching criteria (same as discovery method)
            name_patterns = [
                ("exact_target", lambda name: name == TARGET_DEVICE_NAME),
                ("truncated", lambda name: name == "YahboomRob"),
                ("yahboom_prefix", lambda name: name.startswith("Yahboom")),
                ("yahboom_substring", lambda name: "yahboom" in name.lower()),
                ("robot_substring", lambda name: "robot" in name.lower()),
                ("yah_prefix", lambda name: name.lower().startswith("yah")),
            ]
            
            for device_info in list(devices.values())[:15]:  # Analyze first 15 devices
                device = device_info[0] if isinstance(device_info, tuple) else device_info
                adv_data = device_info[1] if isinstance(device_info, tuple) and len(device_info) > 1 else None
                
                name = device.name or "Unknown"
                address = device.address
                
                # Check pattern matches
                matches = []
                for pattern_name, pattern_func in name_patterns:
                    if device.name and pattern_func(device.name):
                        matches.append(pattern_name)
                        if device not in yahboom_candidates:
                            yahboom_candidates.append(device)
                
                # Check for service UUIDs in advertising data
                adv_service_uuids = []
                if adv_data and hasattr(adv_data, 'service_uuids') and adv_data.service_uuids:
                    adv_service_uuids = [str(uuid) for uuid in adv_data.service_uuids]
                
                # Display device info
                status_indicators = []
                if matches:
                    status_indicators.append("🎯")
                if SERVICE_UUID.lower() in [uuid.lower() for uuid in adv_service_uuids]:
                    status_indicators.append("🔍")
                    
                status = " " + " ".join(status_indicators) if status_indicators else ""
                
                print(f"   • '{name}' ({address}) RSSI: {device.rssi}dBm{status}")
                
                if matches:
                    print(f"     Pattern matches: {', '.join(matches)}")
                    
                if adv_service_uuids:
                    print(f"     Advertised services: {adv_service_uuids}")
                elif matches:  # Only show for potential matches
                    print(f"     Advertised services: None (service UUID only available after connection)")
                    
            # Analysis summary
            print(f"\n📋 Pattern Matching Analysis:")
            print(f"   Expected device name: '{TARGET_DEVICE_NAME}'")
            print(f"   Known truncated name: 'YahboomRob' (nRF Connect finding)")
            
            if yahboom_candidates:
                print(f"   🎯 Found {len(yahboom_candidates)} potential Yahboom devices:")
                for device in yahboom_candidates:
                    actual_name = device.name or "No Name"
                    truncation_note = ""
                    if actual_name == "YahboomRob":
                        truncation_note = " (BLE name truncation detected!)"
                    print(f"      • '{actual_name}' ({device.address}){truncation_note}")
            else:
                print(f"   ❌ No devices matched any Yahboom patterns")
                print(f"   🔍 Patterns checked:")
                print(f"      • Exact match: '{TARGET_DEVICE_NAME}'")
                print(f"      • Truncated: 'YahboomRob'")
                print(f"      • Prefix: 'Yahboom*'")
                print(f"      • Substring: '*yahboom*' (case insensitive)")
                print(f"      • Generic: '*robot*' (case insensitive)")
                print(f"      • Flexible: 'yah*' (case insensitive)")
                
            # Service UUID analysis
            service_uuid_note = f"""
🔍 Service UUID Discovery Analysis:
   Target service: {SERVICE_UUID}
   ❌ Service UUIDs NOT found in advertising data (this is normal!)
   ✅ Service validation happens AFTER connection (Stage 2)
   📱 This matches nRF Connect behavior - services appear after connection"""
            print(service_uuid_note)
                
            # Platform-specific guidance
            print(f"\n🖥️  Platform-Specific Troubleshooting:")
            
            import platform
            system = platform.system().lower()
            
            if system == "linux":
                print("   📟 Linux/Ubuntu detected:")
                print("      • Device name truncation is common (BlueZ behavior)")
                print("      • 'YahboomRobot' becomes 'YahboomRob' due to advertising limits")
                print("      • Service UUID NOT in advertising data (normal)")
                print("      • Run server with: sudo python3 ble_server.py")
                print("      • Our enhanced client handles truncation automatically")
                
            elif system == "darwin":
                print("   🍎 macOS detected:")
                print("      • Device name may or may not be truncated")
                print("      • Check System Preferences → Security & Privacy → Bluetooth")
                print("      • Service UUID still not in advertising (normal BLE behavior)")
                
            elif system == "windows":
                print("   🪟 Windows detected:")
                print("      • BLE advertising behavior varies by Windows version")
                print("      • Ensure Windows Bluetooth drivers are up to date")
                
            print(f"\n💡 Next Steps:")
            print(f"   1. Verify BLE server running: python3 ble_server.py")
            print(f"   2. Look for 'YahboomRob' or similar in device scan")
            print(f"   3. Enhanced client now tries pattern matching + service validation")
            print(f"   4. Check nRF Connect app shows same truncated name")
            print(f"   5. Service validation happens after connection (Stage 2)")
            
        except Exception as e:
            print(f"❌ Debug analysis failed: {e}")
            
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
