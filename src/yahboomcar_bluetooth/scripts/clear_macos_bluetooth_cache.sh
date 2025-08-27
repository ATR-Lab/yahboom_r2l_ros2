#!/bin/bash

# macOS Bluetooth Cache Clearing Script
# =====================================
#
# This script comprehensively clears macOS Bluetooth cache to resolve
# BLE device name caching issues where Python scripts see old device names
# while other BLE apps see current names.
#
# Problem Solved:
#   - test_ros2_bridge.py sees cached "YahboomRobot" 
#   - nRF Connect shows correct "YahboomRacer_Car1"
#   - Standard cache clearing insufficient for BLE device databases
#
# Usage:
#   chmod +x clear_macos_bluetooth_cache.sh
#   ./clear_macos_bluetooth_cache.sh
#
# Requirements:
#   - macOS (tested on Sequoia 15.5+)
#   - Administrator privileges (will prompt for sudo password)
#   - REBOOT REQUIRED after completion
#
# Author: Yahboom R2L Racing Team
# Date: 2024

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "\n${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║${NC}          macOS Bluetooth Cache Clearing Script           ${BLUE}║${NC}"
    echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}\n"
}

# Function to check if running on macOS
check_macos() {
    if [[ "$OSTYPE" != "darwin"* ]]; then
        print_error "This script is designed for macOS only."
        print_error "Current OS: $OSTYPE"
        exit 1
    fi
    
    # Get macOS version
    macos_version=$(sw_vers -productVersion)
    print_status "Detected macOS version: $macos_version"
}

# Function to backup files before deletion
backup_files() {
    print_status "Creating backup directory for Bluetooth preferences..."
    
    backup_dir="$HOME/bluetooth_cache_backup_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$backup_dir"
    
    # Backup files that exist
    if [[ -f "/Library/Preferences/com.apple.Bluetooth.plist" ]]; then
        sudo cp "/Library/Preferences/com.apple.Bluetooth.plist" "$backup_dir/com.apple.Bluetooth.plist" 2>/dev/null || true
    fi
    
    if [[ -f "$HOME/Library/Preferences/com.apple.bluetoothuserd.plist" ]]; then
        cp "$HOME/Library/Preferences/com.apple.bluetoothuserd.plist" "$backup_dir/com.apple.bluetoothuserd.plist" 2>/dev/null || true
    fi
    
    # Backup BLE database files
    if [[ -d "/Library/Bluetooth" ]]; then
        sudo cp -r "/Library/Bluetooth" "$backup_dir/Bluetooth_backup" 2>/dev/null || true
    fi
    
    print_success "Backup created at: $backup_dir"
}

# Function to clear standard Bluetooth cache
clear_standard_cache() {
    print_status "Step 1: Clearing standard Bluetooth cache files..."
    
    # Kill Bluetooth daemon
    print_status "Killing Bluetooth daemon..."
    sudo pkill bluetoothd 2>/dev/null || print_warning "bluetoothd not running or already stopped"
    sleep 1
    
    # Remove system Bluetooth preferences
    if [[ -f "/Library/Preferences/com.apple.Bluetooth.plist" ]]; then
        print_status "Removing system Bluetooth preferences..."
        sudo rm -rf "/Library/Preferences/com.apple.Bluetooth.plist"
        print_success "System Bluetooth preferences removed"
    else
        print_warning "System Bluetooth preferences file not found"
    fi
    
    # Remove user Bluetooth preferences  
    if [[ -f "$HOME/Library/Preferences/com.apple.bluetoothuserd.plist" ]]; then
        print_status "Removing user Bluetooth preferences..."
        rm -rf "$HOME/Library/Preferences/com.apple.bluetoothuserd.plist"
        print_success "User Bluetooth preferences removed"
    else
        print_warning "User Bluetooth preferences file not found"
    fi
}

# Function to clear BLE device database files (KEY STEP)
clear_ble_databases() {
    print_status "Step 2: Clearing BLE device database files (CRITICAL for device name caching)..."
    
    # Clear BLE device databases - unpaired/other devices
    if ls /Library/Bluetooth/com.apple.MobileBluetooth.ledevices.other.db* >/dev/null 2>&1; then
        print_status "Removing BLE other devices database..."
        sudo rm -f /Library/Bluetooth/com.apple.MobileBluetooth.ledevices.other.db*
        print_success "BLE other devices database removed"
    else
        print_warning "BLE other devices database not found"
    fi
    
    # Clear BLE device databases - paired devices
    if ls /Library/Bluetooth/com.apple.MobileBluetooth.ledevices.paired.db* >/dev/null 2>&1; then
        print_status "Removing BLE paired devices database..."
        sudo rm -f /Library/Bluetooth/com.apple.MobileBluetooth.ledevices.paired.db*
        print_success "BLE paired devices database removed"
    else
        print_warning "BLE paired devices database not found"
    fi
    
    # Clear additional MobileBluetooth preferences
    if [[ -f "/Library/Bluetooth/Library/Preferences/com.apple.MobileBluetooth.devices.plist" ]]; then
        print_status "Removing MobileBluetooth devices preferences..."
        sudo rm -f "/Library/Bluetooth/Library/Preferences/com.apple.MobileBluetooth.devices.plist"
        print_success "MobileBluetooth devices preferences removed"
    else
        print_warning "MobileBluetooth devices preferences not found"
    fi
}

# Function to restart Bluetooth daemon
restart_bluetooth() {
    print_status "Step 3: Restarting Bluetooth daemon..."
    
    # Kill Bluetooth daemon (it will auto-restart)
    sudo pkill bluetoothd 2>/dev/null || print_warning "bluetoothd not running"
    sleep 2
    
    # Check if Bluetooth daemon restarted
    if pgrep bluetoothd >/dev/null; then
        print_success "Bluetooth daemon restarted successfully"
        print_status "Bluetooth daemon PID: $(pgrep bluetoothd)"
    else
        print_warning "Bluetooth daemon may not have restarted automatically"
    fi
}

# Function to flush system caches
flush_system_caches() {
    print_status "Step 4: Flushing system caches..."
    
    # Flush DNS/system cache
    sudo dscacheutil -flushcache 2>/dev/null || print_warning "Failed to flush system cache"
    print_success "System caches flushed"
}

# Function to show final instructions
show_final_instructions() {
    print_success "\n✅ Bluetooth cache clearing completed successfully!"
    
    echo -e "\n${YELLOW}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${YELLOW}║${NC}                     IMPORTANT NEXT STEPS                  ${YELLOW}║${NC}"
    echo -e "${YELLOW}╚════════════════════════════════════════════════════════════╝${NC}\n"
    
    print_warning "🔄 REBOOT REQUIRED: You MUST restart your Mac for changes to take effect"
    print_warning "💻 Run: sudo reboot"
    
    echo -e "\n${GREEN}After reboot, test your BLE script:${NC}"
    echo -e "   cd $(pwd)"
    echo -e "   python3 test_ros2_bridge.py"
    echo -e "   # Should now find 'YahboomRacer_Car1' instead of cached 'YahboomRobot'"
    
    echo -e "\n${BLUE}What was cleared:${NC}"
    echo -e "   ✅ Standard Bluetooth preference files"
    echo -e "   ✅ BLE device database files (other devices)"
    echo -e "   ✅ BLE device database files (paired devices)"
    echo -e "   ✅ MobileBluetooth device preferences"
    echo -e "   ✅ System caches flushed"
    echo -e "   ✅ Bluetooth daemon restarted"
    
    if [[ -n "$backup_dir" ]]; then
        echo -e "\n${BLUE}Backup location:${NC} $backup_dir"
    fi
    
    print_warning "\n⚠️  Note: Hidden Bluetooth debug menu (Option+Shift+Bluetooth) was removed in macOS Sequoia 15.5+"
    print_warning "   Manual cache clearing (this script) is now the only reliable method"
}

# Function to get user confirmation
get_confirmation() {
    echo -e "\n${YELLOW}This script will:${NC}"
    echo -e "  1. Kill the Bluetooth daemon"
    echo -e "  2. Remove Bluetooth preference files"
    echo -e "  3. Clear BLE device database files"
    echo -e "  4. Restart Bluetooth daemon"
    echo -e "  5. Flush system caches"
    echo -e "\n${RED}WARNING:${NC} This will remove all Bluetooth pairings and cached device information"
    echo -e "${RED}WARNING:${NC} You will need to re-pair all Bluetooth devices after reboot"
    echo -e "\n${BLUE}A backup will be created before making changes${NC}"
    
    read -p $'\n'"Continue with Bluetooth cache clearing? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_status "Operation cancelled by user"
        exit 0
    fi
}

# Main execution function
main() {
    print_header
    
    # Check if running on macOS
    check_macos
    
    # Get user confirmation
    get_confirmation
    
    # Create backup
    backup_files
    
    # Clear standard Bluetooth cache
    clear_standard_cache
    
    # Clear BLE device databases (critical step)
    clear_ble_databases
    
    # Restart Bluetooth daemon
    restart_bluetooth
    
    # Flush system caches
    flush_system_caches
    
    # Show final instructions
    show_final_instructions
    
    echo -e "\n${GREEN}🎉 Script completed successfully!${NC}"
    echo -e "${YELLOW}💻 Remember to reboot your Mac: sudo reboot${NC}\n"
}

# Check if script is being sourced or executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
