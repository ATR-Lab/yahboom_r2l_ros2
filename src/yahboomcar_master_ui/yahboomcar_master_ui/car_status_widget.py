#!/usr/bin/env python3
"""
Car Status Widget - Individual car monitoring and control card.
Displays comprehensive status for each robot car with integrated connection monitoring.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
    QProgressBar, QFrame, QGridLayout
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPalette, QPainter

from .car_details.detail_dialog import CarDetailDialog


class RotatedLabel(QWidget):
    """Custom widget that can display rotated text without borders."""
    
    def __init__(self, text, rotation_angle=0, parent=None):
        super().__init__(parent)
        self.text_content = text
        self.rotation_angle = rotation_angle
        
        # Make background transparent
        self.setAttribute(Qt.WA_TranslucentBackground, True)
        self.setStyleSheet("background-color: transparent;")
        
    def paintEvent(self, event):
        """Custom paint event to draw rotated text."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Get the center of the widget
        center_x = self.width() // 2
        center_y = self.height() // 2
        
        # Set up font and color
        from PyQt5.QtGui import QColor, QFont
        painter.setPen(QColor(255, 255, 255))  # White color
        font = QFont()
        font.setPointSize(22)  # Larger font for single car layout
        font.setBold(True)
        painter.setFont(font)
        
        # Apply rotation around center
        painter.translate(center_x, center_y)
        painter.rotate(self.rotation_angle)
        
        # Draw text centered within the full widget area
        # Use larger drawing area based on widget dimensions
        text_width = max(250, self.width() - 50)
        text_height = 50
        painter.drawText(-text_width//2, -text_height//2, text_width, text_height, Qt.AlignCenter, self.text_content)
        painter.end()


class CarStatusWidget(QWidget):
    """Widget displaying comprehensive status for a single robot car."""
    
    def __init__(self, car_config, data_manager):
        super().__init__()
        self.car_id = car_config["id"]
        self.car_name = car_config["name"]
        self.car_color = car_config["color"]
        self.car_active = car_config["active"]
        self.data_manager = data_manager
        
        self.setFixedSize(300, 200)
        self._init_ui()
        
        # Update timer for real-time data
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_data)
        self.update_timer.start(200)  # 5Hz updates
    
    def _init_ui(self):
        """Initialize the car status UI."""
        self.setStyleSheet(f"""
            QWidget {{
                border: 2px solid {self.car_color};
                border-radius: 8px;
                background-color: #2b2b2b;
                margin: 2px;
            }}
        """)
        
        layout = QVBoxLayout(self)
        layout.setSpacing(5)
        layout.setContentsMargins(8, 8, 8, 8)
        
        # Header with name and kill button
        header_layout = QHBoxLayout()
        
        # Car name and ID
        name_label = QLabel(f"🚗 CAR #{self.car_id}")
        name_label.setStyleSheet("font-weight: bold; font-size: 12px; color: white;")
        header_layout.addWidget(name_label)
        
        car_name_label = QLabel(f'"{self.car_name}"')
        car_name_label.setStyleSheet(f"color: {self.car_color}; font-weight: bold;")
        header_layout.addWidget(car_name_label)
        
        header_layout.addStretch()
        
        # Kill button
        self.kill_button = QPushButton("🔴 KILL")
        self.kill_button.setFixedSize(60, 25)
        self.kill_button.setStyleSheet("""
            QPushButton {
                background-color: #cc0000;
                color: white;
                font-weight: bold;
                border: 1px solid #ff0000;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #ff0000;
                border: 1px solid #ff3333;
            }
            QPushButton:pressed {
                background-color: #aa0000;
                border: 1px solid #cc0000;
            }
        """)
        self.kill_button.clicked.connect(self._emergency_stop)
        header_layout.addWidget(self.kill_button)
        
        layout.addLayout(header_layout)
        
        # Connection status indicators
        conn_layout = QHBoxLayout()
        
        self.webrtc_indicator = self._create_status_indicator("WebRTC", "🔴")
        self.bluetooth_indicator = self._create_status_indicator("Bluetooth", "🔴")
        self.ros2_indicator = self._create_status_indicator("ROS2", "🔴")
        
        conn_layout.addWidget(self.webrtc_indicator)
        conn_layout.addWidget(self.bluetooth_indicator)
        conn_layout.addWidget(self.ros2_indicator)
        
        layout.addLayout(conn_layout)
        
        # Control mode indicator
        self.control_mode_frame = QFrame()
        self.control_mode_frame.setFixedHeight(25)
        self.control_mode_frame.setStyleSheet("background-color: #404040; border-radius: 3px;")
        
        mode_layout = QHBoxLayout(self.control_mode_frame)
        mode_layout.setContentsMargins(5, 2, 5, 2)
        
        self.control_mode_label = QLabel("RACING MODE")
        self.control_mode_label.setStyleSheet("color: #00ff00; font-weight: bold; font-size: 10px;")
        self.control_mode_label.setAlignment(Qt.AlignCenter)
        mode_layout.addWidget(self.control_mode_label)
        
        layout.addWidget(self.control_mode_frame)
        
        # Vital signs
        vitals_layout = QGridLayout()
        vitals_layout.setSpacing(3)
        
        # Battery
        battery_layout = QHBoxLayout()
        battery_layout.addWidget(QLabel("⚡"))
        
        self.battery_bar = QProgressBar()
        self.battery_bar.setFixedHeight(15)
        self.battery_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #666;
                border-radius: 3px;
                text-align: center;
                font-size: 9px;
            }
            QProgressBar::chunk {
                background-color: #4caf50;
                border-radius: 2px;
            }
        """)
        battery_layout.addWidget(self.battery_bar)
        
        self.battery_voltage_label = QLabel("12.4V")
        self.battery_voltage_label.setStyleSheet("font-size: 9px; color: #aaa;")
        battery_layout.addWidget(self.battery_voltage_label)
        
        vitals_layout.addLayout(battery_layout, 0, 0, 1, 2)
        
        # Speed and position
        self.speed_label = QLabel("🏎️ 0.0 m/s")
        self.speed_label.setStyleSheet("font-size: 10px;")
        vitals_layout.addWidget(self.speed_label, 1, 0)
        
        self.position_label = QLabel("📍 (0.0, 0.0)")
        self.position_label.setStyleSheet("font-size: 10px;")
        vitals_layout.addWidget(self.position_label, 1, 1)
        
        self.heading_label = QLabel("🧭 0°")
        self.heading_label.setStyleSheet("font-size: 10px;")
        vitals_layout.addWidget(self.heading_label, 2, 0)
        
        layout.addLayout(vitals_layout)
        
        # Action buttons
        actions_layout = QHBoxLayout()
        
        self.manual_button = QPushButton("📱 MANUAL")
        self.manual_button.setFixedHeight(25)
        self.manual_button.setStyleSheet("""
            QPushButton {
                background-color: #0066cc;
                color: white;
                font-size: 9px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #0080ff;
                border: 1px solid #33aaff;
            }
            QPushButton:pressed {
                background-color: #004499;
                border: 1px solid #0066cc;
            }
        """)
        self.manual_button.clicked.connect(self._toggle_manual_control)
        actions_layout.addWidget(self.manual_button)
        
        self.reset_button = QPushButton("🔄 RESET")
        self.reset_button.setFixedHeight(25)
        self.reset_button.setStyleSheet("""
            QPushButton {
                background-color: #009900;
                color: white;
                font-size: 9px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #00cc00;
                border: 1px solid #33dd33;
            }
            QPushButton:pressed {
                background-color: #007700;
                border: 1px solid #009900;
            }
        """)
        self.reset_button.clicked.connect(self._reset_car)
        actions_layout.addWidget(self.reset_button)
        
        self.details_button = QPushButton("📊")
        self.details_button.setFixedSize(30, 25)
        self.details_button.setStyleSheet("""
            QPushButton {
                background-color: #666;
                color: white;
                font-size: 9px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #777;
                border: 1px solid #888;
            }
            QPushButton:pressed {
                background-color: #555;
                border: 1px solid #666;
            }
        """)
        self.details_button.clicked.connect(self._show_car_details)
        actions_layout.addWidget(self.details_button)
        
        layout.addLayout(actions_layout)
        
        # Add inactive overlay if car is not active
        self.inactive_overlay = None
        if not self.car_active:
            self._create_inactive_overlay()
    
    def _create_status_indicator(self, name, initial_status):
        """Create a connection status indicator."""
        indicator = QLabel(f"{initial_status} {name}")
        indicator.setStyleSheet("font-size: 9px; padding: 2px;")
        return indicator
    
    def _update_data(self):
        """Update the widget with latest car data."""
        car_data = self.data_manager.get_car_data(self.car_id)
        if not car_data:
            return
        
        # Update connection indicators
        self._update_connection_indicator(
            self.webrtc_indicator, "WebRTC", car_data.webrtc_connected
        )
        self._update_connection_indicator(
            self.bluetooth_indicator, "Bluetooth", car_data.bluetooth_connected
        )
        self._update_connection_indicator(
            self.ros2_indicator, "ROS2", car_data.ros2_connected
        )
        
        # Update control mode
        self._update_control_mode(car_data.control_mode, car_data.emergency_stopped)
        
        # Update battery
        self.battery_bar.setValue(car_data.battery_percentage)
        self.battery_voltage_label.setText(f"{car_data.battery_voltage:.1f}V")
        
        # Update battery color based on level
        if car_data.battery_percentage < 20:
            color = "#f44336"  # Red
        elif car_data.battery_percentage < 40:
            color = "#ff9800"  # Orange
        else:
            color = "#4caf50"  # Green
        
        self.battery_bar.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #666;
                border-radius: 3px;
                text-align: center;
                font-size: 9px;
            }}
            QProgressBar::chunk {{
                background-color: {color};
                border-radius: 2px;
            }}
        """)
        
        # Update speed and position
        self.speed_label.setText(f"🏎️ {car_data.speed_linear:.1f} m/s")
        self.position_label.setText(f"📍 ({car_data.position_x:.1f}, {car_data.position_y:.1f})")
        self.heading_label.setText(f"🧭 {car_data.heading:.0f}°")
        
        # Update button states
        self._update_button_states(car_data)
    
    def _update_connection_indicator(self, indicator, name, connected):
        """Update a connection status indicator."""
        if connected:
            indicator.setText(f"🟢 {name}")
            indicator.setStyleSheet("font-size: 9px; padding: 2px; color: #4caf50;")
        else:
            indicator.setText(f"🔴 {name}")
            indicator.setStyleSheet("font-size: 9px; padding: 2px; color: #f44336;")
    
    def _update_control_mode(self, mode, emergency_stopped):
        """Update the control mode display."""
        if emergency_stopped:
            self.control_mode_label.setText("🚨 EMERGENCY STOPPED")
            self.control_mode_label.setStyleSheet("color: #ff0000; font-weight: bold; font-size: 10px;")
            self.control_mode_frame.setStyleSheet("background-color: #660000; border-radius: 3px;")
        elif mode == "MANUAL":
            self.control_mode_label.setText("🎮 MANUAL CONTROL ACTIVE")
            self.control_mode_label.setStyleSheet("color: #00ccff; font-weight: bold; font-size: 10px;")
            self.control_mode_frame.setStyleSheet("background-color: #003366; border-radius: 3px;")
        else:  # RACING
            self.control_mode_label.setText("🏁 RACING MODE")
            self.control_mode_label.setStyleSheet("color: #00ff00; font-weight: bold; font-size: 10px;")
            self.control_mode_frame.setStyleSheet("background-color: #404040; border-radius: 3px;")
    
    def _update_button_states(self, car_data):
        """Update button enable/disable states."""
        # Kill button always enabled unless already stopped
        self.kill_button.setEnabled(not car_data.emergency_stopped)
        
        # Reset button enabled only if emergency stopped
        self.reset_button.setEnabled(car_data.emergency_stopped)
        
        # Manual button enabled if ROS2 connected and not emergency stopped
        manual_enabled = car_data.ros2_connected and not car_data.emergency_stopped
        self.manual_button.setEnabled(manual_enabled)
        
        # Update manual button text based on current mode
        if car_data.control_mode == "MANUAL":
            self.manual_button.setText("⏹️ RELEASE")
        else:
            self.manual_button.setText("📱 MANUAL")
    
    def _emergency_stop(self):
        """Trigger emergency stop for this car."""
        self.data_manager.emergency_stop(self.car_id)
    
    def _toggle_manual_control(self):
        """Toggle manual control for this car."""
        current_mode = self.data_manager.get_car_control_mode(self.car_id)
        if current_mode == "MANUAL":
            self.data_manager.set_manual_control(self.car_id, False)
        else:
            self.data_manager.set_manual_control(self.car_id, True)
    
    def _reset_car(self):
        """Reset car from emergency stop - Enhanced with ROS2 service call."""
        # Log the user action
        self.data_manager.log_user_action(f"Reset car #{self.car_id}", f"Car: {self.car_name}")
        
        # Call dummy ROS2 service (will be replaced with actual service call)
        self.data_manager.call_car_reset_system(self.car_id)
        
        # Also call the existing data manager method for UI state
        self.data_manager.reset_car(self.car_id)
    
    def _show_car_details(self):
        """Show detailed car diagnostics popup dialog."""
        # Log the user action
        self.data_manager.log_user_action(f"View details for car #{self.car_id}", f"Car: {self.car_name}")
        
        # Open the car detail dialog
        dialog = CarDetailDialog(self.car_id, self.data_manager, self)
        dialog.exec_()
    
    def _create_inactive_overlay(self):
        """Create semi-transparent overlay for inactive car."""
        self.inactive_overlay = QWidget(self)
        self.inactive_overlay.setStyleSheet("""
            QWidget {
                background-color: rgba(40, 40, 40, 0.8);
                border-radius: 8px;
            }
        """)
        
        # Size overlay to match parent widget exactly
        self.inactive_overlay.resize(self.size())
        self.inactive_overlay.move(0, 0)
        
        # Add rotated INACTIVE text label spanning full overlay
        self.inactive_label = RotatedLabel("INACTIVE", -45, self.inactive_overlay)
        
        # Make the label span the full overlay size dynamically
        self.inactive_label.resize(self.inactive_overlay.size())
        self.inactive_label.move(0, 0)
        
        # Block mouse events to prevent interaction
        self.inactive_overlay.setAttribute(Qt.WA_TransparentForMouseEvents, False)
        self.inactive_overlay.show()
    
    def resizeEvent(self, event):
        """Handle resize events to keep overlay properly sized."""
        super().resizeEvent(event)
        if hasattr(self, 'inactive_overlay') and self.inactive_overlay:
            # Resize overlay to match parent widget
            self.inactive_overlay.resize(self.size())
            # Resize label to match overlay
            if hasattr(self, 'inactive_label') and self.inactive_label:
                self.inactive_label.resize(self.inactive_overlay.size())