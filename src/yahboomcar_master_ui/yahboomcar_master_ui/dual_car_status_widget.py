#!/usr/bin/env python3
"""
Dual Car Status Widget - Two-column car monitoring card.
Displays comprehensive status for two robot cars side-by-side for better space utilization.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
    QProgressBar, QFrame, QGridLayout
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPalette, QPainter, QTransform


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
        font.setPointSize(18)  # Slightly smaller font for better fit
        font.setBold(True)
        painter.setFont(font)
        
        # Apply rotation around center
        painter.translate(center_x, center_y)
        painter.rotate(self.rotation_angle)
        
        # Draw text centered within the full widget area
        # Use larger drawing area based on widget dimensions
        text_width = max(200, self.width() - 40)
        text_height = 40
        painter.drawText(-text_width//2, -text_height//2, text_width, text_height, Qt.AlignCenter, self.text_content)
        painter.end()


class SingleCarSection(QWidget):
    """Individual car section within the dual card."""
    
    def __init__(self, car_config, data_manager):
        super().__init__()
        self.car_id = car_config["id"]
        self.car_name = car_config["name"]
        self.car_color = car_config["color"]
        self.car_active = car_config["active"]
        self.data_manager = data_manager
        
        self.setMinimumWidth(240)  # Each car gets more width than before
        self._init_ui()
    
    def _init_ui(self):
        """Initialize the single car section UI."""
        # Create QFrame container with colored border
        container_frame = QFrame()
        container_frame.setObjectName("car_container")
        container_frame.setStyleSheet(f"""
            QFrame#car_container {{
                border: 2px solid {self.car_color};
                border-radius: 8px;
                background-color: #2b2b2b;
                margin: 2px;
            }}
        """)
        
        # Main widget layout contains just the frame
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(container_frame)
        
        # All content goes inside the frame
        layout = QVBoxLayout(container_frame)
        layout.setSpacing(4)
        layout.setContentsMargins(8, 8, 8, 8)
        
        # Header with car number, name, and kill button all on one line
        header_layout = QHBoxLayout()
        
        # Car number and name on single line
        name_label = QLabel(f'🚗 CAR #{self.car_id} "{self.car_name}"')
        name_label.setStyleSheet(f"font-weight: bold; font-size: 12px; color: {self.car_color};")
        header_layout.addWidget(name_label)
        
        header_layout.addStretch()
        
        # Kill button with full text
        self.kill_button = QPushButton("🔴 KILL")
        self.kill_button.setFixedSize(70, 25)
        self.kill_button.setStyleSheet("""
            QPushButton {
                background-color: #cc0000;
                color: white;
                font-weight: bold;
                border: 1px solid #ff0000;
                border-radius: 3px;
                font-size: 10px;
            }
            QPushButton:hover {
                background-color: #ff0000;
                border: 1px solid #ff3333;
            }
            QPushButton:pressed {
                background-color: #aa0000;
                border: 1px solid #cc0000;
                transform: translateY(1px);
            }
        """)
        self.kill_button.clicked.connect(self._emergency_stop)
        header_layout.addWidget(self.kill_button)
        
        layout.addLayout(header_layout)
        
        # Connection status indicators with full labels
        conn_layout = QHBoxLayout()
        conn_layout.setSpacing(8)
        
        self.webrtc_indicator = self._create_status_indicator("WebRTC", "🔴")
        self.bluetooth_indicator = self._create_status_indicator("Bluetooth", "🔴") 
        self.ros2_indicator = self._create_status_indicator("ROS2", "🔴")
        
        conn_layout.addWidget(self.webrtc_indicator)
        conn_layout.addWidget(self.bluetooth_indicator)
        conn_layout.addWidget(self.ros2_indicator)
        
        layout.addLayout(conn_layout)
        
        # Control mode indicator
        self.control_mode_frame = QFrame()
        self.control_mode_frame.setFixedHeight(22)
        self.control_mode_frame.setStyleSheet("background-color: #404040; border-radius: 3px;")
        
        mode_layout = QHBoxLayout(self.control_mode_frame)
        mode_layout.setContentsMargins(4, 2, 4, 2)
        
        self.control_mode_label = QLabel("RACING")
        self.control_mode_label.setStyleSheet("color: #00ff00; font-weight: bold; font-size: 9px;")
        self.control_mode_label.setAlignment(Qt.AlignCenter)
        mode_layout.addWidget(self.control_mode_label)
        
        layout.addWidget(self.control_mode_frame)
        
        # Battery with larger progress bar
        battery_layout = QHBoxLayout()
        battery_layout.addWidget(QLabel("⚡"))
        
        self.battery_bar = QProgressBar()
        self.battery_bar.setFixedHeight(18)  # Slightly larger
        self.battery_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #666;
                border-radius: 3px;
                text-align: center;
                font-size: 10px;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background-color: #4caf50;
                border-radius: 2px;
            }
        """)
        battery_layout.addWidget(self.battery_bar)
        
        self.battery_voltage_label = QLabel("12.4V")
        self.battery_voltage_label.setStyleSheet("font-size: 10px; color: #aaa;")
        battery_layout.addWidget(self.battery_voltage_label)
        
        layout.addLayout(battery_layout)
        
        # Speed and position (better spaced)
        metrics_layout = QVBoxLayout()
        metrics_layout.setSpacing(2)
        
        self.speed_label = QLabel("🏎️ 0.0 m/s")
        self.speed_label.setStyleSheet("font-size: 11px;")
        metrics_layout.addWidget(self.speed_label)
        
        self.position_label = QLabel("📍 (0.0, 0.0)")
        self.position_label.setStyleSheet("font-size: 11px;")
        metrics_layout.addWidget(self.position_label)
        
        self.heading_label = QLabel("🧭 0°")
        self.heading_label.setStyleSheet("font-size: 11px;")
        metrics_layout.addWidget(self.heading_label)
        
        layout.addLayout(metrics_layout)
        
        # Action buttons with full text labels
        actions_layout = QHBoxLayout()
        actions_layout.setSpacing(4)
        
        self.manual_button = QPushButton("📱 MANUAL")
        self.manual_button.setFixedSize(80, 25)
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
                transform: translateY(1px);
            }
        """)
        self.manual_button.clicked.connect(self._toggle_manual_control)
        actions_layout.addWidget(self.manual_button)
        
        self.reset_button = QPushButton("🔄 RESET")
        self.reset_button.setFixedSize(70, 25)
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
                transform: translateY(1px);
            }
        """)
        self.reset_button.clicked.connect(self._reset_car)
        actions_layout.addWidget(self.reset_button)
        
        self.details_button = QPushButton("📊 INFO")
        self.details_button.setFixedSize(65, 25)
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
                transform: translateY(1px);
            }
        """)
        self.details_button.clicked.connect(self._show_car_details)
        actions_layout.addWidget(self.details_button)
        
        layout.addLayout(actions_layout)
        
        # Store reference to container frame for overlay
        self.container_frame = container_frame
        
        # Add inactive overlay if car is not active
        self.inactive_overlay = None
        if not self.car_active:
            self._create_inactive_overlay()
    
    def _create_status_indicator(self, name, initial_status):
        """Create a connection status indicator with full name."""
        indicator = QLabel(f"{initial_status} {name}")
        indicator.setStyleSheet("font-size: 9px; padding: 2px;")
        return indicator
    
    def update_data(self):
        """Update this car section with latest data."""
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
        
        # Update battery color
        if car_data.battery_percentage < 20:
            color = "#f44336"
        elif car_data.battery_percentage < 40:
            color = "#ff9800"
        else:
            color = "#4caf50"
        
        self.battery_bar.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #666;
                border-radius: 3px;
                text-align: center;
                font-size: 10px;
                font-weight: bold;
            }}
            QProgressBar::chunk {{
                background-color: {color};
                border-radius: 2px;
            }}
        """)
        
        # Update metrics
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
            self.control_mode_label.setText("STOPPED")
            self.control_mode_label.setStyleSheet("color: #ff0000; font-weight: bold; font-size: 9px;")
            self.control_mode_frame.setStyleSheet("background-color: #660000; border-radius: 3px;")
        elif mode == "MANUAL":
            self.control_mode_label.setText("MANUAL")
            self.control_mode_label.setStyleSheet("color: #00ccff; font-weight: bold; font-size: 9px;")
            self.control_mode_frame.setStyleSheet("background-color: #003366; border-radius: 3px;")
        else:
            self.control_mode_label.setText("RACING")
            self.control_mode_label.setStyleSheet("color: #00ff00; font-weight: bold; font-size: 9px;")
            self.control_mode_frame.setStyleSheet("background-color: #404040; border-radius: 3px;")
    
    def _update_button_states(self, car_data):
        """Update button states based on car data."""
        self.kill_button.setEnabled(not car_data.emergency_stopped)
        self.reset_button.setEnabled(car_data.emergency_stopped)
        
        manual_enabled = car_data.ros2_connected and not car_data.emergency_stopped
        self.manual_button.setEnabled(manual_enabled)
        
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
        """Show detailed car diagnostics - TODO: Create detailed panel/popup"""
        # Log the user action
        self.data_manager.log_user_action(f"View details for car #{self.car_id}", f"Car: {self.car_name}")
        
        # Call dummy ROS2 diagnostics service
        self.data_manager.call_car_diagnostics(self.car_id)
        
        # TODO: Open detailed diagnostics panel/popup window
        # For now, just log the action
        print(f"DUMMY: Opening detailed view for Car #{self.car_id} '{self.car_name}'")
    
    def _create_inactive_overlay(self):
        """Create semi-transparent overlay for inactive car."""
        self.inactive_overlay = QWidget(self.container_frame)
        self.inactive_overlay.setStyleSheet("""
            QWidget {
                background-color: rgba(40, 40, 40, 0.8);
                border-radius: 8px;
            }
        """)
        
        # Size overlay to match container frame exactly
        self.inactive_overlay.resize(self.container_frame.size())
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
        if hasattr(self, 'inactive_overlay') and self.inactive_overlay and hasattr(self, 'container_frame'):
            # Resize overlay to match container frame
            self.inactive_overlay.resize(self.container_frame.size())
            # Resize label to match overlay
            if hasattr(self, 'inactive_label') and self.inactive_label:
                self.inactive_label.resize(self.inactive_overlay.size())


class DualCarStatusWidget(QWidget):
    """Widget displaying status for two robot cars side-by-side."""
    
    def __init__(self, car_configs_pair, data_manager):
        super().__init__()
        self.car_configs = car_configs_pair
        self.data_manager = data_manager
        self.car_sections = {}
        
        self.setMinimumSize(500, 240)  # Reduced height for screen compatibility
        self.setMaximumHeight(280)     # Prevent excessive growth
        self._init_ui()
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_data)
        self.update_timer.start(200)  # 5Hz updates
    
    def _init_ui(self):
        """Initialize the dual car status UI."""
        # Main container styling
        self.setStyleSheet("""
            DualCarStatusWidget {
                background-color: #2b2b2b;
                border-radius: 8px;
                margin: 3px;
            }
        """)
        
        # Horizontal layout for two cars
        layout = QHBoxLayout(self)
        layout.setSpacing(1)  # Minimal spacing between cars
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Create car sections
        for i, car_config in enumerate(self.car_configs):
            car_section = SingleCarSection(car_config, self.data_manager)
            self.car_sections[car_config["id"]] = car_section
            
            layout.addWidget(car_section)
            
            # Add separator between cars (except after last)
            if i < len(self.car_configs) - 1:
                separator = QFrame()
                separator.setFrameShape(QFrame.VLine)
                separator.setStyleSheet("color: #555; background-color: #555;")
                separator.setFixedWidth(1)
                layout.addWidget(separator)
    
    def _update_data(self):
        """Update all car sections with latest data."""
        for car_section in self.car_sections.values():
            car_section.update_data()