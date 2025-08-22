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
from PyQt5.QtGui import QFont, QPalette


class SingleCarSection(QWidget):
    """Individual car section within the dual card."""
    
    def __init__(self, car_config, data_manager):
        super().__init__()
        self.car_id = car_config["id"]
        self.car_name = car_config["name"]
        self.car_color = car_config["color"]
        self.data_manager = data_manager
        
        self.setMinimumWidth(240)  # Each car gets more width than before
        self._init_ui()
    
    def _init_ui(self):
        """Initialize the single car section UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(4)
        layout.setContentsMargins(8, 8, 8, 8)
        
        # Header with name and kill button
        header_layout = QHBoxLayout()
        
        # Car name and ID
        name_label = QLabel(f"🚗 CAR #{self.car_id}")
        name_label.setStyleSheet("font-weight: bold; font-size: 13px; color: white;")
        header_layout.addWidget(name_label)
        
        header_layout.addStretch()
        
        # Kill button
        self.kill_button = QPushButton("🔴")
        self.kill_button.setFixedSize(30, 25)
        self.kill_button.setStyleSheet("""
            QPushButton {
                background-color: #cc0000;
                color: white;
                font-weight: bold;
                border: 1px solid #ff0000;
                border-radius: 3px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #ff0000;
            }
        """)
        self.kill_button.clicked.connect(self._emergency_stop)
        header_layout.addWidget(self.kill_button)
        
        layout.addLayout(header_layout)
        
        # Car name
        car_name_label = QLabel(f'"{self.car_name}"')
        car_name_label.setStyleSheet(f"color: {self.car_color}; font-weight: bold; font-size: 12px;")
        layout.addWidget(car_name_label)
        
        # Connection status indicators (more compact)
        conn_layout = QHBoxLayout()
        conn_layout.setSpacing(2)
        
        self.webrtc_indicator = self._create_compact_indicator("W", "🔴")
        self.bluetooth_indicator = self._create_compact_indicator("B", "🔴") 
        self.ros2_indicator = self._create_compact_indicator("R", "🔴")
        
        conn_layout.addWidget(self.webrtc_indicator)
        conn_layout.addWidget(self.bluetooth_indicator)
        conn_layout.addWidget(self.ros2_indicator)
        conn_layout.addStretch()
        
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
        
        # Action buttons (larger and more spaced)
        actions_layout = QHBoxLayout()
        actions_layout.setSpacing(3)
        
        self.manual_button = QPushButton("📱")
        self.manual_button.setFixedSize(35, 28)
        self.manual_button.setStyleSheet("""
            QPushButton {
                background-color: #0066cc;
                color: white;
                font-size: 10px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #0080ff;
            }
        """)
        self.manual_button.clicked.connect(self._toggle_manual_control)
        actions_layout.addWidget(self.manual_button)
        
        self.reset_button = QPushButton("🔄")
        self.reset_button.setFixedSize(35, 28)
        self.reset_button.setStyleSheet("""
            QPushButton {
                background-color: #009900;
                color: white;
                font-size: 10px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #00cc00;
            }
        """)
        self.reset_button.clicked.connect(self._reset_car)
        actions_layout.addWidget(self.reset_button)
        
        self.details_button = QPushButton("📊")
        self.details_button.setFixedSize(35, 28)
        self.details_button.setStyleSheet("""
            QPushButton {
                background-color: #666;
                color: white;
                font-size: 10px;
                border-radius: 3px;
            }
        """)
        actions_layout.addWidget(self.details_button)
        
        actions_layout.addStretch()
        layout.addLayout(actions_layout)
    
    def _create_compact_indicator(self, label, status):
        """Create a compact connection status indicator."""
        indicator = QLabel(f"{status}{label}")
        indicator.setStyleSheet("font-size: 9px; padding: 1px; color: #aaa;")
        indicator.setFixedWidth(25)
        return indicator
    
    def update_data(self):
        """Update this car section with latest data."""
        car_data = self.data_manager.get_car_data(self.car_id)
        if not car_data:
            return
        
        # Update connection indicators
        self._update_connection_indicator(
            self.webrtc_indicator, "W", car_data.webrtc_connected
        )
        self._update_connection_indicator(
            self.bluetooth_indicator, "B", car_data.bluetooth_connected
        )
        self._update_connection_indicator(
            self.ros2_indicator, "R", car_data.ros2_connected
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
    
    def _update_connection_indicator(self, indicator, label, connected):
        """Update a connection status indicator."""
        if connected:
            indicator.setText(f"🟢{label}")
            indicator.setStyleSheet("font-size: 9px; padding: 1px; color: #4caf50;")
        else:
            indicator.setText(f"🔴{label}")
            indicator.setStyleSheet("font-size: 9px; padding: 1px; color: #f44336;")
    
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
            self.manual_button.setText("⏹️")
        else:
            self.manual_button.setText("📱")
    
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
        """Reset car from emergency stop."""
        self.data_manager.reset_car(self.car_id)


class DualCarStatusWidget(QWidget):
    """Widget displaying status for two robot cars side-by-side."""
    
    def __init__(self, car_configs_pair, data_manager):
        super().__init__()
        self.car_configs = car_configs_pair
        self.data_manager = data_manager
        self.car_sections = {}
        
        self.setMinimumSize(500, 280)  # Larger minimum size for better readability
        self.setMaximumHeight(320)     # Prevent excessive growth
        self._init_ui()
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_data)
        self.update_timer.start(200)  # 5Hz updates
    
    def _init_ui(self):
        """Initialize the dual car status UI."""
        # Main border styling
        self.setStyleSheet("""
            DualCarStatusWidget {
                border: 2px solid #555;
                border-radius: 8px;
                background-color: #2b2b2b;
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