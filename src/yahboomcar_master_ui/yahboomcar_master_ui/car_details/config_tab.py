#!/usr/bin/env python3
"""
Configuration Tab - Car-specific settings and parameters.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QSlider,
    QSpinBox, QDoubleSpinBox, QCheckBox, QGroupBox, QFormLayout
)
from PyQt5.QtCore import Qt


class ConfigTab(QWidget):
    """Tab widget for car configuration settings."""
    
    def __init__(self, car_id, data_manager):
        super().__init__()
        self.car_id = car_id
        self.data_manager = data_manager
        self.pending_changes = {}
        self._init_ui()
    
    def _init_ui(self):
        """Initialize the configuration tab UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # Speed and Control Settings
        speed_group = self._create_speed_group()
        layout.addWidget(speed_group)
        
        # Safety Parameters
        safety_group = self._create_safety_group()
        layout.addWidget(safety_group)
        
        # Control Sensitivity  
        control_group = self._create_control_group()
        layout.addWidget(control_group)
        
        layout.addStretch()
    
    def _create_speed_group(self):
        """Create speed and motion control settings."""
        group = QGroupBox("🏎️ Speed & Motion Control")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QFormLayout(group)
        
        # Max speed slider
        self.max_speed_slider = QSlider(Qt.Horizontal)
        self.max_speed_slider.setRange(10, 100)
        self.max_speed_slider.setValue(60)  # Default for this car
        self.max_speed_slider.valueChanged.connect(self._on_max_speed_changed)
        
        self.max_speed_label = QLabel("60%")
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(self.max_speed_slider)
        speed_layout.addWidget(self.max_speed_label)
        
        layout.addRow("Max Speed Limit:", speed_layout)
        
        # Acceleration limit
        self.accel_spin = QDoubleSpinBox()
        self.accel_spin.setRange(0.1, 5.0)
        self.accel_spin.setValue(2.0)
        self.accel_spin.setSuffix(" m/s²")
        self.accel_spin.valueChanged.connect(self._on_accel_changed)
        layout.addRow("Max Acceleration:", self.accel_spin)
        
        # Turning rate
        self.turn_rate_spin = QDoubleSpinBox()
        self.turn_rate_spin.setRange(0.5, 3.0)
        self.turn_rate_spin.setValue(1.5)
        self.turn_rate_spin.setSuffix(" rad/s")
        self.turn_rate_spin.valueChanged.connect(self._on_turn_rate_changed)
        layout.addRow("Max Turn Rate:", self.turn_rate_spin)
        
        return group
    
    def _create_safety_group(self):
        """Create safety parameter settings."""
        group = QGroupBox("🛡️ Safety Parameters")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QFormLayout(group)
        
        # Battery warning levels
        self.battery_warn_spin = QSpinBox()
        self.battery_warn_spin.setRange(10, 50)
        self.battery_warn_spin.setValue(25)
        self.battery_warn_spin.setSuffix("%")
        self.battery_warn_spin.valueChanged.connect(self._on_battery_warn_changed)
        layout.addRow("Battery Warning Level:", self.battery_warn_spin)
        
        # Connection timeout
        self.timeout_spin = QSpinBox()
        self.timeout_spin.setRange(1, 30)
        self.timeout_spin.setValue(5)
        self.timeout_spin.setSuffix(" seconds")
        self.timeout_spin.valueChanged.connect(self._on_timeout_changed)
        layout.addRow("Connection Timeout:", self.timeout_spin)
        
        # Emergency stop behavior
        self.auto_stop_check = QCheckBox("Auto-stop on connection loss")
        self.auto_stop_check.setChecked(True)
        self.auto_stop_check.toggled.connect(self._on_auto_stop_changed)
        layout.addRow("Emergency Behavior:", self.auto_stop_check)
        
        return group
    
    def _create_control_group(self):
        """Create control sensitivity settings."""
        group = QGroupBox("🎮 Control Sensitivity")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QFormLayout(group)
        
        # Joystick deadzone
        self.deadzone_slider = QSlider(Qt.Horizontal)
        self.deadzone_slider.setRange(0, 30)
        self.deadzone_slider.setValue(5)
        self.deadzone_slider.valueChanged.connect(self._on_deadzone_changed)
        
        self.deadzone_label = QLabel("5%")
        deadzone_layout = QHBoxLayout()
        deadzone_layout.addWidget(self.deadzone_slider)
        deadzone_layout.addWidget(self.deadzone_label)
        
        layout.addRow("Joystick Deadzone:", deadzone_layout)
        
        # Steering sensitivity
        self.steering_spin = QDoubleSpinBox()
        self.steering_spin.setRange(0.1, 2.0)
        self.steering_spin.setValue(1.0)
        self.steering_spin.setSingleStep(0.1)
        self.steering_spin.valueChanged.connect(self._on_steering_changed)
        layout.addRow("Steering Sensitivity:", self.steering_spin)
        
        # Response curve
        self.response_check = QCheckBox("Linear response (unchecked = curved)")
        self.response_check.setChecked(False)
        self.response_check.toggled.connect(self._on_response_changed)
        layout.addRow("Control Response:", self.response_check)
        
        return group
    
    # Event handlers for setting changes
    def _on_max_speed_changed(self, value):
        """Handle max speed change."""
        self.max_speed_label.setText(f"{value}%")
        self.pending_changes['max_speed'] = value
    
    def _on_accel_changed(self, value):
        """Handle acceleration limit change."""
        self.pending_changes['max_acceleration'] = value
    
    def _on_turn_rate_changed(self, value):
        """Handle turn rate change."""
        self.pending_changes['max_turn_rate'] = value
    
    def _on_battery_warn_changed(self, value):
        """Handle battery warning level change."""
        self.pending_changes['battery_warning'] = value
    
    def _on_timeout_changed(self, value):
        """Handle connection timeout change."""
        self.pending_changes['connection_timeout'] = value
    
    def _on_auto_stop_changed(self, checked):
        """Handle auto-stop behavior change."""
        self.pending_changes['auto_stop_on_disconnect'] = checked
    
    def _on_deadzone_changed(self, value):
        """Handle joystick deadzone change."""
        self.deadzone_label.setText(f"{value}%")
        self.pending_changes['joystick_deadzone'] = value
    
    def _on_steering_changed(self, value):
        """Handle steering sensitivity change."""
        self.pending_changes['steering_sensitivity'] = value
    
    def _on_response_changed(self, checked):
        """Handle response curve change."""
        self.pending_changes['linear_response'] = checked
    
    def has_pending_changes(self):
        """Check if there are unsaved changes."""
        return len(self.pending_changes) > 0
    
    def apply_changes(self):
        """Apply all pending configuration changes."""
        for param_name, value in self.pending_changes.items():
            # TODO: Send parameter changes via ROS2 parameter services
            self.data_manager.set_car_parameter(self.car_id, param_name, value)
        
        self.pending_changes.clear()
    
    def update_data(self):
        """Update configuration tab with current values."""
        # TODO: Refresh current parameter values from robot
        # This would query actual parameter values and update the UI controls
        pass