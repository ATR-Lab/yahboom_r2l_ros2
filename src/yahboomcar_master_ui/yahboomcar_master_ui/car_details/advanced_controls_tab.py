#!/usr/bin/env python3
"""
Advanced Controls Tab - Manual overrides and diagnostic tools.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QSlider,
    QSpinBox, QGroupBox, QFormLayout, QProgressBar, QFrame
)
from PyQt5.QtCore import Qt


class AdvancedControlsTab(QWidget):
    """Tab widget for advanced car controls and diagnostics."""
    
    def __init__(self, car_id, data_manager):
        super().__init__()
        self.car_id = car_id
        self.data_manager = data_manager
        self.pending_changes = {}
        self._init_ui()
    
    def _init_ui(self):
        """Initialize the advanced controls tab UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # Top row: Manual Motor Control
        motor_group = self._create_motor_control_group()
        layout.addWidget(motor_group)
        
        # Middle row: Diagnostic Tools
        diag_layout = QHBoxLayout()
        
        sensor_tests_group = self._create_sensor_tests_group()
        diag_layout.addWidget(sensor_tests_group)
        
        calibration_group = self._create_calibration_group()
        diag_layout.addWidget(calibration_group)
        
        layout.addLayout(diag_layout)
        
        # Bottom row: Maintenance Info
        maintenance_group = self._create_maintenance_group()
        layout.addWidget(maintenance_group)
    
    def _create_motor_control_group(self):
        """Create manual motor control interface."""
        group = QGroupBox("⚙️ Manual Motor Control")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QVBoxLayout(group)
        
        # Warning label
        warning_label = QLabel("⚠️ WARNING: Direct motor control bypasses safety systems")
        warning_label.setStyleSheet("color: #ff9800; font-weight: bold; font-size: 11px;")
        layout.addWidget(warning_label)
        
        # Motor controls
        motor_layout = QHBoxLayout()
        
        # Left motor
        left_frame = QFrame()
        left_frame.setStyleSheet("border: 1px solid #444; border-radius: 3px; padding: 8px;")
        left_layout = QVBoxLayout(left_frame)
        
        left_title = QLabel("🔧 Left Motor")
        left_title.setStyleSheet("font-weight: bold; font-size: 12px;")
        left_layout.addWidget(left_title)
        
        self.left_motor_slider = QSlider(Qt.Horizontal)
        self.left_motor_slider.setRange(-100, 100)
        self.left_motor_slider.setValue(0)
        self.left_motor_slider.valueChanged.connect(self._on_left_motor_changed)
        left_layout.addWidget(self.left_motor_slider)
        
        self.left_motor_label = QLabel("0% | Stop")
        self.left_motor_label.setStyleSheet("color: #ccc; font-size: 10px; text-align: center;")
        left_layout.addWidget(self.left_motor_label)
        
        motor_layout.addWidget(left_frame)
        
        # Right motor
        right_frame = QFrame()
        right_frame.setStyleSheet("border: 1px solid #444; border-radius: 3px; padding: 8px;")
        right_layout = QVBoxLayout(right_frame)
        
        right_title = QLabel("🔧 Right Motor")
        right_title.setStyleSheet("font-weight: bold; font-size: 12px;")
        right_layout.addWidget(right_title)
        
        self.right_motor_slider = QSlider(Qt.Horizontal)
        self.right_motor_slider.setRange(-100, 100)
        self.right_motor_slider.setValue(0)
        self.right_motor_slider.valueChanged.connect(self._on_right_motor_changed)
        right_layout.addWidget(self.right_motor_slider)
        
        self.right_motor_label = QLabel("0% | Stop")
        self.right_motor_label.setStyleSheet("color: #ccc; font-size: 10px; text-align: center;")
        right_layout.addWidget(self.right_motor_label)
        
        motor_layout.addWidget(right_frame)
        
        layout.addLayout(motor_layout)
        
        # Control buttons
        motor_buttons = QHBoxLayout()
        
        stop_motors_btn = QPushButton("🛑 Stop All Motors")
        stop_motors_btn.setStyleSheet("""
            QPushButton {
                background-color: #cc0000;
                color: white;
                font-weight: bold;
                padding: 8px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #ff0000;
            }
            QPushButton:pressed {
                background-color: #aa0000;
            }
        """)
        stop_motors_btn.clicked.connect(self._stop_all_motors)
        motor_buttons.addWidget(stop_motors_btn)
        
        motor_buttons.addStretch()
        
        layout.addLayout(motor_buttons)
        
        return group
    
    def _create_sensor_tests_group(self):
        """Create sensor testing interface."""
        group = QGroupBox("🔍 Sensor Tests")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QVBoxLayout(group)
        
        # Test buttons
        imu_test_btn = QPushButton("📐 Test IMU")
        imu_test_btn.setStyleSheet(self._get_button_style("#0066cc"))
        imu_test_btn.clicked.connect(self._test_imu)
        layout.addWidget(imu_test_btn)
        
        mag_test_btn = QPushButton("🧲 Test Magnetometer")
        mag_test_btn.setStyleSheet(self._get_button_style("#0066cc"))
        mag_test_btn.clicked.connect(self._test_magnetometer)
        layout.addWidget(mag_test_btn)
        
        encoder_test_btn = QPushButton("⚙️ Test Encoders")
        encoder_test_btn.setStyleSheet(self._get_button_style("#0066cc"))
        encoder_test_btn.clicked.connect(self._test_encoders)
        layout.addWidget(encoder_test_btn)
        
        # Test results
        self.test_results = QLabel("No tests run")
        self.test_results.setStyleSheet("color: #ccc; font-size: 11px; margin-top: 10px;")
        layout.addWidget(self.test_results)
        
        layout.addStretch()
        
        return group
    
    def _create_calibration_group(self):
        """Create calibration controls."""
        group = QGroupBox("🎯 Calibration")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QVBoxLayout(group)
        
        # Calibration buttons
        zero_imu_btn = QPushButton("📐 Zero IMU")
        zero_imu_btn.setStyleSheet(self._get_button_style("#ff9800"))
        zero_imu_btn.clicked.connect(self._zero_imu)
        layout.addWidget(zero_imu_btn)
        
        cal_motors_btn = QPushButton("⚙️ Calibrate Motors")
        cal_motors_btn.setStyleSheet(self._get_button_style("#ff9800"))
        cal_motors_btn.clicked.connect(self._calibrate_motors)
        layout.addWidget(cal_motors_btn)
        
        reset_cal_btn = QPushButton("🔄 Reset All Calibration")
        reset_cal_btn.setStyleSheet(self._get_button_style("#f44336"))
        reset_cal_btn.clicked.connect(self._reset_calibration)
        layout.addWidget(reset_cal_btn)
        
        # Calibration status
        self.cal_status = QLabel("Calibration: ✅ All sensors calibrated")
        self.cal_status.setStyleSheet("color: #4caf50; font-size: 11px; margin-top: 10px;")
        layout.addWidget(self.cal_status)
        
        layout.addStretch()
        
        return group
    
    def _create_maintenance_group(self):
        """Create maintenance information display."""
        group = QGroupBox("🔧 Maintenance Info")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QFormLayout(group)
        
        # Maintenance metrics
        self.runtime_hours = QLabel("142.5 hours")
        self.motor_cycles = QLabel("1,247,832 cycles")
        self.service_due = QLabel("Next service: 58 hours")
        
        layout.addRow("Total Runtime:", self.runtime_hours)
        layout.addRow("Motor Cycles:", self.motor_cycles)
        layout.addRow("Service Schedule:", self.service_due)
        
        # Style the labels
        for i in range(layout.rowCount()):
            label = layout.itemAt(i, QFormLayout.LabelRole).widget()
            if label:
                label.setStyleSheet("color: #ccc; font-size: 11px;")
            field = layout.itemAt(i, QFormLayout.FieldRole).widget()
            if field:
                field.setStyleSheet("color: white; font-size: 11px;")
        
        return group
    
    def _get_button_style(self, base_color):
        """Get consistent button styling with specified base color."""
        return f"""
            QPushButton {{
                background-color: {base_color};
                color: white;
                padding: 8px 12px;
                border-radius: 3px;
                font-size: 11px;
            }}
            QPushButton:hover {{
                background-color: {self._lighten_color(base_color)};
            }}
            QPushButton:pressed {{
                background-color: {self._darken_color(base_color)};
            }}
        """
    
    def _lighten_color(self, color):
        """Return a lighter version of the color for hover effect."""
        color_map = {
            "#0066cc": "#0080ff",
            "#ff9800": "#ffb74d", 
            "#f44336": "#ef5350"
        }
        return color_map.get(color, "#777")
    
    def _darken_color(self, color):
        """Return a darker version of the color for pressed effect."""
        color_map = {
            "#0066cc": "#004499",
            "#ff9800": "#f57c00",
            "#f44336": "#d32f2f"
        }
        return color_map.get(color, "#555")
    
    # Event handlers
    def _on_left_motor_changed(self, value):
        """Handle left motor slider change."""
        direction = "Forward" if value > 0 else "Reverse" if value < 0 else "Stop"
        self.left_motor_label.setText(f"{abs(value)}% | {direction}")
        # TODO: Send direct motor command
        
    def _on_right_motor_changed(self, value):
        """Handle right motor slider change."""
        direction = "Forward" if value > 0 else "Reverse" if value < 0 else "Stop"
        self.right_motor_label.setText(f"{abs(value)}% | {direction}")
        # TODO: Send direct motor command
    
    def _stop_all_motors(self):
        """Stop all motors immediately."""
        self.left_motor_slider.setValue(0)
        self.right_motor_slider.setValue(0)
        self.data_manager.log_user_action(f"Manual motor stop", f"Car #{self.car_id}")
    
    def _test_imu(self):
        """Run IMU sensor test."""
        self.test_results.setText("Running IMU test...")
        self.data_manager.log_user_action(f"IMU test initiated", f"Car #{self.car_id}")
        # TODO: Trigger actual sensor test
    
    def _test_magnetometer(self):
        """Run magnetometer test."""
        self.test_results.setText("Running magnetometer test...")
        self.data_manager.log_user_action(f"Magnetometer test initiated", f"Car #{self.car_id}")
        # TODO: Trigger actual sensor test
    
    def _test_encoders(self):
        """Run encoder test."""
        self.test_results.setText("Running encoder test...")
        self.data_manager.log_user_action(f"Encoder test initiated", f"Car #{self.car_id}")
        # TODO: Trigger actual sensor test
    
    def _zero_imu(self):
        """Zero the IMU calibration."""
        self.data_manager.log_user_action(f"IMU zeroed", f"Car #{self.car_id}")
        # TODO: Send IMU zero command
    
    def _calibrate_motors(self):
        """Start motor calibration."""
        self.data_manager.log_user_action(f"Motor calibration started", f"Car #{self.car_id}")
        # TODO: Send motor calibration command
    
    def _reset_calibration(self):
        """Reset all calibration to defaults."""
        self.data_manager.log_user_action(f"All calibration reset", f"Car #{self.car_id}")
        # TODO: Send calibration reset command
    
    def has_pending_changes(self):
        """Check if there are unsaved changes."""
        return len(self.pending_changes) > 0
    
    def apply_changes(self):
        """Apply any configuration changes."""
        # Advanced tab typically handles changes immediately
        self.pending_changes.clear()
    
    def update_data(self):
        """Update advanced controls tab with latest data."""
        car_data = self.data_manager.get_car_data(self.car_id)
        if not car_data:
            return
        
        # Update motor positions from current speed
        # Convert linear/angular velocity to approximate motor percentages
        left_motor_pct = int((car_data.speed_linear + car_data.speed_angular) * 50)
        right_motor_pct = int((car_data.speed_linear - car_data.speed_angular) * 50)
        
        # Only update sliders if user isn't actively dragging them
        if not self.left_motor_slider.isSliderDown():
            self.left_motor_slider.setValue(max(-100, min(100, left_motor_pct)))
        if not self.right_motor_slider.isSliderDown():
            self.right_motor_slider.setValue(max(-100, min(100, right_motor_pct)))
        
        # TODO: Update maintenance metrics from actual robot data