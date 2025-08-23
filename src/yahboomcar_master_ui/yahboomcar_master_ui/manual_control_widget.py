#!/usr/bin/env python3
"""
Manual Control Widget - Physical joystick control interface.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
    QSlider, QProgressBar, QGridLayout, QFrame
)
from PyQt5.QtCore import Qt, QTimer


class ManualControlWidget(QWidget):
    """Widget for manual joystick control."""
    
    def __init__(self, data_manager):
        super().__init__()
        self.data_manager = data_manager
        self.setMinimumHeight(200)
        self.setMaximumHeight(250)
        self._init_ui()
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_data)
        self.update_timer.start(100)  # 10Hz updates
    
    def _init_ui(self):
        """Initialize the manual control UI."""
        self.setStyleSheet("""
            QWidget {
                background-color: #2b2b2b;
                border: 1px solid #555;
                border-radius: 5px;
                padding: 5px;
            }
        """)
        
        layout = QVBoxLayout(self)
        
        # Joystick status
        self.joystick_status_label = QLabel("Joystick: 🔴 Disconnected")
        self.joystick_status_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.joystick_status_label)
        
        # Car selection grid
        selection_label = QLabel("SELECT TARGET CAR:")
        selection_label.setStyleSheet("font-size: 13px; font-weight: bold; margin-top: 10px;")
        layout.addWidget(selection_label)
        
        car_grid = QGridLayout()
        self.car_buttons = {}
        
        # Get dynamic car configuration from data manager
        # TODO: This will be updated from ROS2 /fleet/configuration topic
        car_config_data = self.data_manager.get_car_configuration()
        
        for i, car_config in enumerate(car_config_data):
            car_id = car_config['id']
            car_name = car_config['name']
            car_active = car_config['active']
            
            # Button text includes car number and name
            btn_text = f"Car{car_id}\n{car_name}"
            btn = QPushButton(btn_text)
            btn.setFixedSize(80, 35)  # Increased size for better text fit
            btn.setCheckable(True)
            btn.setEnabled(car_active)  # Disable if car is not active
            btn.clicked.connect(lambda checked, cid=car_id: self._select_car(cid))
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #404040;
                    border: 1px solid #666;
                    border-radius: 3px;
                    font-size: 9px;
                }
                QPushButton:checked {
                    background-color: #0066cc;
                    color: white;
                }
                QPushButton:disabled {
                    background-color: #2a2a2a;
                    color: #666;
                    border: 1px solid #444;
                }
            """)
            self.car_buttons[car_id] = btn
            car_grid.addWidget(btn, 0, i)
        
        layout.addLayout(car_grid)
        
        # Active control indicator
        self.active_label = QLabel("Active: None")
        self.active_label.setStyleSheet("color: #aaa; margin-top: 5px;")
        layout.addWidget(self.active_label)
        
        # Speed limit slider
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Speed Limit:"))
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(10, 100)
        self.speed_slider.setValue(40)
        speed_layout.addWidget(self.speed_slider)
        
        self.speed_label = QLabel("40%")
        speed_layout.addWidget(self.speed_label)
        
        self.speed_slider.valueChanged.connect(self._on_speed_limit_changed)
        
        layout.addLayout(speed_layout)
        
        # Control buttons
        control_layout = QHBoxLayout()
        
        self.emergency_stop_btn = QPushButton("🔴 EMERGENCY STOP")
        self.emergency_stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #cc0000;
                color: white;
                font-weight: bold;
                padding: 8px;
            }
        """)
        self.emergency_stop_btn.clicked.connect(self._emergency_stop_selected)
        control_layout.addWidget(self.emergency_stop_btn)
        
        self.release_btn = QPushButton("⏹️ RELEASE CONTROL")
        self.release_btn.setStyleSheet("""
            QPushButton {
                background-color: #666;
                color: white;
                padding: 8px;
            }
        """)
        self.release_btn.clicked.connect(self._release_control)
        control_layout.addWidget(self.release_btn)
        
        layout.addLayout(control_layout)
    
    def _select_car(self, car_id):
        """Select a car for manual control."""
        # Check if the car button is enabled (active)
        if not self.car_buttons[car_id].isEnabled():
            return  # Don't allow selection of inactive cars
        
        # Uncheck all other buttons
        for cid, btn in self.car_buttons.items():
            if cid != car_id:
                btn.setChecked(False)
        
        # Set manual control
        if self.car_buttons[car_id].isChecked():
            self.data_manager.set_manual_control(car_id, True)
            self.data_manager.log_user_action(f"Manual control activated", f"Car #{car_id}")
        else:
            self.data_manager.set_manual_control(car_id, False)
            self.data_manager.log_user_action(f"Manual control released", f"Car #{car_id}")
    
    def _emergency_stop_selected(self):
        """Emergency stop the selected car."""
        selected_car = self.data_manager.selected_car_id
        if selected_car:
            self.data_manager.emergency_stop(selected_car)
    
    def _release_control(self):
        """Release manual control."""
        selected_car = self.data_manager.selected_car_id
        if selected_car:
            self.data_manager.set_manual_control(selected_car, False)
            self.car_buttons[selected_car].setChecked(False)
    
    def _update_data(self):
        """Update the manual control display."""
        # Update joystick status
        if self.data_manager.is_joystick_connected():
            self.joystick_status_label.setText("Joystick: 🟢 Xbox Connected")
        else:
            self.joystick_status_label.setText("Joystick: 🔴 Disconnected")
        
        # Update active car display
        selected_car = self.data_manager.selected_car_id
        if selected_car:
            car_data = self.data_manager.get_car_data(selected_car)
            if car_data:
                self.active_label.setText(f"Active: Car #{selected_car} \"{car_data.name}\"")
        else:
            self.active_label.setText("Active: None")
    
    def _on_speed_limit_changed(self, value: int):
        """Handle speed limit slider change - TODO: Publish to ROS2"""
        self.speed_label.setText(f"{value}%")
        
        # Log the change and call dummy ROS2 function
        self.data_manager.log_user_action(f"Speed limit changed to {value}%")
        self.data_manager.set_global_speed_limit(value)
        
        # If a car is selected, also set individual car limit
        selected_car = self.data_manager.selected_car_id
        if selected_car:
            self.data_manager.publish_manual_speed_limit(selected_car, value)