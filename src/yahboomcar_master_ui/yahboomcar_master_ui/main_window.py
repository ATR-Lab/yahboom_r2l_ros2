#!/usr/bin/env python3
"""
Main Window - Primary UI container for the master control center.
Organizes and manages all monitoring and control widgets.
"""

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QSplitter, QTabWidget, QLabel, QFrame, QPushButton
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPalette

from .car_status_widget import CarStatusWidget
from .system_status_widget import SystemStatusWidget
from .manual_control_widget import ManualControlWidget
from .game_state_widget import GameStateWidget
from .race_stats_widget import RaceStatsWidget
from .event_log_widget import EventLogWidget


class MasterControlWindow(QMainWindow):
    """Main window for the robot racing master control center."""
    
    def __init__(self, data_manager):
        super().__init__()
        self.data_manager = data_manager
        self.car_widgets = {}
        
        self.setWindowTitle("🏁 Robot Racing Master Control Center 🏁")
        self.setGeometry(100, 100, 1600, 1000)
        
        # Apply dark theme
        self.setStyleSheet(self._get_dark_theme_style())
        
        # Initialize UI components
        self._init_ui()
        
        # Set up data update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_displays)
        self.update_timer.start(100)  # 10Hz UI updates
        
    def _init_ui(self):
        """Initialize the main UI layout."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout with menu bar and content
        main_layout = QVBoxLayout(central_widget)
        
        # Top menu bar
        menu_bar = self._create_menu_bar()
        main_layout.addWidget(menu_bar)
        
        # Main content area with splitters
        content_splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(content_splitter)
        
        # Left column: System Status & Car Grid
        left_widget = self._create_left_panel()
        content_splitter.addWidget(left_widget)
        
        # Right column: Control & Game State
        right_widget = self._create_right_panel()
        content_splitter.addWidget(right_widget)
        
        # Bottom panel: Stats and Logs
        bottom_widget = self._create_bottom_panel()
        main_layout.addWidget(bottom_widget)
        
        # Set splitter proportions
        content_splitter.setSizes([1000, 600])
        
    def _create_menu_bar(self):
        """Create the top menu bar with emergency controls."""
        menu_frame = QFrame()
        menu_frame.setFixedHeight(50)
        menu_frame.setStyleSheet("background-color: #2b2b2b; border-bottom: 2px solid #555;")
        
        layout = QHBoxLayout(menu_frame)
        
        # Menu buttons
        buttons = ["System Overview", "Race Control", "Manual Control", "Settings", "Alerts"]
        for button_text in buttons:
            btn = QPushButton(button_text)
            btn.setStyleSheet("QPushButton { background-color: #404040; border: 1px solid #666; padding: 8px 16px; }")
            layout.addWidget(btn)
        
        layout.addStretch()
        
        # Emergency stop button
        emergency_btn = QPushButton("🚨 EMERGENCY ALL STOP")
        emergency_btn.setStyleSheet("""
            QPushButton {
                background-color: #cc0000;
                color: white;
                font-weight: bold;
                font-size: 14px;
                padding: 8px 20px;
                border: 2px solid #ff0000;
            }
            QPushButton:hover {
                background-color: #ff0000;
            }
        """)
        emergency_btn.clicked.connect(self._emergency_stop_all)
        layout.addWidget(emergency_btn)
        
        return menu_frame
    
    def _create_left_panel(self):
        """Create the left panel with system status and car grid."""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # System Status panel
        system_label = QLabel("📊 SYSTEM STATUS")
        system_label.setStyleSheet("font-weight: bold; font-size: 14px; padding: 5px;")
        layout.addWidget(system_label)
        
        self.system_status_widget = SystemStatusWidget(self.data_manager)
        layout.addWidget(self.system_status_widget)
        
        # Car Status Grid
        cars_label = QLabel("🚗 ROBOT CAR FLEET")
        cars_label.setStyleSheet("font-weight: bold; font-size: 14px; padding: 5px;")
        layout.addWidget(cars_label)
        
        # Grid for car status cards (2x2 for 4 cars)
        car_grid = QWidget()
        car_layout = QGridLayout(car_grid)
        
        # Create car status widgets
        car_configs = [
            {"id": 1, "name": "Lightning", "color": "#ff6b6b"},
            {"id": 2, "name": "Thunder", "color": "#4ecdc4"},
            {"id": 3, "name": "Storm", "color": "#45b7d1"},
            {"id": 4, "name": "Blitz", "color": "#f9ca24"}
        ]
        
        for i, config in enumerate(car_configs):
            car_widget = CarStatusWidget(config, self.data_manager)
            self.car_widgets[config["id"]] = car_widget
            row, col = divmod(i, 2)
            car_layout.addWidget(car_widget, row, col)
        
        layout.addWidget(car_grid)
        layout.addStretch()
        
        return widget
    
    def _create_right_panel(self):
        """Create the right panel with control and game state."""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Manual Control panel
        control_label = QLabel("🎮 MANUAL CONTROL")
        control_label.setStyleSheet("font-weight: bold; font-size: 14px; padding: 5px;")
        layout.addWidget(control_label)
        
        self.manual_control_widget = ManualControlWidget(self.data_manager)
        layout.addWidget(self.manual_control_widget)
        
        # Game State panel
        game_label = QLabel("🎲 GAME STATE")
        game_label.setStyleSheet("font-weight: bold; font-size: 14px; padding: 5px;")
        layout.addWidget(game_label)
        
        self.game_state_widget = GameStateWidget(self.data_manager)
        layout.addWidget(self.game_state_widget)
        
        layout.addStretch()
        
        return widget
    
    def _create_bottom_panel(self):
        """Create the bottom panel with stats and logs."""
        splitter = QSplitter(Qt.Horizontal)
        splitter.setFixedHeight(300)
        
        # Race Stats
        stats_widget = RaceStatsWidget(self.data_manager)
        splitter.addWidget(stats_widget)
        
        # Event Log
        log_widget = EventLogWidget(self.data_manager)
        splitter.addWidget(log_widget)
        
        # Set equal proportions
        splitter.setSizes([800, 800])
        
        return splitter
    
    def _update_displays(self):
        """Update all display widgets with latest data."""
        # This will be called at 10Hz to update UI elements
        # Individual widgets will pull data from the data_manager
        pass
    
    def _emergency_stop_all(self):
        """Emergency stop all cars."""
        self.data_manager.emergency_stop_all()
        # Show confirmation dialog or status update
    
    def _get_dark_theme_style(self):
        """Return dark theme stylesheet."""
        return """
        QMainWindow {
            background-color: #1e1e1e;
            color: #ffffff;
        }
        QWidget {
            background-color: #1e1e1e;
            color: #ffffff;
            font-family: 'Segoe UI', Arial, sans-serif;
        }
        QFrame {
            border: 1px solid #555;
            border-radius: 5px;
            background-color: #2b2b2b;
        }
        QLabel {
            color: #ffffff;
        }
        QPushButton {
            background-color: #404040;
            border: 1px solid #666;
            padding: 5px 10px;
            border-radius: 3px;
        }
        QPushButton:hover {
            background-color: #505050;
        }
        """