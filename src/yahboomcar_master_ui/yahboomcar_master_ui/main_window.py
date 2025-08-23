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

from .dual_car_status_widget import DualCarStatusWidget
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
        
        # Set splitter proportions - more space for left panel with dual cards
        content_splitter.setSizes([800, 800])
        
    def _create_menu_bar(self):
        """Create the top menu bar with emergency controls."""
        menu_frame = QFrame()
        menu_frame.setFixedHeight(50)
        menu_frame.setStyleSheet("background-color: #2b2b2b; border-bottom: 2px solid #555;")
        
        layout = QHBoxLayout(menu_frame)
        
        # Menu buttons with dummy callbacks
        buttons = ["System Overview", "Race Control", "Manual Control", "Settings", "Alerts"]
        for button_text in buttons:
            btn = QPushButton(button_text)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #404040;
                    border: 1px solid #666;
                    padding: 8px 16px;
                    border-radius: 3px;
                }
                QPushButton:hover {
                    background-color: #505050;
                    border: 1px solid #777;
                }
                QPushButton:pressed {
                    background-color: #303030;
                    border: 1px solid #555;
                    transform: translateY(1px);
                }
            """)
            btn.clicked.connect(lambda checked, menu=button_text: self._on_menu_button_clicked(menu))
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
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #ff0000;
                border: 2px solid #ff3333;
            }
            QPushButton:pressed {
                background-color: #aa0000;
                border: 2px solid #cc0000;
                transform: translateY(1px);
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
        
        # Grid for dual car status cards (2 cards showing 4 cars)
        car_grid = QWidget()
        car_layout = QVBoxLayout(car_grid)
        car_layout.setSpacing(8)
        
        # Get dynamic car configurations from data manager
        # TODO: This will be updated from ROS2 /fleet/configuration topic
        car_configs = self.data_manager.get_car_configuration()
        
        # Create two dual car cards
        dual_card_1 = DualCarStatusWidget([car_configs[0], car_configs[1]], self.data_manager)
        dual_card_2 = DualCarStatusWidget([car_configs[2], car_configs[3]], self.data_manager)
        
        # Store references to dual cards for access
        self.dual_cards = [dual_card_1, dual_card_2]
        
        car_layout.addWidget(dual_card_1)
        car_layout.addWidget(dual_card_2)
        
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
        splitter.setMinimumHeight(200)
        splitter.setMaximumHeight(280)
        
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
        # Log the critical user action
        self.data_manager.log_user_action("EMERGENCY ALL STOP", "All cars emergency stopped")
        
        # Call existing emergency stop functionality
        self.data_manager.emergency_stop_all()
        # TODO: Show confirmation dialog or status update
    
    def _on_menu_button_clicked(self, menu_type: str):
        """Handle menu button clicks - TODO: Implement panel navigation"""
        # Log the user action
        self.data_manager.log_user_action(f"Menu: {menu_type} clicked")
        
        # TODO: Implement actual panel switching logic
        if menu_type == "Race Control":
            self._show_race_control_panel()
        elif menu_type == "Settings":
            self._show_settings_panel()
        elif menu_type == "Alerts":
            self._show_alerts_panel()
        else:
            print(f"DUMMY: Menu '{menu_type}' clicked - panel switching not implemented yet")
    
    def _show_race_control_panel(self):
        """Show race control panel - TODO: Implement race management UI"""
        print("DUMMY: Race Control Panel - Start/Stop/Reset race functionality")
        
        # Example of how race control might work:
        race_data = self.data_manager.get_race_data()
        if race_data['race_active']:
            print("Race is active - offering stop/pause options")
            # TODO: Show race control dialog with Stop/Pause/Reset buttons
        else:
            print("No active race - offering start race options")
            # TODO: Show start race dialog with configuration options
    
    def _show_settings_panel(self):
        """Show settings panel - TODO: Implement system configuration UI"""
        print("DUMMY: Settings Panel - System configuration options")
        
        # TODO: Show settings dialog with:
        # - Car configuration (names, colors, active status)
        # - Speed limits and safety parameters
        # - Network and connection settings
        # - Display and UI preferences
    
    def _show_alerts_panel(self):
        """Show alerts panel - TODO: Implement system alerts/warnings UI"""
        print("DUMMY: Alerts Panel - System warnings and notifications")
        
        # TODO: Show alerts panel with:
        # - Active system warnings
        # - Car-specific alerts
        # - Network/connectivity issues
        # - Battery warnings and maintenance alerts
    
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