#!/usr/bin/env python3
"""
Car Details Package - Car-specific detailed interfaces and dialogs.
"""

from .detail_dialog import CarDetailDialog
from .diagnostics_tab import DiagnosticsTab
from .config_tab import ConfigTab
from .ar_integration_tab import ARIntegrationTab
from .advanced_controls_tab import AdvancedControlsTab

__all__ = [
    'CarDetailDialog',
    'DiagnosticsTab', 
    'ConfigTab',
    'ARIntegrationTab',
    'AdvancedControlsTab'
]