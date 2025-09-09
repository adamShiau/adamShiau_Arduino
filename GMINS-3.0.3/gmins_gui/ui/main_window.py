#!/usr/bin/env python3
"""
GMINS GUI Main Window

Main application window with navigation data display, 
real-time plotting, and system control capabilities.
"""

import sys
import time
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QPushButton, QComboBox, QLineEdit, QTextEdit,
    QSplitter, QTabWidget, QStatusBar, QMenuBar, QAction,
    QMessageBox, QProgressBar
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QPixmap, QPalette, QColor

# Import serial manager for real port detection
try:
    from ..communication.serial_manager import SerialManager
except ImportError:
    # Fallback for direct execution - use absolute import
    import sys
    import os
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
    from communication.serial_manager import SerialManager

# Import custom widgets (will create these next)
try:
    from .widgets.connection_panel import ConnectionPanel
    from .widgets.data_display import DataDisplayPanel  
    from .widgets.command_panel import CommandPanel
    from .widgets.plot_widget import PlotWidget
except ImportError:
    # Fallback for development - create placeholder widgets
    ConnectionPanel = None
    DataDisplayPanel = None
    CommandPanel = None
    PlotWidget = None


class MainWindow(QMainWindow):
    """Main application window"""
    
    # Signals for inter-widget communication
    data_received = pyqtSignal(dict)
    connection_status_changed = pyqtSignal(bool)
    
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("GMINS Navigation System - Real-time Monitor & Control")
        self.setGeometry(100, 100, 1400, 900)
        self.setMinimumSize(1000, 700)
        
        # Application state
        self.is_connected = False
        self.current_protocol = "Unknown"
        self.data_count = 0
        
        # Initialize serial manager for real port detection
        self.serial_manager = SerialManager()
        
        # Data buffers
        self.received_data_buffer = bytearray()
        self.last_data_log_time = 0
        
        # Initialize UI components
        self.init_ui()
        self.init_menu_bar()
        self.init_status_bar()
        
        # Setup update timer for real-time data - REDUCED FREQUENCY FOR STABILITY
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(2000)  # Update every 2 seconds (was 100ms) - STABILITY FIX
        
        # Setup data logging timer (every 5 seconds) - DISABLED BY DEFAULT FOR STABILITY
        self.data_log_timer = QTimer()
        self.data_log_timer.timeout.connect(self.log_received_data)
        # Timer is NOT started by default - must be manually enabled
        self.data_logging_enabled = False  # Flag to control logging
        
        # Connect serial manager signals
        self.serial_manager.data_received.connect(self.on_data_received)
        self.serial_manager.connection_changed.connect(self.on_connection_changed)
        self.serial_manager.error_occurred.connect(self.on_serial_error)
        
        # Apply custom stylesheet
        self.apply_styles()
        
        # Load available ports on startup - AFTER UI is fully initialized
        self.refresh_ports()
    
    def init_ui(self):
        """Initialize the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # SIMPLIFIED LAYOUT - Only basic connection controls
        main_layout = QVBoxLayout()
        
        # Only left panel - Connection and basic status
        left_panel = self.create_left_panel()
        main_layout.addWidget(left_panel)
        
        # Set main layout
        central_widget.setLayout(main_layout)
    
    def create_left_panel(self):
        """Create the simplified control panel - MINIMAL FUNCTIONALITY"""
        left_widget = QWidget()
        left_layout = QVBoxLayout()
        
        # ONLY Connection control section
        self.connection_panel = self.create_connection_group()
        left_layout.addWidget(self.connection_panel)
        
        # ONLY Basic data display section  
        self.data_display = self.create_simple_data_display_group()
        left_layout.addWidget(self.data_display)
        
        # REMOVED: Complex command panel that might interfere
        # REMOVED: Complex plotting panels
        # REMOVED: 3D attitude displays
        # REMOVED: Complex logging
        
        # Add stretch to push everything up
        left_layout.addStretch()
        
        left_widget.setLayout(left_layout)
        return left_widget
    
    # REMOVED: create_right_panel - Complex visualization panels removed for stability
    
    def create_connection_group(self):
        """Create connection control group"""
        group = QGroupBox("🔌 Connection Control")
        layout = QGridLayout()
        
        # COM Port selection
        layout.addWidget(QLabel("COM Port:"), 0, 0)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        layout.addWidget(self.port_combo, 0, 1)
        
        # Protocol selection
        layout.addWidget(QLabel("Protocol:"), 1, 0)
        self.protocol_combo = QComboBox()
        self.protocol_combo.addItems(["Auto-detect", "AR1AFC", "MAVLink"])
        layout.addWidget(self.protocol_combo, 1, 1)
        
        # Baud rate selection
        layout.addWidget(QLabel("Baud Rate:"), 2, 0)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["230400", "460800", "115200"])
        layout.addWidget(self.baud_combo, 2, 1)
        
        # Connect/Disconnect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_btn, 3, 0, 1, 2)
        
        # Connection status
        self.connection_status = QLabel("❌ Disconnected")
        layout.addWidget(self.connection_status, 4, 0, 1, 2)
        
        group.setLayout(layout)
        return group
    
    def create_simple_data_display_group(self):
        """Create simplified data display group - MINIMAL INFO ONLY"""
        group = QGroupBox("📊 Connection Status")
        layout = QGridLayout()
        
        # SIMPLIFIED: Only show basic connection info
        layout.addWidget(QLabel("Total Data:"), 0, 0)
        self.total_data_label = QLabel("0 bytes")
        layout.addWidget(self.total_data_label, 0, 1)
        
        layout.addWidget(QLabel("Data Rate:"), 1, 0)
        self.rate_label = QLabel("0 bytes/sec")
        layout.addWidget(self.rate_label, 1, 1)
        
        layout.addWidget(QLabel("Status:"), 2, 0)
        self.connection_status_label = QLabel("❌ Disconnected")
        layout.addWidget(self.connection_status_label, 2, 1)
        
        # Manual data preview button
        self.preview_btn = QPushButton("Show Data Sample")
        self.preview_btn.clicked.connect(self.show_data_sample)
        self.preview_btn.setEnabled(False)
        layout.addWidget(self.preview_btn, 3, 0, 1, 2)
        
        group.setLayout(layout)
        return group
    
    def create_data_display_group(self):
        """Create real-time data display group"""
        group = QGroupBox("📊 Navigation Data")
        layout = QGridLayout()
        
        # Attitude data
        layout.addWidget(QLabel("Roll:"), 0, 0)
        self.roll_label = QLabel("0.00°")
        layout.addWidget(self.roll_label, 0, 1)
        
        layout.addWidget(QLabel("Pitch:"), 1, 0)
        self.pitch_label = QLabel("0.00°")
        layout.addWidget(self.pitch_label, 1, 1)
        
        layout.addWidget(QLabel("Yaw:"), 2, 0)
        self.yaw_label = QLabel("0.00°")
        layout.addWidget(self.yaw_label, 2, 1)
        
        # Position data
        layout.addWidget(QLabel("Latitude:"), 3, 0)
        self.lat_label = QLabel("N/A")
        layout.addWidget(self.lat_label, 3, 1)
        
        layout.addWidget(QLabel("Longitude:"), 4, 0)
        self.lon_label = QLabel("N/A")
        layout.addWidget(self.lon_label, 4, 1)
        
        layout.addWidget(QLabel("Altitude:"), 5, 0)
        self.alt_label = QLabel("N/A")
        layout.addWidget(self.alt_label, 5, 1)
        
        # Data rate
        layout.addWidget(QLabel("Data Rate:"), 6, 0)
        self.rate_label = QLabel("0 Hz")
        layout.addWidget(self.rate_label, 6, 1)
        
        group.setLayout(layout)
        return group
    
    def create_command_group(self):
        """Create command control group"""
        group = QGroupBox("🎮 System Control")
        layout = QVBoxLayout()
        
        # Quick command buttons
        button_layout = QGridLayout()
        
        self.ar1afc_btn = QPushButton("AR1AFC Mode")
        self.ar1afc_btn.clicked.connect(lambda: self.send_command("AR1AFC"))
        button_layout.addWidget(self.ar1afc_btn, 0, 0)
        
        self.mavlink_btn = QPushButton("MAVLink Mode") 
        self.mavlink_btn.clicked.connect(lambda: self.send_command("MAVLINK"))
        button_layout.addWidget(self.mavlink_btn, 0, 1)
        
        self.status_btn = QPushButton("System Status")
        self.status_btn.clicked.connect(lambda: self.send_command("STATUS"))
        button_layout.addWidget(self.status_btn, 1, 0)
        
        self.help_btn = QPushButton("Help")
        self.help_btn.clicked.connect(lambda: self.send_command("HELP"))
        button_layout.addWidget(self.help_btn, 1, 1)
        
        layout.addLayout(button_layout)
        
        # Data logging control
        logging_layout = QHBoxLayout()
        self.logging_btn = QPushButton("Enable Data Logging")
        self.logging_btn.clicked.connect(self.toggle_data_logging)
        self.logging_btn.setStyleSheet("background-color: #ff6b6b; color: white;")  # Red when disabled
        logging_layout.addWidget(self.logging_btn)
        layout.addLayout(logging_layout)
        
        # Custom command input
        layout.addWidget(QLabel("Custom Command:"))
        self.command_input = QLineEdit()
        self.command_input.returnPressed.connect(self.send_custom_command)
        layout.addWidget(self.command_input)
        
        send_cmd_btn = QPushButton("Send Command")
        send_cmd_btn.clicked.connect(self.send_custom_command)
        layout.addWidget(send_cmd_btn)
        
        group.setLayout(layout)
        return group
    
    # REMOVED: create_plots_tab - Complex plotting removed for stability
    # REMOVED: create_attitude_tab - 3D attitude display removed for stability 
    # REMOVED: create_log_tab - Complex logging removed for stability
    
    def init_menu_bar(self):
        """Initialize minimal menu bar"""
        menubar = self.menuBar()
        
        # SIMPLIFIED: Only essential menu items
        # File menu
        file_menu = menubar.addMenu('&File')
        exit_action = QAction('E&xit', self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # View menu - only port refresh
        view_menu = menubar.addMenu('&View')
        refresh_action = QAction('&Refresh Ports', self)
        refresh_action.triggered.connect(self.refresh_ports)
        view_menu.addAction(refresh_action)
        
        # REMOVED: Complex save/help functions that might interfere
    
    def init_status_bar(self):
        """Initialize simplified status bar"""
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        
        # SIMPLIFIED: Only essential status info
        self.connection_indicator = QLabel("❌ Disconnected")
        self.status_bar.addPermanentWidget(self.connection_indicator)
        
        self.status_bar.showMessage("Ready - Minimal Mode")
    
    def apply_styles(self):
        """Apply custom stylesheet"""
        style = '''
            QMainWindow {
                background-color: #f5f5f5;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #cccccc;
                border-radius: 5px;
                margin-top: 1ex;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
            QComboBox, QLineEdit {
                padding: 5px;
                border: 1px solid #ccc;
                border-radius: 3px;
            }
            QLabel {
                color: #333;
            }
        '''
        self.setStyleSheet(style)
    
    # Event handlers and utility methods
    def toggle_connection(self):
        """Toggle connection to GMINS system"""
        if not self.is_connected:
            self.connect_to_system()
        else:
            self.disconnect_from_system()
    
    def connect_to_system(self):
        """Connect to GMINS system"""
        # Get actual port name from combo box data
        if self.port_combo.count() == 0:
            QMessageBox.warning(self, "Warning", "No COM ports available!")
            return
            
        port = self.port_combo.currentData()  # Get actual device name
        display_text = self.port_combo.currentText()
        protocol = self.protocol_combo.currentText()
        baud_rate = self.baud_combo.currentText()
        
        if not port:
            QMessageBox.warning(self, "Warning", "Please select a valid COM port!")
            return
        
        self.log_message(f"🔗 Connecting to {port} at {baud_rate} baud ({protocol} mode)...")
        
        # Convert protocol to SerialManager enum
        from ..communication.serial_manager import ProtocolMode
        try:
            from ..communication.serial_manager import ProtocolMode
        except ImportError:
            from communication.serial_manager import ProtocolMode
            
        if protocol == "AR1AFC":
            protocol_mode = ProtocolMode.AR1AFC
        elif protocol == "MAVLink":
            protocol_mode = ProtocolMode.MAVLINK
        else:
            protocol_mode = ProtocolMode.AUTO_DETECT
        
        # Attempt real connection using SerialManager
        success = self.serial_manager.connect(port, protocol_mode)
        
        if success:
            self.current_protocol = protocol
            self.log_message(f"✅ Connected successfully to {display_text}!")
            self.status_bar.showMessage(f"Connected to {port}")
            
            # Clear data buffer when connecting
            self.received_data_buffer.clear()
        else:
            self.log_message(f"❌ Failed to connect to {port}")
            QMessageBox.critical(self, "Connection Error", f"Failed to connect to {port}")
            self.status_bar.showMessage("Connection failed")
    
    def disconnect_from_system(self):
        """Disconnect from GMINS system"""
        self.log_message("📡 Disconnecting...")
        
        # Use SerialManager to disconnect
        self.serial_manager.disconnect()
        
        # Reset protocol
        self.current_protocol = "Unknown"
        
        # Update UI (will be handled by on_connection_changed signal)
        self.protocol_status.setText("Protocol: Unknown")
        self.data_rate_status.setText("Rate: 0 Hz")
        
        self.status_bar.showMessage("Disconnected")
    
    def send_command(self, command):
        """Send command to GMINS system"""
        if not self.is_connected:
            QMessageBox.warning(self, "Warning", "Please connect to system first!")
            return
        
        self.log_message(f"📤 Sending command: {command}")
        
        # Use SerialManager to send command
        success = self.serial_manager.send_command(command)
        
        if success:
            self.log_message(f"✅ Command '{command}' sent successfully")
            
            # Update protocol if this is a protocol switch command
            if command in ["AR1AFC", "MAVLINK"]:
                self.current_protocol = command
                self.protocol_status.setText(f"Protocol: {command}")
                self.log_message(f"🔄 Protocol switched to {command}")
        else:
            self.log_message(f"❌ Failed to send command: {command}")
            QMessageBox.warning(self, "Command Error", f"Failed to send command: {command}")
    
    def send_custom_command(self):
        """Send custom command"""
        command = self.command_input.text().strip()
        if command:
            self.send_command(command)
            self.command_input.clear()
    
    def update_display(self):
        """Update simplified display (called by timer) - MINIMAL OPERATIONS ONLY"""
        if self.is_connected:
            # MINIMAL: Only update essential connection info
            total_bytes = self.serial_manager.total_bytes if hasattr(self.serial_manager, 'total_bytes') else len(self.received_data_buffer)
            
            # Update simplified displays
            self.total_data_label.setText(f"{total_bytes} bytes")
            
            # Calculate data rate (minimal calculation)
            if hasattr(self.serial_manager, 'connection_time') and self.serial_manager.connection_time > 0:
                elapsed = time.time() - self.serial_manager.connection_time
                if elapsed > 0:
                    rate = int(total_bytes / elapsed)
                    self.rate_label.setText(f"{rate} bytes/sec")
                    
            # REMOVED: All complex attitude calculations
            # REMOVED: All complex plotting updates
            # REMOVED: All complex data parsing
    
    def show_data_sample(self):
        """Show a sample of received data - MANUAL ONLY"""
        if not self.is_connected:
            QMessageBox.information(self, "Info", "Not connected to device")
            return
            
        if len(self.received_data_buffer) > 0:
            # Show first 32 bytes
            sample_size = min(32, len(self.received_data_buffer))
            hex_data = ' '.join([f'{b:02X}' for b in self.received_data_buffer[:sample_size]])
            if len(self.received_data_buffer) > sample_size:
                hex_data += f" ... (+{len(self.received_data_buffer)-sample_size} more bytes)"
                
            total_bytes = len(self.received_data_buffer)
            QMessageBox.information(
                self, 
                "Data Sample", 
                f"Latest data ({total_bytes} bytes in buffer):\n\n{hex_data}"
            )
        else:
            QMessageBox.information(self, "Info", "No data received yet")
    
    # REMOVED: Complex log_message function - replaced with simple message boxes
    
    def toggle_data_logging(self):
        """Toggle data logging on/off"""
        self.data_logging_enabled = not self.data_logging_enabled
        
        if self.data_logging_enabled:
            # Enable logging
            self.data_log_timer.start(5000)
            self.logging_btn.setText("Disable Data Logging")
            self.logging_btn.setStyleSheet("background-color: #4CAF50; color: white;")  # Green when enabled
            self.log_message("✅ Data logging enabled - will print every 5 seconds")
        else:
            # Disable logging
            self.data_log_timer.stop()
            self.logging_btn.setText("Enable Data Logging")
            self.logging_btn.setStyleSheet("background-color: #ff6b6b; color: white;")  # Red when disabled
            self.log_message("🔇 Data logging disabled - no automatic printing")
    
    def on_data_received(self, data):
        """Handle received serial data - OPTIMIZED FOR STABILITY"""
        if data:
            self.received_data_buffer.extend(data)
            self.data_count += len(data)
            # CRITICAL: No automatic logging - this was causing connection drops
            # Data is collected silently for stability
    
    def on_connection_changed(self, connected, message):
        """Handle connection status changes - SIMPLIFIED"""
        self.is_connected = connected
        if connected:
            self.connect_btn.setText("Disconnect")
            self.connection_status.setText("✅ Connected")
            self.connection_indicator.setText("✅ Connected")
            self.connection_status_label.setText("✅ Connected")
            self.preview_btn.setEnabled(True)
        else:
            self.connect_btn.setText("Connect")
            self.connection_status.setText("❌ Disconnected")
            self.connection_indicator.setText("❌ Disconnected")
            self.connection_status_label.setText("❌ Disconnected")
            self.preview_btn.setEnabled(False)
        
        # REMOVED: Complex logging - just update status bar
        self.status_bar.showMessage(f"Status: {message}")
    
    def on_serial_error(self, error_msg):
        """Handle serial errors - SIMPLIFIED"""
        # REMOVED: Complex logging - just show in status bar
        self.status_bar.showMessage(f"Error: {error_msg}")
    
    def log_received_data(self):
        """Log received data every 5 seconds in hex format - TEMPORARILY DISABLED"""
        import time
        
        if not self.is_connected or not self.data_logging_enabled:
            return
            
        current_time = time.time()
        
        # Check if we have new data to log
        if len(self.received_data_buffer) > 0:
            # Format data as hex string like: FE 81 FF 55 87 27 AC...
            hex_data = ' '.join([f'{byte:02X}' for byte in self.received_data_buffer])
            
            # Log the data
            self.log_message(f"📊 Data ({len(self.received_data_buffer)} bytes): {hex_data}")
            
            # Clear the buffer after logging
            self.received_data_buffer.clear()
        else:
            # Log that no data was received in this interval
            if self.is_connected:
                self.log_message("📊 No data received in last 5 seconds")
        
        self.last_data_log_time = current_time
    
    def refresh_ports(self):
        """Refresh available COM ports - SIMPLIFIED"""        
        try:
            # Get real available ports from serial manager
            available_ports = self.serial_manager.get_available_ports()
            
            # Clear current items
            self.port_combo.clear()
            
            # Add detected ports with descriptions
            for port_info in available_ports:
                device = port_info['device']
                description = port_info['description']
                display_text = f"{device} - {description}"
                self.port_combo.addItem(display_text, device)
            
            # If no ports found, add a message
            if not available_ports:
                self.port_combo.addItem("No COM ports detected", "")
            
            # REMOVED: Complex logging - just show in status bar
            self.status_bar.showMessage(f"Found {len(available_ports)} ports")
                    
        except Exception as e:
            # REMOVED: Complex error logging
            self.status_bar.showMessage(f"Port scan error: {e}")
            # Fallback to common port names
            fallback_ports = ["COM3", "COM4", "COM5", "/dev/ttyUSB0", "/dev/ttyACM0"]
            self.port_combo.clear()
            for port in fallback_ports:
                self.port_combo.addItem(port, port)
    
    # REMOVED: save_data - Complex file operations removed for stability
    # REMOVED: show_about - Complex dialogs removed for stability
    
    def closeEvent(self, event):
        """Handle application close event - SIMPLIFIED"""
        # SIMPLIFIED: Always disconnect and close
        if self.is_connected:
            self.disconnect_from_system()
        event.accept()