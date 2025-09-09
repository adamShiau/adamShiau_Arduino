#!/usr/bin/env python3
"""
Simple GMINS Connection Test GUI
- Connect to serial port
- Show connection status
- Confirm data is coming in
"""

import sys
import serial
import serial.tools.list_ports
import time
import threading
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QTextEdit, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont


class SimpleSerialManager(QObject):
    """Minimal serial manager"""
    data_received = pyqtSignal(bytes)
    connection_changed = pyqtSignal(bool, str)
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.is_connected = False
        self.read_thread = None
        self.stop_reading = threading.Event()
        self.total_bytes = 0
        
    def get_ports(self):
        """Get available ports"""
        ports = []
        try:
            for port in serial.tools.list_ports.comports():
                ports.append(port.device)
        except:
            pass
        return ports
    
    def connect(self, port_name, baud_rate=230400):
        """Connect to port"""
        try:
            if self.is_connected:
                self.disconnect()
                
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=baud_rate,
                timeout=1
            )
            
            self.is_connected = True
            self.total_bytes = 0
            
            # Start reading thread
            self.stop_reading.clear()
            self.read_thread = threading.Thread(target=self._read_data, daemon=True)
            self.read_thread.start()
            
            self.connection_changed.emit(True, f"Connected to {port_name}")
            return True
            
        except Exception as e:
            self.connection_changed.emit(False, f"Failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect"""
        self.stop_reading.set()
        if self.serial_port:
            try:
                self.serial_port.close()
            except:
                pass
        self.serial_port = None
        self.is_connected = False
        self.connection_changed.emit(False, "Disconnected")
    
    def _read_data(self):
        """Read data in background"""
        while not self.stop_reading.is_set():
            if self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting > 0:
                        data = self.serial_port.read(self.serial_port.in_waiting)
                        if data:
                            self.total_bytes += len(data)
                            self.data_received.emit(data)
                except Exception:
                    # Silent error handling to avoid GUI interference
                    break
            time.sleep(0.05)  # Increased sleep to reduce CPU usage and GUI interference


class SimpleGUI(QMainWindow):
    """Simple GUI window"""
    
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("Simple GMINS Connection Test")
        self.setGeometry(200, 200, 600, 400)
        
        # Serial manager
        self.serial_manager = SimpleSerialManager()
        
        # Data tracking
        self.data_count = 0
        self.last_data_time = 0
        self.latest_data = None  # Keep latest data for manual preview
        
        # Initialize UI first
        self.init_ui()
        
        # Then connect signals (after UI is ready)
        self.serial_manager.connection_changed.connect(self.on_connection_changed)
        self.serial_manager.data_received.connect(self.on_data_received)
        
        # Status update timer - REDUCED FREQUENCY FOR STABILITY
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(2000)  # Update every 2 seconds (was 1 second)
    
    def init_ui(self):
        """Initialize UI"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()
        
        # Title
        title = QLabel("🚀 Simple GMINS Connection Test")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Connection controls
        conn_layout = QHBoxLayout()
        
        conn_layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        # Don't refresh ports here - do it after UI is complete
        conn_layout.addWidget(self.port_combo)
        
        conn_layout.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["230400", "115200", "460800"])
        conn_layout.addWidget(self.baud_combo)
        
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.connect_btn)
        
        refresh_btn = QPushButton("Refresh Ports")
        refresh_btn.clicked.connect(self.refresh_ports)
        conn_layout.addWidget(refresh_btn)
        
        layout.addLayout(conn_layout)
        
        # Manual data preview button
        preview_layout = QHBoxLayout()
        self.preview_btn = QPushButton("Show Data Sample")
        self.preview_btn.clicked.connect(self.show_data_sample)
        self.preview_btn.setEnabled(False)
        preview_layout.addWidget(self.preview_btn)
        layout.addLayout(preview_layout)
        
        # Status display
        self.status_label = QLabel("❌ Disconnected")
        self.status_label.setFont(QFont("Arial", 12))
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)
        
        # Data statistics
        stats_layout = QVBoxLayout()
        self.data_stats = QLabel("📊 No data received")
        self.data_stats.setFont(QFont("Arial", 11))
        stats_layout.addWidget(self.data_stats)
        
        self.rate_stats = QLabel("📈 Data rate: 0 bytes/sec")
        self.rate_stats.setFont(QFont("Arial", 11))
        stats_layout.addWidget(self.rate_stats)
        
        layout.addLayout(stats_layout)
        
        # Simple log
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(200)
        self.log_text.setFont(QFont("Courier", 9))
        layout.addWidget(QLabel("📝 Connection Log:"))
        layout.addWidget(self.log_text)
        
        central_widget.setLayout(layout)
        
        # Initial log
        self.add_log("🚀 Simple GUI started - ready to connect")
        
        # Now refresh ports after UI is complete
        self.refresh_ports()
    
    def refresh_ports(self):
        """Refresh port list"""
        self.port_combo.clear()
        ports = self.serial_manager.get_ports()
        if ports:
            self.port_combo.addItems(ports)
            self.add_log(f"🔍 Found {len(ports)} ports: {', '.join(ports)}")
        else:
            self.port_combo.addItem("No ports found")
            self.add_log("⚠️ No COM ports detected")
    
    def toggle_connection(self):
        """Toggle connection"""
        if not self.serial_manager.is_connected:
            port = self.port_combo.currentText()
            baud = int(self.baud_combo.currentText())
            
            if port == "No ports found":
                QMessageBox.warning(self, "Error", "No valid port selected!")
                return
                
            self.add_log(f"🔗 Connecting to {port} at {baud} baud...")
            self.serial_manager.connect(port, baud)
        else:
            self.serial_manager.disconnect()
    
    def on_connection_changed(self, connected, message):
        """Handle connection changes"""
        if connected:
            self.status_label.setText("✅ Connected")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self.connect_btn.setText("Disconnect")
            self.preview_btn.setEnabled(True)
            self.data_count = 0
            self.last_data_time = time.time()
        else:
            self.status_label.setText("❌ Disconnected")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            self.connect_btn.setText("Connect")
            self.preview_btn.setEnabled(False)
        
        self.add_log(f"🔗 {message}")
    
    def on_data_received(self, data):
        """Handle received data - OPTIMIZED FOR STABILITY"""
        self.data_count += len(data)
        self.last_data_time = time.time()
        
        # Store latest data for manual preview (no automatic logging)
        self.latest_data = data
        
        # DO NOT LOG EVERY DATA PACKET - this causes connection instability
        # Data is available for manual viewing via button
    
    def update_status(self):
        """Update status display"""
        if self.serial_manager.is_connected:
            # Update data statistics
            total = self.serial_manager.total_bytes
            self.data_stats.setText(f"📊 Total received: {total} bytes")
            
            # Calculate rate
            if self.last_data_time > 0:
                elapsed = time.time() - (self.last_data_time - 1)  # Rough calculation
                if elapsed > 0:
                    rate = int(self.data_count / max(elapsed, 1))
                    self.rate_stats.setText(f"📈 Data rate: ~{rate} bytes/sec")
        else:
            self.data_stats.setText("📊 No data received")
            self.rate_stats.setText("📈 Data rate: 0 bytes/sec")
    
    def show_data_sample(self):
        """Manually show a sample of the latest received data"""
        if not self.serial_manager.is_connected:
            self.add_log("⚠️ Not connected")
            return
        
        if self.latest_data:
            # Show first 32 bytes of latest data
            sample_size = min(32, len(self.latest_data))
            hex_data = ' '.join([f'{b:02X}' for b in self.latest_data[:sample_size]])
            if len(self.latest_data) > sample_size:
                hex_data += f" ... (+{len(self.latest_data)-sample_size} more bytes)"
            
            total_bytes = self.serial_manager.total_bytes
            self.add_log(f"📊 Latest packet ({len(self.latest_data)} bytes): {hex_data}")
            self.add_log(f"📈 Total received: {total_bytes} bytes")
        else:
            self.add_log("⚠️ No data received yet")
    
    def add_log(self, message):
        """Add message to log"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        # Auto scroll
        self.log_text.moveCursor(self.log_text.textCursor().End)
    
    def closeEvent(self, event):
        """Handle close"""
        if self.serial_manager.is_connected:
            self.serial_manager.disconnect()
        event.accept()


def main():
    app = QApplication(sys.argv)
    
    try:
        window = SimpleGUI()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        print(f"Error: {e}")
        QMessageBox.critical(None, "Error", f"Failed to start GUI: {e}")


if __name__ == "__main__":
    main()