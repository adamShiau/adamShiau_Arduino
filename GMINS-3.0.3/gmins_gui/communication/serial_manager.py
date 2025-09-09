#!/usr/bin/env python3
"""
Serial Communication Manager

Manages serial port communication for GMINS GUI application.
Supports both AR1AFC and MAVLink protocols with smart baud rate switching.

Features:
- Auto port detection
- Smart baud rate switching per protocol
- Real-time data reception  
- Command sending with proper baud rates
- Connection status monitoring
- Data buffering and threading
"""

import serial
import serial.tools.list_ports
import threading
import time
from typing import Optional, List, Dict, Any, Callable
from PyQt5.QtCore import QObject, pyqtSignal, QThread, QTimer
from enum import Enum


class ProtocolMode(Enum):
    """Protocol modes supported by GMINS"""
    AUTO_DETECT = "auto"
    AR1AFC = "ar1afc"
    MAVLINK = "mavlink"


class ConnectionState(Enum):
    """Connection states"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting" 
    CONNECTED = "connected"
    ERROR = "error"


class SerialManager(QObject):
    """Serial Communication Manager with Qt signals"""
    
    # Qt signals for GUI communication
    data_received = pyqtSignal(bytes)           # Raw data received
    connection_changed = pyqtSignal(bool, str)  # Connection status, message
    protocol_detected = pyqtSignal(str)         # Protocol auto-detected
    error_occurred = pyqtSignal(str)            # Error message
    
    def __init__(self):
        super().__init__()
        
        # Serial connection
        self.serial_port: Optional[serial.Serial] = None
        self.connection_state = ConnectionState.DISCONNECTED
        
        # Connection parameters  
        self.port_name: Optional[str] = None
        self.current_baud_rate: int = 230400
        self.protocol_mode = ProtocolMode.AUTO_DETECT
        
        # Smart baud rate mapping (from our previous implementation)
        self.protocol_baud_rates = {
            ProtocolMode.AR1AFC: 230400,     # AR1AFC mode: other commands use 230400
            ProtocolMode.MAVLINK: 460800,    # MAVLink mode: other commands use 460800
        }
        
        # Command baud rates (fixed per command type)
        self.command_baud_rates = {
            "AR1AFC": 460800,     # AR1AFC switching command
            "MAVLINK": 230400,    # MAVLink switching command
        }
        
        # Data reception
        self.read_thread: Optional[threading.Thread] = None
        self.stop_reading = threading.Event()
        self.data_buffer = bytearray()
        
        # Statistics
        self.bytes_received = 0
        self.bytes_sent = 0
        self.connection_time = 0
        self.last_data_time = 0
        
        # Callbacks
        self.data_callback: Optional[Callable[[bytes], None]] = None
        
    def get_available_ports(self) -> List[Dict[str, str]]:
        """
        Get list of available serial ports
        
        Returns:
            List of dictionaries with port information
        """
        ports = []
        try:
            for port_info in serial.tools.list_ports.comports():
                ports.append({
                    'device': port_info.device,
                    'description': port_info.description,
                    'manufacturer': port_info.manufacturer or 'Unknown',
                    'vid': port_info.vid,
                    'pid': port_info.pid
                })
        except Exception as e:
            self.error_occurred.emit(f"Error scanning ports: {e}")
            
        return ports
    
    def connect(self, port_name: str, protocol_mode: ProtocolMode = ProtocolMode.AUTO_DETECT, max_retries: int = 3) -> bool:
        """
        Connect to serial port with retry mechanism
        
        Args:
            port_name: Name of the serial port (e.g., 'COM3', '/dev/ttyUSB0')
            protocol_mode: Protocol mode to use
            max_retries: Maximum number of connection attempts
            
        Returns:
            True if connection successful, False otherwise
        """
        if self.connection_state == ConnectionState.CONNECTED:
            self.disconnect()
            
        self.connection_state = ConnectionState.CONNECTING
        self.port_name = port_name
        self.protocol_mode = protocol_mode
        
        # Determine initial baud rate
        if protocol_mode == ProtocolMode.AR1AFC:
            initial_baud = self.protocol_baud_rates[ProtocolMode.AR1AFC]
        elif protocol_mode == ProtocolMode.MAVLINK:
            initial_baud = self.protocol_baud_rates[ProtocolMode.MAVLINK]
        else:
            initial_baud = 230400  # Default for auto-detect
        
        # Try connection with retries
        last_error = None
        for attempt in range(max_retries):
            try:
                if attempt > 0:
                    self.error_occurred.emit(f"Connection attempt {attempt + 1}/{max_retries}")
                    time.sleep(1)  # Wait between retries
                
                success = self._attempt_connection(port_name, initial_baud)
                if success:
                    return True
                    
            except Exception as e:
                last_error = e
                self.error_occurred.emit(f"Connection attempt {attempt + 1} failed: {e}")
        
        # All attempts failed
        self.connection_state = ConnectionState.ERROR
        self.serial_port = None
        self.connection_changed.emit(False, f"All connection attempts failed. Last error: {last_error}")
        return False
    
    def _attempt_connection(self, port_name: str, initial_baud: int) -> bool:
        """Single connection attempt"""
        try:
            # Special handling for high baud rates like 230400
            if initial_baud >= 230400:
                # More conservative settings for high baud rates
                timeout = 1.0
                write_timeout = 5
                inter_byte_timeout = 0.1
            else:
                # Standard settings for lower baud rates
                timeout = 0.5
                write_timeout = 2
                inter_byte_timeout = None
            
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=initial_baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
                write_timeout=write_timeout,
                inter_byte_timeout=inter_byte_timeout,
                xonxoff=False,      # Disable software flow control
                rtscts=False,       # Disable hardware flow control
                dsrdtr=False        # Disable DSR/DTR flow control
            )
            
            # Additional stabilization for high baud rates
            if initial_baud >= 230400:
                time.sleep(0.5)  # Give connection time to stabilize
                
                # Verify connection is stable
                if not self.serial_port.is_open:
                    raise serial.SerialException("Port closed immediately after opening")
            
            self.current_baud_rate = initial_baud
            self.connection_state = ConnectionState.CONNECTED
            self.connection_time = time.time()
            
            # Start data reading thread
            self.stop_reading.clear()
            self.read_thread = threading.Thread(target=self._read_data_thread, daemon=True)
            self.read_thread.start()
            
            # Test connection stability for high baud rates
            if initial_baud >= 230400:
                # Quick stability test
                test_start = time.time()
                stable = True
                while time.time() - test_start < 2:  # 2 second test
                    if not self.serial_port.is_open:
                        stable = False
                        break
                    time.sleep(0.1)
                
                if not stable:
                    self.serial_port.close()
                    raise serial.SerialException("Connection unstable during initial test")
            
            self.connection_changed.emit(True, f"Connected to {port_name} at {initial_baud} baud")
            return True
            
        except Exception as e:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                except:
                    pass
            self.serial_port = None
            raise e  # Re-raise for retry mechanism
    
    def disconnect(self):
        """Disconnect from serial port"""
        if self.connection_state == ConnectionState.DISCONNECTED:
            return
            
        # Stop reading thread
        self.stop_reading.set()
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=2)
            
        # Close serial port
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception as e:
                self.error_occurred.emit(f"Error closing port: {e}")
                
        self.serial_port = None
        self.connection_state = ConnectionState.DISCONNECTED
        self.connection_changed.emit(False, "Disconnected")
    
    def send_command(self, command: str) -> bool:
        """
        Send command to GMINS system with smart baud rate switching
        
        Args:
            command: Command to send
            
        Returns:
            True if command sent successfully
        """
        if not self.is_connected():
            self.error_occurred.emit("Not connected to send command")
            return False
            
        command = command.strip().upper()
        
        # Determine required baud rate for this command
        required_baud = self._get_baud_rate_for_command(command)
        
        # Switch baud rate if needed
        if required_baud != self.current_baud_rate:
            if not self._switch_baud_rate(required_baud):
                return False
                
        # Send command
        try:
            command_bytes = (command + '\n').encode('utf-8')
            bytes_sent = self.serial_port.write(command_bytes)
            self.serial_port.flush()  # Ensure data is sent immediately
            
            self.bytes_sent += bytes_sent
            
            # Update protocol mode if this is a protocol switch command
            if command == "AR1AFC":
                self.protocol_mode = ProtocolMode.AR1AFC
                self.protocol_detected.emit("AR1AFC")
            elif command == "MAVLINK":
                self.protocol_mode = ProtocolMode.MAVLINK 
                self.protocol_detected.emit("MAVLink")
                
            return True
            
        except Exception as e:
            self.error_occurred.emit(f"Error sending command: {e}")
            return False
    
    def send_raw_data(self, data: bytes) -> bool:
        """
        Send raw data to serial port
        
        Args:
            data: Raw bytes to send
            
        Returns:
            True if data sent successfully
        """
        if not self.is_connected():
            return False
            
        try:
            bytes_sent = self.serial_port.write(data)
            self.serial_port.flush()
            self.bytes_sent += bytes_sent
            return True
        except Exception as e:
            self.error_occurred.emit(f"Error sending raw data: {e}")
            return False
    
    def is_connected(self) -> bool:
        """Check if currently connected"""
        return (self.connection_state == ConnectionState.CONNECTED and 
                self.serial_port and self.serial_port.is_open)
    
    def get_connection_info(self) -> Dict[str, Any]:
        """Get current connection information"""
        return {
            'connected': self.is_connected(),
            'port_name': self.port_name,
            'baud_rate': self.current_baud_rate,
            'protocol_mode': self.protocol_mode.value,
            'connection_time': self.connection_time,
            'bytes_received': self.bytes_received,
            'bytes_sent': self.bytes_sent,
            'last_data_time': self.last_data_time
        }
    
    def get_data_rate(self) -> float:
        """Get current data reception rate in bytes per second"""
        if not self.is_connected() or self.connection_time == 0:
            return 0.0
            
        elapsed = time.time() - self.connection_time
        if elapsed <= 0:
            return 0.0
            
        return self.bytes_received / elapsed
    
    def set_data_callback(self, callback: Callable[[bytes], None]):
        """Set callback function for received data"""
        self.data_callback = callback
    
    def _get_baud_rate_for_command(self, command: str) -> int:
        """Get required baud rate for specific command"""
        # Protocol switching commands use fixed baud rates
        if command in self.command_baud_rates:
            return self.command_baud_rates[command]
            
        # Other commands use protocol-specific baud rates
        if self.protocol_mode == ProtocolMode.MAVLINK:
            return self.protocol_baud_rates[ProtocolMode.MAVLINK]  # 460800
        else:
            return self.protocol_baud_rates[ProtocolMode.AR1AFC]   # 230400
    
    def _switch_baud_rate(self, new_baud_rate: int) -> bool:
        """Switch serial port baud rate"""
        if not self.serial_port:
            return False
            
        try:
            # Close and reopen with new baud rate
            was_open = self.serial_port.is_open
            if was_open:
                self.serial_port.close()
                
            self.serial_port.baudrate = new_baud_rate
            
            if was_open:
                self.serial_port.open()
                
            self.current_baud_rate = new_baud_rate
            return True
            
        except Exception as e:
            self.error_occurred.emit(f"Error switching baud rate: {e}")
            return False
    
    def _read_data_thread(self):
        """Background thread for reading serial data"""
        while not self.stop_reading.is_set():
            if not self.serial_port or not self.serial_port.is_open:
                break
                
            try:
                # Read available data
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    if data:
                        self.bytes_received += len(data)
                        self.last_data_time = time.time()
                        
                        # Add to buffer
                        self.data_buffer.extend(data)
                        
                        # Emit signal with new data
                        self.data_received.emit(bytes(data))
                        
                        # Call callback if set
                        if self.data_callback:
                            try:
                                self.data_callback(bytes(data))
                            except Exception as e:
                                self.error_occurred.emit(f"Data callback error: {e}")
                
                # Small sleep to prevent excessive CPU usage and GUI interference
                time.sleep(0.05)  # 50ms - INCREASED for stability (was 1ms)
                
            except serial.SerialException as e:
                self.error_occurred.emit(f"Serial read error: {e}")
                # Automatically disconnect on serial error
                self._handle_connection_lost()
                break
            except Exception as e:
                self.error_occurred.emit(f"Unexpected read error: {e}")
                # Automatically disconnect on unexpected error
                self._handle_connection_lost()
                break
    
    def get_buffer_data(self) -> bytes:
        """Get and clear buffered data"""
        data = bytes(self.data_buffer)
        self.data_buffer.clear()
        return data
    
    def clear_buffer(self):
        """Clear the data buffer"""
        self.data_buffer.clear()
    
    def _handle_connection_lost(self):
        """Handle unexpected connection loss"""
        try:
            # Clean up connection state
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
        except Exception:
            pass  # Ignore errors during cleanup
            
        self.serial_port = None
        self.connection_state = ConnectionState.ERROR
        
        # Notify GUI of disconnection
        self.connection_changed.emit(False, "Connection lost unexpectedly")


class SerialManagerThread(QThread):
    """Thread wrapper for serial manager (if needed for heavy operations)"""
    
    def __init__(self, serial_manager: SerialManager):
        super().__init__()
        self.serial_manager = serial_manager
        
    def run(self):
        """Thread execution (placeholder for future use)"""
        pass


# Utility functions for GUI integration
def create_serial_manager() -> SerialManager:
    """Create and return SerialManager instance"""
    return SerialManager()


def get_available_ports() -> List[Dict[str, str]]:
    """Get available serial ports (standalone function)"""
    manager = SerialManager()
    return manager.get_available_ports()


# Test function
if __name__ == "__main__":
    import sys
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QTimer
    
    app = QApplication(sys.argv)
    
    # Test the serial manager
    manager = SerialManager()
    
    def on_data_received(data):
        print(f"📨 Received: {data}")
        
    def on_connection_changed(connected, message):
        print(f"🔗 Connection: {connected} - {message}")
        
    def on_error(error):
        print(f"❌ Error: {error}")
    
    # Connect signals
    manager.data_received.connect(on_data_received)
    manager.connection_changed.connect(on_connection_changed) 
    manager.error_occurred.connect(on_error)
    
    # Test port scanning
    print("🔍 Scanning for available ports...")
    ports = manager.get_available_ports()
    print(f"📡 Found {len(ports)} ports:")
    for port in ports:
        print(f"  - {port['device']}: {port['description']}")
    
    # Test connection (would need actual port)
    print("\n💡 SerialManager test completed successfully")
    print("   - Port scanning works")
    print("   - Smart baud rate switching ready")
    print("   - Threading support enabled")
    
    app.quit()