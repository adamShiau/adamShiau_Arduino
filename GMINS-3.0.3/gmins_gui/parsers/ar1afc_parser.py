#!/usr/bin/env python3
"""
AR1AFC Protocol Parser

Parser for GMINS AR-1A-FC custom protocol.
Handles 52-byte fixed packet format with navigation data.

Packet Format (52 bytes total):
- Header (4 bytes): 0xFE 0x81 0xFF 0x55  
- Gyro X,Y,Z (12 bytes): Angular velocity in DPS
- Accel X,Y,Z (12 bytes): Acceleration in g
- Temperature (4 bytes): Temperature in °C
- Time Counter (4 bytes): Time counter in ms
- Pitch, Roll, Yaw (12 bytes): Attitude angles in degrees (corrected)
- CRC32 (4 bytes): CRC-32 checksum
"""

import struct
import time
from typing import Optional, Dict, Any
import zlib  # For CRC32 calculation


class AR1AFCParser:
    """AR1AFC Protocol Parser"""
    
    # Protocol constants
    PACKET_SIZE = 52
    HEADER = b'\xFE\x81\xFF\x55'
    HEADER_SIZE = 4
    CRC_SIZE = 4
    PAYLOAD_SIZE = PACKET_SIZE - HEADER_SIZE - CRC_SIZE  # 44 bytes
    
    # Packet structure format (little-endian)
    # 'f' = float (4 bytes), 'I' = unsigned int (4 bytes)
    PACKET_FORMAT = '<4s3f3f1f1I3f1I'  # < means little-endian
    #                ^header ^gyro ^accel ^temp ^time ^attitude ^crc
    
    def __init__(self):
        """Initialize AR1AFC parser"""
        self.packet_count = 0
        self.parse_errors = 0
        self.last_packet_time = 0
        self.start_time = time.time()
        
        # Statistics
        self.bytes_received = 0
        self.valid_packets = 0
        self.crc_errors = 0
        self.header_errors = 0
        
        # Buffer for partial packets
        self.buffer = bytearray()
        
    def parse_data(self, data: bytes) -> Optional[Dict[str, Any]]:
        """
        Parse incoming data for AR1AFC packets
        
        Args:
            data: Raw bytes from serial port
            
        Returns:
            Dictionary with parsed navigation data, or None if no valid packet
        """
        if not data:
            return None
            
        # Add to buffer
        self.buffer.extend(data)
        self.bytes_received += len(data)
        
        # Try to find and parse complete packets
        while len(self.buffer) >= self.PACKET_SIZE:
            packet_data = self._find_and_extract_packet()
            if packet_data:
                parsed = self._parse_packet(packet_data)
                if parsed:
                    return parsed
            else:
                # No valid packet found, remove first byte and try again
                self.buffer.pop(0)
                
        return None
    
    def _find_and_extract_packet(self) -> Optional[bytes]:
        """
        Find and extract a complete AR1AFC packet from buffer
        
        Returns:
            52-byte packet data if found, None otherwise
        """
        # Look for header pattern
        for i in range(len(self.buffer) - self.PACKET_SIZE + 1):
            if self.buffer[i:i+self.HEADER_SIZE] == self.HEADER:
                # Found header, extract complete packet
                packet = bytes(self.buffer[i:i+self.PACKET_SIZE])
                
                # Remove processed data from buffer
                del self.buffer[:i+self.PACKET_SIZE]
                
                return packet
                
        return None
    
    def _parse_packet(self, packet_data: bytes) -> Optional[Dict[str, Any]]:
        """
        Parse a complete 52-byte AR1AFC packet
        
        Args:
            packet_data: 52 bytes of packet data
            
        Returns:
            Dictionary with parsed data, or None if parsing failed
        """
        if len(packet_data) != self.PACKET_SIZE:
            self.parse_errors += 1
            return None
            
        try:
            # Unpack packet data
            unpacked = struct.unpack(self.PACKET_FORMAT, packet_data)
            
            # Extract fields
            header = unpacked[0]
            gyro_x, gyro_y, gyro_z = unpacked[1:4]
            accel_x, accel_y, accel_z = unpacked[4:7]
            temperature = unpacked[7]
            time_counter = unpacked[8]
            pitch, roll, yaw = unpacked[9:12]
            received_crc = unpacked[12]
            
            # Verify header
            if header != self.HEADER:
                self.header_errors += 1
                return None
            
            # Verify CRC32
            payload = packet_data[:-self.CRC_SIZE]  # All except last 4 bytes
            calculated_crc = zlib.crc32(payload) & 0xFFFFFFFF
            
            if calculated_crc != received_crc:
                self.crc_errors += 1
                return None
            
            # Update statistics
            self.valid_packets += 1
            self.packet_count += 1
            self.last_packet_time = time.time()
            
            # Create navigation data dictionary
            nav_data = {
                # Packet info
                'packet_id': self.packet_count,
                'timestamp': self.last_packet_time,
                'protocol': 'AR1AFC',
                'time_counter': time_counter,
                
                # IMU data
                'gyro': {
                    'x': gyro_x,    # DPS (degrees per second)
                    'y': gyro_y,
                    'z': gyro_z
                },
                
                'accel': {
                    'x': accel_x,   # g (gravitational units)
                    'y': accel_y, 
                    'z': accel_z
                },
                
                # Attitude data (already corrected from GMINS)
                'attitude': {
                    'roll': roll,     # degrees
                    'pitch': pitch,   # degrees
                    'yaw': yaw        # degrees
                },
                
                # Environmental data
                'temperature': temperature,  # °C
                
                # Quality indicators
                'crc_valid': True,
                'data_quality': 'good',
                
                # For GUI display
                'display': {
                    'roll_deg': f"{roll:.2f}°",
                    'pitch_deg': f"{pitch:.2f}°", 
                    'yaw_deg': f"{yaw:.2f}°",
                    'temp_c': f"{temperature:.1f}°C",
                    'data_rate': self._calculate_data_rate(),
                    'packet_count': self.packet_count
                }
            }
            
            return nav_data
            
        except struct.error as e:
            self.parse_errors += 1
            return None
        except Exception as e:
            self.parse_errors += 1
            return None
    
    def _calculate_data_rate(self) -> float:
        """Calculate current data rate in Hz"""
        if self.packet_count < 2:
            return 0.0
            
        elapsed_time = time.time() - self.start_time
        if elapsed_time <= 0:
            return 0.0
            
        return self.packet_count / elapsed_time
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get parser statistics"""
        elapsed_time = time.time() - self.start_time
        
        return {
            'protocol': 'AR1AFC',
            'total_packets': self.packet_count,
            'valid_packets': self.valid_packets,
            'bytes_received': self.bytes_received,
            'parse_errors': self.parse_errors,
            'crc_errors': self.crc_errors,
            'header_errors': self.header_errors,
            'elapsed_time': elapsed_time,
            'data_rate_hz': self._calculate_data_rate(),
            'error_rate': self.parse_errors / max(1, self.packet_count),
            'buffer_size': len(self.buffer)
        }
    
    def reset_statistics(self):
        """Reset all statistics"""
        self.packet_count = 0
        self.valid_packets = 0
        self.parse_errors = 0
        self.crc_errors = 0
        self.header_errors = 0
        self.bytes_received = 0
        self.start_time = time.time()
        self.buffer.clear()
    
    def is_ar1afc_data(self, data: bytes) -> bool:
        """
        Quick check if data contains AR1AFC packets
        
        Args:
            data: Raw bytes to check
            
        Returns:
            True if data likely contains AR1AFC packets
        """
        if len(data) < self.HEADER_SIZE:
            return False
            
        # Look for AR1AFC header pattern
        return self.HEADER in data
    
    @staticmethod
    def create_test_packet() -> bytes:
        """
        Create a test AR1AFC packet for debugging
        
        Returns:
            Valid 52-byte AR1AFC packet
        """
        header = AR1AFCParser.HEADER
        
        # Test data
        gyro_x, gyro_y, gyro_z = 1.5, -2.3, 0.8
        accel_x, accel_y, accel_z = 0.02, -0.15, 0.98
        temperature = 35.2
        time_counter = 12345
        roll, pitch, yaw = 5.5, -2.1, 87.3
        
        # Pack payload (without CRC)
        payload = struct.pack('<3f3f1f1I3f', 
                             gyro_x, gyro_y, gyro_z,
                             accel_x, accel_y, accel_z,
                             temperature, time_counter,
                             pitch, roll, yaw)
        
        # Calculate CRC32 
        header_payload = header + payload
        crc32 = zlib.crc32(header_payload) & 0xFFFFFFFF
        
        # Complete packet
        packet = header + payload + struct.pack('<I', crc32)
        
        return packet


# Utility function for GUI integration
def create_ar1afc_parser() -> AR1AFCParser:
    """Create and return AR1AFC parser instance"""
    return AR1AFCParser()


# Test function
if __name__ == "__main__":
    # Test the parser
    parser = AR1AFCParser()
    
    # Create test packet
    test_packet = AR1AFCParser.create_test_packet()
    print(f"📦 Test packet size: {len(test_packet)} bytes")
    print(f"📦 Test packet hex: {test_packet.hex()}")
    
    # Parse test packet
    result = parser.parse_data(test_packet)
    
    if result:
        print("✅ Test packet parsed successfully!")
        print(f"🎯 Attitude: Roll={result['attitude']['roll']:.2f}°, "
              f"Pitch={result['attitude']['pitch']:.2f}°, "
              f"Yaw={result['attitude']['yaw']:.2f}°")
        print(f"🌡️  Temperature: {result['temperature']:.1f}°C")
        print(f"⚡ Gyro: X={result['gyro']['x']:.2f}, "
              f"Y={result['gyro']['y']:.2f}, "
              f"Z={result['gyro']['z']:.2f} DPS")
    else:
        print("❌ Test packet parsing failed!")
    
    # Show statistics
    stats = parser.get_statistics()
    print(f"📊 Parser stats: {stats}")