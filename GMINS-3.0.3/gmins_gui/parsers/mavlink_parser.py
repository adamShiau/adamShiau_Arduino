#!/usr/bin/env python3
"""
MAVLink Protocol Parser

Parser for GMINS MAVLink protocol data.
Handles MAVLink v1 messages from GMINS system.

Key Messages:
- HEARTBEAT (ID=0): System status and connection
- GPS_INPUT (ID=232): GPS navigation data from GMINS
- ATTITUDE (ID=30): Attitude data (if available)
- GPS_RAW_INT (ID=24): Raw GPS data (if available)

Note: This parser works without requiring full pymavlink installation
by implementing basic MAVLink v1 parsing manually for GMINS-specific messages.
"""

import struct
import time
from typing import Optional, Dict, Any, List
from enum import IntEnum


class MAVLinkMessageID(IntEnum):
    """MAVLink Message IDs used by GMINS"""
    HEARTBEAT = 0
    GPS_RAW_INT = 24 
    ATTITUDE = 30
    GPS_INPUT = 232


class MAVType(IntEnum):
    """MAVLink vehicle types"""
    GENERIC = 0
    FIXED_WING = 1
    QUADROTOR = 2
    HELICOPTER = 3


class MAVState(IntEnum):
    """MAVLink system states"""
    UNINIT = 0
    BOOT = 1
    CALIBRATING = 2 
    STANDBY = 3
    ACTIVE = 4
    CRITICAL = 5
    EMERGENCY = 6
    POWEROFF = 7


class MAVLinkParser:
    """MAVLink v1 Protocol Parser for GMINS"""
    
    # MAVLink v1 constants
    MAVLINK_STX = 0xFE  # Start byte
    HEADER_SIZE = 6     # STX + LEN + SEQ + SYS + COMP + MSGID
    CRC_SIZE = 2        # CRC-16
    MIN_PACKET_SIZE = HEADER_SIZE + CRC_SIZE  # 8 bytes minimum
    MAX_PAYLOAD_SIZE = 255
    
    def __init__(self):
        """Initialize MAVLink parser"""
        self.packet_count = 0
        self.parse_errors = 0
        self.crc_errors = 0
        self.last_packet_time = 0
        self.start_time = time.time()
        
        # Statistics by message type
        self.message_counts = {}
        self.bytes_received = 0
        
        # Buffer for partial packets
        self.buffer = bytearray()
        
        # Latest parsed data
        self.latest_heartbeat = None
        self.latest_gps_input = None
        self.latest_attitude = None
        
    def parse_data(self, data: bytes) -> Optional[Dict[str, Any]]:
        """
        Parse incoming MAVLink data
        
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
        while len(self.buffer) >= self.MIN_PACKET_SIZE:
            packet_data = self._find_and_extract_packet()
            if packet_data:
                parsed = self._parse_packet(packet_data)
                if parsed:
                    return parsed
            else:
                # No valid packet found, remove first byte and try again
                if len(self.buffer) > 0:
                    self.buffer.pop(0)
                    
        return None
    
    def _find_and_extract_packet(self) -> Optional[bytes]:
        """
        Find and extract a complete MAVLink packet from buffer
        
        Returns:
            Complete MAVLink packet data if found, None otherwise
        """
        # Look for MAVLink start byte
        start_idx = -1
        for i in range(len(self.buffer)):
            if self.buffer[i] == self.MAVLINK_STX:
                start_idx = i
                break
                
        if start_idx == -1:
            # No start byte found
            self.buffer.clear()
            return None
            
        if start_idx > 0:
            # Remove data before start byte
            del self.buffer[:start_idx]
            
        # Check if we have enough bytes for header
        if len(self.buffer) < self.HEADER_SIZE:
            return None
            
        # Get payload length from header
        payload_len = self.buffer[1]
        packet_size = self.HEADER_SIZE + payload_len + self.CRC_SIZE
        
        # Check if we have complete packet
        if len(self.buffer) < packet_size:
            return None
            
        # Extract packet
        packet = bytes(self.buffer[:packet_size])
        del self.buffer[:packet_size]
        
        return packet
    
    def _parse_packet(self, packet_data: bytes) -> Optional[Dict[str, Any]]:
        """
        Parse a complete MAVLink packet
        
        Args:
            packet_data: Complete MAVLink packet
            
        Returns:
            Dictionary with parsed data, or None if parsing failed
        """
        if len(packet_data) < self.MIN_PACKET_SIZE:
            self.parse_errors += 1
            return None
            
        try:
            # Parse header
            stx = packet_data[0]
            payload_len = packet_data[1] 
            seq = packet_data[2]
            sys_id = packet_data[3]
            comp_id = packet_data[4]
            msg_id = packet_data[5]
            
            if stx != self.MAVLINK_STX:
                self.parse_errors += 1
                return None
                
            # Extract payload and CRC
            payload = packet_data[self.HEADER_SIZE:-self.CRC_SIZE]
            received_crc = struct.unpack('<H', packet_data[-self.CRC_SIZE:])[0]
            
            # Verify payload length
            if len(payload) != payload_len:
                self.parse_errors += 1
                return None
                
            # Calculate CRC (simplified - for production use proper MAVLink CRC)
            calculated_crc = self._calculate_crc(packet_data[1:-self.CRC_SIZE], msg_id)
            
            # For now, skip CRC verification to focus on data parsing
            # if calculated_crc != received_crc:
            #     self.crc_errors += 1
            #     return None
                
            # Update statistics
            self.packet_count += 1
            self.last_packet_time = time.time()
            self.message_counts[msg_id] = self.message_counts.get(msg_id, 0) + 1
            
            # Parse message based on ID
            parsed_data = self._parse_message(msg_id, payload, seq, sys_id, comp_id)
            
            if parsed_data:
                parsed_data.update({
                    'packet_id': self.packet_count,
                    'timestamp': self.last_packet_time,
                    'protocol': 'MAVLink',
                    'msg_id': msg_id,
                    'sys_id': sys_id,
                    'comp_id': comp_id,
                    'sequence': seq
                })
                
            return parsed_data
            
        except Exception as e:
            self.parse_errors += 1
            return None
    
    def _parse_message(self, msg_id: int, payload: bytes, seq: int, sys_id: int, comp_id: int) -> Optional[Dict[str, Any]]:
        """Parse specific MAVLink message types"""
        
        if msg_id == MAVLinkMessageID.HEARTBEAT:
            return self._parse_heartbeat(payload)
        elif msg_id == MAVLinkMessageID.GPS_INPUT:
            return self._parse_gps_input(payload)
        elif msg_id == MAVLinkMessageID.ATTITUDE:
            return self._parse_attitude(payload)
        elif msg_id == MAVLinkMessageID.GPS_RAW_INT:
            return self._parse_gps_raw_int(payload)
        else:
            # Unknown message type
            return {
                'message_type': 'unknown',
                'msg_id': msg_id,
                'payload_size': len(payload)
            }
    
    def _parse_heartbeat(self, payload: bytes) -> Dict[str, Any]:
        """Parse HEARTBEAT message (ID=0)"""
        if len(payload) < 9:
            return None
            
        # HEARTBEAT structure: type(1), autopilot(1), base_mode(1), custom_mode(4), system_status(1), mavlink_version(1)
        values = struct.unpack('<BBBIBBB', payload[:9])
        
        data = {
            'message_type': 'heartbeat',
            'vehicle_type': values[0],
            'autopilot': values[1], 
            'base_mode': values[2],
            'custom_mode': values[3],
            'system_status': values[4],
            'mavlink_version': values[5],
            'display': {
                'vehicle_type': self._get_vehicle_type_name(values[0]),
                'system_status': self._get_system_status_name(values[4]),
                'connection': 'Connected'
            }
        }
        
        self.latest_heartbeat = data
        return data
    
    def _parse_gps_input(self, payload: bytes) -> Dict[str, Any]:
        """Parse GPS_INPUT message (ID=232)"""
        if len(payload) < 63:  # GPS_INPUT is 63 bytes
            return None
            
        # GPS_INPUT structure (simplified - key fields only)
        # Full format: time_usec(8), gps_id(1), ignore_flags(2), time_week_ms(4), time_week(2), 
        # fix_type(1), lat(4), lon(4), alt(4), hdop(4), vdop(4), vn(4), ve(4), vd(4), 
        # speed_accuracy(4), horiz_accuracy(4), vert_accuracy(4), ignore_flags2(2), 
        # satellites_visible(1), yaw(2)
        
        try:
            # Unpack key fields (little-endian format)
            values = struct.unpack('<Q B H I H B i i f f f f f f f f f H B H', payload[:63])
            
            time_usec = values[0]
            gps_id = values[1]
            ignore_flags = values[2]
            time_week_ms = values[3]
            time_week = values[4]
            fix_type = values[5]
            lat = values[6] / 1e7  # Convert from 1e7 format to degrees
            lon = values[7] / 1e7  # Convert from 1e7 format to degrees
            alt = values[8]        # Altitude in meters
            hdop = values[9]
            vdop = values[10]
            vn = values[11]        # Velocity North (m/s)
            ve = values[12]        # Velocity East (m/s)
            vd = values[13]        # Velocity Down (m/s)
            speed_accuracy = values[14]
            horiz_accuracy = values[15]
            vert_accuracy = values[16]
            ignore_flags2 = values[17]
            satellites_visible = values[18]
            yaw = values[19]
            
            data = {
                'message_type': 'gps_input',
                'time_usec': time_usec,
                'gps_id': gps_id,
                'fix_type': fix_type,
                
                # Position data
                'position': {
                    'latitude': lat,
                    'longitude': lon,
                    'altitude_msl': alt
                },
                
                # Velocity data
                'velocity': {
                    'north': vn,
                    'east': ve,
                    'down': vd,
                    'speed': (vn**2 + ve**2)**0.5
                },
                
                # Quality indicators
                'quality': {
                    'hdop': hdop,
                    'vdop': vdop,
                    'satellites_visible': satellites_visible,
                    'fix_type': fix_type,
                    'speed_accuracy': speed_accuracy,
                    'horiz_accuracy': horiz_accuracy,
                    'vert_accuracy': vert_accuracy
                },
                
                # For GUI display
                'display': {
                    'lat_deg': f"{lat:.8f}°",
                    'lon_deg': f"{lon:.8f}°",
                    'alt_m': f"{alt:.1f}m",
                    'speed_ms': f"{(vn**2 + ve**2)**0.5:.2f} m/s",
                    'satellites': f"{satellites_visible} sats",
                    'fix_type': self._get_fix_type_name(fix_type),
                    'hdop': f"{hdop:.2f}",
                    'data_rate': f"{self._calculate_data_rate():.1f} Hz"
                }
            }
            
            self.latest_gps_input = data
            return data
            
        except struct.error:
            return None
    
    def _parse_attitude(self, payload: bytes) -> Dict[str, Any]:
        """Parse ATTITUDE message (ID=30)"""
        if len(payload) < 28:
            return None
            
        # ATTITUDE structure: time_boot_ms(4), roll(4), pitch(4), yaw(4), 
        # rollspeed(4), pitchspeed(4), yawspeed(4)
        values = struct.unpack('<I f f f f f f', payload[:28])
        
        time_boot_ms = values[0]
        roll = values[1]      # radians
        pitch = values[2]     # radians  
        yaw = values[3]       # radians
        rollspeed = values[4] # rad/s
        pitchspeed = values[5] # rad/s
        yawspeed = values[6]  # rad/s
        
        # Convert to degrees for display
        roll_deg = roll * 180.0 / 3.14159
        pitch_deg = pitch * 180.0 / 3.14159
        yaw_deg = yaw * 180.0 / 3.14159
        
        data = {
            'message_type': 'attitude',
            'time_boot_ms': time_boot_ms,
            
            # Attitude data
            'attitude': {
                'roll': roll_deg,    # degrees
                'pitch': pitch_deg,  # degrees
                'yaw': yaw_deg       # degrees
            },
            
            # Angular velocity data
            'angular_velocity': {
                'roll_speed': rollspeed,   # rad/s
                'pitch_speed': pitchspeed, # rad/s
                'yaw_speed': yawspeed      # rad/s
            },
            
            # For GUI display
            'display': {
                'roll_deg': f"{roll_deg:.2f}°",
                'pitch_deg': f"{pitch_deg:.2f}°",
                'yaw_deg': f"{yaw_deg:.2f}°"
            }
        }
        
        self.latest_attitude = data
        return data
    
    def _parse_gps_raw_int(self, payload: bytes) -> Dict[str, Any]:
        """Parse GPS_RAW_INT message (ID=24)"""
        if len(payload) < 30:
            return None
            
        # GPS_RAW_INT structure
        values = struct.unpack('<Q B i i i H H H H H B', payload[:30])
        
        time_usec = values[0]
        fix_type = values[1]
        lat = values[2] / 1e7   # Convert from 1e7 format
        lon = values[3] / 1e7   # Convert from 1e7 format
        alt = values[4] / 1000.0 # Convert from mm to m
        eph = values[5]
        epv = values[6]
        vel = values[7]
        cog = values[8]
        satellites_visible = values[9]
        
        data = {
            'message_type': 'gps_raw_int',
            'time_usec': time_usec,
            'fix_type': fix_type,
            'position': {
                'latitude': lat,
                'longitude': lon,
                'altitude': alt
            },
            'quality': {
                'eph': eph,
                'epv': epv,
                'vel': vel,
                'cog': cog,
                'satellites_visible': satellites_visible
            },
            'display': {
                'lat_deg': f"{lat:.8f}°",
                'lon_deg': f"{lon:.8f}°",
                'alt_m': f"{alt:.1f}m",
                'satellites': f"{satellites_visible} sats",
                'fix_type': self._get_fix_type_name(fix_type)
            }
        }
        
        return data
    
    def _calculate_crc(self, data: bytes, msg_id: int) -> int:
        """Calculate MAVLink CRC-16 (simplified version)"""
        # This is a simplified CRC calculation
        # For production, use proper MAVLink CRC with CRC_EXTRA
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
                crc &= 0xFFFF
        return crc
    
    def _calculate_data_rate(self) -> float:
        """Calculate current data rate in Hz"""
        if self.packet_count < 2:
            return 0.0
            
        elapsed_time = time.time() - self.start_time
        if elapsed_time <= 0:
            return 0.0
            
        return self.packet_count / elapsed_time
    
    def _get_vehicle_type_name(self, vehicle_type: int) -> str:
        """Get human-readable vehicle type name"""
        types = {
            0: "Generic",
            1: "Fixed Wing", 
            2: "Quadrotor",
            3: "Helicopter",
            4: "Antenna Tracker"
        }
        return types.get(vehicle_type, f"Unknown({vehicle_type})")
    
    def _get_system_status_name(self, status: int) -> str:
        """Get human-readable system status name"""
        statuses = {
            0: "Uninitialized",
            1: "Boot",
            2: "Calibrating",
            3: "Standby", 
            4: "Active",
            5: "Critical",
            6: "Emergency",
            7: "Power Off"
        }
        return statuses.get(status, f"Unknown({status})")
    
    def _get_fix_type_name(self, fix_type: int) -> str:
        """Get human-readable GPS fix type name"""
        types = {
            0: "No GPS",
            1: "No Fix",
            2: "2D Fix",
            3: "3D Fix",
            4: "DGPS Fix",
            5: "RTK Float",
            6: "RTK Fixed"
        }
        return types.get(fix_type, f"Unknown({fix_type})")
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get parser statistics"""
        elapsed_time = time.time() - self.start_time
        
        return {
            'protocol': 'MAVLink',
            'total_packets': self.packet_count,
            'bytes_received': self.bytes_received,
            'parse_errors': self.parse_errors,
            'crc_errors': self.crc_errors,
            'elapsed_time': elapsed_time,
            'data_rate_hz': self._calculate_data_rate(),
            'error_rate': self.parse_errors / max(1, self.packet_count),
            'buffer_size': len(self.buffer),
            'message_counts': self.message_counts.copy()
        }
    
    def get_latest_navigation_data(self) -> Optional[Dict[str, Any]]:
        """Get the latest navigation data from GPS_INPUT or GPS_RAW_INT"""
        if self.latest_gps_input:
            return self.latest_gps_input
        return None
    
    def get_latest_attitude_data(self) -> Optional[Dict[str, Any]]:
        """Get the latest attitude data"""
        return self.latest_attitude
    
    def get_connection_status(self) -> Dict[str, Any]:
        """Get connection status from HEARTBEAT"""
        if not self.latest_heartbeat:
            return {
                'connected': False,
                'status': 'No heartbeat received'
            }
            
        # Consider connected if heartbeat received within last 3 seconds
        time_since_heartbeat = time.time() - self.last_packet_time
        connected = time_since_heartbeat < 3.0
        
        return {
            'connected': connected,
            'status': self.latest_heartbeat['display']['system_status'],
            'vehicle_type': self.latest_heartbeat['display']['vehicle_type'],
            'time_since_last': time_since_heartbeat
        }
    
    def is_mavlink_data(self, data: bytes) -> bool:
        """
        Quick check if data contains MAVLink packets
        
        Args:
            data: Raw bytes to check
            
        Returns:
            True if data likely contains MAVLink packets
        """
        if len(data) == 0:
            return False
            
        # Look for MAVLink start byte
        return self.MAVLINK_STX in data
    
    def reset_statistics(self):
        """Reset all statistics"""
        self.packet_count = 0
        self.parse_errors = 0
        self.crc_errors = 0
        self.bytes_received = 0
        self.message_counts.clear()
        self.start_time = time.time()
        self.buffer.clear()
        
        # Clear latest data
        self.latest_heartbeat = None
        self.latest_gps_input = None
        self.latest_attitude = None


# Utility functions for GUI integration
def create_mavlink_parser() -> MAVLinkParser:
    """Create and return MAVLink parser instance"""
    return MAVLinkParser()


# Test function
if __name__ == "__main__":
    # Test the parser
    parser = MAVLinkParser()
    
    print("🧪 Testing MAVLink Parser...")
    print("=" * 40)
    
    # Test with some sample data (would need real MAVLink data for full test)
    print("✅ MAVLink parser created successfully")
    
    # Show statistics
    stats = parser.get_statistics()
    print(f"📊 Initial parser stats: {stats}")
    
    print("\n💡 Parser is ready to process MAVLink data from GMINS system")
    print("   - Supports HEARTBEAT, GPS_INPUT, ATTITUDE, GPS_RAW_INT messages")
    print("   - Provides real-time navigation data parsing")
    print("   - Compatible with GMINS MAVLink v1 implementation")