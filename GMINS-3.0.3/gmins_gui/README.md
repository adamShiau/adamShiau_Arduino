# GMINS Navigation System GUI

Real-time monitoring and control interface for the GMINS Navigation System. Supports both AR1AFC and MAVLink protocols with intelligent baud rate switching.

![GMINS GUI](https://img.shields.io/badge/Status-In_Development-yellow)
![Python](https://img.shields.io/badge/Python-3.7%2B-blue)
![PyQt5](https://img.shields.io/badge/GUI-PyQt5-green)
![License](https://img.shields.io/badge/License-MIT-blue)

## 🚀 Features

### Core Functionality
- **📊 Real-time Data Monitoring**: Live display of navigation data (attitude, position, velocity)
- **🔄 Dual Protocol Support**: AR1AFC and MAVLink protocols with auto-detection
- **📈 Data Visualization**: Real-time plots and 3D attitude display
- **🎮 System Control**: Send commands and switch protocols
- **💾 Data Logging**: Record and export navigation data

### Smart Communication
- **🔌 Auto Port Detection**: Automatic scanning and selection of serial ports
- **⚡ Smart Baud Rate Switching**: Intelligent baud rate selection per protocol/command
- **🔗 Connection Management**: Robust connection handling with error recovery
- **📡 Protocol Auto-Detection**: Automatic identification of AR1AFC vs MAVLink data

## 📋 Requirements

### Software Requirements
- Python 3.7 or higher
- PyQt5 GUI framework
- pyserial for communication
- numpy, pandas for data processing
- matplotlib for plotting

### Hardware Requirements
- GMINS Navigation System
- USB to Serial converter (if needed)
- Available COM port

## 🛠️ Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd GMINS/gmins_gui
```

### 2. Install Dependencies
```bash
pip install -r requirements.txt
```

Or install manually:
```bash
pip install PyQt5 PyQt5-tools pyserial numpy pandas matplotlib pyqtgraph pymavlink
```

### 3. Verify Installation
```bash
python main.py --test
```

## 🎯 Quick Start

### 1. Launch the Application
```bash
cd gmins_gui
python main.py
```

### 2. Connect to GMINS System
1. **Select COM Port**: Choose the correct serial port from the dropdown
2. **Choose Protocol**: Select AR1AFC, MAVLink, or Auto-detect
3. **Click Connect**: The application will establish connection

### 3. Monitor Data
- **Navigation Panel**: Shows real-time attitude, position, and velocity
- **Plots Tab**: View real-time graphs of navigation data
- **3D Attitude Tab**: Visual representation of system orientation
- **Data Log Tab**: System messages and data logging

### 4. Send Commands
- **Protocol Switching**: Use AR1AFC/MAVLink buttons to switch modes
- **System Status**: Query system status and health
- **Custom Commands**: Send any custom command via text input

## 📡 Protocol Support

### AR1AFC Protocol
- **Fixed 52-byte packets** with navigation data
- **Header**: `0xFE 0x81 0xFF 0x55`
- **Data**: Gyro, Accelerometer, Temperature, Attitude
- **Default Baud Rate**: 230400 for other commands, 460800 for switching

### MAVLink Protocol  
- **Standard MAVLink v1** messages
- **Key Messages**: HEARTBEAT, GPS_INPUT, ATTITUDE
- **Default Baud Rate**: 460800 for other commands, 230400 for switching
- **QGroundControl Compatible**: Full compatibility with QGC

## 🔧 Configuration

### Smart Baud Rate Configuration
The system uses intelligent baud rate switching:

```python
# Protocol switching commands (fixed rates)
AR1AFC switching command  → 460800 baud
MAVLINK switching command → 230400 baud

# Other commands (depends on current protocol mode)
AR1AFC mode:  other commands → 230400 baud
MAVLink mode: other commands → 460800 baud
```

### Auto-Detection Logic
1. **Monitor Data Patterns**: Analyze incoming data for protocol signatures
2. **Header Recognition**: AR1AFC (`0xFE 0x81 0xFF 0x55`) vs MAVLink (`0xFE`)
3. **Automatic Switching**: Switch parser and display based on detected protocol

## 🎮 User Interface

### Main Window Layout
```
┌─────────────────────────────────────────────────────────┐
│  GMINS Navigation System GUI                            │
├─────────────────┬─────────────────┬─────────────────────┤
│ 🔌 Connection    │ 📊 Real-time    │ 🎮 System Control   │
│ • Port: COM3    │ • Roll: 5.2°    │ • AR1AFC Mode       │
│ • Baud: 230400  │ • Pitch: -1.8°  │ • MAVLink Mode      │  
│ • Status: OK    │ • Yaw: 87.3°    │ • System Status     │
├─────────────────┼─────────────────┼─────────────────────┤
│ 📈 Plots        │ 🗺️ 3D Attitude  │ 📝 Data Log         │
│ • Attitude      │ • Live 3D view  │ • System messages   │
│ • Position      │ • Horizon       │ • Command history   │
│ • Velocity      │ • Compass       │ • Error log         │
└─────────────────┴─────────────────┴─────────────────────┘
```

### Status Indicators
- 🟢 **Connected**: Successfully connected and receiving data
- 🟡 **Connecting**: Attempting to establish connection
- 🔴 **Disconnected**: No connection or connection lost
- ⚠️ **Error**: Communication error or system fault

## 📊 Data Formats

### AR1AFC Navigation Data
```python
{
    'attitude': {
        'roll': 5.2,      # degrees
        'pitch': -1.8,    # degrees  
        'yaw': 87.3       # degrees
    },
    'gyro': {
        'x': 1.5,         # DPS (degrees per second)
        'y': -2.3,        # DPS
        'z': 0.8          # DPS
    },
    'accel': {
        'x': 0.02,        # g (gravitational units)
        'y': -0.15,       # g
        'z': 0.98         # g
    },
    'temperature': 35.2,  # °C
    'packet_count': 1234
}
```

### MAVLink Navigation Data
```python
{
    'position': {
        'latitude': 34.0522,    # degrees
        'longitude': -118.2437, # degrees
        'altitude_msl': 71.2    # meters
    },
    'velocity': {
        'north': 2.5,     # m/s
        'east': 1.8,      # m/s
        'down': -0.1      # m/s
    },
    'quality': {
        'fix_type': 3,          # 3D GPS fix
        'satellites_visible': 12,
        'hdop': 0.8            # horizontal dilution
    }
}
```

## 🔍 Troubleshooting

### Common Issues

#### Connection Problems
- **Port not found**: Check USB cable and drivers
- **Permission denied**: Run as administrator (Windows) or use `sudo` (Linux)
- **Wrong baud rate**: Try auto-detect or manual protocol selection

#### Data Reception Issues  
- **No data received**: Verify GMINS system is transmitting
- **Garbled data**: Check baud rate and protocol settings
- **Intermittent connection**: Check cable and port stability

#### GUI Issues
- **Blank display**: Ensure PyQt5 is properly installed
- **Slow performance**: Check system resources and data rate
- **Crashes**: Check Python version compatibility (3.7+)

### Debug Mode
```bash
python main.py --debug --verbose
```

### Log Files
- **Application Log**: `logs/gmins_gui.log`
- **Data Log**: `logs/navigation_data.csv`
- **Error Log**: `logs/errors.log`

## 📚 API Reference

### SerialManager Class
```python
manager = SerialManager()
manager.connect('COM3', ProtocolMode.AUTO_DETECT)
manager.send_command('STATUS')
manager.disconnect()
```

### Protocol Parsers
```python
ar1afc_parser = AR1AFCParser()
mavlink_parser = MAVLinkParser()

data = ar1afc_parser.parse_data(raw_bytes)
nav_data = mavlink_parser.parse_data(raw_bytes)
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Make changes and test thoroughly
4. Commit changes: `git commit -m 'Add feature'`
5. Push to branch: `git push origin feature-name`
6. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🆘 Support

- **Issues**: Report bugs and request features on GitHub Issues
- **Documentation**: Check the `docs/` directory for detailed guides
- **Contact**: GMINS Development Team

---

**Version**: 1.0.0  
**Last Updated**: 2025-01-17  
**Status**: ✅ Core functionality complete, advanced features in development