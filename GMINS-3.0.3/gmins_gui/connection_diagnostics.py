#!/usr/bin/env python3
"""
Serial Connection Diagnostics Tool
Helps diagnose why connections are dropping automatically
"""

import serial
import serial.tools.list_ports
import time
import threading


def test_basic_connection(port_name, baud_rate=230400, test_duration=10):
    """Test basic connection stability"""
    print(f"🔍 Testing connection to {port_name} at {baud_rate} baud for {test_duration} seconds...")
    print("=" * 60)
    
    try:
        # Open serial port with same settings as GUI
        ser = serial.Serial(
            port=port_name,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5,
            write_timeout=2,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )
        
        print(f"✅ Successfully opened {port_name}")
        print(f"   - Baud rate: {ser.baudrate}")
        print(f"   - Timeout: {ser.timeout}s")
        print(f"   - Is open: {ser.is_open}")
        print()
        
        # Test connection stability
        start_time = time.time()
        data_count = 0
        error_count = 0
        
        while time.time() - start_time < test_duration:
            try:
                # Check if still connected
                if not ser.is_open:
                    print("❌ Connection lost - port closed unexpectedly")
                    break
                
                # Try to read data
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    data_count += len(data)
                    print(f"📨 Received {len(data)} bytes: {' '.join([f'{b:02X}' for b in data])}")
                
                # Test writing (send a simple command)
                if int(time.time()) % 5 == 0:  # Every 5 seconds
                    try:
                        ser.write(b"PING\n")
                        print("📤 Sent PING command")
                    except Exception as e:
                        print(f"❌ Write error: {e}")
                        error_count += 1
                
                time.sleep(0.1)
                
            except serial.SerialException as e:
                print(f"❌ Serial exception: {e}")
                error_count += 1
                break
            except Exception as e:
                print(f"❌ Unexpected error: {e}")
                error_count += 1
                break
        
        # Close connection
        if ser.is_open:
            ser.close()
            print("✅ Connection closed normally")
        
        # Summary
        elapsed = time.time() - start_time
        print(f"\n📊 Test Results:")
        print(f"   - Duration: {elapsed:.1f} seconds")
        print(f"   - Data received: {data_count} bytes")
        print(f"   - Errors: {error_count}")
        print(f"   - Success rate: {((elapsed/test_duration)*100):.1f}%")
        
        if error_count == 0 and elapsed >= test_duration * 0.9:
            print("✅ Connection appears stable")
        else:
            print("⚠️ Connection has stability issues")
            
        return error_count == 0
        
    except Exception as e:
        print(f"❌ Failed to open connection: {e}")
        return False


def diagnose_port_conflicts(port_name):
    """Check for port conflicts and permissions"""
    print(f"🔍 Diagnosing port conflicts for {port_name}...")
    print("=" * 60)
    
    # Check if port exists
    available_ports = [p.device for p in serial.tools.list_ports.comports()]
    if port_name not in available_ports:
        print(f"❌ Port {port_name} not found in available ports")
        print(f"Available ports: {available_ports}")
        return False
    
    print(f"✅ Port {port_name} is available")
    
    # Try different baud rates
    baud_rates = [9600, 115200, 230400, 460800, 921600]
    successful_bauds = []
    
    for baud in baud_rates:
        try:
            ser = serial.Serial(port_name, baud, timeout=1)
            ser.close()
            successful_bauds.append(baud)
            print(f"✅ Baud rate {baud} - OK")
        except Exception as e:
            print(f"❌ Baud rate {baud} - Failed: {e}")
    
    if successful_bauds:
        print(f"\n✅ Working baud rates: {successful_bauds}")
        return True
    else:
        print(f"\n❌ No baud rates work - possible hardware issue")
        return False


def check_system_resources():
    """Check system resources that might affect serial communication"""
    print(f"🔍 Checking system resources...")
    print("=" * 60)
    
    try:
        import psutil
        
        # Check CPU usage
        cpu_percent = psutil.cpu_percent(interval=1)
        print(f"CPU Usage: {cpu_percent}%")
        
        # Check memory usage
        memory = psutil.virtual_memory()
        print(f"Memory Usage: {memory.percent}%")
        
        # Check disk I/O
        disk_io = psutil.disk_io_counters()
        print(f"Disk I/O: Read={disk_io.read_bytes}, Write={disk_io.write_bytes}")
        
        if cpu_percent > 80:
            print("⚠️ High CPU usage might affect serial communication")
        if memory.percent > 90:
            print("⚠️ High memory usage might affect performance")
            
    except ImportError:
        print("📦 Install 'psutil' for detailed system resource checking")
        print("   pip install psutil")


def main():
    """Main diagnostic function"""
    print("🚀 GMINS Serial Connection Diagnostics")
    print("="*70)
    print()
    
    # Scan for ports
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("❌ No serial ports detected")
        return
    
    print(f"🔍 Found {len(ports)} serial ports:")
    for i, port in enumerate(ports):
        print(f"  {i+1}. {port.device} - {port.description}")
    print()
    
    # Test each port
    for port in ports:
        print(f"\n{'='*20} Testing {port.device} {'='*20}")
        
        # Basic conflict check
        conflicts_ok = diagnose_port_conflicts(port.device)
        
        if conflicts_ok:
            # Stability test
            stable = test_basic_connection(port.device, test_duration=5)
            
            if not stable:
                print(f"⚠️ {port.device} has connection issues")
            else:
                print(f"✅ {port.device} appears to be working correctly")
    
    print(f"\n{'='*20} System Check {'='*20}")
    check_system_resources()
    
    print("\n💡 Common causes of automatic disconnection:")
    print("   1. Port already in use by another application")
    print("   2. Incorrect baud rate for the device")
    print("   3. Hardware flow control conflicts")
    print("   4. USB power management settings")
    print("   5. Driver issues")
    print("   6. Cable or connector problems")
    
    print("\n🔧 Troubleshooting steps:")
    print("   1. Close all other serial applications")
    print("   2. Try different baud rates")
    print("   3. Disable USB power management")
    print("   4. Update device drivers")
    print("   5. Try a different USB cable/port")


if __name__ == "__main__":
    main()