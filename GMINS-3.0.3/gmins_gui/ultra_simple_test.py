#!/usr/bin/env python3
"""
Ultra Simple GMINS Connection Test (No GUI Dependencies)
Just test connection and show data in terminal
"""

import serial
import serial.tools.list_ports
import time
import threading
import sys


class SimpleConnectionTest:
    def __init__(self):
        self.serial_port = None
        self.is_connected = False
        self.stop_reading = False
        self.total_bytes = 0
        self.data_samples = []
        
    def get_ports(self):
        """Get available ports"""
        ports = []
        try:
            for port in serial.tools.list_ports.comports():
                ports.append({
                    'device': port.device,
                    'description': port.description
                })
        except Exception as e:
            print(f"❌ Error scanning ports: {e}")
        return ports
    
    def connect(self, port_name, baud_rate=230400):
        """Connect to port"""
        try:
            print(f"🔗 Connecting to {port_name} at {baud_rate} baud...")
            
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            # Give connection time to stabilize
            time.sleep(0.5)
            
            if self.serial_port.is_open:
                self.is_connected = True
                self.total_bytes = 0
                self.data_samples = []
                print(f"✅ Connected successfully!")
                print(f"   Port: {self.serial_port.port}")
                print(f"   Baud rate: {self.serial_port.baudrate}")
                print(f"   Timeout: {self.serial_port.timeout}s")
                return True
            else:
                print(f"❌ Port failed to open")
                return False
                
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect"""
        self.stop_reading = True
        if self.serial_port:
            try:
                self.serial_port.close()
                print("📡 Disconnected")
            except Exception as e:
                print(f"⚠️ Error during disconnect: {e}")
        self.serial_port = None
        self.is_connected = False
    
    def read_data_test(self, duration=30):
        """Test reading data for specified duration"""
        if not self.is_connected:
            print("❌ Not connected!")
            return
        
        print(f"\n📊 Starting {duration}-second data reception test...")
        print("=" * 60)
        
        start_time = time.time()
        self.stop_reading = False
        last_report = start_time
        
        try:
            while time.time() - start_time < duration and not self.stop_reading:
                current_time = time.time()
                
                # Check if still connected
                if not self.serial_port.is_open:
                    print(f"\n❌ Connection lost at {current_time - start_time:.1f}s")
                    break
                
                # Read data
                try:
                    if self.serial_port.in_waiting > 0:
                        data = self.serial_port.read(self.serial_port.in_waiting)
                        if data:
                            self.total_bytes += len(data)
                            
                            # Keep some data samples for analysis
                            if len(self.data_samples) < 5:
                                hex_sample = ' '.join([f'{b:02X}' for b in data[:16]])
                                if len(data) > 16:
                                    hex_sample += f" ... (+{len(data)-16} more)"
                                self.data_samples.append({
                                    'time': current_time - start_time,
                                    'bytes': len(data),
                                    'sample': hex_sample
                                })
                                print(f"📨 [{current_time - start_time:6.1f}s] {len(data):4d} bytes: {hex_sample}")
                            
                            # Report every 5 seconds
                            if current_time - last_report >= 5:
                                rate = self.total_bytes / (current_time - start_time)
                                print(f"📈 [{current_time - start_time:6.1f}s] Total: {self.total_bytes} bytes, Rate: {rate:.1f} bytes/sec")
                                last_report = current_time
                
                except serial.SerialException as e:
                    print(f"❌ Serial read error: {e}")
                    break
                except Exception as e:
                    print(f"❌ Unexpected error: {e}")
                    break
                
                # Small delay
                time.sleep(0.01)
            
        except KeyboardInterrupt:
            print(f"\n🛑 Test interrupted by user")
            self.stop_reading = True
        
        # Final results
        elapsed = time.time() - start_time
        self.show_results(elapsed)
    
    def show_results(self, elapsed):
        """Show test results"""
        print(f"\n" + "=" * 60)
        print(f"📊 Test Results Summary:")
        print(f"   Duration: {elapsed:.1f} seconds")
        print(f"   Total bytes received: {self.total_bytes}")
        
        if elapsed > 0:
            avg_rate = self.total_bytes / elapsed
            print(f"   Average rate: {avg_rate:.1f} bytes/sec")
        
        if self.data_samples:
            print(f"\n📋 Data Samples ({len(self.data_samples)} samples):")
            for i, sample in enumerate(self.data_samples, 1):
                print(f"   {i}. [{sample['time']:6.1f}s] {sample['bytes']} bytes: {sample['sample']}")
        else:
            print(f"⚠️ No data samples collected")
        
        # Connection stability
        if elapsed >= 25:  # If test ran for most of the time
            print(f"✅ Connection appears stable")
        elif self.total_bytes > 0:
            print(f"⚠️ Connection was unstable but received some data")
        else:
            print(f"❌ No data received - connection or device issue")


def main():
    print("🚀 Ultra Simple GMINS Connection Test")
    print("=" * 60)
    
    tester = SimpleConnectionTest()
    
    try:
        # Scan ports
        print("🔍 Scanning for available ports...")
        ports = tester.get_ports()
        
        if not ports:
            print("❌ No COM ports detected!")
            print("💡 Make sure your device is connected and drivers are installed")
            return
        
        print(f"✅ Found {len(ports)} ports:")
        for i, port in enumerate(ports, 1):
            print(f"   {i}. {port['device']} - {port['description']}")
        
        # Get user choice
        if len(ports) == 1:
            selected_port = ports[0]['device']
            print(f"\n🎯 Auto-selecting only port: {selected_port}")
        else:
            while True:
                try:
                    choice = input(f"\nSelect port (1-{len(ports)}): ").strip()
                    port_idx = int(choice) - 1
                    if 0 <= port_idx < len(ports):
                        selected_port = ports[port_idx]['device']
                        break
                    else:
                        print("❌ Invalid choice, try again")
                except (ValueError, KeyboardInterrupt):
                    print("\n👋 Cancelled")
                    return
        
        # Get baud rate
        print(f"\n📡 Available baud rates:")
        baud_rates = [115200, 230400, 460800]
        for i, baud in enumerate(baud_rates, 1):
            print(f"   {i}. {baud}")
        
        selected_baud = 230400  # Default
        try:
            baud_choice = input(f"Select baud rate (1-3, default=2 for 230400): ").strip()
            if baud_choice:
                baud_idx = int(baud_choice) - 1
                if 0 <= baud_idx < len(baud_rates):
                    selected_baud = baud_rates[baud_idx]
        except ValueError:
            pass
        
        print(f"\n🎯 Using: {selected_port} at {selected_baud} baud")
        
        # Connect
        if tester.connect(selected_port, selected_baud):
            print(f"\n🧪 Starting data reception test...")
            print(f"Press Ctrl+C to stop early")
            
            # Test for 30 seconds
            tester.read_data_test(30)
        
    except KeyboardInterrupt:
        print(f"\n👋 Test cancelled by user")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tester.disconnect()


if __name__ == "__main__":
    main()