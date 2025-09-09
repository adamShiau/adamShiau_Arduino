#!/usr/bin/env python3
"""
Serial1 測試發送器
用於測試 GMINS 系統的 Serial1 命令接收功能

使用方式:
1. 安裝 pyserial: pip install pyserial
2. 執行: python test_serial1_sender.py
3. 選擇正確的 COM port（通常是 Serial1 對應的 port）
4. 觀察 GMINS 系統的 log 輸出
"""

import serial
import serial.tools.list_ports
import time
import sys

# 全局變量：追蹤當前協議模式
current_protocol_mode = "AR1AFC"  # 預設模式

def list_available_ports():
    """列出所有可用的串口"""
    print("🔍 掃描可用的 COM ports...")
    ports = list(serial.tools.list_ports.comports())
    
    if not ports:
        print("❌ 沒有找到可用的 COM ports")
        return []
    
    print(f"📡 找到 {len(ports)} 個 COM ports:")
    for i, port in enumerate(ports):
        print(f"  {i+1}. {port.device} - {port.description}")
    
    return ports

def get_baud_rate_for_command(cmd):
    """根據命令和當前協議模式決定波特率"""
    global current_protocol_mode
    cmd_upper = cmd.upper()
    
    # 協議切換命令使用固定 baud rate
    if cmd_upper == "MAVLINK":
        return 230400  # MAVLINK 切換命令
    elif cmd_upper == "AR1AFC":
        return 460800  # AR1AFC 切換命令
    else:
        # 其他命令根據當前協議模式決定 baud rate
        if current_protocol_mode == "MAVLINK":
            return 460800  # MAVLINK 模式下，其他命令用 460800
        else:  # AR1AFC 模式
            return 230400  # AR1AFC 模式下，其他命令用 230400

def update_protocol_mode(cmd):
    """更新當前協議模式"""
    global current_protocol_mode
    cmd_upper = cmd.upper()
    if cmd_upper == "MAVLINK":
        current_protocol_mode = "MAVLINK"
        print(f"🔄 協議模式已切換到: {current_protocol_mode} (其他命令將使用 460800 baud)")
    elif cmd_upper == "AR1AFC":
        current_protocol_mode = "AR1AFC"  
        print(f"🔄 協議模式已切換到: {current_protocol_mode} (其他命令將使用 230400 baud)")

def send_command_with_baud_rate(port_name, cmd):
    """用正確的波特率發送單一命令"""
    required_baud = get_baud_rate_for_command(cmd)
    
    try:
        print(f"🔗 為命令 '{cmd}' 連接到 {port_name} ({required_baud} baud)...")
        ser = serial.Serial(
            port=port_name,
            baudrate=required_baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        
        time.sleep(0.1)  # 短暫等待連接穩定
        
        # 發送命令
        cmd_with_newline = cmd + "\n"
        ser.write(cmd_with_newline.encode('utf-8'))
        print(f"✅ 發送: '{cmd}' ({len(cmd_with_newline)} bytes at {required_baud} baud)")
        
        # 如果是協議切換命令，更新模式
        update_protocol_mode(cmd)
        
        ser.close()
        time.sleep(1)  # 命令間隔
        
    except Exception as e:
        print(f"❌ 發送 '{cmd}' 失敗: {e}")
        if 'ser' in locals() and ser.is_open:
            ser.close()

def send_test_commands(port_name):
    """發送測試命令 - 智能協議模式切換"""
    global current_protocol_mode
    test_commands = [
        "TEST",       # 使用當前協議模式的 baud rate
        "AR1AFC",     # 460800 baud (切換命令)
        "HELLO",      # AR1AFC 模式: 230400 baud
        "MAVLINK",    # 230400 baud (切換命令)  
        "STATUS"      # MAVLINK 模式: 460800 baud
    ]
    
    print("📤 開始發送測試命令...")
    print(f"🎯 當前協議模式: {current_protocol_mode}")
    print("💡 智能模式切換邏輯:")
    print("   - AR1AFC 切換命令: 460800 baud → 切換後其他命令用 230400 baud")
    print("   - MAVLINK 切換命令: 230400 baud → 切換後其他命令用 460800 baud")
    print("   - 其他命令: 根據當前協議模式自動選擇 baud rate")
    
    for cmd in test_commands:
        send_command_with_baud_rate(port_name, cmd)
    
    print("🎉 測試命令發送完成!")

def manual_mode(port_name):
    """手動命令輸入模式"""
    global current_protocol_mode
    print("\n🎮 進入手動模式")
    print("輸入命令並按 Enter 發送，輸入 'quit' 退出")
    print("建議測試: AR1AFC, MAVLINK, TEST, STATUS, HELLO")
    print(f"🎯 當前協議模式: {current_protocol_mode}")
    print("💡 智能模式切換邏輯:")
    print("   - AR1AFC 命令: 460800 baud → 切換後其他命令用 230400 baud")
    print("   - MAVLINK 命令: 230400 baud → 切換後其他命令用 460800 baud")
    print("   - 其他命令: 根據當前協議模式自動選擇")
    print(f"   - 目前其他命令會使用: {'460800' if current_protocol_mode == 'MAVLINK' else '230400'} baud")
    
    while True:
        try:
            cmd = input(f"\n[{current_protocol_mode}模式]> ").strip()
            
            if cmd.lower() == 'quit':
                break
            
            if cmd.lower() == 'status' and len(cmd.split()) == 1:
                # 特殊命令：顯示當前狀態
                print(f"🎯 當前協議模式: {current_protocol_mode}")
                print(f"📡 其他命令當前使用: {'460800' if current_protocol_mode == 'MAVLINK' else '230400'} baud")
                continue
            
            if not cmd:
                continue
            
            # 使用對應波特率發送命令
            send_command_with_baud_rate(port_name, cmd)
            
        except KeyboardInterrupt:
            print("\n👋 手動模式退出")
            break
        except Exception as e:
            print(f"❌ 發送錯誤: {e}")

def main():
    print("🚀 Serial1 測試發送器")
    print("=" * 50)
    
    # 列出可用端口
    ports = list_available_ports()
    if not ports:
        return
    
    # 選擇端口
    while True:
        try:
            choice = input(f"\n請選擇 COM port (1-{len(ports)}): ").strip()
            port_index = int(choice) - 1
            
            if 0 <= port_index < len(ports):
                selected_port = ports[port_index].device
                break
            else:
                print(f"❌ 請輸入 1 到 {len(ports)} 之間的數字")
                
        except ValueError:
            print("❌ 請輸入有效的數字")
        except KeyboardInterrupt:
            print("\n👋 程序退出")
            return
    
    global current_protocol_mode
    print(f"📌 選擇的端口: {selected_port}")
    print(f"\n🎯 當前協議模式: {current_protocol_mode}")
    print("💡 智能模式切換機制:")
    print("   - AR1AFC 切換命令: 固定使用 460800 baud")
    print("     └─ 切換成功後，其他命令自動改用 230400 baud")
    print("   - MAVLINK 切換命令: 固定使用 230400 baud")
    print("     └─ 切換成功後，其他命令自動改用 460800 baud")
    print("   - 好處: 可以在任何協議模式下測試其他命令的接收")
    
    try:
        # 選擇測試模式
        print("\n📋 測試模式選擇:")
        print("1. 自動發送測試命令")
        print("2. 手動輸入命令")
        print("3. 兩個都做")
        
        mode = input("請選擇模式 (1/2/3): ").strip()
        
        if mode in ['1', '3']:
            send_test_commands(selected_port)
        
        if mode in ['2', '3']:
            manual_mode(selected_port)
            
        if mode not in ['1', '2', '3']:
            print("❌ 無效選擇，執行自動測試")
            send_test_commands(selected_port)
    
    except KeyboardInterrupt:
        print("\n👋 程序被中斷")

if __name__ == "__main__":
    main()