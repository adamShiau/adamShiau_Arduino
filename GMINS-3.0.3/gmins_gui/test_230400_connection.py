#!/usr/bin/env python3
"""
專門測試230400波特率連接穩定性
"""

import serial
import serial.tools.list_ports
import time
import threading


def test_230400_stability(port_name, duration=30):
    """測試230400波特率的連接穩定性"""
    print(f"🔍 測試 {port_name} 在 230400 波特率下的穩定性...")
    print(f"測試時間: {duration} 秒")
    print("=" * 60)
    
    # 嘗試不同的230400波特率設置組合
    test_configs = [
        # 標準設置
        {
            "name": "標準230400設置",
            "baudrate": 230400,
            "timeout": 0.5,
            "write_timeout": 2,
            "inter_byte_timeout": None,
            "xonxoff": False,
            "rtscts": False,
            "dsrdtr": False
        },
        # 更保守的設置
        {
            "name": "保守230400設置",
            "baudrate": 230400,
            "timeout": 1.0,
            "write_timeout": 5,
            "inter_byte_timeout": 0.1,
            "xonxoff": False,
            "rtscts": False,
            "dsrdtr": False
        },
        # 無超時設置
        {
            "name": "無超時230400設置",
            "baudrate": 230400,
            "timeout": None,
            "write_timeout": None,
            "inter_byte_timeout": None,
            "xonxoff": False,
            "rtscts": False,
            "dsrdtr": False
        }
    ]
    
    best_config = None
    best_time = 0
    
    for config in test_configs:
        print(f"\n🧪 測試配置: {config['name']}")
        print("-" * 40)
        
        try:
            ser = serial.Serial(
                port=port_name,
                **{k: v for k, v in config.items() if k != 'name'}
            )
            
            print(f"✅ 成功建立連接")
            print(f"   實際波特率: {ser.baudrate}")
            print(f"   超時設置: {ser.timeout}")
            print(f"   寫入超時: {ser.write_timeout}")
            
            # 測試連接穩定性
            start_time = time.time()
            data_received = 0
            last_activity = start_time
            consecutive_errors = 0
            max_gap = 0
            
            while time.time() - start_time < duration:
                try:
                    current_time = time.time()
                    
                    # 檢查連接狀態
                    if not ser.is_open:
                        print(f"❌ 連接在 {current_time - start_time:.1f}s 後丟失")
                        break
                    
                    # 讀取數據
                    if ser.in_waiting > 0:
                        data = ser.read(ser.in_waiting)
                        if data:
                            data_received += len(data)
                            gap = current_time - last_activity
                            max_gap = max(max_gap, gap)
                            last_activity = current_time
                            consecutive_errors = 0
                            
                            # 顯示接收到的數據（前32字節）
                            hex_data = ' '.join([f'{b:02X}' for b in data[:32]])
                            if len(data) > 32:
                                hex_data += f" ... (+{len(data)-32} more bytes)"
                            print(f"📨 [{current_time - start_time:6.1f}s] 收到 {len(data)} 字節: {hex_data}")
                    
                    # 每10秒發送一個測試命令
                    if int(current_time - start_time) % 10 == 0 and (current_time - start_time) % 1 < 0.1:
                        try:
                            test_cmd = "STATUS\n"
                            ser.write(test_cmd.encode())
                            print(f"📤 [{current_time - start_time:6.1f}s] 發送測試命令: {test_cmd.strip()}")
                        except Exception as e:
                            print(f"❌ 寫入錯誤: {e}")
                            consecutive_errors += 1
                    
                    # 檢查連續錯誤
                    if consecutive_errors >= 3:
                        print(f"❌ 連續錯誤過多，停止測試")
                        break
                    
                    time.sleep(0.01)  # 10ms
                    
                except serial.SerialException as e:
                    print(f"❌ 串口異常 [{time.time() - start_time:.1f}s]: {e}")
                    consecutive_errors += 1
                    if consecutive_errors >= 3:
                        break
                except Exception as e:
                    print(f"❌ 意外錯誤 [{time.time() - start_time:.1f}s]: {e}")
                    consecutive_errors += 1
                    if consecutive_errors >= 3:
                        break
            
            # 關閉連接
            elapsed = time.time() - start_time
            if ser.is_open:
                ser.close()
            
            # 統計結果
            print(f"\n📊 {config['name']} 測試結果:")
            print(f"   - 持續時間: {elapsed:.1f}s / {duration}s")
            print(f"   - 成功率: {(elapsed/duration)*100:.1f}%")
            print(f"   - 數據接收: {data_received} 字節")
            print(f"   - 最大間隔: {max_gap:.2f}s")
            print(f"   - 連續錯誤: {consecutive_errors}")
            
            if elapsed > best_time:
                best_time = elapsed
                best_config = config
                
        except Exception as e:
            print(f"❌ 無法建立連接: {e}")
            continue
    
    # 總結
    print(f"\n🏆 最佳配置結果:")
    if best_config:
        print(f"   配置: {best_config['name']}")
        print(f"   持續時間: {best_time:.1f}s")
        print(f"   成功率: {(best_time/duration)*100:.1f}%")
        
        if best_time >= duration * 0.8:
            print("✅ 找到穩定的230400波特率設置")
            return best_config
        else:
            print("⚠️ 230400波特率不夠穩定")
    else:
        print("❌ 所有230400配置都失敗")
    
    return None


def suggest_alternative_baud_rates(port_name):
    """建議替代波特率"""
    print(f"\n🔄 測試替代波特率...")
    print("-" * 40)
    
    # 測試常用波特率
    alternative_bauds = [115200, 460800, 921600, 57600, 38400, 19200, 9600]
    working_bauds = []
    
    for baud in alternative_bauds:
        try:
            ser = serial.Serial(port_name, baud, timeout=2)
            
            # 簡單測試
            time.sleep(0.5)
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                working_bauds.append((baud, len(data)))
                print(f"✅ {baud} 波特率: 收到 {len(data)} 字節數據")
            else:
                print(f"⚪ {baud} 波特率: 連接成功但無數據")
                working_bauds.append((baud, 0))
            
            ser.close()
            
        except Exception as e:
            print(f"❌ {baud} 波特率: {e}")
    
    if working_bauds:
        print(f"\n💡 建議的替代波特率:")
        # 按數據量排序
        working_bauds.sort(key=lambda x: x[1], reverse=True)
        for baud, data_count in working_bauds[:3]:
            print(f"   - {baud} 波特率 (數據量: {data_count} 字節)")
    else:
        print("❌ 沒有找到可用的替代波特率")
    
    return working_bauds


def main():
    """主函數"""
    print("🚀 GMINS 230400 波特率連接診斷工具")
    print("="*60)
    
    # 掃描端口
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("❌ 未檢測到串口")
        return
    
    print(f"🔍 檢測到 {len(ports)} 個串口:")
    for i, port in enumerate(ports):
        print(f"  {i+1}. {port.device} - {port.description}")
    
    # 測試每個端口的230400穩定性
    for port in ports:
        print(f"\n{'='*20} 測試 {port.device} {'='*20}")
        
        # 測試230400波特率
        best_config = test_230400_stability(port.device, duration=15)
        
        if not best_config:
            # 如果230400不行，建議替代波特率
            suggest_alternative_baud_rates(port.device)
    
    print(f"\n💡 故障排除建議:")
    print("   1. 如果230400不穩定，嘗試115200或460800")
    print("   2. 檢查USB電纜品質")
    print("   3. 嘗試不同的USB端口")
    print("   4. 關閉其他串口應用程序")
    print("   5. 檢查設備是否支持該波特率")
    print("   6. 考慮使用USB轉串口適配器")


if __name__ == "__main__":
    main()