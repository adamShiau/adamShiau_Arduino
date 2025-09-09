1.未來需要移除內容:SYC`ITransport* serial_transport_`：傳輸層介面（註記：**已棄用**，改由 framer 處理，表示傳輸抽象已上移到更通用的封裝）。
好的！讓我教你如何將 MTI 的 YAW 數據加入到 GPS_INPUT 封包中：

  🧭 MTI YAW 整合步驟

  1. 找到 GPS_INPUT 封包設定位置

  在 MAVLinkProtocol.cpp 的 sendGNSSBasedData() 函數中：

  // 目前的設定
  gps_input.yaw = 0;  // 暫時不使用 YAW

  2. MTI YAW 數據來源

  你需要從以下其中一個地方取得 MTI YAW：

  選項 A：從 NavigationState
  // 使用姿態四元數計算 YAW
  float yaw_rad = atan2(2.0f * (nav_state.quat_w * nav_state.quat_z + nav_state.quat_x * nav_state.quat_y),
                        1.0f - 2.0f * (nav_state.quat_y * nav_state.quat_y + nav_state.quat_z * nav_state.quat_z));
  gps_input.yaw = (uint16_t)(yaw_rad * 57.2958f * 100);  // 轉換為度×100

  選項 B：從 IMUData（如果有直接 YAW）
  // 如果 IMUData 有 yaw 欄位
  gps_input.yaw = (uint16_t)(imu_data.yaw * 100);  // 度×100

  3. MAVLink GPS_INPUT 的 YAW 格式

  - 單位：度×100 (centidegrees)
  - 範圍：0-35999 (0-359.99度)
  - 型態：uint16_t

  4. 修改位置

  在 src/protocol/MAVLinkProtocol.cpp 約第120行：
  // 找到這行
  gps_input.yaw = 0;  // 暫時不使用 YAW

  // 改為
  gps_input.yaw = (uint16_t)(你的YAW計算 * 100);  // MTI YAW (度×100)

  你想用哪種方式取得 MTI 的 YAW 數據？四元數計算還是直接從 IMU 欄位？