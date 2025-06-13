# 1 "C:\\Users\\user\\Documents\\Arduino\\test\\test.ino"
# 2 "C:\\Users\\user\\Documents\\Arduino\\test\\test.ino" 2
# 3 "C:\\Users\\user\\Documents\\Arduino\\test\\test.ino" 2
# 4 "C:\\Users\\user\\Documents\\Arduino\\test\\test.ino" 2
# 5 "C:\\Users\\user\\Documents\\Arduino\\test\\test.ino" 2
# 13 "C:\\Users\\user\\Documents\\Arduino\\test\\test.ino"
void getPX4Time(bool is_print=true);
void LLH2ENU(double lat, double lon, float hei, float &pe, float &pn, float &pu);

double lat_0 = 25.0131068, lon_0 = 121.2217855;
float alt_0 = 0;

void setup() {
  SerialUSB.begin(115200);
  while(!SerialUSB);
  Serial1.begin(115200);
  delay(100);
}


void loop() {
  // if (PIXHAWK_SERIAL.available()){
  //   Serial.print(PIXHAWK_SERIAL.available());
  //   Serial.print(' ');
  //   Serial.print(PIXHAWK_SERIAL.read());
  //   Serial.print(' ');
  // }

  // getPX4HomePosition();
  getPX4Time();

  // double lat = 25.0131068, lon = 121.2217855;
  // float pe, pn, pu, hei = 80.5;
  // LLH2ENU(lat, lon, hei, pe, pn, pu);
  // Serial.print(pe); Serial.print(' ');
  // Serial.print(pn); Serial.print(' ');
  // Serial.print(pu); Serial.println(' ');
  // delay(1000);
  // delay(1000);
}

void getPX4HomePosition(bool is_print) {
  mavlink_message_t msg_send, msg_recv;
  uint8_t buf[(255 /*|< Maximum payload length*/ + ((9 /*|< Length of core header (of the comm. layer)*/ + 1) /*|< Length of all header bytes, including core and stx*/ + 2) + 13) /*|< Maximum packet length*/];

  // 發送 GET_HOME_POSITION 指令
  mavlink_msg_command_long_pack(1, 200, &msg_send,
                                1, 0,
                                MAV_CMD_GET_HOME_POSITION, 0,
                                0, 0, 0, 0, 0, 0, 0);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_send);
  Serial1.write(buf, len);

  unsigned long start_time = millis();
  bool received = false;

  // 等待回應，最多等待 100 毫秒
  while (millis() - start_time < 100) {
    if (Serial1.available() > 0) {
      uint8_t c = Serial1.read();
      mavlink_status_t status;
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg_recv, &status)) {
        if (msg_recv.msgid == 242) {
          mavlink_home_position_t home;
          mavlink_msg_home_position_decode(&msg_recv, &home);

          // 更新 Home Position
          lat_0 = home.latitude * 1e-7; // PX4 Home Position 緯度
          lon_0 = home.longitude * 1e-7; // PX4 Home Position 經度
          alt_0 = home.altitude * 1e-3; // PX4 Home Position 高度 (m)

          if (is_print) {
            SerialUSB.print("Home Position: Lat = "); SerialUSB.print(lat_0, 7);
            SerialUSB.print(", Lon = "); SerialUSB.print(lon_0, 7);
            SerialUSB.print(", Alt = "); SerialUSB.println(alt_0, 3);
          }
          received = true;
          break;
        }
      }
    }
  }

  if (!received && is_print) {
    SerialUSB.println("No HOME_POSITION response received.");
  }
}


void getPX4Time(bool is_print) {
  mavlink_message_t msg_send, msg_recv;
  mavlink_timesync_t ts_send, ts_recv;

  // 初始化時間戳
  ts_send.tc1 = 0;
  ts_send.ts1 = micros();

  // 編碼並發送 TIMESYNC 消息
  mavlink_msg_timesync_encode(1, 200, &msg_send, &ts_send);
  uint8_t buffer[(255 /*|< Maximum payload length*/ + ((9 /*|< Length of core header (of the comm. layer)*/ + 1) /*|< Length of all header bytes, including core and stx*/ + 2) + 13) /*|< Maximum packet length*/];
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg_send);
  Serial1.write(buffer, len);

  unsigned long start_time = millis();
  bool received = false;

  // 等待回應，最多等待 100 毫秒
  while (millis() - start_time < 100) {
    if (Serial1.available() > 0) {
      uint8_t c = Serial1.read();
      mavlink_status_t status;
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg_recv, &status)) {
        if (msg_recv.msgid == 111) {
          mavlink_msg_timesync_decode(&msg_recv, &ts_recv);
          received = true;
          if (is_print) {
            SerialUSB.print("ts1: ");
            SerialUSB.println(ts_recv.ts1 * 0.001); // 微秒轉毫秒
            SerialUSB.print("tc1: ");
            SerialUSB.println(ts_recv.tc1 * 0.001); // 微秒轉毫秒
          }
          break;
        }
      }
    }
  }

  if (!received && is_print) {
    SerialUSB.println("No TIMESYNC response received.");
  }
}

void LLH2ENU(double lat, double lon, float hei, float &pe, float &pn, float &pu){
  pe = (lon - lon_0) * (111319.0 * cos(lat_0 * 0.017453292519943295769236907684886));
  pn = (lat - lat_0) * (111319.0);
  pu = hei - alt_0;
}
