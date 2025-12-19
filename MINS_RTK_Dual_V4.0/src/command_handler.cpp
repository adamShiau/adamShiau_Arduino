#include "command_handler.h"
#include "packet_protocol.h"
#include "gnss_control.h"
#include <wiring_private.h>

// ---- 數據輸出控制陣列 ----
#define MAX_OUTPUTS 10
DataOutput data_outputs[MAX_OUTPUTS];

// 輸出類型索引
#define OUTPUT_HEADING 0
#define OUTPUT_POS_HEADING 1



// ---- Command Processing Functions ----
void processCommand1(int value, byte channel) {
  // 命令 1：設定 heading 輸出頻率
  // 封包格式: FA FF 05 00 01 [4-byte float heading] [checksum]
  // 範例 heading 51.25°: FA FF 05 00 01 00 00 4D 42 45

  // 使用 channel 作為輸出槽索引
  if (channel < MAX_OUTPUTS) {
    if (value > 0) {
      data_outputs[channel].enabled = true;
      data_outputs[channel].interval_ms = 1000 / value;
      data_outputs[channel].last_send_time = millis();
      data_outputs[channel].packet_type = PKT_HEADING;
      data_outputs[channel].channel = channel;

      Serial.print("Heading output enabled: ");
      Serial.print(value);
      Serial.println("Hz");
    } else {
      data_outputs[channel].enabled = false;
      Serial.println("Heading output disabled");
    }
  }
}

// 統一的數據輸出處理函數
void processDataOutputs() {
  extern NmeaParser gps_parser;
  extern Uart Serial1;  // 使用 myUART 中定義的 Serial1

  GpsData* gps = gps_parser.getData();
  uint8_t packet_buffer[128];  // 足夠大的緩衝區
  uint16_t packet_size;
  uint32_t current_time = millis();

  for (int i = 0; i < MAX_OUTPUTS; i++) {
    DataOutput* output = &data_outputs[i];


    if (output->enabled && (current_time - output->last_send_time >= output->interval_ms)) {
      bool packet_created = false;
      bool packet_created_UTC = false;
      uint8_t gps_status = PacketProtocol::determineGpsStatus(gps);

      switch (output->packet_type) {
        case PKT_HEADING:
          packet_created = PacketProtocol::createHeadingPacket(gps->heading, packet_buffer, &packet_size, gps_status);
          break;

        case PKT_POS_HEADING:
          packet_created = PacketProtocol::createPosHeadingPacket(gps->latitude, gps->longitude, gps->altitude, gps->heading, packet_buffer, &packet_size, gps_status);
          break;

        case PKT_POSITION:
          packet_created = PacketProtocol::createPositionPacket(gps->latitude, gps->longitude, gps->altitude, packet_buffer, &packet_size, gps_status);
          break;

        case PKT_MOTION:
          packet_created = PacketProtocol::createMotionPacket(gps->speed, gps->heading, packet_buffer, &packet_size, gps_status);
          break;

        case PKT_SATELLITES:
          packet_created = PacketProtocol::createSatellitePacket(gps->satellites_used, gps->satellites_visible, gps->hdop, gps->fix_type, packet_buffer, &packet_size, gps_status);
          break;

        case PKT_TIME:
          packet_created = PacketProtocol::createTimePacket(gps->hour, gps->minute, gps->second, gps->day, gps->month, gps->year, packet_buffer, &packet_size, gps_status);
          break;

        case PKT_ALL_GNSS:
          packet_created = PacketProtocol::createAllGnssPacket(gps, packet_buffer, &packet_size);
          break;
      }

      if (packet_created) {
        Serial1.write(packet_buffer, packet_size);
        output->last_send_time = current_time;

        // Debug: Print packet contents
        Serial.print("Packet sent - Type: 0x");
        Serial.print(output->packet_type, HEX);
        Serial.print(" Size: ");
        Serial.print(packet_size);
        Serial.print(" Data: ");
        for (uint16_t j = 0; j < packet_size; j++) {
          if (packet_buffer[j] < 0x10) Serial.print("0");
          Serial.print(packet_buffer[j], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }
    }
  }
}

void processCommand2(int value, byte channel) {
  // 命令 2：設定位置+高精度時間輸出頻率
  // 封包格式: FA FF 1E 00 02 [8-byte lat] [8-byte lon] [4-byte alt] [9-byte UTC time+ms] [checksum]
  // 範例: FA FF 1E 00 02 [lat] [lon] [alt] [hour min sec ms day month year] [checksum] (36 bytes total)

  // 使用 channel 作為輸出槽索引，覆蓋該通道的設定
  if (channel < MAX_OUTPUTS) {
    if (value > 0) {
      data_outputs[channel].enabled = true;
      data_outputs[channel].interval_ms = 1000 / value;
      data_outputs[channel].last_send_time = millis();
      data_outputs[channel].packet_type = PKT_POSITION;
      data_outputs[channel].channel = channel;

      Serial.print("Position+Time output enabled: ");
      Serial.print(value);
      Serial.println("Hz");
    } else {
      data_outputs[channel].enabled = false;
      Serial.println("Position+Time output disabled");
    }
  }
}

void processCommand3(int value, byte channel) {
  // 命令 3：GNSS 更新頻率控制
  // value = 頻率設定 (1-10Hz)，預設為 4Hz
  // channel 參數在此命令中不使用

  if (value == 0) {
    // 預設設定為 4Hz
    value = 4;
  }

  if (value < 1 || value > 10) {
    Serial.print("❌ 頻率設定錯誤: ");
    Serial.print(value);
    Serial.println("Hz (支援範圍: 1-10Hz)");
    return;
  }

  Serial.print("🛰️ CMD 3 - 設定 GNSS 頻率為 ");
  Serial.print(value);
  Serial.println("Hz");

  // 使用 GNSS 控制模組設定頻率
  bool success = GnssControl::setUpdateFrequency(value);

  if (success) {
    Serial.print("✅ GNSS 頻率設定完成: ");
    Serial.print(value);
    Serial.println("Hz");
  } else {
    Serial.println("❌ GNSS 頻率設定失敗");
  }
}


// ---- Main Command Processor ----
void processCommands() {
  // 使用 myUART 中定義的變數
  extern byte uart_cmd, fog_ch;
  extern int uart_value;
  extern volatile bool cmd_complete;

  if (cmd_complete) {
    switch(uart_cmd) {
      case 1:
        processCommand1(uart_value, fog_ch);
        break;
      case 2:
        processCommand2(uart_value, fog_ch);
        break;
      case 3:
        processCommand3(uart_value, fog_ch);
        break;
      default:
        Serial.println("Unknown command");
        break;
    }

    // 清除命令完成標誌
    cmd_complete = false;
  }
}

// ---- API ----
void command_init(void) {
  // 初始化數據輸出陣列
  for (int i = 0; i < MAX_OUTPUTS; i++) {
    data_outputs[i].enabled = false;
    data_outputs[i].interval_ms = 0;
    data_outputs[i].last_send_time = 0;
    data_outputs[i].packet_type = 0;
    data_outputs[i].channel = 0;
  }
}