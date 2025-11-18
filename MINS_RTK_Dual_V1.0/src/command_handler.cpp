#include "command_handler.h"
#include "packet_protocol.h"
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
  // 命令 2：設定位置+高度+航向輸出頻率
  // 封包格式: FA FF 19 00 03 [8-byte lat] [8-byte lon] [4-byte alt] [4-byte heading] [checksum]
  // 範例: FA FF 19 00 03 [lat] [lon] [alt] [heading] [checksum] (29 bytes total)

  // 使用 channel 作為輸出槽索引，覆蓋該通道的設定
  if (channel < MAX_OUTPUTS) {
    if (value > 0) {
      data_outputs[channel].enabled = true;
      data_outputs[channel].interval_ms = 1000 / value;
      data_outputs[channel].last_send_time = millis();
      data_outputs[channel].packet_type = PKT_POS_HEADING;
      data_outputs[channel].channel = channel;

      Serial.print("Position+Heading output enabled: ");
      Serial.print(value);
      Serial.println("Hz");
    } else {
      data_outputs[channel].enabled = false;
      Serial.println("Position+Heading output disabled");
    }
  }
}

void processCommand3(int value, byte channel) {
  // 命令 3 處理

}

void processCommand4(int value, byte channel) {
  // 命令 4 處理

}

void processCommand5(int value, byte channel) {
  // 命令 5 處理

}

void processCommand6(int value, byte channel) {
  // 命令 6 處理

}

void processCommand7(int value, byte channel) {
  // 命令 7 處理

}

void processCommand8(int value, byte channel) {
  // 命令 8 處理

}

void processCommand9(int value, byte channel) {
  // 命令 9 處理

}

void processCommand10(int value, byte channel) {
  // 命令 10 處理

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
      case 4:
        processCommand4(uart_value, fog_ch);
        break;
      case 5:
        processCommand5(uart_value, fog_ch);
        break;
      case 6:
        processCommand6(uart_value, fog_ch);
        break;
      case 7:
        processCommand7(uart_value, fog_ch);
        break;
      case 8:
        processCommand8(uart_value, fog_ch);
        break;
      case 9:
        processCommand9(uart_value, fog_ch);
        break;
      case 10:
        processCommand10(uart_value, fog_ch);
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