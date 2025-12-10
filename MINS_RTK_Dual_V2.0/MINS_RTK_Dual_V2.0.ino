#include "wiring_private.h"
#include "src/myUART.h"
#include "src/nmea_parser.h"
#include "src/command_handler.h"
#include "src/packet_protocol.h"
#include "src/gnss_control.h"

#define MCU_LED 16

NmeaParser gps_parser;

void setup() {
  myUART_init();
  command_init();
  pinMode(MCU_LED, OUTPUT);
  Blink_MCU_LED();
  Serial.println("MINS");

  // 開機自動設定 GNSS 為 4Hz
  delay(3000);  // 等待 GNSS 晶片完全穩定
  GnssControl::setUpdateFrequency(4);
}

void loop() {
  // 處理 LOC 晶片的 NMEA 數據
  if(Serial4.available()) {
    String nmea_line = Serial4.readStringUntil('\n');
    if(gps_parser.parseLine(nmea_line.c_str())) {
      printGpsData(nmea_line);
    }
  }

  // 處理命令
  processCommands();

  // 處理所有數據輸出
  processDataOutputs();

  delay(100);
}


void printGpsData(const String& raw_line) {
  GpsData* gps = gps_parser.getData();

  // 顯示原始接收資料
  Serial.print("Raw: ");
  Serial.println(raw_line);

  /*
  Serial.println("===== GPS Data =====");

  // 定位狀態和位置
  Serial.print("Fix: ");
  Serial.print(gps->fix_type);
  Serial.print(" | Valid Pos: ");
  Serial.print(gps->valid_position ? "YES" : "NO");
  Serial.print(" | Data Age: ");
  Serial.print(millis() - gps->last_update);
  Serial.println("ms");

  Serial.print("Lat: ");
  Serial.print(gps->latitude, 8);
  Serial.print(" | Lon: ");
  Serial.print(gps->longitude, 8);
  Serial.print(" | Alt: ");
  Serial.print(gps->altitude);
  Serial.println("m");

  // 衛星和精度
  Serial.print("Sats Used: ");
  Serial.print(gps->satellites_used);
  Serial.print(" | Visible: ");
  Serial.print(gps->satellites_visible);
  Serial.print(" | HDOP: ");
  Serial.println(gps->hdop);

  // 運動資料
  Serial.print("Speed: ");
  Serial.print(gps->speed);
  Serial.print("m/s | Heading: ");
  Serial.print(gps->heading);
  Serial.println("°");

  // 時間資料
  if(gps->valid_time) {
    Serial.print("Time: ");
    if(gps->hour < 10) Serial.print("0");
    Serial.print(gps->hour);
    Serial.print(":");
    if(gps->minute < 10) Serial.print("0");
    Serial.print(gps->minute);
    Serial.print(":");
    if(gps->second < 10) Serial.print("0");
    Serial.print(gps->second);
    Serial.print(" | Date: ");
    if(gps->day < 10) Serial.print("0");
    Serial.print(gps->day);
    Serial.print("/");
    if(gps->month < 10) Serial.print("0");
    Serial.print(gps->month);
    Serial.print("/");
    Serial.println(gps->year);
  } else {
    Serial.println("Time: Invalid");
  }

  // 各星系 CNO 原始數據
  const char* system_names[] = {"GPS", "GLO", "GAL"};
  for(int sys = 0; sys < SAT_SYSTEM_COUNT; sys++) {
    SatelliteSystem* sat_sys = &gps->sat_systems[sys];
    if(sat_sys->valid_sats > 0) {
      Serial.print(system_names[sys]);
      Serial.print(" ");
      Serial.print(sat_sys->valid_sats);
      Serial.print("sats CNO:");
      for(int i = 0; i < sat_sys->valid_sats && i < 12; i++) {
        if(sat_sys->cno_values[i] > 0) {
          Serial.print(sat_sys->prn_numbers[i]);
          Serial.print("(");
          Serial.print(sat_sys->cno_values[i]);
          Serial.print("dB) ");
        }
      }
      Serial.print("| ");
    }
  }
  Serial.println();

  Serial.println("==================");
  */
}

void Blink_MCU_LED()
{
  bool A=0;
  for(int i=0; i<10; i++){
    digitalWrite(MCU_LED, A);
    delay(100);
    A = !A;
  }
   delay(100);
}

