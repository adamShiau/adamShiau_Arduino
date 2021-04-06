#include <Arduino.h>
#include "wiring_private.h"
#include <Arduino_LSM6DS3.h>

#define PRINT_GYRO 1
#define PRINT_XLM 1
#define PRINT_TIME 0
#define PERIOD 10000

bool clk_status = 1;
unsigned long start_time = 0;

void setup() {
  Serial.begin(115200);
//  while (!Serial);
  if (!IMU.begin()) {
    while (1);
  }
}

void loop() {
  int ax, ay, az, wx, wy, wz;
  if(clk_status) {
    start_time = micros();
    clk_status = 0;
    // checkByte(0xAA);
    request_xlm(ax, ay, az);
    request_gyro(wx, wy, wz);
    // send_current_time(start_time);
    // checkByte(0xAB);
//    Serial.println(millis());
  }
  output_fogClk(start_time);
}

void send_current_time(unsigned long current_time) {
  if(PRINT_TIME) {
    Serial.print("t");
    Serial.print('\t');
    Serial.println(current_time);
  }
  Serial.write(current_time>>24);
  Serial.write(current_time>>16);
  Serial.write(current_time>>8);
  Serial.write(current_time);
}

void request_xlm(int x, int y, int z) {
  while (!IMU.accelerationAvailable()); 
    IMU.readAcceleration(x, y, z);
    if(PRINT_XLM) {
      Serial.print("a");
      Serial.print('\t');
      Serial.print(x*4.0/32768.0);
      Serial.print('\t');
      Serial.print(y*4.0/32768.0);
      Serial.print('\t');
      Serial.println(z*4.0/32768.0);
    }
    // Serial.write(x>>8);
    // Serial.write(x);
    // Serial.write(y>>8);
    // Serial.write(y);
    // Serial.write(z>>8);
    // Serial.write(z);
}

void request_gyro(int x, int y, int z) {
  while (!IMU.gyroscopeAvailable()); 
    IMU.readGyroscope(x, y, z);
    if(PRINT_GYRO) {
      Serial.print("w");
      Serial.print('\t');
      Serial.print(x);
      Serial.print('\t');
      Serial.print(y);
      Serial.print('\t');
      Serial.println(z);
    }
    // Serial.write(x>>8);
    // Serial.write(x);
    // Serial.write(y>>8);
    // Serial.write(y);
    // Serial.write(z>>8);
    // Serial.write(z);
}

void output_fogClk(unsigned long tin) {
  if((micros()-tin)>=PERIOD) {
    start_time = micros();
    clk_status = 1;
  }
}

void checkByte(byte check) {
  Serial.write(check);
}
