#include <Arduino.h>
#include "wiring_private.h"
#include <Arduino_LSM6DS3.h>

#define PRINT_GYRO 0
#define PRINT_XLM 0
#define PERIOD 10000

bool clk_status = 0;
unsigned long start_time = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); //for HC-05
  while (!Serial);
  if (!IMU.begin()) {
    while (1);
  }
}

void loop() {
  int ax, ay, az, wx, wy, wz;
  if(clk_status) {
    clk_status = 0;
    checkByte(0xAA);
    request_xlm(ax, ay, az);
    request_gyro(wx, wy, wz);
//    Serial.println(millis());
  }
  output_fogClk(start_time);
}


void request_xlm(int x, int y, int z) {
  while (!IMU.accelerationAvailable()); 
    IMU.readAcceleration(x, y, z);
    if(PRINT_XLM) {
      Serial.print("a");
      Serial.print('\t');
      Serial.print(x);
      Serial.print('\t');
      Serial.print(y);
      Serial.print('\t');
      Serial.println(z);
    }
    Serial1.write(x>>8);
    Serial1.write(x);
    Serial1.write(y>>8);
    Serial1.write(y);
    Serial1.write(z>>8);
    Serial1.write(z);
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
    Serial1.write(x>>8);
    Serial1.write(x);
    Serial1.write(y>>8);
    Serial1.write(y);
    Serial1.write(z>>8);
    Serial1.write(z);
}

void output_fogClk(unsigned long tin) {
  if((micros()-tin)>=PERIOD) {
    start_time = micros();
    clk_status = 1;
  }
}

void checkByte(byte check) {
  Serial1.write(check);
}
