#include <Arduino.h>
#include "wiring_private.h"
#include <Arduino_LSM6DS3.h>

#define PERIOD 10000

bool clk_status = 0;
unsigned long start_time = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  if (!IMU.begin()) {
    while (1);
  }
}

void loop() {
  int ax, ay, az, wx, wy, wz;
  if(clk_status) {
    clk_status = 0;
    request_xlm(ax, ay, az);
    request_gyro(wx, wy, wz);
    Serial.println(millis());
  }
  output_fogClk(start_time);
}


void request_xlm(int x, int y, int z) {
  while (!IMU.accelerationAvailable()); 
    IMU.readAcceleration(x, y, z);
    Serial.print("a");
    Serial.print('\t');
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
}

void request_gyro(int x, int y, int z) {
  while (!IMU.gyroscopeAvailable()); 
    IMU.readGyroscope(x, y, z);
    Serial.print("w");
    Serial.print('\t');
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
}

void output_fogClk(unsigned long tin) {
  if((micros()-tin)>=PERIOD) {
    start_time = micros();
    clk_status = 1;
  }
}
