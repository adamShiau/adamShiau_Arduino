

// Includes
#include <Arduino.h>
#include <ASM330LHHSensor.h>
#include "wiring_private.h"

#define SerialPort Serial

#define INT_1 A5

float ODR, SEN;
long FS;
uint8_t ID;

// Components
ASM330LHHSensor IMU(&Wire, ASM330LHH_I2C_ADD_L);

void setup() {

  // Led.
  // pinMode(LED_BUILTIN, OUTPUT);

  // Force INT1 of ASM330LHH low in order to enable I2C
  pinMode(INT_1, OUTPUT);

  digitalWrite(INT_1, LOW);

  delay(200);

  // Initialize serial for output.
  SerialPort.begin(115200);
  
  // Initialize I2C bus.
  Wire.begin();

  IMU.begin();
  IMU.Enable_X();
  IMU.Enable_G();
  IMU.Set_X_ODR(416.0);
  IMU.Set_X_FS(4);  
  IMU.Set_G_ODR(416.0);
  IMU.Set_G_FS(500); 
}

void loop() {
  // IMU.ReadID(&ID);
  // IMU.Get_X_ODR(&ODR);
  // IMU.Get_X_FS(&FS);
  // IMU.Get_X_Sensitivity(&SEN);
  // IMU.Get_G_ODR(&ODR);
  // IMU.Get_G_FS(&FS);
  // IMU.Get_G_Sensitivity(&SEN);
  // Serial.print("ID: ");
  // Serial.println(ID, HEX);
  // Serial.print(ODR);
  // Serial.print(" ");
  // Serial.print(FS);
  // Serial.print(" ");
  // Serial.println(SEN, 6);
  // delay(100);

  // Read accelerometer and gyroscope.
  int32_t accelerometer[3] = {};
  int32_t gyroscope[3] = {};
  IMU.Get_X_Axes(accelerometer);
  // IMU.Get_G_Axes(gyroscope);

  // Output data.
  // SerialPort.print("ASM330LHH: | Acc[mg]: ");
  // SerialPort.print(accelerometer[0]);
  // SerialPort.print(" ");
  // SerialPort.print(accelerometer[1]);
  // SerialPort.print(" ");
  // SerialPort.println(accelerometer[2]);
  // SerialPort.print(" | Gyr[mdps]: ");
  // SerialPort.print(gyroscope[0]);
  // SerialPort.print(" ");
  // SerialPort.print(gyroscope[1]);
  // SerialPort.print(" ");
  // SerialPort.print(gyroscope[2]);
  // SerialPort.println(" |");
  // delay(10);
}
