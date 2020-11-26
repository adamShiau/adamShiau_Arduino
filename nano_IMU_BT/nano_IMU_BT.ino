#include <Arduino.h>
#include "wiring_private.h"
#include <Arduino_LSM6DS3.h>

#define PRINT_GYRO 0
#define PRINT_XLM 0
#define PRINT_TIME 0
#define PERIOD 10000

bool clk_status = 1;
unsigned long start_time = 0;

//Uart mySerial5 (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
//Uart mySerial13 (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);

// Attach the interrupt handler to the SERCOM
//void SERCOM0_Handler()
//{
//    mySerial5.IrqHandler();
//}
//void SERCOM1_Handler()
//{
//    mySerial13.IrqHandler();
//}



void setup() {
  // Reassign pins 5 and 6 to SERCOM alt
//  pinPeripheral(5, PIO_SERCOM_ALT); //RX
//  pinPeripheral(6, PIO_SERCOM_ALT); //TX
  // Reassign pins 13 and 8 to SERCOM (not alt this time)
//  pinPeripheral(13, PIO_SERCOM);
//  pinPeripheral(8, PIO_SERCOM);

//  mySerial5.begin(115200); //rx:p5, tx:p6
//  mySerial13.begin(115200);//rx:p13, tx:p8
  Serial.begin(115200);
  Serial1.begin(115200); //for HC-05
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
    checkByte(0xAA);
    request_xlm(ax, ay, az);
    request_gyro(wx, wy, wz);
    send_current_time(start_time);
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
  Serial1.write(current_time>>24);
  Serial1.write(current_time>>16);
  Serial1.write(current_time>>8);
  Serial1.write(current_time);
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
  if(abs((micros()-tin))>=PERIOD) {
//    start_time = micros();
    clk_status = 1;
  }
}

void checkByte(byte check) {
  Serial1.write(check);
}
