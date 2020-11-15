
#include <Arduino.h>
#include "wiring_private.h"
#include <Arduino_LSM6DS3.h>
#define PRINT_GYRO 0
#define PRINT_XLM 0
#define PRINT_UNO 0
#define FOG_CLK 2
#define HALF_PERIOD 5000

Uart mySerial5 (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
Uart mySerial13 (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);

int cnt_p = 0, cnt_m = 0;
unsigned long start_time;
//byte cnt0_p, cnt1_p, cnt2_p, cnt3_p;
//byte cnt0_m, cnt1_m, cnt2_m, cnt3_m;
//int byte_read;
//byte test[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

// Attach the interrupt handler to the SERCOM
void SERCOM0_Handler()
{
    mySerial5.IrqHandler();
}
void SERCOM1_Handler()
{
    mySerial13.IrqHandler();
}
bool clk_status = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(FOG_CLK,OUTPUT);
//    // Reassign pins 5 and 6 to SERCOM alt
  pinPeripheral(5, PIO_SERCOM_ALT); //RX
  pinPeripheral(6, PIO_SERCOM_ALT); //TX
    // Reassign pins 13 and 8 to SERCOM (not alt this time)
  pinPeripheral(13, PIO_SERCOM);
  pinPeripheral(8, PIO_SERCOM);
  
  mySerial5.begin(115200);
  mySerial13.begin(115200);
  Serial1.begin(115200);
  Serial.begin(115200);

  if (!IMU.begin()) {
//    Serial.println("Failed to initialize IMU!");
    while (1);
  }
//  Serial.print("Accelerometer sample rate = ");
//  Serial.print(IMU.accelerationSampleRate());
//  Serial.println(" Hz");
//  Serial.println();
//  Serial.println("Acceleration in G's");
//  Serial.println("X\tY\tZ");


  start_time = micros();
}

void loop() {
  // put your main code here, to run repeatedly:


  if(mySerial13.available()>=24) {
    checkByte(0xAA);
    requestGyro();
    requestXLM();
  } 
  output_fogClk(start_time);
}

void requestGyro() {

  byte temp[10];
  int omega;
  byte header[2];

    header[0] = mySerial13.read();
    header[1] = mySerial13.read();
    while( (header[0]!=0xC0)||(header[1]!=0xC0)) {
      header[0] = mySerial13.read();
      header[1] = mySerial13.read();
    }
    for(int i=0; i<10; i++) {
      temp[i] = mySerial13.read(); 
    }
    omega = temp[3]<<24 | temp[2]<<16 | temp[1]<<8 | temp[0];

    if(PRINT_GYRO) {
      Serial.print(mySerial13.available()); 
      Serial.print("\t");
      Serial.print(header[0]<<8|header[1], HEX);
      Serial.print("\t");    
      Serial.print(temp[3]);
      Serial.print("\t");
      Serial.print(temp[2]);
      Serial.print("\t");
      Serial.print(temp[1]);
      Serial.print("\t");
      Serial.print(temp[0]);
      Serial.print("\t");
      Serial.print(omega);
      Serial.print("\t");
    }  
    Serial.write(temp[3]);
    Serial.write(temp[2]);
    Serial.write(temp[1]);
    Serial.write(temp[0]);
}

void requestUno() {

  byte temp[4];
  int data;
    for(int i=0; i<4; i++) {
      temp[i] = mySerial13.read(); 
    }
    data = temp[0]<<24 | temp[1]<<16 | temp[2]<<8 | temp[3];
    if(PRINT_UNO){
      Serial.print(mySerial13.available()); 
      Serial.print("\t");
      Serial.print(data);
      Serial.print("\t");
//      Serial.print(temp[0]);
//      Serial.print("\t");
//      Serial.print(temp[1]);
//      Serial.print("\t");
//      Serial.print(temp[2]);
//      Serial.print("\t");
//      Serial.print(temp[3]);
//      Serial.print("\t");

    }  
    Serial1.write(temp[0]);
    Serial1.write(temp[1]);
    Serial1.write(temp[2]);
    Serial1.write(temp[3]);
    
}

void requestXLM() {
  int x, y, z;
  IMU.readAcceleration(x, y, z);
  if(PRINT_XLM) {
    Serial.print(cnt_p);
    Serial.print('\t');
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
    cnt_p++;
  }
  Serial.write(x>>8);
  Serial.write(x);
  Serial.write(y>>8);
  Serial.write(y);
  Serial.write(z>>8);
  Serial.write(z);
}

void checkByte(byte check) {
  Serial.write(check);
}

void output_fogClk(unsigned long tin) {
  if((micros()-tin)>=HALF_PERIOD) {
    start_time = micros();
    digitalWrite(FOG_CLK, clk_status);
    clk_status = !clk_status;
  }
}
