
#include <Arduino.h>
#include "wiring_private.h"
#include <Arduino_LSM6DS3.h>
#define PRINT_GYRO 0
#define PRINT_XLM 1
#define PRINT_UNO 1

Uart mySerial (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);

int cnt_p = 0, cnt_m = 0;
byte cnt0_p, cnt1_p, cnt2_p, cnt3_p;
byte cnt0_m, cnt1_m, cnt2_m, cnt3_m;
int byte_read;

// Attach the interrupt handler to the SERCOM
void SERCOM0_Handler()
{
    mySerial.IrqHandler();
}

void setup() {
  // put your setup code here, to run once:
    // Reassign pins 5 and 6 to SERCOM alt
  pinPeripheral(5, PIO_SERCOM_ALT); //RX
  pinPeripheral(6, PIO_SERCOM_ALT); //TX
  mySerial.begin(115200);
  Serial1.begin(115200);
  Serial.begin(115200);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");
}

void loop() {
  // put your main code here, to run repeatedly:


/**********
  cnt0_p = cnt_p;
  cnt1_p = cnt_p >> 8;
  cnt2_p = cnt_p >> 16;
  cnt3_p = cnt_p >> 24;
  cnt0_m = cnt_m;
  cnt1_m = cnt_m >> 8;
  cnt2_m = cnt_m >> 16;
  cnt3_m = cnt_m >> 24;
  Serial.print(cnt_p);
  Serial.print(", ");
  Serial.println(cnt_m);
  
  Serial1.write(cnt3_p);
  Serial1.write(cnt2_p);
  Serial1.write(cnt1_p);
  Serial1.write(cnt0_p);
  Serial1.write(cnt3_m);
  Serial1.write(cnt2_m);
  Serial1.write(cnt1_m);
  Serial1.write(cnt0_m);
  cnt_p++;
  cnt_m--;
  delay(10);
************/

/********gyro get data SFOC-200*************
//  if(mySerial.available()>=12)
//  {
//    for(int i=0; i<12; i++) {
//      temp[i] = mySerial.read(); 
//    }
//    header = temp[0]<<8 | temp[1];
//    omega = temp[5]<<24 | temp[4]<<16 | temp[3]<<8 | temp[2];
//
//    Serial.print(header, HEX);
//    Serial.print(", ");
//    Serial.print(mySerial.available()); 
//    Serial.print(", ");
//    Serial.print(cnt_p);
//    Serial.print(", ");
//    Serial.println(omega);
//    
//    Serial1.write(temp[5]);
//    Serial1.write(temp[4]);
//    Serial1.write(temp[3]);
//    Serial1.write(temp[2]);
//    cnt_p++;
//  }
****/

  if(mySerial.available()>=12) {
//    requestGyro();
    requestUno();
    requestXLM();
  }

//    if (IMU.accelerationAvailable()) {
//    requestXLM();
//  }


}

void requestGyro() {

  byte temp[12];
  int omega;
  int header;
  
    for(int i=0; i<12; i++) {
      temp[i] = mySerial.read(); 
    }
    header = temp[0]<<8 | temp[1];
    omega = temp[5]<<24 | temp[4]<<16 | temp[3]<<8 | temp[2];

    if(PRINT_GYRO) {
      Serial.print(header, HEX);
      Serial.print("\t");
      Serial.print(mySerial.available()); 
      Serial.print("\t");
//      Serial.print(cnt_p);
//      Serial.print("\t");
      Serial.print(omega);
      Serial.print("\t");
      cnt_p++;
    }  
    Serial1.write(temp[5]);
    Serial1.write(temp[4]);
    Serial1.write(temp[3]);
    Serial1.write(temp[2]);
}

void requestUno() {

  byte temp[4];
  int data;
    for(int i=0; i<4; i++) {
      temp[i] = mySerial.read(); 
    }
    data = temp[0]<<24 | temp[1]<<16 | temp[2]<<8 | temp[3];
    if(PRINT_UNO){
      Serial.print(mySerial.available()); 
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
  Serial1.write(x>>8);
  Serial1.write(x);
  Serial1.write(y>>8);
  Serial1.write(y);
  Serial1.write(z>>8);
  Serial1.write(z);
}
