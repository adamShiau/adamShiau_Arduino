#include <Arduino.h>
#include "wiring_private.h"
#include <Arduino_LSM6DS3.h>
#include <SPI.h>
#include <Wire.h>
#include <SoftWire.h>
#include <AsyncDelay.h>

/*** ADXL355***/
#define SENS_8G 0.0000156
#define SENS_4G 0.0000078
#define SENS_2G 0.0000039
#define ADXL355_ADDR 0x1D
// Memory register addresses:
const int TEMP2  = 0x06;
const int TEMP1  = 0x07;
const int XDATA3 = 0x08;
const int XDATA2 = 0x09;
const int XDATA1 = 0x0A;
const int YDATA3 = 0x0B;
const int YDATA2 = 0x0C;
const int YDATA1 = 0x0D;
const int ZDATA3 = 0x0E;
const int ZDATA2 = 0x0F;
const int ZDATA1 = 0x10;
const int RANGE = 0x2C;
const int POWER_CTL = 0x2D;
const int SYNC = 0x2B;
const int RST = 0x2F;
const int INTERRUPT = 0x2A;
//analog device ID register
const int DEVID_AD = 0x00;

//status register
const int STATUS = 0x04;

//FIFO register
const int FIFO = 0x05;

//filter setting register
const int FILTER = 0x28;

// Device values
const int RANGE_2G = 0x01;
const int RANGE_4G = 0x02;
const int RANGE_8G = 0x03;
const int MEASURE_MODE = 0x06; // Only accelerometer
const int MEASUREwTEMP_MODE = 0x04;
// Operations
const int READ_BYTE = 0x01;
const int WRITE_BYTE = 0x00;

// Pins used for the connection with the sensor
const int CHIP_SELECT_PIN = 10;
/*********************************************/

#define PRINT_GYRO 0
#define PRINT_XLM 0
#define PRINT_ADXL355 0
#define PRINT_UPDATE_TIME 0
#define PRINT_TIME 0
#define PRINT_SFOS200 0
#define PRINT_PP 0
#define PRINT_SPEED 0
#define PRINT_VBOX 1


#define SERIAL2_RX 3
#define SERIALHCI_RX 6
#define FOG_CLK 2
#define PERIOD 10000
#define sdaPin  11
#define sclPin  12

bool clk_status = 1;
unsigned long start_time = 0, old_time = 0;
unsigned int t_old=0, t_new;


// Attach the interrupt handler to the SERCOM



void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(SERIAL2_RX,INPUT);
}
void loop() {
  
		clk_status = digitalRead(FOG_CLK);
		// if(clk_status) 
		// {
			VBOX();
		// }
	   
}





void VBOX() {

  byte temp[3];
  long speed;
  int acc;
  byte header[7];
	while (Serial1.available()<73) {}; 
	// if(Serial2.available()>230) {
		// for(int i=0; i<220; i++) Serial2.read(); 
	// }
	for(int i=0; i<7; i++) header[i] = Serial1.read();
	while( ((header[0]!=0x24)||(header[1]!=0x56)||(header[2]!=0x42)||(header[3]!=0x33)||(header[4]!=0x69)||(header[5]!=0x73)||(header[6]!=0x24))) 
	{
//		 for(int j=0; j<7; j++) {
//			 Serial.println(header[j], HEX);
//		 }
//		 Serial.println("done");
		for(int j=0; j<6; j++) header[j] = header[j+1];
		header[6] = Serial1.read();
		
		delayMicroseconds(1);
	}
	 // Serial.println("pass");
	 
	/***  below read VBOX speed ***/
	// for(int i=0; i<14; i++) Serial1.read();
    // for(int i=0; i<3; i++) {
      // temp[i] = Serial1.read(); 
    // }
    // speed = temp[0]<<16 | temp[1]<<8 | temp[2];
    // Serial.print(temp[0]);
    // Serial.print(", ");
    // Serial.print(temp[1]);
    // Serial.print(", ");
    // Serial.print(temp[2]);
    // Serial.print(", ");
	// Serial.println(speed*0.001);
	
	/***  below read VBOX z-axis acc ***/
	 for(int i=0; i<42; i++) Serial1.read();
    for(int i=0; i<2; i++) {
      temp[i] = Serial1.read(); 
    }
    acc = temp[0]<<8 | temp[1];
	 // Serial.println(acc);
	 
	 /*** ***/
	// if(PRINT_VBOX) {
		// Serial.print(millis());
		// Serial.print("\t");
		// Serial.print(Serial.available()); 
		// Serial.print("\t");
		// Serial.print(temp[0]);
		// Serial.print("\t");
		// Serial.print(temp[1]);
		// Serial.print("\t");
		// Serial.print(temp[2]);
		// Serial.print("\t");
		// Serial.println(speed);
    // }  
	Serial1.write(0xAB);
//  Serial1.write(byte(0));
    Serial1.write(temp[0]);
    Serial1.write(temp[1]);
    Serial1.write(temp[2]);
}


void checkByte(byte check) {
  Serial.write(check);
}
