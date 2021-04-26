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
#define PRINT_UPDATE_TIME 1
#define PRINT_TIME 0
#define PRINT_SFOS200 0
#define PRINT_PP 0
#define PRINT_SPEED 0
#define PRINT_VBOX 0


#define SERIAL2_RX 3
#define SERIALHCI_RX 6
#define FOG_CLK 2
#define PERIOD 10000
#define sdaPin  11
#define sclPin  12

bool clk_status = 1;
unsigned long start_time = 0, old_time = 0;
unsigned int t_old=0, t_new;

Uart mySerial5 (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
Uart mySerial13 (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);

SoftWire sw(sdaPin, sclPin);
char swTxBuffer[24];
char swRxBuffer[24];

AsyncDelay readInterval;

// Attach the interrupt handler to the SERCOM
void SERCOM0_Handler()
{
    mySerial5.IrqHandler();
}
void SERCOM1_Handler()
{
    mySerial13.IrqHandler();
}



void setup() {
  // SPI.begin();
  // SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  // Wire.begin();
  // Wire.setClock(100000);
  sw.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
  sw.setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
  sw.setDelay_us(5);
  sw.setTimeout(1000);
  sw.setClock(1000000);
  sw.begin();
  // Reassign pins 5 and 6 to SERCOM alt
  pinPeripheral(5, PIO_SERCOM_ALT); //RX
  pinPeripheral(6, PIO_SERCOM_ALT); //TX

  // Reassign pins 13 and 8 to SERCOM (not alt this time)
  pinPeripheral(13, PIO_SERCOM);
  pinPeripheral(8, PIO_SERCOM);

  mySerial5.begin(115200); //rx:p5, tx:p6, SRS200
  mySerial13.begin(115200);//rx:p13, tx:p8, PP
  Serial.begin(115200);
  Serial1.begin(115200); //for HC-05
  Serial2.begin(115200); //
  SerialHCI.begin(115200);
  
  pinMode(FOG_CLK,INPUT);
  pinMode(SERIAL2_RX,INPUT);
  // pinMode(SERIALHCI_RX,INPUT);
  
  //Configure ADXL355:
  Soft_I2CWriteData(RST, 0x52);
  delay(100);
  Soft_I2CWriteData(RANGE, RANGE_8G);
  delay(100);
  Soft_I2CWriteData(FILTER, 0b100); //ODR 0b100@250Hz 0b101@125Hz
  delay(100);
  Soft_I2CWriteData(SYNC, 0b010); 
  delay(100);
  Soft_I2CWriteData(POWER_CTL, MEASUREwTEMP_MODE); // Enable measure mode
  // Give the sensor time to set up:
  delay(100);
  if (!IMU.begin()) {
   while (1);
 }
	
}
void loop() {
  int ax, ay, az, wx, wy, wz;
  
		clk_status = digitalRead(FOG_CLK);
		// if(clk_status) 
		// {
			start_time = millis();
			if(PRINT_UPDATE_TIME) Serial.println(start_time - old_time);
			old_time = start_time;
			clk_status = 0;
			checkByte(0xAA);//1
			send_current_time(start_time);//4
			requestSFOS200();//4
			requestPP();//4
			requestSpeed();//3
			request_adxl355(ax, ay, az);//11
			request_nano33_gyro(); //6
			checkByte(0xAB);//1
		// }
	   
}

void request_adxl355(int accX, int accY, int accZ) {
  byte temp_ax1, temp_ax2, temp_ax3;
  byte temp_ay1, temp_ay2, temp_ay3;
  byte temp_az1, temp_az2, temp_az3;
  byte temp1, temp2;
  byte status;
  int RT;
  float RTf;
  bool isReady;

	status = Soft_I2CReadData(STATUS); //0x04
	isReady = status&0b00000001;
	// while(!isReady) {
		// status = I2CReadData(STATUS);
		// isReady = status&0b00000001;
		// delay(1);
	// }
  if(isReady){
      temp_ax1 = Soft_I2CReadData(XDATA3); //0x08
      temp_ax2 = Soft_I2CReadData(XDATA2); //0x09
      temp_ax3 = Soft_I2CReadData(XDATA1); //0x0A
      accX = temp_ax1<<12 | temp_ax2<<4 | temp_ax3>>4;
      if((accX>>19) == 1) accX = accX - 1048576;
    
      temp_ay1 = Soft_I2CReadData(YDATA3); //0x0B
      temp_ay2 = Soft_I2CReadData(YDATA2); //0x0C
      temp_ay3 = Soft_I2CReadData(YDATA1); //0x0D
      accY = temp_ay1<<12 | temp_ay2<<4 | temp_ay3>>4;
      if((accY>>19) == 1) accY = accY - 1048576;
    
      temp_az1 = Soft_I2CReadData(ZDATA3); //0x0E
      temp_az2 = Soft_I2CReadData(ZDATA2); //0x0F
      temp_az3 = Soft_I2CReadData(ZDATA1); //0x10
      accZ = temp_az1<<12 | temp_az2<<4 | temp_az3>>4;
      if((accZ>>19) == 1) accZ = accZ - 1048576;
		
	  temp1 = Soft_I2CReadData(TEMP1);
	  temp2 = Soft_I2CReadData(TEMP2);
	  RT = (temp2&0x07)<<8 | temp1;
	  // RTf = 25 - ((float)RT-1885.0)/9.05;
		
      if(PRINT_ADXL355)
      {
        t_new = micros();
        Serial.print(t_new - t_old);
        Serial.print(", ");
		Serial.print(RT);
        Serial.print(", ");
        Serial.print(accX);
        Serial.print(", ");
        Serial.print((float)accX*SENS_8G);
        Serial.print(", ");
        Serial.print(accY);
        Serial.print(", ");
        Serial.print((float)accY*SENS_8G);
        Serial.print(", ");
        Serial.print(accZ);
        Serial.print(", ");
        Serial.println((float)accZ*SENS_8G);
		/*** below for degug***/
		// Serial.print("ax:");
		// Serial.print(", ");
		// Serial.print(temp_ax1);
		// Serial.print(", ");
		// Serial.print(temp_ax2);
		// Serial.print(", ");
		// Serial.println(temp_ax3);
		// Serial.print("ay:");
		// Serial.print(temp_ay1);
		// Serial.print(", ");
		// Serial.print(temp_ay2);
		// Serial.print(", ");
		// Serial.println(temp_ay3);
        t_old = t_new;
      } 
		// Serial1.write(0xC2);
		Serial1.write(temp_ax1);
		Serial1.write(temp_ax2);
		Serial1.write(temp_ax3);
		// Serial1.write(0xC3);
		Serial1.write(temp_ay1);
		Serial1.write(temp_ay2);
		Serial1.write(temp_ay3);
		// Serial1.write(0xC4);
		Serial1.write(temp_az1);
		Serial1.write(temp_az2);
		Serial1.write(temp_az3);
		
		Serial1.write(temp2);
		Serial1.write(temp1);
    }  
	// else Serial.println("is ready = 0");
}

void send_current_time(unsigned long current_time) {
  if(PRINT_TIME) {
    Serial.print("t");
    Serial.print('\t');
    Serial.print(byte(current_time>>24));
    Serial.print('\t');
    Serial.print(byte(current_time>>16));
    Serial.print('\t');
    Serial.print(byte(current_time>>8));
    Serial.print('\t');
    Serial.print(byte(current_time>>0));
    Serial.print('\t');
    Serial.println(current_time);
  }
  Serial1.write(current_time>>24);
  Serial1.write(current_time>>16);
  Serial1.write(current_time>>8);
  Serial1.write(current_time);
}

void requestSFOS200() {

  byte temp[10];
  int omega;
  byte header[2];
//  unsigned int t_old=0, t_new;

/***sync clock 與MCU smapling time有一點差別時會造成buffer累積，當sync clock比較快時data送進buffer比清空的速度快，
buffer累積到255時會爆掉歸零，此時data傳輸會怪怪的，因此在buffer快接近爆掉時須先清掉一些。
而當sync clock比較慢時data送進buffer比清空的速度慢，buffer會見底，因此當buffer快沒時須等待buffer補充。
***/
	while (mySerial5.available()<24) {
//	    Serial.println(mySerial5.available());
	  }; 
	if(mySerial5.available()>230) {
		for(int i=0; i<220; i++) mySerial5.read(); 
	}
		header[0] = mySerial5.read();
		header[1] = mySerial5.read();
		 while( ((header[0]!=0xC0)||(header[1]!=0xC0))) 
		 {
		 // while( ((header[0]!=0xC0)||(header[1]!=0xC0)) && (mySerial5.available()>=10)) {
		  // if(mySerial5.available()) {
			// Serial.println(mySerial5.available()); 
			header[0] = mySerial5.read();
			header[1] = mySerial5.read();
			delay(1);
			// Serial.println(header[0], HEX); 
			// Serial.println(header[1], HEX); 
		 }
    
    for(int i=0; i<10; i++) {
      temp[i] = mySerial5.read(); 
    }
    omega = temp[3]<<24 | temp[2]<<16 | temp[1]<<8 | temp[0];

    if(PRINT_SFOS200) {
		t_new = micros();
		Serial.print(t_new - t_old);
		Serial.print("\t");
		Serial.print(mySerial5.available()); 
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
		Serial.println(omega);
		t_old = t_new;
    }  
	// Serial1.write(0xC0);
    Serial1.write(temp[3]);
    Serial1.write(temp[2]);
    Serial1.write(temp[1]);
    Serial1.write(temp[0]);
}

void requestPP() {

	byte temp[10];
	int omega;
	byte header;

	while (mySerial13.available()<12) {};
	if(mySerial13.available()>230) {
		for(int i=0; i<220; i++) mySerial13.read(); 
	}
	header = mySerial13.read();
	while( header != 0xAB ) 
	{
		header = mySerial13.read();
		delay(1);
	}
    
    for(int i=0; i<4; i++) {
      temp[i] = mySerial13.read(); 
    }
    omega = temp[0]<<24 | temp[1]<<16 | temp[2]<<8 | temp[3];

    if(PRINT_PP) {
		Serial.print(millis());
		Serial.print("\t");
		Serial.print(mySerial13.available()); 
		Serial.print("\t");
		Serial.print(header, HEX);
		Serial.print("\t");    
		Serial.print(temp[0]);
		Serial.print("\t");
		Serial.print(temp[1]);
		Serial.print("\t");
		Serial.print(temp[2]);
		Serial.print("\t");
		Serial.print(temp[3]);
		Serial.print("\t");
		Serial.println(omega);
    }  
	// Serial1.write(0xC1);
    Serial1.write(temp[0]);
    Serial1.write(temp[1]);
    Serial1.write(temp[2]);
    Serial1.write(temp[3]);
}


void requestSpeed() {

  byte temp[10];
  int omega;
  byte header;

	while (Serial2.available()<16) {};
	if(Serial2.available()>230) {
		for(int i=0; i<220; i++) Serial2.read(); 
	}
		header = Serial2.read();
		while( header != 0xAB ) 
		{
			header = Serial2.read();
			delay(1);
		}
    
    for(int i=0; i<3; i++) {
      temp[i] = Serial2.read(); 
    }
    // omega = temp[0]<<24 | temp[1]<<16 | temp[2]<<8 | temp[3];
	omega = temp[0]<<16 | temp[1]<<8 | temp[2];

    if(PRINT_SPEED) {
		Serial.print(millis());
		Serial.print("\t");
		Serial.print(Serial2.available()); 
		Serial.print("\t");
		Serial.print(header, HEX);
		Serial.print("\t");    
		Serial.print(temp[0]);
		Serial.print("\t");
		Serial.print(temp[1]);
		Serial.print("\t");
		Serial.print(temp[2]);
		Serial.print("\t");
		// Serial.print(temp[3]);
		// Serial.print("\t");
		Serial.println(omega);
    }  
    Serial1.write(temp[0]);
    Serial1.write(temp[1]);
    Serial1.write(temp[2]);
    // Serial1.write(temp[3]);
}

void request_nano33_xlm() {
	int x, y, z;
  while (!IMU.accelerationAvailable()); 
    IMU.readAcceleration(x, y, z);
    if(PRINT_XLM) {
      Serial.print("nano33 axlm");
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


void request_nano33_gyro() {
	int x, y, z;
	
  while (!IMU.gyroscopeAvailable()); 
    IMU.readGyroscope(x, y, z);
    if(PRINT_GYRO) {
      Serial.print("nano33 gyro");
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
	// digitalWrite(FOG_CLK, 1);
    clk_status = 1;
  }
}

void checkByte(byte check) {
  Serial1.write(check);
}

void I2CWriteData(byte addr, byte val)
{
  Wire.beginTransmission(ADXL355_ADDR);//
  Wire.write(addr);//
  Wire.write(val);
  Wire.endTransmission();//
}

void Soft_I2CWriteData(byte addr, byte val)
{  
  sw.beginTransmission(ADXL355_ADDR);//
  sw.write(addr);//
  sw.write(val);
  sw.endTransmission();//
}


byte I2CReadData(byte addr)
{
	bool i=0;
	byte data;
	
	Wire.beginTransmission(ADXL355_ADDR);//
	Wire.write(addr);//
	Wire.endTransmission();
	
	Wire.requestFrom(ADXL355_ADDR,1);
	while (Wire.available())
	{
		data=Wire.read();
	}	
		
	return data;
}

byte Soft_I2CReadData(byte addr)
{
	bool i=0;
	byte data;
	
	
	sw.beginTransmission(ADXL355_ADDR);//
	sw.write(addr);//
	sw.endTransmission();
	
	sw.requestFrom(ADXL355_ADDR,1);
	while (sw.available())
	{
		data=sw.read();
	}	
	
	return data;
}

/* 
 * Read multiple registries
 */
void readMultipleData(int *addresses, int dataSize, int *readedData) {
  digitalWrite(CHIP_SELECT_PIN, LOW);
  for(int i = 0; i < dataSize; i = i + 1) {
    byte dataToSend = (addresses[i] << 1) | READ_BYTE;
    SPI.transfer(dataToSend);
    readedData[i] = SPI.transfer(0x00);
  }
  digitalWrite(CHIP_SELECT_PIN, HIGH);
}
