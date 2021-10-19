#include <Arduino.h>
#include "wiring_private.h"
#include <Arduino_LSM6DS3.h>
#include <SPI.h>
#include <Wire.h>
// #include <SoftWire.h>
// #include <AsyncDelay.h>

/*** Nano33 ***/
#define SENS_XLM 0.000122 // +/- 4g, 4/32768
#define SENS_GYRO 0.00763 // +/- 250dps, 250/32768

/*** ADXL355***/
#define SENS_8G 0.0000156
#define SENS_4G 0.0000078
#define SENS_2G 0.0000039
#define ADXL355_ADDR 0x1D

/*** VBOX HEADER***/
const char VBOX_HEADER[7] = {'$', 'V', 'B', '3', 'i', 's', '$'};
const byte GPSSAT_INCR = 7;
const byte LATITUDE_INCR = 13;
const byte LONGITUDE_INCR = 17;
const byte VELOCITY_INCR = 21;
const byte ALTITUDE_INCR = 26;
const byte V_VELOCITY_INCR = 29;
const byte PITCH_INCR = 33;
const byte ROLL_INCR = 35;
const byte HEADING_INCR = 37;
const byte PITCH_RATE = 39;
const byte ROLL_RATE = 41;
const byte YAW_RATE = 43;
const byte ACCZ_INCR = 49;
const byte CRC_INCR = 71;

/*** Adxl355 gloabal var***/
byte temp_ax1, temp_ax2, temp_ax3;
byte temp_ay1, temp_ay2, temp_ay3;
byte temp_az1, temp_az2, temp_az3;
byte temp1, temp2;

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
#define PRINT_PP 1
#define PRINT_UART2 0
#define PRINT_SPEED 0 
#define PRINT_VBOX 0
#define PRINT_VBOX_BAD_FLAG 0


#define SERIAL2_RX 3
#define SERIALHCI_RX 9
#define FOG_CLK 14
#define PERIOD 10000
#define sdaPin  11
#define sclPin  10
#define I2C_MUXEN  12

bool clk_status = 1, clk_status_old = 0;
unsigned long start_time = 0, old_time = 0;
unsigned int t_old=0, t_new;
bool vbox_bad_flag_arr[3] = {0, 0, 0};
bool vbox_bad_flag = 0, vbox_init_flag = 1;
byte bf_idx = 0;
byte g_gpssat;
int g_latitude, g_longitude, g_velocity, g_altitude, g_v_velocity;
short g_pitch, g_roll, g_heading, g_p_rate, g_r_rate, g_y_rate;
int g_accz;

Uart mySerial5 (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
Uart mySerial13 (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);

// SoftWire sw(sdaPin, sclPin);
// char swTxBuffer[24];
// char swRxBuffer[24];

// AsyncDelay readInterval;

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
   Wire.begin();
   // Wire.setClock(400000L);
  
  // sw.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
  // sw.setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
  // sw.setDelay_us(5);
  // sw.setTimeout(1000);
  // sw.setClock(1000000);
  // sw.begin();
  
  // Reassign pins 5 and 6 to SERCOM alt
  pinPeripheral(5, PIO_SERCOM_ALT); //RX
  pinPeripheral(6, PIO_SERCOM_ALT); //TX

  // Reassign pins 13 and 8 to SERCOM (not alt this time)
  pinPeripheral(13, PIO_SERCOM);
  pinPeripheral(8, PIO_SERCOM);

  mySerial5.begin(115200); //rx:p5, tx:p6, SRS200
  mySerial13.begin(115200);//rx:p13, tx:p8, PP
  Serial.begin(115200); // arduino monitor
  Serial1.begin(115200); //for HC-05
  Serial2.begin(115200); //vbox
  SerialHCI.begin(115200); // uart2
  
  pinMode(FOG_CLK,INPUT);
  pinMode(SERIAL2_RX,INPUT);
  pinMode(I2C_MUXEN, OUTPUT);

  pinMode(SERIALHCI_RX,INPUT);
  digitalWrite(I2C_MUXEN, 0);
  
  //Configure ADXL355:
	I2CWriteData(RST, 0x52);
	delay(100);
	I2CWriteData(RANGE, RANGE_8G);
	delay(100);
	I2CWriteData(FILTER, 0b100); //ODR 0b100@250Hz 0b101@125Hz
	delay(100);
	I2CWriteData(SYNC, 0b010); 
	delay(100);
	I2CWriteData(POWER_CTL, MEASUREwTEMP_MODE); // Enable measure mode
	// Give the sensor time to set up:
	delay(100);
  if (!IMU.begin()) {
   while (1);
 }
	Wire.setClock(400000L); //don't move to otrher position
	digitalWrite(I2C_MUXEN, 1);
	
}
void loop() {
  int ax, ay, az, wx, wy, wz;
		clk_status = digitalRead(FOG_CLK);
		// Serial.print(clk_status);
		// Serial.print("\t");
		// Serial.print(clk_status_old);
		// Serial.print("\t");
		// Serial.println((bool)clk_status & ~clk_status_old);
		if(clk_status & ~clk_status_old) 
		{	
			start_time = millis();
			if(PRINT_UPDATE_TIME) Serial.println(start_time - old_time);
			old_time = start_time;
			 checkByte(0xAA);
			 checkByte(0xAC);
			send_current_time(start_time);
			
			// requestSFOS200();
			requestPP();
			// request_adxl355(ax, ay, az);
			// request_nano33_xlm();//for test
			// request_nano33_gyro(); 
			// requestVBOX();
			checkByte(0xAB);
		}
	   clk_status_old = clk_status;
}

void request_adxl355(int accX, int accY, int accZ) {
  // byte temp_ax1, temp_ax2, temp_ax3;
  // byte temp_ay1, temp_ay2, temp_ay3;
  // byte temp_az1, temp_az2, temp_az3;
  // byte temp1, temp2;
  byte status;
  int RT;
  float RTf;
  bool isReady;
	digitalWrite(I2C_MUXEN, 0);
	status = I2CReadData(STATUS); //0x04
	isReady = status&0b00000001;
	// while(!isReady) {
		// status = I2CReadData(STATUS);
		// isReady = status&0b00000001;
		// delay(1);
	// }
	// Serial.print("DEVID_AD: "); 
	// Serial.println(I2CReadData(DEVID_AD), HEX);
	// Serial.print("RANGE: "); 
	// Serial.println(I2CReadData(RANGE), BIN); 
	// Serial.print("FILTER: "); 
	// Serial.println(I2CReadData(FILTER), BIN);  
  if(isReady){
      temp_ax1 = I2CReadData(XDATA3); //0x08
      temp_ax2 = I2CReadData(XDATA2); //0x09
      temp_ax3 = I2CReadData(XDATA1); //0x0A
      accX = temp_ax1<<12 | temp_ax2<<4 | temp_ax3>>4;
      if((accX>>19) == 1) accX = accX - 1048576;
    
      temp_ay1 = I2CReadData(YDATA3); //0x0B
      temp_ay2 = I2CReadData(YDATA2); //0x0C
      temp_ay3 = I2CReadData(YDATA1); //0x0D
      accY = temp_ay1<<12 | temp_ay2<<4 | temp_ay3>>4;
      if((accY>>19) == 1) accY = accY - 1048576;
    
      temp_az1 = I2CReadData(ZDATA3); //0x0E
      temp_az2 = I2CReadData(ZDATA2); //0x0F
      temp_az3 = I2CReadData(ZDATA1); //0x10
      accZ = temp_az1<<12 | temp_az2<<4 | temp_az3>>4;
      if((accZ>>19) == 1) accZ = accZ - 1048576;
		
	  temp1 = I2CReadData(TEMP1);
	  temp2 = I2CReadData(TEMP2);
	  RT = (temp2&0x07)<<8 | temp1;
	  // RTf = 25 - ((float)RT-1885.0)/9.05;
		digitalWrite(I2C_MUXEN, 1);
      if(PRINT_ADXL355)
      {
        t_new = micros();
        Serial.print(t_new - t_old);
		Serial.print('\t');
        // Serial.print(", ");
		// Serial.print(RT);
        // Serial.print(", ");
        // Serial.print(accX);
        // Serial.print(", ");
		// Serial.print("ADXL355: ");
        // Serial.print('\t');
        Serial.print((float)accX*SENS_8G);
        Serial.print('\t');
        // Serial.print(accY);
        // Serial.print(", ");
        Serial.print((float)accY*SENS_8G);
        Serial.print('\t');
        // Serial.print(accZ);
        // Serial.print(", ");
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
	else {
		digitalWrite(I2C_MUXEN, 1);
		Serial1.write(temp_ax1);
		Serial1.write(temp_ax2);
		Serial1.write(temp_ax3);
		Serial1.write(temp_ay1);
		Serial1.write(temp_ay2);
		Serial1.write(temp_ay3);
		Serial1.write(temp_az1);
		Serial1.write(temp_az2);
		Serial1.write(temp_az3);
		Serial1.write(temp2);
		Serial1.write(temp1);
	}
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
  byte bf;
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
	
	bf = mySerial5.available();
	Serial1.write(bf);
}

void requestSRS200() {

  // byte temp[10];
  // int omega;
  // byte header[2];
  byte bf;

	
	// while (mySerial13.available()<12) {
	  // }; 
	  
	// if(mySerial13.available()>230) {
		// for(int i=0; i<220; i++) mySerial5.read(); 
	// }
		// header[0] = mySerial13.read();
		// header[1] = mySerial13.read();
	// while( ((header[0]!=0xC0)||(header[1]!=0xC0))) 
	// {
		// header[0] = mySerial13.read();
		// header[1] = mySerial13.read();
		// delay(1);
	// }
    
    // for(int i=0; i<10; i++) {
      // temp[i] = mySerial13.read(); 
    // }
    // omega = temp[3]<<24 | temp[2]<<16 | temp[1]<<8 | temp[0];

    // if(PRINT_SRS200) {
		// t_new = micros();
		// Serial.print("SRS200_2: ");
		// Serial.print("\t");
		// Serial.print(t_new - t_old);
		// Serial.print("\t");
		// Serial.print(mySerial13.available()); 
		// Serial.print("\t");
		// Serial.print(header[0]<<8|header[1], HEX);
		// Serial.print("\t");    
		// Serial.print(temp[3]);
		// Serial.print("\t");
		// Serial.print(temp[2]);
		// Serial.print("\t");
		// Serial.print(temp[1]);
		// Serial.print("\t");
		// Serial.print(temp[0]);
		// Serial.print("\t");
		// Serial.println(omega);
		// t_old = t_new;
    // }  
	for(int i=0; i<12; i++) {
		Serial1.write(mySerial5.read());
	}
	bf = mySerial5.available();
	Serial1.write(bf);
	// Serial.println(bf);
    // Serial1.write(temp[3]);
    // Serial1.write(temp[2]);
    // Serial1.write(temp[1]);
    // Serial1.write(temp[0]);
}

void request_UART2() {

	byte temp[10];
	int omega;
	byte header;

	while (SerialHCI.available()<12) {};
	if(SerialHCI.available()>230) {
		for(int i=0; i<220; i++) SerialHCI.read(); 
	}
	header = SerialHCI.read();
	while( header != 0xAB ) 
	{
		header = SerialHCI.read();
		delay(1);
	}
    
    for(int i=0; i<4; i++) {
      temp[i] = SerialHCI.read(); 
    }
    omega = temp[0]<<24 | temp[1]<<16 | temp[2]<<8 | temp[3];

    if(PRINT_UART2) {
		Serial.print(millis());
		Serial.print("\t");
		Serial.print("UART2");
		Serial.print("\t");
		Serial.print(SerialHCI.available()); 
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
		// mySerial13.write(0xAA);
    }  
	// Serial1.write(0xC1);
    Serial1.write(temp[0]);
    Serial1.write(temp[1]);
    Serial1.write(temp[2]);
    Serial1.write(temp[3]);
}

void requestVBOX() {
	
	char header[7];
	byte VBOX_temp[65];
	// byte crc_chk[72];
	byte gpssat;
	int latitude, longitude, velocity, altitude, v_velocity;
	short pitch, roll, heading, p_rate, r_rate, y_rate;
	int accz;
	int crc;

	while (Serial2.available()<10) {};
	
	Serial2.readBytes(header, 7);
	
	while( ((header[0]!=VBOX_HEADER[0])||(header[1]!=VBOX_HEADER[1])||(header[2]!=VBOX_HEADER[2])||
	(header[3]!=VBOX_HEADER[3])||(header[4]!=VBOX_HEADER[4])||(header[5]!=VBOX_HEADER[5])||(header[6]!=VBOX_HEADER[6]))) 
	{
		for(int j=0; j<6; j++) {
			header[j] = header[j+1];
		}
		header[6] = Serial2.read();
		delayMicroseconds(100);
	}
	// Serial.println(header);
	Serial2.readBytes(VBOX_temp, 65);
	
	// for(int i=0; i<72; i++) {
		// if(i<7) temp = header[i];
		// else temp = VBOX_temp[i];
		
		// crc = crc ^ (int(temp) << 8);
		// crc = crc%65536;
		
		// for(int j = 0; j<8; j++) {
			// if ( (crc & 32768)==32768){
				// crc = crc << 1 ;
				// crc = crc ^ 4129;
			// }
			// else crc= crc << 1 ;
		// crc = crc % 65536;
		// }
	// }
	
	// Serial.println(crc);	
	
	// for(int i=0; i<50; i++) {
		// Serial.print(VBOX_temp[i], HEX);
		// Serial.print(", ");
	// }
	// Serial.println("");
	gpssat = VBOX_temp[GPSSAT_INCR-7];
	latitude = VBOX_temp[LATITUDE_INCR-7]<<24 | VBOX_temp[LATITUDE_INCR-6]<<16 | VBOX_temp[LATITUDE_INCR-5]<<8| VBOX_temp[LATITUDE_INCR-4];
	longitude = VBOX_temp[LONGITUDE_INCR-7]<<24 | VBOX_temp[LONGITUDE_INCR-6]<<16 | VBOX_temp[LONGITUDE_INCR-5]<<8| VBOX_temp[LONGITUDE_INCR-4]; 
	velocity = VBOX_temp[VELOCITY_INCR-7]<<16 | VBOX_temp[VELOCITY_INCR-6]<<8| VBOX_temp[VELOCITY_INCR-5]; 
	altitude = VBOX_temp[ALTITUDE_INCR-7]<<16 | VBOX_temp[ALTITUDE_INCR-6]<<8| VBOX_temp[ALTITUDE_INCR-5]; 
	v_velocity = (VBOX_temp[V_VELOCITY_INCR-7]<<16 | VBOX_temp[V_VELOCITY_INCR-6]<<8| VBOX_temp[V_VELOCITY_INCR-5]); 
	// Serial.println(v_velocity);
	// if((v_velocity >> 23)) v_velocity = v_velocity - 16777216;
	p_rate = VBOX_temp[PITCH_RATE-7]<<8 | VBOX_temp[PITCH_RATE-6];
	r_rate = VBOX_temp[ROLL_RATE-7]<<8 | VBOX_temp[ROLL_RATE-6];
	y_rate = VBOX_temp[YAW_RATE-7]<<8 | VBOX_temp[YAW_RATE-6];
	pitch = VBOX_temp[PITCH_INCR-7]<<8 | VBOX_temp[PITCH_INCR-6];
	roll = VBOX_temp[ROLL_INCR-7]<<8 | VBOX_temp[ROLL_INCR-6];
	heading = VBOX_temp[HEADING_INCR-7]<<8 | VBOX_temp[HEADING_INCR-6];
	accz = VBOX_temp[ACCZ_INCR-7]<<8 | VBOX_temp[ACCZ_INCR-6];
	crc = VBOX_temp[CRC_INCR-7]<<8 | VBOX_temp[CRC_INCR-6];
	
	if(vbox_init_flag) {
		vbox_init_flag = 0;
		g_accz = accz;
		g_gpssat = gpssat;
		g_latitude = latitude;
		g_longitude = longitude;
		g_velocity = velocity;
		g_altitude = altitude;
		g_v_velocity = v_velocity;
		g_pitch = pitch;
		g_roll = roll;
		g_heading = heading;
		g_p_rate = p_rate;
		g_r_rate = r_rate;
		g_y_rate = y_rate;
		// g_accz = accz;
	}
	
	if( (g_accz-accz)>200 || (g_accz-accz)<-200 ) vbox_bad_flag_arr[bf_idx] = 1;
	else vbox_bad_flag_arr[bf_idx] = 0;
	
	vbox_bad_flag = vbox_bad_flag_arr[bf_idx];
	bf_idx++;
	if(bf_idx==3) bf_idx = 0;
	
	if( vbox_bad_flag_arr[0]&&vbox_bad_flag_arr[1]&&vbox_bad_flag_arr[2] ) vbox_bad_flag = 0;
	
	if(PRINT_VBOX_BAD_FLAG) {
		Serial.print(g_accz);
		Serial.print("\t");
		Serial.print(accz);
		Serial.print("\t");
		Serial.print(vbox_bad_flag);
		Serial.print("\t");
		Serial.print(vbox_bad_flag_arr[0]);
		Serial.print("\t");
		Serial.print(vbox_bad_flag_arr[1]);
		Serial.print("\t");
		Serial.print(vbox_bad_flag_arr[2]);
		Serial.print("\n");
	}
	
	if(!vbox_bad_flag) {
		g_accz = accz;
		g_gpssat = gpssat;
		g_latitude = latitude;
		g_longitude = longitude;
		g_velocity = velocity;
		g_altitude = altitude;
		g_v_velocity = v_velocity;
		g_pitch = pitch;
		g_roll = roll;
		g_heading = heading;
		g_p_rate = p_rate;
		g_r_rate = r_rate;
		g_y_rate = y_rate;
		g_accz = accz;
	}
	// vbox_bad_flag_r = vbox_bad_flag;
	
	if(PRINT_VBOX) {
		Serial.print(millis());
		Serial.print("\t");
		Serial.print(Serial2.available()); 
		Serial.print("\t");
		Serial.print(g_gpssat);
		Serial.print("\t");
		Serial.print(g_latitude);
		Serial.print("\t");
		Serial.print(g_longitude);
		Serial.print("\t");
		Serial.print(g_velocity);
		Serial.print("\t");
		Serial.print(g_altitude);
		Serial.print("\t");
		Serial.print(g_v_velocity);
		Serial.print("\t");
		Serial.print(g_pitch);
		Serial.print("\t");
		Serial.print(g_roll);
		Serial.print("\t");
		Serial.print(g_heading);
		Serial.print("\t");
		Serial.print(g_p_rate);
		Serial.print("\t");
		Serial.print(g_r_rate);
		Serial.print("\t");
		Serial.print(g_y_rate);
		Serial.print("\t");
		Serial.print((short)g_accz);
		Serial.print("\t");
		Serial.print(crc);
		Serial.print("\n");		
	}  
	
	Serial1.write(g_gpssat);
	Serial1.write(g_latitude>>24);
	Serial1.write(g_latitude>>16);
	Serial1.write(g_latitude>>8);
	Serial1.write(g_latitude);
	Serial1.write(g_longitude>>24);
	Serial1.write(g_longitude>>16);
	Serial1.write(g_longitude>>8);
	Serial1.write(g_longitude);
	Serial1.write(g_velocity>>16);
	Serial1.write(g_velocity>>8);
	Serial1.write(g_velocity);
	Serial1.write(g_altitude>>16);
	Serial1.write(g_altitude>>8);
	Serial1.write(g_altitude);
	Serial1.write(g_v_velocity>>16);
	Serial1.write(g_v_velocity>>8);
	Serial1.write(g_v_velocity);
	Serial1.write(g_pitch>>8);
	Serial1.write(g_pitch);
	Serial1.write(g_roll>>8);
	Serial1.write(g_roll);
	Serial1.write(g_heading>>8);
	Serial1.write(g_heading);
	Serial1.write(g_accz>>8);
	Serial1.write(g_accz);
}

void requestVBOX_bk() {

	byte VBOX_temp[150];
	byte idx;
	long speed;
	int gpssat, latitude, longitude, velocity, altitude, v_velocity, pitch, roll, heading, accz;
	short p_rate, r_rate, y_rate;
	// char header[7];
	idx = 0;
	while (Serial2.available()<150) {};
	Serial2.readBytes(VBOX_temp, 150);
	// for(int i=0; i<73; i++) {
		// temp[i] = Serial2.read();
		// Serial.print((char)temp[i]);
	// }
	// Serial.println("");
	
	while(1)
	{
		for(int i=0; i<73; i++) {
			if( ((VBOX_temp[i]==VBOX_HEADER[0])&&(VBOX_temp[i+1]==VBOX_HEADER[1])&&(VBOX_temp[i+2]==VBOX_HEADER[2])&&(VBOX_temp[i+3]==VBOX_HEADER[3])&&(VBOX_temp[i+4]==VBOX_HEADER[4])&&(VBOX_temp[i+5]==VBOX_HEADER[5])&&(VBOX_temp[i+6]==VBOX_HEADER[6])) )
			{
				idx = i;
				break;
			}
		}
		break;
	}
	// for(int i=73; i<150; i++) {
		// VBOX_temp[i] = Serial2.read();
	// }
	Serial.print("idx: ");
	Serial.println(idx);
	
	for(int i=idx; i<idx+7; i++) Serial.print((char)VBOX_temp[i]);
	Serial.println("");
	
	// while( ((header[0]!=VBOX_HEADER[0])||(header[1]!=VBOX_HEADER[1])||(header[2]!=VBOX_HEADER[2])||
	// (header[3]!=VBOX_HEADER[3])||(header[4]!=VBOX_HEADER[4])||(header[5]!=VBOX_HEADER[5])||(header[6]!=VBOX_HEADER[6]))) 
	// {
		// for(int j=0; j<6; j++) {
			// Serial.print(header[j]);
			// Serial.print("\t");
			// header[j] = header[j+1];
		// }
		// header[6] = Serial2.read();
		// Serial.println(header[6]);
	// }
	 
	// for(int i=0; i<67; i++) temp[i] = Serial2.read();
	
	// gpssat = temp[GPSSAT_INCR];
	// latitude = temp[LATITUDE_INCR]<<24 | temp[LATITUDE_INCR+1]<<16 | temp[LATITUDE_INCR+2]<<8| temp[LATITUDE_INCR+3];
	// longitude = temp[LONGITUDE_INCR]<<24 | temp[LONGITUDE_INCR+1]<<16 | temp[LONGITUDE_INCR+2]<<8| temp[LONGITUDE_INCR+3]; 
	// velocity = temp[VELOCITY_INCR]<<16 | temp[VELOCITY_INCR+1]<<8| temp[VELOCITY_INCR+2]; 
	// altitude = temp[ALTITUDE_INCR]<<16 | temp[ALTITUDE_INCR+1]<<8| temp[ALTITUDE_INCR+2]; 
	// v_velocity = temp[V_VELOCITY_INCR]<<16 | temp[V_VELOCITY_INCR+1]<<8| temp[V_VELOCITY_INCR+2]; 
	p_rate = VBOX_temp[idx + PITCH_RATE]<<8 | VBOX_temp[idx + PITCH_RATE+1];
	r_rate = VBOX_temp[idx + ROLL_RATE]<<8 | VBOX_temp[idx + ROLL_RATE+1];
	y_rate = VBOX_temp[idx + YAW_RATE]<<8 | VBOX_temp[idx + YAW_RATE+1];
	pitch = VBOX_temp[idx + PITCH_INCR]<<8 | VBOX_temp[idx + PITCH_INCR+1];
	roll = VBOX_temp[idx + ROLL_INCR]<<8 | VBOX_temp[idx + ROLL_INCR+1];
	heading = VBOX_temp[idx + HEADING_INCR]<<8 | VBOX_temp[idx + HEADING_INCR+1];
	accz = VBOX_temp[idx + ACCZ_INCR]<<8 | VBOX_temp[idx + ACCZ_INCR+1];
	
	 
	 /*** ***/
	if(PRINT_VBOX) {
		Serial.print(millis());
		Serial.print("\t");
		Serial.print(Serial2.available()); 
		Serial.print("\t");
		Serial.print(gpssat);
		Serial.print("\t");
		Serial.print(latitude);
		Serial.print("\t");
		Serial.print(longitude);
		Serial.print("\t");
		Serial.print(velocity);
		Serial.print("\t");
		Serial.print(altitude);
		Serial.print("\t");
		Serial.print(v_velocity);
		Serial.print("\t");
		Serial.print(pitch);
		Serial.print("\t");
		Serial.print(roll);
		Serial.print("\t");
		Serial.print(heading);
		Serial.print("\t");
		// Serial.print(p_rate);
		// Serial.print("\t");
		// Serial.print(r_rate);
		// Serial.print("\t");
		// Serial.print(y_rate);
		// Serial.print("\t");
		Serial.print(accz);
		Serial.print("\n");
    }  
	// Serial1.write(0xAB);
//  Serial1.write(byte(0));
    Serial1.write(accz>>8);
    Serial1.write(accz);
}


void request_nano33_xlm() {
	int x, y, z;
  while (!IMU.accelerationAvailable()); 
    IMU.readAcceleration(x, y, z);
    if(PRINT_XLM) {
      Serial.print("nano33: ");
      Serial.print('\t');
      Serial.print(x*SENS_XLM);
      Serial.print('\t');
      Serial.print(y*SENS_XLM);
      Serial.print('\t');
      Serial.println(z*SENS_XLM);
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
      Serial.print(x*SENS_GYRO);
      Serial.print('\t');
      Serial.print(y*SENS_GYRO);
      Serial.print('\t');
      Serial.println(z*SENS_GYRO);
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

// void Soft_I2CWriteData(byte addr, byte val)
// {  
  // sw.beginTransmission(ADXL355_ADDR);//
  // sw.write(addr);//
  // sw.write(val);
  // sw.endTransmission();//
// }


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

// byte Soft_I2CReadData(byte addr)
// {
	// bool i=0;
	// byte data;
	
	
	// sw.beginTransmission(ADXL355_ADDR);//
	// sw.write(addr);//
	// sw.endTransmission();
	
	// sw.requestFrom(ADXL355_ADDR,1);
	// while (sw.available())
	// {
		// data=sw.read();
	// }	
	
	// return data;
// }

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
