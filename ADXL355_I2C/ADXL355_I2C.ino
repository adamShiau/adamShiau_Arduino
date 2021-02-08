#include <Wire.h>

#define SENS_8G 0.0000156
#define SENS_4G 0.0000078
#define SENS_2G 0.0000039
#define ADXL355_ADDR 0x1D

// Memory register addresses:
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

// Operations
const int READ_BYTE = 0x01;
const int WRITE_BYTE = 0x00;

// Pins used for the connection with the sensor
const int CHIP_SELECT_PIN = 10;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  


 I2CWriteData(RST, 0x52);
  delay(100);
  I2CWriteData(RANGE, RANGE_8G);
  delay(100);
  I2CWriteData(FILTER, 0b101); //ODR 0b100@250Hz 0b101@125Hz
  delay(100);
  I2CWriteData(SYNC, 0b010); 
  delay(100);
  I2CWriteData(POWER_CTL, MEASURE_MODE); // Enable measure mode
  // Give the sensor time to set up:
  delay(100);
}
unsigned int t_old=0, t_new;
void loop() {
  byte temp_ax1, temp_ax2, temp_ax3;
  byte temp_ay1, temp_ay2, temp_ay3;
  byte temp_az1, temp_az2, temp_az3;
  int accX, accY, accZ; 
  byte status;
  bool isReady;



	status = I2CReadData(STATUS);
	isReady = status&0b00000001;
	// Serial.print("status: ");
	// Serial.println(status, BIN);
	// Serial.print("isReady: ");
	// Serial.println(isReady);
	
  if(isReady == 1)
  {
  temp_ax1 = I2CReadData(XDATA3);
      temp_ax2 = I2CReadData(XDATA2);
      temp_ax3 = I2CReadData(XDATA1);
      accX = temp_ax1<<12 | temp_ax2<<4 | temp_ax3>>4;
      if((accX>>19) == 1) accX = accX - 1048576;
    
      temp_ay1 = I2CReadData(YDATA3);
      temp_ay2 = I2CReadData(YDATA2);
      temp_ay3 = I2CReadData(YDATA1);
      accY = temp_ay1<<12 | temp_ay2<<4 | temp_ay3>>4;
      if((accY>>19) == 1) accY = accY - 1048576;
    
      temp_az1 = I2CReadData(ZDATA3);
      temp_az2 = I2CReadData(ZDATA2);
      temp_az3 = I2CReadData(ZDATA1);
      accZ = temp_az1<<12 | temp_az2<<4 | temp_az3>>4;
      if((accZ>>19) == 1) accZ = accZ - 1048576;
    t_new = micros();
        Serial.print(t_new - t_old);
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
        t_old = t_new;
  }
  
}

void I2CWriteData(byte addr, byte val)
{
  Wire.beginTransmission(ADXL355_ADDR);//
  Wire.write(addr);//
  Wire.write(val);
  Wire.endTransmission();//
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
