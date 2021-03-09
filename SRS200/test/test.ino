#include <Arduino.h>
#include <Wire.h>

/*** ADXL355***/
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

const int CHIP_SELECT_PIN = 10;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  //Configure ADXL355:
  // writeRegister(RST, 0x52);
  I2CWriteData(RST, 0x52);
  delay(100);
  // writeRegister(RANGE, RANGE_8G); // 2G
  I2CWriteData(RANGE, RANGE_8G);
  delay(100);
  I2CWriteData(FILTER, 0b00000100); //ODR 0b100@250Hz 0b101@125Hz
  delay(100);
  I2CWriteData(SYNC, 0b010); 
  delay(100);
  I2CWriteData(POWER_CTL, MEASURE_MODE); // Enable measure mode
  // Give the sensor time to set up:
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("RST: ");
  Serial.println(I2CReadData(RST), HEX);
  Serial.print("RANGE: ");
  Serial.println(I2CReadData(RANGE), HEX);
  Serial.print("FILTER: ");
  Serial.println(I2CReadData(FILTER), HEX);
  Serial.print("SYNC: ");
  Serial.println(I2CReadData(SYNC), HEX);
  Serial.print("POWER_CTL: ");
  Serial.println(I2CReadData(POWER_CTL), HEX);
  delay(500);
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
