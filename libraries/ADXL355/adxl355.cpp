#include "adxl355.h"

#define ADXL355_ADDR 		0x1D	//Adxl355 I2C address
#define I2C_STANDARD_MODE 	100000
#define I2C_FAST_MODE 		400000

/*** register address***/
#define DEVID_AD_ADDR	0x00
#define STATUS_ADDR 	0x04
#define FIFO_ADDR 		0x05
#define TEMP2_ADDR 		0x06
#define TEMP1_ADDR  	0x07
#define XDATA3_ADDR  	0x08
#define XDATA2_ADDR  	0x09
#define XDATA1_ADDR  	0x0A
#define YDATA3_ADDR  	0x0B
#define YDATA2_ADDR  	0x0C
#define YDATA1_ADDR  	0x0D
#define ZDATA3_ADDR  	0x0E
#define ZDATA2_ADDR  	0x0F
#define ZDATA1_ADDR  	0x10
#define FILTER_ADDR  	0x28
#define INTERRUPT_ADDR 	0x2A
#define SYNC_ADDR  		0x2B
#define RANGE_ADDR  	0x2C
#define POWER_CTL_ADDR 	0x2D
#define RST_ADDR  		0x2F

/*** range reg parameter***/
#define H_MODE		0x80
#define F_MODE		0x00
#define RANGE_2G 	0x01
#define RANGE_4G 	0x02
#define RANGE_8G 	0x03

#define SENS_8G 0.0000156
#define SENS_4G 0.0000078
#define SENS_2G 0.0000039
/*** reset parameter ***/
#define POR 		0x52
/*** ODR parameter ***/
#define ODR_4000	0b0000
#define ODR_2000 	0b0001
#define ODR_1000 	0b0010
#define ODR_500 	0b0011
#define ODR_250 	0b0100
#define ODR_125 	0b0101

/*** sync parameter ***/
#define INT_SYNC	0x00
#define EXT_SYNC	0x02

/*** status reg mask***/
#define DATA_RDY_MSK 	0x01
#define FIFO_FULL_MSK 	0x02
#define FIFO_OVR_MSK 	0x04

/*** power control parameter***/
#define MEASURE_MODE	0x00 
#define TEMP_OFF_MSK	0x02

Adxl355::Adxl355(int scl_en)
{
	Serial.begin(115200);
	_scl_en = scl_en;
}

void Adxl355::init() 
{	
	Wire.begin();
	Wire.setClock(I2C_FAST_MODE);
	pinMode(_scl_en, OUTPUT);
	/*** set adxl355 parameters ***/
	p_scl_mux_enable();
	setRegVal(RST_ADDR, POR);
	setRegVal(RANGE_ADDR, F_MODE | RANGE_8G);
	setRegVal(FILTER_ADDR, ODR_500);
	setRegVal(SYNC_ADDR, EXT_SYNC); 
	setRegVal(POWER_CTL_ADDR, TEMP_OFF_MSK| MEASURE_MODE);
	p_scl_mux_disable();
}

void Adxl355::readData(unsigned char temp_a[9]) 
{
	int accX, accY, accZ;
	
	p_scl_mux_enable();
	while(!(p_I2CReadData(STATUS_ADDR) & DATA_RDY_MSK)) {}; //wait ADXL322 data available
	temp_a[0] = p_I2CReadData(XDATA3_ADDR); 
	temp_a[1] = p_I2CReadData(XDATA2_ADDR); 
	temp_a[2] = p_I2CReadData(XDATA1_ADDR); 

	temp_a[3] = p_I2CReadData(YDATA3_ADDR); 
	temp_a[4] = p_I2CReadData(YDATA2_ADDR); 
	temp_a[5] = p_I2CReadData(YDATA1_ADDR); 

	temp_a[6] = p_I2CReadData(ZDATA3_ADDR); 
	temp_a[7] = p_I2CReadData(ZDATA2_ADDR); 
	temp_a[8] = p_I2CReadData(ZDATA1_ADDR); 
	p_scl_mux_disable();
	
} 

void Adxl355::readFakeData(unsigned char temp_a[9]) 
{
	temp_a[0] = 0; 
	temp_a[1] = 0; 
	temp_a[2] = 3; 

	temp_a[3] = 0; 
	temp_a[4] = 0; 
	temp_a[5] = 5; 

	temp_a[6] = 0; 
	temp_a[7] = 0; 
	temp_a[8] = 5; 
	
} 


void Adxl355::setRegVal(unsigned char addr, unsigned char val)
{
	p_I2CWriteData(addr, val);
	// delay(1);
}

void Adxl355::printRegVal(char name[], unsigned char addr, unsigned char rep)
{
	p_scl_mux_enable();
	Serial.print(name);
	Serial.print(": ");
	Serial.println(p_I2CReadData(addr), rep);
	p_scl_mux_disable();
}

void Adxl355::printRegAll()
{
	printRegVal("DEV_ID", DEVID_AD_ADDR, HEX);
	printRegVal("SYNC", SYNC_ADDR, HEX);
	printRegVal("POWER_CTL", POWER_CTL_ADDR, HEX);
	printRegVal("ODR", FILTER_ADDR, HEX);
	printRegVal("INT MAP", INTERRUPT_ADDR, HEX);
}

void Adxl355::p_I2CWriteData(unsigned char addr, unsigned char val)
{
	Wire.beginTransmission(ADXL355_ADDR);
	Wire.write(addr);
	Wire.write(val);
	Wire.endTransmission();
}

unsigned char Adxl355::p_I2CReadData(unsigned char addr)
{
	unsigned char data;
	
	Wire.beginTransmission(ADXL355_ADDR);
	Wire.write(addr);
	Wire.endTransmission();
	Wire.requestFrom(ADXL355_ADDR,1);
	while (Wire.available()) data=Wire.read();
	return data;
}

void Adxl355::p_scl_mux_enable()
{
	digitalWrite(_scl_en, 0);
}

void Adxl355::p_scl_mux_disable()
{
	digitalWrite(_scl_en, 1);
	delayMicroseconds(20);
}
