#include "Arduino.h"
#include "adxl355.h"

#define ADXL355_ADDR 		0x1D	//I2C address
#define PIN_SCL_EN			12
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

/*** ODR ***/
#define ODR_4000	0b0000
#define ODR_2000 	0b0001
#define ODR_1000 	0b0010
#define ODR_500 	0b0011
#define ODR_250 	0b0100
#define ODR_125 	0b0101

/*** sync parameter ***/
#define INT_SYNC			0x00
#define EXT_SYNC			0x01
#define EXT_SYNC_INTFILTER	0x02

/*** status reg mask***/
#define DATA_RDY_MSK 	0x01
#define FIFO_FULL_MSK 	0x02
#define FIFO_OVR_MSK 	0x04

/*** power control parameter***/
#define MEASURE_MODE	0x00 
#define TEMP_OFF_MSK	0x02


adxl355::adxl355(int scl_en)
{
	Wire.begin();
	Serial.begin(115200);
	_scl_en = scl_en;
}

void adxl355::init() 
{
	Wire.setClock(I2C_FAST_MODE);
	pinMode(PIN_SCL_EN, OUTPUT);
	p_scl_mux_enable();
	setRegVal(RST_ADDR, 0x52);
	setRegVal(RANGE_ADDR, F_MODE | RANGE_8G);
	setRegVal(FILTER_ADDR, ODR_500);
	setRegVal(SYNC_ADDR, EXT_SYNC); 
	setRegVal(POWER_CTL_ADDR, MEASURE_MODE);
	p_scl_mux_disable();
}
void adxl355::setRegVal(unsigned char addr, unsigned char val)
{
	p_I2CWriteData(addr, val);
	delay(1);
}

void adxl355::printRegVal(char name[], unsigned char addr, unsigned char rep)
{
	p_scl_mux_enable();
	Serial.print(name);
	Serial.print(": ");
	Serial.println(p_I2CReadData(addr), rep);
	p_scl_mux_disable();
}

void adxl355::printRegAll()
{
	printRegVal("DEV_ID", DEVID_AD_ADDR, HEX);
	printRegVal("SYNC", SYNC_ADDR, BIN);
	printRegVal("ODR", FILTER_ADDR, BIN);
}

void adxl355::p_I2CWriteData(unsigned char addr, unsigned char val)
{
	Wire.beginTransmission(ADXL355_ADDR);
	Wire.write(addr);
	Wire.write(val);
	Wire.endTransmission();
}

unsigned char adxl355::p_I2CReadData(unsigned char addr)
{
	unsigned char data;
	
	Wire.beginTransmission(ADXL355_ADDR);
	Wire.write(addr);
	Wire.endTransmission();
	Wire.requestFrom(ADXL355_ADDR,1);
	while (Wire.available()) data=Wire.read();
	return data;
}



void adxl355::p_scl_mux_enable()
{
	digitalWrite(PIN_SCL_EN, 0);
}

void adxl355::p_scl_mux_disable()
{
	digitalWrite(PIN_SCL_EN, 1);
	delayMicroseconds(20);
}

