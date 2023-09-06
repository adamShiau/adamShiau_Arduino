#include "adxl357_I2C.h"

Adxl357_I2C::Adxl357_I2C(TwoWire &p ) : myWire(p)
{
	_sercom_mode = True;
	Serial.begin(115200);
	_scl_en = 12;
}

Adxl357_I2C::~Adxl357_I2C()
{
}

void Adxl357_I2C::init() 
{	
	if(_sercom_mode){
		// myWire.begin();
		// myWire.setClock(I2C_FAST_MODE_PLUS);
	}
	
	else {
		Wire.begin();
		Wire.setClock(I2C_FAST_MODE);
	}
	
	pinMode(_scl_en, OUTPUT);
	/*** set adxl357 parameters ***/
	p_scl_mux_enable();
	setRegVal(RST_ADDR, POR);
	setRegVal(RANGE_ADDR, F_MODE | RANGE_10G);
	setRegVal(FILTER_ADDR, ODR_500);
	// setRegVal(SYNC_ADDR, EXT_SYNC); 
	setRegVal(SYNC_ADDR, INT_SYNC); 
	setRegVal(POWER_CTL_ADDR, TEMP_OFF_MSK| MEASURE_MODE);
	p_scl_mux_disable();
}

void Adxl357_I2C::testI2C()
{
	// Serial.println("hi");
	myWire.beginTransmission(ADXL357_ADDR);
	myWire.write(0x1D);
	myWire.write(0x01);
	myWire.endTransmission();
	delay(4);
}

void Adxl357_I2C::readData(unsigned char temp_a[9]) 
{
	int accX, accY, accZ;
	p_scl_mux_enable();
	while(!(p_I2CReadData(STATUS_ADDR) & DATA_RDY_MSK)) {}; //wait ADXL322 data available
	//Serial.println("pass");
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

void Adxl357_I2C::readFakeData(unsigned char temp_a[9]) 
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


void Adxl357_I2C::setRegVal(unsigned char addr, unsigned char val)
{
	p_I2CWriteData(addr, val);
	// Serial.print(123);
	// delay(1);
}

void Adxl357_I2C::printRegVal(char name[], unsigned char addr, unsigned char rep)
{
	p_scl_mux_enable();
	Serial.print(name);
	Serial.print(": ");
	Serial.println(p_I2CReadData(addr), rep);
	p_scl_mux_disable();
}

void Adxl357_I2C::printRegAll()
{
	printRegVal("DEV_ID", DEVID_AD_ADDR, HEX);
	printRegVal("SYNC", SYNC_ADDR, HEX);
	printRegVal("POWER_CTL", POWER_CTL_ADDR, HEX);
	printRegVal("ODR", FILTER_ADDR, HEX);
	printRegVal("INT MAP", INTERRUPT_ADDR, HEX);
	Serial.println();
}

void Adxl357_I2C::p_I2CWriteData(unsigned char addr, unsigned char val)
{
	if(_sercom_mode){
		myWire.beginTransmission(ADXL357_ADDR);
		myWire.write(addr);
		myWire.write(val);
		myWire.endTransmission();
	}
	else{
		Wire.beginTransmission(ADXL357_ADDR);
		Wire.write(addr);
		Wire.write(val);
		Wire.endTransmission();
	}
	
}

unsigned char Adxl357_I2C::p_I2CReadData(unsigned char addr)
{
	unsigned char data;
	
	if(_sercom_mode){
		myWire.beginTransmission(ADXL357_ADDR);
		myWire.write(addr);
		myWire.endTransmission();
		myWire.requestFrom(ADXL357_ADDR,1);
		while (myWire.available()) data=myWire.read();
	}
	else{
		Wire.beginTransmission(ADXL357_ADDR);
		Wire.write(addr);
		Wire.endTransmission();
		Wire.requestFrom(ADXL357_ADDR,1);
		while (Wire.available()) data=Wire.read();
	}

	return data;
}

void Adxl357_I2C::p_scl_mux_enable()
{
	digitalWrite(_scl_en, 0);
}

void Adxl357_I2C::p_scl_mux_disable()
{
	digitalWrite(_scl_en, 1);
	delayMicroseconds(20);
}
