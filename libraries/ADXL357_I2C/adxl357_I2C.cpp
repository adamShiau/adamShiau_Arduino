#include "adxl357_I2C.h"

Adxl357_I2C::Adxl357_I2C(TwoWire &p ) : myWire(p)
{
	_sercom_mode = True;
	Serial.begin(115200);
	// _scl_en = 12;
}

Adxl357_I2C::~Adxl357_I2C()
{
}

void Adxl357_I2C::init() 
{	
	/*** set adxl357 parameters ***/
	setRegVal(RST_ADDR, POR);
	setRegVal(RANGE_ADDR, F_MODE | RANGE_40G);
	setRegVal(FILTER_ADDR, ODR_500);
	// setRegVal(SYNC_ADDR, EXT_SYNC); 
	setRegVal(SYNC_ADDR, INT_SYNC); 
	setRegVal(POWER_CTL_ADDR, MEASURE_MODE);

	Serial.println("-------ADXL357 Parameters------");
	getSensitivity();
	getSyncMode();
	getDataODR_LPF();
	getTemperature();
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

/***
void Adxl357_I2C::readData(unsigned char temp_a[9]) 
{
	int accX, accY, accZ;
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
} 
*/


void Adxl357_I2C::readData(unsigned char temp_a[9]) 
{
	unsigned char i=0;

	while(!(p_I2CReadData(STATUS_ADDR) & DATA_RDY_MSK)) {}; //wait ADXL322 data available
	myWire.beginTransmission(ADXL357_ADDR);
	myWire.write(XDATA3_ADDR);
	myWire.endTransmission();
	myWire.requestFrom(ADXL357_ADDR,9);
	while (myWire.available()) temp_a[i++]=myWire.read();

} 

void Adxl357_I2C::readData_f(float acc[3]) 
{
	unsigned char temp_a[9];
	unsigned char i=0;
	int accX, accY, accZ;

	while(!(p_I2CReadData(STATUS_ADDR) & DATA_RDY_MSK)) {}; //wait ADXL322 data available
	myWire.beginTransmission(ADXL357_ADDR);
	myWire.write(XDATA3_ADDR);
	myWire.endTransmission();
	myWire.requestFrom(ADXL357_ADDR,9);
	while (myWire.available()) temp_a[i++]=myWire.read();
	
	accX = temp_a[0]<<12 | temp_a[1]<<4 | temp_a[2]>>4;
	if((accX>>19) == 1) accX = accX - (1<<20);
	accY = temp_a[3]<<12 | temp_a[4]<<4 | temp_a[5]>>4;
	if((accY>>19) == 1) accY = accY - (1<<20);
	accZ = temp_a[6]<<12 | temp_a[7]<<4 | temp_a[8]>>4;
	if((accZ>>19) == 1) accZ = accZ - (1<<20);
	
	acc[0] = (float)accX*p_snesitivity;
	acc[1] = (float)accY*p_snesitivity;
	acc[2] = (float)accZ*p_snesitivity;

	// Serial.print(acc[0]);
	// Serial.print('\t');
	// Serial.print(acc[1]);
	// Serial.print('\t');
	// Serial.println(acc[2]);

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
	// p_scl_mux_enable();
	Serial.print(name);
	Serial.print(": ");
	Serial.println(p_I2CReadData(addr), rep);
	// p_scl_mux_disable();
}

void Adxl357_I2C::printRegAll()
{
	printRegVal("DEV_ID", DEVID_AD_ADDR, HEX);
	printRegVal("SYNC", SYNC_ADDR, HEX);
	printRegVal("POWER_CTL", POWER_CTL_ADDR, HEX);
	printRegVal("ODR", FILTER_ADDR, HEX);
	printRegVal("INT MAP", INTERRUPT_ADDR, HEX);
	printRegVal("RANGE", RANGE_ADDR, HEX);
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

void Adxl357_I2C::p_I2CReadData(unsigned char addr, char num, char *data)
{
	char i=0;

	myWire.beginTransmission(ADXL357_ADDR);
	myWire.write(addr);
	myWire.endTransmission();
	myWire.requestFrom(ADXL357_ADDR,num);
	while (myWire.available()) data[i++]=myWire.read();
}

void Adxl357_I2C::print_Data_i2c(byte *temp_a)
{
	int accX, accY, accZ;
	
	accX = temp_a[0]<<12 | temp_a[1]<<4 | temp_a[2]>>4;
	if((accX>>19) == 1) accX = accX - (1<<20);
	accY = temp_a[3]<<12 | temp_a[4]<<4 | temp_a[5]>>4;
	if((accY>>19) == 1) accY = accY - (1<<20);
	accZ = temp_a[6]<<12 | temp_a[7]<<4 | temp_a[8]>>4;
	if((accZ>>19) == 1) accZ = accZ - (1<<20);
	
	// t_new = micros();
	// Serial.print(t_new - t_old);
	// Serial.print('\t');
	Serial.print((float)accX*p_snesitivity);
	Serial.print('\t');
	Serial.print((float)accY*p_snesitivity);
	Serial.print('\t');
	Serial.println((float)accZ*p_snesitivity);
	// t_old = t_new;
}


char Adxl357_I2C::getRange()
{
	char rt;
	rt = p_I2CReadData(RANGE_ADDR) & 0x03;
	Serial.print("Range: ");
	if(rt==0x01) Serial.println("10g");
	else if(rt==0x02) Serial.println("20g");
	else if(rt==0x03) Serial.println("40g");
	
	return rt;
}

void Adxl357_I2C::getSyncMode()
{
	char rt = p_I2CReadData(SYNC_ADDR) & 0x03;
	Serial.print("SYNC: ");
	if(rt==0x00) Serial.println("Internal Sync");
	else if(rt==0x02) Serial.println("External Sync");
}

void Adxl357_I2C::getSensitivity()
{
	char rt = getRange();
	
	Serial.print("Snesitivity: ");
	if(rt==0x01) p_snesitivity = SENS_10G;
	else if(rt==0x02) p_snesitivity = SENS_20G;
	else if(rt==0x03) p_snesitivity = SENS_40G;
	Serial.println(p_snesitivity, 7);
}

void Adxl357_I2C::getDataODR_LPF()
{
	char rt = p_I2CReadData(FILTER_ADDR) & 0x0F;
	Serial.print("[ODR, LPF] (Hz): ");
	if(rt==0x00) Serial.println("[4000, 1000]");
	else if(rt==0x01) Serial.println("[2000, 500]");
	else if(rt==0x02) Serial.println("[1000, 250]");
	else if(rt==0x03) Serial.println("[500, 125]");
	else if(rt==0x04) Serial.println("[250, 62.5]");
	else if(rt==0x05) Serial.println("[125, 31.25]");
	else if(rt==0x06) Serial.println("[62.5, 15.625]");
	else if(rt==0x07) Serial.println("[31.25, 7.813]");
	else if(rt==0x08) Serial.println("[15.625, 3.906]");
	else if(rt==0x09) Serial.println("[7.813, 1.953]");
	else if(rt==0x0A) Serial.println("[3.906, 0.977]");
}

float Adxl357_I2C::getTemperature()
{
	char temp[2];
	float T=0;
	p_I2CReadData(TEMP2_ADDR, 2, temp);
	T = (float)(1885 - ((temp[0]&0x0F)<<8 | temp[1]) )/9.05 + 25.0;
	// Serial.print(temp[0], HEX);
	// Serial.print(", ");
	// Serial.print(temp[1], HEX);
	// Serial.print(", ");
	// Serial.println(T);
	return T;
}