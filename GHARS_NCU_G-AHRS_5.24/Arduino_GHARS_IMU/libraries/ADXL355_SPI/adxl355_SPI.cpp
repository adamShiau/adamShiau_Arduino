#include "adxl355_SPI.h"


#define SPI_CLOCK_8M 8000000
#define CHIP_SELECT_PIN 2

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

#define True 1
#define False 0

#define READ_BYTE  0x01
#define WRITE_BYTE 0x00


Adxl355_SPI::Adxl355_SPI(SPIClass &p , char ss) : mySPI(p)
{
	_sercom_mode = True;
	Serial.begin(115200);
	_ss = ss;
}

Adxl355_SPI::~Adxl355_SPI()
{
}

void Adxl355_SPI::init() 
{	
	pinMode(_ss, OUTPUT);
	digitalWrite(_ss, HIGH);
	/*** set adxl355 parameters ***/
	// p_scl_mux_enable();
	setRegVal(RST_ADDR, POR);
	setRegVal(RANGE_ADDR, F_MODE | RANGE_8G);
	setRegVal(FILTER_ADDR, ODR_500);
	// setRegVal(SYNC_ADDR, EXT_SYNC); 
	setRegVal(SYNC_ADDR, INT_SYNC); 
	setRegVal(POWER_CTL_ADDR, TEMP_OFF_MSK| MEASURE_MODE);
	// p_scl_mux_disable();
}


void Adxl355_SPI::readData(unsigned char temp_a[9]) 
{
	int accX, accY, accZ;
	// p_scl_mux_enable();
	while(!(p_SPIReadData(STATUS_ADDR) & DATA_RDY_MSK)) {}; //wait ADXL322 data available
	//Serial.println("pass");
	temp_a[0] = p_SPIReadData(XDATA3_ADDR); 
	temp_a[1] = p_SPIReadData(XDATA2_ADDR); 
	temp_a[2] = p_SPIReadData(XDATA1_ADDR); 

	temp_a[3] = p_SPIReadData(YDATA3_ADDR); 
	temp_a[4] = p_SPIReadData(YDATA2_ADDR); 
	temp_a[5] = p_SPIReadData(YDATA1_ADDR); 

	temp_a[6] = p_SPIReadData(ZDATA3_ADDR); 
	temp_a[7] = p_SPIReadData(ZDATA2_ADDR); 
	temp_a[8] = p_SPIReadData(ZDATA1_ADDR); 
	// p_scl_mux_disable();
} 

void Adxl355_SPI::readFakeData(unsigned char temp_a[9]) 
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


void Adxl355_SPI::setRegVal(unsigned char addr, unsigned char val)
{
	p_SPIWriteData(addr, val);
	// Serial.print(123);
	// delay(1);
}

void Adxl355_SPI::printRegVal(char name[], unsigned char addr, unsigned char rep)
{
	// p_scl_mux_enable();
	Serial.print(name);
	Serial.print(": ");
	Serial.println(p_SPIReadData(addr), rep);
	// p_scl_mux_disable();
}

void Adxl355_SPI::printRegAll()
{
	printRegVal("DEV_ID", DEVID_AD_ADDR, HEX);
	printRegVal("SYNC", SYNC_ADDR, HEX);
	printRegVal("POWER_CTL", POWER_CTL_ADDR, HEX);
	printRegVal("ODR", FILTER_ADDR, HEX);
	printRegVal("INT MAP", INTERRUPT_ADDR, HEX);
	Serial.println();
}

void Adxl355_SPI::p_SPIWriteData(unsigned char addr, unsigned char val)
{
	unsigned char dataToSend = (addr << 1) | WRITE_BYTE;
	
	digitalWrite(_ss, LOW);
	mySPI.transfer(dataToSend);
	mySPI.transfer(val);
	digitalWrite(_ss, HIGH);
	
}

unsigned char Adxl355_SPI::p_SPIReadData(unsigned char addr)
{
	unsigned char data;
	unsigned char dataToSend = (addr << 1) | READ_BYTE;
	
	digitalWrite(_ss, LOW);
	mySPI.transfer(dataToSend);
	data = mySPI.transfer(0x00);
	digitalWrite(_ss, HIGH);
	
	return data;
}

// void Adxl355_SPI::p_scl_mux_enable()
// {
	// digitalWrite(_scl_en, 0);
// }

// void Adxl355_SPI::p_scl_mux_disable()
// {
	// digitalWrite(_scl_en, 1);
	// delayMicroseconds(20);
// }
