#ifndef ADXL357_I2C_H
#define ADXL357_I2C_H

#include <Arduino.h>
#include <Wire.h>


#define ADXL357_ADDR 		0x1D	//Adxl357 I2C address
#define I2C_STANDARD_MODE 	100000
#define I2C_FAST_MODE 		400000
#define I2C_FAST_MODE_PLUS     1000000

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
#define RANGE_10G 	0x01
#define RANGE_20G 	0x02
#define RANGE_40G 	0x03


#define SENS_10G 0.0000195
#define SENS_20G 0.000039
#define SENS_40G 0.000078

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

class Adxl357_I2C
{
	public:
		Adxl357_I2C(TwoWire &);
		~Adxl357_I2C();
		void init(void);
		void setRegVal(unsigned char, unsigned char);
		void printRegVal(char [], unsigned char, unsigned char);
		void printRegAll(void);
		void readData(unsigned char []);
		void readFakeData(unsigned char []);
		void testI2C(void);
		
	private:
		int _scl_en;
		int _ext_sync;
		bool _sercom_mode;
		void p_I2CWriteData(unsigned char, unsigned char);
		unsigned char p_I2CReadData(unsigned char);
		void p_scl_mux_enable(void);
		void p_scl_mux_disable(void);
		TwoWire &myWire;
		
};


#endif