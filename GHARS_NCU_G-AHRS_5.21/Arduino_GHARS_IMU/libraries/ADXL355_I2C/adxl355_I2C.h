#ifndef ADXL355_I2C_H
#define ADXL355_I2C_H

#include <Arduino.h>
#include <Wire.h>

class Adxl355_I2C
{
	public:
		Adxl355_I2C(TwoWire &);
		~Adxl355_I2C();
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