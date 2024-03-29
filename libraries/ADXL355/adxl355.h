#ifndef ADXL355_H
#define ADXL355_H

#include <Arduino.h>
#include <Wire.h>

class Adxl355
{
	public:
		Adxl355(int);
		~Adxl355();
		void init(void);
		void setRegVal(unsigned char, unsigned char);
		void printRegVal(char [], unsigned char, unsigned char);
		void printRegAll(void);
		void readData(unsigned char []);
		void readFakeData(unsigned char []);
		
	private:
		int _scl_en;
		int _ext_sync;
		void p_I2CWriteData(unsigned char, unsigned char);
		unsigned char p_I2CReadData(unsigned char);
		void p_scl_mux_enable(void);
		void p_scl_mux_disable(void);
		

	
};


#endif