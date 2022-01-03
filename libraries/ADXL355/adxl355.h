#ifndef ADXL355_H
#define ADXL355_H

#include <Arduino.h>
#include <Wire.h>


class adxl355
{
	public:
		adxl355(int scl_en);
		void init(void);
		void setRegVal(unsigned char, unsigned char);
		void printRegVal(char [], unsigned char, unsigned char);
		void printRegAll(void);
	
	private:
		int _scl_en;
		
		void p_I2CWriteData(unsigned char, unsigned char);
		unsigned char p_I2CReadData(unsigned char);
		void p_scl_mux_enable(void);
		void p_scl_mux_disable(void);
		

	
};


#endif