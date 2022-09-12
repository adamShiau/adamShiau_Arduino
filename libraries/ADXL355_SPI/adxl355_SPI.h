#ifndef ADXL355_SPI_H
#define ADXL355_SPI_H

#include <Arduino.h>
#include <SPI.h>

class Adxl355_SPI
{
	public:
		Adxl355_SPI(SPIClass &, char);
		~Adxl355_SPI();
		void init(void);
		void setRegVal(unsigned char, unsigned char);
		void printRegVal(char [], unsigned char, unsigned char);
		void printRegAll(void);
		void readData(unsigned char []);
		void readFakeData(unsigned char []);
		
	private:
		char _ss;
		int _ext_sync;
		bool _sercom_mode;
		void p_SPIWriteData(unsigned char, unsigned char);
		unsigned char p_SPIReadData(unsigned char);
		// void p_scl_mux_enable(void);
		// void p_scl_mux_disable(void);
		SPIClass &mySPI;
		
};


#endif