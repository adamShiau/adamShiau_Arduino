#ifndef PIGV2_H
#define PIGV2_H

#include <Arduino.h>

class PIG
{
	public:
		PIG(void);
		void init(void);
		void sendCmd(unsigned char, unsigned int);
		void printVal(char [], int);
		void readData(unsigned char []);
		void readData_debug(unsigned char []);
		
	private:		

	
};

#endif