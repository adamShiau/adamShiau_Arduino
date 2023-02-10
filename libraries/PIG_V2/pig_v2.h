#ifndef PIGV2_H
#define PIGV2_H

#include <Arduino.h>

class PIG
{
	public:
		PIG(Stream &);
		~PIG(void);
		void init(void);
		void sendCmd(unsigned char, unsigned int);
		void printVal(char [], int);
		void readData(unsigned char [], unsigned char []);
		unsigned char* readData();
		void readDataCRC(unsigned char [], unsigned char []);
		void readFakeDataCRC(unsigned char [], unsigned char []);
		void readFakeData(unsigned char []);
		void readData_debug(unsigned char []);
		void resetFakeDataTime(void);
		char setSyncMode(unsigned int);
		void printData(unsigned char []);
		unsigned char* alignHeader_4B(unsigned char []);
		unsigned char* alignHeader_2B(unsigned char []);
		unsigned char* checkFakeHeader(unsigned char []);
		
	private:		
		unsigned int p_time_cnt;
		const unsigned char KVH_HEADER[4] = {0xFE, 0x81, 0xFF, 0x55};
		const unsigned char PIG_HEADER[2] = {0xAB, 0xBA};
		Stream &port;
	
};

#endif