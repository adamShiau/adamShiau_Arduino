#ifndef PIGV2_H
#define PIGV2_H

#include <Arduino.h>
#include "uartRT.h"




class PIG
{
	public:
		PIG(Stream &, byte l=14);
		~PIG(void);
		void init(void);
		void sendCmd(unsigned char, unsigned int);
		void sendCmd(unsigned char*, unsigned char, unsigned char*, unsigned int);
		void updateParameter(unsigned char*, unsigned char, unsigned char*, unsigned int, unsigned char);
		void printVal(char [], int);
		void readData(unsigned char [], unsigned char []);
		unsigned char* readData(void);
		unsigned char* readData(uint8_t* expected_header, uint8_t header_size=2, uint16_t* try_cnt = nullptr,
								uint8_t* expected_trailer = nullptr, uint8_t trailer_sizes=1, uint8_t print = 0);
		unsigned char checkAck(unsigned char);
		void port_read(void);
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
		// uint16_t resend_cnt=0;
		const unsigned char KVH_HEADER[4] = {0xFE, 0x81, 0xFF, 0x55};
		const unsigned char PIG_HEADER[2] = {0xAB, 0xBA};
		Stream &port;
		uartRT myUart;
	
};

#endif