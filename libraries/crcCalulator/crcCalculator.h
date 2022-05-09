#ifndef CRC_CALCULATOR_H
#define CRC_CALCULATOR_H

#include <Arduino.h>
#include <stdint.h>

class crcCal
{
	public:
		crcCal();
		uint8_t crc_8(uint8_t [], uint8_t );
		uint32_t crc_32(uint8_t [], uint8_t );
	
	private:
	
};
	
#endif