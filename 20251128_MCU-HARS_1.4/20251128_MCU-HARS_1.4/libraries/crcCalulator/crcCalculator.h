#ifndef CRC_CALCULATOR_H
#define CRC_CALCULATOR_H

#include <Arduino.h>
#include <stdint.h>

class crcCal
{
	public:
		crcCal(void);
		~crcCal(void);
		void crc_8(uint8_t [], uint8_t,  uint8_t*);
		void crc_32(uint8_t [], uint8_t,  uint8_t*);
	
	private:
	
};
	
#endif