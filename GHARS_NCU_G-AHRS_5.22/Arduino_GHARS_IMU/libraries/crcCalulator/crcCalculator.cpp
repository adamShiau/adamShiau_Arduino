#include "crcCalculator.h"

#define WIDTH_8 8
#define TOPBIT_8 (1 << (WIDTH_8 - 1))
#define POLYNOMIAL_8 0x07

#define WIDTH_32 32
#define TOPBIT_32 (1 << (WIDTH_32 - 1))
#define POLYNOMIAL_32 0x04C11DB7

crcCal::crcCal()
{
}

crcCal::~crcCal()
{
}


void crcCal::crc_8(uint8_t  message[], uint8_t nBytes, uint8_t *crc)
{
	uint8_t  remainder = 0x00;
	
    for (int byte = 0; byte < nBytes; ++byte)
    {
        remainder ^= (message[byte] << (WIDTH_8 - 8));

        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (remainder & TOPBIT_8)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL_8;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
    }
	*crc = remainder;
    // return (remainder);
}

void crcCal::crc_32(uint8_t  message[], uint8_t nBytes, uint8_t *crc)
{
	uint32_t  remainder = 0xFFFFFFFF;
	
	
	for (int byte = 0; byte < nBytes; ++byte)
	{
		remainder ^= (message[byte] << (WIDTH_32 - 8));
		
		
		for (uint8_t bit = 8; bit > 0; --bit)
		{
			if (remainder & TOPBIT_32)
			{
				remainder = (remainder << 1) ^ POLYNOMIAL_32;
			}
			else
			{
				remainder = (remainder << 1);
			}
		}
	}
	for (int i=0; i<sizeof(remainder); i++) 
	{
		*(crc + i) = remainder >> (24 - (i<<3));
		
	}
	
	// Serial.println(remainder, HEX);
	// Serial.print(", ");
	// return (remainder);
}