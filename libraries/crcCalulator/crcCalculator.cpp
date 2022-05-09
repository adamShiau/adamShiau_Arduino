#include "crcCalculator.h"

#define WIDTH_8 8*sizeof(uint8_t)
#define TOPBIT_8 (1 << (WIDTH_8 - 1))
#define POLYNOMIAL_8 0x07

#define WIDTH_32 8*sizeof(uint32_t)
#define TOPBIT_32 (1 << (WIDTH_32 - 1))
#define POLYNOMIAL_32 0x04C11DB7

crcCal::crcCal()
{
	
}


uint8_t crcCal::crc_8(uint8_t  message[], uint8_t nBytes)
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
    return (remainder);
}

uint32_t crcCal::crc_32(uint8_t  message[], uint8_t nBytes)
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
	return (remainder);
}