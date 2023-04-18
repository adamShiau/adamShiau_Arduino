#ifndef UART_RT_H
#define UART_RT_H

#include <Arduino.h>
#include <stdint.h>

class uartRT
{
	public:
		uartRT(Stream &);
		~uartRT(void);
		unsigned char* readData(uint8_t* TRAILER = nullptr);
		unsigned char* readData_2(uint8_t expected_header[] = {0xAB, 0xBA}, uint8_t header_size = 2, uint8_t* expected_trailer = nullptr, uint16_t*);
		
	
	private:
		Stream &port;
		const unsigned char PIG_HEADER[2] = {0xAB, 0xBA};
		
};
	
#endif