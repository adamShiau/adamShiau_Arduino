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
		
	
	private:
		Stream &port;
		const unsigned char PIG_HEADER[2] = {0xAB, 0xBA};
		
};
	
#endif