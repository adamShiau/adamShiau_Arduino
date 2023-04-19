#ifndef UART_RT_H
#define UART_RT_H

#include <Arduino.h>
#include <stdint.h>

class uartRT
{
	public:
		uartRT(Stream &);
		~uartRT(void);
		unsigned char* readData(uint8_t* expected_header, uint8_t header_size=2, uint16_t* try_cnt = nullptr,
								uint8_t* expected_trailer = nullptr, uint8_t trailer_sizes=1);
	
	private:
		Stream &port;
		
};
	
#endif