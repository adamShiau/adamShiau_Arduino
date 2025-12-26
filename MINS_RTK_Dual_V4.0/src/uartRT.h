#ifndef UART_RT_H
#define UART_RT_H

#include <Arduino.h>
#include <stdint.h>

class uartRT
{
	public:
		// uartRT(Stream &);
		uartRT(Stream &, uint8_t);
		~uartRT(void);
		// unsigned char* readData(uint8_t* expected_header, uint8_t header_size=2, uint16_t* try_cnt = nullptr,
		// 						uint8_t* expected_trailer = nullptr, uint8_t trailer_sizes=1);

		unsigned char* readData(uint8_t* expected_header, uint8_t header_size=2, uint16_t* try_cnt = nullptr,
								uint8_t* expected_trailer = nullptr, uint8_t trailer_sizes=1, uint8_t print = 0);

		unsigned char* readData_2(uint8_t* expected_header, uint8_t header_size=2, uint16_t* try_cnt = nullptr,
								uint8_t* expected_trailer = nullptr, uint8_t trailer_sizes=1, uint8_t print = 0);

		unsigned char* readData_3(uint8_t* expected_header, uint8_t header_size=2, uint16_t* try_cnt = nullptr,
								uint8_t* expected_trailer = nullptr, uint8_t trailer_sizes=1, uint8_t print = 0);


	
	private:
		Stream &port;
		uint8_t *buffer;
		uint8_t buffer_size;
		int bytes_received = 0;
		int bytes_received_3 = 0;


};
	
#endif