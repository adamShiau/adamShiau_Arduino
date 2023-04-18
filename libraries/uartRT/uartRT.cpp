#include "uartRT.h"

#define TEST_MODE

uartRT::uartRT(Stream &p ) : port(p)
{}

uartRT::~uartRT()
{}


unsigned char* uartRT::readData(uint8_t* TRAILER)
{
	const uint8_t expected_header[2] = {PIG_HEADER[0], PIG_HEADER[1]};
	const uint8_t buffer_size = 4;
    const uint8_t header_size = 2;
	const uint8_t data_size_expected = buffer_size;

    
    if (port.available() == 0) return nullptr; //return immediately if no serial data in buffer 
    uint8_t data = port.read();


	static uint8_t buffer[buffer_size];
	static int bytes_received = 0;
	static enum {
		EXPECTING_HEADER, 
		EXPECTING_PAYLOAD,
        EXPECTING_TRAILER
	} state = EXPECTING_HEADER; // state machine definition
	
	switch (state) {
		case EXPECTING_HEADER:
            #if defined(TEST_MODE)
                Serial.println("\nstate : EXPECTING_HEADER");
            #endif
			
			if (data != expected_header[bytes_received++])
			{
				state = EXPECTING_HEADER;
				bytes_received = 0;
			}

            #if defined(TEST_MODE)
                Serial.print("bytes_received: ");
                Serial.print(bytes_received);
                Serial.print(", ");
                Serial.println(data, HEX);
            #endif

			if(bytes_received >= header_size){
				state = EXPECTING_PAYLOAD;
				bytes_received = 0;
			}

			break;

		case EXPECTING_PAYLOAD:
			#if defined(TEST_MODE)
                Serial.println("\nstate : EXPECTING_PAYLOAD");
            #endif

			buffer[bytes_received++] = data;

            #if defined(TEST_MODE)
                Serial.print("bytes_received: ");
                Serial.print(bytes_received);
                Serial.print(", ");
                Serial.println(buffer[bytes_received-1], HEX);
            #endif

			if(bytes_received >= data_size_expected)
            {
                bytes_received = 0;
                // state = EXPECTING_HEADER;
                // Serial.print("buf: ");
                // Serial.print((long)buffer, HEX);
                // Serial.print(", ");
                // Serial.print((long)&buffer[0], HEX);
                // Serial.print(", ");
                // Serial.println((long)&buffer[1], HEX);

                if(TRAILER == nullptr) {
                    state = EXPECTING_HEADER;
                    return buffer;
                }
                else state = EXPECTING_TRAILER;
			}
			break;
        case EXPECTING_TRAILER:
            #if defined(TEST_MODE)
                Serial.println("\nstate : EXPECTING_TRAILER");
                Serial.print("Trailer: ");
                Serial.println(data, HEX);
            #endif

            state = EXPECTING_HEADER;
            if(data == *TRAILER ) return buffer;
        break;
	}
	return nullptr;
    // printData(data);
}