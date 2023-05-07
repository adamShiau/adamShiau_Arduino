#include "uartRT.h"

#define TEST_MODE

// uartRT::uartRT(Stream &p ) : port(p)
// {}

uartRT::uartRT(Stream &p, uint8_t size) : port(p)
{
    // buffer = new uint8_t[size];
    buffer = (uint8_t*) malloc(size*sizeof(uint8_t));
    buffer_size = size;
}

uartRT::~uartRT()
{
    // delete[] buffer;
    free(buffer);
}

/*** 
 * readData V0.9
 * designed by Adam, 2023/04/18
 * 
 ***/

unsigned char* uartRT::readData(uint8_t* expected_header, uint8_t header_size, uint16_t* try_cnt, uint8_t* expected_trailer, uint8_t trailer_size)
{
	const uint8_t data_size_expected = buffer_size;
    
    if (port.available() == 0) return nullptr; //return immediately if no serial data in buffer 
    uint8_t data = port.read();
    #if defined(TEST_MODE)
        Serial.print("\ndata : ");
        Serial.print(port.available());
        Serial.print(", ");
        Serial.println(data, HEX);
    #endif


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
                (*try_cnt)++;
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

                if(expected_trailer == nullptr) {
                    state = EXPECTING_HEADER;
                    *try_cnt = 0;
                    #if defined(TEST_MODE)
                        Serial.print("reset try_cnt: ");
                        Serial.println(*try_cnt);
                    #endif
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

            if (data != expected_trailer[bytes_received++])
			{
				state = EXPECTING_HEADER;
				bytes_received = 0;
                (*try_cnt)++;
			}

            if(bytes_received >= trailer_size){
				state = EXPECTING_HEADER;
				bytes_received = 0;
                *try_cnt = 0;
                return buffer;
			}
        break;
	}
	return nullptr;
}


unsigned char* uartRT::readData_2(uint8_t* expected_header, uint8_t header_size, uint16_t* try_cnt, uint8_t* expected_trailer, uint8_t trailer_size)
{
	const uint8_t data_size_expected = buffer_size;
    
    if (port.available() == 0) return nullptr; //return immediately if no serial data in buffer 
    uint8_t data = port.read();
    #if defined(TEST_MODE)
        Serial.print("\ndata in : ");
        Serial.print(port.available());
        Serial.print(", ");
        Serial.println(data, HEX);
    #endif
    // #if defined(TEST_MODE)
    //     Serial.println("\nreadData_2");
    // #endif

	static int bytes_received = 0;
	static enum {
        HEADER_FILTER,
		EXPECTING_HEADER_AB_BA, 
        EXPECTING_HEADER_CD_DC, 
		EXPECTING_PAYLOAD,
        EXPECTING_TRAILER
	} state = HEADER_FILTER; // state machine definition
	
	switch (state) {
        case HEADER_FILTER:
            #if defined(TEST_MODE)
                Serial.println("\nstate : HEADER_FILTER");
            #endif
            if(data == 0xAB) {
                state = EXPECTING_HEADER_AB_BA;
                bytes_received++;
            }
            if(data == 0xCD) {
                state = EXPECTING_HEADER_CD_DC;
                bytes_received++;
            }
            #if defined(TEST_MODE)
                Serial.print("bytes_received: ");
                Serial.println(bytes_received);
            #endif
            // bytes_received = 0;
        break;

		case EXPECTING_HEADER_AB_BA:
            #if defined(TEST_MODE)
                Serial.println("\nstate : EXPECTING_HEADER_AB_BA");
            #endif
			
			if (data != expected_header[bytes_received++])
			{
				state = HEADER_FILTER;
				bytes_received = 0;
                (*try_cnt)++;
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

        case EXPECTING_HEADER_CD_DC:
            #if defined(TEST_MODE)
                Serial.println("\nstate : EXPECTING_HEADER_CD_DC");
            #endif
			
			if (data != expected_header[bytes_received++])
			{
				state = HEADER_FILTER;
				bytes_received = 0;
                (*try_cnt)++;
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

                if(expected_trailer == nullptr) {
                    state = HEADER_FILTER;
                    *try_cnt = 0;
                    #if defined(TEST_MODE)
                        Serial.print("reset try_cnt: ");
                        Serial.println(*try_cnt);
                    #endif
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

            if (data != expected_trailer[bytes_received++])
			{
				state = HEADER_FILTER;
				bytes_received = 0;
                (*try_cnt)++;
			}

            if(bytes_received >= trailer_size){
				state = HEADER_FILTER;
				bytes_received = 0;
                *try_cnt = 0;
                return buffer;
			}
        break;
	}
	return nullptr;
}