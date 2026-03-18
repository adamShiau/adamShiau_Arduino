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

unsigned char* uartRT::readData(uint8_t* expected_header, uint8_t header_size, uint16_t* try_cnt, uint8_t* expected_trailer, uint8_t trailer_size, uint8_t print)
{
	const uint8_t data_size_expected = buffer_size;
    
    if (port.available() == 0) return nullptr; //return immediately if no serial data in buffer 
    uint8_t data = port.read();
    #if defined(TEST_MODE)
        if(print==1){
            Serial.print("\ndata in: ");
            Serial.print(port.available());
            Serial.print(", ");
            Serial.println(data, HEX);    
            
        }
        
    #endif


	// static int bytes_received = 0;
	static enum {
		EXPECTING_HEADER, 
		EXPECTING_PAYLOAD,
        EXPECTING_TRAILER
	} state = EXPECTING_HEADER; // state machine definition
	
	switch (state) {
		case EXPECTING_HEADER:
            #if defined(TEST_MODE)
                if(print==1){
                    
                    Serial.println("state : EXPECTING_HEADER ");
                    Serial.print(bytes_received);
                    Serial.print(", ");
                    Serial.println( expected_header[bytes_received], HEX);
                    
                }
            #endif
			
			if (data != expected_header[bytes_received++])
			{
				state = EXPECTING_HEADER;
				bytes_received = 0;
                (*try_cnt)++;

			}

            #if defined(TEST_MODE)
                if(print==1){
                    Serial.print("bytes_received: ");
                    Serial.print(bytes_received);
                    Serial.print(", ");
                    Serial.println(data, HEX);
                 
                }
            #endif

			if(bytes_received >= header_size){
				state = EXPECTING_PAYLOAD;
				bytes_received = 0;
			}

			break;

		case EXPECTING_PAYLOAD:
			#if defined(TEST_MODE)
                if(print==1){
                    Serial.print("\nstate : EXPECTING_PAYLOAD");
                    Serial.println(bytes_received);
                 
                }
            #endif

			buffer[bytes_received++] = data;

            #if defined(TEST_MODE)
                if(print==1){
                    Serial.print("bytes_received: ");
                    Serial.print(bytes_received);
                    Serial.print(", ");
                    Serial.println(buffer[bytes_received-1], HEX);
                 
                }
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
                    if(print==1){
                        Serial.print("reset try_cnt: ");
                        Serial.println(*try_cnt);
                    }
                    #endif
                    return buffer;
                }
                else state = EXPECTING_TRAILER;
			}
			break;
        case EXPECTING_TRAILER:
            #if defined(TEST_MODE)
            if(print==1){
                Serial.println("\nstate : EXPECTING_TRAILER");
                Serial.print("Trailer: ");
                Serial.println(data, HEX);

            }
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

// /***
//  * 

unsigned char* uartRT::readData_2(uint8_t* expected_header, uint8_t header_size, uint16_t* try_cnt, uint8_t* expected_trailer, uint8_t trailer_size, uint8_t print)
{
	const uint8_t data_size_expected = buffer_size;
    
    if (port.available() == 0) return nullptr; //return immediately if no serial data in buffer 
    uint8_t data = port.read();
    #if defined(TEST_MODE)
        if(print==1){
            Serial.print("\ndata in: ");
            Serial.print(port.available());
            Serial.print(", ");
            Serial.println(data, HEX);    
            
        }
        
    #endif


	// static int bytes_received = 0;
	static enum {
		EXPECTING_HEADER, 
		EXPECTING_PAYLOAD,
        EXPECTING_TRAILER
	} state = EXPECTING_HEADER; // state machine definition
	
	switch (state) {
		case EXPECTING_HEADER:
            #if defined(TEST_MODE)
                if(print==1){
                    
                    Serial.println("state : EXPECTING_HEADER ");
                    Serial.print(bytes_received);
                    Serial.print(", ");
                    Serial.println( expected_header[bytes_received], HEX);
                    
                }
            #endif
			
			if (data != expected_header[bytes_received++])
			{
				state = EXPECTING_HEADER;
				bytes_received = 0;
                (*try_cnt)++;

			}

            #if defined(TEST_MODE)
                if(print==1){
                    Serial.print("bytes_received: ");
                    Serial.print(bytes_received);
                    Serial.print(", ");
                    Serial.println(data, HEX);
                 
                }
            #endif

			if(bytes_received >= header_size){
				state = EXPECTING_PAYLOAD;
				bytes_received = 0;
			}

			break;

		case EXPECTING_PAYLOAD:
			#if defined(TEST_MODE)
                if(print==1){
                    Serial.print("\nstate : EXPECTING_PAYLOAD");
                    Serial.println(bytes_received);
                 
                }
            #endif

			buffer[bytes_received++] = data;

            #if defined(TEST_MODE)
                if(print==1){
                    Serial.print("bytes_received: ");
                    Serial.print(bytes_received);
                    Serial.print(", ");
                    Serial.println(buffer[bytes_received-1], HEX);
                 
                }
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
                    if(print==1){
                        Serial.print("reset try_cnt: ");
                        Serial.println(*try_cnt);
                    }
                    #endif
                    return buffer;
                }
                else state = EXPECTING_TRAILER;
			}
			break;
        case EXPECTING_TRAILER:
            #if defined(TEST_MODE)
            if(print==1){
                Serial.println("\nstate : EXPECTING_TRAILER");
                Serial.print("Trailer: ");
                Serial.println(data, HEX);

            }
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

unsigned char* uartRT::readData_3(uint8_t* expected_header, uint8_t header_size, uint16_t* try_cnt, uint8_t* expected_trailer, uint8_t trailer_size, uint8_t print)
{
	const uint8_t data_size_expected = buffer_size;
    static enum {
		EXPECTING_HEADER, 
		EXPECTING_PAYLOAD,
        EXPECTING_TRAILER
	} state_3 = EXPECTING_HEADER; // state machine definition
    if (port.available() == 0) return nullptr; //return immediately if no serial data in buffer 
    uint8_t data = port.read();
    #if defined(TEST_MODE)
        if(print==1){
            Serial.print("\nava, data in: ");
            Serial.print(port.available());
            Serial.print(", ");
            Serial.println(data, HEX);    
        }
        
    #endif


	// static int bytes_received = 0;
	
	
	switch (state_3) {
        
		case EXPECTING_HEADER:
            #if defined(TEST_MODE)
                if(print==1){
                    Serial.println(" ---EXPECTING_HEADER--- ");
                    Serial.print(" b_recv, expec_header: ");
                    Serial.print(bytes_received_3);
                    Serial.print(", ");
                    Serial.println( expected_header[bytes_received_3], HEX);
                }
            #endif
			
			if (data != expected_header[bytes_received_3++])
			{
				state_3 = EXPECTING_HEADER;
				bytes_received_3 = 0;
                (*try_cnt)++;

			}

            #if defined(TEST_MODE)
                if(print==1){
                    Serial.print("b_recv, data: ");
                    Serial.print(bytes_received_3);
                    Serial.print(", ");
                    Serial.println(data, HEX);
                 
                }
            #endif

			if(bytes_received_3 >= header_size){
				state_3 = EXPECTING_PAYLOAD;
				bytes_received_3 = 0;
			}

			break;
         
        
		case EXPECTING_PAYLOAD:
			#if defined(TEST_MODE)
                if(print==1){
                    Serial.println("\n---EXPECTING_PAYLOAD---");
                    Serial.print(" b_recv");
                    Serial.println(bytes_received_3);
                 
                }
            #endif

			buffer[bytes_received_3++] = data;

            #if defined(TEST_MODE)
                if(print==1){
                    Serial.print("b_recv, buffer[b_recv-1] ");
                    Serial.print(bytes_received_3);
                    Serial.print(", ");
                    Serial.println(buffer[bytes_received_3-1], HEX);
                 
                }
            #endif

			if(bytes_received_3 >= data_size_expected)
            {
                bytes_received_3 = 0;

                if(expected_trailer == nullptr) {
                    state_3 = EXPECTING_HEADER;
                    *try_cnt = 0;
                    #if defined(TEST_MODE)
                    if(print==1){
                        Serial.print("reset try_cnt: ");
                        Serial.println(*try_cnt);
                    }
                    #endif
                    return buffer;
                }
                else state_3 = EXPECTING_TRAILER;
			}
			break;
        case EXPECTING_TRAILER:
            #if defined(TEST_MODE)
            if(print==1){
                Serial.println("\n---EXPECTING_TRAILER---");
                Serial.print("Trailer: ");
                Serial.println(data, HEX);

            }
            #endif

            if (data != expected_trailer[bytes_received_3++])
			{
				state_3 = EXPECTING_HEADER;
				bytes_received_3 = 0;
                (*try_cnt)++;
			}

            if(bytes_received_3 >= trailer_size){
				state_3 = EXPECTING_HEADER;
				bytes_received_3 = 0;
                *try_cnt = 0;
                return buffer;
			}
        break;
	}
	return nullptr;
}
// */

