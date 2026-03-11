#ifndef MYSTREAM_H
#define MYSTREAM_H

#include <Arduino.h>
#include <stdint.h>

/* Typedefs ------------------------------------------------------------------*/

typedef enum
{
  MYSTREAM_OK = 0,
  MYSTREAM_ERROR = -1
} MYSTREAMStatusTypeDef;

typedef enum {
    EXPECTING_HEADER, 
    EXPECTING_PAYLOAD,
    EXPECTING_TRAILER
} MYSTREAM_SM;

/*** Input Buffer Section*/

// The receive buffer size
#define	RX_BUF_SIZE			256				// Must be an even power of two
#define	RX_BUF_SIZE_MASK	RX_BUF_SIZE - 1

typedef struct
{
	uint8_t rxBuf[RX_BUF_SIZE];

	uint32_t rxBufCount;
	uint32_t rxBufPut;
	uint32_t rxBufTake;
} UartBuffer_t;


class myStream
{
    public:
        myStream(Stream &p, const uint8_t header[], uint8_t header_size);
        myStream(Stream &p, const uint8_t header[], uint8_t header_size, const uint8_t trailer[], uint8_t trailer_size);
        ~myStream(void);

        MYSTREAMStatusTypeDef ReadUartStream(uint8_t* buf, uint8_t buf_size);

        MYSTREAMStatusTypeDef PutToBuffer(uint8_t en, uint8_t data);
        MYSTREAMStatusTypeDef GetByteData(uint8_t* data);
        uint32_t DataAvailable(void);

    private:
        uint8_t _bytes_received, _trailer_empty;
        uint8_t _header[4]; //assume max header lenth is 4 
        uint8_t _header_size;
        uint8_t _trailer[4]; //assume max _trailer lenth is 4 
        uint8_t _trailer_size;

        MYSTREAM_SM _state; //data stream state machine 

        //uart class obj
        Stream &dev_serial; 

        //buffer structure
        UartBuffer_t _myBuf;
};

#endif