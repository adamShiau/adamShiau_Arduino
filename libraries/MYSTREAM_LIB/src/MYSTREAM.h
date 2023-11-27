#ifndef MYSTREAM_H
#define MYSTREAM_H

// #include <Arduino.h>
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

class myStream
{
    public:
        myStream(const uint8_t header[], uint8_t header_size);
        myStream(const uint8_t header[], uint8_t header_size, const uint8_t trailer[], uint8_t trailer_size);
        ~myStream(void);

        MYSTREAMStatusTypeDef ReadStream(uint8_t* buf, uint8_t buf_size, uint8_t data_ava, uint8_t data);

    private:
        uint8_t _bytes_received, _trailer_empty;
        uint8_t _header[4]; //assume max header lenth is 4 
        uint8_t _header_size;
        uint8_t _trailer[4]; //assume max _trailer lenth is 4 
        uint8_t _trailer_size;

        MYSTREAM_SM _state;
};

#endif