#include "MYSTREAM.h"

myStream::myStream(const uint8_t header[], uint8_t header_size)
{
    // initialize the state machine to EXPECTING_HEADER
    _state = EXPECTING_HEADER;
    _bytes_received = 0;
    _header_size = header_size;

    for(int i=0; i<_header_size; i++) {
        _header[i] = header[i];
    }

    _trailer_empty = 1;
    _trailer_size = 0;
}

myStream::myStream(const uint8_t header[], uint8_t header_size, const uint8_t trailer[], uint8_t trailer_size)
{
    // initialize the state machine to EXPECTING_HEADER
    _state = EXPECTING_HEADER;
    _bytes_received = 0;
    _header_size = header_size;

    for(int i=0; i<_header_size; i++) {
        _header[i] = header[i];
    }

    _trailer_empty = 0;
    _trailer_size = trailer_size;
    for(int i=0; i<_trailer_size; i++) {
        _trailer[i] = trailer[i];
    }
    
}

myStream::~myStream() {}

MYSTREAMStatusTypeDef myStream::ReadStream(uint8_t* buf, uint8_t buf_size, uint8_t data_ava, uint8_t data)
{
    

    // if(!data_ava) return MYSTREAM_ERROR;
    if(data_ava) Serial.print(data, HEX);
    else return MYSTREAM_ERROR;
    // Serial.println(_state);
    // Serial.print(data, HEX);
    // delayMicroseconds(100);
    switch (_state)
    {
    case EXPECTING_HEADER:
        if (data != _header[_bytes_received++])
        {
            
            _state = EXPECTING_HEADER;
            _bytes_received = 0;
        }

        if(_bytes_received >= _header_size)
        {
            _state = EXPECTING_PAYLOAD;
            _bytes_received = 0;
        }
        break;
        
    case EXPECTING_PAYLOAD:
        buf[_bytes_received++] = data;
        if(_bytes_received >= buf_size)
        {
            _bytes_received = 0;

            if(_trailer_empty) {
                _state = EXPECTING_HEADER;
                return MYSTREAM_OK;
            }
            else _state = EXPECTING_TRAILER;
        }
        break;

    case EXPECTING_TRAILER:
        if (data != _trailer[_bytes_received++])
        {
            _state = EXPECTING_HEADER;
            _bytes_received = 0;
        }

        if(_bytes_received >= _trailer_size)
        {
            _state = EXPECTING_HEADER;
            _bytes_received = 0;
            return MYSTREAM_OK;
        }
        break;
    
    default:
        break;
    }
    return MYSTREAM_ERROR;
}