#include "MYSTREAM.h"

myStream::myStream(Stream &p, const uint8_t header[], uint8_t header_size): dev_serial(p)
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

    _myBuf.rxBufCount = 0;
    _myBuf.rxBufPut = 0;
    _myBuf.rxBufTake = 0;
}

myStream::myStream(Stream &p, const uint8_t header[], uint8_t header_size, const uint8_t trailer[], uint8_t trailer_size): dev_serial(p)
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

    _myBuf.rxBufCount = 0;
    _myBuf.rxBufPut = 0;
    _myBuf.rxBufTake = 0;
    
}


myStream::~myStream() {}


MYSTREAMStatusTypeDef myStream::ReadUartStream(uint8_t* buf, uint8_t buf_size)
{
    uint8_t data;

    if(!dev_serial.available()) return MYSTREAM_ERROR;
    data = dev_serial.read();
    // Serial.println(dev_serial.available());
    // Serial.print(", ");
    // Serial.println(data, HEX);
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

MYSTREAMStatusTypeDef myStream::PutToBuffer(uint8_t en, uint8_t data)
{
    if(en) {
        // Serial.print(_myBuf.rxBufPut);
        // Serial.print(", ");
        // Serial.println(data, HEX);
        _myBuf.rxBuf[_myBuf.rxBufPut] = data; // put new data to buffer

        _myBuf.rxBufPut++; // increament the rxBufPut index
        _myBuf.rxBufPut &= RX_BUF_SIZE_MASK; //wrap rxBufPut between 0 and RX_BUF_SIZE

        if(_myBuf.rxBufCount<RX_BUF_SIZE) _myBuf.rxBufCount++; // Increment the rxBufCount while preventing buffer overrun

        return MYSTREAM_OK;
    }
    else return MYSTREAM_ERROR;
}

MYSTREAMStatusTypeDef myStream::GetByteData(uint8_t* data)
{
    if(DataAvailable()==0) return MYSTREAM_ERROR;
    else {
        *data = _myBuf.rxBuf[_myBuf.rxBufTake];

        _myBuf.rxBufTake++;
        _myBuf.rxBufTake &= RX_BUF_SIZE_MASK;

         _myBuf.rxBufCount--;

         return MYSTREAM_OK;
    }
}

uint32_t myStream::DataAvailable()
{
    return _myBuf.rxBufCount;
}