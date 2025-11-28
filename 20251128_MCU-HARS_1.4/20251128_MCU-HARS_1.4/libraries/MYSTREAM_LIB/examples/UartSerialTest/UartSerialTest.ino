#include <MYSTREAM.h>
#include "wiring_private.h"

uint8_t HEADER[2] = {0xAB, 0xBA}, TRAILER[2] = {0x55, 0x56};
uint8_t size = 2;
uint8_t buf[5];

/** first streamObj for both has Header and Trailer case*/
// myStream streamObj(Serial1, HEADER, size, TRAILER, size);

/** second streamObj for only Header case*/
myStream streamObj(Serial1, HEADER, size);

void setup()
{
    Serial.begin(230400);
    Serial1.begin(230400);
}


void loop()
{
    if(!streamObj.ReadUartStream(buf, 5)) {
        for(int i=0; i<5; i++) {
            Serial.print(buf[i], HEX);
            Serial.print(" ");
            Serial1.write(buf[i]);
        }
        Serial.println("");
    }
}