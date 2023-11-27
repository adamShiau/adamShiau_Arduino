#include <MYSTREAM.h>
#include "wiring_private.h"

uint8_t HEADER[2] = {0xAB, 0xBA}, TRAILER[2] = {0x55, 0x56};
uint8_t size = 2;
uint8_t buf[5];

myStream streamObj(HEADER, size, TRAILER, size);

void setup()
{
    Serial.begin(230400);
    Serial1.begin(230400);
}


void loop()
{
    if(!streamObj.ReadStream(buf, 5, Serial1.available(), Serial1.read())) {
        for(int i=0; i<5; i++) {
            Serial.print(buf[i]);
            Serial.print(" ");
        }
        Serial.println("");
    }
}