#include <MYSTREAM.h>
#include "wiring_private.h"

uint8_t HEADER[2] = {0xAB, 0xBA};
uint8_t size = 2;

myStream streamObj(Serial1, HEADER, size);

void setup()
{
    Serial.begin(230400);
    Serial1.begin(230400);
}

void loop()
{
  uint8_t data;
  
    if(Serial1.available()){
        if(!streamObj.PutToBuffer(true, Serial1.read()));
    }

    if(streamObj.DataAvailable()==10){
      for(int i=0; i<10; i++){
        streamObj.GetByteData(&data);
        Serial.print(i);
        Serial.print(": ");
        Serial.print(data, HEX);
        Serial.print(", ");
        Serial.println(streamObj.DataAvailable());
      }
    }
}