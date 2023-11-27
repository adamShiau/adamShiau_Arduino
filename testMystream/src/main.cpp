#include <Arduino.h>
#include "MYSTREAM.h"

const uint8_t header[2] = {0xAB, 0xBA};
const uint8_t data[] = {0xAB, 0xBA, 1, 2, 3, 4, 5, 6, 7};

myStream stream1(header, 2);
uint8_t *buf;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
}

void loop() {
  for(int i=0; i<9; i++) {
    Serial.print(i);
    Serial.print(", ");
    Serial.println(!(stream1.ReadStream(buf, 7, true, data[i])));
    // if( !(stream1.ReadStream(buf, 7, true, data[i])) ) {
    //   Serial.println("PASS");
      // for(int i=0; i<7; i++) Serial.println(buf[i], HEX);
    // }
    // if(1) Serial.println("pass");
  }
  delay(100);
}

