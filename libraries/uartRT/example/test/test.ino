#include "wiring_private.h"
#include "uartRT.h"

uartRT myUart(Serial1);
uint8_t trailer = 0x55;
uint8_t header[2] = {0xAB, 0xBA};
uint16_t try_cnt=0;

void setup() {
    Serial.begin(230400);
    Serial1.begin(230400);

}

void loop() {
    // myUart.readData();
  byte *data;

/****
  data = myUart.readData(&trailer);
  if(data) {
  Serial.println("\nreceived data: ");
    for(int i=0; i<sizeof(data); i++) {
      Serial.print(data[i], HEX);
      Serial.print(" ");
     }
     Serial.println("");
  }
***/
  data = myUart.readData(header, &trailer, &try_cnt);
  if(data) {
  Serial.println("\nreceived data: ");
    for(int i=0; i<sizeof(data); i++) {
      Serial.print(try_cnt);
      Serial.print(", ");
      Serial.print(data[i], HEX);
      Serial.print(" ");
     }
     Serial.println("");
  }



}