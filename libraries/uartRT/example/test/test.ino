#include "wiring_private.h"
#include "uartRT.h"

uartRT myUart(Serial1);
uint8_t trailer = 0x55;

void setup() {
    Serial.begin(230400);
    Serial1.begin(230400);

}

void loop() {
    // myUart.readData();
  byte *data;

  data = myUart.readData(&trailer);
  // Serial.print(" data: ");
  // Serial.println((long) data);
  if(data) {
  // uint8_t ss =  sizeof(data);
    // Serial.println(sizeof(data));
  Serial.println("\nreceived data: ");
    for(int i=0; i<sizeof(data); i++) {
      Serial.print(data[i], HEX);
      Serial.print(" ");
     }
     Serial.println("");
  }

}