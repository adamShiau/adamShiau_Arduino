#include "wiring_private.h"
#include "uartRT.h"

uartRT myUart(Serial1, 4);
uint8_t header[] = {0xAB, 0xBA};
uint8_t trailer[] = {0x55};
uint16_t try_cnt;

void setup() {
    Serial.begin(230400);
    Serial1.begin(230400);

}

void loop() {

  byte *data;
  const uint8_t sizeofheader = sizeof(header);
  const uint8_t sizeoftrailer = sizeof(trailer);
  
  /*** test default case, only Header no trailer***/
  // data = myUart.readData(header, sizeofheader, &try_cnt);

  /*** test both have Header and trailer case***/
  data = myUart.readData(header, sizeofheader, &try_cnt, trailer, sizeoftrailer);
  if(try_cnt > 0) {
    Serial.print("try_cnt: ");
    Serial.println(try_cnt);
  }
  
  
  if(data) {
  Serial.println("\nreceived data: ");
    for(int i=0; i<sizeof(data); i++) {
      Serial.print(data[i], HEX);
      Serial.println(" ");
     }
     Serial.println("");
  }



}