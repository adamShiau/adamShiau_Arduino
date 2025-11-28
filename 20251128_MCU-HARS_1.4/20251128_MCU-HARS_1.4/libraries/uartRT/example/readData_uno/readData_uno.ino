#include "uartRT.h"

uint8_t data_size = 4;
uartRT myUart(Serial, data_size);
uint8_t header[] = {0xCD, 0xDC};
uint8_t trailer[] = {0x55};
uint16_t try_cnt;

void setup() {
    Serial.begin(115200);
    byte *buffer2;
    buffer2 = (byte*) malloc(4*sizeof(byte));
}

void loop() {

  byte *data;
  const uint8_t sizeofheader = sizeof(header);
  const uint8_t sizeoftrailer = sizeof(trailer);
  
  /*** test default case, only Header no trailer***/
  // data = myUart.readData(header, sizeofheader, &try_cnt);

  /*** test both have Header and trailer case***/
  data = myUart.readData_2(header, sizeofheader, &try_cnt, trailer, sizeoftrailer);
  // if(try_cnt > 0) {
  //   Serial.print("try_cnt: ");
  //   Serial.println(try_cnt);
  // }
  
  
  if(data) {
  Serial.println("\nreceived data: ");
    for(int i=0; i<data_size; i++) {
      Serial.print(data[i], HEX);
      Serial.println(" ");
     }
     Serial.println("");
  }



}