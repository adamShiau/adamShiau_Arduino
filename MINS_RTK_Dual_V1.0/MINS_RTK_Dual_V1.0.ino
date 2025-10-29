#include "wiring_private.h"
#include "src/myUART.h"

#define MCU_LED 16

void setup() {
  myUART_init();
  pinMode(MCU_LED, OUTPUT);
  Blink_MCU_LED();
  Serial.println("MINS");
}

void loop() { 
  // if(Serial1.available()) {
  //   Serial.println(Serial1.available());
  // }
  // delay(100);
  
}


void Blink_MCU_LED()
{
  bool A=0;
  for(int i=0; i<10; i++){
    digitalWrite(MCU_LED, A);
    delay(100);
    A = !A;
  }
   delay(100);
}

