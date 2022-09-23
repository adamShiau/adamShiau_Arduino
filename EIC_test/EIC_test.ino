volatile int cnt = 0;
#include "pig_v2.h"
//byte tt=0;
#define TRIG_OUT 3

PIG pig_ser1(Serial1);

void setup() {
  // put your setup code here, to run once:

  attachInterrupt(2, ISR_test, RISING);
  pinMode(TRIG_OUT, OUTPUT);
  digitalWrite(TRIG_OUT, LOW);
  Serial.begin(230400);
  Serial1.begin(9600);
}

void loop() {
byte header_ser1[2];
byte fog_ser1[14];
  
pig_ser1.readData(header_ser1, fog_ser1);
Serial1.write(fog_ser1, 14);

//digitalWrite(TRIG_OUT, LOW);
EIC->CONFIG[1].reg = 0x100;


}


void ISR_test()
{
  EIC->CONFIG[1].reg = 0;
  digitalWrite(TRIG_OUT, HIGH);
  loopdelay(10);
  digitalWrite(TRIG_OUT, LOW);
//  EIC->CONFIG[1].reg = 0x100;
}

void loopdelay(int dly)
{
//  flag = 0;
  
  for(int i=0; i<dly; i++) {
    Serial.println(i);
  }
//  EIC->CONFIG[1].reg = 0x100;
//  delay(10);
//  flag = 1;
}
