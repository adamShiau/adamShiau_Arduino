#include <AD5541.h>
#include <SPI.h>

#define DACC 9
#define DACout 60000

AD5541 ad5541;

void setup() {
  // put your setup code here, to run once:
  pinMode(DACC,OUTPUT);
  ad5541.init();
  ad5541.SetPin(DACC);

}

void loop() {
  // put your main code here, to run repeatedly:
  ad5541.NormalWrite(0);
  delay(1000);
  ad5541.NormalWrite(32768);
  delay(1000);
  ad5541.NormalWrite(65535);
  delay(1000);

}
