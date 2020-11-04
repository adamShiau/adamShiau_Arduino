#include <AD5541.h>
#include <SPI.h>

#define DACC 9
#define DACout 0

AD5541 ad5541;

void setup() {
  // put your setup code here, to run once:
ad5541.init();
ad5541.SetPin(DACC);
ad5541.ModeWrite(DACout);

}

void loop() {
  // put your main code here, to run repeatedly:

}
