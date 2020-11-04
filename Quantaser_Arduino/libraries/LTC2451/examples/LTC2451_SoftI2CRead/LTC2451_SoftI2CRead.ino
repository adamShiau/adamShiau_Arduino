//#include <I2cMaster.h>
//#include <Wire.h>
#include <LTC2451.h>
#define SDA_PIN 4
#define SCL_PIN 3
LTC2451 adc;
void setup() {

  Serial.begin(9600);
  Serial.println(adc.SoftI2CInit(SDA_PIN, SCL_PIN, MODE30HZ));
 }
void loop() {
  unsigned int x; 
  x =adc.SoftI2CRead();
  Serial.println(x);
}


