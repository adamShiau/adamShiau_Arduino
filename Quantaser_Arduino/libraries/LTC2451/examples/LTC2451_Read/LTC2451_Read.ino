#include <LTC2451.h>
LTC2451 adc;
void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);
   adc.Init(MODE30HZ);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned int x; 
  x =adc.Read();
  Serial.println(x);

}
