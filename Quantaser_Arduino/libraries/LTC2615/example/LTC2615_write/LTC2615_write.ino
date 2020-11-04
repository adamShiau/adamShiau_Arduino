#include <LTC2615.h>

LTC2615 ltc2615;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ltc2615.init();
  ltc2615.write(CH_E, 1.0);
  delay(2000);
  ltc2615.write(CH_F, 2.0);
  delay(3000);
  ltc2615.write(CH_G, 2.0);
}

void loop() {
  // put your main code here, to run repeatedly:
//  ltc2615.write(CH_E, 1.0);
//  delay(1000);
//  ltc2615.write(CH_E, 2.0);
//  delay(1000);
//  ltc2615.write(CH_E, 3.3);
//  delay(1000);
}
