#include <ZeroRegs.h>

void setup() {
  Serial.begin(9600);
  while (! Serial) {}  // wait for serial monitor to attach
  ZeroRegOptions opts = { Serial, false };
  printZeroRegs(opts);
  Serial.println("HIHIHIHI");
  Serial.println(SYSCTRL->OSC32K.bit.ENABLE);
  Serial.println(SYSCTRL->OSC32K.bit.EN32K);
  Serial.println(SYSCTRL->OSC32K.bit.EN1K);
  Serial.println(SYSCTRL->OSC32K.bit.RUNSTDBY);
  Serial.println(SYSCTRL->OSC32K.bit.ONDEMAND);
  Serial.println(SYSCTRL->OSC32K.bit.STARTUP);
  Serial.println(SYSCTRL->OSC32K.bit.WRTLOCK);
  Serial.println(SYSCTRL->OSC32K.bit.CALIB);
  Serial.println(SYSCTRL->OSC32K.reg, HEX);
}

void loop() {
  // Do nothing
}
