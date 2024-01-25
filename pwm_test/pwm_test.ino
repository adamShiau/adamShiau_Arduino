#include "wiring_private.h"
// #include "myPWM.h"


#define PWM_PIN 2

void setup() {
// pwm_init();
pinMode(PWM_PIN, OUTPUT);
analogWrite(PWM_PIN, 127);
}


void loop() {
    delay(1);
}