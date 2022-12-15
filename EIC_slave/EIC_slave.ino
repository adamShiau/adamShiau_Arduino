// PWM
#include <SAMD21turboPWM.h>
#define PWM100 7
#define PWM200 5
#define PWM250 11
TurboPWM  pwm;
//

byte cnt=0, cnt2=0;
volatile byte int_flag = 0;

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(2, ISR_test, RISING);
  Serial1.begin(230400);
  pwm.setClockDivider(2, false); //48MHz/4 = 12MHz
  pwm.timer(2, 2, 24000, false); //12M/2/24000 = 250Hz
  pwm.timer(1, 2, 60000, false); //12M/2/60000 = 100Hz
  pwm.timer(0, 2, 30000, false); //12M/2/30000 = 200Hz

  pwm.analogWrite(PWM100, 500);  
  pwm.analogWrite(PWM200, 500);  
  pwm.analogWrite(PWM250, 500);
}

void loop() {

  if(int_flag==1) {
    int_flag = 0;
    send_serial_data(Serial1, cnt);
    cnt++;
  }
}

void ISR_test()
{
  int_flag = 1;
}

void send_serial_data(Stream &ser, byte cnt)
{
  ser.write(0xAB);
  ser.write(0xBA);
  ser.write(0xA0);
  ser.write(0xA1);
  ser.write(0xA2);
  ser.write(0xA3);
  ser.write(0xA4);
  ser.write(0xA5);
  ser.write(0xA6);
  ser.write(0xA7);
  ser.write(0xA8);
  ser.write(0xA9);
  ser.write(0xB0);
  ser.write(0xB1);
  ser.write(0xB2);
  ser.write(cnt);
  
}
