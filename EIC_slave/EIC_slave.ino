#include <SAMD21turboPWM.h>


byte cnt=0, cnt2=0;
volatile byte int_flag = 0;

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(6, ISR_test, RISING);
  Serial1.begin(9600);
g_APinDescription[6].ulExtInt = 4;
}

void loop() {

if(int_flag==1) {
  int_flag = 0;
  send_serial_data(Serial1, cnt);
  cnt++;
  delay(20);
}
Serial.println(g_APinDescription[6].ulExtInt);
//Serial.println(g_APinDescription[9].ulExtInt);
delay(20);
}

void ISR_test()
{
  int_flag = 1;
  Serial.println(cnt2++);
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
