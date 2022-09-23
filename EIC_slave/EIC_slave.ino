#include <SAMD21turboPWM.h>

TurboPWM  pwm;
byte cnt=0;
volatile byte int_flag = 0;

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(2, ISR_test, RISING);
  Serial1.begin(9600);

/*** pwm ***/

  /*** Set input clock divider and Turbo Mode (which uses a 96MHz instead of a 48Mhz input clock): ***/
  pwm.setClockDivider(200, false); // Main clock 48MHz divided by 200 => 240KHz
  
  /*** Initialise timer x, with prescaler, with steps (resolution), 
  with fast aka single-slope PWM (or not -> double-slope PWM): 
  For the Arduino Nano 33 IoT, you need to initialise timer 1 for pins 4 and 7, timer 0 for pins 5, 6, 8, and 12, 
  and timer 2 for pins 11 and 13;
  ***/
  pwm.timer(2, 2, 240, false);   // Use timer 2 for pin 11, divide clock by 4, resolution 600, dual-slope PWM
  pwm.analogWrite(11, 500);        // PWM frequency = 120000/step/2, dutycycle is 500 / 1000 * 100% = 50%
                                  // current setup: 120000/240/2 = 250Hz
                                  // step = 600 for 100Hz
    /*--- for Sparrow demo ---*/

}

void loop() {

if(int_flag==1) {
  int_flag = 0;
  send_serial_data(Serial1, cnt);
  cnt++;
  delay(20);
}


}

void ISR_test()
{
  int_flag = 1;
  Serial.print("hi");
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
