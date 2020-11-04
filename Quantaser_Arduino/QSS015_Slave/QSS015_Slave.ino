#include <Wire.h>
#include "QSS015_cmn.h"

//for integrator usage
#define S1 0
#define S2 1

#define DEBUG 0

unsigned long g_int_time = 100 * 1000;

void setup() {
  //for integrator usage
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  reset(30);
#if DEBUG
  Serial.begin(115200);
#endif
  Wire.begin(SLAVE_MCU_I2C_ADDR);
  Wire.onReceive(I2CReceive);
}

void loop() {

  while (1)
  {
    /////// integrate process /////
    reset(30);//不得<30, int hold end
    hold(10);
    integrate(g_int_time);
    hold(10);
  }

}

void reset(int wait) //10
{
  PORTD = ((PORTD & B11111100) | (1 << S1));
  //digitalWrite(S1, HIGH);
  //digitalWrite(S2, LOW);
  delayMicroseconds(wait);
}

void hold(int wait) //11
{
  PORTD = ((PORTD & B11111100) | (1 << S1) | (1 << S2));
  //digitalWrite(S1, HIGH);
  //digitalWrite(S2, HIGH);
  delayMicroseconds(wait);
}

void integrate(unsigned long wait) //01
{
  unsigned int bg;
  PORTD = ((PORTD & B11111100) | (1 << S2)); //int start
  //digitalWrite(S1, LOW);
  //digitalWrite(S2, HIGH);
  delayMicroseconds(wait);
}

void I2CReceive()
{
  unsigned char temp[4];
  //  unsigned char fbc_lower, fbc_upper, vmodoffset_upper, vmodoffset_lower;
  unsigned long t1, t2, t_delta;

  for (int i = 0; i < 4; i++)
  {
    temp[i] = 0;
  }

  while (Wire.available() == 4)
  {
    t1 = micros();
    temp[0] = Wire.read();
    temp[1] = Wire.read();
    temp[2] = Wire.read();
    temp[3] = Wire.read();
    t2 = micros();
    t_delta = t2 - t1; //
  }

  if (t_delta < 500)
  {
    Wire.write(temp, 4);
    g_int_time = temp[0] << 24 | temp[1] << 16 | temp[2] << 8 | temp[3];
#if DEBUG
    Serial.print(temp[0]);
    Serial.print(",");
    Serial.print(temp[1]);
    Serial.print(",");
    Serial.print(temp[2]);
    Serial.print(",");
    Serial.print(temp[3]);
    Serial.print("====");
    Serial.print(g_int_time);
    Serial.println("====");
#endif
  }
}
