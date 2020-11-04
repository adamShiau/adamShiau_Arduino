#include <Wire.h>
#include "QSS014_cmn.h"


#define PIN 8
#define PIN2 9

/********glogal variable***************/
int g_freq = 1000, g_phase = 180;
int g_period = int((1.0/g_freq*1e6)/2);
int g_phaseDelay = int(g_phase/180.0*g_period);

int freq_temp = g_freq;
int phase_temp = g_phase;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(SLAVE_MCU_I2C_ADDR);
  Wire.onReceive(ReceiveEvent);
  //set pins as outputs
  pinMode(PIN, OUTPUT);
  pinMode(PIN2, OUTPUT);
  PORTD = B00000000;
  PORTB = B00000000;
}

void loop() {
  // put your main code here, to run repeatedly:
  if(g_freq != freq_temp)
  {
    freq_temp = g_freq;
    g_period = int((1.0/g_freq*1e6)/2);

    phase_temp = g_phase;
    g_phaseDelay = int(g_phase/180.0*g_period);
    Serial.print("freq change: ");
    Serial.print(g_freq);
    Serial.print(", ");
    Serial.println(g_period);
  }
  if(g_phase != phase_temp)
  {
    phase_temp = g_phase;
    g_phaseDelay = int(g_phase/180.0*g_period);
    Serial.print("phase change");
    Serial.print(g_phase);
    Serial.print(", ");
    Serial.println(g_phaseDelay);
  }
  PORTB = B00000010;
  delayMicroseconds(g_phaseDelay);
  PORTB = B00000011;
  delayMicroseconds(g_period-g_phaseDelay);
  
  PORTB = B00000001;
  delayMicroseconds(g_phaseDelay);
  PORTB = B00000000;
  delayMicroseconds(g_period-g_phaseDelay);
}

void ReceiveEvent(int howmany)
{
  I2CReceive();
}

void I2CReceive()
{
  unsigned char temp[2], com;
//  unsigned char fbc_lower, fbc_upper, vmodoffset_upper, vmodoffset_lower;
  unsigned long t1,t2,t_delta;
  temp[0]=0;
  temp[1]=0;  
  while(Wire.available() == 3)
  {
    t1=micros();
    com=Wire.read();
    temp[0]=Wire.read();
    temp[1]=Wire.read();
    t2=micros();
    t_delta=t2-t1;//
  }
  
 if(t_delta<500) 
 { 
  switch(com)
  {
    case I2C_MOD_FREQ:
      g_freq = temp[0]<<8 | temp[1];
      Serial.print("g_freq: ");
      Serial.println(g_freq);
    break;  
    case I2C_MOD_PHASE:
      g_phase = temp[0]<<8 | temp[1];
      Serial.print("g_phase: ");
      Serial.println(g_phase);
    break;  
  }
 }
}
