boolean toggle0 = 0;
boolean toggle1 = 0;



/********glogal variable***************/
int g_freq = 2000, g_phase = 0;


int freq_temp = g_freq;
int phase_temp = g_phase;

#include <Wire.h>
#include "QSS014_cmn.h"

#define PIN 7
#define PIN2 13
ISR(TIMER0_COMPA_vect){//timer0 interrupt 2kHz toggles pin 8
//generates pulse wave of frequency 2kHz/2 = 1kHz (takes two cycles for full wave- toggle high then toggle low)
  if (toggle0){
    PORTD = B10000000;
    toggle0 = 0;
  }
  else{
    PORTD = B00000000;
    toggle0 = 1;
  }
}

ISR(TIMER1_COMPA_vect){//timer0 interrupt 2kHz toggles pin 8
//generates pulse wave of frequency 2kHz/2 = 1kHz (takes two cycles for full wave- toggle high then toggle low)
  if (toggle1){
    PORTB = B00100000;
    toggle1 = 0;
  }
  else{
    PORTB = B00000000;
    toggle1 = 1;
  }
}

void setTimeIntr(void)
{
  cli();//stop interrupts
//  GTCCR = (1<<TSM)|(1<<PSRASY)|(0<<PSRSYNC); 
//  Serial.println(TSM);
//  Serial.println(PSRASY);
//  Serial.println(PSRSYNC);
  TCCR0A = 0;// set entire TCCR2A register to 0
  TCCR0B = 0;// same for TCCR2B
  TCNT0  = 0;//initialize counter value to 0
  OCR0A = 16000000/(g_freq*64L) - 1; //clock_freq / (2000*64) - 1 (must be <256)
  TCCR0A |= (1 << WGM01);
  TCCR0B |= (1 << CS01) | (1 << CS00);   
  TIMSK0 |= (1 << OCIE0A); 

  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = g_phase;//initialize counter value to 0
  OCR1A = 16000000/(g_freq*64L) - 1; //clock_freq / (2000*64) - 1 (must be <256)
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11) | (1 << CS10);  
  TIMSK1 |= (1 << OCIE1A);


//  TCCR2A = 0;// set entire TCCR2A register to 0
//  TCCR2B = 0;// same for TCCR2B
//  TCNT2  = g_phase;//initialize counter value to 0; change this value to change phase, -124 = 180, -62 = 90
//  OCR2A = 16000000/(g_freq*64L) - 1; //clock_freq / (2000*64) - 1 (must be <256)
//  TCCR2A |= (1 << WGM21);
//  TCCR2B |= (1 << CS22);   
//  TIMSK2 |= (1 << OCIE2A);
  GTCCR = 0; // release all timers
  delay(1000);
  sei();//allow interrupts
  
}

void setup(){
  Serial.begin(9600);
  Wire.begin(SLAVE_MCU_I2C_ADDR);
  Wire.onReceive(ReceiveEvent);
  //set pins as outputs
  pinMode(PIN, OUTPUT);
  pinMode(PIN2, OUTPUT);
  setTimeIntr();
  
}//end setup

void loop() {
  if(g_freq != freq_temp)
  {
    freq_temp = g_freq;
    setTimeIntr();
    Serial.println("freq change");
  }
  if(g_phase != phase_temp)
  {
    phase_temp = g_phase;
    setTimeIntr();
    Serial.println("phase change");
  }
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
