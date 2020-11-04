/*
========Revised version 1.1===========
1.don't shut-down current when output error 
2.keep current when output error

========Revised version 1.2===========
1. combine "set current value&limitation" to ldswtype part 
2. turn off Vcc when LD disable && DACset=65535
3. move read EEPROMto setup
*/
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include "LCD200_MCUV07.h"




LiquidCrystal_I2C lcd(IOEXPADDRESS, 16, 2);
volatile long DACset, currentDac;//
volatile char lastEncoded;
volatile byte cursorposition, curState, currentLim;
volatile float currentset, currentstep;
volatile int vPlus,Vd,dError;
volatile bool VfState ,checkState,flag;
volatile byte Vth1, Vth2;
unsigned long enc1Time1 = 0, enc1Time2, enc2Time1 = 0, enc2Time2;
int addrlim = ADDLIM, addrvth1 =ADDVTH1, addrvth2=ADDVTH2;


void setup() {
//  Serial.begin(9600);
  pinmodeSetting(); //setting pin-mode
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  
  AD5541Write(ENDAC, 65535);
  
  Wire.begin();

  digitalWrite(PWR_OFF, HIGH);
  vccSetting(LOW,LOW,HIGH);

  AD5541Write(ENDAC, 65535);

  lcd.init();
  lcd.print("Qantaser");
  lcd.setCursor(9, 0);
  lcd.print("LCD200");
  lcd.setCursor(0, 1);
  lcd.print("Ver:1.2");//Ver1.2
 

  LTC2451init();
  
  digitalWrite(LD_EN, LOW);
  digitalWrite(ENC2_A, HIGH);
  digitalWrite(ENC2_B, HIGH);
  attachInterrupt(0, runEncoder, CHANGE);

  currentLim = EEPROM.read(addrlim);
  Vth1 = EEPROM.read(addrvth1);
  Vth2 = EEPROM.read(addrvth2);
  
  currentset = 0.00;
  currentstep = 1.00;
  cursorposition = 10;
  curState = 0;
  DACset = 65535;
  VfState = 1;
  checkState = 1;
  dError=0;
  delay(2000);
}

void loop()
{
  bool ldswtype, stateChange,beepState, LDOpen, LDShort;
  unsigned int adcread;
  long temp;
  float currentact, Ilim, VfTH1, VfTH2 ,Vf;
  /* Vcc on/off selection */
  ldswtype = digitalRead(LDSW);
  if (ldswtype == HIGH)
    {
     if (checkState == 1)
      {
        beepState = 1;
        
        while (OpenShortCheck(&LDOpen,&LDShort))
         {
          if (beepState == 1)
          {
            lcd.clear();
            lcd.noCursor();
            if (LDOpen == 1)
              {lcd.print("LD OPEN!");}
            if (LDShort == 1)
              {lcd.print("LD SHORT!");}
            tone(PZT, BEEP, BEEPLONG);
            beepState =0;
          }  
         }
        checkState = 0;
      }
      digitalWrite(PWR_OFF, LOW);
      currentDac = currentset * RATIO;//
     }
  else
   {
    if(DACset == 65535) digitalWrite(PWR_OFF,HIGH);//
    checkState = 1;
    currentDac = 0;//
    dError=0;//
   }
 
  /*check V+ enough or not*/
  vPlus = analogRead(V_SENS);
  if (DACset == 65535 || vPlus < 550)
    digitalWrite(LD_EN, LOW);
  else
    digitalWrite(LD_EN, HIGH);
  
//  currentLim = EEPROM.read(addrlim);
//  Vth1 = EEPROM.read(addrvth1);
//  Vth2 = EEPROM.read(addrvth2);
  VfTH1 =((float)Vth1)/10.0;
  VfTH2 =((float)Vth2)/10.0;
//  limDac = currentLim * RATIO;

  /*Vf check @Iset>2mA*/
  if (DACset < 64880 && VfState)
  {
    VfCheck();
    VfState = !VfState;
  }

    temp = currentDac - (65535 - DACset);

  if (temp != 0)
  {
    if (temp > DACSTEP) DACset -= DACSTEP;
    else if (abs(temp) <= DACSTEP) DACset -= temp;
    else DACset += DACSTEP;
    AD5541Write(ENDAC, DACset);
    dError=0;
    delay(5);
  }

  adcread = LTC2451Read();
  currentact = float(adcread) / RATIO;
  Vf = float(analogRead(VLD))*5/1024.0;
  
  cursorBottom();
//  Serial.print("SW: ");
//  Serial.println(digitalRead(ENC2_SW));
  if (curState == 0 || curState == 2)
  {
    lcd.clear();
    lcd.noCursor();
    lcd.setCursor(0, 0);
    lcd.print("Iset:");
    if (currentset >= 100.00)
      lcd.setCursor(CUR100POSITION, 0);
    else if (currentset >= 10.00)
      lcd.setCursor(CUR10POSITION, 0);
    else
      lcd.setCursor(CUR1POSITION, 0);
    lcd.print(currentset, 2);
    lcd.setCursor(14, 0);
    lcd.print("mA");

    lcd.setCursor(0, 1);
    lcd.print("ILim:");
    
    if (currentLim >= 100)//
      lcd.setCursor(11, 1);
    else if (currentLim >= 10)//
      lcd.setCursor(12, 1);
    else
      lcd.setCursor(13, 1);
    lcd.print(currentLim);
    lcd.setCursor(14, 1);
    lcd.print("mA");
    delay(100);
  }


  if (curState == 1)
  {
    lcd.setCursor(cursorposition, 0);
    lcd.cursor();
    lcd.blink();
    delay(100);
  }

  if (curState == 2)
  {
    lcd.setCursor(13, 1);
    lcd.cursor();
    lcd.blink();
    delay(100);
  }
  
  if (curState == 3)
  {
    lcd.clear();
    lcd.noCursor();
    lcd.noBlink();
    lcd.print("*V2:");
    lcd.print(VfTH2);
    lcd.setCursor(12,0);
    lcd.print(currentact);
    lcd.setCursor(0,1);
    lcd.print(" V1:");
    lcd.print(VfTH1);
    lcd.setCursor(12,1);
    lcd.print(Vf);
    delay(100);
  }
 
  if (curState == 4)
  {
    lcd.clear();
    lcd.print(" V2:");
    lcd.print(VfTH2);
    lcd.setCursor(12,0);
    lcd.print(currentact);
    lcd.setCursor(0,1);
    lcd.print("*V1:");
    lcd.print(VfTH1);
    lcd.setCursor(12,1);
    lcd.print(Vf);
    delay(100);
  }

   /*Error detection*/
   if(temp ==0&&ldswtype==HIGH&&abs(currentset-currentact)>=4.0) //reach steady state but current error>4mA
   {
    dError++;
   }
   if(dError>50)
   {
    while(ldswtype == HIGH)
    {
      ldswtype=digitalRead(LDSW);
      lcd.clear();
      lcd.print("OUTPUT ERROR!!");
      delay(100);//Ver1.1
      dError=0;//Ver1.1
      currentset=currentact;//Ver1.1
      currentDac= currentset*RATIO;//Ver1.1
      AD5541Write(ENDAC,65535-currentDac);//Ver1.1
    }
   }  

 }






void vccSetting(bool ctr1, bool ctr2, bool ctr3)
 {
  digitalWrite(VFC1, ctr1);
  digitalWrite(VFC2, ctr2);
  digitalWrite(VFC3, ctr3);
 }

void pinmodeSetting()
 {
  pinMode(ENC2_A, OUTPUT);
  pinMode(ENC2_SW, INPUT);
  pinMode(ENC2_B, OUTPUT);
  pinMode(PWR_OFF, OUTPUT);
  pinMode(LD_EN, OUTPUT);
  pinMode(PZT, OUTPUT);
  pinMode(ENDAC, OUTPUT);
  pinMode(VFC1, OUTPUT);
  pinMode(LDSW, INPUT);
  pinMode(VFC2, OUTPUT);
  pinMode(VFC3, OUTPUT);
  pinMode(VLD, INPUT);
  pinMode(V_SENS, INPUT);
  pinMode(LCDSW, INPUT);
 }

void cursorBottom()
{
  // curState = 0 no blink for 1mA current setting one push to curState 1, 1s push for curState2, 10s push for curState3
  // curState = 1 blink for under 1mA current setting one push to curState 1, 1s push for curState2, 10s push for curState3
  // curState = 2 for current limit setting one push to curState 0
  // curState = 3 for Vth1 setting one push to curState 4
  // curState = 4 for Vth2 setting one push to curState 0

  unsigned long startTime, duration;
 if (digitalRead(ENC2_SW) == HIGH)
  flag = 1;
 if (digitalRead(ENC2_SW) == LOW)
  { 
   if (curState == 2 || curState == 4)
    {
     while (digitalRead(ENC2_SW) == LOW);
     curState = 0;
     flag = 0;
     currentstep = 1.00;
     cursorposition = CUR1POSITION;
     lcd.setCursor(CUR1POSITION, 0);
     lcd.blink();
     delay(500);
    }
   
   if (curState == 0 || curState == 1)
    {
     startTime = millis();
     duration = millis() - startTime;
     while (digitalRead(ENC2_SW) == LOW && duration <= 1000)
      duration = millis() - startTime;
     if (duration > 999)
      {
       curState = 2;
       lcd.setCursor(13, 1);
       lcd.cursor();
       lcd.blink();
       delay(500);
       while (digitalRead(ENC2_SW) == LOW && duration <= 6000)
        duration = millis() - startTime;
       if (duration > 5999)
        {
         curState = 3;
         flag = 0;
        }
      }
    } 
    if( (curState == 0 || curState == 1) && flag)
     {
      curState = 1;
      flag = 1;
      if (currentstep == 0.01)
      {
        cursorposition = CUR1POSITION;
        currentstep = 1.0;
      }
      else
      {
        cursorposition ++;
        currentstep = currentstep / 10.00;
        if (cursorposition == DOTPOSITION)
          cursorposition ++;
      }
     }
    if (curState == 3 && flag)
     {
      while (digitalRead(ENC2_SW) == LOW);
      curState = 4;
      flag = 1;
     }
  }
}

void runEncoder()
{
  if (curState == 0 || curState == 1)
    updateEncoder();
  else if (curState == 2)
    updateEncoder2();
  else if (curState == 3)
    updateEncoder3();
  else
    updateEncoder4();
}


void updateEncoder()
{
  char encoded1, sum1;
  bool MSB1, LSB1;
  enc1Time2 = millis();
  if (enc1Time2 - enc1Time1 < DEBOUNCETIME) return;
  MSB1 = digitalRead(ENC2_B); //MSB = most significant bit
  LSB1 = digitalRead(ENC2_A); //LSB = last significant bit

  encoded1 = (MSB1 << 1) | LSB1;
  sum1 = lastEncoded << 2 | encoded1;
  if (sum1 == 0b0110 || sum1 == 0b1001)
  {
    if (currentset < float (currentLim))
     currentset += currentstep;
  }
  if (sum1 == 0b0011 || sum1 == 0b1100)
  {
    currentset -= currentstep;
    if (currentset < 0.00)
      currentset = 0.00;
  }
  lastEncoded = encoded1;
  enc1Time1 = enc1Time2;
  curState = 0;
}


void updateEncoder2()
{
  char encoded2, sum2;
  bool MSB2, LSB2;
  enc2Time2 = millis();
  if (enc2Time2 - enc2Time1 < DEBOUNCETIME) return;
  MSB2 = digitalRead(ENC2_B); //MSB = most significant bit
  LSB2 = digitalRead(ENC2_A); //LSB = last significant bit
  encoded2 = (MSB2 << 1) | LSB2;
  sum2 = lastEncoded << 2 | encoded2;
  if (sum2 == 0b0110 || sum2 == 0b1001)
  {
    if (currentLim <= 199)//
      currentLim ++;
  }
  if (sum2 == 0b0011 || sum2 == 0b1100)
  {
    if (currentLim >= 1)//
      currentLim --;
  }
  lastEncoded = encoded2;
  enc2Time1 = enc2Time2;
  EEPROM.write(addrlim, currentLim);
}

void updateEncoder3()
{
  unsigned long enc3Time1,enc3Time2;
  char encoded3, sum3;
  bool MSB3, LSB3;
  enc3Time2 = millis();
  if(enc3Time2 -enc3Time1 < DEBOUNCETIME) return;
  MSB3 = digitalRead(ENC2_B);
  LSB3 = digitalRead(ENC2_A);
  encoded3 = (MSB3 <<1) | LSB3;
  sum3 = lastEncoded << 2| encoded3;
  if(sum3 == 0b0110 || sum3 == 0b1001)
  {
    if (Vth2 < 48)
      Vth2 = Vth2 +1;
  }
  if (sum3 ==0b0011 || sum3== 0b1100)
  {
    if(Vth2 > Vth1 && Vth2 > 11)
     Vth2 = Vth2 -1;
  }
  lastEncoded = encoded3;
  enc3Time1 = enc3Time2;
  EEPROM.write(addrvth2,Vth2);
}

void updateEncoder4()
{
  unsigned long enc4Time1,enc4Time2;
  char encoded4, sum4;
  bool MSB4, LSB4;
  enc4Time2 = millis();
  if(enc4Time2 -enc4Time1 < DEBOUNCETIME) return;
  MSB4 = digitalRead(ENC2_B);
  LSB4 = digitalRead(ENC2_A);
  encoded4 = (MSB4 <<1) | LSB4;
  sum4 = lastEncoded << 2| encoded4;
  if(sum4 == 0b0110 || sum4 == 0b1001)
  {
    if (Vth1 < Vth2)
      Vth1 = Vth1 +1;
  }
  if (sum4 ==0b0011 || sum4 ==0b1100)
  {
    if(Vth1 > 11)
    Vth1 = Vth1 -1;
  }
  lastEncoded = encoded4;
  enc4Time1 = enc4Time2;
  EEPROM.write(addrvth1,Vth1);
}


void PCF8541Write(uint8_t pin, uint8_t value8, uint8_t address, bool value)
{
  if (value == LOW)
   value8 &= ~(1 << pin);
  else
   value8 |= (1 << pin);

  Wire.beginTransmission(address);
  Wire.write(value8);
  Wire.endTransmission();
}

int PCF8541Read8(int address)
{
  int data;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 1);
  data = Wire.read();
  Wire.endTransmission();
  return data;
}

void VfCheck()
{
  unsigned int Vf;
  analogRead(VLD);
  delay(100);
  Vf =analogRead(VLD)*50/1024; //convert Vf to VfTH dimension

  if (Vf > Vth2)
  {
    digitalWrite(VFC3, LOW);
    digitalWrite(VFC2, LOW);
    digitalWrite(VFC1, HIGH);
  }
  else if (Vf > Vth1)
  {
    digitalWrite(VFC3, LOW);
    digitalWrite(VFC2, HIGH);
    digitalWrite(VFC1, LOW);
  }
}



bool OpenShortCheck(bool* LDOpen, bool* LDShort)
{
  unsigned int Vmonb, Vf;
  bool ErrorState;
  ErrorState =0;
  *LDOpen =0;
  *LDShort =0;
  digitalWrite(LD_EN, HIGH);
  delay(200);
  AD5541Write(ENDAC, checkcurrent);
  delay(500);
//  lcd.clear();
//  for(int i=0;i<6;i++)
//  {
//  Vmonb = LTC2451Read();
//  if(i>2)
//   lcd.setCursor(5*(i-3),1);
//  else
//   lcd.setCursor(i*5,0);
//  lcd.print(Vmonb);
//  delay(500);
//  }
  Vmonb = LTC2451Read();
  delay(100);
  Vmonb = LTC2451Read();
  Vf= analogRead(VLD);
  if (Vmonb < OpenVth && digitalRead(LDSW) == HIGH)
  {
    *LDOpen = 1;
    ErrorState =1;
  }
  if(Vf < VFSHORT &&  digitalRead(LDSW) == HIGH)
  {
    *LDShort =1;
    ErrorState =1;
  }
  AD5541Write(ENDAC, 65535);
  delay(200);
  digitalWrite(LD_EN, LOW);
  delay(200);
   
  return ErrorState;
}

void LTC2451init ()
{
  Wire.beginTransmission(ADCADDRESS);
  Wire.write(1);
  Wire.endTransmission();
}

unsigned int LTC2451Read()
{
  bool i = 0;
  unsigned int voltage[2];
  unsigned int vout;
  Wire.requestFrom(ADCADDRESS, 2);
  while (Wire.available())
  {
    voltage[i] = Wire.read();
    i++;
  }
  vout = (voltage [0] << 8) + voltage[1];
  return vout;
}

void AD5541Write (char adcselectpin, unsigned int dacvalue)
{
  byte highbyte, lowbyte;
  highbyte = byte ((dacvalue & 0xFF00) >> 8);
  lowbyte = byte (dacvalue & 0x00FF);
  digitalWrite(adcselectpin, LOW);
  SPI.transfer(highbyte);
  SPI.transfer(lowbyte);
  digitalWrite(adcselectpin, HIGH);
}
