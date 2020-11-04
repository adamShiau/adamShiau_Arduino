#include <Arduino.h>
#include <Wire.h>
//#include <SoftI2C.h>
#include <LTC2451.h>
#include <AD5541.h>
#include <SPI.h>
#include <LCD200_P08.h>
//#include <LCD200_P08_MS.h>
#include <DTC03_MS.h>



LCD200::LCD200()
{}
void LCD200::SetPinMode()
{
//    pinMode(ENC2_A, OUTPUT);
//  	pinMode(ENC2_SW, INPUT);
    pinMode(PWR_OFF, OUTPUT);
  	pinMode(LD_EN, OUTPUT);
  	pinMode(ENDAC, OUTPUT);
  	pinMode(VFC1, OUTPUT);
  	pinMode(VFC2, OUTPUT);
  	pinMode(VFC3, OUTPUT);
//  	pinMode(VLD, INPUT);
//  	pinMode(V_SENS, INPUT);
//  	pinMode(LCDSW, INPUT);
}

void LCD200::DACInit()
{
	ad5541.SetPin(ENDAC);
  	ad5541.init();
//  	ad5541.ModeWrite(0);
	ad5541.NormalWrite(65535);
}
void LCD200::PWROnOff(bool en) // set en LOW to turn OFF VCC
{
	
  if (en)
    digitalWrite(PWR_OFF, LOW);
  else
    digitalWrite(PWR_OFF, HIGH);
}
void LCD200::SetVCC(unsigned char vcc) 
{
  if(vcc == VCCLOW)
  {
    digitalWrite(VFC1, LOW);
    digitalWrite(VFC2, LOW);
    digitalWrite(VFC3, HIGH); 
  }
  if(vcc == VCCMEDIUM)
  {
    digitalWrite(VFC1, LOW);
    digitalWrite(VFC2, HIGH);
    digitalWrite(VFC3, LOW); 
  }
  if(vcc == VCCHIGH)
  {
    digitalWrite(VFC1, HIGH);
    digitalWrite(VFC2, LOW);
    digitalWrite(VFC3, LOW); 
  }
}
void LCD200::AnaBoardInit()
{
  PWROnOff(LOW);
  SetVCC(LOW);
  ltc2451.SoftI2CInit(SOFTSDAPIN, SOFTSCLPIN, 1);
  ad5541.NormalWrite(65535); // !!??Need to check why need this line??
  digitalWrite(LD_EN, LOW);


}
void LCD200::ResetFlag()
{
  g_LDOpenFlag =0;
  g_LDShortFlag =0;
  g_initfinished =0;
  g_AnyErrFlag =0;
  g_checkflag =1;
  g_dacoutslow = 65535;
  g_dacout = 65535;
  g_outerrorcounter =0;
}
void LCD200::readMonitor()
{
	g_vmon = ltc2451.SoftI2CRead();
//	Serial.println(g_vmon);
}
bool LCD200::OpenShortVfCheck()
{
  unsigned int vf, vth1, vth2;
 
  vth1 = (float)g_vfth1*20.46;
  vth2 = (float)g_vfth2*20.46;
//  digitalWrite(LD_EN, HIGH); //LD_EN LOW : bypass LD current
  delay(200);
  ad5541.NormalWrite(CHECKCURRENT);
  delay(500);
//  g_vmon = ltc2451.SoftI2CRead(); // !!??Check if read twice is necessary!!
  vf = analogRead(VLD);
//  if(g_vmon < OPENVTH) 
//  {
//    g_LDOpenFlag =1;
//    g_AnyErrFlag =1;
//    ad5541.NormalWrite(65535); 
//    Serial.println("a");
//
//  }
//  if(vf < VFSHORT)
//  {
//    g_LDShortFlag =1;
//    g_AnyErrFlag =1;
//    ad5541.NormalWrite(65535);
//    Serial.println("b");
//  }
  PWROnOff(HIGH);
  if(vf >vth2)
    SetVCC(VCCHIGH);
  else if(vf > vth1)
    SetVCC(VCCMEDIUM);
  g_checkflag = 0;
}

void LCD200::PWRCheck()
{
  unsigned int vplus;
  vplus = analogRead(V_SENS);
  if((g_dacoutslow == 65535) || (vplus < POWERGOOD)) digitalWrite(LD_EN, LOW); 
  
}

bool LCD200::IoutSlow()
{
  int deltaiout;
  unsigned int absdeltaiout;
  deltaiout = g_dacout- g_dacoutslow; // deltaiout > 0 =>LD current decrease
  absdeltaiout = abs(deltaiout);
  
  digitalWrite(LD_EN, HIGH);

  //Change the dacout slowly
  if(deltaiout > IOUTSTEP)  
    g_dacoutslow += IOUTSTEP;
  else if(absdeltaiout < IOUTSTEP)
//    g_dacoutslow += absdeltaiout;
    g_dacoutslow += deltaiout;
  else
    g_dacoutslow -= IOUTSTEP;

  ad5541.NormalWrite(g_dacoutslow);
//  ad5541.NormalWrite(65535);
//  Serial.print(deltaiout);
//  Serial.print(",");
//  Serial.print(g_dacout);
//  Serial.print(",");
//  Serial.println(g_dacoutslow);
  // Check if program is in slow state.......
  if(absdeltaiout == 0)
    return 0;
  else return 1;
}

void LCD200::CheckOutputErr() // Need to be use while IoutSlow = False
{
  unsigned int vf, ioutreal, ioutset;
  int deltaiout;
  vf = analogRead(VLD);
  ioutreal = ltc2451.SoftI2CRead(); 
  ioutset = 65535- g_dacout;
  deltaiout = ioutset - ioutreal;
  if(abs(deltaiout)> IOUTSTEP)
    g_outerrorcounter ++;
  
  if(g_outerrorcounter > IOUTCOUNTERMAX)
  {
    g_OutErrFlag =1;
    g_AnyErrFlag =1;
    Serial.println("c");
  } 
}
void LCD200::OnReceiveEvent()
{
  unsigned char com,temp[2];
  unsigned long t1, t2;
  unsigned int deltat;
  while(Wire.available()==3)
  {
    t1= micros();
    com = Wire.read();
    temp[0] = Wire.read();
    temp[1] = Wire.read();
    t2 = micros();
    deltat = t2-t1;
  }

  if(deltat < I2CREADTIMEMAX)
  {
    switch(com)
    {
      case LCD200_COM_LDEN:
        g_com_lden = temp[0];
//        Serial.println(F("EN:"));
//        Serial.println(g_com_lden);
      break;
      
      case LCD200_COM_IOUT:
      	Serial.println(g_AnyErrFlag);
        if(g_AnyErrFlag) {} // if there is no error status, update the g_dacout
        else
        {
        	g_dacout = temp[1] << 8 | temp[0];
//          	Serial.println(F("dac:"));
//          	Serial.println(g_dacout);
		}
          
      break;

      case LCD200_COM_VFTH1:
        g_vfth1 = temp[0];
//        Serial.println(F("VF1:"));
//        Serial.println(g_vfth1);
      break;

      case LCD200_COM_VFTH2:
        g_vfth2 = temp[0];
        g_initfinished = 1;
//        Serial.println(F("VF2, wakeup:"));
//        Serial.print(g_vfth2);
//        Serial.print(F(", "));
//        Serial.println(g_initfinished);
      break;
      
      case I2C_COM_TEST1:
//      	Serial.print(F("t1:"));
//    	Serial.println(temp[0]);
      break;
  

      
    }
  }
}
void LCD200::OnRequestEvent()
{
  unsigned char com,temp[2];

  while(Wire.available() == 1)
    com = Wire.read();
  switch(com)
  {
    case LCD200_COM_IIN:
      temp[1] = g_vmon >> 8;
      temp[0] = g_vmon;
    break;

    case LCD200_COM_ERR:
      if(g_LDShortFlag) temp[0] |= LCD200_ERRMASK_LDOPEN;
      else temp[0] &= (~LCD200_ERRMASK_LDOPEN);
      
      if(g_LDOpenFlag) temp[0] |= LCD200_ERRMASK_LDSHORT;
      else temp[0] &= (!LCD200_ERRMASK_LDSHORT);
      temp[1] = 0;

      if(g_OutErrFlag) temp[0] |= LCD200_ERRMASK_OUTERR;
      else temp[0] &= (~LCD200_ERRMASK_OUTERR);
    break;
    
    case I2C_COM_TEST1:
    	temp[0]=g_initfinished;
    break;
  }
  Wire.write(temp,2);
}

void LCD200::ad5541Test()
{
	ad5541.NormalWrite(65535);
	delay(1000);
	ad5541.NormalWrite(32768);
	delay(1000);
}








