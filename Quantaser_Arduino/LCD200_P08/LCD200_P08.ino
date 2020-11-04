#include <Wire.h>
//#include <SoftI2C.h>
#include <LTC2451.h>
#include <SPI.h>
#include <AD5541.h>
#include <LCD200_P08.h>
#include <DTC03_MS.h>


LCD200 lcd200;
bool g_first_turn_on_flag;
void setup() {
  // put your setup code here, to run once:
//  Serial.begin(9600);
  Wire.begin(LCD200ADD);
  Wire.onReceive(ReceiveEvent);
  Wire.onRequest(RequestEvent);
  lcd200.SetPinMode();
  lcd200.DACInit();
  lcd200.AnaBoardInit();
  lcd200.ResetFlag();
  while(!lcd200.g_initfinished) delay(1);
}

void loop() 
{
  lcd200.PWRCheck();
  lcd200.readMonitor();
  if(lcd200.g_com_lden)
  {
    if(lcd200.g_checkflag) lcd200.OpenShortVfCheck();   
    lcd200.IoutSlow();
//    if(lcd200.IoutSlow()) {}
//    else lcd200.CheckOutputErr();
   }
  else  lcd200.ResetFlag();
}
void ReceiveEvent(int howmany)
{
  lcd200.OnReceiveEvent();
}
void RequestEvent()
{
  lcd200.OnRequestEvent();
}
