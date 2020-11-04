/*20161031

*/

#include <DTC03Master_P02.h>
#include <DTC03_MS.h>
#include <Wire.h>
#include <openGLCD.h>
#include <fonts/SystemFont5x7.h>
#include <fonts/Iain5x7.h>
#include <fonts/fixed_bold10x15.h>

DTC03Master master;
void setup() {
  // put your setup code here, to run once:
  float tset, tact, itec;
  unsigned int vact_mv;
  int itec_mv;
  master.SetPinMode();
  master.ParamInit();
  master.WelcomeScreen();
  master.I2CReadAll();
  master.VarrayInit();
  vact_mv = (master.g_vactsum >> VAVGPWR);
  master.IarrayInit(); //set array=0
  itec = 0;
  tact = master.ReturnTemp(vact_mv, master.g_sensortype);
  master.g_tset = master.ReturnTemp(master.g_vset, master.g_sensortype);
  master.BackGroundPrint();
  master.PrintTset();
  master.PrintTact(tact);
  master.PrintItec(itec);
  master.PrintIlim();
  master.PrintP();
  master.PrintKi();
  master.PrintB();
//  master.PrintSensor();
  master.PrintModStatus();
  attachInterrupt(digitalPinToInterrupt(ENC_A),CheckEncoder,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B),CheckEncoder,CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(master.g_cursorstate<7) //do it when normal-mode
  {
    master.CheckStatus();
    //delay(400);//20161103
  }
  master.blinkTsetCursor();
  master.CursorState();
//  master.ShowCursor();
  master.UpdateParam();
  master.UpdateEnable();//


}
void CheckEncoder()
{
  master.Encoder();
}


