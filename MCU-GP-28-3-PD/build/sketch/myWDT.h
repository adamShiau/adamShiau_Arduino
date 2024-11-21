#line 1 "C:\\Users\\user\\Documents\\Arduino\\MCU-GP-28-3-PD\\myWDT.h"
#ifndef MYWDT_H
#define MYWDT_H

#include "IMU_PIG_DEFINE.h"
#include "EEPROM_MANAGE.h" 
#include "wiring_private.h"
#include "myUART.h"

// extern void read_fog_parameter_from_eeprom(int&, unsigned int);


//EXT WDT
#define WDI 5
#define EXT_WDT_EN 4


namespace MYWDT {
    extern bool wdi_status;
}

/*** * SW Watch dog  * **/
static void   WDTsync() {
  while (WDT->STATUS.bit.SYNCBUSY == 1); //Just wait till WDT is free
}

bool MYWDT::wdi_status = false;

//============= resetWDT ===================================================== 
void resetWDT() {
  // reset the WDT watchdog timer.
  // this must be called before the WDT resets the system
  WDT->CLEAR.reg= 0xA5; // reset the WDT
  WDTsync(); 
  // Serial.println("resetWDT");
}

//============= systemReset ================================================== 
void systemReset() {
  // use the WDT watchdog timer to force a system reset.
  // WDT MUST be running for this to work
  WDT->CLEAR.reg= 0x00; // system reset via WDT
  WDTsync(); 
}

//============= setupWDT =====================================================
void setupWDT( uint8_t period) {
  // initialize the WDT watchdog timer

  WDT->CTRL.reg = 0; // disable watchdog
  WDTsync(); // sync is required

  WDT->CONFIG.reg = min(period,11); // see Table 17-5 Timeout Period (valid values 0-11)

  WDT->CTRL.reg = WDT_CTRL_ENABLE; //enable watchdog
  WDTsync(); 
  Serial.println("setupWDT");
}

//============= disable WDT =====================================================
void disableWDT() {
  WDT->CTRL.reg = 0; // disable watchdog
  WDTsync(); // sync is required
  Serial.println("disableWDT");
}

/*** * EXT Watch dog  * **/
void reset_EXT_WDI(char wdi_pin){
  MYWDT::wdi_status = !MYWDT::wdi_status;
  digitalWrite(wdi_pin, MYWDT::wdi_status);
}

void disable_EXT_WDT(char en_pin){
  Serial.println("disable_EXT_WDT");
  digitalWrite(en_pin, HIGH);
}

void enable_EXT_WDT(char en_pin){
  Serial.print("EEPROM_EXTWDT:");
  Serial.println(EEPROM_EXTWDT);
  if(EEPROM_EXTWDT==SET_EXTWDT_ENABLE){
    msg_out("EXT_WDT is enable.");
    digitalWrite(en_pin, LOW);
  }
  else{
    msg_out("EXT_WDT is not enable, check EEPROM_EXTWDT setting.");
  }
  
}

void myWDT_init()
{
  pinMode(WDI, OUTPUT);
  pinMode(EXT_WDT_EN, OUTPUT);

  disableWDT();
  disable_EXT_WDT(EXT_WDT_EN);
}

#endif