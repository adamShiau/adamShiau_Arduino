  /*Ver.3.00*/

#include <SPI.h>
#include <Wire.h>
#include <AD5541.h>
#include <LTC1865.h>
#include <DTC03_Slave_V300.h>
#include <PID.h>
#include <EEPROM.h>
#include <DTC03_MS.h>
#define PRINTLOOP 1
#define PIDOUTPUTLIMIT 65535

DTC03 dtc;
PID ipid, tpid;

unsigned int i=0;
unsigned long loop_time[5];
unsigned char ilim_kp=20, ilim_ki=10, ilim_ls=1;
void setup() {
  Serial.begin(9600);
  Wire.begin(DTC03P05);
  Wire.onReceive(ReceiveEvent);
  Wire.onRequest(RequestEvent);
  dtc.SetSPI();
  dtc.SetPinMode();
  dtc.ParamInit();
  dtc.DynamicVcc();
//  if(dtc.g_sensortype) digitalWrite(SENSOR_TYPE, HIGH);
//  else digitalWrite(SENSOR_TYPE, LOW);
  dtc.CheckSensorType();
  dtc.CheckTemp();
//  ipid.Init(32768,32768,0x7FFFFFFF,0);
//  tpid.Init(32768,32768,0x7FFFFFFF,0 );
  ipid.Init(32768,32768,dtc.g_ki,dtc.g_ls,0);
  tpid.Init(32768,32768,1,2,0 );
  dtc.dacforilim.ModeWrite(0);
  dtc.dacformos.ModeWrite(0);
}

void loop() {
  
  // put your main code here, to run repeatedly:
  int isense; //
  long ioutput,toutput,output;
  int ierr;
  long terr;
  
  unsigned int pidoffset = dtc.g_tpidoffset*1000;
  
//  if (i==5) {
//    i=0;
//    for (int j=0;j<5;j++) Serial.println(loop_time[j]);
//    Serial.println(); 
//  }
//  loop_time[i] = micros();
 
//  if(dtc.g_sensortype) digitalWrite(SENSOR_TYPE, HIGH);
//  else digitalWrite(SENSOR_TYPE,LOW);
  
  
  
  dtc.ReadVpcb();
  dtc.CheckSensorType();
  dtc.CheckTemp();
  
  dtc.CurrentLimit();
  dtc.ReadIsense();
  isense =abs((int)(dtc.g_itecread)-(int)(dtc.g_isense0));
  ierr = isense - dtc.g_iteclimitset; 
//
  dtc.ReadVoltage(1);
  terr = (long)dtc.g_vact - (long)dtc.g_vset_limitt;
//  if(tpid.g_index==0)
//  {
//    Serial.print("out ");
//    Serial.print(ierr);
//    Serial.print(", ");
////    Serial.println(terr);
////    Serial.print(", ");
//    Serial.print(ioutput);
//    Serial.print(", ");
//    Serial.println(toutput);
//  }
///////////////current limit section start////////////////////////////////////////////
//  if(ierr > -20) // 200mA
//  {
//    ioutput=ipid.Compute(dtc.g_en_state, ierr, ilim_kp, ilim_ki, ilim_ls); ;//old:kp=58,ki=1,ls=2, new:20,10,1  
//    while( (abs(ioutput)<(abs(toutput)+pidoffset) &&  dtc.g_en_state )) //run current limit &&  
//    {   
//      
//      dtc.CurrentLimit();// get dtc.g_iteclimitset
//      dtc.ReadIsense();
//      isense =abs((int)(dtc.g_itecread)-(int)(dtc.g_isense0));
//      ierr = isense - dtc.g_iteclimitset;
//      ioutput=ipid.Compute(dtc.g_en_state, ierr, ilim_kp, ilim_ki, ilim_ls); 
//      output = (long)(abs(ioutput)+dtc.g_fbc_base);
//     
//      if (output>PIDOUTPUTLIMIT) output= PIDOUTPUTLIMIT;
//
//      dtc.ReadVoltage(1);
//      terr = (long)dtc.g_vact - (long)dtc.g_vset_limitt;   
//      tpid.g_errorsum=0;
//      toutput=tpid.Compute(dtc.g_en_state, terr, dtc.g_p, 0, 0);    
//
//     /////////////////use to find ilim_kp, ilim_ki and ilim_ls///
////     if(ipid.g_index==0)
////      {
////         Serial.print(dtc.g_itecread);
////         Serial.print(", ");
////         Serial.print(dtc.g_isense0);
////         Serial.print(", ");
////         Serial.print(isense);
////         Serial.print(", ");
////         Serial.print(dtc.g_iteclimitset);
////         Serial.print(", ");
////         Serial.println(ierr);
////         Serial.println("");
//
////          Serial.print(ierr);
////          Serial.print(", ");
////          Serial.println(terr);
//
////            Serial.print(ioutput);
////            Serial.print(", ");
////            Serial.println(toutput);
////      }
///////////////////////////////////////////////////////////////  
//
//      if(toutput<=0) dtc.SetMos(HEATING,output);
//      else dtc.SetMos(COOLING,output);     
//    } 
//  }
//////////////current limit section end////////////////////////////////////////////////////////////////////////
  
//  if (dtc.g_overshoot == 1){
//    dtc.g_overshoot = 0;
//    tpid.g_errorsum = 0;
//  }

  toutput=tpid.Compute(dtc.g_en_state, terr, dtc.g_p, dtc.g_ki, dtc.g_ls); 
  output = (long)(abs(toutput)+dtc.g_fbc_base);
  if(output>PIDOUTPUTLIMIT) output=PIDOUTPUTLIMIT;
  if (toutput<=0) dtc.SetMos(HEATING,output);
  else if (toutput>0) dtc.SetMos(COOLING,output);  
  i++;
}


void ReceiveEvent(int howmany)
{
  dtc.I2CReceive();
}
void RequestEvent()
{ 
  dtc.I2CRequest();
}
