/*Ver.3.00*/

#include <SPI.h>
#include <Wire.h>
#include <AD5541.h>
#include <LTC1865.h>
#include <DTC03_P05v2.h>
#include <PID.h>
#include <EEPROM.h>
#include <DTC03_MS.h>
#define PRINTLOOP 1
#define PIDOUTPUTLIMIT 65535

DTC03 dtc;
PID ipid, tpid;

unsigned int i=0;
unsigned long loop_time[5];
void setup() {
  Wire.begin(DTC03P05);
  Wire.onReceive(ReceiveEvent);
  Wire.onRequest(RequestEvent);
  dtc.SetSPI();
  dtc.SetPinMode();
  dtc.ParamInit();
  dtc.DynamicVcc();
  if(dtc.g_sensortype) digitalWrite(SENSOR_TYPE, HIGH);
  else digitalWrite(SENSOR_TYPE, LOW);
  dtc.CheckSensorType();
  dtc.CheckTemp();
//  dtc.CheckTemp();
//  ipid.Init(65535,32768,0x40000000);

  ipid.Init(32768,32768,0x7FFFFFFF);
  tpid.Init(32768,32768,0x7FFFFFFF);
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
 
  if(dtc.g_sensortype) digitalWrite(SENSOR_TYPE, HIGH);
  else digitalWrite(SENSOR_TYPE,LOW);
  
  dtc.ReadVoltage(1);
  dtc.ReadIsense();
  dtc.ReadVpcb();
  dtc.CheckSensorType();
  dtc.CheckTemp();

  isense =abs((int)(dtc.g_itecread)-(int)(dtc.g_isense0));
  ierr = isense - dtc.g_iteclimitset; 
  terr = (long)dtc.g_vact - (long)dtc.g_vset_limitt;
//  if (i%2000==0) {
//    Serial.print(dtc.ReturnTemp(dtc.g_vact,0));
//    Serial.print(", ");
//    Serial.println(dtc.ReturnTemp(dtc.g_vset_limitt,0));
//    Serial.println(dtc.g_overshoot);
//  }
  if(ierr > -20) 
  {
    ioutput=ipid.Compute(dtc.g_en_state, ierr, 58, 1, 2);//kp=58,ki=1,ls=2, 20161116
        
    while(abs(ioutput)<(abs(toutput)+pidoffset)) //run current limit
    {
     output = (long)(abs(ioutput)+dtc.g_fbc_base);
     if (output>PIDOUTPUTLIMIT) output= PIDOUTPUTLIMIT;
     if(toutput<=0) dtc.SetMos(HEATING,output);
     else dtc.SetMos(COOLING,output);

     ioutput=ipid.Compute(dtc.g_en_state, ierr, 58, 1, 2); 
     tpid.g_errorsum=0; // 1112@Adam
     toutput=tpid.Compute(dtc.g_en_state, terr, dtc.g_p, 0, 0); // 1112@Adam, only compare to Pterm     
//     dtc.CurrentLimit();// get dtc.g_iteclimitset
     
     isense =abs((int)(dtc.g_itecread)-(int)(dtc.g_isense0));
     ierr = isense - dtc.g_iteclimitset;
     dtc.ReadVoltage(1);
     terr = (long)dtc.g_vact - (long)dtc.g_vset_limitt;      
    } 
  }
  if (dtc.g_overshoot == 1){
    dtc.g_overshoot = 0;
    tpid.g_errorsum = 0;
  }
  
  toutput=tpid.Compute(dtc.g_en_state, terr, dtc.g_p, dtc.g_ki, dtc.g_ls); 
  output = (long)(abs(toutput)+dtc.g_fbc_base);
  if(output>PIDOUTPUTLIMIT) output=PIDOUTPUTLIMIT;//
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
