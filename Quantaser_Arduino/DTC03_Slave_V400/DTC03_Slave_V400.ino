  /*Ver.3.01*/

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
#define ISENSE_GAIN 7
#define MAPPING 0.586

DTC03 dtc;
PID ipid, tpid;

unsigned int i=0;
unsigned long loop_time[5], t_off;
void setup() {
  
  Wire.begin(DTC03P05);
  Wire.onReceive(ReceiveEvent);
  Wire.onRequest(RequestEvent);
  dtc.SetSPI();
  dtc.SetPinMode();
  dtc.ParamInit();
  dtc.DynamicVcc();
  dtc.CheckSensorType();
  dtc.CheckTemp();
  ipid.Init(32768,32768,dtc.g_ki,dtc.g_ls,ISENSE_GAIN);
  tpid.Init(50000,32768,1,2,0 );
//  ipid.Init(32768,32768,0x7FFFFFFF,ISENSE_GAIN);
//  tpid.Init(32768,32768,0x7FFFFFFF,0);
  dtc.dacforilim.ModeWrite(0);
  dtc.dacformos.ModeWrite(0);
  Serial.begin(9600);
  t_off = millis();
}

void loop() {
  
  // put your main code here, to run repeatedly:
  long isense, ierr, iset, iset2;
  long ioutput,toutput,output, terr, iteclimit;
//  if (i==5) {
//    i=0;
//    for (int j=0;j<5;j++) Serial.println(loop_time[j]);
//    Serial.println(); 
//  }
//  loop_time[i] = micros();
//  i++;

  dtc.ReadVoltage(1);
  dtc.ReadIsense();
  dtc.ReadVpcb();
  dtc.CheckSensorType();
  dtc.CheckTemp();
  dtc.CurrentLimit();
  iteclimit=(long)dtc.g_iteclimitset<<ISENSE_GAIN;
  
  terr = (long)dtc.g_vact - (long)dtc.g_vset_limitt;
//  Serial.print(dtc.g_vset_limitt);
//  Serial.print(", ");
//  Serial.println( dtc.ReturnTemp(dtc.g_vset_limitt,0),3); 
  toutput=tpid.Compute(dtc.g_en_state, terr, dtc.g_p, dtc.g_ki, dtc.g_ls); 
  
//  iset2=abs(map( toutput,-65535,65535,-300<<ISENSE_GAIN,300<<ISENSE_GAIN));
//  iset=abs( ((toutput*300)>>16)<<ISENSE_GAIN); // maping toutput to Iset
  iset=abs(toutput*MAPPING);
  if(iset > iteclimit) iset=iteclimit;
  
  isense =abs( ( (long)(dtc.g_itecread)-(long)(dtc.g_isense0) )<<ISENSE_GAIN );
  ierr = isense - iset;
//  ierr = isense - iteclimit;
  
  ioutput=ipid.Compute(dtc.g_en_state, ierr, 20, 10, 1);//old :kp=58,ki=1,ls=2, new : 20,10,1

  if(dtc.g_en_state) output = (long)(10000+dtc.g_fbc_base); //22445 ~= 28.5
  else output = (long) dtc.g_fbc_base;
  dtc.SetMos(HEATING,output);

  if (i%500==0) {
      Serial.print(millis()-t_off);
      Serial.print(", ");
      Serial.print(output);
      Serial.print(", ");
      Serial.println(dtc.g_vact);
  }
  if(i==2000) i=0;
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
