/* modification for disable sensortype purpose : data 2016/11/13
	1. in ReadEEPROM(), set g_sensortype to 0;
	2. add g_mod_status variable in DTC03_P06.h
*/
/*
  1. Add ReadEEPROMnew()
  2. Add NOEE_DUMMY and EEADD_DUMMY define in header file
*/
#include <Wire.h>
#include <SPI.h>
#include <LTC1865.h>
#include <EEPROM.h>
#include <AD5541.h>
#include <PID.h>
#include <DTC03_P05v2.h>
#include <DTC03_MS.h>

DTC03::DTC03()
{}

void DTC03::ParamInit()
{
  dacformos.SetPin(DACC);
  dacforilim.SetPin(CURRENT_LIM);
  SetVcc(VCCHIGH);
  SetMosOff();
  dacforilim.ModeWrite(0);
  ltc1865.init(LTC1865CONV, CHVACT);  // CHVACT: first channel to read of ltc1865.Read
  g_en_state = 0;
  g_errcode1 = 0;
  g_errcode2 = 0;
  g_vactavgsum = 0;
  g_itecavgsum = 0;
  g_currentindex = 0;
  g_vactindex = 0;
  g_vpcbindex = 0;
  g_ilimdacout = 65535;

  g_tpidoffset = 2;
  g_wakeup = 0;
  g_overshoot = 0;
  ADCSRA &=~PS_128;
  ADCSRA |=PS_32;
}
void DTC03::SetPinMode()
{
  pinMode(NMOSC_IN,OUTPUT);
  pinMode(NMOSH_IN,OUTPUT);
  pinMode(DACC,OUTPUT);
  pinMode(CURRENT_LIM,OUTPUT);
  pinMode(FBSEL,OUTPUT);
  pinMode(SENSOR_TYPE,OUTPUT);
  pinMode(TEMP_SENSOR,INPUT);
  pinMode(VCC1,OUTPUT);
  pinMode(VCC2,OUTPUT);
  pinMode(VCC3,OUTPUT);
}
void DTC03::SetSPI()
{
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
}
void DTC03::SetVcc(unsigned char state)
{
  #ifdef DEBUGFLAG02 
  state = DEBUGFLAG02;
  #else
  #endif
	switch (state)
	{
		case VCCLOW:
			digitalWrite(VCC2,LOW);
 			digitalWrite(VCC1,LOW);
 		break;
 		case VCCMEDIUM:
 			digitalWrite(VCC2,LOW);
 			digitalWrite(VCC1,HIGH);
 		break;
 		case VCCHIGH:
 	 		digitalWrite(VCC2,HIGH);
 			digitalWrite(VCC1,LOW);
 		break;
 		default:
 		break;

	}
}
void DTC03::SetMos(bool status, unsigned int fb_value)
{
	PORTD = (PORTD| MOS_ON_OFF_STATUS_ADD) & ((status << NMOSC_IN)|(!status<< NMOSH_IN)|(!status << FBSEL));
	dacformos.ModeWrite(fb_value);
}
void DTC03::SetMosOff()
{
  PORTD = PORTD | MOS_ON_OFF_STATUS_ADD;
  dacformos.ModeWrite(0);
}

void DTC03::DynamicVcc()
{
    float restec, g_r1_f, g_r2_f;
    if(g_sensortype) digitalWrite(SENSOR_TYPE,g_sensortype);
    SetMosOff();
    BuildUpArray(1,1,1);
    while(g_wakeup == 0) delay(1);
    ReadIsense();
    g_isense0 = g_itecavgsum>>AVGPWR;
    g_r1_f = float(g_r1)*0.1;
    g_r2_f = float(g_r2)*0.1;
    
    #ifdef DEBUGFLAG01
      Serial.begin(9600);
      Serial.println("g_fbc_base:");
      Serial.println(g_fbc_base);
      Serial.println("R1, R2 for dynamic Vcc selection: ");
      Serial.print(g_r1_f);
      Serial.print(", ");
      Serial.println(g_r2_f);
      Serial.print("Isense0: ");
      Serial.println(g_isense0);//20161031
    #else
    #endif
    
	restec = CalculateR(RMEASUREVOUT,RMEASUREDELAY,RMEASUREAVGTIME,AVGTIME);
    if (restec < g_r1_f ) SetVcc(VCCLOW);
    else if(restec < g_r2_f ) SetVcc(VCCMEDIUM);
    else SetVcc(VCCHIGH);
	
}
float DTC03::CalculateR(unsigned int fb_value, unsigned int stabletime, int ravgtime, int vavgtime)
{
  
  int i,vtec0;
  float rsum=0, rtec, ravg, vtec, itec;

  vtec0 = ReadVtec(vavgtime);

  #ifdef DEBUGFLAG01
    Serial.print("Vtec0: ");
    Serial.println(vtec0);
    Serial.print("Vgs to R calculate =");
  Serial.println(fb_value);
    Serial.println("n   vtec  itec   rtec ");//20161031
  #else
  #endif

  SetMos(COOLING, fb_value);  // using cooling path and dac output = fb_value;
  BuildUpArray(0,1,0);    
  delay(stabletime);          // delay stabletime in ms
  for (i=0; i< ravgtime; i++)
  {
  	ReadIsense();
    vtec = float(ReadVtec(vavgtime)-vtec0);   
    itec = float( (g_itecavgsum>>AVGPWR)- g_isense0 );
    rtec = (vtec/itec)/RTECRatio;
    rsum += rtec;
    #ifdef DEBUGFLAG01
      Serial.print(i);
      Serial.print(", ");
      Serial.print(vtec);
      Serial.print(", ");
      Serial.print(itec);
      Serial.print(", ");
      Serial.println(rtec);
    #else
    #endif
  }
  ravg = rsum/float(ravgtime);
  #ifdef DEBUGFLAG01
    Serial.print("ravg:");
    Serial.println(ravg);
  #else
  #endif
  SetMosOff();
  BuildUpArray(0,1,0); 
  return ravg;
}
int DTC03::ReadVtec(int avgtime)
{
  int i, vtec;
  long vtecsum=0;

  for (i=0; i < avgtime; i++) vtecsum += analogRead(TEC_VOLT);

  vtec = vtecsum/avgtime;

  return vtec;
}

void DTC03::BuildUpArray(bool build_vact, bool build_itec, bool build_vpcb) {
	unsigned char i;
	
	if (build_vact == 1) g_vactavgsum = 0;
	if (build_itec == 1) g_itecavgsum = 0;
	if (build_vpcb == 1) g_vpcbavgsum = 0;
	
	for (i=0; i<AVGTIME; i++) {
		if (build_vact == 1) {
			Vactarray[i] = ltc1865.Read(CHVACT);
			g_vactavgsum += Vactarray[i];
		}		
		if (build_itec == 1) {
			Itecarray[i] = analogRead(ISENSE0);
			g_itecavgsum += Itecarray[i];
		}
		if (build_vpcb == 1) {
			Vpcbarray[i] = analogRead(TEMP_SENSOR);
			g_vpcbavgsum += Vpcbarray[i];
		}
	}	
}
void DTC03::ReadVoltage(bool en_vmod)
{
	noInterrupts();
    g_vactavgsum -= Vactarray[g_vactindex];
    if (en_vmod == 0) Vactarray[g_vactindex] = ltc1865.Read(CHVACT);
    else {
    	Vactarray[g_vactindex] = ltc1865.Read(CHVMOD);
    	g_vmod = ltc1865.Read(CHVACT);
	}
    g_vactavgsum += Vactarray[g_vactindex]; 
  
    g_vact = Vactarray[g_vactindex];
    g_vactindex ++;
    if(g_vactindex == AVGTIME) g_vactindex = 0;
    interrupts();  
}
void DTC03::ReadIsense()
{ 
  noInterrupts();
  g_itecavgsum -= Itecarray[g_currentindex];
  Itecarray[g_currentindex] = analogRead(ISENSE0);
  g_itecavgsum += Itecarray[g_currentindex]; 
  
  g_itecread = Itecarray[g_currentindex];
  g_currentindex ++;
  if(g_currentindex == AVGTIME) g_currentindex = 0;
  interrupts(); 
}
void DTC03::ReadVpcb() 
{
  noInterrupts();
  g_vpcbavgsum -= Vpcbarray[g_vpcbindex];
  Vpcbarray[g_vpcbindex] = analogRead(TEMP_SENSOR);
  g_vpcbavgsum += Vpcbarray[g_vpcbindex]; 
  
  g_vpcbindex ++;
  if(g_vpcbindex == AVGTIME) g_vpcbindex = 0;
  interrupts(); 
}

void DTC03::CheckSensorType()
{
  if(g_sensortype) digitalWrite(SENSOR_TYPE, HIGH); // High for AD590, Low for NTC
  else digitalWrite(SENSOR_TYPE, LOW);
  if((g_sensortype && g_vact < V_NOAD590)|(g_sensortype ==0 && g_vact==65535))
    {
      g_errcode1=1;
      g_en_state = 0;
    }
  else
    g_errcode1=0;
}
void DTC03::CheckTemp()
{
  g_Vtemp = (g_vpcbavgsum>>AVGPWR);
  if(g_Vtemp > g_otp) 
    {
      g_errcode2 = 1;
      g_en_state =0;
    }

//  else g_errcode2 = 0;
}
void DTC03::setVset() {
	long vmod, vset_limit_long;
	
	vmod = long(g_vmod) - long(g_vmodoffset);
	if (g_mod_status) vset_limit_long=(long)(g_vset_limit)+vmod;// in SDTC case, g_mod_status is alway 0
    else vset_limit_long=(long)(g_vset_limit);
  
    if(vset_limit_long>65535) vset_limit_long=65535;
    else if (vset_limit_long<0) vset_limit_long=0;
  
    g_vset_limitt = (unsigned int)vset_limit_long;
}

void DTC03::CurrentLimit()
{
  g_iteclimitset = 50+5*g_currentlim;
}

void DTC03::I2CRequest()
{
  unsigned char temp[2], com;
  unsigned int vact;
  int itec;
  bool itecsign=0;
  unsigned char i;//
  while(Wire.available()==1) com = Wire.read();

  switch(com)
  { 
    case I2C_COM_VACT:
    vact=g_vactavgsum >> AVGPWR;
    temp[0]=vact;
    temp[1]=vact >> 8;
    break;

    case I2C_COM_ITEC_ER:
    
    itec = (g_itecavgsum >> AVGPWR)-g_isense0;//g_isense0~612
    if(itec<0) itecsign = 1;
    else itecsign = 0;
    temp[0]=abs(itec);
    temp[1]=abs(itec) >> 8;
       
    if(g_errcode1)  temp[1] |= REQMSK_ERR1;
    else temp[1] &= (~REQMSK_ERR1);//20161101 add
//    if(g_errcode2)  temp[1] |= REQMSK_ERR2;
//    else temp[1] &= (~REQMSK_ERR2);//
    if(g_errcode2)  temp[1] &= REQMSK_ERR2;
    else temp[1] &= (~REQMSK_ERR2);//
    if(itecsign) temp[1]|= REQMSK_ITECSIGN;
    else temp[1] &= (~REQMSK_ITECSIGN);//
    break;

    case I2C_COM_PCB:
    temp[0] = g_Vtemp;
    temp[1] = g_Vtemp >> 8;
    break;
  }
  Wire.write(temp,2);
}
void DTC03::I2CReceive()
{
  unsigned char temp[2], com, errcodeall, bconst_upper, bconst_lower, vset_upper, vset_lower;
  unsigned char fbc_lower, fbc_upper, vmodoffset_upper, vmodoffset_lower;
  unsigned long t1,t2,t_delta;//added

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
    case I2C_COM_INIT:
    g_b_lower = temp[0];
    g_b_upper = REQMSK_BUPPER & temp[1];
    g_en_state = REQMSK_ENSTATE & temp[1];
//    g_sensortype = temp[1] & REQMSK_SENSTYPE; //20161113
	g_mod_status = temp[1] & REQMSK_SENSTYPE; 
//    Serial.println("INTI:");
//    Serial.print(g_en_state);
//    Serial.print(", ");
//    Serial.println(g_mod_status);
    break;

    case I2C_COM_CTR:
    g_currentlim = temp[0];
    g_p = temp[1];
    CurrentLimit();
    
//    Serial.println("CTR:");
//    Serial.print(g_currentlim);
//    Serial.print("p: ");
//    Serial.println(g_p);
    break;

    case I2C_COM_VSET:
    vset_lower =  temp[0];
    vset_upper = temp[1];
    g_vset_limit = vset_upper<<8 | vset_lower;
    setVset();
//    Serial.println("VSET:");
//    Serial.println( ReturnTemp(g_vset_limit,0) );
    break;

    
    case I2C_COM_KI:
    g_ls = temp[0];
    g_ki = temp[1];
    
//    Serial.print("LSKI:");
//    Serial.print(g_ls);
//    Serial.print(", ");
//    Serial.println(g_ki);
    break;

    case I2C_COM_R1R2:
    g_r1 = temp[0];
    g_r2 = temp[1];
//	Serial.println("R1R2:");
//    Serial.print(g_r1);
//    Serial.print(", ");
//    Serial.println(g_r2);
    break;

//    case I2C_COM_VBEC:
//    g_vbec1 = temp[0];
//    g_vbec2 = temp[1];
//    g_tpidoffset = g_vbec1;
//    //Serial.println("BC");
//    break;

    case I2C_COM_FBC:
    fbc_lower = temp[0];
    fbc_upper = temp[1];
    g_fbc_base =(fbc_upper <<8)|fbc_lower;//20161101
//    Serial.println("FBC:");
//    Serial.println(g_fbc_base);
    break;

//    case I2C_COM_VMOD:
//    vmodoffset_lower = temp[0];
//    vmodoffset_upper = temp[1];
//    g_vmodoffset = (vmodoffset_upper << 8)| vmodoffset_lower;
//    //Serial.println("MOD");
//    break;

    case I2C_COM_OTP:
    	g_otp = temp[1]<<8 | temp[0];
//    	Serial.println("OTP:");
//    	Serial.println(g_otp);
    break;
    
    case I2C_COM_WAKEUP:
    	g_wakeup = temp[0];
		g_overshoot = temp[1];
//		Serial.print("wu:");
//    	Serial.println(g_wakeup);
//    	Serial.print("os:");
//    	Serial.println(g_overshoot);
    break;
    
    
    case I2C_COM_TEST:  		
    	t_master = (temp[1] << 8) | temp[0];
//    	Serial.println(t_master);
//    	Serial.print(", ");
//    	Serial.println(temp[1],HEX);
    break;
    
    case I2C_COM_TEST1:

    Serial.print("p_temp:");
	Serial.println(temp[1]<<8 | temp[0]);
    break;
    
    case I2C_COM_TEST2:
    
    Serial.print("g_cursorstate:");
	Serial.println(temp[0]);
//	Serial.print("p_tBlink_toggle:");
//	Serial.println(temp[1]);
    break;
  }
 }

}
float DTC03::ReturnTemp(unsigned int vact, bool type)
{
  float tact;
  if(type)
    tact = (float)(vact/129.8701) - 273.15;
  else
    tact = 1/(log((float)vact/RTHRatio)/BVALUE+T0INV)-273.15;
  return tact;
}

