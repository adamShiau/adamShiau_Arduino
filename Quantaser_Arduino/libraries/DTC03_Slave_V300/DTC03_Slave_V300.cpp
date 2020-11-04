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
#include <DTC03_Slave_V300.h>
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
  g_sensortype=0;
  g_errcode1 = 0;
  g_errcode2 = 0;
  g_vactavgsum = 0;
  g_autunAactavgsum = 0;
  g_itecavgsum = 0;
  g_currentindex = 0;
  g_vactindex = 0;
  g_vpcbindex = 0;
  g_atunDone = 0;
  g_ilimdacout = 65535;
  g_atune_flag = 0;// 改成master接收 
  g_runTimeflag = 0;
  g_DBRflag = 0;
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
//  pinMode(VCC3,OUTPUT);
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
#if 0 //use VCC3 only for P08, otherwise use VCC2
			digitalWrite(VCC3,LOW); 
#else
			digitalWrite(VCC2,LOW);
#endif
 			digitalWrite(VCC1,LOW);
 		break;

 		case VCCMEDIUM:
#if 0 //use VCC3 only for P08, otherwise use VCC2
 			digitalWrite(VCC3,LOW);
#else
 			digitalWrite(VCC2,LOW);
#endif
 			digitalWrite(VCC1,HIGH);
 		break;

 		case VCCHIGH:
#if 0 //use VCC3 only for P08, otherwise use VCC2
 			digitalWrite(VCC3,HIGH);
#else
 	 		digitalWrite(VCC2,HIGH);
#endif
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
    if(g_sensortype) digitalWrite(SENSOR_TYPE,LOW);
    SetMosOff();
    BuildUpArray(1,1,1);
    while(g_wakeup == 0) delay(1);
    ReadIsense();
    g_isense0 = g_itecavgsum>>AVGPWR;
    g_r1_f = float(g_r1)*0.1;
    g_r2_f = float(g_r2)*0.1;
    
    #ifdef DEBUGFLAG01
      Serial.begin(9600);
      Serial.print(F("g_fbc_base:"));
      Serial.println(g_fbc_base);
      Serial.print(F("VGS for Rtec:"));
      Serial.println(g_Rmeas);
      Serial.print(F("R1, R2 for dynamic Vcc selection: "));
      Serial.print(g_r1_f);
      Serial.print(F(", "));
      Serial.println(g_r2_f);
      Serial.print(F("Isense0: "));
      Serial.println(g_isense0);
    #else
    #endif
    
	restec = CalculateR(g_Rmeas,RMEASUREDELAY,RMEASUREAVGTIME,AVGTIME);
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
//  fb_value = 0;//test PID tuning
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
	
	if (build_vact == 1) g_vactavgsum = 0;
	if (build_itec == 1) g_itecavgsum = 0;
	if (build_vpcb == 1) g_vpcbavgsum = 0;
	
	for (int i=0; i<AVGTIME; i++) 
	{
		if (build_vact == 1) {
			Vactarray[i] = ltc1865.Read(CHVACT);
			g_vactavgsum += Vactarray[i];
			if(i<4) 
			{
//				Serial.print(i);
//				Serial.print(", ");
//				Serial.print(g_autunAactavgsum);
//				Serial.print(", ");
//				Serial.print( Vactarray[i]);
//				Serial.print(", ");
				AtuneActArray[i] =  Vactarray[i];
				g_autunAactavgsum += AtuneActArray[i];
//				Serial.println(g_autunAactavgsum);
			}		
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
    g_autunAactavgsum -= AtuneActArray[g_vactindex%ATUNEAVGTIME];  

    if (en_vmod == 0) Vactarray[g_vactindex] = ltc1865.Read(CHVACT);
    else 
	{
    	Vactarray[g_vactindex] = ltc1865.Read(CHVMOD);
    	AtuneActArray[g_vactindex%ATUNEAVGTIME] = Vactarray[g_vactindex];
    	g_vmod = ltc1865.Read(CHVACT);	
		  	
	}
    g_vactavgsum += Vactarray[g_vactindex]; 
    g_autunAactavgsum += AtuneActArray[g_vactindex%ATUNEAVGTIME];  
  	g_vact_MV = g_vactavgsum >> AVGPWR;
  	g_atuneVact_MV = g_autunAactavgsum >> ATUNEAVGPWR;
    g_vact = Vactarray[g_vactindex];
    g_vactindex ++;
    
    if(g_vactindex == AVGTIME) g_vactindex = 0;
    
    if(g_mod_status) //when g_mod_status = 1
	{
		setVset(); //when enable MOD function, read Vmod every loop
		p_enterSetVFlag = 1;
	}
    else //when g_mod_status = 0
	{
		g_vmod = g_vmodoffset; 
		if(p_enterSetVFlag) setVset();
	}
    interrupts();  
}
void DTC03::ReadIsense()
{ 
  noInterrupts();
  g_itecavgsum -= Itecarray[g_currentindex];
  Itecarray[g_currentindex] = analogRead(ISENSE0);
  g_itecavgsum += Itecarray[g_currentindex]; 
  // Serial.print(Itecarray[g_currentindex]);
  // Serial.print(", ");
  // Serial.println(analogRead(ISENSE0));
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
//  if(g_sensortype) digitalWrite(SENSOR_TYPE, HIGH); // High for AD590, Low for NTC
//  else digitalWrite(SENSOR_TYPE, LOW);
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
}
void DTC03::setVset() {
	long vmod, vset_limit_long;
	
	p_enterSetVFlag = 0;
	vmod = long(g_vmod) - long(g_vmodoffset);
    vset_limit_long=(long)(g_vset_limit)+vmod;// in SDTC case, g_mod_status is alway 0
  
    if(vset_limit_long>65535) vset_limit_long=65535;
    else if (vset_limit_long<0) vset_limit_long=0;
    g_vset_limitt = (unsigned int)vset_limit_long;
    
    //below used to test Vmos function:
    //1. disconnect Vmod BNC to check the correction of VmodOffset, 
    //2. input Vmod, check if the Tset is what you expect. note: Vmod_set=Vmod_in/2
//    Serial.print(g_mod_status);
//    Serial.print(", ");
//    Serial.print(g_vset_limitt); 
//    Serial.print(", ");
//	Serial.println( ReturnTemp(g_vset_limitt,0),3); 
}

void DTC03::CurrentLimit()
{
  g_iteclimitset = 45+5*g_currentlim;
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
//    vact=g_vactavgsum >> AVGPWR;
    temp[0]=g_vact_MV;
    temp[1]=g_vact_MV >> 8;
	#ifdef DEBUGFLAG01
	   Serial.print((float)g_vact_MV/65535.0*5);
	   Serial.print(", ");
	   Serial.println(ReturnTemp(g_vact_MV,0));
   #endif
    break;

    case I2C_COM_ITEC_ER:
    
    itec = (g_itecavgsum >> AVGPWR)-g_isense0;//g_isense0~612
    if(itec<0) itecsign = 1;
    else itecsign = 0;
    temp[0]=abs(itec);
    temp[1]=abs(itec) >> 8;
       
    if(g_errcode1) temp[1] |= REQMSK_ERR1;//B0001 0000, Err1:Sensor type
    else temp[1] &= (~REQMSK_ERR1);
    if(g_errcode2) temp[1] |= REQMSK_ERR2;//Err2: OTP
//    else temp[1] &= (~REQMSK_ERR2);//B0010 0000
    if(itecsign) temp[1]|= REQMSK_ITECSIGN;//B0000 0100
    else temp[1] &= (~REQMSK_ITECSIGN);

    if(g_wakeup) temp[1]|=REQMSK_WAKEUP;//B0100 0000
    else temp[1] &= (~REQMSK_WAKEUP);
//    Serial.println("2");
//	Serial.print("itec: ");
//	Serial.print(g_itecavgsum >> AVGPWR);
//	Serial.print(", ");
//	Serial.println(g_itecread);
//	Serial.print(", ");
//	Serial.print(g_errcode1);
//	Serial.print(", ");
//	Serial.println(g_errcode2);
    break;

    case I2C_COM_PCB:
    temp[0] = g_Vtemp;
    temp[1] = g_Vtemp >> 8;
//    Serial.println("3");
//    Serial.print("Vopt: ");
//    Serial.println(g_Vtemp);
    break;
    
    case I2C_COM_ATUN:
    	temp[0] = 0;
//    	temp[1] = 0;
    	if(g_runTimeflag) temp[0] |= REQMSK_ATUNE_RUNTIMEERR;
    	else temp[0] &= ~REQMSK_ATUNE_RUNTIMEERR;
    	if(g_DBRflag) temp[0] |= REQMSK_ATUNE_DBR;
    	else temp[0] &= ~REQMSK_ATUNE_DBR;
    	if(g_atunDone) temp[0] |= REQMSK_ATUNE_DONE;
    	else temp[0] &= ~REQMSK_ATUNE_DONE;
    	
//    	Serial.print("slave=");
//    	Serial.print(g_atunDone);
//    	Serial.print(g_runTimeflag);
//    	Serial.println(g_DBRflag);
    break;
    
    case I2C_COM_ATKpKi:
    	temp[0] = g_atune_kp;
    	temp[1] = g_atune_ki;
//    	Serial.print("g_atune_kp:");
//    	Serial.println(g_atune_kp);
//    	Serial.print("g_atune_ki:");
//    	Serial.println(g_atune_ki);
    	break;
  }
  Wire.write(temp,2);
}
void DTC03::I2CReceive()
{
  unsigned char temp[2], com, errcodeall, bconst_upper, bconst_lower, vset_upper, vset_lower;
  unsigned char fbc_lower, fbc_upper, vmodoffset_upper, vmodoffset_lower;
  unsigned long t1,t2,t_delta;//added
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
    case I2C_COM_INIT:
    g_b_lower = temp[0];
    g_b_upper = REQMSK_BUPPER & temp[1];
    g_en_state = REQMSK_ENSTATE & temp[1]; //B10000000
//    g_sensortype = temp[1] & REQMSK_SENSTYPE; //20161113
	g_mod_status = temp[1] & REQMSK_SENSTYPE; //B01000000
//    Serial.print("g_en_state:");
//    Serial.println(g_en_state);
//    Serial.print("g_mod_status: ");
//    Serial.println(g_mod_status);
    break;

    case I2C_COM_CTR:
    g_currentlim = temp[0];
    g_p = temp[1];
    CurrentLimit();
    
   // Serial.print("g_currentlim:");
   // Serial.println(g_currentlim);
   // Serial.print("kp: ");
   // Serial.println(g_p);
    break;

    case I2C_COM_VSET:
    vset_lower =  temp[0];
    vset_upper = temp[1];
    g_vset_limit = vset_upper<<8 | vset_lower;
    setVset();
//    Serial.print("VSET:");
//    Serial.print(", ");
//    Serial.print(vset_upper);
//    Serial.print(", ");
//    Serial.print(vset_lower);
//    Serial.print(", ");
//    Serial.println( ReturnTemp(g_vset_limit,0),3);
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

    case I2C_COM_TPIDOFF:
    g_tpidoffset = temp[0];
//     = temp[1];
//	Serial.print("TPIDOFF: ");
//    Serial.println(g_tpidoffset);
    break;

    case I2C_COM_FBC:
    fbc_lower = temp[0];
    fbc_upper = temp[1];
    g_fbc_base =(fbc_upper <<8)|fbc_lower;
//    Serial.print("FBC:");
//    Serial.println(g_fbc_base);
    break;

    case I2C_COM_VMOD:
    vmodoffset_lower = temp[0];
    vmodoffset_upper = temp[1];
    g_vmodoffset = (vmodoffset_upper << 8)| vmodoffset_lower;
//    Serial.print("MOD offset");
//    Serial.println(g_vmodoffset);
    break;
    
    case I2C_COM_ATUN:
    	g_atune_flag = temp[0] & REQMSK_ATUNE_STATUS; // temp[0] | 0x01;
    	g_atunDone = 0; 
    	g_DBRflag = 0;
    	g_runTimeflag = 0;
    	g_T_atune = temp[0] >> 1;//0~127 for 0~12.7 degree offset;
    	g_p_atune = temp[1];
//    	Serial.print("g_p_atune=");
//    	Serial.println(g_p_atune);
//    	Serial.print("g_T_atune=");
//    	Serial.println(g_T_atune);
//    	Serial.print("g_atune_flag=");
//    	Serial.println(g_atune_flag);
//    	Serial.print("temp[0]=");
//    	Serial.println(temp[0], BIN);
//    	Serial.print("REQMSK_ATUNE_STATUS=");
//    	Serial.println(REQMSK_ATUNE_STATUS);
    	break;
    	
    case I2C_COM_ATSTABLE:
    	g_stableCode_atune = temp[0];
//    	Serial.print("g_stableCode_atune:");
//    	Serial.println(g_stableCode_atune);
    	break;

    case I2C_COM_OTP:
    	g_otp = temp[1]<<8 | temp[0];
//    	Serial.print("OTP:");
//    	Serial.println(float(g_otp)/4.0-20.5,0);
    break;
    
    case I2C_COM_RMEAS:
    	g_Rmeas = temp[1]<<8 | temp[0];
//        Serial.print("g_Rmeas:");
//    	Serial.println(g_Rmeas);
    break;
    
    case I2C_COM_WAKEUP:
    	g_wakeup = temp[0];
		g_overshoot = temp[1];
//		Serial.print("wu:");
//    	Serial.println(g_wakeup);
//    	Serial.print("os:");
//    	Serial.println(g_overshoot);
    break;
    
   
    case I2C_COM_TEST1:  		
//    	t_master = (temp[1] << 8) | temp[0];
//    	Serial.println(t_master);
//    	Serial.print(", ");
//    	Serial.println(temp[1],HEX);
    break;
    case I2C_COM_TEST2:  		
//    	t_master = (temp[1] << 8) | temp[0];
//    	Serial.println(t_master);
//    	Serial.print("g_cursor:");
//    	Serial.println(temp[0]);
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
unsigned int DTC03::ReturnVset(float tset, bool type)
{
  unsigned int vset;
  float temp;
  if(type)
    vset = (unsigned int)((tset+273.15)*129.8701);
  else
    vset = (unsigned int)RTHRatio*exp(-1*(float)BVALUE*(T0INV-1/(tset+273.15)));
  return vset;
}
// new for autotune//

int DTC03::autotune(float &kp, float &ki)
{
	int peakcount, peaktype, peakstemp,A , period_count;
	bool justchanged,ismax,ismin, relay_heating_flag, relay_cooling_flag, find_period_flag, init_flag;
	uint8_t findBiasCurrentStatus, p_def=10, ki_def=1, p_dbr=30, ki_dbr=7;
	unsigned long peaktime[MAXPEAKS],now,t1,Pu, relay_period[4], runtime=0;
	float Ku, ki2, TC; 
	unsigned int step_out, in, lastinput[MAXLBACK], peaks[MAXPEAKS], v_bias_find, v_bias[FINDBIASARRAY], v_bias_relay;
	float t_leave;
	
	peakcount = -1;
  	peaktype = 0;
  	period_count = 0;
  	init_flag = 1;
  	relay_heating_flag = 1;
  	relay_cooling_flag = 1;
  	find_period_flag = 0;
  	findBiasCurrentStatus = 0;
  	A=0;
  	Pu=0; 
  	
  	int k=0;
  	unsigned long ts;
  	

	while(findBiasCurrentStatus!=2) 
	{
		v_bias_relay = FindBiasCurrent(t_leave, findBiasCurrentStatus, v_bias_find, v_bias, ts, runtime, k);
		if(g_DBRflag) 
		{
//			Serial.println("DBR case!");
			g_atune_flag = 0;
			g_atune_kp = p_dbr;
		    g_atune_ki = ki_dbr;
			g_atunDone = 1;
		}
		if(runtime>RUNTIMELIMIT) 
		{
//			Serial.println("Runtime time ERR!");
			g_atune_flag = 0;
			g_runTimeflag = 1;	
			g_atune_kp = p_def;
	        g_atune_ki = ki_def;
			g_atunDone = 1;		
			break;			
		}
	}	
//	Serial.println("leave");
	if(!g_atune_flag) return(0); // for runtime err case
	
	

    input_bias(p_noise_Mid,1);
//    Serial.print("v_bias_relay= ");
//	Serial.println(v_bias_relay);
//	Serial.print("p_noise_Mid= ");
//	Serial.println(p_noise_Mid);
    while(!find_period_flag) 
    {
    	RelayMethod(v_bias_relay, in, &init_flag, &relay_heating_flag, &relay_cooling_flag, find_period_flag, relay_period, period_count, step_out);
	}

//	Serial.print("Period = ");
//	Serial.println(p_relayT);
	AtunSamplingTime();
	if(g_DBRflag) 
	{
//		Serial.println("DBR case!");
		g_atune_flag = 0;
		g_atune_kp = p_dbr;
	    g_atune_ki = ki_dbr;
		g_atunDone = 1;
	}
	if(!g_atune_flag) return(0); // for DBR case
	
	for (int i=0;i<MAXLBACK;i++) 
   	{
	    ReadVoltage(1);     
	    lastinput[i]=g_vact;; // initialize the lastinput array 
   	} 
	while (peakcount < (MAXPEAKS-1))
	{
		now = millis();
		ismax=true;
    	ismin=true;
		RelayMethod(v_bias_relay, in, &init_flag, &relay_heating_flag, &relay_cooling_flag, find_period_flag, relay_period, period_count, step_out);
		lookbackloop(in, lastinput, &ismax, &ismin);
      	lastinput[0]=in;
      	peakrecord(in, &ismax, &ismin, &peaktype, &peakcount, peaks, &peakstemp, peaktime, now, &t1, &justchanged); 
      	if(peakcount >=2 && justchanged ) 
	    {
	        parameter(&peakcount, peaks, peaktime, &A, &Pu);
	        Ku = 4*OUTSTEP/(A*3.14159);      
	        kp = 0.4*Ku;
	        ki = 480.0*Ku/(float)Pu; 
	        ki2 = 1000.0*Ku/(float)Pu;
	        TC = 1.0/ki2/1.5;
	        g_atune_kp = atunKp(Ku);
	        g_atune_ki = atunKiLs(TC);
	        if(peakcount == MAXPEAKS-1)
	        {
	        	#ifdef DEBUGFLAG01
		          Serial.print("A=,");
		          Serial.println(A);
		          Serial.print("stepD=,");
		          Serial.println(OUTSTEP);
		          Serial.print("Ku=,");
		          Serial.println(Ku);
		          Serial.print("Pu=,");
		          Serial.println(Pu);
		          Serial.print("kp=,");
		          Serial.println(kp);
		          Serial.print("ki=,");
		          Serial.println(ki);
		          Serial.print("ki2=,");
		          Serial.println(ki2);
		          Serial.print("TC=,");
		          Serial.println((float)1.0/(ki),1); 
		          Serial.print("Tc2=,");
		          Serial.println(1.0/ki2,1);
		          Serial.print("g_atune_kp=,");
		          Serial.println(g_atune_kp);
		          Serial.print("g_atune_ki=,");
		          Serial.println(g_atune_ki);
	         	#else
    			#endif
	          g_atune_flag = 0;
	          g_atunDone = 1;
	        }
	    }       
      	delay(p_samplingTime);  
	}
}
void DTC03::AtunSamplingTime()
{
	if(60 <= p_relayT && p_relayT < 120) p_samplingTime = 600;
	else if(30 <= p_relayT && p_relayT < 60) p_samplingTime = 300;
	else if(15 <= p_relayT && p_relayT < 30) p_samplingTime = 150;
	else if(8 <= p_relayT && p_relayT < 15) p_samplingTime = 75;
	else if(4 <= p_relayT && p_relayT < 8) p_samplingTime = 38;
	else if(2 <= p_relayT && p_relayT < 4) p_samplingTime = 19;
//	else if(1 <= p_relayT && p_relayT < 2) p_samplingTime = 10;
//	else p_samplingTime = 5; //DBR type, show flag and use fix parameters 
    else g_DBRflag = 1;
//    if(!g_DBRflag)
//    {
//    	Serial.print("p_samplingTime=");
//	    Serial.println(p_samplingTime);
//	}
}
void DTC03::RelayMethod(unsigned int &v_bias_relay, unsigned int &in, bool *init_flag, bool *relay_heating_flag, bool *relay_cooling_flag, bool &find_period_flag, unsigned long *relay_period, int &period_count, unsigned int &step_out)
{	
	input_bias(in,0);
	
	
//	Serial.print(in);
//	Serial.print(", ");	
//	Serial.print((int)(in-p_noise_Mid));
//	Serial.print(", ");
	ReadIsense();
	if ( (int)(in-p_noise_Mid) >NOISEBAND) // cooling
    {
        step_out = v_bias_relay + OUTSTEP/2;  //Heater
        *init_flag = 0;
        if(*relay_cooling_flag && !find_period_flag) 
        {    
		  RelaySwitchTime(relay_period, period_count, find_period_flag);  
          *relay_heating_flag = 1;
          *relay_cooling_flag = 0;
        }     
      }
      else if ( (int)(in-p_noise_Mid) <-NOISEBAND) //heating
      {
        step_out = v_bias_relay - OUTSTEP/2;  
        *init_flag = 0;

        if(*relay_heating_flag && !find_period_flag) 
        {
          RelaySwitchTime(relay_period, period_count, find_period_flag);
          *relay_heating_flag = 0;
          *relay_cooling_flag = 1;
        }
      }
      else if (*init_flag)
      {
        step_out = v_bias_relay + OUTSTEP/2; //heating initially
      }
//      Serial.print(find_period_flag);
//	  Serial.print(", ");
      output_bias(step_out,1); 
      delay(50);
}
void DTC03::RelaySwitchTime(unsigned long *relay_period, int &counter, bool &find_period_flag)
{
	relay_period[counter]=millis();
  	counter += 1;
//  	Serial.print("counter#");
//	Serial.print(counter);
//	Serial.print(", ");
//	Serial.println(millis()) ;
  	if(counter==4) 
  	{   	
//    	Serial.print("Period = ");
//    	Serial.print(relay_period[0]);
//    	Serial.print(", ");
//    	Serial.print(relay_period[1]);
//    	Serial.print(", ");
//    	Serial.print(relay_period[2]);
//    	Serial.print(", ");
//    	Serial.print(relay_period[3]);
//    	Serial.print(", ");
//    	Serial.println(relay_period[3]-relay_period[1]);
    	
    	counter = 0;
    	p_relayT = (relay_period[3]-relay_period[1])/1000;
    	find_period_flag = 1;
  	}
  	
}
void DTC03::parameter(int *peakcount, unsigned int *peaks, unsigned long *peaktime, int *A, unsigned long *Pu) //peakcount should >=2
{
  int Atemp;
  unsigned long Putemp;
  Atemp = abs( (int)peaks[*peakcount]-(int)peaks[*peakcount-1]);
  Putemp = peaktime[*peakcount]-peaktime[*peakcount-2];
  
  if (*A ==0) *A = (abs((int)peaks[*peakcount]-(int)peaks[*peakcount-1])+abs((int)peaks[*peakcount-1]-(int)peaks[*peakcount-2])) >>1;
  else *A = (*A+Atemp) >>1;
  
  if (*Pu==0) *Pu = Putemp;
  else *Pu = (*Pu+Putemp)>>1; 
//  Serial.print("peakcount:");
//  Serial.print(*peakcount);
//  Serial.print(", Atemp:");
//  Serial.print(Atemp);
//  Serial.print(", Putemp:");
//  Serial.println(Putemp);
  
  
}
void DTC03::peakrecord (unsigned int &input, bool *ismax, bool *ismin, int *peaktype, int *peakcount, unsigned int *peaks, int *peakstemp, unsigned long *peaktime, unsigned long now, unsigned long *t1, bool *justchanged)
{
  //當*ismax or *ismin為T時，紀錄peak之時間與振幅，只有第一次的*ismax or *ismin是T時才給定*peaktype值(1 or -1)，而當*peaktype變號時則代表發現peak了。
  if(*ismax)
  {
      if(*peaktype == 0) 
      {
        *peaktype =1;
      }
      if(*peaktype ==-1) 
      {
        *peaktype = 1;
        *peakcount+=1;
        peaktime[*peakcount] = *t1;
        peaks[*peakcount]=*peakstemp;
        *justchanged = true;
        //////atune data print-4/////
//        Serial.print(", ");
//        Serial.print("min #");
//        Serial.println(*peakcount);
//        Serial.print(", ");
//        Serial.print(peaktime[*peakcount]);       
//        Serial.print(", ");
//        Serial.print(peaks[*peakcount]);  
      }
//      Serial.println();
      *t1 = now;
      *peakstemp = input;    
  }
  else if(*ismin)
  {
      if(*peaktype ==0) 
      {
        *peaktype =-1;        
      }
      if(*peaktype == 1) 
      {
        *peaktype = -1;
        *peakcount +=1;
        peaktime[*peakcount] = *t1;
        peaks[*peakcount]=*peakstemp;
        *justchanged = true;
        //////atune data print-5/////
//        Serial.print(", ");
//        Serial.print("max #");
//        Serial.println(*peakcount);
//        Serial.print(", ");
//        Serial.print(peaktime[*peakcount]);       
//        Serial.print(", ");
//        Serial.print(peaks[*peakcount]);        
      }
//      Serial.println();
      *t1 = now;
      *peakstemp = input;         
  }   
  else
  {
//    Serial.println();
  }

}
void DTC03::lookbackloop (unsigned int &input, unsigned int *lastinput, boolean *ismax, boolean *ismin) //如果在指定的lookback 數目裡有發現possible maxima(minima)的話*ismax(*ismin)才會是T, 否則為F
{
  for (int i=MAXLBACK-2;i >=0;i--) 
  {
     if(*ismax) *ismax= input>lastinput[i];
     if(*ismin) *ismin= input<lastinput[i];
     lastinput[i+1]=lastinput[i];     
  }
  //////atune data print-3/////
//  Serial.print(",");
//  if(*ismax) Serial.print("1");
//  else if (*ismin) Serial.print("-1");
//  else Serial.print("0");
}

void DTC03::input_bias(unsigned int &in_add, uint8_t MV_ON)
{
	ReadVoltage(1);
	switch(MV_ON)
	{
		case 0:
			in_add = (int)g_vact;
			break;
		case 1:
			in_add = g_atuneVact_MV;
			break;
		case 2:
			in_add = g_vact_MV;
			break;
	}
//	if(MV_ON) in_add = g_atuneVact_MV;
//	else in_add = (int)g_vact;
}
void DTC03::output_bias(unsigned int Out, bool mode)
{	
	SetMos(HEATING,Out);
	if(mode)
	{
	    ////atune data print-2/////
//	    Serial.print(", ");
//	    Serial.println(Out);	    
	}
	else
	{
	//    Serial.print(Out);
	//    Serial.print(", ");    
	//    Serial.println(dtc.g_vact);
//	    if(AUTUNE_MV_STATUS) p_noise_Mid = g_vact_MV;
//	    else p_noise_Mid = g_vact;
//	    Serial.println(p_noise_Mid);
	//    delay(500);
	}
}
unsigned int DTC03::FindBiasCurrent(float &t_leave, uint8_t &flag, unsigned int &v_bias_find, unsigned int (&v_bias)[FINDBIASARRAY], unsigned long &ts, unsigned long &runtime, int &k)
{
	unsigned int v_now;
	float  t_now, t_bias;
	long err, out, tout, iteclimit, iset, ierr, isense, ioutput;
	unsigned long te;
	switch(flag)
	{
		case 0:
			pid.Init(32768,0,1,2,0 ); //PID::Init( long long p_limit, long long i_limit, unsigned char ki, unsigned char ls, unsigned char err_gain)
			ipid.Init(32768,32768,g_ki,g_ls,7);
			for(int i=0;i<FINDBIASARRAY;i++) v_bias[i]=0; // array space for determing stability condition
			input_bias(v_now,0);
			t_now = ReturnTemp(v_now,0);
			t_leave = t_now;
			t_bias = t_now + g_T_atune*0.1;
			v_bias_find = ReturnVset(t_bias, 0);			
			ts = millis();	
			flag = 1;			
			g_dbrCounter_flag = 1;
			return(0);
//			Serial.print("t_bias=");
//			Serial.println(t_bias,1);
			
		break;
		case 1:
			input_bias(v_now,1);
			ReadIsense();
			CurrentLimit();
			iteclimit = (long)g_iteclimitset<<7;
		
			err = (long)v_now - (long)v_bias_find; 
			tout = pid.Compute(1, err, g_p_atune, 0, 0); // PID::Compute(bool en, long errin, unsigned char kp, unsigned char ki, unsigned char ls)
			iset=abs(tout*0.586);
  			if(iset > iteclimit) iset=iteclimit;
  			isense =abs(((long)(g_itecread)-(long)(g_isense0))<<7 );
  			ierr = isense - iset;
  			ioutput=ipid.Compute(1, ierr, 20, 10, 1);
			
			out = (long)(abs(ioutput)+g_fbc_base);
			v_bias[k%FINDBIASARRAY] = v_now;
			if(tout<=0) SetMos(HEATING,out);
  			else SetMos(COOLING,out);
  			
			
			bool stable_flag = 0;
			unsigned int v_bias_max, v_bias_min;
			te = millis();
//			Serial.print("test condition:");
//			Serial.print(te-ts);
//			Serial.print(", ");
//			Serial.println((ReturnTemp(v_now,0)-t_leave));
			if((te-ts)>=SAMPLINGTINE && (ReturnTemp(v_now,0)-t_leave)>0.1)
			{		
				
				if(g_dbrCounter_flag)
				{
					g_dbrCounter_flag = 0;
					g_dbr_counter[0] = millis();
				}
				v_bias_max = v_bias[0];
				for(int i=0; i<FINDBIASARRAY;i++)
				{
					if(v_bias[i] > v_bias_max) v_bias_max = v_bias[i];				
				}
				
				v_bias_min = v_bias[0];
				for(int i=0; i<FINDBIASARRAY;i++)
				{
					if(v_bias[i] < v_bias_min) v_bias_min = v_bias[i];	
//					Serial.print(v_bias[i]);
//					if(i<FINDBIASARRAY)	Serial.print(", ");
//					else Serial.println();		
				}

//				Serial.print(v_bias_max-v_bias_min);
//				Serial.print(", ");
//				Serial.println(g_stableCode_atune);
				if((v_bias_max - v_bias_min) <= g_stableCode_atune) stable_flag=1;
				if((v_bias_max - v_bias_min)<=15) 
				{
					g_dbr_counter[1] = millis();
					unsigned long dbr_counter_tdiff = g_dbr_counter[1]-g_dbr_counter[0];					
//					Serial.print("dbr_counter_tdiff=");
//					Serial.println(dbr_counter_tdiff);
					if(dbr_counter_tdiff < 20000) // means time to reach dT<15 code status less than 20s -> judge as DBR case
					{
						g_DBRflag=1;
						flag = 2;
						return(out);
					}					
				}
				
								
				runtime = k*(te-ts);
//				Serial.print("runtime=");
//				Serial.println(runtime);
				ts = te;	
				k++;		
			}		
				
			
			if(stable_flag) flag = 2;						
			return(out);
		break;
	}	
}
uint8_t DTC03::atunKp(float &kp)
{
	int kp_temp;
	
	kp_temp = round(kp);
	return(kp_temp);
}
uint8_t DTC03::atunKiLs(float &tc)
{
	int tc_temp, a, b;

	if((int)(tc*10) <= 20)// input: 0.1~2.0
	{
		tc_temp = tc*10+2;
		return(tc_temp);
	} 
	else if((int)(tc+0.5) <= 5)// input: 2.5~5.0
	{
		tc_temp = tc*10;
		a = tc_temp/10; 
		b = (tc-a)*10;
		if(b<5) 
		{
			if((5-b)>b) tc_temp = a*10;
			else tc_temp = a*10+5;			
		}
		else 
		{
			if((10-b)>(b-5)) tc_temp = a*10+5;
			else tc_temp = (a+1)*10;
		}
		switch(tc_temp)
		{
			case 20:
				tc_temp = 22;
			break;	
			case 25:
				tc_temp = 23;
			break;
			case 30:
				tc_temp = 24;
			break;
			case 35:
				tc_temp = 25;
			break;
			case 40:
				tc_temp = 26;
			break;
			case 45:
				tc_temp = 27;
			break;
			case 50:
				tc_temp = 28;
			break;
			case 55:
				tc_temp = 28;
			break;
		}
		return(tc_temp);
	}
	else
	{
		tc_temp = round(tc);
		if(tc_temp==11) tc_temp=10;
		else if(tc_temp==11) tc_temp=10;
		else if(tc_temp==13) tc_temp=12;
		else if(tc_temp==15) tc_temp=14;
		else if(tc_temp==17) tc_temp=16;
		else if(tc_temp==19) tc_temp=18;
		else if(tc_temp==21 || tc_temp==22) tc_temp=20;
		else if(tc_temp==23 || tc_temp==24 || tc_temp==26 || tc_temp==27) tc_temp=25;
		else if(tc_temp==28 || tc_temp==29 || tc_temp==31 || tc_temp==32) tc_temp=30;
		else if(tc_temp==33 || tc_temp==34 || tc_temp==36 || tc_temp==37) tc_temp=35;
		else if(tc_temp==38 || tc_temp==39 || tc_temp==41 || tc_temp==42) tc_temp=40;
		else if(tc_temp==43 || tc_temp==44 || tc_temp==46 || tc_temp==47) tc_temp=45;
		else if(tc_temp==48 || tc_temp==49 || tc_temp==51 || tc_temp==52) tc_temp=50;
		else if(tc_temp==53 || tc_temp==54 || tc_temp==56 || tc_temp==57) tc_temp=55;
		else if(tc_temp==58 || tc_temp==59 || tc_temp==61 || tc_temp==62) tc_temp=60;
		else if(tc_temp==63 || tc_temp==64 || tc_temp==66 || tc_temp==67) tc_temp=65;
		else if(tc_temp==68 || tc_temp==69 || tc_temp==71 || tc_temp==72) tc_temp=70;
		else if(tc_temp==73 || tc_temp==74 || tc_temp==76 || tc_temp==77) tc_temp=75;
		else if(tc_temp==78 || tc_temp==79) tc_temp=80;		
		switch(tc_temp)
		{
			case 6:
				tc_temp = 29;
			break;
			case 7:
				tc_temp = 30;
			break;
			case 8:
				tc_temp = 31;
			break;
			case 9:
				tc_temp = 32;
			break;
			case 10:
				tc_temp = 33;
			break;
			case 12:
				tc_temp = 34;
			break;
			case 14:
				tc_temp = 35;
			break;
			case 16:
				tc_temp = 36;
			break;
			case 18:
				tc_temp = 37;
			break;
			case 20:
				tc_temp = 38;
			break;
			case 25:
				tc_temp = 39;
			break;
			case 30:
				tc_temp = 40;
			break;
			case 35:
				tc_temp = 41;
			break;
			case 40:
				tc_temp = 42;
			break;
			case 45:
				tc_temp = 43;
			break;
			case 50:
				tc_temp = 44;
			break;
			case 55:
				tc_temp = 45;
			break;
			case 60:
				tc_temp = 46;
			break;
			case 65:
				tc_temp = 47;
			break;
			case 70:
				tc_temp = 48;
			break;
			case 75:
				tc_temp = 49;
			break;
			case 80:
				tc_temp = 50;
			break;
		}
		return(tc_temp);
	}
}

//void DTC03::CheckSerial()
//{
//	unsigned int in;
////	Serial.begin(9600);
//	if(Serial.available()>0)
//  	{
//	    in = Serial.read();
//	    switch(in)
//	    {
//	      case '1':
//	        Serial.println(is<<8 | js);
//	        js++;
//	        break;
//	       case '2':
//	        Serial.println(is<<8 | js);
//	        is++;
//	        break;
//		}  
//	}
////	Serial.end();
//}






















