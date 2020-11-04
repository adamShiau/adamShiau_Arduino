/* modification for disable sensortype purpose : data 2016/11/13
	1. in ReadEEPROM(), set g_sensortype to 0;
	2. add g_mod_status variable in DTC03_P06.h
*/
//#include <Wire.h>
//#include <SPI.h>
//#include <LTC1865.h>
//#include <EEPROM.h>
//#include <AD5541.h>
//#include <PID.h>
#include <DTC03_INITCHECK.h>
//#include <DTC03_MSv2.h>

DTC03::DTC03()
{}

void DTC03::CheckInitValue(bool sw1, bool sw2, bool sw3) 
{
	#ifdef INITIAL_VALUE_CHECK
		Serial.begin(9600);
		int itec_array[3], itec, izero;
		byte i=0, di=0;
		unsigned int fbc_heating, i_step=0, fbc_cooling, cal_R_value, mod_avg, mod_std;
		unsigned long mod_sum = 0;
		boolean fbc_flag = 1;
		SetMosOff();
		digitalWrite(SENSOR_TYPE,0);

		if(sw1)
		{
			Serial.println(F("#Please measure VCC voltage"));

#if 0	//sherry+- 2017.11.30
			SetVcc(VCCLOW);
			Serial.print("Vcc LOW : ");
			for (int i=10; i>0; i--) {
				Serial.println(i);
				delay(1000);
			}
			SetVcc(VCCHIGH);
			Serial.print("Vcc HIGH : ");
			for (int i=5; i>0; i--) {
				Serial.println(i);
				delay(1000);
			}
			SetVcc(VCCMEDIUM);
			Serial.print("Vcc MID : ");
			for (int i=5; i>0; i--) {
				Serial.println(i);
				delay(1000);
			}
#else
			Serial.println(F("Set Vcc LOW : "));
			SetVcc(VCCLOW);
			Serial.println(F("Press ENTER to continue after read Vcc LOW : "));
			while(Serial.available() == 0) ;
			Serial.readString();

			Serial.println(F("Set Vcc HIGH : "));
			SetVcc(VCCHIGH);
			Serial.println(F("Press ENTER to continue after read Vcc HIGH : "));
			while(Serial.available() == 0) ;
			Serial.readString();

			Serial.println(F("Set Vcc MID : "));
			SetVcc(VCCMEDIUM);
			Serial.println(F("Press ENTER to continue after read Vcc MID : "));
			while(Serial.available() == 0) ;
			Serial.readString();
#endif
		}

		Serial.println(F("---------------------------"));
		Serial.println(F("#Searching  Fbc_base Value"));
		Serial.print("Vgs_low = ");
		Serial.println(FBCCHECK_LOW);
		Serial.print("Vgs_high = ");
		Serial.println(FBCCHECK_HIGH);
		
		
		if(sw2)
		{
			//SetMosOff();	
			SetMos(HEATING, 0);
			izero = ReadIsense();
			Serial.print("current0 = ");
		    Serial.println(izero);				
			////heating//////
			Serial.println("heating :");
			Serial.println("Vgs   , Itec");
			itec = ReadIsense();
	//		Serial.println(itec);
			while( ( abs(itec - izero) ) < 5) 
			{ // old value 5
				SetMos(HEATING, FBCCHECK_LOW + i_step);
				delay(100);
				itec = ReadIsense();
	//			Serial.println(itec);
	//			delay(5000);
				if (i>2) i=2;
				itec_array[i] = itec-izero; 
				Serial.print(FBCCHECK_LOW + i_step);
				Serial.print("  , ");
#if 0  //sherry+- 2018.8.2
				Serial.println(itec_array[i]);
//				Serial.print("  , ");
//				Serial.println(itec*5/1023); //check read voltage value 
#else	//sherry+- 2018.3.28
				Serial.print(itec_array[i]);
				Serial.print("  , ");
				Serial.println(itec*5/1023); //check read voltage value 
#endif
				
				if ( (i = 2) && fbc_flag )
				{
					if ( (abs(itec_array[0]) > di) && (abs(itec_array[1]) > di) && (abs(itec_array[2]) > di) && fbc_flag ){
						fbc_flag = 0; 
						fbc_heating = FBCCHECK_LOW + i_step - 400;
					}  
					 itec_array[0] = itec_array[1];
					 itec_array[1] = itec_array[2];
				}
				i_step += 100;
				if((FBCCHECK_LOW + i_step)>FBCCHECK_HIGH) i_step = 0;
				i++;
			}	
			
		
		
			////cooling////
			//SetMosOff();
			SetMos(COOLING, 0);			
			izero = ReadIsense();
			i_step = 0; 
			i = 0; 
			fbc_flag = 1;

			Serial.println("cooling :");
			itec = ReadIsense();
			
			while( ( abs(itec - izero) ) < 49)
			{
				SetMos(COOLING, FBCCHECK_LOW + i_step);
//				SetMos(COOLING, 0);
//				SetMosOff();
				delay(100);
				itec = ReadIsense();
				if (i>2) i=2;
				itec_array[i] = itec-izero;
				Serial.print(FBCCHECK_LOW + i_step);
				Serial.print("  , ");
				Serial.println(itec_array[i]);
//				Serial.println(itec);
				
				if ( (i = 2) && fbc_flag )
				{
					if ( (abs(itec_array[0]) > di) && (abs(itec_array[1]) > di) && (abs(itec_array[2]) > di) && fbc_flag ){
						fbc_flag = 0; 
						fbc_cooling = FBCCHECK_LOW + i_step - 400;
					}  
					 itec_array[0] = itec_array[1];
					 itec_array[1] = itec_array[2];
				}
				cal_R_value = FBCCHECK_LOW + i_step; 
				i_step += 100;
				if((FBCCHECK_LOW + i_step)>FBCCHECK_HIGH) i_step = 0;
				i++;
			}
			SetMosOff();
		
		
		    Serial.println("    ");
		    Serial.print("fbc_base heating = ");
		    Serial.println(fbc_heating);
		    Serial.print("fbc_base cooling = ");
		    Serial.println(fbc_cooling);
		    Serial.println("Choose lower one as fbc_base!");
		    Serial.println("    ");
		    Serial.print("0.5A for calculate R = ");
		    Serial.println(cal_R_value);		
		    Serial.println(" ");
	    }
		
		Serial.println(F("---------------------------"));

		if(sw3)
		{
			String sInputString;         // a String to hold incoming data
			int cInput = 0;
			long cOutput = 0;
			int mod_avg_0V = 0;
			int mod_avg_diff = 0;
			float temp = 0;
       
			Serial.println("#Checking Mod input offest value");
		    Serial.println("Unplug BNC cable from MOD Input.");
	    	for ( int i=0; i<100; i++) 
     		{
	    		ReadVoltage();
	    		mod_sum += g_vmod;
	    		Serial.print(g_vact);
	    		Serial.print(", ");
	    		Serial.print(g_vmod);
	    		Serial.print(", ");
	    		Serial.println((float)ReadIsense()*5/1023);		
		    }
    		mod_avg = mod_sum/100;
	    	Serial.print("100 times AVG MOD offset value = ");
	    	Serial.println(mod_avg);
	    	Serial.println("Normal value is around 32768; if not, ");
	    	Serial.println("check +5VB, MOD buffer OPA");
	    	mod_sum = 0; 
	    	for ( int i=0; i<100; i++) 
			{
		    	ReadVoltage();
		    	mod_sum += sq(g_vmod - mod_avg); 			
		    }
	    	mod_std = sqrt(mod_sum/100);
	    	Serial.print("STD = ");
	    	Serial.println(mod_std);
	    	
	    	Serial.println(F("---------------------------"));
	    	Serial.println(F("#Check modulation function"));

#if 0	//sherry+- 2017.12.1
	    	Serial.println(F("Add DC 100mV to mod BNC:"));
#else
	    	mod_avg_0V = mod_avg;
		    Serial.println(F("Plug BNC cable from MOD Input."));
	    	Serial.println(F("Set DC about 400mV , 200mv , -200mv and -400mv to mod BNC:"));
	    	Serial.println(F("[ The actual value will be divided by 2 ]"));
	   		Serial.println();

	    	for ( int x = 0; x < 4; x++) 
     		{
     			mod_sum = 0;
     			g_vmod = 0;
     			mod_avg = 0;

				Serial.println(F("Input mv value after set voltage at MULTIMETER : "));
				while(Serial.available() == 0) ;
				sInputString = Serial.readStringUntil('\n');
				cInput = sInputString.toInt()/2;
				Serial.print(cInput);
				cOutput = cInput * 65535 / 5000;
				Serial.print(" mv output should be around : ");
				Serial.println(cOutput);

		    	for ( int i = 0; i < 100; i++) 
     			{
	    			ReadVoltage();
	    			mod_sum += g_vmod;
		    		//Serial.print(g_vmod);
		    		Serial.print(", ");
		    		//if ((i % 10) == 9)
			    	//	Serial.println();
			    }
			    Serial.println();

   		 		mod_avg = mod_sum/100;
	    		Serial.print("100 times AVG MOD offset value = ");
		    	Serial.println(mod_avg);
		    	Serial.print("diff from 0V = ");
		    	mod_avg_diff = mod_avg - mod_avg_0V;
		    	Serial.print(mod_avg_diff);
		    	Serial.print(", temp = ");
		    	temp = ReturnTemp(mod_avg_diff + 26214);
		    	Serial.println(temp,1);
		    	Serial.print("diff = ");
	    		Serial.println(mod_avg_diff - cOutput);
	    		Serial.println();
	    	}
#endif

		}
		
		
		
				
	#else
		Serial.println(F("Open INITIAL_VALUE_CHECK flag to use CheckFbcValue() function"));
	#endif		

}
float DTC03::ReturnTemp(unsigned int vact)
{
  float tact;
  tact = 1/(log((float)vact/RTHRatio)/BVALUE+T0INV)-273.15;
  return tact;
}
void DTC03::DynamicVcc()
{
    float restec, r1, r2;
  	
    if(g_sensortype) digitalWrite(SENSOR_TYPE,g_sensortype);
    
    g_isense0 = ReadIsense();
    r1 = float(g_r1)*0.1;
    r2 = (float(g_r2)*0.1);
    #ifdef DEBUGFLAG01
      Serial.begin(9600);
      Serial.println("g_fbc_base: (if g_fbc_base > RMEASUREVOUT, Vgs=RMEASUREVOUT; otherwise Vgs = g_fbc_base+6000 )");
      Serial.println(g_fbc_base);
      Serial.println("  ");
      Serial.println("R1, R2 for dynamic Vcc selection: ");
      Serial.print(r1);
      Serial.print(", ");
      Serial.println(r2);
      Serial.println("=====Isense0 parameter====");
      Serial.print("Avgtime:");
      Serial.println(IAVGTIME);
      Serial.print("Isense0:");
      Serial.println(g_isense0);//20161031
    #else
    #endif
    
//    if (g_fbc_base > RMEASUREVOUT) restec = CalculateR(RMEASUREVOUT,RMEASUREDELAY,RMEASUREAVGTIME,IAVGTIME);
//	else restec = CalculateR(g_fbc_base + 6000 ,RMEASUREDELAY,RMEASUREAVGTIME,IAVGTIME);
	restec = CalculateR(RMEASUREVOUT,RMEASUREDELAY,RMEASUREAVGTIME,IAVGTIME);
    if (restec < g_r1 ) SetVcc(VCCLOW);
    else if(restec < g_r2 ) SetVcc(VCCMEDIUM);
    else SetVcc(VCCHIGH);
	
//	if (restec < VCCRTH_LM) SetVcc(VCCLOW);
//    else if(restec < VCCRTH_MH) SetVcc(VCCMEDIUM);
//    else SetVcc(VCCHIGH);
}
float DTC03::CalculateR(unsigned int fb_value, unsigned int stabletime, int ravgtime, int vavgtime)
{
  
  int i,vtec0;
  float rsum=0, rtec, ravg, vtec, itec;

  SetMosOff();
  g_itecavgsum = 0;
  vtec0=ReadVtec(vavgtime);

  #ifdef DEBUGFLAG01
    Serial.println("=====TEC parameter====");
    Serial.print("Avgtime:");
    Serial.println(vavgtime);
    Serial.print("Vtec0:");
    Serial.println(vtec0);
    Serial.println("n   vtec  itec   rtec ");//20161031
  #else
  #endif

  SetMos(COOLING, fb_value);  // using cooling path and dac output = fb_value;
  Serial.print("Vgs to R calculate =");
  Serial.println(fb_value);
  delay(stabletime);          // delay stabletime in ms
  for (i=0; i< ravgtime; i++)
  {
    vtec = float(ReadVtec(vavgtime)-vtec0);
//    Serial.println("pass vtec");
//    delay(1000);
    itec = float(ReadIsense()-g_isense0);
//    Serial.println("pass itec");
//    delay(1000);
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
  //  Serial.println("end caculate");
//  delay(1000);
  ravg = rsum/float(ravgtime);
  #ifdef DEBUGFLAG01
    Serial.print("ravg:");
    Serial.println(ravg);
  #else
  #endif
//  delay(5000);
  SetMosOff();//20161031 turn off all mos after measuring R
  return ravg;
  
}
void DTC03::ParamInit()
{
  dacformos.SetPin(DACC);
  dacforilim.SetPin(CURRENT_LIM);
  SetVcc(VCCHIGH);
  SetMosOff();
  dacforilim.ModeWrite(0);
  g_en_state = 0;
  g_errcode1 = 0;
  g_errcode2 = 0;
  g_vactavgsum = 0;
  g_itecavgsum = 0;
  g_currentindex = 0;
  g_vactindex = 0;
  g_ilimdacout = 65535;
  g_limcounter =0;

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
void DTC03::SetMos(bool heating, unsigned int fb_value)
{
	PORTD = (PORTD| MOS_ON_OFF_STATUS_ADD) & ((heating << NMOSC_IN)|(!heating<< NMOSH_IN)|(!heating << FBSEL));
	dacformos.ModeWrite(fb_value);
}
void DTC03::SetMosOff()
{
  PORTD = PORTD | MOS_ON_OFF_STATUS_ADD;
  dacformos.ModeWrite(0);
}
int DTC03::ReadIsense()
{ 
  int i, currentavg;
  g_itecavgsum=0;//20161031
  for (i=0; i< ITECAVGTIME; i++) 
    {
      Itecarray[i]= analogRead(ISENSE0);
      g_itecavgsum += Itecarray[i];
    }
  currentavg = g_itecavgsum >> ITECAVGPWR;
  return currentavg;
}
int DTC03::ReadVtec(int avgtime)
{
  int i, vtec;
  long vtecsum=0;

  for (i=0; i < avgtime; i++) vtecsum += analogRead(TEC_VOLT);

  vtec = vtecsum/avgtime;

  return vtec;
}



unsigned int DTC03::InitVactArray()
{
  int i;
  unsigned int vactavg;
  ltc1865.init(LTC1865CONV, CHVACT);  // CHVACT: next channel to read 
  for(i=0; i < VACTAVGTIME; i++)
  {
    Vactarray[i]=ltc1865.Read(CHVACT);  // CHVACT: next channel to read 
    g_vactavgsum += Vactarray[i];
  }
  vactavg= g_vactavgsum >> VACTAVGPWR;
  g_vact = vactavg;
  return vactavg;
}
//void DTC03::ReadEEPROMnew()
//{
//  unsigned char ee_vset_upper, ee_vset_lower, ee_b_upper,ee_b_lower, ee_fbc_base_upper, ee_fbc_base_lower, ee_i;
//  unsigned char ee_vmodoffset_upper, ee_vmodoffset_lower, ee_dummy;
//  ee_dummy = EEPROM.read(EEADD_DUMMY);
//  if (ee_dummy == NOEE_DUMMY)
//  {
//    g_p = EEPROM.read(EEADD_P);
//    g_kiindex = EEPROM.read(EEADD_KIINDEX);//
//    g_ki = pgm_read_word_near(kilstable+g_kiindex*2+1);//20161109
//    g_ls = pgm_read_word_near(kilstable+g_kiindex*2);//
//    g_currentlim = EEPROM.read(EEADD_currentlim);
//    ee_vset_upper = EEPROM.read(EEADD_Vset_upper);
//    ee_vset_lower = EEPROM.read(EEADD_Vset_lower);
//    g_b_upper = EEPROM.read(EEADD_B_upper);
//    g_b_lower = EEPROM.read(EEADD_B_lower);
////    g_sensortype = EEPROM.read(EEADD_Sensor_type);
//    g_sensortype = 0;
//    g_r1 = EEPROM.read(EEADD_R1); // vbeh1*256 - vbeh2*16*g_currentlim = g_vbeh(vbe target when reach currentlimit)
//    g_r2 = EEPROM.read(EEADD_R2);
//    g_tpidoffset = EEPROM.read(EEADD_PIDOFFSET);
//    g_vbec2 = EEPROM.read(EEADD_VBE_C2);
//    ee_fbc_base_upper = EEPROM.read(EEADD_FBC_base_upper);
//    ee_fbc_base_lower = EEPROM.read(EEADD_FBC_base_lower);
//    ee_vmodoffset_upper = EEPROM.read(EEADD_Vmodoffset_upper);
//    ee_vmodoffset_lower = EEPROM.read(EEADD_Vmodoffset_lower); 
//    g_vset_limit = ee_vset_upper<<8 | ee_vset_lower;
//    g_fbc_base = ee_fbc_base_upper<<8 | ee_fbc_base_lower;
//    g_vmodoffset = ee_vmodoffset_upper<<8 | ee_vmodoffset_lower;
//  }
//  else
//  {
//    g_p = NOEE_P;
//    g_ki = NOEE_KI;
//    g_ls = NOEE_LS;
//    g_kiindex = NOEE_KIINDEX;//20161103
//    g_currentlim = NOEE_ILIM;
//    g_vset_limit = NOEE_VSET;
//    g_sensortype = NOEE_SENS;
//    g_b_upper = NOEE_B>>8;
//    g_b_lower = NOEE_B;
//    g_r1 = NOEE_R1; //R1
//    g_r2 = NOEE_R2; //R2
//    g_tpidoffset = NOEE_PIDOFFSET; //Tpid offset
//    g_tpidoffset = g_tpidoffset;
//    g_vbec2 = NOEE_VBEC2; //no use now
//
//    g_fbc_base = NOEE_FBC;
//    g_vmodoffset =NOEE_OFFSET;
//    EEPROM.write(EEADD_DUMMY, NOEE_DUMMY);
//    
//  }
//  }
//void DTC03::ReadEEPROM()
//{
//  unsigned char ee_vset_upper, ee_vset_lower, ee_b_upper,ee_b_lower, ee_fbc_base_upper, ee_fbc_base_lower, ee_i;
//  unsigned char ee_vmodoffset_upper, ee_vmodoffset_lower;
//
//  #ifdef DEBUGFLAG03
//
//
//    g_p = NOEE_P;
//    g_ki = NOEE_KI;
//    g_ls = NOEE_LS;
//    g_kiindex = NOEE_KIINDEX;//20161103
//    g_currentlim = NOEE_ILIM;
//    g_vset_limit = NOEE_VSET;
//    g_sensortype = NOEE_SENS;
//    g_b_upper = NOEE_B>>8;
//    g_b_lower = NOEE_B;
//    g_vbeh1 = NOEE_R1; //R1
//    g_r2 = NOEE_R2; //R2
//    g_tpidoffset = NOEE_PIDOFFSET; //Tpid offset
//    g_tpidoffset = g_tpidoffset;
//    g_vbec2 = NOEE_VBEC2; //no use now
//    g_vbeh = (g_vbeh1<<8) - (g_r2<<4)*g_currentlim;//20161031 ,add() on g_vbeh1<<8,
//    g_vbec = (g_tpidoffset<<8) - (g_vbec2<<4)*g_currentlim;//20161031 ,as ablove
//    g_fbc_base = NOEE_FBC;
//    g_vmodoffset =32405;
//  
//  #else
//
//    g_p = EEPROM.read(EEADD_P);
//    g_kiindex = EEPROM.read(EEADD_KIINDEX);//
//    g_ki = pgm_read_word_near(kilstable+g_kiindex*2+1);//20161109
//    g_ls = pgm_read_word_near(kilstable+g_kiindex*2);//
//    g_currentlim = EEPROM.read(EEADD_currentlim);
//    ee_vset_upper = EEPROM.read(EEADD_Vset_upper);
//    ee_vset_lower = EEPROM.read(EEADD_Vset_lower);
//    g_b_upper = EEPROM.read(EEADD_B_upper);
//    g_b_lower = EEPROM.read(EEADD_B_lower);
////    g_sensortype = EEPROM.read(EEADD_Sensor_type);
//    g_sensortype = 0;
//    g_vbeh1 = EEPROM.read(EEADD_R1); // vbeh1*256 - vbeh2*16*g_currentlim = g_vbeh(vbe target when reach currentlimit)
//    g_r2 = EEPROM.read(EEADD_R2);
//    g_tpidoffset = EEPROM.read(EEADD_PIDOFFSET);
//    g_tpidoffset = g_tpidoffset;
//    g_vbec2 = EEPROM.read(EEADD_VBE_C2);
//    ee_fbc_base_upper = EEPROM.read(EEADD_FBC_base_upper);
//    ee_fbc_base_lower = EEPROM.read(EEADD_FBC_base_lower);
//    ee_vmodoffset_upper = EEPROM.read(EEADD_Vmodoffset_upper);
//    ee_vmodoffset_lower = EEPROM.read(EEADD_Vmodoffset_lower); 
//    g_vbeh = (g_vbeh1<<8) - (g_r2<<4)*g_currentlim;//20161031 ,add() on g_vbeh1<<8,
//    g_vbec = (g_tpidoffset<<8) - (g_vbec2<<4)*g_currentlim;//20161031 ,add() on g_vbeh1<<8,
//    g_vset_limit = ee_vset_upper<<8 | ee_vset_lower;
//    g_fbc_base = ee_fbc_base_upper<<8 | ee_fbc_base_lower;
//    g_vmodoffset = ee_vmodoffset_upper<<8 | ee_vmodoffset_lower;
//  #endif
//}
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
  int Vtemp;
  Vtemp = analogRead(TEMP_SENSOR);
  if(Vtemp > V_OVERTEMP) 
    {
      g_errcode2 = 1;
      g_en_state =0;
    }
  else g_errcode2 = 0;
}
void DTC03::ReadVoltage()
{
  long vmod,vset_limit_long;
  g_vact = ltc1865.Read(CHVMOD);
  g_vmod = ltc1865.Read(CHVACT);
  //vmod = g_vmod - g_vmodoffset;
//  vmod = long(g_vmod) - long(g_vmodoffset);//
//  g_vactavgsum -= Vactarray[g_vactindex];
//  Vactarray[g_vactindex] = g_vact;
//  g_vactavgsum += g_vact;
//  g_vactindex++;
//  if(g_vactindex >= VACTAVGTIME) g_vactindex =0;
//  
////  vset_limit_long=(long)(g_vset_limit)+vmod;//20161113
//  if (g_mod_status) vset_limit_long=(long)(g_vset_limit)+vmod;//20161113
//  else vset_limit_long=(long)(g_vset_limit);//20161113
//  
//  if(vset_limit_long>65535) vset_limit_long=65535;
//  else if (vset_limit_long<0) vset_limit_long=0;
//  g_vset_limitt = (unsigned int)vset_limit_long;
//
//  //g_vset_limit = (unsigned int)(long(g_vset_limit)+vmod);//
//  #ifdef DEBUGFLAG03
//  #else 
//  //g_vset +=vmod;  //can unsigned int add int?
//  #endif
}
//void DTC03::VsetSlow()
//{
//  if(g_en_state)
//  {
//    if((g_vset <= g_vset_limit) &&(g_limcounter % LIMCOUNTER ==0))
//    {
//      g_vset += VSETSLOWSTEP;
//      if(g_vset >= g_vset_limit) g_vset = g_vset_limit;
//    }
//    else if((g_vset > g_vset_limit) &&(g_limcounter % LIMCOUNTER ==0))
//    {
//      g_vset -= VSETSLOWSTEP;
//      if(g_vset <= g_vset_limit) g_vset = g_vset_limit;
//    }
//  }
//  else g_vset = g_vact;
//}
//void DTC03::CurrentLimitGain(bool heating)
//{
//  if(heating)
//    g_ilimgain = (float)g_vbeh/(float)(ILIMDACOUTSTART+ILIMDACSTEP*g_currentlim);
//  else
//    g_ilimgain = (float)g_vbec/(float)(ILIMDACOUTSTART+ILIMDACSTEP*g_currentlim);
//}
//void DTC03::CurrentLimit()
//{
//  unsigned int currentabs;
//  unsigned int dacout;
//  unsigned char i;//
//  noInterrupts();//
//  g_itecavgsum -=Itecarray[g_currentindex];
//  Itecarray[g_currentindex] = analogRead(ISENSE0);
//  g_itecavgsum +=Itecarray[g_currentindex];//20161101 wrong sign
//  g_itecread = Itecarray[g_currentindex];//
//  interrupts();//
//  //g_currentabs = abs(int(g_itecavgsum>>ITECAVGPWR)-int(g_isense0));//
//  g_currentindex ++;
//  if(g_currentindex == ITECAVGTIME) g_currentindex = 0;
//  g_iteclimitset=(int)(500+50*(g_currentlim))/10;//
//}
//void DTC03::I2CRequest()
//{
//  unsigned char temp[2], com;
//  unsigned int vact;
//  int itec;
//  bool itecsign=0;
//  unsigned char i;//
//  while(Wire.available()==1) com = Wire.read();
//
//  switch(com)
//  {
//    case I2C_COM_INIT:
//    temp[0] = g_b_lower;
//    temp[1] = g_b_upper;
////    if(g_sensortype) temp[1] |=REQMSK_SENSTYPE;//20161113
//	if(g_mod_status) temp[1] |=REQMSK_SENSTYPE;//20161113
//    break;
//
//    case I2C_COM_CTR:
//    temp[0] = g_currentlim;
//    temp[1] = g_p;
//    break;
//
//    case I2C_COM_VSET:
//    temp[0] = g_vset_limit;
//    temp[1] = g_vset_limit >>8;
//    break;
//
//    case I2C_COM_KI:
//    temp[0] = g_ki;
//    temp[1] = g_ls;
//    break;
//
//    case I2C_COM_KIINDEX://
//    temp[0] = g_kiindex;//
//    break;
//    
//    case I2C_COM_VACT:
//    vact=g_vactavgsum >> VACTAVGPWR;
//    temp[0]=vact;
//    temp[1]=vact >> 8;
//    break;
//
//    case I2C_COM_ITEC_ER:
//    
//    itec = (g_itecavgsum >> ITECAVGPWR)-g_isense0;
//    if(itec<0) itecsign = 1;
//    else itecsign = 0;
//    temp[0]=abs(itec);
//    //temp[1]=abs(itec >> 8);
//    temp[1]=abs(itec) >> 8;//
//    
//    
//    if(g_errcode1)  temp[1] |= REQMSK_ERR1;
//    else temp[1] &= (~REQMSK_ERR1);//20161101 add
//    if(g_errcode2)  temp[1] |= REQMSK_ERR2;
//    else temp[1] &= (~REQMSK_ERR2);//
//    if(itecsign) temp[1]|= REQMSK_ITECSIGN;
//    else temp[1] &= (~REQMSK_ITECSIGN);//
//    //Serial.print(temp[1]);
////    Serial.print(", ");
////    Serial.print(temp[0]);
////    Serial.print(", ");
////    Serial.println(g_itecavgsum);
//    
//    break;
//
//    case I2C_COM_R1R2:
//    temp[0] = g_r1;
//    temp[1] = g_r2;
//    break;
//
//    case I2C_COM_PIDOFFSET:
//    temp[0] = g_tpidoffset;
//    temp[1] = g_vbec2;
//    break;
//
//    case I2C_COM_FBC:
//    temp[0] = g_fbc_base;
//    temp[1] = g_fbc_base>>8;
//    break;
//
//    case I2C_COM_VMOD:
//    temp[0] = g_vmodoffset; //20161101 modified
//    temp[1] = g_vmodoffset>>8; //
//    break;
//  }
//  Wire.write(temp,2);
//}
//void DTC03::I2CReceive()
//{
//  unsigned char temp[2], com, errcodeall, bconst_upper, bconst_lower, vset_upper, vset_lower;
//  unsigned char fbc_lower, fbc_upper, vmodoffset_upper, vmodoffset_lower;
//  unsigned long t1,t2,t_delta;//added
//  while(Wire.available() == 3)
//  {
////    t1=millis();//
//    t1=micros();
//    com=Wire.read();
//    temp[0]=Wire.read();
//    temp[1]=Wire.read();
////    t2=millis();//
//    t2=micros();
//    t_delta=t2-t1;//
//  }
//  g_ee_changed = 1;
//  
// if(t_delta<500) 
// { 
//  switch(com)
//  {
//    case I2C_COM_INIT:
//    g_b_lower = temp[0];
//    g_b_upper = REQMSK_BUPPER & temp[1];
//    g_en_state = REQMSK_ENSTATE & temp[1];
////    g_sensortype = temp[1] & REQMSK_SENSTYPE; //20161113
//	g_mod_status = temp[1] & REQMSK_SENSTYPE; 
//    g_ee_change_state = EEADD_B_lower;
//    //Serial.println("EN");
//    break;
//
//    case I2C_COM_CTR:
//    g_currentlim = temp[0];
//    g_p = temp[1];
//    g_ee_change_state = EEADD_P;
//    //Serial.println("LIM");
//    break;
//
//    case I2C_COM_VSET:
//    vset_lower =  temp[0];
//    vset_upper = temp[1];
//    g_vset_limit = vset_upper<<8 | vset_lower;
//    g_ee_change_state = EEADD_Vset_lower;
//    //Serial.println("SET");
//    break;
//
//    case I2C_COM_KIINDEX://20161101 add
//    g_kiindex = temp[0];//
//    g_ee_change_state = EEADD_KIINDEX;//
//    //Serial.println("INDEX");
//    break;//
//    
//    case I2C_COM_KI:
//    //g_ki = temp[0];
//    //g_ls = temp[1];
//    g_ls = temp[0];//20161104 changed
//    g_ki = temp[1];
//    //Serial.println("KILS");
//    break;
//
//    case I2C_COM_R1R2:
//    g_r1 = temp[0];
//    g_r2 = temp[1];
//    g_ee_change_state = EEADD_R1;
//    //Serial.println("HB");
//    break;
//
//    case I2C_COM_PIDOFFSET:
//    g_tpidoffset = temp[0];
//    g_vbec2 = temp[1];
//    g_ee_change_state = EEADD_PIDOFFSET;
//    //Serial.println("BC");
//    break;
//
//    case I2C_COM_FBC:
//    fbc_lower = temp[0];
//    fbc_upper = temp[1];
//    g_fbc_base =(fbc_upper <<8)|fbc_lower;//20161101
//    g_ee_change_state = EEADD_FBC_base_lower;
//    //Serial.println("FB");
//    break;
//
//    case I2C_COM_VMOD:
//    vmodoffset_lower = temp[0];
//    vmodoffset_upper = temp[1];
//    g_vmodoffset = (vmodoffset_upper << 8)| vmodoffset_lower;
//    g_ee_change_state = EEADD_Vmodoffset_lower;
//    //Serial.println("MOD");
//    break;
//  }
// }
//
//}
//void DTC03::SaveEEPROM()
//{
//  unsigned char vset_lower, vset_upper, fbc_upper, fbc_lower, vmodoffset_upper, vmodoffset_lower;
//  g_ee_changed = 0;
//  switch(g_ee_change_state)
//  {
//    case EEADD_P:
//    EEPROM.write(EEADD_P, g_p);
//    EEPROM.write(EEADD_currentlim, g_currentlim);
//    break;
//
//    //case EEADD_KI:
//    //EEPROM.write(EEADD_KI, g_ki);
//    //EEPROM.write(EEADD_LS, g_ls);
//    //break;
//    
//    case EEADD_KIINDEX://20161101 add
//    EEPROM.write(EEADD_KIINDEX, g_kiindex);//
//    break;//
//
//    case EEADD_Vset_lower:
//    vset_lower = g_vset_limit;
//    vset_upper = g_vset_limit>>8;
//    EEPROM.write(EEADD_Vset_lower, vset_lower);
//    EEPROM.write(EEADD_Vset_upper, vset_upper);
//    break;
//
//    case EEADD_B_lower:
//    EEPROM.write(EEADD_B_lower, g_b_lower);
//    EEPROM.write(EEADD_B_upper, g_b_upper);
//    EEPROM.write(EEADD_Sensor_type, g_sensortype);
//    break;
//
//    case EEADD_R1:
//    EEPROM.write(EEADD_R1, g_r1);
//    EEPROM.write(EEADD_R2, g_r2);
//    break;
//
//    case EEADD_PIDOFFSET:
//    EEPROM.write(EEADD_PIDOFFSET, g_tpidoffset);
//    EEPROM.write(EEADD_VBE_C2, g_vbec2);
//    break;
//
//    case EEADD_FBC_base_lower:
//    fbc_upper = g_fbc_base >>8;
//    fbc_lower = g_fbc_base;
//    EEPROM.write(EEADD_FBC_base_lower, fbc_lower);
//    EEPROM.write(EEADD_FBC_base_upper, fbc_upper);
//    break;
//
//    case EEADD_Vmodoffset_lower:
//    vmodoffset_lower = g_vmodoffset;
//    vmodoffset_upper = g_vmodoffset>>8;
//    EEPROM.write(EEADD_Vmodoffset_lower, vmodoffset_lower);
//    EEPROM.write(EEADD_Vmodoffset_upper, vmodoffset_upper);
//    break;
//
//  }
//}




