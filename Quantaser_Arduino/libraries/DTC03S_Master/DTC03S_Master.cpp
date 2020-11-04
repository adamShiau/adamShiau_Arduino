#include <DTC03S_Master.h>
//P02
DTC03SMaster::DTC03SMaster()
{

}
void DTC03SMaster::SetPinMode()
{
	pinMode(ENC_A, INPUT);
	pinMode(PUSHB, INPUT);
	pinMode(SCANB, INPUT);
	pinMode(ENSW, INPUT);
}
void DTC03SMaster::ParamInit()
{
	digitalWrite(ENC_A, HIGH);
	lcd.Init();
	Wire.begin();
	loopindex = 0;
	g_en_state =0;
	g_scan =0;
	g_timer=0;
	g_tfine = 0;
	g_tenc[0] =0;
	g_tenc[1] =0;
	g_tenc[2] =0;
	p_en[0] = p_en[1] = 0;
	p_scan[0] = p_scan[1] = 0;
	p_tnow_flag[0] = 0;
	p_tnow_flag[1] = 0;
	g_oldcursorstate = 0;
	p_curstatus0flag = 0;
	g_paramterupdate = 0;
	p_rateflag = 0;
	p_rateflag = 0;
	p_loopcount = 0;
	p_EngFlag = 0;
	p_ee_changed = 0;
	p_overshoot_scan = 0;
	p_overshoot_noscan = 0;
	p_overshoot_cancel_Flag_scan = 1;
	p_overshoot_cancel_Flag_noscan = 1;
}
void DTC03SMaster::WelcomeScreen()
{
	lcd.SelectFont(SystemFont5x7);
	lcd.GotoXY(0,0);
	lcd.print("DTC03S Ver.2.01");
	lcd.GotoXY(0,ROWPIXEL0507*1);
	lcd.print("Initializing...");
	for (byte i=9; i>0; i--)
  {
    lcd.GotoXY(0,ROWPIXEL0507*2);
    lcd.print(i);
    delay(1000);
  }
}
void DTC03SMaster::ReadEEPROM()
{
	unsigned char noeedummy, temp_upper, temp_lower;
	noeedummy = EEPROM.read(EEADD_DUMMY);
	if(noeedummy == NOEE_DUMMY)
	{
		g_vstart = EEPROM.read(EEADD_VSTART_UPPER)<<8 | EEPROM.read(EEADD_VSTART_LOWER);
		g_vend = EEPROM.read(EEADD_VEND_UPPER)<<8 | EEPROM.read(EEADD_VEND_LOWER);
		g_rateindex = EEPROM.read(EEADD_RATE_INDEX);
		g_p = EEPROM.read(EEADD_P);
		g_kiindex = EEPROM.read(EEADD_KIINDEX);
		g_fbcbase = EEPROM.read(EEADD_FBC_UPPER)<<8 | EEPROM.read(EEADD_FBC_LOWER);
        g_currentlim = EEPROM.read(EEADD_currentlim);
        g_r1 = EEPROM.read(EEADD_R1);
        g_r2 = EEPROM.read(EEADD_R2);
        g_otp = EEPROM.read(EEADD_TOTP_UPPER)<<8 | EEPROM.read(EEADD_TOTP_LOWER);
        
	}
	else
	{
		EEPROM.write(EEADD_DUMMY, NOEE_DUMMY);
		EEPROM.write(EEADD_VSTART_UPPER, NOEE_VSTART>>8);
		EEPROM.write(EEADD_VSTART_LOWER, NOEE_VSTART);
		EEPROM.write(EEADD_VEND_UPPER, NOEE_VEND>>8);
		EEPROM.write(EEADD_VEND_LOWER, NOEE_DUMMY);
		EEPROM.write(EEADD_RATE_INDEX, NOEE_RATE);
		EEPROM.write(EEADD_FBC_UPPER, NOEE_FBC>>8);
		EEPROM.write(EEADD_FBC_LOWER, NOEE_FBC);
		EEPROM.write(EEADD_P, NOEE_P);
		EEPROM.write(EEADD_KIINDEX, NOEE_kiindex);
		EEPROM.write(EEADD_currentlim, NOEE_ILIM);
		EEPROM.write(EEADD_R1, NOEE_R1);
		EEPROM.write(EEADD_R2, NOEE_R2);
		EEPROM.write(EEADD_TOTP_UPPER, NOEE_TOTP>>8);
		EEPROM.write(EEADD_TOTP_LOWER, NOEE_TOTP);

		g_vstart = NOEE_VSTART;
		g_vend = NOEE_VEND;
		g_rateindex = NOEE_RATE;
		g_p = NOEE_P;
		g_kiindex = NOEE_kiindex;
		g_r1 = NOEE_R1;
		g_r2 = NOEE_R2;
		g_fbcbase = NOEE_FBC;
		g_otp = NOEE_TOTP; 
		g_currentlim = NOEE_ILIM;
	}
	g_vset = g_vstart;
    g_tstart = ReturnTemp(g_vstart, 0);
    g_tnow = g_tstart;
    g_tend = ReturnTemp(g_vend, 0);
	g_trate = pgm_read_word_near(RateTable+g_rateindex);
	p_rate = float(g_trate)*SCANSAMPLERATE/100000.0;
}
void DTC03SMaster::SaveEEPROM() {
	
	if (p_ee_changed==1) {
		p_ee_changed = 0;
		switch(p_ee_change_state){
			
            case EEADD_VSTART_UPPER:
                EEPROM.write(EEADD_VSTART_UPPER, g_vstart>>8 );
                EEPROM.write(EEADD_VSTART_LOWER, g_vstart);
                break;
    
            case EEADD_VEND_UPPER:
                EEPROM.write(EEADD_VEND_UPPER, g_vend>>8 );
                EEPROM.write(EEADD_VEND_LOWER, g_vend );
                break;

            case EEADD_RATE_INDEX:
                EEPROM.write(EEADD_RATE_INDEX, g_rateindex);
                break;
                
            case EEADD_P:
                EEPROM.write(EEADD_P, g_p);
                break;
                
            case EEADD_KIINDEX:
                EEPROM.write(EEADD_KIINDEX, g_kiindex);
                break;
                
            case EEADD_currentlim:
                EEPROM.write(EEADD_currentlim, g_currentlim);
                break;
                
            case EEADD_FBC_UPPER:
                EEPROM.write(EEADD_FBC_UPPER, g_fbcbase>>8);
                EEPROM.write(EEADD_FBC_LOWER, g_fbcbase);
                break;
             
			case EEADD_R1:
                EEPROM.write(EEADD_R1, g_r1);
                break;
               
			case EEADD_R2:
                EEPROM.write(EEADD_R2, g_r2);
                break;
               
			case EEADD_TOTP_UPPER:
                EEPROM.write(EEADD_TOTP_UPPER, g_otp>>8);
                EEPROM.write(EEADD_TOTP_LOWER, g_otp);
                break;             
        }
	}
}
void DTC03SMaster::I2CWriteAll()
{
	for (int i=I2C_COM_INIT; i<=I2C_COM_WAKEUP; i++) I2CWriteData(i);
}

void DTC03SMaster::I2CWriteData(unsigned char com)
{
	unsigned char temp[2];
	temp[0]=0;
    temp[1]=0;
  	switch(com)
  	{
    	case I2C_COM_INIT:
    	temp[0] = BVALUE;
    	temp[1] = (BVALUE >>8);
			if(g_en_state) temp[1] |= REQMSK_ENSTATE;
		break;

    	case I2C_COM_CTR:
    		temp[0]= g_currentlim;
    		temp[1]= g_p;
    	break;

    	case I2C_COM_VSET:
    		temp[0]=g_vset;
    		temp[1]=g_vset>>8;
    	break;

    	case I2C_COM_R1R2:
    		temp[0] = g_r1;
    		temp[1] = g_r2;
    	break;

    	case I2C_COM_FBC:
    		temp[0] = g_fbcbase;
    		temp[1] = g_fbcbase>>8;
    	break;
   
    	case I2C_COM_KI:
    		temp[0]=pgm_read_word_near(kilstable280+g_kiindex*2);
            temp[1]=pgm_read_word_near(kilstable280+g_kiindex*2+1);
    	break;
    	
    	case I2C_COM_OTP:
    		temp[0] = g_otp;
    		temp[1] = g_otp>>8;
    	break;
    	
    	case I2C_COM_WAKEUP:
    		temp[0] = 1;
    		temp[1] = p_overshoot_noscan | p_overshoot_scan;
    	break;
    	
    	case I2C_COM_TEST:
    		temp[0] = p_tlp;
    		temp[1] = p_tlp>>8;
    	break;

  }
  Wire.beginTransmission(DTC03P05);//20161031 add
  Wire.write(com);//
  Wire.write(temp, 2);//
  Wire.endTransmission();
  delayMicroseconds(I2CSENDDELAY);
}
void DTC03SMaster::I2CReadData(unsigned char com)
{
  unsigned char temp[2];
  unsigned int itectemp;
  bool itecsign;
  
  Wire.beginTransmission(DTC03P05);
  Wire.write(com);
  Wire.endTransmission();
  delayMicroseconds(I2CREADDELAY);
  Wire.requestFrom(DTC03P05,2);
  while(Wire.available()==2)
  {
    temp[0] = Wire.read();
    temp[1] = Wire.read();
  }
  switch(com)
  {
    case I2C_COM_VACT:
    g_vact =(temp[1] <<8) | temp[0];
    break;

    case I2C_COM_ITEC_ER:
    itectemp = ((temp[1] & REQMSK_ITECU) << 8)| temp[0];
    itecsign = temp[1] & REQMSK_ITECSIGN;
    g_errcode1 = temp[1] & REQMSK_ERR1;
    g_errcode2 = temp[1] & REQMSK_ERR2;
    if(itecsign) g_itec = (-1)*(int)itectemp;
    else g_itec = (int)itectemp;
    break;

    case I2C_COM_PCB:
    g_tpcb = (temp[1]<<8)|temp[0];
    break;
  }
 delayMicroseconds(I2CSENDDELAY);//20161031
}

void DTC03SMaster::CheckStatus()
{
		float tact, itec_f, tpcb_f;
		
		switch (p_EngFlag) {
			case 0:
				I2CReadData(I2C_COM_VACT);
  	    		tact = ReturnTemp(g_vact,0);
  	    		PrintTact(tact);								
			break;
			case 1:
				if (loopindex%3==0) {
					I2CReadData(I2C_COM_ITEC_ER);
		            itec_f = float(g_itec)*CURRENTRatio;
		            PrintItec(itec_f);
				}				
				if (loopindex%3==1) {
					I2CReadData(I2C_COM_PCB);
		            tpcb_f = float(g_tpcb)/4.0-20.5;
		            PrintTpcb(tpcb_f);
				}
				if (loopindex%3==2) {
					I2CReadData(I2C_COM_VACT);
	  	    		tact = ReturnTemp(g_vact,0);
	  	    		PrintTact(tact);
				}
			break;						
		}
	    loopindex++;		       
}
void DTC03SMaster::RuntestI2C() {
	if (loopindex%500==0) I2CWriteData(I2C_COM_TEST);		
}
float DTC03SMaster::ReturnTemp(unsigned int vact, bool type)
{
  float tact;
  if(type)
    tact = (float)(vact/129.8701) - 273.15;
  else
    tact = 1/(log((float)vact/RTHRatio)/BVALUE+T0INV)-273.15;
  return tact;
}
void DTC03SMaster::PrintBG()
{
	lcd.ClearScreen(0);
	lcd.SelectFont(SystemFont5x7);
	lcd.GotoXY(TSTART_COORD_X, TSTART_COORD_Y);
	lcd.print(Text_Tstart);
	lcd.GotoXY(TEND_COORD_X, TEND_COORD_Y);
	lcd.print(Text_Tstop);
	lcd.GotoXY(RATE_COORD_X, RATE_COORD_Y);
	lcd.print(Text_RATE);
	lcd.GotoXY(EN_COORD_X, EN_COORD_Y);
	lcd.print(Text_CTRL);
}
void DTC03SMaster::PrintEngBG()
{
	lcd.ClearScreen(0);
	lcd.SelectFont(SystemFont5x7);
	lcd.GotoXY(TS_X, TS_Y);
	lcd.print(Text_TS);
	lcd.GotoXY(TA_X, TA_Y);
	lcd.print(Text_TA);
	lcd.GotoXY(Ilim_X, Ilim_Y);
	lcd.print(Text_Ilim);
	lcd.GotoXY(Ic_X, Ic_Y);
	lcd.print(Text_Ic);
	lcd.GotoXY(P_X, P_Y);
	lcd.print(Text_P);
	lcd.GotoXY(I_X, I_Y);
	lcd.print(Text_I);
	lcd.GotoXY(R1_X, R1_Y);
	lcd.print(Text_R1);
	lcd.GotoXY(R2_X, R2_Y);
	lcd.print(Text_R2);
	lcd.GotoXY(FBC_X, FBC_Y);
	lcd.print(Text_FBC);
	lcd.GotoXY(Totp_X, Totp_Y);
	lcd.print(Text_Totp);
	lcd.GotoXY(Tpcb_X, Tpcb_Y);
	lcd.print(Text_Tpcb);
}
void DTC03SMaster::PrintTstart()
{
	lcd.SelectFont(SystemFont5x7);
	if (p_EngFlag == 1) lcd.GotoXY(TS_X2, TS_Y);	
	else lcd.GotoXY(TSTART_COORD_X2, TSTART_COORD_Y);		
	if(g_tstart < 9.993) lcd.print(" ");
    lcd.print(g_tstart,2);
	
}
void DTC03SMaster::PrintTend()
{
	lcd.SelectFont(SystemFont5x7);
	lcd.GotoXY(TEND_COORD_X2, TEND_COORD_Y);
	if(g_tend < 9.991) lcd.print(" ");
	lcd.print(g_tend,2);
	 
}
void DTC03SMaster::PrintTact(float tact)
{
		
	if (p_EngFlag == 1) {
		lcd.SelectFont(SystemFont5x7);
		lcd.GotoXY(TA_X2, TA_Y);
	}
	else {
		lcd.SelectFont(Arial_bold_14);
		lcd.GotoXY(TACT_COORD_X, TACT_COORD_Y);
	}
	if(g_errcode2 == 1) lcd.print("error2");
//	else if (g_errcode1 == 1) lcd.print("error1");
	else {
		if(tact< 9.991) lcd.print(" ");
		lcd.print(tact, 2); 		
	}
	Overshoot_Cancelation(tact);	
}
void DTC03SMaster::PrintRate()
{
	lcd.SelectFont(SystemFont5x7);
	lcd.GotoXY(RATE_COORD_X2, RATE_COORD_Y);
	lcd.print(float(g_trate)/100,2);
}
void DTC03SMaster::PrintScan()
{
	lcd.SelectFont(SystemFont5x7);
	lcd.GotoXY(SCAN_COORD_X, SCAN_COORD_Y);
	if(g_scan) lcd.print("SCAN");
	else lcd.print("STOP");
}
void DTC03SMaster::PrintEnable()
{
	lcd.SelectFont(SystemFont5x7);
	lcd.GotoXY(EN_COORD_X2, SCAN_COORD_Y);
	if(g_en_state==0) lcd.print("OFF");
	else lcd.print(" ON");	
}
void DTC03SMaster::PrintTnow()
{
	lcd.SelectFont(SystemFont5x7);
	lcd.GotoXY(TFINE_COORD_X, TFINE_COORD_Y);
	if (p_tnow_flag[1]) lcd.print(g_tnow+g_tfine);
	else lcd.print("     ");	
}
void DTC03SMaster::PrintP()
{
  lcd.GotoXY(P_X2, P_Y);
  lcd.SelectFont(SystemFont5x7);
  if(g_p<10) lcd.print(" ");
  lcd.print(g_p);
}
void DTC03SMaster::PrintKi()
{
  float tconst;
  tconst = float(pgm_read_word_near(timeconst+g_kiindex))/100.0;
  lcd.GotoXY(I_X2, I_Y);
  lcd.SelectFont(SystemFont5x7);
  if (g_kiindex < 3) {
  	if (g_kiindex==1) lcd.print(" OFF");
  	else lcd.print(tconst,2);
  }   
  else if (g_kiindex < 33){
   lcd.print(" ");
   lcd.print(tconst,1);
  }
  else{
  lcd.print("  ");
  lcd.print(tconst,0);
  }
}
void DTC03SMaster::PrintVfbc()
{
	lcd.SelectFont(SystemFont5x7);
	lcd.GotoXY(FBC_X2, FBC_Y);
	lcd.print(g_fbcbase);
}
void DTC03SMaster::PrintIlim()
{
  float currentlim;
  currentlim =ILIMSTART + ILIMSTEP *(float)(g_currentlim);
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(Ilim_X2,Ilim_Y);
  lcd.print(currentlim,2);
}
void DTC03SMaster::PrintItec(float itec)
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(Ic_X2,Ic_Y);
  if ( abs(itec) <= 0.015 ) itec = 0;
  if(itec <0.00) lcd.print(itec,2); 
  else
   {
     lcd.print(" ");
     lcd.print(itec,2);
   } 
}
void DTC03SMaster::PrintTpcb(float tpcb)
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(Tpcb_X2, Tpcb_Y);
  if (tpcb < 100.0 ) lcd.print(" ");
  lcd.print(tpcb,0);
}
void DTC03SMaster::PrintR1()
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(R1_X2, R1_Y);
  if (g_r1<10) lcd.print(" ");
  lcd.print(g_r1);
}
void DTC03SMaster::PrintR2()
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(R2_X2, R2_Y);
  if (g_r2<10) lcd.print(" ");
  lcd.print(g_r2);
}
void DTC03SMaster::PrintTotp()
{
	float Topt_set;
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(Totp_X2, Totp_Y);
  Topt_set = float(g_otp)/4.0-20.5;
  if (Topt_set < 99.5 ) lcd.print(" ");
  lcd.print(Topt_set,0);
}

unsigned int DTC03SMaster::ReturnVset(float tset, bool type)
{
  unsigned int vset;
  float temp;
  if(type)
    vset = (unsigned int)((tset+273.15)*129.8701);
  else
    vset = (unsigned int)RTHRatio*exp(-1*(float)BVALUE*(T0INV-1/(tset+273.15)));
  return vset;
}
void DTC03SMaster::checkTnowStatus()
{
	// p_en[1] : the most updated enble state  
	if (p_en[0] == 1) {
		if (p_scan[1] < p_scan[0]) p_tnow_flag[1] = 1; // falling edge @ enable = 1 
	}
	else {
		p_tnow_flag[1] = 0;
		g_tfine = 0;
		g_tnow = g_tstart;
	}
	if (p_scan[1]) p_tnow_flag[1] = 0;
	//
	if (p_tnow_flag[0] != p_tnow_flag[1]) { // enter this "if statement" only when p_tnow_flag toggle 
		if (p_tnow_flag[1]==1) {
		    g_cursorstate = 3;
		    p_curstatus0flag = 1;
	    }
		else {
			g_cursorstate = 0;
			g_oldcursorstate = 0;
		}
		PrintTnow();
	}
	p_tnow_flag[0] = p_tnow_flag[1];
	p_en[0] = p_en[1];
	p_scan[0] = p_scan[1];
}

void DTC03SMaster::CalculateRate()
{
	unsigned int t_temp;
	
	t_temp = millis(); 
	if ( (g_en_state==1) && (g_scan==1) && (p_EngFlag==0)) {
		
		if ( (t_temp-p_trate) >= SCANSAMPLERATE ) {			
			
			if(g_tend > g_tstart) 
			{
				g_tnow += p_rate;
				if( (g_tnow+g_tfine) > g_tend) g_tnow = g_tend - g_tfine;								
			}	
		else 
			{
				g_tnow -= p_rate;
				if( (g_tnow+g_tfine) < g_tend) g_tnow = g_tend - g_tfine;				
			}
		p_rateflag = 1;	
	    }
	    p_enableFlag = 1; // change to 1 only when EN ON && Scan ON && ~ENG mode 
	}
	//
//	p_tlp = millis(); // use to check loop time, send p_tlp	
//	I2CWriteData(I2C_COM_TEST);	
	//
	if (p_rateflag == 1) {
		p_rateflag = 0;
		p_trate = t_temp; 
//		I2CWriteData(I2C_COM_TEST);// use to check rate update time
		g_vset = ReturnVset(g_tnow+g_tfine, 0);
	    I2CWriteData(I2C_COM_VSET);
	}
	
	if ( (p_enableFlag==1) && (g_en_state==0) ) {
		p_enableFlag = 0;
		g_vset = ReturnVset(g_tstart, 0);
	    I2CWriteData(I2C_COM_VSET);
	    setKpKiLs(g_tstart);
	}
		
}
void DTC03SMaster::Overshoot_Cancelation(float tact){
	
	checkOvershoot(tact);
	if ( p_overshoot_scan==1 && p_overshoot_cancel_Flag_scan==1 &&  abs(tact-g_tend) > 0.5 ) {
		p_overshoot_cancel_Flag_scan = 0;
		setKpKiLs(g_tend);
	    I2CWriteData(I2C_COM_WAKEUP);			
	}
	if ( p_overshoot_noscan==1 && p_overshoot_cancel_Flag_noscan==1 &&  abs(tact-g_tstart) > 0.5 ) {
		p_overshoot_cancel_Flag_noscan = 0;		
		I2CWriteData(I2C_COM_WAKEUP);
	}	

	
}
void DTC03SMaster::checkOvershoot(float tact) {
	if(g_en_state==1) {
		if (g_scan == 1) {
			if ( abs( tact-g_tend )<0.1 ) p_overshoot_scan=1 ;
		}
		else {
			if ( abs(tact-g_tstart)<0.1 ) p_overshoot_noscan=1 ;
		}		
	}
}

void DTC03SMaster::setKpKiLs(float tin) {
	if ( (tin>6.99) && (tin<=14.99) ) {
		g_p = 3;
		I2CWriteData(I2C_COM_CTR);
		g_kiindex=25; //Time constamt: 3.5s		        
		I2CWriteData(I2C_COM_KI);
	}
	else if( tin < 22.49) {
		g_p = 6;
		I2CWriteData(I2C_COM_CTR);
		g_kiindex=22; //Time constamt: 2s		        
		I2CWriteData(I2C_COM_KI);		
	}
//	else if( tin < 22.49) {
//		g_p = 9;
//		I2CWriteData(I2C_COM_CTR);
//		g_kiindex=14; //Time constamt: 1.2s		        
//		I2CWriteData(I2C_COM_KI);		
//	}
	else if( tin < 24.99) {
		g_p = 11;
		I2CWriteData(I2C_COM_CTR);
		g_kiindex=13; //Time constamt: 1.1s		        
		I2CWriteData(I2C_COM_KI);		
	}
	else {
		g_p = 13;
		I2CWriteData(I2C_COM_CTR);
		g_kiindex=12; //Time constamt: 1s		        
		I2CWriteData(I2C_COM_KI);
	}
}

void DTC03SMaster::UpdateEnable()//20161101
{
	if(analogRead(ENSW)>ANAREADVIH) p_en[1] = 1;
	else {
		p_en[1] = 0;
		p_overshoot_scan = 0;
		p_overshoot_noscan = 0;
		p_overshoot_cancel_Flag_scan = 1;
		p_overshoot_cancel_Flag_noscan = 1;
	}
	if(g_en_state != p_en[1])
	{
		
		g_en_state = p_en[1];
		I2CWriteData(I2C_COM_INIT);
		PrintEnable();
	}
}
void DTC03SMaster::CheckScan()
{
	unsigned long t_temp;
	if(digitalRead(SCANB)==0) 
	{
		t_temp = millis();
		if ((t_temp - g_tscan) > 500 ) {
		g_scan = !g_scan;
		p_scan[1] = g_scan;
		PrintScan();
		}
	}
	g_tscan = t_temp;	
}
void DTC03SMaster::CursorState()
{
	unsigned long t_temp;
	
	if(analogRead(PUSHB)< ANAREADVIL)
	{
		t_temp = millis();
		if ( (t_temp-g_tpush) > 1000 ) g_cursorstate += 1 ;	
				
		if(g_cursorstate ==3 || g_cursorstate ==4) g_cursorstate =0;
		g_oldcursorstate = g_cursorstate;
		if ( g_tstart<7.01 && g_en_state==0 && g_scan==1 && g_trate==1 && p_EngFlag==0 ) {
			g_cursorstate = 5;	
			p_EngFlag = 1;		
			PrintEngBG();
			PrintTstart();
			PrintP();
			PrintKi();
			PrintIlim();
			PrintVfbc();
			PrintR1();
			PrintR2();
			PrintTotp();
		}	

        if( (g_cursorstate==14) && g_scan) g_cursorstate = 6;
        if( (g_cursorstate==14) && ~g_scan) {
        	g_cursorstate =0;
        	p_EngFlag = 0;
			PrintBG();
			PrintTstart();
			PrintTend();
			PrintRate();
			PrintEnable();
			PrintScan();     	
		}
	}
	g_tpush = t_temp;
}

void DTC03SMaster::ShowCursor()
{
	switch(g_cursorstate)
	{
		case 0:
			lcd.SelectFont(SystemFont5x7,BLACK);
			if (p_curstatus0flag) {
				p_curstatus0flag = 0;
				lcd.GotoXY(TFINE_COORD_X-COLUMEPIXEL0507, TFINE_COORD_Y);
			}
			else lcd.GotoXY(RATE_COORD_X-COLUMEPIXEL0507, RATE_COORD_Y);
			lcd.print(" ");
			lcd.SelectFont(SystemFont5x7, WHITE);
			lcd.GotoXY(TSTART_COORD_X-COLUMEPIXEL0507, TSTART_COORD_Y);
			lcd.print(" ");
		break;

		case 1:
			lcd.SelectFont(SystemFont5x7,BLACK);
			lcd.GotoXY(TSTART_COORD_X-COLUMEPIXEL0507, TSTART_COORD_Y);
			lcd.print(" ");
			lcd.SelectFont(SystemFont5x7, WHITE);
			lcd.GotoXY(TEND_COORD_X-COLUMEPIXEL0507, TEND_COORD_Y);
			lcd.print(" ");
		break;

		case 2:
			lcd.SelectFont(SystemFont5x7,BLACK);
			lcd.GotoXY(TEND_COORD_X-COLUMEPIXEL0507, TEND_COORD_Y);
			lcd.print(" ");
			lcd.SelectFont(SystemFont5x7, WHITE);
			lcd.GotoXY(RATE_COORD_X-COLUMEPIXEL0507, RATE_COORD_Y);
			lcd.print(" ");
		break;
		case 3:
			lcd.SelectFont(SystemFont5x7,BLACK);
			switch (g_oldcursorstate){
				case 0 :
					lcd.GotoXY(TSTART_COORD_X-COLUMEPIXEL0507, TSTART_COORD_Y);
			        break;
			    case 1 :
					lcd.GotoXY(TEND_COORD_X-COLUMEPIXEL0507, TEND_COORD_Y);
			        break;
			    case 2 :
					lcd.GotoXY(RATE_COORD_X-COLUMEPIXEL0507, RATE_COORD_Y);
			        break;					
			}
			lcd.print(" ");
			lcd.SelectFont(SystemFont5x7, WHITE);
			lcd.GotoXY(TFINE_COORD_X-COLUMEPIXEL0507, TFINE_COORD_Y);
			lcd.print(" ");
		break;

		case 6: //vset
			lcd.SelectFont(SystemFont5x7,BLACK);
			lcd.GotoXY(Totp_X-COLUMEPIXEL0507, Totp_Y);			
			lcd.print(" ");
			lcd.SelectFont(SystemFont5x7, WHITE);
			lcd.GotoXY(TS_X-COLUMEPIXEL0507, TS_Y);
			lcd.print(" ");
		break;
		case 7: //Ilim
			lcd.SelectFont(SystemFont5x7,BLACK);
			lcd.GotoXY(TS_X-COLUMEPIXEL0507, TS_Y);
			lcd.print(" ");
			lcd.SelectFont(SystemFont5x7, WHITE);
			lcd.GotoXY(Ilim_X-COLUMEPIXEL0507, Ilim_Y);
			lcd.print(" ");
		break;		
		case 8: //p
			lcd.SelectFont(SystemFont5x7,BLACK);
			lcd.GotoXY(Ilim_X-COLUMEPIXEL0507, Ilim_Y);
			lcd.print(" ");
			lcd.SelectFont(SystemFont5x7, WHITE);
			lcd.GotoXY(P_X-COLUMEPIXEL0507, P_Y);
			lcd.print(" ");
		break;
		case 9: //ki
			lcd.SelectFont(SystemFont5x7,BLACK);
			lcd.GotoXY(P_X-COLUMEPIXEL0507, P_Y);
			lcd.print(" ");
			lcd.SelectFont(SystemFont5x7, WHITE);
			lcd.GotoXY(I_X-COLUMEPIXEL0507, I_Y);
			lcd.print(" ");
		break;
		
		case 10: //R1
			lcd.SelectFont(SystemFont5x7,BLACK);
			lcd.GotoXY(I_X-COLUMEPIXEL0507, I_Y);
			lcd.print(" ");
			lcd.SelectFont(SystemFont5x7, WHITE);
			lcd.GotoXY(R1_X-COLUMEPIXEL0507, R1_Y);
			lcd.print(" ");
		break;
		case 11: //R2
			lcd.SelectFont(SystemFont5x7,BLACK);
			lcd.GotoXY(R1_X-COLUMEPIXEL0507, R1_Y);
			lcd.print(" ");
			lcd.SelectFont(SystemFont5x7, WHITE);
			lcd.GotoXY(R2_X-COLUMEPIXEL0507, R2_Y);
			lcd.print(" ");
		break;
		case 12: //fbcbase
			lcd.SelectFont(SystemFont5x7,BLACK);
			lcd.GotoXY(R2_X-COLUMEPIXEL0507, R2_Y);
			lcd.print(" ");
			lcd.SelectFont(SystemFont5x7, WHITE);
			lcd.GotoXY(FBC_X-COLUMEPIXEL0507, FBC_Y);
			lcd.print(" ");
		break;
		case 13: //Totp set
			lcd.SelectFont(SystemFont5x7,BLACK);
			lcd.GotoXY(FBC_X-COLUMEPIXEL0507, FBC_Y);
			lcd.print(" ");
			lcd.SelectFont(SystemFont5x7, WHITE);
			lcd.GotoXY(Totp_X-COLUMEPIXEL0507, Totp_Y);
			lcd.print(" ");
		break;
	}
}
void DTC03SMaster::UpdateParam()
{
	unsigned long t1;

	if(g_paramterupdate)
	{
		g_paramterupdate =0;
		p_ee_changed = 1;
		switch(g_cursorstate)
		{
			case 0:
				g_tstart += g_counter2*0.01;
				g_tnow = g_tstart;
				if(g_tstart > 60.00) g_tstart =60.00;
				if(g_tstart < 7.00) g_tstart = 7.00;
				if(g_en_state==0 ){ // EN switch OFF
					g_vset = ReturnVset(g_tstart, 0);
					I2CWriteData(I2C_COM_VSET);
				}
				else{ // EN switch ON
					if (g_scan==0 ) { // Scan OFF
						g_vset = ReturnVset(g_tnow+g_tfine, 0);
						I2CWriteData(I2C_COM_VSET);
					}
				}
				setKpKiLs(g_tstart);
				PrintTstart();
				g_vstart = ReturnVset(g_tstart, 0);
				p_ee_change_state = EEADD_VSTART_UPPER;
			break;

			case 1:
				g_tend += g_counter2*0.01;
				if(g_tend > 60.00) g_tend = 60.00;
				if(g_tend< 7.00) g_tend = 7.00;
				PrintTend(); 
				g_vend = ReturnVset(g_tend, 0);
				p_ee_change_state = EEADD_VEND_UPPER;
			break;

			case 2: 
				g_rateindex +=g_counter;
				if(g_rateindex <1) g_rateindex =1;
				if(g_rateindex > MAXRATEINDEX) g_rateindex = MAXRATEINDEX;				
				g_trate = pgm_read_word_near(RateTable+g_rateindex);
				p_rate = float(g_trate)*SCANSAMPLERATE/100000.0; // g_trate/100 : k/s, SCANSAMPLERATE/1000 convertion from ms to s
				PrintRate(); 
				p_ee_change_state = EEADD_RATE_INDEX;
			break;
			case 3:
				g_tfine += g_counter*0.01;	
				if (g_tfine > FINETUNEAMP) g_tfine = FINETUNEAMP;
				if (g_tfine < -FINETUNEAMP) g_tfine = -FINETUNEAMP;
				g_vset = ReturnVset(g_tnow+g_tfine, 0);
				I2CWriteData(I2C_COM_VSET);
				PrintTnow() ;
			break;
			
			case 6:
				g_tstart += g_counter2*0.01;
				if(g_tstart > 60.00) g_tstart =60.00;
				if(g_tstart < 7.00) g_tstart = 7.00;
				g_vset = ReturnVset(g_tstart, 0);
				I2CWriteData(I2C_COM_VSET);
				PrintTstart();
				g_vstart = ReturnVset(g_tstart, 0);
				p_ee_change_state = EEADD_VSTART_UPPER;
				
            break;
            
            case 7:
            	g_currentlim += g_counter;
				if(g_currentlim>50) g_currentlim=50; //20161031
		        if(g_currentlim<1) g_currentlim=1;//		        
		        I2CWriteData(I2C_COM_CTR);
		        PrintIlim();
				p_ee_change_state = EEADD_currentlim;				
			break;
			
			case 8:
				g_p += g_counter;
				if(g_p>99) g_p=99;
                if(g_p<1) g_p=1;             
                I2CWriteData(I2C_COM_CTR);
                PrintP();
                p_ee_change_state = EEADD_P;
            break;
			
			case 9:
				g_kiindex += g_counter;
				if(g_kiindex>50) g_kiindex=50;
		        if(g_kiindex<1) g_kiindex=1;		        
		        I2CWriteData(I2C_COM_KI);
		        PrintKi();
		        p_ee_change_state = EEADD_KIINDEX;
			break;
	
			case 10:				
                g_r1 += g_counter;
                if(g_r1>30) g_r1=30; // R1, 1~30 for 0.1~3.0 ohm set 
                if(g_r1<1) g_r1=1;//
                I2CWriteData(I2C_COM_R1R2);
                PrintR1();
                p_ee_change_state = EEADD_R1;
				
			break;
			
			case 11:				
                g_r2 += g_counter;
                if(g_r2>30) g_r2=30;
                if(g_r2<1) g_r2=1;//R2, 1~30 for 1.0~3.0 ohm set 
                I2CWriteData(I2C_COM_R1R2);
                PrintR2();
				p_ee_change_state = EEADD_R2;
			break;
			
			case 12:
				g_fbcbase += g_counter2;
				if(g_fbcbase>44900) g_fbcbase=44900;//
                if(g_fbcbase<16100) g_fbcbase=16100;//
                I2CWriteData(I2C_COM_FBC);
                PrintVfbc();
				p_ee_change_state = EEADD_FBC_UPPER;
			break;
			
			case 13:
				g_otp += (g_counter*4);
				if (g_otp < 281) g_otp = 281; // 50C
				if (g_otp > 561) g_otp = 561; //120C
				I2CWriteData(I2C_COM_OTP);
                PrintTotp();
                p_ee_change_state = EEADD_TOTP_UPPER;
			break;				
		}

	}
}
void DTC03SMaster::Encoder() // use rising edge triger of ENC_B
{
	unsigned char encoded, sum;
	unsigned int dt;
	unsigned long tenc;
	bool MSB, LSB;
	tenc = millis();
	
	if (( abs(tenc - g_tenc[0])) > COUNTERINCRE ) {
		g_tenc[1] = 0;
		g_tenc[2] = 0;
	}
	else g_tenc[1] = tenc - g_tenc[0];
	g_tenc[2] += g_tenc[1];
	 
	MSB = digitalRead(ENC_B);
	LSB = digitalRead(ENC_A);
	encoded = (MSB <<1)| LSB;
    if(encoded == 0b10)
	{
		g_paramterupdate =1;
		g_counter =-1;
		if (g_tenc[2] > COUNTERSPEEDUP) g_counter2 = -50;
		else g_counter2 =-1;		
	}
	else if(encoded == 0b11)
	{
		g_paramterupdate =1;
		g_counter =1;
		if (g_tenc[2] > COUNTERSPEEDUP) g_counter2 =50;
		else g_counter2 = 1;	
	}
	g_tenc[0] = tenc;	
}


