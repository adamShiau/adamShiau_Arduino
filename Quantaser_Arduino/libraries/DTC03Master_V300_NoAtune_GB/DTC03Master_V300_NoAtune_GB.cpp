/* 
	09/19/2017
*/
#include <DTC03Master_V300_NoAtune_GB.h>
DTC03Master::DTC03Master()
{
}
void DTC03Master::SetPinMode()
{
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
//  pinMode(PUSHB, INPUT);
//  pinMode(ENSW, INPUT);
}
void DTC03Master::ParamInit()
{
  digitalWrite(ENC_A, HIGH);
  digitalWrite(ENC_B, HIGH);
  
  Wire.begin();
  lcd.Init();
  g_paramupdate = 0;
  g_sensortype=0;
  g_tsetstep = 1.00;
  g_en_state = 0;
  g_countersensor = 0;
  g_atune_status = 0;
  g_cursorstate=1;
  p_cursorStateCounter[0]=0;
  p_cursorStateCounter[1]=0;
  p_cursorStateCounter[2]=0;
  p_cursorStayTime=0;
  p_tBlink=0;
  p_tBlink_toggle=0;
  p_engModeFlag=0;
  p_blinkTsetCursorFlag=0;
  p_loopindex=0;
  p_ee_changed=0;
  p_holdCursorTimer=0;
  p_HoldCursortateFlag=0;
  g_wakeup = 1;
  p_vact_MV_sum=0;
  p_mvindex=0;
  p_keyflag = 0;
  g_atunDone = 0;
  p_atunProcess_flag = 0;
  g_LCDlock_flag = 0;
  g_kpkiFromAT = 0;
  for (int i=0;i<MVTIME;i++) p_vact_array[i]=0;
}
void DTC03Master::WelcomeScreen()
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(0,0);
//  lcd.print("DTC03 Ver.3.01");
  lcd.print("DTC03 Ver.3.03"); // 3.02 for autotune
  lcd.GotoXY(0,ROWPIXEL0507*1);
  lcd.print("Initializing");
  for (byte i=5; i>0; i--)
  {
    lcd.GotoXY(COLUMNPIXEL0507*(12+1),ROWPIXEL0507*1);
    lcd.print(i);
    delay(1000);
  }
  lcd.ClearScreen(0);//0~255 means ratio of black  
}
void DTC03Master::ReadEEPROM()
{
	unsigned char noeedummy, temp_upper, temp_lower;
	noeedummy = EEPROM.read(EEADD_DUMMY);
	if(noeedummy == NOEE_DUMMY)
	{
		g_vset = EEPROM.read(EEADD_VSET_UPPER)<<8 | EEPROM.read(EEADD_VSET_LOWER);
		g_currentlim = EEPROM.read(EEADD_currentlim);
		g_p = EEPROM.read(EEADD_P);
		g_kiindex = EEPROM.read(EEADD_KIINDEX);		
		g_bconst=EEPROM.read(EEADD_BCONST_UPPER)<<8 | EEPROM.read(EEADD_BCONST_LOWER);		
		g_mod_status = EEPROM.read(EEADD_MODSTATUS);
        g_r1 = EEPROM.read(EEADD_R1);
        g_r2 = EEPROM.read(EEADD_R2);
        g_tpidoff = EEPROM.read(EEADD_TPIDOFF);
        g_fbcbase = EEPROM.read(EEADD_FBC_UPPER)<<8 | EEPROM.read(EEADD_FBC_LOWER);
        g_vmodoffset = EEPROM.read(EEADD_MODOFF_UPPER)<<8 |  EEPROM.read(EEADD_MODOFF_LOWER);
        g_Rmeas = EEPROM.read(EEADD_RMEAS_UPPER)<<8 | EEPROM.read(EEADD_RMEAS_LOWER); 
        g_otp = EEPROM.read(EEADD_TOTP_UPPER)<<8 | EEPROM.read(EEADD_TOTP_LOWER);
        g_p_atune = EEPROM.read(EEADD_PAP);
        g_T_atune = EEPROM.read(EEADD_TBIAS);
        g_stableCode_atune = EEPROM.read(EEADD_ATSTABLE);
	}
	else
	{
		EEPROM.write(EEADD_DUMMY, NOEE_DUMMY);
		EEPROM.write(EEADD_VSET_UPPER, NOEE_VSET>>8);
		EEPROM.write(EEADD_VSET_LOWER, NOEE_VSET);
		EEPROM.write(EEADD_currentlim, NOEE_ILIM);		
		EEPROM.write(EEADD_P, NOEE_P);		
		EEPROM.write(EEADD_KIINDEX, NOEE_kiindex);
		EEPROM.write(EEADD_BCONST_UPPER, NOEE_BCONST>>8);
		EEPROM.write(EEADD_BCONST_LOWER, NOEE_BCONST);
		EEPROM.write(EEADD_MODSTATUS, NOEE_MODSTATUS);
		EEPROM.write(EEADD_R1, NOEE_R1);
		EEPROM.write(EEADD_R2, NOEE_R2);		
		EEPROM.write(EEADD_TPIDOFF, NOEE_TPIDOFF);
		EEPROM.write(EEADD_FBC_UPPER, NOEE_FBC>>8);
		EEPROM.write(EEADD_FBC_LOWER, NOEE_FBC);
		EEPROM.write(EEADD_MODOFF_UPPER, NOEE_MODOFF>>8);
		EEPROM.write(EEADD_MODOFF_LOWER, NOEE_MODOFF);		
		EEPROM.write(EEADD_RMEAS_UPPER, NOEE_RMEAS>>8);
		EEPROM.write(EEADD_RMEAS_LOWER, NOEE_RMEAS);
		EEPROM.write(EEADD_TOTP_UPPER, NOEE_TOTP>>8);
		EEPROM.write(EEADD_TOTP_LOWER, NOEE_TOTP);
		EEPROM.write(EEADD_PAP, NOEE_PAP);
		EEPROM.write(EEADD_TBIAS, NOEE_TBIAS);
		EEPROM.write(EEADD_ATSTABLE, NOEE_ATSTABLE);

		g_vset = NOEE_VSET;
		g_currentlim = NOEE_ILIM;
		g_p = NOEE_P;
		g_p_atune = NOEE_PAP;
		g_kiindex = NOEE_kiindex;
		g_bconst = NOEE_BCONST;
		g_mod_status = NOEE_MODSTATUS;
		g_r1 = NOEE_R1;
		g_r2 = NOEE_R2;
		g_tpidoff = NOEE_TPIDOFF;		
		g_fbcbase = NOEE_FBC;
		g_vmodoffset = NOEE_MODOFF;
		g_Rmeas = NOEE_RMEAS;
		g_otp = NOEE_TOTP; 
		g_T_atune = NOEE_TBIAS;
		g_stableCode_atune = NOEE_ATSTABLE;
	}	
    g_tset = ReturnTemp(g_vset, 0); 
   
}
void DTC03Master::SaveEEPROM() {	
	if (p_ee_changed==1) {
		p_ee_changed = 0;
		switch(p_ee_change_state){
			
            case EEADD_VSET_UPPER:
                EEPROM.write(EEADD_VSET_UPPER, g_vset>>8 );
                EEPROM.write(EEADD_VSET_LOWER, g_vset);
                break;
    
            case EEADD_BCONST_UPPER:
                EEPROM.write(EEADD_BCONST_UPPER, g_bconst>>8 );
                EEPROM.write(EEADD_BCONST_LOWER, g_bconst); 
                break;

            case EEADD_MODSTATUS:
                EEPROM.write(EEADD_MODSTATUS, g_mod_status);
                break;
                
            case EEADD_currentlim:
                EEPROM.write(EEADD_currentlim, g_currentlim);
                break;
                
            case EEADD_FBC_UPPER:
                EEPROM.write(EEADD_FBC_UPPER, g_fbcbase>>8);
                EEPROM.write(EEADD_FBC_LOWER, g_fbcbase);
                break;
                
            case EEADD_P:
                EEPROM.write(EEADD_P, g_p);
                break;
                
            case EEADD_KIINDEX:
                EEPROM.write(EEADD_KIINDEX, g_kiindex );
                break;
             
			case EEADD_TOTP_UPPER:
                EEPROM.write(EEADD_TOTP_UPPER, (g_otp>>8));
                EEPROM.write(EEADD_TOTP_LOWER, g_otp);
                break;
               
			case EEADD_R1:
                EEPROM.write(EEADD_R1, g_r1);
                break;
               
			case EEADD_R2:
                EEPROM.write(EEADD_R2, g_r2);
                break;  
                
			case EEADD_TPIDOFF:
                EEPROM.write(EEADD_TPIDOFF, g_tpidoff);
                break;
			case EEADD_MODOFF_UPPER:
                EEPROM.write(EEADD_MODOFF_UPPER, g_vmodoffset >>8);
                EEPROM.write(EEADD_MODOFF_LOWER, g_vmodoffset );
                break;
                
			case EEADD_RMEAS_UPPER:
                EEPROM.write(EEADD_RMEAS_UPPER, g_Rmeas >>8);
                EEPROM.write(EEADD_RMEAS_LOWER, g_Rmeas );
                break;    
				
			case EEADD_PAP:
                EEPROM.write(EEADD_PAP, g_p_atune);
                break;      
				
			case EEADD_TBIAS:
                EEPROM.write(EEADD_TBIAS, g_T_atune);
                break;   
                
            case EEADD_ATSTABLE:
                EEPROM.write(EEADD_ATSTABLE, g_stableCode_atune);
                break;  
        }
	}
}
void DTC03Master::PrintTestValue()
{
	lcd.SelectFont(SystemFont5x7);
    lcd.GotoXY(Test1_COORD_X,Test1_COORD_Y);
    lcd.print(g_atunDone);
    lcd.print(g_runTimeflag);
    lcd.print(g_DBRflag);

}
void DTC03Master::CheckStatus()
{
		float tact, itec_f, tpcb_f;
				if (p_loopindex%300==0) {
					I2CReadData(I2C_COM_ITEC_ER);
		            itec_f = float(g_itec)*CURRENTRatio;
		            if(!p_engModeFlag) PrintItec(itec_f);
                if(!g_wakeup) I2CWriteAll();
				}								
				if (p_loopindex%300==1) {
					I2CReadData(I2C_COM_VACT);
					vact_MV();
					if (MV_STATUS) tact = ReturnTemp(g_vact_MV,0);
	  	    		else tact = ReturnTemp(g_vact,0);
	  	    		if(!p_engModeFlag) PrintTact(tact);
				}	
				if (p_loopindex%300==2) {
					I2CReadData(I2C_COM_PCB);
		            tpcb_f = float(g_tpcb)/4.0-20.5;
		            if(p_engModeFlag && (g_cursorstate > 9) ) PrintTpcb(tpcb_f);
				}	
				if (p_loopindex%300==3) {
					I2CReadData(I2C_COM_ATUN);
				
		            if(g_atunDone) 
					{	            	
		            	I2CReadData(I2C_COM_ATKpKi);
		            	PrintAtuneDone();
					}
				}
//				if(p_loopindex%300==4) 
//				{
//					PrintTestValue();
//				}
	    p_loopindex++;		       
}
void DTC03Master::vact_MV()
{
	p_vact_MV_sum -= p_vact_array[p_mvindex];
	p_vact_array[p_mvindex] = g_vact;
	p_vact_MV_sum += p_vact_array[p_mvindex];
	g_vact_MV = p_vact_MV_sum>>MVTIME_POWER;
	p_mvindex++;
	if(p_mvindex==MVTIME) p_mvindex=0;
}
void DTC03Master::I2CWriteAll()
{
	for (int i=I2C_COM_INIT; i<=I2C_COM_WAKEUP; i++) I2CWriteData(i);
}
void DTC03Master::I2CWriteData(unsigned char com)
{
  unsigned char temp[2];
  switch(com)
  {
    case I2C_COM_INIT:
        temp[0]= g_bconst - BCONSTOFFSET;
        temp[1]= (g_bconst - BCONSTOFFSET) >> 8;
        if(g_en_state) temp[1] |= REQMSK_ENSTATE; //B10000000
//    if(g_sensortype) temp[1]|= REQMSK_SENSTYPE; 
	    if(g_mod_status) temp[1]|= REQMSK_SENSTYPE; //B01000000   
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

    case I2C_COM_TPIDOFF:
        temp[0] = g_tpidoff;
        temp[1] = 0;
        break;

    case I2C_COM_FBC:
        temp[0] = g_fbcbase;
        temp[1] = g_fbcbase>>8;
        break;

    case I2C_COM_VMOD:
        temp[0] = g_vmodoffset;
        temp[1] = g_vmodoffset >>8;
        break;
    
    case I2C_COM_KI:
        temp[0]=pgm_read_word_near(kilstable230+g_kiindex*2);
        temp[1]=pgm_read_word_near(kilstable230+g_kiindex*2+1);
        break;
    
    case I2C_COM_RMEAS:
    	temp[0]=g_Rmeas;
    	temp[1]=g_Rmeas>>8;
    	break;
    	
    case I2C_COM_OTP:
    		temp[0] = g_otp;
    		temp[1] = g_otp>>8;
    	break;
    	
    case I2C_COM_ATUN:
    	    temp[0] = (g_T_atune << 1) | g_atune_status;
    	    temp[1] = g_p_atune;
    	break;
    	
    case I2C_COM_WAKEUP:
    		temp[0] = 1;
    		temp[1] = 0; // overshoot cancelation, set 0 in DTC03
    	break;
    	
    case I2C_COM_ATSTABLE:
    	    temp[0] = g_stableCode_atune;
    	break;	
      
    case I2C_COM_TEST1:
        temp[0]=p_temp;
        temp[1]=p_temp>>8;
        break;
    
    case I2C_COM_TEST2:
        temp[0]=g_cursorstate;
        temp[1]=p_tBlink_toggle;
        break;

  }
  Wire.beginTransmission(DTC03P05);//
  Wire.write(com);//
  Wire.write(temp, 2);//
  Wire.endTransmission();//
  delayMicroseconds(I2CSENDDELAY);//
}
void DTC03Master::I2CReadData(unsigned char com)
{
  unsigned char temp[2], b_upper, b_lower;
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
        if(itectemp<=1) itectemp=0;
        itecsign = temp[1] & REQMSK_ITECSIGN;
        g_errcode1 = temp[1] & REQMSK_ERR1;
        g_errcode2 = temp[1] & REQMSK_ERR2;
        g_wakeup = temp[1] & REQMSK_WAKEUP;
        if(itecsign) g_itec = (-1)*(int)itectemp;
        else g_itec = (int)itectemp;
        break;
        
    case I2C_COM_PCB:
        g_tpcb = (temp[1]<<8)|temp[0];
        break;
        
    case I2C_COM_ATUN:
    	g_runTimeflag = temp[0] & REQMSK_ATUNE_RUNTIMEERR;
    	g_atunDone = temp[0] & REQMSK_ATUNE_DONE;  	
    	g_DBRflag = temp[0] & REQMSK_ATUNE_DBR;
    	break;
    case I2C_COM_ATKpKi:
    	g_p = temp[0];   	
    	g_kiindex = temp[1];
    	g_paramupdate = 1;
    	g_cursorstate = 3;
    	g_kpkiFromAT = 1;
    	break;
    
  }
}

float DTC03Master::ReturnTemp(unsigned int vact, bool type)
{
  float tact;
  if(type)
    tact = (float)(vact/129.8701) - 273.15;
  else
    tact = 1/(log((float)vact/RTHRatio)/(float)g_bconst+T0INV)-273.15;
  return tact;
}
unsigned int DTC03Master::ReturnVset(float tset, bool type)
{
  unsigned int vset;
  float temp;
  if(type)
    vset = (unsigned int)((tset+273.15)*129.8701);
  else
    vset = (unsigned int)RTHRatio*exp(-1*(float)g_bconst*(T0INV-1/(tset+273.15)));
  return vset;
}
void DTC03Master::BackGroundPrint()
{
  lcd.SelectFont(Iain5x7);
  lcd.GotoXY(TSET_COORD_X,TSET_COORD_Y);
  lcd.print(Text_SET);
  lcd.GotoXY(TACT_COORD_X,TACT_COORD_Y);    
  lcd.print(Text_ACT);
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(ITEC_COORD_X, ITEC_COORD_Y);
  lcd.print(Text_ITEC);
  lcd.GotoXY(ILIM_COORD_X, ILIM_COORD_Y);
  lcd.print(Text_ILIM);
  lcd.GotoXY(P_COORD_X,P_COORD_Y);
  lcd.print(Text_P);
  lcd.GotoXY(I_COORD_X,I_COORD_Y);
  lcd.print(Text_I);  
  lcd.GotoXY(BCONST_COORD_X, BCONST_COORD_Y);
  lcd.print(Text_B);
  lcd.GotoXY(VMOD_COORD_X, VMOD_COORD_Y);
  lcd.print(Text_MS);
//  lcd.GotoXY(ATUNE_COORD_X, ATUNE_COORD_Y);
//  lcd.print(Text_AT);
}
void DTC03Master::PrintNormalAll()
{
	PrintTset();
	PrintIlim();
	PrintP();
	PrintKi();
	PrintB();
	PrintModStatus();
//	PrintAtune();
	//No need to add print Itec and Vact here, checkstatus() will do this
}

void DTC03Master::PrintTset()
{
  lcd.SelectFont(fixed_bold10x15);
  lcd.GotoXY(TSET_COORD_X2,TSET_COORD_Y);
  if(g_tset<= -10.000)
    lcd.print("");
  else if(g_tset<0.000)
    lcd.print(" ");
  else if(g_tset<10.000)
    lcd.print("  ");
  else if(g_tset<100.000)
    lcd.print(" ");
  
  lcd.print(g_tset,3);
}
void DTC03Master::PrintTact(float tact)
{

  lcd.SelectFont(Arial_bold_14);
  lcd.GotoXY(TACT_COORD_X2,TACT_COORD_Y);
  if(g_errcode1) 
    {
      lcd.print("_error1");
      return;
    }
    if(g_errcode2)
    {
      lcd.print("_error2");
      return;
    }

  if(tact<=0.000)
   {
    if(abs(tact)<10.000)
      lcd.print("  ");
    else
      lcd.print(" ");
    lcd.print(tact,3);
   }
  else
  {
   if(tact<10.000)
    lcd.print("   ");
   else if (tact<100.000)
    lcd.print("  ");
   else
    lcd.print(" ");

    lcd.print(tact,3);
  }
  lcd.print(" ");
}
void DTC03Master::PrintItec(float itec)
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(ITEC_COORD_X2,ITEC_COORD_Y);
//  if ( abs(itec) <= 0.015 ) itec = 0;
  if(itec <0.00) lcd.print(itec,2); 

  else
   {
     lcd.print(" ");
     lcd.print(itec,2);
   } 
}
void DTC03Master::PrintIlim()
{
  float currentlim;
  currentlim =ILIMSTART + ILIMSTEP *(float)(g_currentlim);
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(ILIM_COORD_X2,ILIM_COORD_Y);
//  lcd.print(" ");
  lcd.print(currentlim,2);
}
void DTC03Master::PrintP()
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(P_COORD_X2, P_COORD_Y );
  if(!g_runTimeflag)
  {
  	if(g_p<10) lcd.print("  ");  
    else if (g_p<100) lcd.print(" ");      
    lcd.print(g_p);
  }
  else
  {
  	lcd.print("RTE");
//    g_runTimeflag = 0;
  }
  
}

void DTC03Master::PrintKi()
{
  //unsigned int tconst;
  float tconst;
  lcd.SelectFont(SystemFont5x7);
  tconst = float(pgm_read_word_near(timeconst+g_kiindex))/100.0;
  lcd.GotoXY(I_COORD_X2, I_COORD_Y);
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
void DTC03Master::PrintB()
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(BCONST_COORD_X2, BCONST_COORD_Y);
  lcd.print(g_bconst); 
}
void DTC03Master::PrintModStatus() 
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(VMOD_COORD_X2, VMOD_COORD_Y);
  if(g_mod_status == 0) lcd.print("OFF");
  else lcd.print(" ON"); 
}
void DTC03Master::PrintAtune()
{
	lcd.SelectFont(SystemFont5x7);
	if(p_atunProcess_flag)
	{
		g_LCDlock_flag = 1;
		p_atunProcess_flag = 0;
		lcd.GotoXY(P_COORD_X, P_COORD_Y);
		lcd.print(" ____  ");
		lcd.GotoXY(I_COORD_X, I_COORD_Y);
		lcd.print("|Auto|");
		lcd.GotoXY(BCONST_COORD_X, BCONST_COORD_Y);
		lcd.print("|Tune|");
		lcd.GotoXY(VMOD_COORD_X, VMOD_COORD_Y);
		lcd.print("|....|");
		lcd.GotoXY(ATUNE_COORD_X, ATUNE_COORD_Y);
		lcd.print("|..  |");
//		delay(1000);
//		g_atunDone = 1;
	}
	else
	{
		lcd.GotoXY(ATUNE_COORD_X2, ATUNE_COORD_Y);
        if(g_atune_status == 0) lcd.print("OFF");
        else lcd.print(" ON");
	}
    
}
void DTC03Master::PrintAtuneDone()
{
	    g_LCDlock_flag = 0;
		g_atunDone = 0;
		g_atune_status = 0;
		I2CWriteData(I2C_COM_ATUN); //after recieve g_atunDone from slave, send this to slave zero the three flag(g_atunDone, g_DBRflag and g_runTimeflag) 
		
		BackGroundPrint();
		PrintNormalAll();		
}
void DTC03Master::PrintEnable() 
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(Test1_COORD_X, Test1_COORD_Y);
  lcd.print(g_en_state); 
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(Test3_COORD_X, Test3_COORD_Y);
  if(en_temp<10) lcd.print("   ");
  else if(en_temp<100) lcd.print("  ");
  else if (en_temp<1000) lcd.print(" ");
  lcd.print(en_temp); 
}
void DTC03Master::UpdateEnable()
{
 bool en_state;
 if(analogRead(ENSW)>500) en_state=1;
 else en_state=0;
 if(g_en_state != en_state)
 {
  g_en_state=en_state;
  I2CWriteData(I2C_COM_INIT);
 }
 //----------------------//
//  en_temp = analogRead(ENSW);
// if(en_temp>500) en_state=1;
// else en_state=0;
// if(g_en_state != en_state)
// {
//  g_en_state=en_state;
//  I2CWriteData(I2C_COM_INIT);
//  PrintEnable();
// }
}

void DTC03Master::PrintFactaryMode() //show error message to avoid entering Eng.Mode 
{
   lcd.ClearScreen(0); //clear the monitor
   lcd.SelectFont(Callibri11_bold);
   lcd.GotoXY(12,4);   
   lcd.print("FACTORY MODE");
   lcd.SelectFont(SystemFont5x7,WHITE);
   lcd.GotoXY(14,32);   
   lcd.print("Turn Off POWER");
   lcd.SelectFont(Wendy3x5);
   lcd.GotoXY(0,25);   
   lcd.print("PLEASE"); 
   lcd.GotoXY(36,42);   
   lcd.print("TO RE_START THE SYSTEM"); 
   delay(1000);
}

void DTC03Master::PrintEngBG()
{
  lcd.ClearScreen(0);
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(R1_COORD_X,R1_COORD_Y);
  lcd.print(Text_R1);
  lcd.GotoXY(R2_COORD_X,R2_COORD_Y);
  lcd.print(Text_R2);
  lcd.GotoXY(TPIDOFF_COORD_X,TPIDOFF_COORD_Y);
  lcd.print(Text_pidOS);
  lcd.GotoXY(VFBC_COORD_X,VFBC_COORD_Y);
  lcd.print(Text_Vfbc);
  lcd.GotoXY(VMOD_COOED_X,VMOD_COOED_Y);
  lcd.print(Text_Vmod);
  lcd.GotoXY(RMEAS_COORD_X,RMEAS_COORD_Y);
  lcd.print(Text_Rmeas);
  lcd.GotoXY(TOTP_COORD_X, TOTP_COORD_Y);
  lcd.print(Text_Totp);
  lcd.GotoXY(TPCB_COORD_X, TPCB_COORD_Y);
  lcd.print(Text_Tpcb);
  lcd.GotoXY(P_AT_COORD_X, P_AT_COORD_Y);
  lcd.print(Text_PAT);
  lcd.GotoXY(TBIAS_COORD_X, TBIAS_COORD_Y);
  lcd.print(Text_TAT);
  lcd.GotoXY(ATSTABLE_COORD_X, ATSTABLE_COORD_Y);
  lcd.print(Text_SAT);
}
void DTC03Master::PrintEngAll()
{
	PrintR1();
	PrintR2();
	PrintTpidoff();
	PrintVfbc();
	PrintVmod();
	PrintRmeas();
	PrintTotp();
	PrintP_Atune();
	PrintTbias();
	PrintATStable();
	CheckStatus();
}
void DTC03Master::PrintR1() //g_cursorstate=10
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(R1_COORD_X2, R1_COORD_Y);
  lcd.print(g_r1);
  lcd.print("  ");//
}
void DTC03Master::PrintR2() //g_cursorstate=11
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(R2_COORD_X2, R2_COORD_Y);
  lcd.print(g_r2);
  lcd.print("  ");//
}
void DTC03Master::PrintTpidoff() //g_cursorstate=12
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(TPIDOFF_COORD_X2, TPIDOFF_COORD_Y);
  lcd.print(g_tpidoff);
  lcd.print("  ");//
}
void DTC03Master::PrintVfbc()//g_cursorstate=13
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(VFBC_COORD_X2, VFBC_COORD_Y);
  lcd.print(g_fbcbase);
}
void DTC03Master::PrintVmod()//g_cursorstate=14
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(VMOD_COOED_X2, VMOD_COOED_Y);
  lcd.print(g_vmodoffset);
  lcd.print("  ");//
}
void DTC03Master::PrintRmeas()//g_cursorstate=15
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(RMEAS_COORD_X2, RMEAS_COORD_Y);
  lcd.print(g_Rmeas);
}
void DTC03Master::PrintTotp()//g_cursorstate=16
{
  float Topt_set;
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(TOTP_COORD_X2, TOTP_COORD_Y);
  Topt_set = float(g_otp)/4.0-20.5;
  if (Topt_set < 99.5 ) lcd.print(" ");
  lcd.print(Topt_set,0);
}
void DTC03Master::PrintTpcb(float tpcb)
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(TPCB_COORD_X2, TPCB_COORD_Y);
  if (tpcb < 100.0 ) lcd.print(" ");
  lcd.print(tpcb,0);
}
void DTC03Master::PrintP_Atune()
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(P_AT_COORD_X2, P_AT_COORD_Y );
  if(g_p_atune<10) lcd.print(" ");  
  lcd.print(g_p_atune);
}

void DTC03Master::PrintTbias()
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(TBIAS_COORD_X2, TBIAS_COORD_Y );
  if(g_T_atune<10) lcd.print(" ");  
  lcd.print(g_T_atune);
}

void DTC03Master::PrintATStable()
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(ATSTABLE_COORD_X2, ATSTABLE_COORD_Y );
  if(g_stableCode_atune<10) lcd.print(" ");  
  lcd.print(g_stableCode_atune);
}
void DTC03Master::CursorState()
{
  unsigned long t1, d1;
  unsigned int t_temp;
  if(!g_LCDlock_flag)
  {
  	if(analogRead(PUSHB)>HIGHLOWBOUNDRY)
    {
  	  p_engmodeCounter=0;
    }           
  
    if(analogRead(PUSHB)<=HIGHLOWBOUNDRY) //change cursorstate when push encoder switch  
    {
  	  t_temp=millis();
  	
  	  if( abs(t_temp-p_cursorStateCounter[0])<ACCUMULATE_TH ) //ACCUMULATE_TH=50
	  {
  		p_cursorStateCounter[1]=t_temp-p_cursorStateCounter[0];
  		p_cursorStateCounter[2]+=p_cursorStateCounter[1]; 		
	  }  
  	  else p_cursorStateCounter[2]=0;
  	
  	  if(!p_engModeFlag) //normal mode
  	  {
  		if ( p_cursorStateCounter[2]>LONGPRESSTIME ) //long press case:
		{
	  		if (abs(t_temp-p_cursorStayTime) > CURSORSTATE_STAYTIME && p_tBlink_toggle )
			{	
				p_HoldCursortateFlag=0; 
					
	  			if( g_cursorstate==0 || g_cursorstate==1 ) g_cursorstate=2;
		  		else g_cursorstate++;
		  		
		  		if( g_cursorstate>6 ) g_cursorstate=2;//Atune case : g_cursorstate>7	
		  		ShowCursor(0);//the index is not important
		  		p_engmodeCounter++;
				if(p_engmodeCounter > ENGCOUNTER) 
		        {
		        	g_cursorstate = 8;
		        	p_engModeFlag=1;
		        	p_cursorStateCounter[2]=0; 
		         	PrintFactaryMode();						        
		        } 
		  		p_cursorStayTime=t_temp;
			} 		
		}
	  	else //short press case:
		{
			if( abs(t_temp-p_tcursorStateBounce)> DEBOUNCE_WAIT ) //DEBOUNCE_WAIT=ACCUMULATE_TH*4
			{
				if( g_cursorstate==0 ) g_cursorstate=1;
		  		if ( g_cursorstate==1 )
				{
		  			if(g_tsetstep <= 0.001) g_tsetstep = 1.0;
		            else g_tsetstep = g_tsetstep/10.0;
		            ShowCursor(0);
				}
		  		else //g_cursorstate=2~7
				{
					if(g_cursorstate==7 && g_atune_status) 	
				    {
					    p_keyflag = 1;
				    	g_paramupdate = 1;					
				    }	
					p_HoldCursortateFlag=1;
					p_timerResetFlag=1;
				}								
				p_tcursorStateBounce=t_temp;
			} 	  		
		}
	  } 	
    	else //eng mode
    	{		
	    	if( abs(t_temp-p_tcursorStateBounce)> DEBOUNCE_WAIT )
	    	{		    			
		    	g_cursorstate++;
		        if( p_cursorStateCounter[2]>LONGPRESSTIME ) 
		        {
			      g_cursorstate=1;
			      p_engModeFlag=0;
			      lcd.ClearScreen(0);
			      BackGroundPrint();
			      PrintNormalAll();			
	     	    }
		       if( g_cursorstate==9 ) 
		       {
			      PrintEngBG();
			      PrintEngAll();
			      g_cursorstate=10;
		        }
		       if( g_cursorstate>19 ) g_cursorstate=10;
	  	       ShowCursor(0);
	  	       p_tcursorStateBounce=t_temp;
		   }		
	    }
//	I2CWriteData(I2C_COM_TEST2);
	p_cursorStateCounter[0]	= t_temp;
    }
  }  
}

void DTC03Master::HoldCursortate() //put this method in loop
{
	unsigned int t_temp, timer;
	unsigned char oldCursorState;
	if(p_HoldCursortateFlag==1)
	{
		//start timer when enter this section-----
		t_temp=millis();
		if(p_timerResetFlag==1) 
		{
			p_timerResetFlag=0;
			p_holdCursorTimer=t_temp;
		}
		timer=t_temp-p_holdCursorTimer; //alway reset the timer when g_cursorstate=2~6 @ short press case
		
		if( timer < DEBOUNCE_WAIT*2 ) {} // do nothing if reset the timer			
		else //wait too long! g_imer > 2*DEBOUNCE_WAIT
		{
			p_HoldCursortateFlag=0;
			oldCursorState=g_cursorstate;
			g_cursorstate=0;
			ShowCursor(oldCursorState);//here oldCursorState from 2~6
			g_cursorstate=1;
		}		
	}
}
void DTC03Master::blinkTsetCursor()
{
	unsigned int t_temp;
	if( p_blinkTsetCursorFlag==1 )
	{
		t_temp=millis();
		
		lcd.SelectFont(fixed_bold10x15);
		if(g_tsetstep == 1.0) lcd.GotoXY(TSET_COORD_X2+2*COLUMNPIXEL1015, TSET_COORD_Y);
	    else if(g_tsetstep == 0.1) lcd.GotoXY(TSET_COORD_X2+4*COLUMNPIXEL1015, TSET_COORD_Y);
	    else if(g_tsetstep == 0.01) lcd.GotoXY(TSET_COORD_X2+5*COLUMNPIXEL1015, TSET_COORD_Y);
	    else lcd.GotoXY(TSET_COORD_X2+6*COLUMNPIXEL1015, TSET_COORD_Y);
	    
		if( abs(t_temp-p_tBlink)>BLINKDELAY )
		{		  	
			if(p_tBlink_toggle) {
				lcd.print(" ");
				p_tBlink_toggle=!p_tBlink_toggle;
			}
			else{
				PrintTset();
				p_tBlink_toggle=!p_tBlink_toggle;
			}
			p_tBlink=t_temp;		
		}
	}
			
}
void DTC03Master::ShowCursor(unsigned char state_old)
{
		if( g_cursorstate!=1) p_blinkTsetCursorFlag=0;

		switch(g_cursorstate)
		{
		    case 0:
		    lcd.SelectFont(SystemFont5x7);
		    switch(state_old)
		    {
		    	case 2:
		    		lcd.GotoXY(ILIM_COORD_X-COLUMNPIXEL0507, ILIM_COORD_Y);
		    	break;
		    	case 3:
		    		lcd.GotoXY(P_COORD_X-COLUMNPIXEL0507, P_COORD_Y);
		    	break;
		    	case 4:
		    		lcd.GotoXY(I_COORD_X-COLUMNPIXEL0507, I_COORD_Y);
		    	break;
		    	case 5:
		    		lcd.GotoXY(BCONST_COORD_X-COLUMNPIXEL0507, BCONST_COORD_Y);
		    	break;
		    	case 6:
		    		lcd.GotoXY(VMOD_COORD_X-COLUMNPIXEL0507, VMOD_COORD_Y);		  		    
		    	break;	
		    	case 7:
		    		lcd.GotoXY(ATUNE_COORD_X-COLUMNPIXEL0507, ATUNE_COORD_Y);		  		    
		    	break;	
			}
			lcd.print(" ");		    
		    break;
		
		    case 1:
		    p_blinkTsetCursorFlag=1;
		    break;
		
		    case 2:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(ILIM_COORD_X-COLUMNPIXEL0507, ILIM_COORD_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(VMOD_COORD_X-COLUMNPIXEL0507, VMOD_COORD_Y);//
		    lcd.print(" ");
		    break;
		    
		    case 3:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(P_COORD_X-COLUMNPIXEL0507, P_COORD_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(ILIM_COORD_X-COLUMNPIXEL0507, ILIM_COORD_Y);
		    lcd.print(" ");
		    break;
		    
		    case 4:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(I_COORD_X-COLUMNPIXEL0507, I_COORD_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(P_COORD_X-COLUMNPIXEL0507, P_COORD_Y);
		    lcd.print(" ");
		    break;
		
		    case 5:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(BCONST_COORD_X-COLUMNPIXEL0507, BCONST_COORD_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(I_COORD_X-COLUMNPIXEL0507, I_COORD_Y);
		    lcd.print(" ");
		    break;
		
		    case 6:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(VMOD_COORD_X-COLUMNPIXEL0507, VMOD_COORD_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(BCONST_COORD_X-COLUMNPIXEL0507, BCONST_COORD_Y);
		    lcd.print(" ");
		    break;
		    
		    case 7:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(ATUNE_COORD_X-COLUMNPIXEL0507, ATUNE_COORD_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(VMOD_COORD_X-COLUMNPIXEL0507, VMOD_COORD_Y);
		    lcd.print(" ");
		    break;
		
		    case 10:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(R1_COORD_X-COLUMNPIXEL0507, R1_COORD_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(ATSTABLE_COORD_X-COLUMNPIXEL0507, ATSTABLE_COORD_Y);
		    lcd.print(" ");
		    break;
		
		    case 11:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(R2_COORD_X-COLUMNPIXEL0507, R2_COORD_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(R1_COORD_X-COLUMNPIXEL0507, R1_COORD_Y);
		    lcd.print(" ");
		    break;
		
		    case 12:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(TPIDOFF_COORD_X-COLUMNPIXEL0507, TPIDOFF_COORD_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(R2_COORD_X-COLUMNPIXEL0507, R2_COORD_Y);
		    lcd.print(" ");
		    break;
		    
		    case 13:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(VFBC_COORD_X-COLUMNPIXEL0507, VFBC_COORD_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(TPIDOFF_COORD_X-COLUMNPIXEL0507, TPIDOFF_COORD_Y);
		    lcd.print(" ");
		    break;
		
		    case 14:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(VMOD_COOED_X-COLUMNPIXEL0507, VMOD_COOED_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(VFBC_COORD_X-COLUMNPIXEL0507, VFBC_COORD_Y);
		    lcd.print(" ");
		    break;
		
		    case 15:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(RMEAS_COORD_X-COLUMNPIXEL0507, RMEAS_COORD_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(VMOD_COOED_X-COLUMNPIXEL0507, VMOD_COOED_Y);
		    lcd.print(" ");
		    break;
		
		    case 16:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(TOTP_COORD_X-COLUMNPIXEL0507, TOTP_COORD_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(RMEAS_COORD_X-COLUMNPIXEL0507, RMEAS_COORD_Y);
		    lcd.print(" ");
		    break;
		    
		    case 17:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(P_AT_COORD_X-COLUMNPIXEL0507, P_AT_COORD_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(TOTP_COORD_X-COLUMNPIXEL0507, TOTP_COORD_Y);
		    lcd.print(" ");
		    break;
		    
		    case 18:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(TBIAS_COORD_X-COLUMNPIXEL0507, TBIAS_COORD_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(P_AT_COORD_X-COLUMNPIXEL0507, P_AT_COORD_Y);
		    lcd.print(" ");
		    break;
		    
		    case 19:
		    lcd.SelectFont(SystemFont5x7, WHITE);
		    lcd.GotoXY(ATSTABLE_COORD_X-COLUMNPIXEL0507, ATSTABLE_COORD_Y);
		    lcd.print(" ");
		    lcd.SelectFont(SystemFont5x7);
		    lcd.GotoXY(TBIAS_COORD_X-COLUMNPIXEL0507, TBIAS_COORD_Y);
		    lcd.print(" ");
		    break;
		}
  
}
void DTC03Master::UpdateParam() // Still need to add the upper and lower limit of each variable
{
  unsigned char ki, ls;
  unsigned long timer1, timer2;
  if(g_paramupdate)
  {
    g_paramupdate = 0;
    p_ee_changed = 1;
    switch(g_cursorstate)
    {
      case 0:
      break;

      case 1:
        g_tset += g_tsetstep*g_counter;
        if(g_tset>25) g_tset=25;
        if(g_tset<-10) g_tset=-10;
        g_vset = ReturnVset(g_tset, g_sensortype);
        I2CWriteData(I2C_COM_VSET);
        PrintTset();
        p_blinkTsetCursorFlag=0;
        p_ee_change_state=EEADD_VSET_UPPER;	
      break; 
      
      case 2:
      	g_currentlim += g_counter;
        if(g_currentlim>51) g_currentlim=51;
        if(g_currentlim<1) g_currentlim=1;        
        I2CWriteData(I2C_COM_CTR);
        PrintIlim();
        p_ee_change_state=EEADD_currentlim; 
      break;

      case 3:
      	if(!g_kpkiFromAT) g_p += g_counter; 
        if(g_p>150) g_p=150;
        if(g_p<1) g_p=1;    
        I2CWriteData(I2C_COM_CTR);
        PrintP();
        p_ee_change_state=EEADD_P;
        if(!g_kpkiFromAT) break;
        else p_ee_changed = 1;

      case 4:
      	if(!g_kpkiFromAT) g_kiindex += g_counter;
      	else 
		{
			g_kpkiFromAT = 0;
      		g_cursorstate = 1;
		}
        if(g_kiindex>50) g_kiindex=50;
        if(g_kiindex<1) g_kiindex=1;      
        I2CWriteData(I2C_COM_KI);
        PrintKi();
        p_ee_change_state=EEADD_KIINDEX;
      break;

      case 5:
      	g_bconst += g_counter;
	    if(g_bconst>4499) g_bconst=4499;
	    if(g_bconst<3501) g_bconst=3501;
	    I2CWriteData(I2C_COM_INIT);
  	    g_vset = ReturnVset(g_tset, g_sensortype);
	    I2CWriteData(I2C_COM_VSET);//only send Vset, Bconst is not important for slave
	    PrintB();
	    p_ee_change_state=EEADD_BCONST_UPPER;
	    break;

      case 6:
        g_mod_status = g_countersensor;
        I2CWriteData(I2C_COM_INIT);
        PrintModStatus(); 
        p_ee_change_state=EEADD_MODSTATUS;
        break;
        
      case 7:
        g_atune_status = g_countersensor;
        if(g_atune_status && p_keyflag) 
		{
			p_keyflag = 0;
			p_atunProcess_flag = 1;
			I2CWriteData(I2C_COM_ATUN);
		}
        if(!g_atune_status) I2CWriteData(I2C_COM_ATUN);
        PrintAtune(); 
//        p_ee_change_state=EEADD_MODSTATUS;
        break;

      case 9:
        PrintFactaryMode();
        break;
        
      case 10:
      	g_r1 += g_counter;
	    if(g_r1>30) g_r1=30; // R1, 1~30 for 0.1~3.0 ohm set 
	    if(g_r1<1) g_r1=1;//
      	I2CWriteData(I2C_COM_R1R2);
        PrintR1();
        p_ee_change_state=EEADD_R1;
        break;

      case 11:
      	g_r2 += g_counter;
        if(g_r2>30) g_r2=30;
        if(g_r2<1) g_r2=1;//R2, 1~30 for 1.0~3.0 ohm set 
        I2CWriteData(I2C_COM_R1R2);
        PrintR2();
        p_ee_change_state=EEADD_R2;
        break;

      case 12:
      	g_tpidoff += g_counter;
        if(g_tpidoff>10) g_tpidoff=10; //Tpid offset, 0~10 for 1~10000 @1000 step
        if(g_tpidoff<1) g_tpidoff=1;
      
        I2CWriteData (I2C_COM_TPIDOFF);
        PrintTpidoff();
        p_ee_change_state=EEADD_TPIDOFF;
        break;


      case 13:
      	g_fbcbase +=(g_counter*100);
        if(g_fbcbase>50000) g_fbcbase=50000;
        if(g_fbcbase<20000) g_fbcbase=20000;
        I2CWriteData(I2C_COM_FBC);
        PrintVfbc();
        p_ee_change_state=EEADD_FBC_UPPER;
        break;

      case 14:
      	g_vmodoffset +=g_counter;
        if(g_vmodoffset>33199) g_vmodoffset=33199;
//        if(g_vmodoffset<32199) g_vmodoffset=32199;  //sherry+- 2018.3.31
        if(g_vmodoffset<32100) g_vmodoffset=32100;
        I2CWriteData(I2C_COM_VMOD);
        PrintVmod();
        p_ee_change_state=EEADD_MODOFF_UPPER;
        break;

      case 15:
      	g_Rmeas += (g_counter*100);
      	if(g_Rmeas>60000) g_Rmeas=60000;
        if(g_Rmeas<20000) g_Rmeas=20000;
        I2CWriteData(I2C_COM_RMEAS);
        PrintRmeas();
        p_ee_change_state=EEADD_RMEAS_UPPER;
        break;

      case 16:
      	g_otp += (g_counter*4);
		if (g_otp < 281) g_otp = 281; // 50C
		if (g_otp > 561) g_otp = 561; //120C
		I2CWriteData(I2C_COM_OTP);
        PrintTotp();  
        p_ee_change_state=EEADD_TOTP_UPPER;
        break;
        
       case 17:
      	g_p_atune += g_counter;
		if(g_p_atune>99) g_p_atune=99;
        if(g_p_atune<1) g_p_atune=1;    
        I2CWriteData(I2C_COM_ATUN);
        PrintP_Atune();
        p_ee_change_state=EEADD_PAP;
        break;
        
        case 18:
      	g_T_atune += g_counter;
		if(g_T_atune>99) g_T_atune=99;
        if(g_T_atune<1) g_T_atune=1;    
        I2CWriteData(I2C_COM_ATUN);
        PrintTbias();
        p_ee_change_state=EEADD_TBIAS;
        break;
        
        case 19:
      	g_stableCode_atune += g_counter;
		if(g_stableCode_atune>99) g_stableCode_atune=99;
        if(g_stableCode_atune<1) g_stableCode_atune=1;    
        I2CWriteData(I2C_COM_ATSTABLE);
        PrintATStable();
        p_ee_change_state=EEADD_ATSTABLE;
        break;
    }
  }
}
void DTC03Master::Encoder()
{
  unsigned char encoded, sum, dt;
  unsigned long tenc;
  bool MSB, LSB;
  if(!g_LCDlock_flag)
  {
  	tenc= millis();
    dt = tenc - g_tenc;
    if(dt < DEBOUNCETIME) return;
    if(dt > COUNTRESETTIME) g_icount =0;
    MSB = digitalRead(ENC_B);
    LSB = digitalRead(ENC_A);
    encoded = (MSB << 1)|LSB;
    sum = (g_lastencoded << 2)| encoded;
    if(g_icount%4==0)
    {
      g_paramupdate=1;// 20161031 when ineterrupt=4times g_paramupdate=1
      if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
      {

        g_counter = -1;
        g_countersensor =1;
      }
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) 
    {

      g_counter = 1;
      g_countersensor =0;
    }
  }
  g_lastencoded = encoded;
  g_tenc = tenc;
  g_icount++;
  }
  
}








