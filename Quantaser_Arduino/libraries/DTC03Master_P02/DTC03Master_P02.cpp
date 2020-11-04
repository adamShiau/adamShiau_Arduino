/* modification for disable sensortype purpose : data 2016/11/13
	1. in I2CReadData(), set g_sensortype to 0;
	2. add g_mod_status variable in DTC03Master_P02.h
	3. add VMOD x and y GLCD coordinate in DTC03Master_P02.h
	4. add PrintModStatus

*/
#include <DTC03Master_P02.h>
DTC03Master::DTC03Master()
{
}
void DTC03Master::SetPinMode()
{
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(PUSHB, INPUT);
  pinMode(ENSW, INPUT);
}
void DTC03Master::ParamInit()
{
  digitalWrite(ENC_A, HIGH);
  digitalWrite(ENC_B, HIGH);
  
  Wire.begin();
  lcd.Init();
  g_iarrayindex = 0;
  g_varrayindex = 0;
  g_itecsum = 0;
  g_vactsum = 0;
  g_paramupdate = 0;
  g_tsetstep = 1.00;
  g_en_state = 0;
  g_countersensor = 0;
  g_ttstart = 18.00;
  g_testgo =0;
  
}
void DTC03Master::WelcomeScreen()
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(0,0);
  lcd.print("DTC03 Ver.2.01");
  lcd.GotoXY(0,ROWPIXEL0507*1);
  lcd.print("Initializing");
  for (byte i=9; i>0; i--)
  {
    lcd.GotoXY(COLUMNPIXEL0507*(12+1),ROWPIXEL0507*1);
    lcd.print(i);
    delay(1000);
  }
  lcd.ClearScreen(0);//0~255 means ratio of black  
}
void DTC03Master::I2CWriteData(unsigned char com)
{
  unsigned char temp[2];
  switch(com)
  {
    case I2C_COM_INIT:
    temp[0]= g_bconst - BCONSTOFFSET;
    temp[1]= (g_bconst - BCONSTOFFSET) >> 8;
    if(g_en_state) temp[1] |= REQMSK_ENSTATE;
//    if(g_sensortype) temp[1]|= REQMSK_SENSTYPE; //20161113 mark
	if(g_mod_status) temp[1]|= REQMSK_SENSTYPE; //20161113
    
    break;

    case I2C_COM_CTR:
    temp[0]= g_currentlim;
    temp[1]= g_p;
    break;

    case I2C_COM_VSET:
    temp[0]=g_vset;
    temp[1]=g_vset>>8;
    break;

    case I2C_COM_KIINDEX:
    temp[0]=g_kiindex;
    temp[1]=0;
    break;

    case I2C_COM_VBEH:
    temp[0] = g_r1;
    temp[1] = g_r2;
    break;

    case I2C_COM_VBEC:
    temp[0] = g_tpidoff;
    temp[1] = g_vbec2;
    break;

    case I2C_COM_FBC:
    temp[0] = g_fbcbase;
    temp[1] = g_fbcbase>>8;//
    break;

    case I2C_COM_VMOD:
    temp[0] = g_vmodoffset;
    temp[1] = g_vmodoffset >>8;
    break;
    
    case I2C_COM_KI:
    temp[0]=pgm_read_word_near(kilstable+g_kiindex*2);
    temp[1]=pgm_read_word_near(kilstable+g_kiindex*2+1);
    break;

  }
  Wire.beginTransmission(DTC03P05);//20161031 add
  Wire.write(com);//
  Wire.write(temp, 2);//
  Wire.endTransmission();//
  delayMicroseconds(I2CSENDDELAY);//

}
void DTC03Master::I2CReadData(unsigned char com)
{
  unsigned char temp[2], b_upper, b_lower;
  unsigned int itectemp;
  int itec;
  bool itecsign;
  Wire.beginTransmission(DTC03P05);
  Wire.write(com);
  Wire.endTransmission();
  delay(I2CREADDELAY);
  Wire.requestFrom(DTC03P05,2);
  while(Wire.available()==2)
  {
    temp[0] = Wire.read();
    temp[1] = Wire.read();
  }
  switch(com)
  {
    case I2C_COM_INIT:
    b_lower = temp[0];
    b_upper = temp[1] & REQMSK_BUPPER;
//    g_sensortype = temp[1] & REQMSK_SENSTYPE; 20161113
	g_mod_status = temp[1] & REQMSK_SENSTYPE;//20161113
    g_bconst = (b_upper << 8) | b_lower + BCONSTOFFSET;//
    g_sensortype = 0;//20161113
    break;

    case I2C_COM_CTR:
    g_currentlim = temp[0];
    g_p = temp[1];
    break;

    case I2C_COM_VSET:
    g_vset = (temp[1]<<8) | temp[0];
    break;

    case I2C_COM_VACT:
    g_vact =(temp[1] <<8) | temp[0];
    break;

    case I2C_COM_ITEC_ER:
    itectemp = ((temp[1] & REQMSK_ITECU) << 8)| temp[0];
    itecsign = temp[1] & REQMSK_ITECSIGN;
    g_errcode1 = temp[1] & REQMSK_ERR1;
    g_errcode2 = temp[1] & REQMSK_ERR2;
    if(itecsign) itec = (-1)*(int)itectemp;//
    else itec = (int)itectemp;//
    g_itecsum -=Iarray[g_iarrayindex];
    Iarray[g_iarrayindex]=itec;//
    //g_itecsum += itec;
    g_itecsum +=Iarray[g_iarrayindex];//20161101
    g_iarrayindex ++;
    if(g_iarrayindex == IAVGTIMES) g_iarrayindex =0;
    break;

    case I2C_COM_VBEH:
    g_r1 = temp[0];
    g_r2 = temp[1];
    break;

    case I2C_COM_VBEC:
    g_tpidoff = temp[0];
    g_vbec2 = temp[1];
    break;

    case I2C_COM_FBC:
    g_fbcbase = (temp[1]<<8)|temp[0];
    break;

    case I2C_COM_VMOD:
    g_vmodoffset = (temp[1]<<8)|temp[0];
    break ;

    case I2C_COM_KIINDEX:
    g_kiindex = temp[0];
    break ;
  }
 delayMicroseconds(I2CSENDDELAY);//20161031
}
void DTC03Master::I2CReadAll()
{
  unsigned char i;
  for(i=I2C_COM_INIT; i <= I2C_COM_VMOD; i++)
  {
    I2CReadData(i);
  }
  
}
void DTC03Master::VarrayInit()
{
  
  for(unsigned char i=0; i<VAVGTIMES;i++)
  {
    if (i% IAVGTIMES ==0) I2CReadData(I2C_COM_VACT); //20161031 I2CRead every 8
    Varray[i]=g_vact;
    g_vactsum += g_vact;
  }
}
void DTC03Master::IarrayInit()
{
  for(unsigned char i=0; i<IAVGTIMES;i++) Iarray[i]=0;
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
}

void DTC03Master::PrintTset()
{
  lcd.SelectFont(fixed_bold10x15);
  lcd.GotoXY(TSET_COORD_X2,TSET_COORD_Y);
  if(g_tset<10.000)
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
  if ( abs(itec) <= 0.015 ) itec = 0;
  if(itec <0.00) lcd.print(itec,2); 

  else
   {
     lcd.print(" ");
     lcd.print(itec,2);
     //lcd.print(itec,3);
   } 
  //lcd.print(" "); 
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
  if(g_p<10)
   lcd.print("  ");
  else if (g_p<100)
   lcd.print(" ");
  lcd.print(g_p);
}
void DTC03Master::PrintKi()
{
  //unsigned int tconst;
  float tconst;
  lcd.SelectFont(SystemFont5x7);
  tconst = float(pgm_read_word_near(timeconst+g_kiindex))/100.0;
  lcd.GotoXY(I_COORD_X2, I_COORD_Y);
  if (g_kiindex==1) lcd.print(tconst);
  else if (g_kiindex<32)
  {
   lcd.print(" ");
   if (g_kiindex==0) lcd.print("OFF");
   else lcd.print(tconst,1);
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
void DTC03Master::Encoder()
{
  unsigned char encoded, sum, dt;
  unsigned long tenc;
  bool MSB, LSB;
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
void DTC03Master::CursorState()
{
  unsigned long t1;//
  unsigned int d1;
  //Curstate = 0 for 1C temp, go to Curstate = 1 when short push
  if(analogRead(PUSHB)>HIGHLOWBOUNDRY)
  {
   g_flag=1;
   g_engmode=0;          
  }
  
  if(analogRead(PUSHB)<=HIGHLOWBOUNDRY)
  {
    if((g_cursorstate==0 || g_cursorstate == 1) && g_flag)
    {
      t1 = millis();
      d1 = millis()-t1;
      while((analogRead(PUSHB)<=HIGHLOWBOUNDRY)&&(d1 < LONGPRESSTIME))
        d1 = millis()-t1;
      
      if(d1 >= LONGPRESSTIME)
      {
        g_cursorstate = 2;
        g_flag = 0; 
        ShowCursor();//20161031 showcoursor when first into state2
      }
    }
    if((g_cursorstate==0 || g_cursorstate == 1) && g_flag)//
    {
      g_cursorstate = 1;
      g_flag = 0;

      if(g_tsetstep <= 0.001) g_tsetstep = 1.0;
      else g_tsetstep = g_tsetstep/10.0;
    }
    if((g_cursorstate >1) && (g_cursorstate <7)) // g_cursorstate 2~6
    {
      t1 = millis();
      d1 = millis()-t1;
      while((analogRead(PUSHB)<=HIGHLOWBOUNDRY)&&(d1 < LONGPRESSTIME))
        d1 = millis()-t1;
      if(d1 >= LONGPRESSTIME)
      {
        g_cursorstate++;
        g_engmode++;
        g_flag = 0;
        if(g_cursorstate >6) g_cursorstate =2;//
        if(g_engmode > ENGCOUNTER) 
        {
         g_cursorstate = 9;
         PrintFactaryMode();//20161031
        }
      }
     else if(g_flag) g_cursorstate=0;//
    }
    
    if((g_cursorstate >=9) && (g_cursorstate <17) && g_flag)//
     {
      while(analogRead(PUSHB)<=HIGHLOWBOUNDRY);
      //if(g_cursorstate ==15) g_cursorstate = 0;
      //else g_cursorstate++;
      
      g_cursorstate++;//20161102
      if(g_cursorstate ==17) g_cursorstate = 0;//20161102
      else if (g_cursorstate ==10)//20161102
      {     
       PrintEngBG();
       PrintR1();
       PrintR2();
       PrintTpidoff();
       PrintVfbc();
       PrintVmod();
       PrintTtstart();
       PrintTestgo();
      }
     }
  
  }
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
void DTC03Master::UpdateParam() // Still need to add the upper and lower limit of each variable
{
  unsigned char ki, ls;
  unsigned long timer1, timer2;
  if(g_paramupdate)
  {
    g_paramupdate = 0;
    switch(g_cursorstate)
    {
      case 0:
        g_tset += g_tsetstep*g_counter;
        if(g_tset>100) g_tset=100;//20161031 limit
        if(g_tset<7) g_tset=7;//
        g_vset = ReturnVset(g_tset, g_sensortype);
        I2CWriteData(I2C_COM_VSET);
        PrintTset();
      break;

      case 1:
        g_tset += g_tsetstep*g_counter;
        g_vset = ReturnVset(g_tset, g_sensortype);
        I2CWriteData(I2C_COM_VSET);
        PrintTset();
        g_cursorstate=0;//20161031 return to state0 when update tset
      break; 
      
      case 2:
        if(g_currentlim>49) g_currentlim=49; //20161031
        if(g_currentlim<1) g_currentlim=1;//
        g_currentlim += g_counter;
        I2CWriteData(I2C_COM_CTR);
        PrintIlim();
      break;

      case 3:
        if(g_p>254) g_p=254;//20161031
        if(g_p<1) g_p=1;//
        g_p += g_counter;
        I2CWriteData(I2C_COM_CTR);
        PrintP();
      break;

      case 4:
        if(g_kiindex>48) g_kiindex=48;//20161103
        if(g_kiindex<1) g_kiindex=1;//
        g_kiindex += g_counter;//
        I2CWriteData(I2C_COM_KIINDEX);//20161031
        delayMicroseconds(I2CSENDDELAY);//
        I2CWriteData(I2C_COM_KI);//20161101 ki,ls change when kiindex change
        PrintKi();
      break;

      case 5:
      if(g_bconst>4499) g_bconst=4499;//20161031
      if(g_bconst<3501) g_bconst=3501;//
      g_bconst += g_counter;
      I2CWriteData(I2C_COM_INIT);
      g_vset = ReturnVset(g_tset, g_sensortype);//20161101 vset change when bconst change
      I2CWriteData(I2C_COM_VSET);//
      PrintB();
      break;

      case 6:

      g_mod_status = g_countersensor;//20161113
      I2CWriteData(I2C_COM_INIT);
      PrintModStatus(); //20161113

      break;

      case 9:
      PrintFactaryMode();
      break;

      case 10:
      if(g_r1>30) g_r1=30; // R1, 1~30 for 0.1~3.0 ohm set 
      if(g_r1<1) g_r1=1;//
      g_r1 += g_counter;
      I2CWriteData(I2C_COM_VBEH);
      PrintR1();
      break;

      case 11:
      if(g_r2>30) g_r2=30;
      if(g_r2<1) g_r2=1;//R2, 1~30 for 1.0~3.0 ohm set 
      g_r2 += g_counter;
      I2CWriteData(I2C_COM_VBEH);
      PrintR2();
      break;

      case 12:

      if(g_tpidoff>10) g_tpidoff=10; //Tpid offset, 0~10 for 1~10000 @1000 step
      if(g_tpidoff<1) g_tpidoff=1;//
      g_tpidoff += g_counter;
      I2CWriteData (I2C_COM_VBEC);
      PrintTpidoff();
      break;


      case 13:
      if(g_fbcbase>44900) g_fbcbase=44900;//
      if(g_fbcbase<16100) g_fbcbase=16100;//
      g_fbcbase +=(g_counter*100);
      I2CWriteData(I2C_COM_FBC);
      PrintVfbc();
      break;

      case 14:
      if(g_vmodoffset>33199) g_vmodoffset=33199;//
      if(g_vmodoffset<32199) g_vmodoffset=32199;//
      g_vmodoffset +=g_counter;
      I2CWriteData(I2C_COM_VMOD);
      PrintVmod();
      break;

      case 15:
      g_ttstart += float(g_counter);
      PrintTtstart();
      break;

      case 16:
      if(g_counter >=0) g_testgo =1;
      else g_testgo =0;
      PrintTestgo();
      if(g_testgo = 1)
      {

        timer1 = millis()+TESTGOPERIOD;
        timer2 =millis();
        g_vset = ReturnVset(g_ttstart, g_sensortype);
        I2CWriteData(I2C_COM_VSET);
        g_en_state =1;
        I2CWriteData(I2C_COM_INIT);
        while(timer2 < timer1) 
          {
            timer2=millis();
            CheckStatus();
          }
        g_vset = ReturnVset(g_ttstart+TESTGOSTEP, g_sensortype);
        I2CWriteData(I2C_COM_VSET);
        timer1 = millis()+TESTGOPERIOD;
        timer2 =millis();
        while(timer2 < timer1) 
          {
            timer2=millis();
            CheckStatus();
          }
        g_en_state =0;
        I2CWriteData(I2C_COM_INIT);
      }
    }
  }
}

void DTC03Master::UpdateEnable()//20161101
{
 bool en_state;
 if(analogRead(ENSW)>500) en_state=1;
 else en_state=0;
 if(g_en_state != en_state)
 {
  g_en_state=en_state;
  I2CWriteData(I2C_COM_INIT);
 }
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

}
void DTC03Master::CheckStatus()
{
  float tact, itec;
  I2CReadData(I2C_COM_ITEC_ER);
  itec = float(g_itecsum)/IAVGTIMES*CURRENTRatio;//
  PrintItec(itec);
  
  I2CReadData(I2C_COM_VACT);
  tact = ReturnTemp(g_vact,g_sensortype);
  PrintTact(tact);
}

void DTC03Master::PrintEngBG()
{
  lcd.ClearScreen(0);
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(VR1_X,VR1_Y);
  lcd.print("    R1:");
  lcd.GotoXY(VR2_X,VR2_Y);
  lcd.print("    R2:");
  lcd.GotoXY(TPIDOFF_X,TPIDOFF_Y);
  lcd.print(" pidOS:");
  lcd.GotoXY(VFBC_X,VFBC_Y);
  lcd.print("  Vfbc:");
  lcd.GotoXY(VMOD_X,VMOD_Y);
  lcd.print("  Vmod:");
  lcd.GotoXY(TTSTART_X,TTSTART_Y);
  lcd.print("Tstart:");
  lcd.GotoXY(TESTGO_X, TESTGO_Y);
  lcd.print("TestGo:");
}
void DTC03Master::PrintR1() //change to print R1 20161114
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(VR1_X2, VR1_Y);
  lcd.print(g_r1);
  lcd.print("  ");//
}
void DTC03Master::PrintR2() //change to print R2 20161114
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(VR2_X2, VR2_Y);
  lcd.print(g_r2);
  lcd.print("  ");//
}
void DTC03Master::PrintTpidoff() //change to print Tpid offset 20161114
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(TPIDOFF_X2, TPIDOFF_Y);
  lcd.print(g_tpidoff);
  lcd.print("  ");//
}
// void DTC03Master::PrintVbec2() //no print
// {
//   lcd.SelectFont(SystemFont5x7);
//   lcd.GotoXY(VBEC2_X2, VBEC2_Y);
//   lcd.print(g_vbec2);
//   lcd.print("  ");//
// }
void DTC03Master::PrintVfbc()
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(VFBC_X2, VFBC_Y);
  lcd.print(g_fbcbase);
}
void DTC03Master::PrintVmod()
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(VMOD_X2, VMOD_Y);
  lcd.print(g_vmodoffset);
  lcd.print("  ");//
}
void DTC03Master::PrintTtstart()
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(TTSTART_X2, TTSTART_Y);
  lcd.print(g_ttstart);
  lcd.print("  ");//
}
void DTC03Master::PrintTestgo()
{
  lcd.SelectFont(SystemFont5x7);
  lcd.GotoXY(TESTGO_X2, TTSTART_Y);
  lcd.print(g_testgo);
}

void DTC03Master::ShowCursor()
{
  switch(g_cursorstate)
  {
    case 0:
    lcd.SelectFont(SystemFont5x7);
    lcd.GotoXY(ILIM_COORD_X-COLUMNPIXEL0507, ILIM_COORD_Y);
    lcd.print(" ");
    lcd.GotoXY(P_COORD_X-COLUMNPIXEL0507, P_COORD_Y);
    lcd.print(" ");
    lcd.GotoXY(I_COORD_X-COLUMNPIXEL0507, I_COORD_Y);
    lcd.print(" ");
    lcd.GotoXY(BCONST_COORD_X-COLUMNPIXEL0507, BCONST_COORD_Y);
    lcd.print(" ");
    lcd.GotoXY(VMOD_COORD_X-COLUMNPIXEL0507, VMOD_COORD_Y);
    lcd.print(" ");
    break;

    case 1:
    lcd.SelectFont(fixed_bold10x15);
    if(g_tsetstep == 1.0) lcd.GotoXY(TSET_COORD_X2+2*COLUMNPIXEL1015, TSET_COORD_Y);
    else if(g_tsetstep == 0.1) lcd.GotoXY(TSET_COORD_X2+4*COLUMNPIXEL1015, TSET_COORD_Y);//
    else if(g_tsetstep == 0.01) lcd.GotoXY(TSET_COORD_X2+5*COLUMNPIXEL1015, TSET_COORD_Y);//
    else lcd.GotoXY(TSET_COORD_X2+6*COLUMNPIXEL1015, TSET_COORD_Y);//
    lcd.print(" ");
    delay(BLINKDELAY);
    PrintTset();
    delay(BLINKDELAY);//add
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

    case 10:
    lcd.SelectFont(SystemFont5x7, WHITE);
    lcd.GotoXY(VR1_X, VR1_Y);
    lcd.print(" ");
    break;

    case 11:
    lcd.SelectFont(SystemFont5x7, WHITE);
    lcd.GotoXY(VR2_X, VR2_Y);
    lcd.print(" ");
    lcd.SelectFont(SystemFont5x7);
    lcd.GotoXY(VR1_X, VR1_Y);
    lcd.print(" ");
    break;

    case 12:
    lcd.SelectFont(SystemFont5x7, WHITE);
    lcd.GotoXY(TPIDOFF_X, TPIDOFF_Y);
    lcd.print(" ");
    lcd.SelectFont(SystemFont5x7);
    lcd.GotoXY(VR2_X, VR2_Y);
    lcd.print(" ");
    break;
    
    case 13:
    lcd.SelectFont(SystemFont5x7, WHITE);
    lcd.GotoXY(VFBC_X, VFBC_Y);
    lcd.print(" ");
    lcd.SelectFont(SystemFont5x7);
    lcd.GotoXY(TPIDOFF_X, TPIDOFF_Y);
    lcd.print(" ");
    break;

    case 14:
    lcd.SelectFont(SystemFont5x7, WHITE);
    lcd.GotoXY(VMOD_X, VMOD_Y);
    lcd.print(" ");
    lcd.SelectFont(SystemFont5x7);
    lcd.GotoXY(VFBC_X, VFBC_Y);
    lcd.print(" ");
    break;

    case 15:
    lcd.SelectFont(SystemFont5x7, WHITE);
    lcd.GotoXY(TTSTART_X, TTSTART_Y);
    lcd.print(" ");
    lcd.SelectFont(SystemFont5x7);
    lcd.GotoXY(VMOD_X, VMOD_Y);
    lcd.print(" ");
    break;

    case 16:
    lcd.SelectFont(SystemFont5x7, WHITE);
    lcd.GotoXY(TESTGO_X, TESTGO_Y);
    lcd.print(" ");
    lcd.SelectFont(SystemFont5x7);
    lcd.GotoXY(TTSTART_X, TTSTART_Y);
    lcd.print(" ");
    break;
  }

}









