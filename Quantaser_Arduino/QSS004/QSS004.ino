#include <LTC2615.h>
#include <LTC2451.h>

LTC2615 ltc2615;
LTC2451 ltc2451;

#define DEGUG 0

unsigned long ul_time_begin = 0, ul_time_current = 0;
unsigned long ul_ReadCounter = 0;
volatile unsigned long ul_Counter = 0;

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PD2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PD2),AddCounter,RISING);
  inputString.reserve(20);
  ltc2615.init();
  ltc2451.Init(0);
  ul_time_begin = millis();

}

void loop() {
  // put your main code here, to run repeatedly:

  ul_time_current = millis();
  if ( (ul_time_current - ul_time_begin) > 1000 )
  {
    ul_ReadCounter = ul_Counter;
    ul_Counter = 0;
    ul_time_begin = millis();
//    Serial.print("inside:");
    //Serial.println(ul_ReadCounter);
  }

  if (stringComplete)
  {
    
    char *c_inputString = (char*)inputString.c_str();
    char *set_vol_str = strstr(c_inputString, "SetVoltage");
    char *read_cnt_str = strstr(c_inputString, "ReadCounter");
    char *read_vol_str = strstr(c_inputString, "ReadVoltage");
 
    if (set_vol_str != NULL)
    {
      char *ch_str = c_inputString + 11;
      unsigned char ch = atoi(ch_str);
      //Serial.println(ch);
      char *vol_str = ch_str + 2;
      unsigned int vol = atoi(vol_str);
      //Serial.println(vol);
      SetVoltage(ch, vol);
	  
    }

    if (read_cnt_str != NULL)
    {
      //Serial.print("asked:");
      Serial.println(ul_ReadCounter);
    }

    if (read_vol_str != NULL)
    {
#if 0
      char *mv_str = c_inputString + 12;
      unsigned int mv = atoi(mv_str);
      ReadVoltage(mv);
#else
      ReadVoltage();
#endif
    }

    // clear the string:
    inputString = "";
    stringComplete = false;
  }

}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
//    Serial.print(inChar);
  }
}

void SetVoltage(unsigned char ch, unsigned int vol)
{
    char tempStr[30];

  if (ch == 1)
  {
    ltc2615.writeint(CH_A, vol);
#if DEBUG
    sprintf(tempStr, "ch = %d, value = %u", ch, vol);
    Serial.println(tempStr);
#endif
  }
  else if (ch == 2)
  {
    ltc2615.writeint(CH_B, vol);
#if DEBUG
    sprintf(tempStr, "ch = %d, value = %u", ch, vol);
    Serial.println(tempStr);
#endif
  }
  else if (ch == 3)
  {
    ltc2615.writeint(CH_C, vol);
#if DEBUG
    sprintf(tempStr, "ch = %d, value = %u", ch, vol);
    Serial.println(tempStr);
#endif
  }
  else if (ch == 4)
  {
    ltc2615.writeint(CH_D, vol);
#if DEBUG
    sprintf(tempStr, "ch = %d, value = %u", ch, vol);
    Serial.println(tempStr);
#endif
  }
  else if (ch == 5)
  {
    ltc2615.writeint(CH_E, vol);
#if DEBUG
    sprintf(tempStr, "ch = %d, value = %u", ch, vol);
    Serial.println(tempStr);
#endif
  }
}

void AddCounter()
{
  ul_Counter++;
}

#if 0
void ReadVoltage(unsigned int mv)
{
  unsigned int ui_ReadVoltage = 0, ui_TotalVoltage = 0, i = 0;
  //Serial.println(mv);
  for (i = 0; i < mv; i++)
  {
    ui_ReadVoltage = ltc2451.Read();
    ui_TotalVoltage += ui_ReadVoltage;
  }
  //Serial.println(ui_TotalVoltage);
  ui_ReadVoltage = ui_TotalVoltage / mv;
  Serial.println(ui_ReadVoltage);
}
#else
void ReadVoltage()
{
  unsigned int ui_ReadVoltage = 0;
  ui_ReadVoltage = ltc2451.Read();
  Serial.println(ui_ReadVoltage);
}
#endif
