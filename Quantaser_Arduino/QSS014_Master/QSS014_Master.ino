
#define I2CSENDDELAY 100 //delay100us
#include <Wire.h>
#include "QSS014_cmn.h"
#include <LTC2451.h>


String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

/********number of uart command number **********/
#define COMMAND_NUM 3

/********glogal variable***************/
int g_freq, g_phase;



typedef struct table {
  char *cmd;
  void (*action)(char *);
} table_t;

table_t cmd_list[COMMAND_NUM];
LTC2451 adc;

void setup() {
/***** register command and act function here ******/
  cmd_list[0].cmd = "MOD_FREQ";
  cmd_list[0].action = ACT_setModFreq;

  cmd_list[1].cmd = "MOD_PHASE";
  cmd_list[1].action = ACT_setModPhase;

  cmd_list[2].cmd = "READ_ADC";
  cmd_list[2].action = ACT_readAdc;


/****************************************************/

  adc.Init(MODE60HZ);
  
  Serial.begin(9600);
  Wire.begin();
}

void loop() {

  if (stringComplete)
  {
    char *c_inputString = (char*)inputString.c_str();
//    Serial.print("loop: ");
//    Serial.println(inputString);
    match_cmd(c_inputString, cmd_list);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}


void match_cmd(char *input_string, table_t cmd[])
{
  for(int i=0; i<COMMAND_NUM; i++)
  {
    if (strstr(input_string, cmd[i].cmd) != NULL) cmd[i].action(input_string);   
  }
}

void ACT_setModFreq(char *string)
{
   String str = string;

   //value 起始位址 = command 起始位址 + 空格處index + 1
   g_freq = atoi(string + str.indexOf(' ') + 1);
   I2CWriteData(I2C_MOD_FREQ);
   Serial.println(g_freq);
}

void ACT_setModPhase(char *string)
{
   String str = string;

   g_phase = atoi(string + str.indexOf(' ') + 1);
   I2CWriteData(I2C_MOD_PHASE);
   Serial.println(g_phase);
}

void ACT_readAdc(char *string)
{
   unsigned int value;

   value = adc.Read();
   Serial.println(value);
}


void I2CWriteData(unsigned char com)
{
  unsigned char temp[2];
  switch (com)
  {
    case I2C_MOD_FREQ:
      temp[0] = g_freq >> 8;
      temp[1] = g_freq;
      break;

    case I2C_MOD_PHASE:
      temp[0] = g_phase >> 8;
      temp[1] = g_phase;
      break;
  }
  Wire.beginTransmission(SLAVE_MCU_I2C_ADDR);//
  Wire.write(com);//
  Wire.write(temp, 2);//
  Wire.endTransmission();//
  delayMicroseconds(I2CSENDDELAY);//
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
//    Serial.print("event: ");
//    Serial.println(inputString);
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
