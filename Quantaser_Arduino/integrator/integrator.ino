#include <AD5541.h>
#include <LTC1865.h>

#define DACC 7
#define LTC1865CONV 9
#define COMMAND_NUM 1

//for integrator usage 
#define S1 6
#define S2 5
#define S3 4
#define S4 3
#define S5 A2
#define S6 A3
#define INJECTION_CHARGE_TIME 5
#define NEGATIVE_SAMPLING 10

#define TEST 1

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

typedef struct table {
  char *cmd;
  void (*action)(char *);
} table_t;

table_t cmd_list[COMMAND_NUM];

unsigned long g_dac = 0;

AD5541 ad5541;
LTC1865 ltc1865;
long cnt=0;
void setup() {
	Serial.begin(115200);
	pinMode(DACC,OUTPUT);
	ad5541.init();
	ad5541.SetPin(DACC);
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	ltc1865.init(LTC1865CONV,0);// ch0 is the first channel to read)
	
	//for integrator usage 
	pinMode(S1, OUTPUT);
	pinMode(S2, OUTPUT);
	pinMode(S3, OUTPUT);
	pinMode(S4, OUTPUT);
	pinMode(S5, OUTPUT);
	pinMode(S6, OUTPUT);
	digitalWrite(S3, LOW);
	digitalWrite(S4, LOW);
  digitalWrite(S6, LOW);
  digitalWrite(S5, HIGH);
	reset(30);

/***** register command and act function here ******/
  cmd_list[0].cmd = "DAC";
  cmd_list[0].action = ACT_setDAC;
/****************************************************/
  
}

void loop() {
	float v_init, v_step;
	unsigned long pts, data_rate, delay_time_us, mv_times;
  unsigned long dt;
	unsigned int dv, data0, data1;
	long v0, start_time = micros(), t0, sum_data1;
	bool stop_flag = 0;
	int i=0;
//  scanVar("DAC(mV): ", g_dac);
//  ad5541.NormalWrite((int)((float)g_dac/5000.0*65535));
//	while(1)
//	{

		reset(30);//不得<30, int hold end
		hold(10);
		integrate(1);

		hold_sample(5); 

//		checkStop(stop_flag); // input any number greater than 0 will break the while loop
//		if(stop_flag) break;
//	}
  
}

void match_cmd(char *input_string, table_t cmd[])
{
  for(int i=0; i<COMMAND_NUM; i++)
  {
    if (strstr(input_string, cmd[i].cmd) != NULL) cmd[i].action(input_string);   
  }
}

void ACT_setDAC(char *string)
{
   String str = string;

   //value 起始位址 = command 起始位址 + 空格處index + 1
   g_dac = atoi(string + str.indexOf(' ') + 1);
   Serial.println(g_dac);
}

void updataData(long &start_time, unsigned long delay_time_us, unsigned int data0, unsigned int data1)
{
  unsigned int dataH = 0, dataL = 0;
	while((micros() - start_time) < delay_time_us);
	start_time = micros();
  
}

void checkStop(bool &stop_flag)
{
  char r;
  while(Serial.available() > 0)
  {
    stop_flag = Serial.parseInt();
    r=Serial.read();
  }  
}

void scanVar(String s, unsigned long &var)
{
	char r;

	Serial.print(s);

	while(Serial.available() == 0); // hold here till input from uart
	var = Serial.parseInt();
	r=Serial.read(); //read "\n"
	Serial.println(var);
}

void scanVar(String s, float &var)
{
	char r;
#if TEST
	Serial.print(s);
#endif
	while(Serial.available() == 0);
	var = Serial.parseFloat();
	r=Serial.read(); //read "\n"
	Serial.println(var);
}

void reset(int wait) //10
{
  PORTD = ((PORTD & B10011111) | (1<<S1)); 
  delayMicroseconds(wait);
}

void hold(int wait) //11
{
  PORTD = ((PORTD & B10011111) | (1<<S1) | (1<<S2)); 
  delayMicroseconds(wait);
}

void hold_sample(int wait) //11
{
  PORTD = ((PORTD & B10000111) | (1<<S1) | (1<<S2) | (1<<S4)); //start sampling ad620 negative input
  delayMicroseconds(wait);
  PORTD = (PORTD & B11100111) ; // //stop sampling ad620 negative input
}

void integrate(unsigned long wait) //01
{
  unsigned int bg;
  PORTD = ((PORTD & B10011111) | (1<<S2)); //int start
  delayMicroseconds(INJECTION_CHARGE_TIME); 
  PORTD = ((PORTD & B11100111) | (1<<S3) ); //start sampling ad620 positive input
  delayMicroseconds(NEGATIVE_SAMPLING); 
  PORTD = (PORTD & B11100111); //stop sampling ad620 positive input
  delay(wait);
}
//void integrate(unsigned long wait) //01
//{
//  unsigned int bg;
//  PORTD = ((PORTD & B10011111) | (1<<S2)); //int start
//
//  delay(wait);
//}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    Serial.println(inputString);
    if (inChar == '\n') {
      stringComplete = true;
    }
    Serial.println(stringComplete);
  }
}
