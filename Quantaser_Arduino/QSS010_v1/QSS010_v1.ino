#include <AD5541.h>
#include <LTC1865.h>

#define DACC 7
#define LTC1865CONV 9

//for integrator usage 
#define S1 6
#define S2 5
#define S3 4
#define S4 3
#define S5 A2
#define S6 A3
#define INJECTION_CHARGE_TIME 5
#define NEGATIVE_SAMPLING 1

#define TEST 0

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
	digitalWrite(S5, HIGH);
	digitalWrite(S6, LOW);
	reset(30);
}

void loop() {
	float v_init, v_step;
	unsigned long dt, pts, data_rate, delay_time_us, mv_times;
	unsigned int dv, data0, data1;
	long v0, start_time = micros(), t0, sum_data1;
	bool stop_flag = 0;
	int i=0;
	scanVar("initial Voatage(V): ", v_init);
	scanVar("scan step(V): ", v_step);
	scanVar("scan points: ", pts); 
	scanVar("integrate time(us): ", dt);
	scanVar("update rate(Hz): ", data_rate);
//  scanVar("moving average: ", mv_times);
	delay_time_us = (1000000/data_rate);
	v0 = (v_init/5.0)*65535;
	dv = (v_step/5.0)*65535;
//  for(int j=0; j<mv_times; j++)sum_data1+=adc_read_ch1;
#if TEST
	Serial.println(v0);
	Serial.println(dv);
	Serial.println(pts); //scan half period points
	Serial.println(dt);
	Serial.println(1000000/delay_time_us);
//  Serial.println(mv_times);
#endif

//  t0 = millis();
	while(1)
	{
		ad5541.NormalWrite(v0);
		/////// integrate process /////
		reset(30);//不得<30, int hold end
		hold(10);
		integrate(dt);
    hold(10);
//		hold_sample(1); 
		///////////////////////////////
		data0 = ltc1865.Read(1); // read the ch0 data and next channel is ch1
		data1 = ltc1865.Read(0); // read the ch1 data and next chanel is ch0
		if(i < pts)
		{
			v0 += dv;
			if(v0 > 65535) v0=65535;
		}
		else if(i < 2*pts)
		{
			v0 -= dv;
			if(v0 < 0) v0 = 0;
		}
		i++;
		if(i == 2*pts) i = 0;
		updataData(start_time, delay_time_us, data0, data1);
//    cnt++;
//    if(cnt==data_rate)
//    {
//      cnt=0;
//      Serial.print("time_update= ");
//      Serial.println(millis()-t0);
//      t0=millis();
//    }
		checkStop(stop_flag); // input any number greater than 0 will break the while loop
		if(stop_flag) break;
	}
  
}
void updataData(long &start_time, unsigned long delay_time_us, unsigned int data0, unsigned int data1)
{
  unsigned int dataH = 0, dataL = 0;
	while((micros() - start_time) < delay_time_us);
	start_time = micros();
  #if TEST
  Serial.print(start_time);
  Serial.print(",");
  #endif
#if 0
	Serial.print(data0);
	Serial.print(",");
	Serial.println(data1);
#else
//  dataH = (data0 & 0xFF00) >> 8;
//  Serial.write(dataH);
//  dataL = data0 & 0x00FF;
//  Serial.write(dataL);
//  dataH = (data1 & 0xFF00) >> 8;
//  Serial.write(dataH);
//  dataL = data1 & 0x00FF;
//  Serial.write(dataL);
#endif
  
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
#if TEST
	Serial.print(s);
#endif
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
//  PORTD = ((PORTD & B10001111) | (1<<S1) | (1<<S2) | (1<<S3)); //start sampling ad620 positive input
  PORTD = ((PORTD & B10000111) | (1<<S1) | (1<<S2) | (1<<S4)); //start sampling ad620 negative input
  delayMicroseconds(wait);
  PORTD = (PORTD & B11100111) ; // //stop sampling ad620 negative input
}

//void integrate(int wait) //01
//{
//  unsigned int bg;
//  PORTD = ((PORTD & B10011111) | (1<<S2)); //int start
//  delayMicroseconds(INJECTION_CHARGE_TIME); 
////  PORTD = ((PORTD & B11100111) | (1<<S4) ); //start sampling ad620 negative input
//  PORTD = ((PORTD & B11100111) | (1<<S3) ); //start sampling ad620 positive input
//  delayMicroseconds(NEGATIVE_SAMPLING); 
//  PORTD = (PORTD & B11100111); //stop sampling ad620 positive input
//  delayMicroseconds(wait);
//}
void integrate(unsigned long wait) //01
{
  unsigned int bg;
  PORTD = ((PORTD & B10011111) | (1<<S2)); //int start
//  delayMicroseconds(INJECTION_CHARGE_TIME); 
//  PORTD = ((PORTD & B11100111) | (1<<S3) ); //start sampling ad620 positive input
//  delayMicroseconds(NEGATIVE_SAMPLING); 
//  PORTD = (PORTD & B11100111); //stop sampling ad620 positive input
//  delay(wait);
  delayMicroseconds(wait);
}
