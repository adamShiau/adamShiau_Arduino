#include "pig_v2.h"

#define CHECK_BYTE		170
#define CHECK_BYTE2		171
#define CHECK_BYTE3		172

#define MOD_FREQ_ADDR			0
#define MOD_AMP_H_ADDR  		1
#define MOD_AMP_L_ADDR  		2
#define ERR_OFFSET_ADDR 		3
#define POLARITY_ADDR  			4
#define WAIT_CNT_ADDR  			5
#define ERR_TH_ADDR  			6
#define ERR_AVG_ADDR  			7
#define TIMER_RST_ADDR  		8
#define GAIN1_ADDR  			9
#define GAIN2_ADDR  			10
#define FB_ON_ADDR  			11
#define CONST_STEP_ADDR  		12
#define FPGA_Q_ADDR				13
#define FPGA_R_ADDR  			14
#define DAC_GAIN_ADDR  			50
#define DATA_INT_DELAY_ADDR 	98
#define DATA_OUT_START_ADDR		99

#define INT_SYNC	1
#define EXT_SYNC 	1<<1
#define STOP_SYNC 	1<<2

PIG::PIG()
{
	
}

void PIG::init()
{
	Serial.begin(115200);
	Serial1.begin(115200);
	p_time_cnt = 0;
}

void PIG::sendCmd(unsigned char addr, unsigned int value)
{
	Serial1.write(addr);
	Serial1.write(value>>24 & 0xFF);
	Serial1.write(value>>16 & 0xFF);
	Serial1.write(value>>8 & 0xFF);
	Serial1.write(value & 0xFF);
	delay(1);
	
}

char PIG::setSyncMode(unsigned int CTRLREG)
{
	char run_fog_flag = 0;
	
	switch(CTRLREG) {
		case INT_SYNC: {
			sendCmd(DATA_OUT_START_ADDR, 1);
			run_fog_flag = 1;
			break;
		}
		case EXT_SYNC: {
			sendCmd(DATA_OUT_START_ADDR, 2);
			run_fog_flag = 1;
			break;
		}
		case STOP_SYNC: {
			sendCmd(DATA_OUT_START_ADDR, 0);
			resetFakeDataTime();
			run_fog_flag = 0;
			break;
		}
		default: break;
	}
	return run_fog_flag;
}

void PIG::printVal(char name[], int val)
{
	Serial.print(name);
	Serial.print(": ");
	Serial.println(val);
}

void PIG::readData(unsigned char data[16])
{
	unsigned char val1, val2, val3;
	unsigned int time, PD_T;
	int err, step;
	
	Serial1.readBytes(&val1, 1);
	Serial1.readBytes(&val3, 1);
	while(val1 != CHECK_BYTE or val3 != CHECK_BYTE3){
		val1 = val3;
		Serial1.readBytes(&val3, 1);
	}
	Serial1.readBytes(data, 16);
	Serial1.readBytes(&val2, 1);
}

void PIG::resetFakeDataTime()
{
	p_time_cnt = 0;
}

void PIG::readFakeData(unsigned char data[16])
{
	data[0] = p_time_cnt >> 24;
	data[1] = p_time_cnt >> 16;
	data[2] = p_time_cnt >> 8;
	data[3] = p_time_cnt ;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 1;
	data[8] = 0;
	data[9] = 0;
	data[10] = 0;
	data[11] = 2;
	data[12] = 0;
	data[13] = 0;
	data[14] = 0;
	data[15] = 3;
	p_time_cnt++;
}

void PIG::readData_debug(unsigned char data[16])
{
	unsigned char val1, val2, val3;
	unsigned int time, PD_T;
	int err, step;
	
	Serial1.readBytes(&val1, 1);
	Serial1.readBytes(&val3, 1);
	while(val1 != CHECK_BYTE or val3 != CHECK_BYTE3){
		val1 = val3;
		Serial1.readBytes(&val3, 1);
	}
	Serial1.readBytes(data, 16);
	Serial1.readBytes(&val2, 1);
	time = data[0]<<24 | data[1]<<16 | data[2]<<8 | data[3];
	err  = (int)(data[4]<<24 | data[5]<<16 | data[6]<<8 | data[7]);
	step = (int)(data[8]<<24 | data[9]<<16 | data[10]<<8 | data[11]);
	PD_T = data[12]<<24 | data[13]<<16 | data[14]<<8 | data[15];
	printVal("time: ", time);
	printVal("err: ", err);
	printVal("val1: ", val1);
	printVal("val3: ", val3);
	printVal("val2: ", val2);
}