#include "pig_v2.h"

#define CHECK_BYTE		170
#define CHECK_BYTE2		171
#define CHECK_BYTE3		172

PIG::PIG()
{
	
}

void PIG::init()
{
	Serial.begin(115200);
	Serial1.begin(115200);
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