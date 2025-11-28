#include "pig_v2.h"

/*** PIG SYNC mode***/
#define INT_SYNC	1
#define EXT_SYNC 	2

#define MOD_AMP_H_ADDR  		1
#define MOD_AMP_L_ADDR  		2
#define DATA_OUT_START_ADDR		99

PIG pig_v2;

void setup() {
	pig_v2.init();
	delay(1000);
	pig_v2.sendCmd(MOD_AMP_H_ADDR, 3000);
	pig_v2.sendCmd(MOD_AMP_L_ADDR, -3000);
	pig_v2.sendCmd(DATA_OUT_START_ADDR, INT_SYNC);
}

void loop() {
	
	byte data[16];
	
	pig_v2.readData_debug(data);	
}