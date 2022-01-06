#include "pig_v2.h"

#define DATA_OUT_START_ADDR		99

PIG pig_v2;

void setup() {
	pig_v2.init();
	delay(1000);
	pig_v2.sendCmd(DATA_OUT_START_ADDR, 1);
}

void loop() {
	
	byte *ret, data[9];
	
	ret = pig_v2.readData(data);
	
}