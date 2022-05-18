#include "testSerial.h"

testSerial::testSerial(Stream &p) : port(p) {
	
}

testSerial::~testSerial(){
	
}

void testSerial::printTest()
{
	for(int i=0; i<100; i++){
		port.println(i);
		delay(100);
	}
	
}