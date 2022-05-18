#ifndef TESTSERIAL__H
#define TESTSERIAL__H

#include <Arduino.h>

// class testSerial {
  // public:
    // testSerial(Stream &p) : port(p) {
    // }
	
	// void printTest()
	// {
		// for(int i=0; i<100; i++){
			// port.println(i);
			// delay(100);
		// }
		
	// }

  // private:
    // Stream &port;
// };

class testSerial {
  public:
    testSerial(Stream &);
	~testSerial(void);
	
	void printTest(void);
	

  private:
    Stream &port;
};




#endif