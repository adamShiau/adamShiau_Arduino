#ifndef TESTSERIAL__H
#define TESTSERIAL__H

class testSerial {
  public:
    testSerial(Stream &p) : port(p) {
    }

    void sayHello() {
      port.println("Hello World");
    }
	
	void printTest()
	{
		for(int i=0; i<100; i++){
			port.println(i);
			delay(100);
		}
		
	}

  private:
    Stream &port;
};



#endif