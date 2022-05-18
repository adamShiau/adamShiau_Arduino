// class sensor {
  // public:
    // sensor(Stream &p) : port(p) {
    // }

    // void sayHello() {
      // port.println("Hello World");
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

#include "testSerial.h"
#include "wiring_private.h"
#define UART_SERIAL_5

#ifdef UART_SERIAL_5
Uart mySerial5 (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM0_Handler() 
{
	mySerial5.IrqHandler();
}
#endif


testSerial xs(mySerial5);

void setup() {
	#ifdef UART_SERIAL_5
		mySerial5.begin(115200); //to Quantaser Sparrow
		pinPeripheral(5, PIO_SERCOM_ALT); //RX
		pinPeripheral(6, PIO_SERCOM_ALT); //TX
	#endif
  Serial1.begin(115200);
  xs.printTest();
}

void loop() {
}