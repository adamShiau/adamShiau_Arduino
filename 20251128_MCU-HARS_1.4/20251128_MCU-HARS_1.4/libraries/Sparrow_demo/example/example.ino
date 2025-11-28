#include "wiring_private.h"
#include "SparrowParaDefine.h"
#include "Sparrow_read.h"

Uart mySerial5 (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);

Sparrow_read sparrow(mySerial5);

void SERCOM0_Handler()
{
	mySerial5.IrqHandler();
}

void setup() {
    pinPeripheral(5, PIO_SERCOM_ALT); //RX
    pinPeripheral(6, PIO_SERCOM_ALT); //TX
    mySerial5.begin(115200); //to Quantaser Sparrow
	Serial.begin(230400);

	delay(1000);
	sparrow.gyroInitialize(50);
	sparrow.flushInputBuffer();
	Serial.print("initial buffer: ");
	Serial.println(mySerial5.available());
	sparrow.startRead(1);
}

int t0 = 0, t1 = 0;

void loop() {
	byte data[6];

	sparrow.readData(data);
	convertGyro(data);
}


void convertGyro(byte data[6])
{
	int fog=0;

	fog = data[0]<<24 | data[1]<<16 | data[2]<<8 | data[3];

	t1 = millis();
	Serial.print(fog);
	Serial.print(", ");
	Serial.println(t1 - t0);
	t0 = t1;

};

	




