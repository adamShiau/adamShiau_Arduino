/*Developed by:
/ Harrison Stahl https://github.com/Harryman
/ The Everything Corp
/ If you find this useful please pitch in: 1Coaxrz39TwnPkuQDDioDimNbMbKKoeQEV 
 ^^^^^^^^^^Include when copying^^^^^^^^^^^^^^

 This is a quick example to test and make sure you have everything hooked up right
 Simply writes to a few address and reads them back to see if it actually worked
 */

#include "wiring_private.h"
 #include <SPIPRM.h>
 #include <SPI.h>

#define SPI_CLOCK_8M 8000000
#define CHIP_SELECT_PIN 2


uint16_t var1 = 11111;
uint16_t var2 = 22222;
uint16_t addr1 = 1000;
uint16_t addr2 = 10000;

SPIClass mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);

SPIPRM prm(mySPI, 128, 65536, 2);



void setup(){
	

	mySPI.begin();
	mySPI.beginTransaction(SPISettings(SPI_CLOCK_8M, MSBFIRST, SPI_MODE0));
	pinPeripheral(3, PIO_SERCOM_ALT);
	pinPeripheral(22, PIO_SERCOM_ALT);
	pinPeripheral(23, PIO_SERCOM_ALT);

	prm.SPIset();

	Serial.begin(9600);
}

void loop(){
	prm.writeStart(addr1);
	prm.write16(var1);
	prm.endCmd();
	prm.readStart(addr1);
	if(var1 == prm.read16()){
		Serial.println('address 1 data 1 good');
	}
	if(var2 == prm.read16()){
		Serial.println('address 1 data 2 good');
	}
	prm.endCmd();
	prm.writeStart(addr2);
	prm.write16(var1);
	prm.write16(var2);
	prm.endCmd();
	prm.readStart(addr2);
	if(var1 == prm.read16()){
		Serial.println('address 2 data 1 good');
	}
	if(var2 == prm.read16()){
		Serial.println('address 2 data 2 good');
	}
	prm.endCmd();
	delay(2000);
}
