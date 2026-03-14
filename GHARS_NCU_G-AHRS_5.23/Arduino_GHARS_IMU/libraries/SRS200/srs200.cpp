#include "srs200.h"

SRS200::SRS200()
{
	
}

void SRS200::init()
{

	mySerial5.begin(230400); //rx:p5, tx:p6
	/*** Reassign pins 5 and 6 to SERCOM alt ***/
	pinPeripheral(5, PIO_SERCOM_ALT); //RX
	pinPeripheral(6, PIO_SERCOM_ALT); //TX
}

void SRS200::readData(unsigned char header[2], unsigned char data[10])
{
	checkHeader(header);
	mySerial5.readBytes(data, 10);
}

unsigned char* SRS200::checkHeader(unsigned char headerArr[2])
{
	unsigned char header[2], hold;
	
	mySerial5.readBytes(headerArr, 2);
	hold = 1;
	while(hold)
	{
		if(	(headerArr[0] == HEADER[0]) && 
			(headerArr[1] == HEADER[1])
			){
				hold = 0;
				return headerArr ;
			}
		else {
			headerArr[0] = headerArr[1];
			headerArr[1] = mySerial5.read();
		}
	}
}