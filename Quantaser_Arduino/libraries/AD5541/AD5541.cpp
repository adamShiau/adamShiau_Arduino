//include <SPI.h>
#include <SPI.h>
#include <AD5541.h>

AD5541::AD5541()
{}
void AD5541::SetPin(unsigned char pin)
{
	_adcsel = pin;
}
void AD5541::init()
{

	SPI.begin();
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV2);
}
void AD5541::ModeWrite(unsigned int dacvalue) // with spi.begin() and spi.end(), Need to set bit order in setup
{
	unsigned char high_byte, low_byte;
	high_byte = dacvalue >>8;
	low_byte = dacvalue;
	SPI.setDataMode(SPI_MODE0);
	SPI.transfer(0);
	SPI.transfer(0);
	digitalWrite(_adcsel, LOW);
	SPI.transfer(high_byte);
	SPI.transfer(low_byte);
	digitalWrite(_adcsel, HIGH);

}
void AD5541::NormalWrite(unsigned int dacvalue) // Need to use spi setting in setup 
{
	unsigned char high_byte, low_byte;
	high_byte = dacvalue >>8;
	low_byte = dacvalue;
	SPI.transfer(0);
	SPI.transfer(0);
	digitalWrite(_adcsel, LOW);
	SPI.transfer(high_byte);
	SPI.transfer(low_byte);
	digitalWrite(_adcsel, HIGH);
}