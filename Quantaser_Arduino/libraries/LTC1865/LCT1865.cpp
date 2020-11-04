#include <LTC1865.h>

LTC1865::LTC1865()
{


}
void LTC1865::init(byte convpin, uint8_t firstch)
{
	_convpin = convpin;
	pinMode(_convpin, OUTPUT);
	digitalWrite(_convpin, LOW);

	if(firstch)
	{
		SPI.transfer(ADC_CH1_H);
		SPI.transfer(ADC_CH1_L);
	}
	else
	{
		SPI.transfer(ADC_CH0_H);
		SPI.transfer(ADC_CH0_L);	
	}
}

unsigned int LTC1865::Read(uint8_t nextch)
{
	unsigned int data;
	digitalWrite(_convpin, HIGH);
	delayMicroseconds(4);
	digitalWrite(_convpin, LOW);
	if (nextch)
	data = (SPI.transfer(ADC_CH1_H))<<8 | (SPI.transfer(ADC_CH1_L));
	else
	data = (SPI.transfer(ADC_CH0_H))<<8 | (SPI.transfer(ADC_CH0_L));
return data;
}