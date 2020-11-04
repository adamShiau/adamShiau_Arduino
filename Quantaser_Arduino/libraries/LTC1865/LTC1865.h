#include <Arduino.h>
#include <SPI.h>
// ============Revison History 1.01 @ 2016/10/21 by Tina =========
// Change class LTC1863 from LTC1865::LTC1865(byte convpin) to LTC1865::LTC1865()
// So that other class can use this conveniently.
// add to test

#define ADC_CH0_H 0x80
#define ADC_CH0_L 0x00
#define ADC_CH1_H 0xC0
#define ADC_CH1_L 0x00

class LTC1865
{
private: 
	uint8_t _convpin;

public:
	LTC1865();
	void init(byte convpin, uint8_t firstch);
	unsigned int Read(uint8_t nextch);
};
