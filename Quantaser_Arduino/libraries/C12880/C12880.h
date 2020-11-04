#include <SPI.h>
#include <SD.h>

#ifndef C12880_H
#define C12880_H

#define DEBUG_MODE	0

#define CLKPERIOD1	1 	// 1us
#define CLKPERIOD2	100 	// 100us


// =======define pin for the spectrometer ========
// Spectro A:
// CLK : pin 3
// STA : pin 4
// so use DDRD to fast manipulate
#define PIN_VEDIOA	A3
#define PIN_CLKA	A2
#define PIN_STA		A1

// Spectro B:
#define PIN_VEDIOB	A0
#define PIN_CLKB	A4
#define PIN_STB		A6

#define CHANNEL_NUMBER	288
#define PAUSE_NUMBER	87

#define CLKAB_HIGH	B00010100 // use | to implement clk high with PORTC
#define CLKAB_LOW	B11101011 // use & to implement clk low with PORTC

#define CLKA_HIGH 	B00000100 // use | to implement clk high with PORTC
#define CLKA_LOW 	B11111011 // use & to implement clk low with PORTC
#define STA_HIGH	B00000010 // use & to implement STA high with PORTC

#define CLKB_HIGH 	B00010000 // use | to implement clk high with PORTC
#define CLKB_LOW 	B11101111 // use & to implement clk low with PORTC
#define STB_HIGH	B01000000 // use & to implement STB high with PORTD

#define ADC_READA	B01000011 // set the reference to 5V, Set the result to the right adjust, ReadCh3
#define ADC_READB	B01000000 // set the reference to 5V, Set the result to the right adjust, ReadCh0

#define nop asm volatile ("nop\n\t") // use nop to tune the delay

#define SD_CSPIN	4

enum{
	ABSameTime = 0,
	ATimeBig2B,
	BTimeBig2A,
};

enum{
	NoPrint = 0,
	WriteSerial,
	WriteSD,
};

class C12880
{
private:
	
	unsigned char guc_led1, guc_led2;
	unsigned char guc_opst1h, guc_opst1l, guc_opst2h, guc_opst2l; // fast port manipulation constant

#if DEBUG_MODE
	byte data[CHANNEL_NUMBER*4];
#endif

	uint32_t I_timeBothAB = 0, P_timeBothAB = 0;
	uint8_t ucFlagAB = ABSameTime;
	uint8_t ucPrintMode = NoPrint;

public:
	C12880();

	void SpectroInit();
	void LEDInit(unsigned char led1, unsigned char led2);
	void StartLED(unsigned int delaytime);
	void PulseClkA(unsigned long pulse);
	void PulseClkB(unsigned long pulse);
	void PulseClkAB(unsigned long pulse);
	void StartIntegAB();
	void StopIntegA();
	void StopIntegB();
#if DEBUG_MODE
	void ReadVedioAB(byte *buffer);
#else
	void ReadVedioAB(uint8_t ucPrintMode, File myFile);
#endif
	void RunDevice(uint32_t I_timeA, uint32_t I_timeB, uint8_t ucPrintMode, File myFile);
#if DEBUG_MODE
	void PrintData();
#endif
};

#endif