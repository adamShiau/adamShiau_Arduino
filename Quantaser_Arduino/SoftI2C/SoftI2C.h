#if ARDUINO < 100
#include <WProgram.h>
#else  // ARDUINO
#include <Arduino.h>
#endif  // ARDUINO

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef SoftI2C_H
#define SoftI2C_H

class SoftI2C {
private:
	uint8_t sdaport, sclport, _sdapin, _sclpin;
	uint8_t sdabit, sclbit;
	uint8_t timer, oldSREG;
	volatile uint8_t *sdaout, *sclout, *sdareg, *sclreg;

public:
	SoftI2C();
	void init(uint8_t sdapin, uint8_t sclpin);
	bool Write(uint8_t data);
	bool Start(uint8_t address, uint8_t RW);
	void Stop();
	uint8_t Read(uint8_t last);
private:
	void turnOffPWM(uint8_t timer);
};
#endif
