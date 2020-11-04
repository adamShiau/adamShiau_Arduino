#include <LTC2615.h>

LTC2615::LTC2615(){}

void LTC2615::init()
{
	Wire.begin();
}

void LTC2615::write(uint8_t ch, float value)
{
	uint8_t t[2];
	uint16_t code;
	
	code = (uint16_t)(value/_ref*max);
	
	#ifdef DAC_BIT_14
		t[0] = (code >> 8)<<2 | ((uint8_t)code & 0b11000000)>>6; //high byte
		t[1] = (uint8_t)code << 2; //low byte
	#endif
	#ifdef DAC_BIT_16
		t[0] = code >> 8;
		t[1] = (uint8_t)code; 
	#endif
	// t[0] = (code >> 8)<<2 | ((uint8_t)code & 0b11000000)>>6;
	// t[1] = (uint8_t)code << 2;
	// Serial.println((CC << 4) | ch, BIN);
	// Serial.println(t[0],BIN);
	// Serial.println(t[1],BIN);
	Wire.beginTransmission(ADD);
	Wire.write((CC << 4) | ch);
	Wire.write(t,2); 
	Wire.endTransmission();
}

void LTC2615::writeint(uint8_t ch, unsigned int code)
{
	uint8_t t[2];
		
	#ifdef DAC_BIT_14
		t[0] = (code >> 8)<<2 | ((uint8_t)code & 0b11000000)>>6; //high byte
		t[1] = (uint8_t)code << 2; //low byte
	#endif
	#ifdef DAC_BIT_16
		t[0] = code >> 8;
		t[1] = (uint8_t)code; 
	#endif
	// t[0] = (code >> 8)<<2 | ((uint8_t)code & 0b11000000)>>6;
	// t[1] = (uint8_t)code << 2;
	// Serial.println((CC << 4) | ch, BIN);
	// Serial.println(t[0],BIN);
	// Serial.println(t[1],BIN);
	Wire.beginTransmission(ADD);
	Wire.write((CC << 4) | ch);
	Wire.write(t,2); 
	Wire.endTransmission();
}