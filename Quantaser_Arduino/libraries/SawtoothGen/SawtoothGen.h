#ifndef SAWTOOTHGEN_H
#define SAWTOOTHGEN_H
#include <Arduino.h>
#define AI_MAX 1023
#define AI_MIN 0

class SawtoothGen 
{
	private :
		uint8_t p_outputsense, p_chargecontrol;
		
	public :
		SawtoothGen(uint8_t, uint8_t);
		void Sawtooth_out();
	
};
#endif 
