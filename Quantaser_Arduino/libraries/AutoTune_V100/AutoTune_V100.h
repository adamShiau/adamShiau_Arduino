#ifndef AUTOTUNE_H
#define AUTOTUNE_H

#include <Arduino.h>
//#include <DTC03_Slave_V300.h>

class DTC03;
//class AD5541;
//class LT1865;
class Atune
{
	public:		
		DTC03 dtc2;
		
		
		Atune(); //constructor
//		void autotune();
		void input_bias();
//		void output_bias(unsigned int, bool);
//		void RelaySwitchTime(unsigned long *, unsigned char *);
		
		
	private:
		
//		unsigned int p_DAC_in;
};


#endif
