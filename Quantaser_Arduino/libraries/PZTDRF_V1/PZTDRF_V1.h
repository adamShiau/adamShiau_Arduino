#ifndef PZTDRF_V1_H
#define PZTDRF_V1_H
#include <Arduino.h>



class PZTDRF 
{
	public:	
		PZTDRF(uint8_t, uint8_t, uint8_t); 
		void PowerOn(float, uint8_t);
		void PowerOff(float, uint8_t); 
		void init();
		
	private:
		
		
		uint8_t p_voltStatusPin, p_dcbiasSw, p_drv2700En;
		bool p_powerOffFlag;
};


#endif
