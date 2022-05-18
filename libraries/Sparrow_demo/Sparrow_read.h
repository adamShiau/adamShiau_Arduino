#ifndef TESTSERIAL__H
#define TESTSERIAL__H

#include <Arduino.h>
#include "SparrowParaDefine.h"

class Sparrow_read {
	public :
		Sparrow_read(Stream &);
		~Sparrow_read(void);
		void flushInputBuffer(void);
		void gyroInitialize(int);
		void sendComCmd(String);
		void startRead(byte);
		void setStop(void);
		void setOutputMode(void);
		float setModHigh(int);
		float setModLow(int);
		float setModFreq(int);
		float setPiVth(int);
		void setPolarity(bool);
		void setIgnor(int);
		float setOffset(int);
		float setStepVth(int);
		void setAVG(int);
		void setGain1(int);
		void setGain2(int);
		void setResetLadder(void);
		void setModOff(void);
		void setFstIntegratorLimit(void);
		void setUpperBand(void);
		void setLowerBand(void);
		
	private:
		Stream &port;
	
};




#endif