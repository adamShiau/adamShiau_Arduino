#include "Sparrow_read.h"

Sparrow_read::Sparrow_read(Stream &p) : port(p)
{
	Serial.begin(230400);
}

Sparrow_read::~Sparrow_read(void) {
	
}

void Sparrow_read::readData(unsigned char data[6]) {
    while(!port.available()) {};
    port.readBytes(data, 6);
}

void Sparrow_read::flushInputBuffer()
{
	while(port.available())
		port.read();
}

void Sparrow_read::sendComCmd(String cmd)
{
	port.println(cmd);
	Serial.println(cmd);
}

void Sparrow_read::startRead(byte ch)
{
  String cmd;

	cmd = readSPI_cmd + ANGULAR_V + String(ch);
	sendComCmd(cmd);
}

void Sparrow_read::setStop()
{
  String cmd;

	cmd =  readSPI_cmd + ANGULAR_V + Stop_cmd;
	sendComCmd(cmd);
}

void Sparrow_read::setOutputMode()
{
	String cmd;

	cmd = setSPI_cmd + OUTPUT_ADD + CLOSE_MODE;
	sendComCmd(cmd);
}


float Sparrow_read::setModHigh(int modHigh)
{
	String cmd;
	cmd = setSPI_cmd + MOD_HIGH_ADD + String(modHigh);
	sendComCmd(cmd);
	return modHigh*DAC_RATIO;
}
		
float Sparrow_read::setModLow(int modLow)
{
	String cmd;
	cmd = setSPI_cmd + MOD_LOW_ADD + String(modLow);
	sendComCmd(cmd);
	return modLow*DAC_RATIO;
}
	
float Sparrow_read::setModFreq(int modFreq)
{
	String cmd;
	cmd = setSPI_cmd + MOD_FREQ_ADD + String(modFreq);
	sendComCmd(cmd);
	return FREQ_RATIO/float(modFreq);
}
		
float Sparrow_read::setPiVth(int piVth)
{
	String cmd;
	cmd = setSPI_cmd + MOD_PiVth_ADD + String(piVth);
	sendComCmd(cmd);
	return DAC_RATIO*float(piVth);
}

void Sparrow_read::setPolarity(bool polarity)
{
	String cmd;
	cmd = setSPI_cmd + Polarity_ADD + String(polarity);
	sendComCmd(cmd);
}

void Sparrow_read::setIgnor(int ignor)
{
	String cmd;
	cmd = setSPI_cmd + IGNOR_ADD + String(ignor);
	sendComCmd(cmd);
}

float Sparrow_read::setOffset(int offset)
{
	String cmd;
	cmd = setSPI_cmd + OFFSET_ADD + String(offset);
	sendComCmd(cmd);
	return float(offset)*0.2442;
}
				
float Sparrow_read::setStepVth(int stepVth)
{
	String cmd;
	cmd = setSPI_cmd + StepVth_ADD + String(stepVth);
	sendComCmd(cmd);
	return float(stepVth) * 0.2442;
}

void Sparrow_read::setAVG(int inavg)
{
	String cmd;
	cmd = setSPI_cmd + AVG_ADD + String(inavg);
	sendComCmd(cmd);
}
	
	
void Sparrow_read::setGain1(int gain1pwr)
{
	String cmd;
	cmd = setSPI_cmd + GAIN1PWR_ADD + String(gain1pwr);
	sendComCmd(cmd);
}


void Sparrow_read::setGain2(int gain2pwr)
{
	String cmd;
	cmd = setSPI_cmd + GAIN2PWR_ADD + String(gain2pwr);
	sendComCmd(cmd);
}

void Sparrow_read::setResetLadder()
{
	String cmd;
	cmd = setSPI_cmd + RESET_LADDER + "1";
	sendComCmd(cmd);
	delay(50);
	cmd = setSPI_cmd + RESET_LADDER + "0";
	sendComCmd(cmd);
}

void Sparrow_read::setModOff()
{
	String cmd;
	cmd = setSPI_cmd + MOD_OFF_CMD;
	sendComCmd(cmd);
}	

void Sparrow_read::setFstIntegratorLimit()
{
	String cmd;
	cmd = setSPI_cmd + INTER_LIMIT_ADD + "8388607";
	sendComCmd(cmd);
}

void Sparrow_read::setUpperBand()
{
	String cmd;
	cmd = setSPI_cmd + UPPER_BAND_ADD + "1";
	sendComCmd(cmd);
}

void Sparrow_read::setLowerBand()
{
	String cmd;
	cmd = setSPI_cmd + LOWER_BAND_ADD + "-1";
	sendComCmd(cmd);
}

void Sparrow_read::gyroInitialize(int dly)
{
	setOutputMode();
	delay(dly);
	setModOff();
	delay(dly);
	setGain1(5);
	delay(dly);
	setGain2(4);
	delay(dly);
	setModHigh(3869);
	delay(dly);
	setModLow(0);
	delay(dly);
	setModFreq(170);
	delay(dly);	
	setPiVth(8191);
	delay(dly);
	setPolarity(1);
	delay(dly);
	setIgnor(25);
	delay(dly);
	setOffset(0);
	delay(dly);
	setStepVth(0);
	delay(dly);
	setAVG(6);
	delay(dly);
	setFstIntegratorLimit();
	delay(dly);
	setUpperBand();
	delay(dly);
	setLowerBand();
	delay(dly);
	setResetLadder();
	delay(dly);
}