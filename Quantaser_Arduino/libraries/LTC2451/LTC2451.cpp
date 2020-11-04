#include <LTC2451.h>

LTC2451::LTC2451()
{}
void LTC2451::Init(unsigned char mode)
{
	switch(mode)
	{
		case 0:
			p_waitTime = 17;// 60Hz mode
			break;	
		case 1:
			p_waitTime = 35; //30Hz mode
			break;		
	}
	p_timeStart = millis();
	Wire.begin();
	Wire.beginTransmission(LTC2451ADD);
	Wire.write(mode);
	Wire.endTransmission();
}
unsigned int LTC2451::Read()
{
	bool i=0;
	unsigned char voltage[2];
	unsigned int vout, currentTime = millis();
	if ( abs(currentTime-p_timeStart)>=p_waitTime )
	{
		Wire.requestFrom(LTC2451ADD,2);
		while (Wire.available())
		{
			voltage[i]=Wire.read();
			i++;
		}		
		p_vout = voltage[0] << 8 | voltage[1];
		p_timeStart = currentTime;		
	}
	vout = p_vout;
	return vout;
}
bool LTC2451::SoftI2CInit(unsigned char sdapin, unsigned char sclpin, unsigned char mode)
{

	g_sdapin = sdapin;
	g_sclpin = sclpin;
	softi2c.init(g_sdapin,g_sclpin);	
	switch(mode)
	{
		case 0:
			p_waitTime = 17;// 60Hz mode
			break;	
		case 1:
			p_waitTime = 35; //30Hz mode
			break;		
	}
	p_timeStart = millis();	
	if(softi2c.start(LTC2451ADD_SWRITE))  
	{	
		softi2c.write(mode);
		softi2c.stop();
		return 1;
	}
	
	else return 0;
}
unsigned int LTC2451::SoftI2CRead()
{
	unsigned char i, data[2];
	unsigned int vout, currentTime = millis();
	
	if ( abs(currentTime-p_timeStart)>=p_waitTime )
	{
			if(softi2c.start(LTC2451ADD_SREAD)) 
		{
			for(i =0; i< 2; i++)
				{
					data[i] = softi2c.read(0);
				}
			softi2c.stop();
			p_vout = data[0] << 8 | data[1];		
		}	
		
		p_timeStart = currentTime;		
	}
	vout = p_vout;
	return vout;
}

