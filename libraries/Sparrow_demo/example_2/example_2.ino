#include "wiring_private.h"
#include "SparrowParaDefine.h"
#define UART_SERIAL_5

#ifdef UART_SERIAL_5
Uart mySerial5 (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM0_Handler() 
{
	mySerial5.IrqHandler();
}
#endif
int cnt = 0;

void setup() {
  #ifdef UART_SERIAL_5
		mySerial5.begin(115200); //to Quantaser Sparrow
		pinPeripheral(5, PIO_SERCOM_ALT); //RX
		pinPeripheral(6, PIO_SERCOM_ALT); //TX
	#endif
	Serial.begin(230400);
	Serial1.begin(115200); // to PC
	delay(2000);
	gyroInitialize(50);
	flushInputBuffer();
	Serial.print("initial ava: ");
	Serial.println(mySerial5.available());
	startRead(1);

}

unsigned char data[6];
int t0 = 0, t1 = 0;


void loop() {
	// t0 = millis();
	if(mySerial5.available())
	{
		mySerial5.readBytes(data, 6);
		convertGyro(data);
		Serial1.write(data, 6);
	}
	
	// t1 = millis();
	// Serial.println(t1 - t0);
	// t0 = t1;

	cnt++;
  // delay(50);
  // Serial.println(cnt);
  // if(cnt==1000) setStop();
  // startRead(1);
  // delay(50);

}

void flushInputBuffer(void)
{
	while(mySerial5.available())
		mySerial5.read();
}

void convertGyro(byte data[6])
{
	int fog=0;
	
	fog = data[0]<<24 | data[1]<<16 | data[2]<<8 | data[3];
	
	// Serial.print(data[0], HEX);
	// Serial.print(", ");
	// Serial.print(data[1], HEX);
	// Serial.print(", ");
	// Serial.print(data[2], HEX);
	// Serial.print(", ");
	// Serial.print(data[3], HEX);
	// Serial.print(", ");
	// Serial.print(data[4], HEX);
	// Serial.print(", ");
	// Serial.print(data[5], HEX);
	// Serial.print(", ");
	Serial.println(fog);
	
};

void gyroInitialize(int dly)
{
	setOutputMode();
	delay(dly);
	setModOff();
	delay(dly);
	setGain1(6);
	delay(dly);
	setGain2(6);	
	delay(dly);
	setModHigh(4096);
	delay(dly);
	setModLow(0);
	delay(dly);
	setModFreq(125);
	delay(dly);	
	setPiVth(8191);
	delay(dly);
	setPolarity(1);
	delay(dly);
	setIgnor(32);
	delay(dly);
	setOffset(0);
	delay(dly);
	setStepVth(0);
	delay(dly);
	setAVG(4);
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

void sendComCmd(String cmd)
{
	mySerial5.println(cmd);
	Serial.println(cmd);
}

void startRead(byte ch)
{
  String cmd;

	cmd = readSPI_cmd + ANGULAR_V + String(ch);
	sendComCmd(cmd);
}

void setStop()
{
  String cmd;

	cmd =  readSPI_cmd + ANGULAR_V + Stop_cmd;
	sendComCmd(cmd);
}

void setOutputMode()
{
	String cmd;

	cmd = setSPI_cmd + OUTPUT_ADD + CLOSE_MODE;
	sendComCmd(cmd);
}


float setModHigh(int modHigh)
{
	String cmd;
	cmd = setSPI_cmd + MOD_HIGH_ADD + String(modHigh);
	sendComCmd(cmd);
	return modHigh*DAC_RATIO;
}
		
float setModLow(int modLow)
{
	String cmd;
	cmd = setSPI_cmd + MOD_LOW_ADD + String(modLow);
	sendComCmd(cmd);
	return modLow*DAC_RATIO;
}
	
float setModFreq(int modFreq)
{
	String cmd;
	cmd = setSPI_cmd + MOD_FREQ_ADD + String(modFreq);
	sendComCmd(cmd);
	return FREQ_RATIO/float(modFreq);
}
		
float setPiVth(int piVth)
{
	String cmd;
	cmd = setSPI_cmd + MOD_PiVth_ADD + String(piVth);
	sendComCmd(cmd);
	return DAC_RATIO*float(piVth);
}

void setPolarity(bool polarity)
{
	String cmd;
	cmd = setSPI_cmd + Polarity_ADD + String(polarity);
	sendComCmd(cmd);
}

void setIgnor(int ignor)
{
	String cmd;
	cmd = setSPI_cmd + IGNOR_ADD + String(ignor);
	sendComCmd(cmd);
}

float setOffset(int offset)
{
	String cmd;
	cmd = setSPI_cmd + OFFSET_ADD + String(offset);
	sendComCmd(cmd);
	return float(offset)*0.2442;
}
				
float setStepVth(int stepVth)
{
	String cmd;
	cmd = setSPI_cmd + StepVth_ADD + String(stepVth);
	sendComCmd(cmd);
	return float(stepVth) * 0.2442;
}

void setAVG(int inavg)
{
	String cmd;
	cmd = setSPI_cmd + AVG_ADD + String(inavg);
	sendComCmd(cmd);
}
	
	
void setGain1(int gain1pwr)
{
	String cmd;
	cmd = setSPI_cmd + GAIN1PWR_ADD + String(gain1pwr);
	sendComCmd(cmd);
}


void setGain2(int gain2pwr)
{
	String cmd;
	cmd = setSPI_cmd + GAIN2PWR_ADD + String(gain2pwr);
	sendComCmd(cmd);
}

void setResetLadder()
{
	String cmd;
	cmd = setSPI_cmd + RESET_LADDER + "1";
	sendComCmd(cmd);
	delay(50);
	cmd = setSPI_cmd + RESET_LADDER + "0";
	sendComCmd(cmd);
}

void setModOff()
{
	String cmd;
	cmd = setSPI_cmd + MOD_OFF_CMD;
	sendComCmd(cmd);
}	

void setFstIntegratorLimit()
{
	String cmd;
	cmd = setSPI_cmd + INTER_LIMIT_ADD + "8388607";
	sendComCmd(cmd);
}

void setUpperBand()
{
	String cmd;
	cmd = setSPI_cmd + UPPER_BAND_ADD + "1";
	sendComCmd(cmd);
}

void setLowerBand()
{
	String cmd;
	cmd = setSPI_cmd + LOWER_BAND_ADD + "-1";
	sendComCmd(cmd);
}
	




