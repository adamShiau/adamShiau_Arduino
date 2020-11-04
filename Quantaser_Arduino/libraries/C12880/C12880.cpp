#include "Arduino.h"

#include <C12880.h>

C12880::C12880()
{}

void C12880::SpectroInit()
{
	pinMode(PIN_CLKA, OUTPUT);
	pinMode(PIN_STA, OUTPUT);
	pinMode(PIN_CLKB, OUTPUT);
	pinMode(PIN_STB, OUTPUT);
	digitalWrite(PIN_CLKA, HIGH);
	digitalWrite(PIN_STA, LOW);
	digitalWrite(PIN_CLKB, HIGH);
	digitalWrite(PIN_STB, LOW);
}

void C12880::LEDInit(unsigned char led1, unsigned char led2)
{
	guc_led1 = led1;
	guc_led2 = led2;
	pinMode(guc_led1, OUTPUT);
	pinMode(guc_led2, OUTPUT);
	digitalWrite(guc_led1, LOW);
	digitalWrite(guc_led2, LOW);
}

void C12880::StartLED(unsigned int delaytime)
{
	digitalWrite(guc_led1,HIGH);
	digitalWrite(guc_led2,HIGH);
	delayMicroseconds(delaytime);
}

void C12880::PulseClkA(unsigned long pulse)
{
	unsigned int i;
	for (i = 0; i < pulse; i++)
	{
		PORTC |= CLKA_HIGH;
		PORTC &= CLKA_LOW;
	}
}

void C12880::PulseClkB(unsigned long pulse)
{
	unsigned int i;
	for (i = 0; i < pulse; i++)
	{
		PORTC |= CLKB_HIGH;
		PORTC &= CLKB_LOW;
	}
}

void C12880::PulseClkAB(unsigned long pulse)
{
	unsigned int i;
	for (i = 0; i < pulse; i++)
	{
		PORTC |= CLKAB_HIGH;
		PORTC &= CLKAB_LOW;
	}
}

void C12880::StartIntegAB()
{
	PORTC |= STA_HIGH;
	PORTC |= STB_HIGH; 
}

void C12880::StopIntegA()
{
	PORTC &= ~STA_HIGH;
}

void C12880::StopIntegB()
{
	PORTC &= ~STB_HIGH;
}

#if DEBUG_MODE	//2 for byte
void C12880::ReadVedioAB(byte *buffer)
{
	unsigned int i, low, high;
	for (i=0; i < CHANNEL_NUMBER*4; i++)
	{
		// read A
		ADMUX = ADC_READA;
		ADCSRA |= B01000000;
		while(ADCSRA & B01000000);
		low = ADCL;
		high = ADCH;
		buffer[i] = low;
		buffer[++i] = high;
		// read B
		ADMUX = ADC_READB;
		ADCSRA |= B01000000;
		while(ADCSRA & B01000000);
		low = ADCL;
		high = ADCH;
		buffer[++i] = low;
		buffer[++i] = high;
		PulseClkAB(1);
	}
}
#else //2 for byte and print
void C12880::ReadVedioAB(uint8_t ucPrintMode, File myFile)
{
	unsigned int i, low, high, j;
	byte data[384];	//288/3*4

  if (ucPrintMode == WriteSD)
  {
	for (j=0; j < 3; j++)
	{
	  for (i=0; i < 384; i++)
	  {
		// read A
		ADMUX = ADC_READA;
		ADCSRA |= B01000000;
		while(ADCSRA & B01000000);
		low = ADCL;
		high = ADCH;
		data[i] = low;
		data[++i] = high;
		// read B
		ADMUX = ADC_READB;
		ADCSRA |= B01000000;
		while(ADCSRA & B01000000);
		low = ADCL;
		high = ADCH;
		data[++i] = low;
		data[++i] = high;
		PulseClkAB(1);
	  }
	  myFile.write(data,384);
	}
  }
  else
  {
	for (i=0; i < CHANNEL_NUMBER; i++)
	{
		// read A
		ADMUX = ADC_READA;
		ADCSRA |= B01000000;
		while(ADCSRA & B01000000);
		low = ADCL;
		high = ADCH;
		if (ucPrintMode == WriteSerial)
		{
		  Serial.write(low);
		  Serial.write(high);
		}
		// read B
		ADMUX = ADC_READB;
		ADCSRA |= B01000000;
		while(ADCSRA & B01000000);
		low = ADCL;
		high = ADCH;
		if (ucPrintMode == WriteSerial)
		{
		  Serial.write(low);
		  Serial.write(high);
		}
		PulseClkAB(1);
	}
  }
}
#endif

void C12880::RunDevice(uint32_t I_timeA, uint32_t I_timeB, uint8_t ucPrintMode, File myFile)
{
  uint32_t t1 = 0, t2 = 0, ptime = 0;

  PulseClkAB(3);

#if DEBUG_MODE
  Serial.print("A time = ");
  Serial.println(I_timeA);
  Serial.print("B time = ");
  Serial.println(I_timeB);
#endif

  if (I_timeA == I_timeB)
  {
    I_timeBothAB = I_timeA;
  	ucFlagAB = ABSameTime;
  }
  else if (I_timeB > I_timeA)
  {
    I_timeBothAB = I_timeA;
    I_timeB -= I_timeA;
    ucFlagAB = BTimeBig2A;
  }
  else // if (I_timeA > I_timeB)
  {
    I_timeBothAB = I_timeB;
    I_timeA -= I_timeB;
    ucFlagAB = ATimeBig2B;
  }

  StartIntegAB();
#if DEBUG_MODE
  Serial.println("Start Integ AB");
#endif
  PulseClkAB(I_timeBothAB);
#if DEBUG_MODE
  Serial.print("Start AB clock = ");
  Serial.println(I_timeBothAB);
#endif

  if (ucFlagAB == ABSameTime)
  {
    StopIntegA();
    StopIntegB();
#if DEBUG_MODE
    Serial.println("Stop A and B Integ");
#endif
    PulseClkAB(PAUSE_NUMBER);
#if DEBUG_MODE
    Serial.print("Start AB clock = ");
    Serial.println(PAUSE_NUMBER);
#endif
  }
  else if (ucFlagAB == BTimeBig2A)
  {
    StopIntegA();
#if DEBUG_MODE
    Serial.println("Stop A Integ");
#endif

    if (I_timeB < PAUSE_NUMBER)
    {
      PulseClkAB(I_timeB);
#if DEBUG_MODE
      Serial.print("Start AB clock = ");
      Serial.println(I_timeB);
#endif

      StopIntegB();
#if DEBUG_MODE
      Serial.println("Stop B Integ");
#endif

      P_timeBothAB = PAUSE_NUMBER - I_timeB;
      PulseClkAB(P_timeBothAB);
#if DEBUG_MODE
      Serial.print("Start AB clock = ");
      Serial.println(P_timeBothAB);
#endif

      PulseClkB(I_timeB);
#if DEBUG_MODE
      Serial.print("Start B clock = ");
      Serial.println(I_timeB);
#endif
    }
    else // if (I_timeB > PAUSE_NUMBER)
    {
      PulseClkAB(PAUSE_NUMBER);
#if DEBUG_MODE
      Serial.print("Start AB clock = ");
      Serial.println(PAUSE_NUMBER);
#endif

      I_timeB -= PAUSE_NUMBER;
      PulseClkB(I_timeB);
#if DEBUG_MODE
      Serial.print("Start B clock = ");
      Serial.println(I_timeB);
#endif

      StopIntegB();
#if DEBUG_MODE
      Serial.println("Stop B Integ");
#endif

      PulseClkB(PAUSE_NUMBER);
#if DEBUG_MODE
      Serial.print("Start B clock = ");
      Serial.println(PAUSE_NUMBER);
#endif
    }
  }
  else // if (ucFlagAB == ATimeBig2B)
  {
    StopIntegB();
#if DEBUG_MODE
    Serial.println("Stop B Integ");
#endif

    if (I_timeA < PAUSE_NUMBER)
    {
      PulseClkAB(I_timeA);
#if DEBUG_MODE
      Serial.print("Start AB clock = ");
      Serial.println(I_timeA);
#endif

      StopIntegA();
#if DEBUG_MODE
      Serial.println("Stop A Integ");
#endif

      P_timeBothAB = PAUSE_NUMBER - I_timeA;
      PulseClkAB(P_timeBothAB);
#if DEBUG_MODE
      Serial.print("Start AB clock = ");
      Serial.println(P_timeBothAB);
#endif

      PulseClkA(I_timeA);
#if DEBUG_MODE
      Serial.print("Start A clock = ");
      Serial.println(I_timeA);
#endif
    }
    else // if (I_timeA > PAUSE_NUMBER)
    {
      PulseClkAB(PAUSE_NUMBER);
#if DEBUG_MODE
      Serial.print("Start AB clock = ");
      Serial.println(PAUSE_NUMBER);
#endif

      I_timeA -= PAUSE_NUMBER;
      PulseClkA(I_timeA);
#if DEBUG_MODE
      Serial.print("Start A clock = ");
      Serial.println(I_timeA);
#endif

      StopIntegA();
#if DEBUG_MODE
      Serial.println("Stop A Integ");
#endif

      PulseClkA(PAUSE_NUMBER);
#if DEBUG_MODE
      Serial.print("Start A clock = ");
      Serial.println(PAUSE_NUMBER);
#endif
    }
  }

  t1 = micros();
#if DEBUG_MODE
  ReadVedioAB(data);
#else
  ReadVedioAB(ucPrintMode, myFile);
#endif
  t2 = micros();
  ptime = t2 - t1;
  if (ucPrintMode == WriteSerial)
	Serial.println();
  Serial.print("Mode:");
  Serial.print(ucPrintMode);
  Serial.print(" time = ");
  Serial.println(ptime);

#if DEBUG_MODE
  Serial.println();
#endif
}

#if DEBUG_MODE
void C12880::PrintData()
{
  int i;

  for (i=0; i< CHANNEL_NUMBER*4; i++)
  {
    unsigned char n1 = 0, n2 = 0, n3 = 0, n4 = 0;
    n1 = data[i];
    n2 = data[++i];
    n3 = data[++i];
    n4 = data[++i];
    Serial.print(n1|(n2<<8));
    Serial.print(',');
    Serial.println(n3|(n4<<8));
  }
}
#endif

