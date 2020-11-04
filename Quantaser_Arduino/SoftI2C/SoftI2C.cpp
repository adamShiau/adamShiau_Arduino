#include <SoftI2C.h>
#include <Arduino.h>

#define I2C_READ 1
#define I2C_WRITE 0
#define I2C_DELAY_USEC 2

SoftI2C::SoftI2C()
{}

void SoftI2C::init(uint8_t sdapin, uint8_t sclpin)
{
	_sdapin= sdapin;
	_sclpin= sclpin;
	// Check if the pin is PWM or not
	timer = digitalPinToTimer(_sdapin);
	if(timer != NOT_ON_TIMER) turnOffPWM(timer);
	timer = digitalPinToTimer(_sclpin);
	if(timer != NOT_ON_TIMER) turnOffPWM(timer);

	// Get the port and bit information
	sdaport =digitalPinToPort(_sdapin);
	sclport =digitalPinToPort(_sclpin);
	sdabit = digitalPinToBitMask(_sdapin);
	sclbit = digitalPinToBitMask(_sclpin);

	//get output port and reg information
	sdareg = portModeRegister(sdaport);
	sdaout = portOutputRegister(sdaport);
	sclreg = portModeRegister(sclport);
	sclout = portOutputRegister(sclport);
	
	oldSREG = SREG;		//Disable the interrupt
	cli();	
	// set the SDA and SCL pin as OUTPUT and set their level is HIGH		
	*sdareg |= sdabit;	//Set SDA OUTPUT
	*sclreg |= sclbit;	//Set SCL OUTPUT
	*sdaout |= sdabit; // Set SDA HIGH
	*sclout |= sclbit; // Set SCL HIGH

	SREG = oldSREG; // Enable the interrupt 
}

uint8_t SoftI2C::Read(uint8_t last)
{
	uint8_t b, i;
	b=0;
	oldSREG = SREG;		//Disable the interrupt
	cli();
	*sdaout |= sdabit; // Set SDA HIGH
	*sdareg &= ~sdabit;  // set SDA as input
	SREG = oldSREG; // Enable the interrupt
	for(i = 0; i <8; i++)
	{
		b <<=1;
		oldSREG = SREG;		//Disable the interrupt
		cli();
		*sclout |= sclbit; // Set SCL HIGH
		SREG = oldSREG;

		delayMicroseconds(I2C_DELAY_USEC);
		
		oldSREG = SREG;		//Disable the interrupt
		cli();
		*sclout |= sclbit; // Set SCL HIGH
		if(*portInputRegister(sdaport)&sdabit) b &=1;
		*sclout &= ~sclbit; // set SCL LOW
		SREG = oldSREG;
	} 
	// Send AK or NAK 
	oldSREG = SREG;		//Disable the interrupt
	cli();
	*sdareg |= sdabit;	//Set SDA OUTPUT
	if(last)  	*sdaout |= sdabit; // set SDA Hight
	else		*sdaout &= ~sdabit; // set SDA Low
	*sclout |= sclbit; // set SCL High
	SREG = oldSREG;
	delayMicroseconds(I2C_DELAY_USEC);
	oldSREG = SREG;
	cli();
	*sclout &= ~sclbit; // set SCL LOW
	*sdaout &= ~sdabit; // set SDA Low
	SREG = oldSREG;
	return b;
}
void SoftI2C::Stop()
{
	oldSREG = SREG;
 	cli();
 	*sdaout &= ~sdabit; // set SDA Low
 	SREG = oldSREG;
 	delayMicroseconds(I2C_DELAY_USEC);
 	oldSREG = SREG;
 	cli();
	*sclout |= sclbit; // set SCL High
 	SREG = oldSREG;
 	delayMicroseconds(I2C_DELAY_USEC);
 	oldSREG = SREG;
 	cli();
 	*sdaout |= sdabit; // set SDA Hight
 	SREG = oldSREG;
 	delayMicroseconds(I2C_DELAY_USEC);



}

 bool SoftI2C::Start(uint8_t address, uint8_t RW)
 {
 	uint8_t addRW;
 	addRW = ((address << 1) | RW);
 	oldSREG = SREG;
 	cli();
 	//*sdaout &= ~sdabit; // set SDA Low
 	digitalWrite(_sdapin,LOW);
	
 	delayMicroseconds(I2C_DELAY_USEC);
 	digitalWrite(_sclpin, LOW);
 	//*sclout &= ~sclbit; // set SCL LOW
 	SREG = oldSREG;
 	return Write(addRW);
 	
 }

bool SoftI2C::Write(uint8_t data)
{
	uint8_t m, oldSREG;
	

	for(m = 0x80; m !=0; m >>=1)
	{	
		
		oldSREG = SREG;
		cli();

		if( (m & data) == LOW)
				*sdaout &= ~sdabit; // set SDA Low

		
		else		
				*sdaout |= sdabit; // set SDA Hight
			
		*sclout |= sclbit; // set SCL High
		SREG = oldSREG;

		delayMicroseconds(I2C_DELAY_USEC);
		oldSREG = SREG;
		cli();
		*sclout &= ~sclbit; // set SCL LOW
		SREG = oldSREG;
	}
	//oldSREG = SREG;
	//cli();
	pinMode(_sdapin, INPUT);
	digitalWrite(_sdapin, HIGH);
	digitalWrite(_sclpin, HIGH);

	//*sdareg &= ~sdabit;  // set SDA as input
	//*sdaout |= sdabit; // set SDA High
	//*sclout |= sclbit; // set SCL High
	//SREG = oldSREG;
	delayMicroseconds(I2C_DELAY_USEC);
	uint8_t rtn =digitalRead(_sdapin);

	//uint8_t rtn = (*portInputRegister(sdaport))& sdabit;
	Serial.print("rtn:");
	Serial.println(rtn);

	//oldSREG = SREG;
	//cli();
	*sclout &= ~sclbit; // set SCL LOW
	*sdareg |= sdabit;	// set SDA Output
	*sdaout &= ~sdabit; // set SDA Low
	SREG = oldSREG;
	
	return (rtn == 0);
}
void SoftI2C::turnOffPWM(uint8_t timer)
{
	switch(timer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:   cbi(TCCR1A, COM1C1);    break;
		#endif
		
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   cbi(TCCR2, COM21);      break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0B1)
		case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2A1)
		case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
		#endif					
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
		#endif			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	cbi(TCCR4C, COM4D1);	break;
		#endif			
			
		#if defined(TCCR5A)
		case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
		case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
		case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
		#endif
	}
}

