#include <Arduino.h>
#include <Wire.h>
#include <EEPROM_25LC512_SPI.h>

using namespace std;


EEPROM_25LC512_SPI::EEPROM_25LC512_SPI(SPIClass &p, char ss) : mySPI(p)
{
   _ss = ss;
}

void EEPROM_25LC512_SPI::init() { //pinMode 
   pinMode(_ss, OUTPUT);
	digitalWrite(_ss, HIGH);
}  

void EEPROM_25LC512_SPI::Parameter_Read(unsigned int eeaddress, unsigned char* buf) {
	unsigned char i;
	eeaddress <<= 2;

   digitalWrite(_ss, LOW);
	mySPI.transfer(EEPROM_25LC512_READ);
   mySPI.transfer(eeaddress>>8);
   mySPI.transfer(eeaddress & 0xff);
   for(i=0; i<4; i++) buf[i] = mySPI.transfer(0x00);
   
	digitalWrite(_ss, HIGH);
}

void EEPROM_25LC512_SPI::Info_Read(unsigned int eeaddress, byte *buf, int num) {
	// unsigned char i;
	// eeaddress <<= 2;

   digitalWrite(_ss, LOW);
	mySPI.transfer(EEPROM_25LC512_READ);
   mySPI.transfer(eeaddress>>8);
   mySPI.transfer(eeaddress & 0xff);
   for(int i=0; i<num; i++) buf[i] = mySPI.transfer(0x00);
   
	digitalWrite(_ss, HIGH);
}

void EEPROM_25LC512_SPI::Read(unsigned int eeaddress, unsigned char* buf) {
	digitalWrite(_ss, LOW);
	mySPI.transfer(EEPROM_25LC512_READ);
   mySPI.transfer(eeaddress>>8);
   mySPI.transfer(eeaddress & 0xff);
   *buf = mySPI.transfer(0x00);
	digitalWrite(_ss, HIGH);
}

/***
 * eeaddress << 2 for 32bit write
 */
void EEPROM_25LC512_SPI::Parameter_Write(unsigned int eeaddress, int value) {
	eeaddress <<= 2;

   writeEN();
	digitalWrite(_ss, LOW);
	mySPI.transfer(EEPROM_25LC512_WRITE);
   mySPI.transfer(eeaddress>>8);
   mySPI.transfer(eeaddress & 0xff);
   mySPI.transfer(value);
   mySPI.transfer(value>>8);
   mySPI.transfer(value>>16);
   mySPI.transfer(value>>24);
	digitalWrite(_ss, HIGH);
	delay(5);
}

void EEPROM_25LC512_SPI::Info_Write(unsigned int eeaddress, byte *buf, int num) {
	// eeaddress <<= 2;

   writeEN();
	digitalWrite(_ss, LOW);
	mySPI.transfer(EEPROM_25LC512_WRITE);
   mySPI.transfer(eeaddress>>8);
   mySPI.transfer(eeaddress & 0xff);
   // mySPI.transfer(value);
   // mySPI.transfer(value>>8);
   // mySPI.transfer(value>>16);
   // mySPI.transfer(value>>24);
   for(int i=0;i<num;i++) mySPI.transfer(buf[i]);
   // mySPI.transfer(1);
   // mySPI.transfer(2);
   // mySPI.transfer(3);
   // mySPI.transfer(4);
	digitalWrite(_ss, HIGH);
	delay(5);
}

void EEPROM_25LC512_SPI::Write(unsigned int eeaddress, char value) {
   writeEN();
	digitalWrite(_ss, LOW);
	mySPI.transfer(EEPROM_25LC512_WRITE);
   mySPI.transfer(eeaddress>>8);
   mySPI.transfer(eeaddress & 0xff);
   mySPI.transfer(value);
	digitalWrite(_ss, HIGH);
   delay(5);
}

void EEPROM_25LC512_SPI::writeEN()
{
   digitalWrite(_ss, LOW);
   mySPI.transfer(EEPROM_25LC512_WREN);
   digitalWrite(_ss, HIGH);
}
 

