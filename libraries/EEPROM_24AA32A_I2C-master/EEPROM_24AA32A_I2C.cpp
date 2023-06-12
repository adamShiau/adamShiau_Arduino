/*
Created by: 	Guillermo sampallo
Last updated:	08/09/2020
gsampallo.com

Use to save data to eeprom memory 24LC32A via I2C
Based on https://www.hobbytronics.co.uk/eeprom-page-write
*/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM_24AA32A_I2C.h>

using namespace std;

EEPROM_24AA32A_I2C::EEPROM_24AA32A_I2C(int eeprom_addres, TwoWire &p) : eeprom(eeprom_addres), myWire(p)
{

}

EEPROM_24AA32A_I2C::EEPROM_24AA32A_I2C(TwoWire &p) : myWire(p)
{

}

void EEPROM_24AA32A_I2C::begin() {
    myWire.begin();
}

void EEPROM_24AA32A_I2C::Parameter_Read(unsigned int eeaddress, unsigned char* buf) {
	unsigned char i=0;
	eeaddress <<= 2;
	myWire.beginTransmission(eeprom);
	myWire.write(eeaddress >> 8);   // MSB
	myWire.write(eeaddress & 0xFF); // LSB
	myWire.endTransmission();
	myWire.requestFrom(eeprom, 4);
	while(myWire.available()) buf[i++] = myWire.read();
}

void EEPROM_24AA32A_I2C::Read(unsigned int eeaddress, unsigned char* buf) {
	myWire.beginTransmission(eeprom);
	myWire.write(eeaddress >> 8);   // MSB
	myWire.write(eeaddress & 0xFF); // LSB
	myWire.endTransmission();
	myWire.requestFrom(eeprom, 1);
	while(myWire.available()) *buf = myWire.read();;
}

void EEPROM_24AA32A_I2C::read(unsigned int eeaddress, unsigned char* data, unsigned int num_chars) {
  unsigned char i=0;
  myWire.beginTransmission(eeprom);
  myWire.write(eeaddress >> 8);   // MSB
  myWire.write(eeaddress & 0xFF); // LSB
  myWire.endTransmission();
 
  myWire.requestFrom(eeprom,num_chars);
 
  while(myWire.available()) data[i++] = myWire.read();

}

/***
 * eeaddress << 2 for 32bit write
 */
void EEPROM_24AA32A_I2C::Parameter_Write(unsigned int eeaddress, int value) {
	eeaddress <<= 2;
	myWire.beginTransmission(eeprom);
	myWire.write(eeaddress >> 8);   // MSB
	myWire.write(eeaddress & 0xFF); // LSB
	myWire.write(value);
	myWire.write(value>>8);
	myWire.write(value>>16);
	myWire.write(value>>24);
	myWire.endTransmission();
	delay(5);
}

void EEPROM_24AA32A_I2C::Write(unsigned int eeaddress, char value) {
	myWire.beginTransmission(eeprom);
	myWire.write(eeaddress >> 8);   // MSB
	myWire.write(eeaddress & 0xFF); // LSB
	myWire.write(value);
	myWire.endTransmission();
	delay(5);
}

void EEPROM_24AA32A_I2C::write(unsigned int eeaddress, char* data) {
  // Uses Page Write for 24LC256
  // Allows for 64 byte page boundary
  // Splits string into max 16 byte writes
  unsigned char i=0, counter=0;
  unsigned int  address;
  unsigned int  page_space;
  unsigned int  page=0;
  unsigned int  num_writes;
  unsigned int  data_len=0;
  unsigned char first_write_size;
  unsigned char last_write_size;  
  unsigned char write_size;  
  
  // Calculate length of data
  do{ data_len++; } while(data[data_len]);   
   
  // Calculate space available in first page
  page_space = int(((eeaddress/64) + 1)*64)-eeaddress;

  // Calculate first write size
  if (page_space>16){
     first_write_size=page_space-((page_space/16)*16);
     if (first_write_size==0) first_write_size=16;
  }   
  else 
     first_write_size=page_space; 
    
  // calculate size of last write  
  if (data_len>first_write_size) 
     last_write_size = (data_len-first_write_size)%16;   
  
  // Calculate how many writes we need
  if (data_len>first_write_size)
     num_writes = ((data_len-first_write_size)/16)+2;
  else
     num_writes = 1;  
     
  i=0;   
  address=eeaddress;
  for(page=0;page<num_writes;page++) 
  {
     if(page==0) write_size=first_write_size;
     else if(page==(num_writes-1)) write_size=last_write_size;
     else write_size=16;
  
     myWire.beginTransmission(eeprom);
     myWire.write((int)((address) >> 8));   // MSB
     myWire.write((int)((address) & 0xFF)); // LSB
     counter=0;
     do{ 
        myWire.write((byte) data[i]);
        i++;
        counter++;
     } while((data[i]) && (counter<write_size));  
     myWire.endTransmission();
     address+=write_size;   // Increment address for next write
     
     delay(6);  // needs 5ms for page write
  }
}
 

