/*
Created by: 	Guillermo sampallo
Last updated:	08/09/2020
gsampallo.com

Example of how to use library to read and write on 24LC32A via I2C.

Based on https://www.hobbytronics.co.uk/eeprom-page-write

*/
#include <EEPROM_24AA32A_I2C.h>
#include "wiring_private.h"

#define I2C_FAST_MODE     400000
/*** TwoWire Wire(&sercom, PIN_WIRE_SDA, PIN_WIRE_SCL);***/
TwoWire myWire(&sercom0, 27, 20);
void SERCOM0_Handler()
{
  myWire.onService();
}

EEPROM_24AA32A_I2C eprom = EEPROM_24AA32A_I2C(myWire);

typedef union
{
  float v;
  uint8_t b[sizeof(float)];
  uint32_t as_int;
}
cracked_float_t;

cracked_float_t my_f, my_f_r;
 
unsigned int addr0 = 0x0A0B, addr1 = 1;
unsigned char data[4], data1[4];
unsigned char temp[2];
char str_data[]={"hello Adam"};

void setup() {
  
  my_f.v = -1.23456;

  Serial.begin(9600);
  //I2C
  myWire.begin();
  myWire.setClock(I2C_FAST_MODE);
  pinPeripheral(27, PIO_SERCOM);
  pinPeripheral(20, PIO_SERCOM);
  // eprom.Parameter_Write(0,8300);
  // eprom.Parameter_Write(1,-8300);
}

void loop() {


  eprom.Parameter_Write(0,8300);
  // eprom.Parameter_Write(1,-8300);
  eprom.Parameter_Read(0,data);
  Serial.write(data, 4);
  Serial.println("");
  // eprom.Parameter_Read(1,data);
  // Serial.write(data, 4);
  // Serial.println("");
  delay(500);
  // Serial.println(my_f.as_int, HEX);
  // my_f_r.as_int = my_f.as_int;
  // Serial.println(my_f_r.v, 5);
  // delay(500);
}