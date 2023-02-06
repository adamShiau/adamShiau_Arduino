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
  float float_val;
  uint8_t bin_val[4];
  int int_val;
}
my_float_t;

my_float_t my_f;
// my_float_t my_f_r;
my_float_t *my_f_r = (my_float_t *) malloc(4);
 
unsigned int addr0 = 0x0A0B, addr1 = 1;
unsigned char data[4];

void setup() {
  
  Serial.begin(9600);
  //I2C
  myWire.begin();
  myWire.setClock(I2C_FAST_MODE);
  pinPeripheral(27, PIO_SERCOM);
  pinPeripheral(20, PIO_SERCOM);
  eprom.Parameter_Write(0,8300);
  eprom.Parameter_Write(1,-8300);
  my_f.float_val = -1.23456;
  eprom.Parameter_Write(2, my_f.int_val);
}

void loop() {

  eprom.Parameter_Read(2,data);
  Serial.print(data[0], HEX);
  Serial.print(", ");
  Serial.print(data[1], HEX);
  Serial.print(", ");
  Serial.print(data[2], HEX);
  Serial.print(", ");
  Serial.println(data[3], HEX);
  // my_f_r.bin_val[0] = data[0];
  // my_f_r.bin_val[1] = data[1];
  // my_f_r.bin_val[2] = data[2];
  // my_f_r.bin_val[3] = data[3];
  my_f_r = (my_float_t*) &data;  
  // Serial.print((long)my_f_r, HEX);
  // Serial.print(", ");
  // Serial.println((long)&data, HEX);
  // Serial.print(my_f_r.bin_val[0], HEX);
  // Serial.print(" ");
  // Serial.print(my_f_r.bin_val[1], HEX);
  // Serial.print(" ");
  // Serial.print(my_f_r.bin_val[2], HEX);
  // Serial.print(" ");
  // Serial.print(my_f_r.bin_val[3], HEX);
  // Serial.println(" ");
  // Serial.println(my_f_r.float_val);  
  Serial.println(my_f_r->float_val);  
  
  delay(500);
  // Serial.println(my_f.as_int, HEX);
  // my_f_r.as_int = my_f.as_int;
  // Serial.println(my_f_r.v, 5);
  // delay(500);
}