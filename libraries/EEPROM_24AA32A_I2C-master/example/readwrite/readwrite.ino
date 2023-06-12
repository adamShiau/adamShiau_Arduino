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
unsigned char data[4], tt_data;

/******SP13  *******/
#define MOD_FREQ_INIT_SP13 102
#define WAIT_CNT_INIT_SP13 50
#define ERR_AVG_INIT_SP13 5
#define MOD_AMP_H_INIT_SP13 4096
#define MOD_AMP_L_INIT_SP13 -4096
#define ERR_TH_INIT_SP13 0
#define ERR_OFFSET_INIT_SP13 0
#define POLARITY_INIT_SP13 0
#define CONST_STEP_INIT_SP13 16384
#define FPGA_Q_INIT_SP13 10
#define FPGA_R_INIT_SP13 104
#define GAIN1_INIT_SP13 6
#define GAIN2_INIT_SP13 5
#define FB_ON_INIT_SP13 1
#define DAC_GAIN_INIT_SP13 346
#define DATA_INT_DELAY_SP13 2220

#define TEST_ADD1 4094
#define TEST_ADD2 4095

void setup() {
  
  Serial.begin(115200);
  //I2C
  myWire.begin();
  myWire.setClock(I2C_FAST_MODE);
  pinPeripheral(27, PIO_SERCOM);
  pinPeripheral(20, PIO_SERCOM);
  eprom.Parameter_Write(0,MOD_FREQ_INIT_SP13);
  eprom.Parameter_Write(1,WAIT_CNT_INIT_SP13);
  eprom.Parameter_Write(2,ERR_AVG_INIT_SP13);
  eprom.Parameter_Write(3,MOD_AMP_H_INIT_SP13);
  eprom.Parameter_Write(4,MOD_AMP_L_INIT_SP13);
  eprom.Parameter_Write(5,ERR_TH_INIT_SP13);
  eprom.Parameter_Write(6,ERR_OFFSET_INIT_SP13);
  eprom.Parameter_Write(7,POLARITY_INIT_SP13);
  eprom.Parameter_Write(8,CONST_STEP_INIT_SP13);
  eprom.Parameter_Write(9,FPGA_Q_INIT_SP13);
  eprom.Parameter_Write(10,FPGA_R_INIT_SP13);
  eprom.Parameter_Write(11,GAIN1_INIT_SP13);
  eprom.Parameter_Write(12,GAIN2_INIT_SP13);
  eprom.Parameter_Write(13,FB_ON_INIT_SP13);
  eprom.Write(TEST_ADD1, 1);
  eprom.Write(TEST_ADD2, 0);

  /***
  my_f.float_val = -1.23456;
  eprom.Parameter_Write(2, my_f.int_val);
  */

}

void loop() {

  // eprom.Parameter_Read(2,data);
  // Serial.print(data[0], HEX);
  // Serial.print(", ");
  // Serial.print(data[1], HEX);
  // Serial.print(", ");
  // Serial.print(data[2], HEX);
  // Serial.print(", ");
  // Serial.println(data[3], HEX);

  // my_f_r = (my_float_t*) &data;   
  // Serial.println(my_f_r->float_val);  
  // delay(500);
  eprom.Parameter_Read(0,my_f.bin_val);
  Serial.println(my_f.int_val);
  eprom.Parameter_Read(3,my_f.bin_val);
  Serial.println(my_f.int_val);
  eprom.Parameter_Read(4,my_f.bin_val);
  Serial.println(my_f.int_val);
  eprom.Read(TEST_ADD1, &tt_data);
  Serial.println(tt_data);
  eprom.Read(TEST_ADD2, &tt_data);
  Serial.println(tt_data);
  Serial.println(" ");
  delay(500);

}