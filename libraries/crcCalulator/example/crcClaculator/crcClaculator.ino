#include "crcCalculator.h"

crcCal myCRC;

uint8_t msg_8[4] = {0xFE, 0x8E, 0xFF, 0x55}, CRC_8;
uint8_t msg_32[32] = {0xFE, 0x81, 0xFF, 0x55, 0xBC, 0x64, 0x6C, 0x1E, 0x3C, 0x3F , 0xF9 , 0x81 , 0x3B , 0x3C , 0x1E, 0x78 , 0xBC, 0x06 , 0x5F , 0xB7 , 0xBB 
, 0x2F , 0xBD , 0x79 , 0x3F , 0x7E , 0x98 , 0x00 , 0x77 , 0x47 , 0x00 , 0x14 };
uint8_t data1[8] =  {0xFE, 0x81, 0xFF, 0x55, 0xBC, 0x64, 0x6C, 0x1E};
uint8_t data2[8] =  {0x3C, 0x3F, 0xF9, 0x81, 0x3B, 0x3C, 0x1E, 0x78};
uint8_t data3[8] =  {0xBC, 0x06, 0x5F, 0xB7, 0xBB, 0x2F, 0xBD, 0x79};
uint8_t data4[8] =  {0x3F, 0x7E, 0x98, 0x00, 0x77, 0x47, 0x00, 0x14};



uint32_t CRC_32;

int i = 0;
unsigned int t_old, t_new;
float t_cal = 0;

void setup() {
  Serial.begin(115200);
  
}


void loop() {

  // t_old = micros();
  // for(i=0; i<100; i++){
  //   CRC_8 = myCRC.crc_8(msg_8, sizeof(msg_8));
  // }
  // t_new = micros();
  // t_cal = (float)(t_new - t_old)/100.0;

  // Serial.print(CRC_8, HEX);
  // Serial.print(", ");
  // Serial.println(t_cal);

  t_old = micros();
  for(i=0; i<100; i++){
    uint8_t* data_all = (uint8_t*)malloc(32*sizeof(uint8_t));
    memcpy(data_all, data1, 8*sizeof(uint8_t));
    memcpy(data_all+8, data2, 8*sizeof(uint8_t));
    memcpy(data_all+16, data3, 8*sizeof(uint8_t));
    memcpy(data_all+24, data4, 8*sizeof(uint8_t));
    CRC_32 = myCRC.crc_32(data_all, 32);
    free(data_all);
  }
  t_new = micros();
  t_cal = (float)(t_new - t_old)/100.0;

  // Serial.println(sizeof(data_all));
  Serial.print(CRC_32, HEX);
  Serial.print(", ");
  Serial.println(t_cal);

  t_old = micros();
  for(i=0; i<100; i++){
    CRC_32 = myCRC.crc_32(msg_32, sizeof(msg_32));
  }
  t_new = micros();
  t_cal = (float)(t_new - t_old)/100.0;

  Serial.print(CRC_32, HEX);
  Serial.print(", ");
  Serial.println(t_cal);
  Serial.println();

}