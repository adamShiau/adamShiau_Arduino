#include "mySPISensor.h"

const bool is_output_bin = false;
ulong t0;
my_data_u4 pre_time;
MyCRC myCRC; 

ADIS16505 adis16505;


void setup() {
  //Serial Part
  Serial.begin(115200);
  while(!Serial);
  delay(100);
    
//   SPI Part
  adis16505 = ADIS16505(10, 9, 8);  // CS, DRDY, RST
  adis16505.initSensor();
  delay(200);    
  attachInterrupt(digitalPinToInterrupt(9), printADIS16505, RISING);
  
  t0 = micros();
  pre_time.ulong_val = t0;
}

void loop() {
    //if print without interrupts
    // if (adis16505.checkDRDY()){
    //   printADIS16505();
    // }
}

void printADIS16505(){
  my_data_3f omg;
  my_data_3f acc;
  my_data_u4 temp;
  int32_t counter = 0;

  if (adis16505.readData(omg.float_val, acc.float_val, temp.float_val, counter)){
    pre_time.ulong_val = micros() - t0;
    
    if (is_output_bin){
      uint8_t buffer[38];  // 根據需要的總長度來分配buffer
      memcpy(buffer, HEADER, 2);
      memcpy(buffer + 2, pre_time.bin_val, 4);
      memcpy(buffer + 6, omg.bin_val, 12);
      memcpy(buffer + 18, acc.bin_val, 12);
      memcpy(buffer + 30, temp.bin_val, 4);
      myCRC.calCRC(buffer, 38);
      Serial.write(buffer, 38);
    }
    else{
      Serial.print(pre_time.ulong_val * 1e-6, 3);Serial.print(',');
      for (int i=0;i<3;i++){Serial.print(omg.float_val[i], 3);Serial.print(',');}
      for (int i=0;i<3;i++){Serial.print(acc.float_val[i], 3);Serial.print(',');}
      Serial.print(temp.float_val, 1);Serial.print(',');
      Serial.print(counter);
      Serial.println();
    }
  }
}
