#include "mySPISensor.h"


const bool is_output_bin = false;
ulong t0;
my_data_u4 pre_time;
MyCRC myCRC; 

GYPRO4300 tdk_gyro;


void setup() {
  //Serial Part
  Serial.begin(115200);
  while(!Serial);
  delay(100);

    // SPI Part
  tdk_gyro = GYPRO4300(14, 15, 16, 17, 8);
  tdk_gyro.initSensor();
  tdk_gyro.disableCalibrationMode();
  delay(200);
  attachInterrupt(digitalPinToInterrupt(15), printGYPRO4300, RISING);
  
  t0 = micros();
  pre_time.ulong_val = t0;
}

void loop() {
    // print without interrupts
    // if (adis16505.checkDRDY()){
    //   printADIS16505();
    // }
}

void printGYPRO4300(){
  my_data_u4 temp0;
  my_data_u4 omg0;
  if (tdk_gyro.readData(omg0.float_val, temp0.float_val)){
  pre_time.ulong_val = micros() - t0;
    if (is_output_bin){
      uint8_t buffer[18];  // 根據需要的總長度來分配buffer
      memcpy(buffer, HEADER, 2);
      memcpy(buffer + 2, pre_time.bin_val, 4);
      memcpy(buffer + 6, omg0.bin_val, 4);
      memcpy(buffer + 10, temp0.bin_val, 4);
      myCRC.calCRC(buffer, 18);
      Serial.write(buffer, 18);
    }
    else{
      Serial.print(pre_time.ulong_val * 1e-6, 5);Serial.print(',');
      Serial.print(omg0.float_val, 3); // Send outputs
      Serial.print(",");
      Serial.print(temp0.float_val, 3); // Send outputs
      Serial.println();
    }
  }
}
