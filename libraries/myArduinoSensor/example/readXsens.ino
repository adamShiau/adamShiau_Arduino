#include "myUARTSensor.h"


const bool is_output_bin = false;
ulong t0;
my_data_u4 pre_time;
MyCRC myCRC; 

XsensUart xsens(Serial1);


void setup() {
  //Serial Part
  Serial.begin(115200);
  while(!Serial);
  delay(100);

  Serial1.begin(230400);
  delay(100);
  xsens.setDataRate(200);
  xsens.ToConfigMode();
  xsens.AHRSData();
  xsens.InitMT();
  if (xsens.ToMeasurementMode()){};
  
  t0 = micros();
  pre_time.ulong_val = t0;
}

void loop() {
  printXsens();
}


void printXsens(){
  my_data_3f omg, acc, ori, mag, vel;
  my_data_4f qut;
  my_data_u4 XsensTime, temp, pressure, hei, status;
  my_data_2d latlon;
  my_data_u2 xsens_counter;
  
  xsens.getMeasures(MTDATA2);
  xsens.parseData(&xsens_counter, &XsensTime, &omg, &acc, &mag, &pressure, &vel, &latlon, &hei, &ori, &qut, &status, &temp);

  if(xsens.isAvailable()){
    if (is_output_bin){
      uint8_t buffer[62];  // 根據需要的總長度來分配buffer
      memcpy(buffer, HEADER, 2);
      memcpy(buffer + 2, XsensTime.bin_val, 4);
      memcpy(buffer + 6, omg.bin_val, 12);
      memcpy(buffer + 18, acc.bin_val, 12);
      memcpy(buffer + 30, mag.bin_val, 12);
      memcpy(buffer + 42, ori.bin_val, 12);
      memcpy(buffer + 54, temp.bin_val, 4);
      myCRC.calCRC(buffer, 62);
      Serial.write(buffer, 62);
    }
    else{
        char buffer[512];
        int index = 0;
        index = appendValue2Str(buffer, 512, index, XsensTime.ulong_val * 1e-4, 3);
        index = appendValues2Str(buffer, 512, index, latlon.float_val, 2, 7);
        index = appendValue2Str(buffer, 512, index, hei.float_val, 3);
        index = appendValues2Str(buffer, 512, index, vel.float_val, 3, 2);
        index = appendValues2Str(buffer, 512, index, qut.float_val, 4, 4);
        index = appendValues2Str(buffer, 512, index, omg.float_val, 3, 2);
        Serial.println(buffer);
    }
  }
}

