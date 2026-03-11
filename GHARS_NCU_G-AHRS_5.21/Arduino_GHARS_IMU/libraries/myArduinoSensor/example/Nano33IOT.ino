#include "MyNavigation.h"
#include "myI2CSensor.h"

#define TIME_SCALE 0.9891
#define IS_OUTPUT_BIN true


Navigation::ComplementaryFilter my_cpf;
MyCRC myCRC;
uint32_t pre_time;
uint32_t t0;
SYSTEM_STATE sys_state = INITIALIZING;
OutputMode output_mode = OUT_MODE_MAVLINK;


void setup() {
  Serial.begin(115200);
  // while(!Serial) {blinkLED(sys_state);}
  pinMode(13, OUTPUT);


  // Initialize IMU
  if (!sensor.begin()){
    Serial.println("Failed to initialize IMU!");
    while (1) {blinkLED(sys_state);}
  }
  Serial.println("Connect to IMU");

  #ifdef ARDUINO_ARDUINO_NANO33BLE
  sensor.onInterrupt(ISR_readIMU);

  // Initialize BARO
  if (!baro.begin()){
      Serial.println("Failed to initialize BARO!");
      while (1) {blinkLED(sys_state);}
  }
  Serial.println("Connect to BARO");
  #endif

  // Initialize CPF Parameters
  my_cpf.setIMUError(Nano33, 100);
  my_cpf.setThresholdBySTD();
  my_cpf.setWindowSizeLC(100);
  my_cpf.setLevelingConstant(1);
  my_cpf.setEnableLC_Output(true);
  float pos[3] = {25.013332647853254, 121.22195612772002, 0};
  my_cpf.setPOS(pos);
  // my_cpf.setZupward(false);
  my_cpf.startLC();
  Serial.println("Initialize CPF"); 

  // Initialize variables
  t0 = micros();
  pre_time = (micros() - t0) * TIME_SCALE;
  sys_state = IMU_MEASURING;
  Serial.println("Start measuring...");
}


void loop() {
  INS();
  checkCommand();
  blinkLED(sys_state);
}

void INS() {
  //  calculate time
  my_data_3f omg, acc, ori, mag, new_omg, new_acc;
  my_data_u4 imu_time, temp, bar;

  imu_time.ulong_val = (micros() - t0) * TIME_SCALE;
  bar.float_val = NAN;
  mag.float_val[0] = NAN;
  mag.float_val[1] = NAN;
  mag.float_val[2] = NAN;

  uint32_t dt = imu_time.ulong_val - pre_time;
  if (dt > 0) {
      pre_time = imu_time.ulong_val;
      sensor.getIMUData(omg.float_val, acc.float_val);

      #ifdef ARDUINO_ARDUINO_NANO33BLE
      sensor.getMAGData(mag.float_val);
      baro.getBARData(bar.float_val);
      temp.float_val = baro.geTEMPData();
      #endif

      // calculate attitude
      my_cpf.run(imu_time.ulong_val * 1e-6, omg.float_val, acc.float_val);  
      my_cpf.getEularAngle(ori.float_val);
      my_cpf.getCaliRate(omg.float_val, new_omg.float_val);
      my_cpf.getCaliACC(acc.float_val, new_acc.float_val);
      
      if (output_mode == OUT_MODE_BIN){
        // transport data by byte
        uint8_t buffer[66];
        memcpy(buffer, HEADER, 2);
        memcpy(buffer + 2, imu_time.bin_val, 4);
        memcpy(buffer + 6, new_omg.bin_val, 12);
        memcpy(buffer + 18, acc.bin_val, 12);
        memcpy(buffer + 30, mag.bin_val, 12);
        memcpy(buffer + 42, ori.bin_val, 12);
        memcpy(buffer + 54, bar.bin_val, 4);
        memcpy(buffer + 58, temp.bin_val, 4);
        myCRC.calCRC(buffer, 66);
        Serial.write(buffer, 66);
      }
      else if (output_mode == OUT_MODE_STR){
        char buffer[255];
        int index = 0;
        index = appendValue2Str(buffer, 255, index, imu_time.ulong_val * 1e-6, 3);
        index = appendValues2Str(buffer, 255, index, omg.float_val, 3, 4);
        index = appendValues2Str(buffer, 255, index, acc.float_val, 3, 4);
        index = appendValues2Str(buffer, 255, index, ori.float_val, 3, 2);
        Serial.println(buffer);
      }
      
  }
  else{
      Serial.print("dt = ");
      Serial.println(dt, 4);
  }
}

void checkCommand(){
  if (Serial.available()){
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove any leading or trailing whitespace
    if (command == "STR"){
      output_mode = OUT_MODE_STR;
      Serial.println("Output mode: STR");
    }
    else if (command == "BIN"){
      output_mode = OUT_MODE_BIN;
      Serial.println("Output mode: BIN");
    }
  }
}


