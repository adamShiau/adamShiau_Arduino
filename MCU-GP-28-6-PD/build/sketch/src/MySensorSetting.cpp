#line 1 "C:\\Users\\user\\Documents\\Arduino\\MCU-GP-28-4-PD\\src\\MySensorSetting.cpp"
#include "MySensorSetting.h"

Nano33IOT::Nano33IOT(): AccGyr(&Wire, LSM6DS3_ACC_GYRO_I2C_ADDRESS_LOW){}

void Nano33IOT::init(){
    // connect I2C
    Wire.begin();
    // connect to IMU
    AccGyr.begin();
    AccGyr.Set_X_FS(16.0f);
    AccGyr.Enable_X();
    AccGyr.Enable_G();
}

void Nano33IOT::getIMUData(float (&omg)[3], float (&acc)[3]){
    int32_t int_acc[3];
    int32_t int_omg[3];
    
    //load gyro and accl data
    AccGyr.Get_X_Axes(int_acc);
    AccGyr.Get_G_Axes(int_omg);
    for (int i=0;i<3;i++){
      acc[i] = float(int_acc[i]) / 1000 * 9.8;
      omg[i] = float(int_omg[i]) / 1000;
    }
}
