#include "acq_imu.h"


void reset_SYNC(); // define  in .ino
void acc_cali(float acc_cli[3], float acc[3]); // define  in .ino
void gyro_cali(float gyro_cli[3], float gyro[3]); // define  in .ino
void print_imu_data(bool on, float acc[3], float gyro[3]); // define  in .ino
void clear_SEL_EN(byte &select_fn); // define  in .ino

static byte *reg_fog;

void acq_imu(byte &select_fn, unsigned int value, byte ch)
{
    my_acc_t my_memsGYRO;
    my_float_t pd_temp;
    static my_float_t myfog_GYRO;
    static my_acc_t my_memsXLM, my_memsXLM_cali;
    static my_acc_t my_GYRO, my_GYRO_cali, my_att;

    byte *fog;
	uint8_t CRC32[4];

    if(select_fn&SEL_IMU)
	{
        CtrlReg = value;

        if(ch==1) run_fog_flag = sp13.setSyncMode(CtrlReg);
        else if(ch==2) run_fog_flag = sp14.setSyncMode(CtrlReg);
        else if(ch==3) run_fog_flag = sp9.setSyncMode(CtrlReg);

        switch(CtrlReg){
            case INT_SYNC:
                data_cnt = 0;
                EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
                eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
                setupWDT(11);
                enable_EXT_WDT(EXT_WDT_EN);
                reset_EXT_WDI(WDI);
                
            break;
            case EXT_SYNC:
                data_cnt = 0;
                Serial.println("Enter EXT_SYNC mode");
                Serial.println("Set EXTT to CHANGE");

                ahrs_attitude.captureYawZeroLocalCase();

                EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
                eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
                setupWDT(11);
                enable_EXT_WDT(EXT_WDT_EN);
                reset_EXT_WDI(WDI);
            break;

            case EXT_SYNC2:
                data_cnt = 0;
                Serial.println("Enter EXT_SYNC2 mode");
                Serial.println("Set EXTT to CHANGE");
                EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
                eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
                setupWDT(11);
                enable_EXT_WDT(EXT_WDT_EN);
                reset_EXT_WDI(WDI);
            break;

            case STOP_SYNC:
                reset_SYNC();
                data_cnt = 0;
                ahrs_attitude.resetAttitude(true);
                EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
                eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
                disableWDT();
                disable_EXT_WDT(EXT_WDT_EN);
            break;

            default:
            break;
        }
    }
    
    if(run_fog_flag) {
    fog = sp14.readData(header, sizeofheader, &try_cnt);
    if(fog) {
      reg_fog = fog;
      myfog_GYRO.bin_val[0] = reg_fog[11];
      myfog_GYRO.bin_val[1] = reg_fog[10];
      myfog_GYRO.bin_val[2] = reg_fog[9];
      myfog_GYRO.bin_val[3] = reg_fog[8];
    }

    if(ISR_PEDGE)
    {
      uint8_t* imu_data = (uint8_t*)malloc(36+12); // KVH_HEADER:4 + adxl355:9 + nano33_w:6 + nano33_a:6 + pig:14
      data_cnt++;
      mcu_time.ulong_val = millis() - t_previous;

      ISR_PEDGE = false;

      /*** get sensor raw data*/
      // IMU.Get_X_Axes_g_f(my_memsXLM.float_val);// get mems XLM data in m/s^2
      /*** ------get xlm raw data -----***/
      IMU.Get_X_Axes_f(my_memsXLM.float_val);// get mems XLM data in g
      /*** ------mis-alignment calibration xlm raw data -----***/
      acc_cali(my_memsXLM_cali.float_val, my_memsXLM.float_val);

      /*** ------get gyro raw data -----***/
      IMU.Get_G_Axes_f(my_memsGYRO.float_val);// get mems GYRO data in degree/s
      my_GYRO.float_val[0] = my_memsGYRO.float_val[0]; 
      my_GYRO.float_val[1] = my_memsGYRO.float_val[1];
      my_GYRO.float_val[2] = my_memsGYRO.float_val[2];
      // my_GYRO.float_val[2] = myfog_GYRO.float_val;
      // my_GYRO.float_val[2] = myfog_GYRO.float_val * DEG_TO_RAD;
      /*** ------mis-alignment calibration gyro raw data -----***/
      gyro_cali(my_GYRO_cali.float_val, my_GYRO.float_val);
      // my_GYRO_cali.float_val[0]*=-1; my_GYRO_cali.float_val[1]*=-1; my_GYRO_cali.float_val[2]*=-1;

      // --- 座標旋轉至輸出IMU顯示正確 --- 
      my_acc_t my_GYRO_case_frame, my_memsXLM_case_frame;
      ahrs_attitude.sensorVecToCase(my_GYRO_cali.float_val,     my_GYRO_case_frame.float_val);
      ahrs_attitude.sensorVecToCase(my_memsXLM_cali.float_val,  my_memsXLM_case_frame.float_val);
      // rotate2NED(my_GYRO_case_frame.float_val, my_GYRO_cali.float_val);
      // rotate2NED(my_memsXLM_case_frame.float_val, my_memsXLM_cali.float_val);

      print_imu_data(false, my_memsXLM_cali.float_val, my_GYRO_cali.float_val);

      memcpy(imu_data, KVH_HEADER, 4);
      // memcpy(imu_data+4, my_GYRO_cali.bin_val, 12);//wx, wy, wz
      // memcpy(imu_data+16, my_memsXLM_cali.bin_val, 12);//ax, ay, az
      memcpy(imu_data+4, my_GYRO_case_frame.bin_val, 12);//wx, wy, wz
      memcpy(imu_data+16, my_memsXLM_case_frame.bin_val, 12);//ax, ay, az
      memcpy(imu_data+28, MARS_PD_TEMP, 4);// PD temp
      memcpy(imu_data+32, mcu_time.bin_val, 4);
      memcpy(imu_data+36, my_att.bin_val, 12);
      myCRC.crc_32(imu_data, 48, CRC32);

      free(imu_data);
      
      #ifdef UART_RS422_CMD
      if(data_cnt >= DELAY_CNT)
      {
        Serial1.write(KVH_HEADER, 4);
        // Serial1.write(my_GYRO_cali.bin_val, 12);   //wx, wy, wz
        // Serial1.write(my_memsXLM_cali.bin_val, 12);//ax, ay, az
        Serial1.write(my_GYRO_case_frame.bin_val, 12);   //wx, wy, wz
        Serial1.write(my_memsXLM_case_frame.bin_val, 12);//ax, ay, az
        Serial1.write(MARS_PD_TEMP, 4);         // PD temp
        Serial1.write(mcu_time.bin_val, 4);
        Serial1.write(my_att.bin_val, 12);
        Serial1.write(CRC32, 4);
      }
      #endif  
      resetWDT(); 
      reset_EXT_WDI(WDI); 

      my_acc_t my_GYRO_att_calculate;
      if(abs(my_GYRO_cali.float_val[0]) > attitude_cali_coe._f.std_wx) my_GYRO_att_calculate.float_val[0] = my_GYRO_cali.float_val[0];
      else my_GYRO_att_calculate.float_val[0] = 0.0f;
      if(abs(my_GYRO_cali.float_val[1]) > attitude_cali_coe._f.std_wy) my_GYRO_att_calculate.float_val[1] = my_GYRO_cali.float_val[1];
      else my_GYRO_att_calculate.float_val[1] = 0.0f;
      if(abs(my_GYRO_cali.float_val[2]) > attitude_cali_coe._f.std_wz) my_GYRO_att_calculate.float_val[2] = my_GYRO_cali.float_val[2];
      else my_GYRO_att_calculate.float_val[2] = 0.0f;
      // Serial.print("std_wx: "); Serial.print(attitude_cali_coe._f.std_wx);
      // Serial.print(", std_wy: "); Serial.print(attitude_cali_coe._f.std_wy);
      // Serial.print(", std_wz: "); Serial.println(attitude_cali_coe._f.std_wz);

      ahrs_attitude.updateIMU(my_GYRO_att_calculate.float_val[0], my_GYRO_att_calculate.float_val[1], my_GYRO_att_calculate.float_val[2],
         my_memsXLM_cali.float_val[0], my_memsXLM_cali.float_val[1], my_memsXLM_cali.float_val[2]);
      

      float r = ahrs_attitude.getLocalCaseRoll();
      float p = ahrs_attitude.getLocalCasePitch();
      float y = ahrs_attitude.getLocalCaseYaw();
      my_att.float_val[0] =  p; // pitch
      my_att.float_val[1] =  r; // roll
      my_att.float_val[2] =  y; //
      // my_att.float_val[2] =  wrapDeg(y - yaw0); // yaw 減去零位，並 wrap 到 [-180,180]
    }
	}
	clear_SEL_EN(select_fn);
}