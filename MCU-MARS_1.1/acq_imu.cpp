#include "acq_imu.h"


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

        // if(ch==1) run_fog_flag = sp13.setSyncMode(CtrlReg);
        // else if(ch==2) run_fog_flag = sp14.setSyncMode(CtrlReg);
        // else if(ch==3) run_fog_flag = sp9.setSyncMode(CtrlReg);

    }
}