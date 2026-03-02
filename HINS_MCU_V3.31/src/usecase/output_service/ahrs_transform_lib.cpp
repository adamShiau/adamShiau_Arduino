#include "ahrs_transform_lib.h"

#include "../../app/app_state.h"  // global ahrs_attitude

void ahrs_transform_sensorVecToCase(const my_att_t* gyro_cali_dps,
                                    const my_att_t* accl_cali_mps2,
                                    my_sensor_t* sensor_cali)
{
    if (!gyro_cali_dps || !accl_cali_mps2 || !sensor_cali) return;

    my_att_t gyro_case = {};
    my_att_t acc_case  = {};

    ahrs_attitude.sensorVecToCase(gyro_cali_dps->float_val, gyro_case.float_val);
    ahrs_attitude.sensorVecToCase(accl_cali_mps2->float_val, acc_case.float_val);

    sensor_cali->fog.fogx.step.float_val = gyro_case.float_val[0];
    sensor_cali->fog.fogy.step.float_val = gyro_case.float_val[1];
    sensor_cali->fog.fogz.step.float_val = gyro_case.float_val[2];

    sensor_cali->adxl357.ax.float_val = acc_case.float_val[0];
    sensor_cali->adxl357.ay.float_val = acc_case.float_val[1];
    sensor_cali->adxl357.az.float_val = acc_case.float_val[2];
}
