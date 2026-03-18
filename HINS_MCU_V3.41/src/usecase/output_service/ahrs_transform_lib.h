#pragma once

#include "../../common.h"  // my_att_t, my_sensor_t

// #ifdef __cplusplus
// extern "C" {
// #endif

// Transform calibrated sensor vectors from sensor frame to case frame.
// - gyro_cali_dps: calibrated gyro vector (dps)
// - accl_cali_mps2: calibrated accel vector (m/s^2)
// - sensor_cali: updated in-place (fog.step and adxl357 axes)
void ahrs_transform_sensorVecToCase(const my_att_t* gyro_cali_dps,
                                    const my_att_t* accl_cali_mps2,
                                    my_sensor_t* sensor_cali);

// #ifdef __cplusplus
// } // extern "C"
// #endif
