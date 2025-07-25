#line 1 "C:\\Users\\user\\Documents\\Arduino\\MCU-GP-28-4-PD\\src\\SensorParams.cpp"
#include "SensorParams.h"

//----------------------------------------------------------------//
//                          IMU KF Parameters                     //
//----------------------------------------------------------------//

// Input BR, noise, RRW, sted, BS, type ("gyro" or "accl")
SensorParams::SensorParams(float BR, float noise, float RRW, float std, float BS, std::string sensor_type) {
    if (sensor_type == "gyro") {
        this->BR = radians(BR) / 3600.0;
        this->noise = radians(noise) / 60.0;
        this->RRW = radians(RRW) / pow(60.0, 3);
        this->std = radians(std) / 3600.0;
        this->BS = radians(BS) / 3600.0;
    } else if (sensor_type == "accl") {
        float ug = 9.8 * 1e-6;
        this->BR = BR * ug;
        this->noise = noise * ug;
        this->RRW = RRW * ug;
        this->std = std * ug;
        this->BS = BS * ug;
    }
    this->sensor_type = sensor_type;
}

// Constructor: Initializes with 3 copies of gyro and accl for each axis
IMUParams::IMUParams(const SensorParams& gyro, const SensorParams& accl) {
    this->gyro.fill(gyro);
    this->accl.fill(accl);
}

// Set GYRO for a specific axis (1-3 for x, y, z)
void IMUParams::setGYRO(const SensorParams& gyro, int axis) {
    if (axis >= 1 && axis <= 3) {
        this->gyro[axis - 1] = gyro;
    }
}

// Set ACCL for a specific axis (1-3 for x, y, z)
void IMUParams::setACCL(const SensorParams& accl, int axis) {
    if (axis >= 1 && axis <= 3) {
        this->accl[axis - 1] = accl;
    }
}

// Get GYRO BR values
std::array<float, 3> IMUParams::getGYRO_BR() const {
    return {gyro[0].BR, gyro[1].BR, gyro[2].BR};
}

// Get GYRO BS values
std::array<float, 3> IMUParams::getGYRO_BS() const {
    return {gyro[0].BS, gyro[1].BS, gyro[2].BS};
}

// Get GYRO std values
std::array<float, 3> IMUParams::getGYRO_STD() const {
    return {gyro[0].std, gyro[1].std, gyro[2].std};
}

// Get GYRO 6 parameters (noise and RRW for all axes)
std::array<float, 6> IMUParams::getGYRO_6params() const {
    std::array<float, 6> params;
    for (size_t i = 0; i < 3; ++i) {
        params[i] = gyro[i].noise;        // ARW
        params[i + 3] = gyro[i].RRW;      // RRW
    }
    return params;
}

// Get GYRO 4 parameters (noise and RRW for X and Y axes)
std::array<float, 4> IMUParams::getGYRO_4params() const {
    std::array<float, 4> params;
    for (int i = 0; i < 2; ++i) {
        params[i] = gyro[i].noise;        // ARW for X and Y
        params[i + 2] = gyro[i].RRW;      // RRW for X and Y
    }
    return params;
}

// Get ARW (noise) for all axes
std::array<float, 3> IMUParams::getARW() const {
    return {gyro[0].noise, gyro[1].noise, gyro[2].noise};
}

// Get ACCL 6 parameters (noise and RRW for all axes)
std::array<float, 6> IMUParams::getACCL_6params() const {
    std::array<float, 6> params;
    for (size_t i = 0; i < 3; ++i) {
        params[i] = accl[i].noise;        // ARW
        params[i + 3] = accl[i].RRW;      // RRW
    }
    return params;
}

// Get ACCL BR values
std::array<float, 3> IMUParams::getACCL_BR() const {
    return {accl[0].BR, accl[1].BR, accl[2].BR};
}

// Get ACCL BS values
std::array<float, 3> IMUParams::getACCL_BS() const {
    return {accl[0].BS, accl[1].BS, accl[2].BS};
}

// Get ACCL std values
std::array<float, 3> IMUParams::getACCL_STD() const {
    return {accl[0].std, accl[1].std, accl[2].std};
}

// Get IMU 12 parameters (noise and RRW for both gyro and accl on all axes)
std::array<float, 12> IMUParams::getIMU_12params() const {
    std::array<float, 12> params;
    size_t index = 0;

    // Get noise and RRW for gyro
    for (const auto& g : gyro) {
        params[index++] = g.noise;  // ARW
    }
    for (const auto& g : gyro) {
        params[index++] = g.RRW;    // RRW
    }

    // Get noise and RRW for accl
    for (const auto& a : accl) {
        params[index++] = a.noise;  // ARW
    }
    for (const auto& a : accl) {
        params[index++] = a.RRW;    // RRW
    }

    return params;
}

IMUParams getSensorParams(Sensor_ID id) {
    switch (id) {
        case Nano33:{
            SensorParams GYRO(250, 0.5, 45, 250, 250, "gyro");
            SensorParams ACCL(2000, 220, 8, 3000, 2000, "accl");
            IMUParams IMU(GYRO, ACCL);
            return IMU;
        }

        case Nano33Sense:{
            SensorParams GYRO(50, 0.366, 15, 150, 50, "gyro");
            SensorParams ACCL(560, 100, 15, 2, 560, "accl");
            IMUParams IMU(GYRO, ACCL);
            return IMU;
        }
           
        case Xsens:{
            SensorParams MIT7_GYRO(0, 0.15, 20, 100, 45, "gyro");
            SensorParams MTI7_ACCL(0, 40, 1, 500, 300, "accl");
            IMUParams MTI7_params(MIT7_GYRO, MTI7_ACCL);
            return MTI7_params;
        }
            
        case AR_1A_EC:{
            SensorParams ASM330_GYRO(0, 0.4, 7, 300, 20, "gyro");
            SensorParams ASM330_ACCL(0, 85, 3.4, 1000, 350, "accl");
            IMUParams ASM330_params(ASM330_GYRO, ASM330_ACCL);
            // SensorParams wz(0, 0.05, 2.7, 20.46, 2.5, "gyro");
            // ASM330_params.setGYRO(wz, 3);
            return ASM330_params;
        }

        case AR_1A_UY:{
            SensorParams ASM330_GYRO(0, 0.4, 7, 300, 20, "gyro");
            SensorParams ASM330_ACCL(0, 85, 3.4, 1000, 350, "accl");
            IMUParams ASM330_params(ASM330_GYRO, ASM330_ACCL);
            SensorParams wz(0, 0.05, 2.7, 20.46, 2.5, "gyro");
            ASM330_params.setGYRO(wz, 3);
            return ASM330_params;
        }

            
        default:{
            SensorParams old_nano33_gyro(0, degrees(0.001) * 60, 0, 0, 0, "gyro");
            SensorParams old_nano33_accl(0, 0, 0, 0.0007 / (9.8 * 1e-6) * 9.8, 0, "accl");
            IMUParams old_nano33(old_nano33_gyro, old_nano33_accl);
            return old_nano33;
        }
            
    }
}