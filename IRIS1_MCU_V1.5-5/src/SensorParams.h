#ifndef SENSORPARAMS_H
#define SENSORPARAMS_H

#include <string>
#include <array>
#include <algorithm>
#include <cmath>

#define PI          3.1415926535897932384626433832795
#define HALF_PI     1.5707963267948966192313216916398
#define TWO_PI      6.283185307179586476925286766559
#define DEG_TO_RAD  0.017453292519943295769236907684886
#define RAD_TO_DEG  57.295779513082320876798154814105
#define EULER       2.718281828459045235360287471352

#ifndef radians
#define radians(deg) ((deg)*DEG_TO_RAD)
#endif

#ifndef degrees
#define degrees(rad) ((rad)*RAD_TO_DEG)
#endif

//----------------------------------------------------------------//
//                           IMU Parameters                       //
//----------------------------------------------------------------//
enum Sensor_ID{
    Xsens, 
    Nano33,
    Nano33Sense,
    AR_1A_EC,
    AR_1A_UY,
    MU_1A_AC,
    MU_2A_AC,
    IRIS
};

class SensorParams {
    public:
        float BR;
        float noise;
        float RRW;
        float std;
        float BS;
        std::string sensor_type;
        SensorParams(float BR = 0, float noise = 0, float RRW = 0, float std = 0, float BS = 0, std::string sensor_type = "gyro");
};

class IMUParams {
    public:
        std::array<SensorParams, 3> gyro;  // Using array to hold gyro data for 3 axes
        std::array<SensorParams, 3> accl;   // Using array to hold accl data for 3 axes

        IMUParams(const SensorParams& gyro, const SensorParams& accl);
        void setGYRO(const SensorParams& gyro, int axis);
        void setACCL(const SensorParams& accl, int axis);
        std::array<float, 3> getGYRO_BR() const;
        std::array<float, 3> getGYRO_BS() const;
        std::array<float, 3> getGYRO_STD() const;
        std::array<float, 6> getGYRO_6params() const;
        std::array<float, 4> getGYRO_4params() const;
        std::array<float, 3> getARW() const;
        std::array<float, 6> getACCL_6params() const;
        std::array<float, 3> getACCL_BR() const;
        std::array<float, 3> getACCL_BS() const;
        std::array<float, 3> getACCL_STD() const;
        std::array<float, 12> getIMU_12params() const;
};

IMUParams getSensorParams(Sensor_ID id);


#endif