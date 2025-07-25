#line 1 "C:\\Users\\user\\Documents\\Arduino\\MCU-GP-28-4-PD\\src\\DataOutput.h"
#ifndef DATAOUTPUT_H
#define DATAOUTPUT_H

#include <ArduinoEigenDense.h>
#include <vector>

using namespace Eigen;


class Integration{
    private:
        float pre_time;
    public:
        std::vector<float> ang;
        Integration();
        std::vector<float> update(const float &time, float (&omg)[3]);
};


class AxesDefine{
  private:
    Matrix3f R_b2l;
  public:
    AxesDefine();
    void setInit(const float &R11, const float &R12, const float &R13, const float &R21, const float &R22, const float &R23, const float &R31, const float &R32, const float &R33);
    void update(float (&input)[3]);
    void reset();
    bool isRotationMatrix(const Matrix3f& matrix);
};

#endif