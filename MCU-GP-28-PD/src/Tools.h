#ifndef MYTOOLS_H
#define MYTOOLS_H

#include <ArduinoEigenDense.h>
#include <vector>

using namespace Eigen;


// generate gravity by position
float genGravity(const float &lat, const float &height);

//generate pitch and roll by acceleration measurement
Vector2f AccLeveling(const float &fx, const float &fy, const float &fz);

float normalizeAngle(float *angle, int size);

#endif