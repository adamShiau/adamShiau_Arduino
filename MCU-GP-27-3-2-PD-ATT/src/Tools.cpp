#include "Tools.h"

using namespace Eigen;

float genGravity(const float &lat, const float &height){
    double a1 = 9.7803267714;
    double a2 = 0.0052790414;
    double a3 = 0.0000232718;
    double a4 = -0.0000030876910891;
    double a5 = 0.0000000043977311;
    double a6 = 0.0000000000007211;

    double gv = a1 * (1 + a2 * sin(lat) * sin(lat) + a3 * pow(sin(lat), 4))
                + (a4 + a5 * sin(lat) * sin(lat)) * height + a6 * height * height;
    return float(gv);
}

Vector2f AccLeveling(const float &fx, const float &fy, const float &fz){
    float pitch = atan2(fy, sqrt(fx*fx + fz*fz));
    float roll = atan2(-fx, fz);
    Vector2f pr(pitch, roll);
    return pr;
}

float normalizeAngle(float *angle, int size){
    for (int i=0;i<size;i++){
        angle[i] = fmod(angle[i] + M_PI, 2 * M_PI) - M_PI;

        if (angle[i] > M_PI) {
            angle[i] -= 2 * M_PI;
        }

        if (angle[i] <= -M_PI) {
            angle[i] += 2 * M_PI;
        }
    }
}
