#include "Quaternion.h"


using namespace Eigen;

namespace MyQuaternion{
    Quaternion::Quaternion(){
        Matrix3f temp = Matrix3f::Identity(3, 3);        
        genQbyR(temp);
    }

    Quaternion::Quaternion(const Quaternion &qut) : q(qut.q){}

    Quaternion::Quaternion(const Matrix3f &R_b2l){
        genQbyR(R_b2l);
    }

    Quaternion::Quaternion(const float &pitch, const float &roll, const float &yaw){
        genQbyOri(pitch, roll, yaw);
    }

    Quaternion::Quaternion(const Vector4f &q) : q(q){}

    Quaternion::Quaternion(const float &R11, const float &R12, const float &R13, const float &R21, const float &R22, const float &R23, const float &R31, const float &R32, const float &R33){
        Matrix3f R_b2l;
        R_b2l(0,0) = R11;
        R_b2l(0,1) = R12;
        R_b2l(0,2) = R13;
        R_b2l(1,0) = R21;
        R_b2l(1,1) = R22;
        R_b2l(1,2) = R23;
        R_b2l(2,0) = R31;
        R_b2l(2,1) = R32;
        R_b2l(2,2) = R33;
        genQbyR(R_b2l);
    }

    Quaternion &Quaternion::operator=(const Quaternion &other) {
        if (this != &other) {
            q = other.q;
        }
        return *this;
    }

    void Quaternion::genQbyR(const Matrix3f &R_b2l){
        float q4 = 0.5 * sqrt(1 + R_b2l(0, 0) + R_b2l(1, 1) + R_b2l(2, 2));
        if (q4 < -1 || q4 > 1) {
            q4 = std::min(std::max(q4, -1.0f), 1.0f);
        }
        float q1 = 0.25 * (R_b2l(2, 1) - R_b2l(1, 2)) / q4;
        float q2 = 0.25 * (R_b2l(0, 2) - R_b2l(2, 0)) / q4;
        float q3 = 0.25 * (R_b2l(1, 0) - R_b2l(0, 1)) / q4;
        q << q1, q2, q3, q4;
    }

    void Quaternion::genQbyOri(const float &pitch, const float &roll, const float &yaw){
        // float q4 = cos(pitch/2) * cos(roll/2) * cos(yaw/2) + sin(pitch/2) * sin(roll/2) * sin(yaw/2);
        // float q1 = sin(pitch/2) * cos(roll/2) * cos(yaw/2) - cos(pitch/2) * sin(roll/2) * sin(yaw/2);
        // float q2 = cos(pitch/2) * sin(roll/2) * cos(yaw/2) + sin(pitch/2) * cos(roll/2) * sin(yaw/2);
        // float q3 = cos(pitch/2) * cos(roll/2) * sin(yaw/2) - sin(pitch/2) * sin(roll/2) * cos(yaw/2);
        // q << q1, q2, q3, q4;

        Matrix3f ro_x;
        ro_x << 1, 0, 0,
            0, cos(-pitch), sin(-pitch),
            0, -sin(-pitch), cos(-pitch);

        Matrix3f ro_y;
        ro_y << cos(-roll), 0, -sin(-roll),
                0, 1, 0,
                sin(-roll), 0, cos(-roll);

        Matrix3f ro_z;
        ro_z << cos(-yaw), sin(-yaw), 0,
                -sin(-yaw), cos(-yaw), 0,
                0, 0, 1;

        // Combine the rotation matrices in the order roll, pitch, yaw
        genQbyR(ro_z * ro_x * ro_y);
    }

    void Quaternion::rotate(const float &in_wx, const float &in_wy, const float &in_wz, const float &dt){
        const float wx = in_wx;
        const float wy = in_wy;
        const float wz = in_wz;
        Matrix4f omg;
        omg <<  0, wz, -wy, wx,
                -wz, 0, wx, wy,
                wy, -wx, 0, wz,
                -wx, -wy, -wz, 0;
        omg = omg * dt;
        float theta = sqrt(wx * wx + wy * wy + wz * wz) * dt;
        if (theta > 0){
            float c_ = 2 * (cos(theta / 2) - 1);
            float s_ = 2 / theta * sin(theta/2);
            Matrix4f eye4 = MatrixXf::Identity(4,4);
            Matrix4f upd_term = c_ * eye4 + s_ * omg;
            q = q + 0.5 * upd_term * q;
            if (abs(q(3)) > 1){
                q(3) = std::min(std::max(q(3), -1.0f), 1.0f);
            }
        }
    }

    Matrix3f Quaternion::getR_b2l() const{
        float q1 = q(0);
        float q2 = q(1);
        float q3 = q(2);
        float q4 = q(3);

        Matrix3f R_b2l;
        R_b2l(0,0) = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;
        R_b2l(0,1) = 2 * (q1 * q2 - q3 * q4);
        R_b2l(0,2) = 2 * (q1 * q3 + q2 * q4);
        R_b2l(1,0) = 2 * (q1 * q2 + q3 * q4);
        R_b2l(1,1) = -q1 * q1 + q2 * q2 - q3 * q3 + q4 * q4;
        R_b2l(1,2) = 2 * (q2 * q3 - q1 * q4);
        R_b2l(2,0) = 2 * (q1 * q3 - q2 * q4);
        R_b2l(2,1) = 2 * (q2 * q3 + q1 * q4);
        R_b2l(2,2) = -q1 * q1 - q2 * q2 + q3 * q3 + q4 * q4;
        return R_b2l;
    }
    
    void Quaternion::getOri(float (&ori)[3]) const{
        Matrix3f R_b2l = getR_b2l();
        float pitch = atan2(R_b2l(2, 1), sqrt(R_b2l(0, 1) * R_b2l(0, 1) + R_b2l(1, 1) * R_b2l(1, 1)));
        float yaw = -atan2(R_b2l(0, 1), R_b2l(1, 1));
        float roll = -atan2(R_b2l(2, 0), R_b2l(2, 2));

        // float q1 = q(0);
        // float q2 = q(1);
        // float q3 = q(2);
        // float q4 = q(3);

        // double pitch = std::atan2(2 * (q4 * q1), 1 - 2 * (q1 * q1 + q2 * q2));
        
        // float temp = 2 * (q4 * q2 - q1 * q3);
        // temp = std::min(std::max(temp, -1.0f), 1.0f); // Clip to [-1, 1]
        // double roll = -M_PI/2 + 2 * std::atan2(std::sqrt(1 + temp), std::sqrt(1 - temp));
        
        // double yaw = std::atan2(2 * (q4 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));

        ori[0] = pitch;
        ori[1] = roll;
        ori[2] = yaw;
    }

    std::vector<float> Quaternion::getQ() const{
        std::vector<float> qq(4);
        for(int i=0;i<4;i++){
            qq[i] = q(i);
        }
        return qq;
    }
}


