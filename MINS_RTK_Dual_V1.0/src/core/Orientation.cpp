#include "Orientation.h"


using namespace Eigen;


namespace MyDirectCosineMatrix{
    DirectCosineMatrix::DirectCosineMatrix(){
        R_b2l = Matrix3f::Identity(3, 3);
    }

    DirectCosineMatrix::DirectCosineMatrix(const float &pitch, const float &roll, const float &yaw){
        float ori[3] = {pitch, roll, yaw};
        normalizeAngle(ori);
        genRbyOri(ori[0], ori[1], ori[2]);
    }

    DirectCosineMatrix::DirectCosineMatrix(const Matrix3f &R_b2l) : R_b2l(R_b2l){}

    DirectCosineMatrix &DirectCosineMatrix::operator=(const DirectCosineMatrix &other){
        if (this != &other) {
            R_b2l = other.R_b2l;
        }
        return *this;
    }

    void DirectCosineMatrix::genRbyOri(const float &pitch, const float &roll, const float &yaw){
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
        
        R_b2l = ro_z * ro_x * ro_y;    
    }

    void DirectCosineMatrix::rotate(Vector3f& vector){
        R_b2l = R_b2l * vector2R(vector);
    }

    void DirectCosineMatrix::kf_correct(Vector3f& vector){
        R_b2l = vector2R(vector) * R_b2l;
    }

    Matrix3f DirectCosineMatrix::getR_b2l() const{
        return R_b2l;
    }

    // generate orientation in radians
    void DirectCosineMatrix::getOri(float (&ori)[3]) const{
        ori[0] = atan2(R_b2l(2, 1), sqrt(R_b2l(0, 1) * R_b2l(0, 1) + R_b2l(1, 1) * R_b2l(1, 1)));
        ori[1] = -atan2(R_b2l(2, 0), R_b2l(2, 2));
        ori[2] = -atan2(R_b2l(0, 1), R_b2l(1, 1));
    }

    void DirectCosineMatrix::getQ(float (&q)[4]) const{
        float trace = R_b2l(0, 0) + R_b2l(1, 1) + R_b2l(2, 2);
        if (trace > 0){
            q[3] = 0.5 * sqrt(1 + trace);
        }
        else{
            float t1 = (R_b2l(2, 1) - R_b2l(1, 2)) * (R_b2l(2, 1) - R_b2l(1, 2));
            float t2 = (R_b2l(0, 2) - R_b2l(2, 0)) * (R_b2l(0, 2) - R_b2l(2, 0));
            float t3 = (R_b2l(1, 0) - R_b2l(0, 1)) * (R_b2l(1, 0) - R_b2l(0, 1));
            q[3] = 0.5 * sqrt( t1 + t2 + t3) / sqrt(3 - trace);
        }
            

        if (q[3] < -1 || q[3] > 1) {
            q[3] = std::min(std::max(q[3], -1.0f), 1.0f);
        }

        q[0] = 0.25 * (R_b2l(2, 1) - R_b2l(1, 2)) / q[3];
        q[1] = 0.25 * (R_b2l(0, 2) - R_b2l(2, 0)) / q[3];
        q[2] = 0.25 * (R_b2l(1, 0) - R_b2l(0, 1)) / q[3];
    }

    void DirectCosineMatrix::resetOri(const float &pitch, const float &roll, const float &yaw){
        float ori[3] = {pitch, roll, yaw};
        normalizeAngle(ori);
        genRbyOri(ori[0], ori[1], ori[2]);
    }
}

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
        float ori[3] = {pitch, roll, yaw};
        normalizeAngle(ori);
        genQbyOri(ori[0], ori[1], ori[2]);
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
        float trace = R_b2l(0, 0) + R_b2l(1, 1) + R_b2l(2, 2);
        float q4;
        if (trace > 0){
            q4 = 0.5 * sqrt(1 + trace);
        }
        else{
            float t1 = (R_b2l(2, 1) - R_b2l(1, 2)) * (R_b2l(2, 1) - R_b2l(1, 2));
            float t2 = (R_b2l(0, 2) - R_b2l(2, 0)) * (R_b2l(0, 2) - R_b2l(2, 0));
            float t3 = (R_b2l(1, 0) - R_b2l(0, 1)) * (R_b2l(1, 0) - R_b2l(0, 1));
            q4 = 0.5 * sqrt( t1 + t2 + t3) / sqrt(3 - trace);
        }
            

        if (q4 < -1 || q4 > 1) {
            q4 = std::min(std::max(q4, -1.0f), 1.0f);
        }

        float q1 = 0.25 * (R_b2l(2, 1) - R_b2l(1, 2)) / q4;
        float q2 = 0.25 * (R_b2l(0, 2) - R_b2l(2, 0)) / q4;
        float q3 = 0.25 * (R_b2l(1, 0) - R_b2l(0, 1)) / q4;
        q << q1, q2, q3, q4;
    }

    void Quaternion::genQbyOri(const float &pitch, const float &roll, const float &yaw){
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

    void Quaternion::resetOri(const float &pitch, const float &roll, const float &yaw){
        float ori[3] = {pitch, roll, yaw};
        normalizeAngle(ori);
        genQbyOri(ori[0], ori[1], ori[2]);
    }
}


void normalizeAngle(float (&ori)[3]){
    for (int i=0;i<3;i++){
        ori[i] = fmod(ori[i] + M_PI, 2 * M_PI) - M_PI;

        if (ori[i] > M_PI) {
            ori[i] -= 2 * M_PI;
        }
        else if (ori[i] <= -M_PI) {
            ori[i] += 2 * M_PI;
        }
    }
}

Matrix3f vector2skew(const Vector3f& v) {
    Matrix3f skew;
    skew <<     0, -v(2),  v(1),
             v(2),     0, -v(0),
            -v(1),  v(0),     0;
    return skew;
}

Matrix3f vector2R(const Vector3f& rotation_vect){
    float theta = rotation_vect.norm();  // Equivalent to sqrt(np.dot(vector, vector))
    
    // If theta is near zero, return the identity matrix
    if (theta <= 0) {
        return Matrix3f::Identity();
    }

    Matrix3f omg_w_lb_b = vector2skew(rotation_vect) / theta;  // Skew-symmetric matrix
    float s = std::sin(theta);
    float c = 1.0f - std::cos(theta);

    // Compute the rotation matrix
    return Matrix3f::Identity() + s * omg_w_lb_b + c * omg_w_lb_b * omg_w_lb_b;
}

Matrix3f vector2R_apx(const Vector3f& rotation_vect){
    // Compute the rotation matrix
    return Matrix3f::Identity() + vector2skew(rotation_vect);
}

void LLH2ENU(float lat, float lon, float alt, float lat0, float lon0, float alt0, float* x, float* y, float* z){
    float dlat = radians(lat - lat0);
    float dlon = radians(lon - lon0);
    float dalt = alt - alt0;
    float rad_lat0 = radians(lat0);
    
    float R_m = WGS84_a * (1.0f - WGS84_e2) / pow(1 - WGS84_e2 * sin(rad_lat0) * sin(rad_lat0), 1.5);
    float R_n = WGS84_a / sqrt(1.0f - WGS84_e2 * sin(rad_lat0) * sin(rad_lat0));
    *x = dlon * (R_n + alt0) * cos(rad_lat0);
    *y = dlat * (R_m + alt0);
    *z = dalt;
}
