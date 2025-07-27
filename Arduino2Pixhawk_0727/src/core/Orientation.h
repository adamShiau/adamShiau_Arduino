#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <ArduinoEigenDense.h>
#include <vector>

using namespace Eigen;

#define WGS84_a     6378137.0f
#define WGS84_b     6356752.314245f
#define WGS84_e     0.08181919f
#define WGS84_e2    0.0066943798522561f
#define WGS84_f     0.00335281066474748f
#define WGS84_we    7.29211514670698e-5f

void normalizeAngle(float (&ori)[3]);
void LLH2ENU(float lat, float lon, float alt, float lat0, float lon0, float alt0, float* x, float* y, float* z);
Matrix3f vector2skew(const Vector3f& v);
Matrix3f vector2R(const Vector3f& rotation_vect);
Matrix3f vector2R_apx(const Vector3f& rotation_vect);

namespace MyDirectCosineMatrix{
    class DirectCosineMatrix{
        private:
            Matrix3f R_b2l;

            // force Euler angle between +- 180
            // void normalizeAngle(float (&ori)[3]);
            void genRbyOri(const float &pitch, const float &roll, const float &yaw);

        public:
            DirectCosineMatrix();
            DirectCosineMatrix(const float &pitch, const float &roll, const float &yaw);
            DirectCosineMatrix(const Matrix3f &R_b2l);
            DirectCosineMatrix &operator=(const DirectCosineMatrix &other);

            // update dcm by rotation vector
            void rotate(Vector3f& vector);
            
            // update dcm by kf correction rotation vector
            void kf_correct(Vector3f& vector);

            // generate 4 params of Quaternion by DSM
            void getQ(float (&q)[4]) const;

            // generate DSM and transport
            Matrix3f getR_b2l() const;

            // generate orientation in radians
            void getOri(float (&ori)[3]) const;

            // reset orientation by Euler angle
            void resetOri(const float &pitch, const float &roll, const float &yaw);
    };
}


namespace MyQuaternion {
    class Quaternion
    {
        private:
            Vector4f q;
            void genQbyOri(const float &pitch, const float &roll, const float &yaw);

        public:
            // define Quaternion at initial orientation, pitch=0, roll=0, yaw=0
            Quaternion();

            // define Quaternion by a known Quaternion
            Quaternion(const Quaternion &qut);

            // define Quaternion by direction cosine matrix
            Quaternion(const Matrix3f &R_b2l);

            // define Quaternion by specific orientation in eular angle with radians
            Quaternion(const float &pitch, const float &roll, const float &yaw);

            // define Quaternion by a known Quaternion
            Quaternion(const Vector4f &q);

            // define Quaternion by direction cosine matrix
            Quaternion(const float &R11, const float &R12, const float &R13, const float &R21, const float &R22, const float &R23, const float &R31, const float &R32, const float &R33);

            // redefine Quaternion by another Quaternion
            Quaternion &operator=(const Quaternion &other);

            // update Quaternion by angular rate
            void rotate(const float &wx, const float &wy, const float &wz, const float &dt);

            // generate 4 params of Quaternion by DSM
            void genQbyR(const Matrix3f &R_b2l);

            // generate DSM and transport
            Matrix3f getR_b2l() const;

            // generate orientation in radians
            void getOri(float (&ori)[3]) const;

            // generate 4 params of Quternion and transport
            std::vector<float> getQ() const;

            // reset orientation by Euler angle
            void resetOri(const float &pitch, const float &roll, const float &yaw);
    };
}

#endif