#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "Quaternion.h"

using namespace Eigen;

namespace KalmanFilter{
    class KalmanFilter{
        protected:
            Matrix3f var_omg = Matrix3f::Zero();
            Matrix2f F = Matrix2f::Identity();
            Matrix2f Q = Matrix2f::Zero();
            Matrix2f P = Matrix2f::Identity();
            Matrix2f R = Matrix2f::Identity(2,2);
            Matrix2f H = Matrix2f::Identity(2,2);
            Vector2f dx = Vector2f::Zero();
            Vector2f Z = Vector2f::Zero();
            bool work = true;
            float ma_value = -1;
            float Z_list[100][2];
            int counter = 0;
            MyQuaternion::Quaternion qut;
            void compensate();
            void genZ(const float (&acc)[3]);
            void genQ(const float dt);
            Vector2f AccLeveling(const float &fx, const float &fy, const float &fz);
            void makeZList();
            float normalizeAngle(float *angle, int size);
            void normalizeAcc(const float (&ori_acc)[3], float (&new_acc)[3]) const;
        public:
            KalmanFilter();
            void setInit(const float (&std_imu)[6]);
            void setInitOri(const float pitch, const float roll, const float yaw);
            void getEularAngle(float (&temp_ori)[3]) const;
    };

    class EKF : public KalmanFilter{
        private:
            void predict();
            void update();
        public:
            EKF();
            void run(const float &dt, const float (&omg)[3], const float (&acc)[3]);
    };

    class UKF : public KalmanFilter{
        private:
            float W[5];
            void update();
        public:
            UKF();
            void run(const float &dt, const float (&omg)[3], const float (&acc)[3]);
    };
}

#endif