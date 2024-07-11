#ifndef CPF_H
#define CPF_H

#include "Quaternion.h"
#include "Tools.h"

using namespace Eigen;


namespace CPF{
    class ComplementaryFilter{
        private:
            MyQuaternion::Quaternion _qut;
            Matrix2f weight = Matrix2f::Identity();

        public:
            ComplementaryFilter();
            ComplementaryFilter(float ini_ori[3]);
            void setIMUError(const float (&std_imu)[6]);
            void run(const float dt, const float (&omg)[3], const float (&acc)[3]);
            void getEularAngle(float (&ori)[3]) const;
    };
}



#endif