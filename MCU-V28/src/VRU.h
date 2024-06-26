// #ifndef VRU_H
// #define VRU_H

// #include "Quaternion.h"
// #include <ArduinoEigenDense.h>
// #include <vector>
// #include "Tools.h"


// using namespace Eigen;



// class VRU_Prediction{
//     private:
//         Matrix3f var_imu;
//         Matrix2f F = Matrix2f::Identity(2, 2);
//         Matrix2f Q;
//         const float alpha = 1.0005;
//     public:
//         void setInit(const float (&std_omg)[3]);
//         void input(const float &dt, const Matrix3f &R_b2l);
//         void predict(Matrix2f& P);
// };


// class AccLeveling_Update{
//     private:
//         Matrix2f R = Matrix2f::Identity(2,2);
//         Matrix2f H = Matrix2f::Identity(2,2);
//         Vector2f dx = Vector2f::Zero();
//         Vector2f Z = Vector2f::Zero();
//         bool work = true;
//     public:
//         void setInit(const float (&std_acc)[3]);
//         void input(const float (&acc)[3], const float (&qut_ori)[3]);
//         void update(Matrix2f& P);
//         void compensate(MyQuaternion::Quaternion &qut);
// };


// class VRU{
//     private:
//         // float _time = -1;
//         Matrix2f P;
//         MyQuaternion::Quaternion _qut;
//         VRU_Prediction predictor;
//         AccLeveling_Update updator;
//     public:
//         void setInit(const float (&std_imu)[6]);
//         void run(const float &dt, const float (&omg)[3], const float (&acc)[3]);
//         void getOri(float (&temp_ori)[3]) const;
// };


// #endif

