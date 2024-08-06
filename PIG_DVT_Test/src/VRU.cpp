// #include "VRU.h"

// using namespace Eigen;


// void VRU_Prediction::setInit(const float (&std_omg)[3])
// {   
//     var_imu.setZero();
//     Q.setZero();
//     for(int i=0;i<3;i++){
//         var_imu(i, i) = std_omg[i] * std_omg[i];
//     }
// }

// void VRU_Prediction::input(const float &dt, const Matrix3f &R_b2l){
//     Matrix<float,2,3> G;
//     G = R_b2l.block<2,3>(0,0);
//     Q = F * G * var_imu * G.transpose() * F.transpose() * dt;
// }

// void VRU_Prediction::predict(Matrix2f& P)
// {
//     P = alpha * alpha * F * P * F.transpose() + Q;
// }



// void AccLeveling_Update::setInit(const float (&std_acc)[3]){
//     R(0,0) = (std_acc[1] / 9.8) * ((std_acc[1] / 9.8));
//     R(1,1) = (std_acc[0] / 9.8) * ((std_acc[0] / 9.8));
// }

// void AccLeveling_Update::input(const float (&acc)[3], const float (&qut_ori)[3]){
//     if (work == true){
//         Vector2f new_pr;
//         new_pr = AccLeveling(acc[0], acc[1], acc[2]);
//         Z << qut_ori[0] - new_pr(0), qut_ori[1] - new_pr(1);
//     }
// }

// void AccLeveling_Update::update(Matrix2f& P)
// {
//     if (work == true){
//         Matrix2f eye = Matrix2f::Identity(2,2);
//         Matrix2f K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
//         dx = K * Z;
//         P = (eye - K * H) * P * (eye - K * H).transpose() + K * R * K.transpose();
//     }
// }

// void AccLeveling_Update::compensate(MyQuaternion::Quaternion &qut){
//     if (work == true){
//         qut.rotate(-dx[0], -dx[1], 0, 1);
//         dx.setZero();
//     }
// }



// void VRU::setInit(const float (&std_imu)[6]){
//     _qut = MyQuaternion::Quaternion(0,0,0);
//     P = Matrix2f::Identity(2, 2);
//     float std_omg[3];
//     float std_acc[3];
//     for(int i=0;i<3;i++){
//         std_omg[i] = std_imu[i];
//         std_acc[i] = std_imu[i+3];
//     }
//     predictor.setInit(std_omg);
//     updator.setInit(std_acc);
// }

// void VRU::run(const float &dt, const float (&omg)[3], const float (&acc)[3]){
//     predictor.input(dt, _qut.getR_b2l());
//     predictor.predict(P);

//     _qut.rotate(omg[0], omg[1], omg[2], dt);

//     if (abs(sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2] * acc[2]) - 9.8) < 0.2){
//         float temp_ori[3];
//         _qut.getOri(temp_ori);
//         updator.input(acc, temp_ori);
//         updator.update(P);
//         updator.compensate(_qut);
//     }
// }

// void VRU::getOri(float (&temp_ori)[3]) const{
//     float ori_rad[3];
//     _qut.getOri(ori_rad);
//     for (int i=0;i<3;i++){
//         temp_ori[i] = degrees(ori_rad[i]);
//     }
// }