#include "ComplementaryFilter.h"


using namespace Eigen;


CPF::ComplementaryFilter::ComplementaryFilter(){
    _qut = MyQuaternion::Quaternion(0,0,0);
}

CPF::ComplementaryFilter::ComplementaryFilter(float ini_ori[3]){
    _qut = MyQuaternion::Quaternion(ini_ori[0], ini_ori[1], ini_ori[2]);
}

void CPF::ComplementaryFilter::setIMUError(const float (&std_imu)[6]){
    Vector2f error_acc, error_omg;
    float dt = 0.01;
    error_omg(0) = std_imu[0] * std_imu[0] * dt;
    error_omg(1) = std_imu[1] * std_imu[1] * dt;
    error_acc(0) = std_imu[4] * std_imu[4] / 9.8 / 9.8;
    error_acc(1) = std_imu[3] * std_imu[3] / 9.8 / 9.8;
    weight(0,0) = error_omg(0) / (error_omg(0) + error_acc(0));
    weight(1,1) = error_omg(1) / (error_omg(1) + error_acc(1));
}


void CPF::ComplementaryFilter::run(const float dt, const float (&omg)[3], const float (&acc)[3]){
    float _ori_temp[3];
    _qut.rotate(omg[0], omg[1], omg[2], dt);
    _qut.getOri(_ori_temp);

    Vector2f ori_omg;
    ori_omg << _ori_temp[0], _ori_temp[1];
    Vector2f ori_acc = AccLeveling(acc[0], acc[1], acc[2]);
    Vector2f del_ori = weight * (ori_acc - ori_omg);
    _qut.rotate(del_ori(0), del_ori(1), 0, 1);
}

void CPF::ComplementaryFilter::getEularAngle(float (&ori)[3]) const{
    float ori_rad[3];
    _qut.getOri(ori_rad);
    for (int i=0;i<3;i++){
        ori[i] = degrees(ori_rad[i]);
    }
}