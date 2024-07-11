#include "DataOutput.h"


Integration::Integration() : ang(3, 0.0), pre_time(-1){}

std::vector<float> Integration::update(const float &time, float (&omg)[3]){
    if (pre_time != -1){
        float dt = time - pre_time;
        for (int i=0;i<3;i++){
            ang[i] += omg[i] * dt;
            if (ang[i] > 360){
                ang[i] -=360;
            }
            else if (ang[i] < 0){
                ang[i] += 360;
            }
        }
    }
    pre_time = time;    
    return ang;
}



AxesDefine::AxesDefine(){
    reset();
}

void AxesDefine::setInit(const float &R11, const float &R12, const float &R13, const float &R21, const float &R22, const float &R23, const float &R31, const float &R32, const float &R33){
    Matrix3f R_l2b;
    R_l2b(0,0) = R11;
    R_l2b(0,1) = R12;
    R_l2b(0,2) = R13;
    R_l2b(1,0) = R21;
    R_l2b(1,1) = R22;
    R_l2b(1,2) = R23;
    R_l2b(2,0) = R31;
    R_l2b(2,1) = R32;
    R_l2b(2,2) = R33;
    R_b2l = R_l2b.transpose();   
    // if (isRotationMatrix(R_l2b)){
    //   R_b2l = R_l2b.transpose();    
    // }
}

void AxesDefine::update(float (&input)[3]){
    float temp[] = {0,0,0};
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            temp[i] += R_b2l(i,j) * input[j];
        }
    }
    for (int i = 0; i < 3; i++) {
        input[i] = temp[i];
    }
}

void AxesDefine::reset(){
    R_b2l = MatrixXf::Identity(3,3);
}

bool AxesDefine::isRotationMatrix(const Matrix3f& matrix) {
    if (!matrix.isApprox(matrix.transpose()) || !matrix.transpose().isApprox(matrix)) {
        return false;
    }
    if (abs(matrix.determinant() - 1.0f) > 1e-6) {
        return false;
    }
    return true;
}
