#include "KalmanFilter.h"

using namespace Eigen;

KalmanFilter::KalmanFilter::KalmanFilter(){}

void KalmanFilter::KalmanFilter::setInit(const float (&std_imu)[6]){
    for (int i = 0; i < 3; i++){
        var_omg(i, i) = std_imu[i] * std_imu[i];
    }
    R(0,0) = std_imu[4] * std_imu[4];
    R(1,1) = std_imu[3] * std_imu[3];
}

void KalmanFilter::KalmanFilter::genZ(const float (&acc)[3]){
    Vector2f new_pr = AccLeveling(acc[0], acc[1], acc[2]);
    float ori[3];
    qut.getOri(ori);
    float d_pr[] = {ori[0] - new_pr(0), ori[1] - new_pr(1)};
    normalizeAngle(d_pr, 2);
    Z << d_pr[0], d_pr[1];
    
    makeZList();
    
    if (abs(ori[0]) > 0.785 || abs(ori[1]) > 0.785){
        work = false;
    }

    if (work){
        float z2 = (Z(0) * Z(0) + Z(1) * Z(1));
        if (ma_value > 0 && z2 > 4 * ma_value){
            work = false;
        }
    }
}

void KalmanFilter::KalmanFilter::compensate(){
    if (work){
        qut.rotate(-dx[0], -dx[1], 0, 1);
    }
}

void KalmanFilter::KalmanFilter::makeZList(){
    float z2 = (Z(0) * Z(0) + Z(1) * Z(1));
    if (ma_value > 0){
        ma_value -= (Z_list[counter][0] * Z_list[counter][0] + Z_list[counter][1] * Z_list[counter][1]) / 100;
        ma_value += z2 / 100;
    }
    
    Z_list[counter][0] = Z(0);
    Z_list[counter][1] = Z(1);

    if (++counter == 99){
        if (ma_value <=0){
            ma_value = 0;
            for (int i = 0; i < 100;i++){
                ma_value += Z_list[i][0] * Z_list[i][0] + Z_list[i][1] * Z_list[i][1];
            }
            ma_value /= 100;
        }
        counter = 0;
    }
}

void KalmanFilter::KalmanFilter::genQ(const float dt){
    Matrix3f R_b2l = qut.getR_b2l();
    Matrix<float,2,3> G = R_b2l.block<2,3>(0,0);
    Q = G * var_omg * G.transpose() * dt;
}

void KalmanFilter::KalmanFilter::setInitOri(const float pitch, const float roll, const float yaw){
    qut = MyQuaternion::Quaternion(pitch, roll, yaw);
}

void KalmanFilter::KalmanFilter::getEularAngle(float (&temp_ori)[3]) const{
    float ori_rad[3];
    qut.getOri(ori_rad);
    for (int i=0;i<3;i++){
        temp_ori[i] = degrees(ori_rad[i]);
    }
}

void KalmanFilter::KalmanFilter::normalizeAcc(const float (&ori_acc)[3], float (&new_acc)[3]) const{
    float abs_acc = sqrt(ori_acc[0] * ori_acc[0] + ori_acc[1] * ori_acc[1] + ori_acc[2] * ori_acc[2]);
    for (int i=0;i<3;i++){
        new_acc[i] = ori_acc[i] / abs_acc;
    }
}

Vector2f KalmanFilter::KalmanFilter::AccLeveling(const float &fx, const float &fy, const float &fz){
    float pitch = atan2(fy, sqrt(fx*fx + fz*fz));
    float roll = atan2(-fx, fz);
    Vector2f pr(pitch, roll);
    return pr;
}

float KalmanFilter::KalmanFilter::normalizeAngle(float *angle, int size){
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


KalmanFilter::EKF::EKF(){}

void KalmanFilter::EKF::run(const float &dt, const float (&omg)[3], const float (&acc)[3]){
    genQ(dt);
    predict();
    qut.rotate(omg[0], omg[1], omg[2], dt);
    work = true;
    
    genZ(acc);
    update();
    compensate();
    
}

void KalmanFilter::EKF::predict(){
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::EKF::update(){
    if (work){
        Matrix2f eye = Matrix2f::Identity(2,2);
        Matrix2f K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        dx = K * Z;
        P = (eye - K * H) * P * (eye - K * H).transpose() + K * R * K.transpose();
    }
}





KalmanFilter::UKF::UKF(){
    // Create wieghts
    int LD = 1;
    int L = 2;
    int num_points = 2 * L + LD;
    W[0] = float(LD) / float(L+LD);
    for (int i = 0; i < num_points-1; i++){
        W[i+1] = float(LD) / float(L + LD) / 2.f;
    }
}

void KalmanFilter::UKF::run(const float &dt, const float (&omg)[3], const float (&acc)[3]){
    genQ(dt);
    qut.rotate(omg[0], omg[1], omg[2], dt);
    
    if (abs(sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2] * acc[2]) - 9.8) < 0.2){
        work = true;
        genZ(acc);
        update();
        compensate();
    }
}

void KalmanFilter::UKF::update(){
    int LD = 1;
    int L = 2;
    int num_points = 2 * L + LD;

    // Calculate sigma points
    Matrix2f A = ((LD+L)*P).llt().matrixL(); // Cholesky decomposition

    // Create a vector of sigma points
    Matrix<float, 5, 2> sigma_points;
    sigma_points << dx.transpose(), A.row(0), A.row(1), -A.row(0), -A.row(1);

    // predict_status
    Matrix<float, 2, 5> predict_sigma_points = F * sigma_points.transpose();
    Vector2f X_ = Vector2f::Zero();

    for (int i = 0; i < num_points ;i++){
        X_ += W[i] * predict_sigma_points.col(i);
    }

    Matrix<float, 2, 5> dif_x = predict_sigma_points - X_.replicate(1, 5);
    Matrix2f P_ = Matrix2f::Zero() + Q;
    for (int i = 0; i < num_points ;i++){
        P_ += W[i] * (dif_x.col(i) * dif_x.col(i).transpose());
    }

    //predict measurements
    Matrix<float, 2, 5> Y = H * predict_sigma_points;
    Vector2f Y_ = Vector2f::Zero();
    for (int i = 0; i < num_points ;i++){
        Y_ += W[i] * Y.col(i);
    }

    Matrix<float, 2, 5> dif_y = Y - Y_.replicate(1, 5);
    Matrix2f Pyy = Matrix2f::Zero() + R;
    Matrix2f Pxy = Matrix2f::Zero();
    for (int i = 0; i < num_points ;i++){
        Pyy += W[i] * (dif_y.col(i) * dif_y.col(i).transpose());
        Pxy += W[i] * (dif_x.col(i) * dif_y.col(i).transpose());
    }

    //measurement update
    Matrix2f K = Pxy * Pyy.inverse();
    dx = X_ + K * (Z - Y_);
    P = P_ - K * Pyy * K.transpose();
}