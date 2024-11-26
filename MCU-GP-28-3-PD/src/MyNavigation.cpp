#include "MyNavigation.h"

namespace Navigation{
    
    //----------------------------------------------------------------//
    //                      Complementary Filter                      //
    //----------------------------------------------------------------//

    ComplementaryFilter::ComplementaryFilter(){
        dcm = MyDirectCosineMatrix::DirectCosineMatrix();
    }

    ComplementaryFilter::ComplementaryFilter(float ini_ori[3]){
        dcm = MyDirectCosineMatrix::DirectCosineMatrix(ini_ori[0], ini_ori[1], ini_ori[2]);
    }

    void ComplementaryFilter::setIMUError(Sensor_ID sensor_id, int fs){
        my_sensor = sensor_id;
        setDataRate(fs);
    }

    void ComplementaryFilter::setDataRate(int fs){
        IMUParams sensor_params = getSensorParams(my_sensor);
        std::array<float, 3> ARW = sensor_params.getARW();
        std::array<float, 3> BS_ACCL = sensor_params.getACCL_BS();
        float error_gyro[] = {ARW[0] / sqrt(fs), ARW[1] / sqrt(fs)};
        float error_accl[] = {BS_ACCL[1] / 9.79, BS_ACCL[0] / 9.79};

        weight[0] = error_gyro[0] / (error_gyro[0] + error_accl[0]);
        weight[1] = error_gyro[1] / (error_gyro[1] + error_accl[1]);

        LC.resetWindowSize(120 * fs);
    }

    void ComplementaryFilter::setWindowSizeLC(int wf){
        LC.resetWindowSize(wf);
    }

    void ComplementaryFilter::enableCheckACC(bool enable_check_acc){
        this->enable_check_acc = enable_check_acc;
    }
    
    bool ComplementaryFilter::checkALByAutocorrelation(const Vector2f& new_pr){     
        if (enable_check_acc)
        {
            pr_list.push_back(new_pr); 

            if (pr_list.size() > 20) {
                pr_list.erase(pr_list.begin());

                // 使用 Eigen 計算平均值
                Vector2f mean = Vector2f::Zero();
                for (const auto& pr : pr_list) {
                    mean += pr;
                }
                mean /= pr_list.size();

                // 計算 dif
                MatrixXf dif(pr_list.size(), 2);
                for (int i = 0; i < pr_list.size(); ++i) {
                    dif.row(i) = pr_list[i] - mean;
                }

                // 計算 up_term 和 bt_term
                MatrixXf up_term_mat = (dif.topRows(dif.rows() - 1).transpose() * dif.bottomRows(dif.rows() - 1)).diagonal();
                float up_term = up_term_mat.sum();

                MatrixXf bt_term_mat = (dif.transpose() * dif).diagonal();
                float bt_term = bt_term_mat.sum();

                // 計算 pho 並更新 work
                float pho = abs(up_term / bt_term);
                // 2/sqrt(20)
                if (pho >= 0.45) {
                    return false;
                }
            }
        }
        

        return true;
    }

    void ComplementaryFilter::run(const float t, const float (&omg)[3], const float (&acc)[3]){
        float LC_omg[3] = {omg[0], omg[1], omg[2]};
        if (LC.isBiasAvailable(LC_omg)){
            if (pre_time >=0){
                Vector3f vec = Vector3f::Zero();
                for (int i=0;i<3;i++){
                    if (abs(LC_omg[i]) > threshold[i]){
                        vec[i] = LC_omg[i];
                    }
                }

                vec = (radians(vec) - dcm.getR_b2l().transpose() * WE_IE_L) * (t - pre_time);
                dcm.rotate(vec);

                Vector2f pr_acc = accLeveling(acc[0], acc[1], acc[2]);
                if (checkALByAutocorrelation(pr_acc)){
                    float ori_temp[3];
                    dcm.getOri(ori_temp);

                    float new_pr[2];
                    new_pr[0] = ori_temp[0] - weight[0] * (ori_temp[0] - pr_acc(0));
                    new_pr[1] = ori_temp[1] - weight[1] * (ori_temp[1] - pr_acc(1));
                    
                    dcm.resetOri(new_pr[0], new_pr[1], ori_temp[2]);
                }
            }

            else{
                Vector2f pr_acc = accLeveling(acc[0], acc[1], acc[2]);
                dcm.resetOri(pr_acc(0), pr_acc(1), 0);
            }
        }

        else{
            dcm.resetOri(0, 0, 0);
        }

        pre_time = t;
    }

    void ComplementaryFilter::getEularAngle(float (&ori)[3]) const{
        float ori_rad[3];
        dcm.getOri(ori_rad);
        // qut.getOri(ori_rad);
        for (int i=0;i<3;i++){
            ori[i] = degrees(ori_rad[i]);
        }
    }

    void ComplementaryFilter::setPOS(float (&pos)[3]){
        float lat = radians(pos[0]);
        float lon = radians(pos[1]);
        g0 = genGravity(lat, pos[2]);
        WE_IE_L = genW_IE_L(lat, lon);
    }

    void ComplementaryFilter::setWE_IE_L_Zero(){
        WE_IE_L = Vector3f::Zero();
    }

    void ComplementaryFilter::resetEuler(float pitch, float roll, float yaw){
        dcm.resetOri(pitch, roll, yaw);
    }

    void ComplementaryFilter::setThreshold(float x_axis, float y_axis, float z_axis){
        threshold[0] = x_axis;
        threshold[1] = y_axis;
        threshold[2] = z_axis;
    }

    void ComplementaryFilter::setThresholdBySTD(){
        IMUParams sensor_params = getSensorParams(my_sensor);
        for (int i=0;i<3;i++){
            threshold[i] = degrees(3 * sensor_params.getGYRO_STD()[i]);
        }
    }

    void ComplementaryFilter::startLC(){
        LC.start();
    }

    void ComplementaryFilter::stopLC(){
        LC.stop();
    }

    //----------------------------------------------------------------//
    //                           Kalman Filter                        //
    //----------------------------------------------------------------//

    void KalmanFilter::setInit(const float (&std_imu)[6]){
        for (int i = 0; i < 3; i++){
            var_omg(i, i) = std_imu[i] * std_imu[i];
        }
        RR(0,0) = std_imu[4] * std_imu[4];
        RR(1,1) = std_imu[3] * std_imu[3];
        PP(3,3) = var_omg(0, 0);
        PP(4,4) = var_omg(1, 1);
        PP(5,5) = var_omg(2, 2);
    }

    void KalmanFilter::genF(const float t,const float dt){
        MatrixXf A = MatrixXf::Zero(6,6);
        A.block<3,3>(0,3) = qut.getR_b2l();

        FF = MatrixXf::Identity(6,6) + A * dt;
        float e = exp(-t);
        FF(3,3) *= e;
        FF(4,4) *= e;
        FF(5,5) = 0;
    }

    void KalmanFilter::genQ(const float dt){
        MatrixXf G(6, 3); 
        G.block<3,3>(0,0) = qut.getR_b2l();
        G.block<3,3>(3,0).setIdentity(); 
        QQ = G * var_omg * G.transpose() * dt;
    }

    void KalmanFilter::genH(){}

    void KalmanFilter::genZ(const float (&acc)[3]){
        // checkALbyGravity(acc);
        float mean_acc[3] = {(acc[0]+pre_acc[0])/2, (acc[1]+pre_acc[1])/2, (acc[2]+pre_acc[2])/2};
        Vector2f new_pr = accLeveling(mean_acc);
        checkALByAutocorrelation(new_pr);
        if (work){
            float ori[3];
            qut.getOri(ori);
            float d_pr[] = {ori[0] - new_pr[0], ori[1] - new_pr[1]};
            ZZ << d_pr[0], d_pr[1];
        }      
    }

    void KalmanFilter::checkALbyGravity(const float(&acc)[3]){
        float abs_acc = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        if (abs(abs_acc - 9.7895) > 9.7895 * 0.05){
            work = false;
        }
    }

    void KalmanFilter::checkResidual(){
        if (ma_value > 0){
            ma_value -= (Z_list[counter][0] * Z_list[counter][0] + Z_list[counter][1] * Z_list[counter][1]) / 100;
            ma_value += (ZZ(0) * ZZ(0) + ZZ(1) * ZZ(1)) / 100;
        }
        
        Z_list[counter][0] = ZZ(0);
        Z_list[counter][1] = ZZ(1);

        if (++counter == 100){
            if (ma_value <=0){
                ma_value = 0;
                for (int i = 0; i < 100;i++){
                    ma_value += Z_list[i][0] * Z_list[i][0] + Z_list[i][1] * Z_list[i][1];
                }
                ma_value /= 100;
            }
            counter = 0;
        }
        
        float z2 = (ZZ(0) * ZZ(0) + ZZ(1) * ZZ(1));
        if (ma_value > 0 && z2 > 4 * ma_value){
            work = false;
        }
    }

    void KalmanFilter::checkALByAutocorrelation(const Vector2f& new_pr){       
        pr_list.push_back(new_pr); 

        if (pr_list.size() > 20) {
            pr_list.erase(pr_list.begin());

            // 使用 Eigen 計算平均值
            Vector2f mean = Vector2f::Zero();
            for (const auto& pr : pr_list) {
                mean += pr;
            }
            mean /= pr_list.size();

            // 計算 dif
            MatrixXf dif(pr_list.size(), 2);
            for (int i = 0; i < pr_list.size(); ++i) {
                dif.row(i) = pr_list[i] - mean;
            }

            // 計算 up_term 和 bt_term
            MatrixXf up_term_mat = (dif.topRows(dif.rows() - 1).transpose() * dif.bottomRows(dif.rows() - 1)).diagonal();
            float up_term = up_term_mat.sum();

            MatrixXf bt_term_mat = (dif.transpose() * dif).diagonal();
            float bt_term = bt_term_mat.sum();

            // 計算 pho 並更新 work
            float pho = up_term / bt_term;
            if (pho >= 0.6) {
                work = false;
            }
        }

    }

    void KalmanFilter::normalizeAcc(const float (&ori_acc)[3], float (&new_acc)[3]) const{
        float abs_acc = sqrt(ori_acc[0] * ori_acc[0] + ori_acc[1] * ori_acc[1] + ori_acc[2] * ori_acc[2]);
        new_acc[0] = ori_acc[0] / abs_acc;
        new_acc[1] = ori_acc[1] / abs_acc;
        new_acc[2] = ori_acc[2] / abs_acc;
    }

    Vector2f KalmanFilter::accLeveling(const float (&acc)[3]){
        float fx = acc[0];
        float fy = acc[1];
        float fz = acc[2];
        Vector2f new_pr = {atan2(fy, sqrt(fx*fx + fz*fz)), atan2(-fx, fz)};
        return new_pr;
    }

    void KalmanFilter::copyACC(const float (&acc)[3]){
        memcpy(pre_acc, acc, 3 * sizeof(float)); 
    }

    void KalmanFilter::genR(){}   

    void KalmanFilter::compensate(){
        if (work){
            float ori[3];
            qut.getOri(ori);
            dx(2) = 0;
            dx(5) = 0;
            
            qut = MyQuaternion::Quaternion(ori[0] - dx(0), ori[1] - dx(1), ori[2] - dx(2));
            bias_omg[0] += dx(3);
            bias_omg[1] += dx(4);
            bias_omg[2] += dx(5);
        }
    }

    void KalmanFilter::setInitOri(const float pitch, const float roll, const float yaw){
        qut = MyQuaternion::Quaternion(pitch, roll, yaw);
    }

    void KalmanFilter::getEularAngle(float (&temp_ori)[3]) const{
        float ori_rad[3];
        qut.getOri(ori_rad);
        temp_ori[0] = degrees(ori_rad[0]);
        temp_ori[1] = degrees(ori_rad[1]);
        temp_ori[2] = degrees(ori_rad[2]);
    }

    void KalmanFilter::getBias(float (&temp_bias)[3]) const{
        temp_bias[0] = degrees(bias_omg[0]);
        temp_bias[1] = degrees(bias_omg[1]);
        temp_bias[2] = degrees(bias_omg[2]);
    }

    //Yaw in degrees units
    void KalmanFilter::resetYaw(const float yaw){
        float ori_rad[3];
        qut.getOri(ori_rad);
        qut.resetOri(ori_rad[0], ori_rad[1], radians(yaw));
    }

    void KalmanFilter::getCompensatedGYRO(float (&cali_omg)[3], float(&raw_omg)[3]) const{
        if (LC.isReady()){
            cali_omg[0] = raw_omg[0] - degrees(bias_omg[0]);
            cali_omg[1] = raw_omg[1] - degrees(bias_omg[1]);
            cali_omg[2] = raw_omg[2] - degrees(bias_omg[2]);    
        }
        else{
            memcpy(cali_omg, raw_omg, sizeof(cali_omg));
        }        
    }


    //----------------------------------------------------------------//
    //                     Extended-Kalman Filter                     //
    //----------------------------------------------------------------//


    EKF::EKF(const float pitch, const float roll, const float yaw, const int ws_lc, Sensor_ID id) : KalmanFilter(pitch, roll, yaw, ws_lc){
        float std_imu[6];
        // std::array<float, 6> sensor_params = getSensorParams(Nano33);
        setInit(std_imu);
    };


    void EKF::run(const float t, const float (&omg)[3], const float (&acc)[3]){
        //The first epoch would be skiped and the initial pitch, roll will be estimated. Meanwihile, t and acc will be stored.
        if (t0 > 0){
            work = true;
            float dt = t - pre_time;
            float rad_omg[3] = {radians(omg[0]), radians(omg[1]), radians(omg[2])};

            predict(t - t0, dt);
            qut.rotate(rad_omg[0] - bias_omg[0], rad_omg[1] - bias_omg[1], rad_omg[2] - bias_omg[2], dt);  
            update(acc);
            compensate();
        }
        else{
            t0 = t;
            Vector2f pr = accLeveling(acc);
            qut.resetOri(pr(0), pr(1), 0);
        }
        copyACC(acc);
        pre_time = t;
    }

    void EKF::predict(const float t, const float dt){
        genF(t, dt);
        genQ(dt);
        PP = FF * PP * FF.transpose() + QQ;
    }

    void EKF::update(const float (&acc)[3]){
        genZ(acc);
        if (work){
            // genH();
            // genR();
            MatrixXf K = PP * HH.transpose() * (HH * PP * HH.transpose() + RR).inverse();
            dx = K * ZZ;
            PP = (MatrixXf::Identity(6,6) - K * HH) * PP * (MatrixXf::Identity(6,6) - K * HH).transpose() + K * RR * K.transpose();
        }        
    }

    //----------------------------------------------------------------//
    //                    Unscented-Kalman Filter                     //
    //----------------------------------------------------------------//
    
    UKF::UKF(const float pitch, const float roll, const float yaw, const int ws_lc) : KalmanFilter(pitch, roll, yaw, ws_lc){
        // Create wieghts
        int LD = 1;
        int L = 2;
        int num_points = 2 * L + LD;
        W[0] = float(LD) / float(L+LD);
        for (int i = 0; i < num_points-1; i++){
            W[i+1] = float(LD) / float(L + LD) / 2.f;
        }
    }

    void UKF::run(const float &dt, const float (&omg)[3], const float (&acc)[3]){
        genQ(dt);
        qut.rotate(omg[0], omg[1], omg[2], dt);
        
        if (abs(sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2] * acc[2]) - 9.8) < 0.2){
            work = true;
            genZ(acc);
            update();
            compensate();
        }
    }

    void UKF::update(){
        int LD = 1;
        int L = 2;
        int num_points = 2 * L + LD;

        // Calculate sigma points
        Matrix2f A = ((LD+L)*PP).llt().matrixL(); // Cholesky decomposition

        // Create a vector of sigma points
        Matrix<float, 5, 2> sigma_points;
        sigma_points << dx.transpose(), A.row(0), A.row(1), -A.row(0), -A.row(1);

        // predict_status
        Matrix<float, 2, 5> predict_sigma_points = FF * sigma_points.transpose();
        Vector2f X_ = Vector2f::Zero();

        for (int i = 0; i < num_points ;i++){
            X_ += W[i] * predict_sigma_points.col(i);
        }

        Matrix<float, 2, 5> dif_x = predict_sigma_points - X_.replicate(1, 5);
        Matrix2f P_ = Matrix2f::Zero() + QQ;
        for (int i = 0; i < num_points ;i++){
            P_ += W[i] * (dif_x.col(i) * dif_x.col(i).transpose());
        }

        //predict measurements
        Matrix<float, 2, 5> Y = HH * predict_sigma_points;
        Vector2f Y_ = Vector2f::Zero();
        for (int i = 0; i < num_points ;i++){
            Y_ += W[i] * Y.col(i);
        }

        Matrix<float, 2, 5> dif_y = Y - Y_.replicate(1, 5);
        Matrix2f Pyy = Matrix2f::Zero() + RR;
        Matrix2f Pxy = Matrix2f::Zero();
        for (int i = 0; i < num_points ;i++){
            Pyy += W[i] * (dif_y.col(i) * dif_y.col(i).transpose());
            Pxy += W[i] * (dif_x.col(i) * dif_y.col(i).transpose());
        }

        //measurement update
        Matrix2f K = Pxy * Pyy.inverse();
        dx = X_ + K * (ZZ - Y_);
        PP = P_ - K * Pyy * K.transpose();
    }

    

    //----------------------------------------------------------------//
    //                          LinearCorrection                     //
    //----------------------------------------------------------------//

    LinearCorrection::LinearCorrection(const int ws) : window_size(ws) , count(0){
        for(int i=0;i<3;i++){
            sum[i] = 0.;
            bias[i] = 0.;
        }
        
    }

    void LinearCorrection::start() {
        for(int i=0;i<3;i++){
            sum[i] = 0;
            bias[i] = 0;
        }
        count = 0;
        is_biasCalculated = false;
    }

    void LinearCorrection::stop(){
        for(int i=0;i<3;i++){
            sum[i] = 0;
            bias[i] = 0;
        }
        count = 0;
        is_biasCalculated = true;
    }


    bool LinearCorrection::isBiasAvailable(float (&data)[3]) {
        if (!is_biasCalculated){
            if (bias[0] == 0){
                if (count < window_size) {
                    for(int i=0;i<3;i++){
                        sum[i] += data[i];
                    }
                    count++;
                }
                else if (count == window_size) {
                    float n = static_cast<float>(window_size);
                    for(int i=0;i<3;i++){
                        bias[i] = sum[i] / n;
                    }
                    is_biasCalculated = true;
                }
            }
        }

        if (is_biasCalculated){
            for(int i=0; i<3; i++){
                data[i] -= bias[i];
            }
        }

        return is_biasCalculated;
    }


    void LinearCorrection::setBias(float* bias_omg){
        bias_omg[0] = bias[0];
        bias_omg[1] = bias[1];
        bias_omg[2] = bias[2];
    }

    bool LinearCorrection::isReady() const{
        return is_biasCalculated;
    }

    void LinearCorrection::resetWindowSize(const int ws){
        window_size = ws;
        stop();
    }

}

float genGravity(const float &lat, const float &height){
    float a1 = 9.7803267714;
    float a2 = 0.0052790414;
    float a3 = 0.0000232718;
    float a4 = -0.0000030876910891;
    float a5 = 0.0000000043977311;
    float a6 = 0.0000000000007211;

    float gv = a1 * (1 + a2 * sin(lat) * sin(lat) + a3 * pow(sin(lat), 4))
                + (a4 + a5 * sin(lat) * sin(lat)) * height + a6 * height * height;
    return float(gv);
}

Vector2f accLeveling(const float &fx, const float &fy, const float &fz){
    float pitch = atan2(fy, sqrt(fx*fx + fz*fz));
    float roll = atan2(-fx, fz);
    Vector2f pr(pitch, roll);
    return pr;
}

float normalizeAngle(float *angle, int size){
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

Vector3f genW_IE_L(float lat, float lon){
    float cos_lat = std::cos(lat);
    float sin_lat = std::sin(lat);
    float cos_lon = std::cos(lon);
    float sin_lon = std::sin(lon);

    Matrix3f R_l2e;
    R_l2e << 
        -sin_lon, -cos_lon * sin_lat, cos_lon * cos_lat,
         cos_lon, -sin_lon * sin_lat, sin_lon * cos_lat,
         0,        cos_lat,            sin_lat;

    return R_l2e.transpose() * Vector3f(0, 0, 7.292115e-5);
}
