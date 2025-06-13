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

    void ComplementaryFilter::setInit(Sensor_ID sensor_id, int fs){
        reset();
        my_sensor = sensor_id;
        IMUParams params = getSensorParams(my_sensor);
        std::array<float, 3> br = params.getACCL_BR();
        std::array<float, 3> std = params.getACCL_STD();
        for (int i=0;i<3;i++){
            error_acc += br[i] * br[i];
            error_acc += std[i] * std[i];
        }
        error_acc = sqrt(error_acc);

        setDataRate(fs);
    }

    void ComplementaryFilter::setDataRate(int fs){
        reset();
        IMUParams sensor_params = getSensorParams(my_sensor);
        std::array<float, 3> ARW = sensor_params.getARW();
        std::array<float, 3> BS_ACCL = sensor_params.getACCL_BS();
        std::array<float, 3> STD_ACCL = sensor_params.getACCL_STD();
        float error_gyro[] = {ARW[0] / sqrt(fs), ARW[1] / sqrt(fs)};
        float error_accl[] = {  sqrt(BS_ACCL[1] * BS_ACCL[1] + STD_ACCL[1] * STD_ACCL[1]) / 9.79, 
                                sqrt(BS_ACCL[0] * BS_ACCL[0] + STD_ACCL[0] * STD_ACCL[0]) / 9.79};

        weight[0] = error_gyro[0] / (error_gyro[0] + error_accl[0]);
        weight[1] = error_gyro[1] / (error_gyro[1] + error_accl[1]);
        
        for (int i=0;i<2;i++){
            cov(i, i) = error_gyro[i] * error_gyro[i] + error_accl[i] * error_accl[i];
        }

        LC.resetWindowSize(120 * fs);
    }

    void ComplementaryFilter::setEnableCheckACC(bool enable_check_acc){
        reset();
        this->enable_check_acc = enable_check_acc;
    }
    
    void ComplementaryFilter::setWE_IE_L_Zero(){
        reset();
        WE_IE_L = Vector3f::Zero();
    }

    void ComplementaryFilter::setThreshold(float x_axis, float y_axis, float z_axis){
        reset();
        threshold[0] = x_axis;
        threshold[1] = y_axis;
        threshold[2] = z_axis;
    }

    void ComplementaryFilter::setThresholdBySTD(){
        reset();
        IMUParams sensor_params = getSensorParams(my_sensor);
        for (int i=0;i<3;i++){
            threshold[i] = degrees(3 * sensor_params.getGYRO_STD()[i]);
        }
    }

    void ComplementaryFilter::setPOS(float (&pos)[3]){
        reset();
        float lat = radians(pos[0]);
        float lon = radians(pos[1]);
        g0 = genGravity(lat, pos[2]);
        WE_IE_L = genW_IE_L(lat, lon);
    }

    void ComplementaryFilter::setWindowSizeLC(int wf){
        reset();
        LC.resetWindowSize(wf);
    }

    void ComplementaryFilter::setLevelingConstant(int value){
        reset();
        if (value >= 1 && value <=100) { 
            window_size_acc = value; 
        }
        else {
            Serial.println("Constant should between  1 and 100."); 
        }
    }

    void ComplementaryFilter::setZupward(bool is_upward){
        reset();
        z_upward = is_upward;
    }

    void ComplementaryFilter::resetEuler(float pitch, float roll, float yaw){
        reset();
        dcm.resetOri(pitch, roll, yaw);
    }

    void ComplementaryFilter::reset(){
        dcm.resetOri(0, 0, 0);
        pr_list.clear();
        sum_pr = {0, 0};
        stopLC();
    }

    void ComplementaryFilter::startLC(){
        LC.start();
    }

    void ComplementaryFilter::stopLC(){
        LC.stop();
    }

    void ComplementaryFilter::run(const float t, const float (&omg)[3], const float (&acc)[3]){
        float LC_omg[3];
        float new_acc[3];
        zCalibrate(omg, LC_omg);
        zCalibrate(acc, new_acc);
        if (LC.caliBias(LC_omg)){
            if (pre_time >=0){
                Vector3f vec = Vector3f::Zero();
                for (int i=0;i<3;i++){
                    if (abs(LC_omg[i]) > threshold[i]){
                        vec[i] = LC_omg[i];
                    }
                }
                vec = (radians(vec) - dcm.getR_b2l().transpose() * WE_IE_L) * (t - pre_time);
                dcm.rotate(vec);

                Vector2f pr_acc = accLeveling(new_acc[0], new_acc[1], new_acc[2]);
                pr_list.push_back(pr_acc);
                for (int i=0;i<2;i++) { 
                    sum_pr[i] += pr_list.back()[i]; 
                }
                if (pr_list.size() > window_size_acc){
                    for (int i=0;i<2;i++) { 
                        sum_pr[i] -= pr_list.front()[i]; 
                    }
                    pr_list.pop_front();
                }
                
                float ori_temp[3];
                dcm.getOri(ori_temp);
                if (checkACC(new_acc)){
                    Vector2f new_pr_acc = sum_pr / pr_list.size();
                    float new_pr[2];
                    new_pr[0] = ori_temp[0] - weight[0] * normalizeAngle_float(ori_temp[0] - new_pr_acc(0));
                    new_pr[1] = ori_temp[1] - weight[1] * normalizeAngle_float(ori_temp[1] - new_pr_acc(1));
                    
                    dcm.resetOri(new_pr[0], new_pr[1], ori_temp[2]);
                }
            }

            else{
                Vector2f pr_acc = accLeveling(new_acc[0], new_acc[1], new_acc[2]);
                dcm.resetOri(pr_acc(0), pr_acc(1), 0);
            }

            pre_time = t;
        }
        else{
            dcm.resetOri(0, 0, 0);
        }
    }

    void ComplementaryFilter::getEularAngle(float (&ori)[3]) const{
        float ori_rad[3];
        dcm.getOri(ori_rad);
        // dcm.getOri(ori_rad);
        for (int i=0;i<3;i++){
            ori[i] = degrees(ori_rad[i]);
        }
    }

    void ComplementaryFilter::getCaliOMG(const float (&omg)[3], float (&new_omg)[3]){
        zCalibrate(omg, new_omg);
        LC.calibrate(new_omg);
    }

    void ComplementaryFilter::getCaliACC(const float (&acc)[3], float (&new_acc)[3]){
        zCalibrate(acc, new_acc);
    }
    
    bool ComplementaryFilter::checkACC(const float (&acc)[3]){     
        if (enable_check_acc)
        {    
            float error = sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]) - g0;
            float D = error / error_acc;
            if (D > 2 && D < -2){
                return false;
            }
        }
        return true;
    }
    
    void ComplementaryFilter::zCalibrate(const float (&data)[3], float (&new_data)[3]){
        if (z_upward){
            new_data[0] = data[0];
            new_data[1] = data[1];
            new_data[2] = data[2];
        }
        else{
            new_data[0] = -data[0];
            new_data[1] = data[1];
            new_data[2] = -data[2];
        }
    }



    //----------------------------------------------------------------//
    //                           Kalman Filter                        //
    //----------------------------------------------------------------//

    void KalmanFilter::setInit(Sensor_ID sensor_id){
        my_sensor = sensor_id;
        IMUParams params = getSensorParams(my_sensor);
        std::array<float, 3> gyro_br = params.getGYRO_BR();
        std::array<float, 6> arw = params.getGYRO_6params();
        std::array<float, 3> accl_std = params.getACCL_STD();
        std::array<float, 3> accl_br = params.getACCL_BR();

        PP(0,0) = radians(1) * radians(1);
        PP(1,1) = radians(1) * radians(1);
        PP(2,2) = radians(0.01) * radians(0.01);
        PP(3,3) = gyro_br[0] * gyro_br[0];
        PP(4,4) = gyro_br[1] * gyro_br[1];
        PP(5,5) = gyro_br[2] * gyro_br[2];

        for (int i = 0; i < 3; i++){
            var_omg(i, i) = arw[i] * arw[i];
            RR(i, i) = accl_std[i] * accl_std[i] + accl_br[i] * accl_br[i];
            error_acc += RR(i, i);
        }
        error_acc = sqrt(error_acc);
    }

    void KalmanFilter::setEnableCheckACC(bool enable_check_acc){
        reset();
        this->enable_check_acc = enable_check_acc;
    }

    void KalmanFilter::setWE_IE_L_Zero(){
        reset();
        WE_IE_L = Vector3f::Zero();
    }

    void KalmanFilter::setThreshold(float x_axis, float y_axis, float z_axis){
        reset();
        threshold[0] = x_axis;
        threshold[1] = y_axis;
        threshold[2] = z_axis;
    }

    void KalmanFilter::setThresholdBySTD(){
        reset();
        IMUParams sensor_params = getSensorParams(my_sensor);
        for (int i=0;i<3;i++){
            threshold[i] = degrees(3 * sensor_params.getGYRO_STD()[i]);
        }
    }

    void KalmanFilter::setPOS(float (&pos)[3]){
        reset();
        float lat = radians(pos[0]);
        float lon = radians(pos[1]);
        g0 = genGravity(lat, pos[2]);
        WE_IE_L = genW_IE_L(lat, lon);
    }

    void KalmanFilter::setLevelingConstant(int value){
        reset();
        if (value >= 1 && value <=100) { 
            window_size_acc = value; 
        }
        else {
            Serial.println("Constant should between  1 and 100."); 
        }
    }

    void KalmanFilter::setZupward(bool is_upward){
        reset();
        z_upward = is_upward;
    }

    // Input yaw in degrees
    void KalmanFilter::resetYaw(const float yaw){
        init_yaw = radians(yaw);
        reset();
    }

    void KalmanFilter::reset(){
        LC.stop();
        pre_time = 0;
        dcm.resetOri(0, 0, init_yaw);
        for (int i=0;i<3;i++){
            sum_acc[i] = 0.0f;
            bias_omg[i] = 0.0f;
            bias_omg[i] = 0.0f;
        }
    }

    // Get the result in degrees
    void KalmanFilter::getEularAngle(float (&temp_ori)[3]) const{
        float ori_rad[3];
        dcm.getOri(ori_rad);
        temp_ori[0] = degrees(ori_rad[0]);
        temp_ori[1] = degrees(ori_rad[1]);
        temp_ori[2] = degrees(ori_rad[2]);
    }

    // Get the bias of gyro in dps
    void KalmanFilter::getBias(float (&temp_bias)[3]) const{
        temp_bias[0] = degrees(bias_omg[0]);
        temp_bias[1] = degrees(bias_omg[1]);
        temp_bias[2] = degrees(bias_omg[2]);
    }

    //Get Compensated angular rate
    void KalmanFilter::getCaliOMG(const float (&omg)[3], float (&new_omg)[3]){
        zCalibrate(omg, new_omg);
        LC.calibrate(new_omg);
        new_omg[0] = new_omg[0] - degrees(bias_omg[0]);
        new_omg[1] = new_omg[1] - degrees(bias_omg[1]);
        new_omg[2] = new_omg[2] - degrees(bias_omg[2]);
    }

    void KalmanFilter::getCaliACC(const float (&acc)[3], float (&new_acc)[3]){
        zCalibrate(acc, new_acc);
    }

    bool KalmanFilter::checkALbyGravity(const float(&acc)[3]){
        float abs_acc = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        if (abs(abs_acc - g0) / error_acc >= 3){
            return false;
        }
        return true;
    }

    // void KalmanFilter::checkALByAutocorrelation(const Vector2f& new_pr){       
    //     pr_list.push_back(new_pr); 

    //     if (pr_list.size() > 20) {
    //         pr_list.erase(pr_list.begin());

    //         // 使用 Eigen 計算平均值
    //         Vector2f mean = Vector2f::Zero();
    //         for (const auto& pr : pr_list) {
    //             mean += pr;
    //         }
    //         mean /= pr_list.size();

    //         // 計算 dif
    //         MatrixXf dif(pr_list.size(), 2);
    //         for (int i = 0; i < pr_list.size(); ++i) {
    //             dif.row(i) = pr_list[i] - mean;
    //         }

    //         // 計算 up_term 和 bt_term
    //         MatrixXf up_term_mat = (dif.topRows(dif.rows() - 1).transpose() * dif.bottomRows(dif.rows() - 1)).diagonal();
    //         float up_term = up_term_mat.sum();

    //         MatrixXf bt_term_mat = (dif.transpose() * dif).diagonal();
    //         float bt_term = bt_term_mat.sum();

    //         // 計算 pho 並更新 work
    //         float pho = up_term / bt_term;
    //         if (pho >= 0.6) {
    //             work = false;
    //         }
    //     }

    // }

    void KalmanFilter::zCalibrate(const float (&data)[3], float (&new_data)[3]){
        if (z_upward){
            new_data[0] = data[0];
            new_data[1] = data[1];
            new_data[2] = data[2];
        }
        else{
            new_data[0] = -data[0];
            new_data[1] = data[1];
            new_data[2] = -data[2];
        }
    }

    void KalmanFilter::setWindowSizeLC(int wf){
        reset();
        LC.resetWindowSize(wf);
    }

    //----------------------------------------------------------------//
    //                     Extended-Kalman Filter                     //
    //----------------------------------------------------------------//

    // Input the measurements. Time in sec, angular rate in dps and acceleration in m/s^2
    void EKF::run(const float t, const float (&omg)[3], const float (&acc)[3]){
        //The first epoch would be skiped and the initial pitch, roll will be estimated. Meanwihile, t and acc will be stored.
        if (pre_time > 0){
            float new_omg[3];
            float new_acc[3];
            zCalibrate(omg, new_omg);
            zCalibrate(acc, new_acc);

            if (!LC.caliBias(new_omg)){
                return;
            }

            float dt = t - pre_time;
            Vector3f vec = Vector3f::Zero();
            for (int i=0;i<3;i++){
                if (abs(new_omg[i]) > threshold[i]){
                    vec[i] = radians(new_omg[i]) - bias_omg[i];
                }
            }
            vec = (vec - dcm.getR_b2l().transpose() * WE_IE_L) * dt;
            if (!is_predict){
                predict(dt);
            }

            dcm.rotate(vec);
            
            if (is_predict){
                if (window_size_acc > 1){
                    // Leveling Constant
                    float mean_acc[3];
                    acc_list.push_back(std::vector<float>(new_acc, new_acc + 3));
                    for (int i=0;i<3;i++){ sum_acc[i] += new_acc[i]; }
                    if (acc_list.size() > window_size_acc){
                        for (int i=0;i<3;i++){ sum_acc[i] -= acc_list.front()[i]; }
                        acc_list.pop_front();
                    }
                    for (int i=0;i<3;i++){
                        mean_acc[i] = sum_acc[i] / acc_list.size();
                    }
                    update(mean_acc);
                }

                else{
                    update(acc);
                }
            }
            is_predict = !is_predict;
        }
        else{
            Vector2f pr = accLeveling(acc[0], acc[1], acc[2]);
            dcm.resetOri(pr(0), pr(1), init_yaw);
        }
        pre_time = t;
    }

    void EKF::predict(const float dt){
        Matrix<float, 6, 6> A = MatrixXf::Zero(6,6);
        A.block<3,3>(0,3) = dcm.getR_b2l();
        Matrix<float, 6, 6> FF = MatrixXf::Identity(6,6) + A * dt;

        Matrix<float, 6, 3> G = Matrix<float, 6, 3>::Zero(); 
        G.block<3,3>(0,0) = dcm.getR_b2l();
        Matrix<float, 6, 6> QQ = FF * G * var_omg * G.transpose() * FF.transpose() * dt;

        PP = FF * PP * FF.transpose() + QQ;
    }

    bool EKF::update(const float (&acc)[3]){
        if (checkALbyGravity(acc)){
            Vector3f f_l = dcm.getR_b2l() * Vector3f(acc[0], acc[1], acc[2]);
            Vector3f ZZ = {f_l(0), f_l(1), f_l(2) - g0};

            MatrixXf HH = MatrixXf::Zero(3,6);
            HH.block<3,3>(0,0) = vector2skew(-f_l);

            MatrixXf K = PP * HH.transpose() * (HH * PP * HH.transpose() + RR).inverse();
            MatrixXf dx = K * ZZ;
            PP = (MatrixXf::Identity(6,6) - K * HH) * PP;
            // PP = (MatrixXf::Identity(6,6) - K * HH) * PP * (MatrixXf::Identity(6,6) - K * HH).transpose() + K * RR * K.transpose();
            
            // Compensation
            Vector3f vec(-dx(0), -dx(1), -dx(2));
            dcm.kf_correct(vec);
            bias_omg[0] += dx(3);
            bias_omg[1] += dx(4);
            bias_omg[2] += dx(5);
            return true;
        }
        return false;
    }


    //----------------------------------------------------------------//
    //                    Unscented-Kalman Filter                     //
    //----------------------------------------------------------------//
    #ifdef UKF_H
    // Initializes the Kalman Filter with the given initial orientation (specified by Euler angles in radians) and the window size for linear correction.
    UKF::UKF(const float pitch, const float roll, const float yaw, const int ws_lc) : KalmanFilter(yaw){
        // Create wieghts
        int LD = 1;
        int L = 2;
        int num_points = 2 * L + LD;
        W[0] = float(LD) / float(L+LD);
        for (int i = 0; i < num_points-1; i++){
            W[i+1] = float(LD) / float(L + LD) / 2.f;
        }
    }

    // Input measurements. Time (sec), angular rate (dps) and acceleration (m/s^2)
    void UKF::run(const float t, const float (&omg)[3], const float (&acc)[3]){
        if (pre_time > 0){
            float dt = t - pre_time;
            float new_omg[3];
            float new_acc[3];
            zCalibrate(omg, new_omg);
            zCalibrate(acc, new_acc);
            Vector3f vec = Vector3f::Zero();
            for (int i=0;i<3;i++){
                if (abs(new_omg[i]) > threshold[i]){
                    vec[i] = radians(new_omg[0]) - bias_omg[0];
                }
            }
            vec = (vec - dcm.getR_b2l().transpose() * WE_IE_L) * dt;
            genQ(dt);
            dcm.rotate(vec);  
            
            // genZ need to be updated
            // genZ(acc);  
            update();
            compensate();
        }

        else{
            Vector2f pr = accLeveling(acc);
            dcm.resetOri(pr(0), pr(1), init_yaw);
        }
        pre_time = t;
    }

    void UKF::update(){
        int LD = 1;
        int L = 2;
        int num_points = 2 * L + LD;

        // Calculate sigma points
        Matrix2f A = ((LD+L)*PP).llt().matrixL(); // Cholesky decomposition

        // Create a vector of sigma points
        Matrix<float, 5, 3> sigma_points;
        sigma_points << dx.transpose(), A.row(0), A.row(1), -A.row(0), -A.row(1);

        // predict_status
        Matrix<float, 3, 5> predict_sigma_points = FF * sigma_points.transpose();
        Vector2f X_ = Vector2f::Zero();

        for (int i = 0; i < num_points ;i++){
            X_ += W[i] * predict_sigma_points.col(i);
        }

        Matrix<float, 3, 5> dif_x = predict_sigma_points - X_.replicate(1, 5);
        Matrix3f P_ = Matrix2f::Zero() + QQ;
        for (int i = 0; i < num_points ;i++){
            P_ += W[i] * (dif_x.col(i) * dif_x.col(i).transpose());
        }

        //predict measurements
        Matrix<float, 3, 5> Y = HH * predict_sigma_points;
        Vector3f Y_ = Vector2f::Zero();
        for (int i = 0; i < num_points ;i++){
            Y_ += W[i] * Y.col(i);
        }

        Matrix<float, 3, 5> dif_y = Y - Y_.replicate(1, 5);
        Matrix3f Pyy = Matrix3f::Zero() + RR;
        Matrix3f Pxy = Matrix3f::Zero();
        for (int i = 0; i < num_points ;i++){
            Pyy += W[i] * (dif_y.col(i) * dif_y.col(i).transpose());
            Pxy += W[i] * (dif_x.col(i) * dif_y.col(i).transpose());
        }

        //measurement update
        Matrix2f K = Pxy * Pyy.inverse();
        dx = X_ + K * (ZZ - Y_);
        PP = P_ - K * Pyy * K.transpose();
    }
    #endif

    

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
        count = - 10;
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

    bool LinearCorrection::caliBias(float (&data)[3]) {
        if (!is_biasCalculated){
            if (bias[0] == 0){
                if  (count < 0){ count++; }
                else if (count < window_size) {
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

        calibrate(data);
        return is_biasCalculated;
    }

    void LinearCorrection::calibrate(float (&data)[3]){
        if (is_biasCalculated){            
            for(int i=0; i<3; i++){ data[i] -= bias[i]; }
        }
        else{
            for(int i=0; i<3; i++){ data[i] = 0; }
        }
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
    float pitch = atan2(fy, max(sqrt(fx*fx + fz*fz), 1e-8));
    float roll = 0;
    if (fz > 0){
        roll = atan2(-fx, sqrt(fz*fz + 1e-6*fx*fx));
    }
    else{
        roll = atan2(-fx, -sqrt(fz*fz + 1e-6*fx*fx));
    }    
    
    Vector2f pr(pitch, roll);
    return pr;
}

// angle in radian
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

//angle in radian 
float normalizeAngle_float(float angle){
    
    float new_angle = fmod(angle + M_PI, 2 * M_PI) - M_PI;

    if (new_angle > M_PI) {
        new_angle -= 2 * M_PI;
    }

    if (new_angle <= -M_PI) {
        new_angle += 2 * M_PI;
    }
    return new_angle;   
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
