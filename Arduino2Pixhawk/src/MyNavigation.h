#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "Orientation.h"
#include "SensorParams.h"
#include "DataOutput.h"
#include <map>
#include <deque>


using namespace Eigen;

namespace Navigation{

    //----------------------------------------------------------------//
    //                       LinearCorrection                         //
    //----------------------------------------------------------------//

    class LinearCorrection {
        private:
            int window_size;
            int count;
            float sum[3];
            float bias[3];
            bool is_biasCalculated = true;

        public:
            LinearCorrection(const int ws);
            bool caliBias(float (&data)[3]);
            void calibrate(float (&data)[3]);
            bool isReady() const;
            void setBias(float* bias_omg);
            void start();
            void stop();
            void resetWindowSize(int ws);
    };

    
    //----------------------------------------------------------------//
    //                      Complementary Filter                      //
    //----------------------------------------------------------------//

    class ComplementaryFilter{
        private:
            MyDirectCosineMatrix::DirectCosineMatrix dcm;
            LinearCorrection LC = LinearCorrection(12000);
            float threshold[3] = {0, 0, 0};  //degree
            float weight[2] = {1, 1};
            float pre_time = -1;
            float g0 = 9.7895;
            bool z_upward = true;
            Vector3f WE_IE_L = Vector3f::Zero();
            Sensor_ID my_sensor;

            // Leveling Constant
            std::deque<Vector2f> pr_list;
            int window_size_acc = 1;
            Vector2f sum_pr = {0, 0};
            Matrix2f cov = Matrix2f::Identity();
            
            // check ACC
            bool enable_check_acc = true;
            float error_acc = 0;
            bool checkACC(const float (&abs_acc)[3]);
            void zCalibrate(const float (&data)[3], float (&new_data)[3]);

        public:
            ComplementaryFilter();
            ComplementaryFilter(float ini_ori[3]);
            void setInit(Sensor_ID sensor_id, int fs=100);
            void setDataRate(int fs);
            void setEnableCheckACC(bool enable_check_acc);
            void setWE_IE_L_Zero();
            void setThreshold(float x_axis, float y_axis, float z_axis);
            void setThresholdBySTD();
            void setPOS(float (&pos)[3]);  // units: lat(deg), lon(deg), hei(meters)
            void setWindowSizeLC(int wf);
            void setLevelingConstant(int value);
            void setZupward(bool is_upward);
            void resetEuler(float pitch, float roll, float yaw);
            void reset();
            
            void startLC();
            void stopLC();
            
            void run(const float t, const float (&omg)[3], const float (&acc)[3]);
            void getEularAngle(float (&ori)[3]) const;
            void getCaliOMG(const float (&omg)[3], float (&new_omg)[3]);
            void getCaliACC(const float (&acc)[3], float (&new_acc)[3]);
            bool isAvailable() {return LC.isReady();};
    };

    //----------------------------------------------------------------//
    //                         Kalman Filter                          //
    //----------------------------------------------------------------//

    class KalmanFilter{
        protected:
            MyDirectCosineMatrix::DirectCosineMatrix dcm;
            LinearCorrection LC = LinearCorrection(100);
            Sensor_ID my_sensor;

            // Kalman Filter Parameters
            MatrixXf PP = MatrixXf::Identity(6,6);
            Matrix3f RR = Matrix3f::Identity(3,3);
            Matrix3f var_omg = Matrix3f::Zero();
            Vector3f WE_IE_L = Vector3f::Zero();
            float bias_omg[3] = {0.0f, 0.0f, 0.0f};

            // Leveling Constant Parameters
            std::deque<std::vector<float>> acc_list;
            float sum_acc[3] = {0, 0, 0};
            int window_size_acc = 1;
            
            bool enable_check_acc = true;
            bool z_upward = true;

            float threshold[3] = {0, 0, 0};
            float error_acc = 0.0f;
            float pre_time = 0.0f;
            float init_yaw = 0.0f;
            float g0 = 9.7895f;

            bool checkALbyGravity(const float(&acc)[3]);
            // void checkALByAutocorrelation(const Vector2f& new_pr);
            void zCalibrate(const float (&data)[3], float (&new_data)[3]);

        public:
            // Initialize Kalman Filter with initial orientation by euler angle in radians and Linear-correction window size (ws_lc).
            KalmanFilter(const float yaw=0.0f, Sensor_ID sensor=Nano33) : dcm(0, 0, radians(yaw)), my_sensor(sensor){};
            void setInit(Sensor_ID sensor_id);
            void setEnableCheckACC(bool enable_check_acc);
            void setWE_IE_L_Zero();
            void setThreshold(float x_axis, float y_axis, float z_axis);
            void setThresholdBySTD();
            void setPOS(float (&pos)[3]);  // units: lat(deg), lon(deg), hei(meters)
            void setLevelingConstant(int value);
            void setZupward(bool is_upward);
            void setWindowSizeLC(int wf);

            void resetYaw(const float yaw);
            void reset();

            void getEularAngle(float (&temp_ori)[3]) const;
            void getBias(float (&temp_bias)[3]) const;
            void getCaliOMG(const float (&omg)[3], float (&new_omg)[3]);
            void getCaliACC(const float (&acc)[3], float (&new_acc)[3]);

            void startLC() {LC.start();};
            void stopLC()  {LC.stop();};
    };

    //----------------------------------------------------------------//
    //                    Extended-Kalman Filter                      //
    //----------------------------------------------------------------//

    class EKF : public KalmanFilter{
        private:
            bool is_predict = true;
            void predict(const float dt);
            bool update(const float (&acc)[3]);
        public:
            EKF() : KalmanFilter(){};
            void run(const float t, const float (&omg)[3], const float (&acc)[3]);
    };

    //----------------------------------------------------------------//
    //                    Unscented-Kalman Filter                     //
    //----------------------------------------------------------------//
    #ifdef UKF_H
    class UKF : public KalmanFilter{
        private:
            float W[5];
            void update();
        public:
            UKF(const float pitch, const float roll, const float yaw, const int ws_lc);
            void run(const float t, const float (&omg)[3], const float (&acc)[3]);
    }; 
    #endif   
};


// generate gravity by position
float genGravity(const float &lat, const float &height);

// generate pitch and roll by acceleration measurement
Vector2f accLeveling(const float &fx, const float &fy, const float &fz);

// normalize angles to positive and negative half of pi in radians
float normalizeAngle(float *angle, int size);
float normalizeAngle_float(float angle);

// generate earth rotation on certain position
Vector3f genW_IE_L(float lat, float lon);


#endif