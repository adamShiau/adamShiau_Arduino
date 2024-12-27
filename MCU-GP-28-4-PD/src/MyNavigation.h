#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "Orientation.h"
#include "SensorParams.h"
#include "DataOutput.h"
#include <map>


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
            bool isBiasAvailable(float (&data)[3]);
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
            float weight[2] = {1, 1};
            float pre_time = -1;
            float g0 = 9.7895;
            float threshold[3] = {0, 0, 0};  //degree
            Vector3f WE_IE_L = Vector3f::Zero();
            bool enable_check_acc = false;
            std::vector<Vector2f> pr_list; 
            Sensor_ID my_sensor;
            bool checkALByAutocorrelation(const Vector2f& new_pr);

        public:
            ComplementaryFilter();
            ComplementaryFilter(float ini_ori[3]);
            void setIMUError(Sensor_ID sensor_id, int fs);
            void setDataRate(int fs);
            void run(const float t, const float (&omg)[3], const float (&acc)[3]);
            void getEularAngle(float (&ori)[3]) const;
            void enableCheckACC(bool enable_check_acc);

            // units: lat(deg), lon(deg), hei(meters)
            void setPOS(float (&pos)[3]);
            void setWE_IE_L_Zero();
            void resetEuler(float pitch, float roll, float yaw);
            void setThreshold(float x_axis, float y_axis, float z_axis);
            void setThresholdBySTD();
            void startLC();
            void stopLC();
            void setWindowSizeLC(int wf);
    };

    //----------------------------------------------------------------//
    //                         Kalman Filter                          //
    //----------------------------------------------------------------//

    class KalmanFilter{
        protected:
            Matrix3f var_omg = Matrix3f::Zero();
            MatrixXf FF = MatrixXf::Zero(6,6);
            MatrixXf QQ = MatrixXf::Zero(6,6);
            MatrixXf PP = MatrixXf::Identity(6,6);
            Matrix2f RR = Matrix2f::Identity(2,2);
            MatrixXf HH = MatrixXf::Identity(2,6);
            VectorXf dx = VectorXf::Zero(6);
            Vector2f ZZ = Vector2f::Zero();
            std::vector<Vector2f> pr_list; 
            bool work = true;
            float ma_value = -1;
            float pre_acc[3]={0,0,0};
            float Z_list[100][2];
            float bias_omg[3] = {0.0f, 0.0f, 0.0f};
            int counter = 0;
            MyQuaternion::Quaternion qut;
            LinearCorrection LC;
            void genF(const float t,const float dt);
            void genQ(const float dt);
            void genH();
            void genZ(const float (&acc)[3]);
            void genR();
            void compensate();           
            Vector2f accLeveling(const float (&acc)[3]);
            void checkALbyGravity(const float(&acc)[3]);
            void checkALByAutocorrelation(const Vector2f& new_pr);
            void checkResidual();
            void normalizeAcc(const float (&ori_acc)[3], float (&new_acc)[3]) const;
            void copyACC(const float (&acc)[3]);
        public:
            // Initialize Kalman Filter with initial orientation by euler angle in radians and Linear-correction window size (ws_lc).
            KalmanFilter(const float pitch, const float roll, const float yaw, const int ws_lc) : LC(ws_lc), qut(pitch, roll, yaw){};

            void setInit(const float (&std_imu)[6]);
            
            // Input orientation in radians
            void setInitOri(const float pitch, const float roll, const float yaw);

            // Input yaw in degrees
            void resetYaw(const float yaw);

            // Get the result in degrees
            void getEularAngle(float (&temp_ori)[3]) const;

            // Get the bias of gyro in dps
            void getBias(float (&temp_bias)[3]) const;

            //Get Compensated angular velocity
            void getCompensatedGYRO(float (&raw_omg)[3], float(&cali_omg)[3]) const;
    };

    //----------------------------------------------------------------//
    //                    Extended-Kalman Filter                      //
    //----------------------------------------------------------------//

    class EKF : public KalmanFilter{
        private:
            float t0 = 0.0f;
            float pre_time = 0.0f;
            void predict(const float t, const float dt);
            void update(const float (&acc)[3]);
        public:
            // Initialize Kalman Filter with initial orientation by euler angle in radians and Linear-correction window size (ws_lc).
            EKF(const float pitch, const float roll, const float yaw, const int ws_lc, Sensor_ID sensor);

            // Input the measurements. Time in sec, angular rate in dps and acceleration in m/s^2
            void run(const float t, const float (&omg)[3], const float (&acc)[3]);
    };

    //----------------------------------------------------------------//
    //                    Unscented-Kalman Filter                     //
    //----------------------------------------------------------------//

    class UKF : public KalmanFilter{
        private:
            float W[5];
            void update();
        public:
            // Initializes the Kalman Filter with the given initial orientation (specified by Euler angles in radians) and the window size for linear correction.
            UKF(const float pitch, const float roll, const float yaw, const int ws_lc);

            // Input measurements. Time (sec), angular rate (dps) and acceleration (m/s^2)
            void run(const float &dt, const float (&omg)[3], const float (&acc)[3]);
    };    
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