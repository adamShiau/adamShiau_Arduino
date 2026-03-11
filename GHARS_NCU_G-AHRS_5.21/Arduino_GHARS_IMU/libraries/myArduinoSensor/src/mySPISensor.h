#ifndef MYSENSOR_H
#define MYSENSOR_H

#include <SPI.h>
#include <queue>
#include "myMessage.h"

#define FILT_CTRL    	0x5C  //Filter control
#define MSC_CTRL    	0x60  //Miscellaneous control
#define UP_SCALE    	0x62  //Clock scale factor, PPS mode
#define DEC_RATE    	0x64  //Decimation rate control (output data rate)
#define DIAG_STAT     0x02
#define PROD_ID    		0x72  //Product identification 
#define SERIAL_NUM    0x74  //Serial number (relative to assembly lot)
#define X_ACCL_LOW  	0x10  //X-axis accelerometer output, lower word
#define X_ACCL_OUT  	0x12  //X-axis accelerometer output, upper word
#define Y_ACCL_LOW  	0x14  //Y-axis accelerometer output, lower word
#define Y_ACCL_OUT  	0x16  //Y-axis accelerometer output, upper word
#define Z_ACCL_LOW  	0x18  //Z-axis accelerometer output, lower word
#define Z_ACCL_OUT  	0x1A  //Z-axis accelerometer output, upper word
#define RANG_MDL 0x5E
#define ACCL_16bit_SF 0.002446875   // (m/s^2) / LSB
#define ACCL_32bit_SF 26756268.0      // LSB/(m/sec2) 
#define GYRO_16bit_SF 40.0            // LSB/°/sec
#define GYRO_32bit_SF 2621440.0       // LSB/°/sec
#define NUM_DATAPACKAGE_16BIT 18
#define NUM_DATAPACKAGE_32BIT 30


class ADIS16505{
    public:
        ADIS16505();
        ADIS16505(uint16_t CS, uint16_t DRDY, u_int8_t RST);
        void initSensor();
        bool readData(float (&omg)[3], float (&acc)[3], float &temp, int32_t &ct);
        bool checkDRDY();

    private:
        u_int8_t CS;
        u_int8_t DRDY;
        u_int8_t RST;
        int32_t counter = 0;

        void select();
        void deselect();
        int16_t readReg(uint8_t regAddr);
        int regWrite(uint8_t regAddr, int16_t regData); 
        bool checksum(uint8_t * burstArray, uint8_t num_bytes);
};


class GYPRO4300{
    public:
        GYPRO4300();
        GYPRO4300(uint16_t CS, uint16_t DRDY, uint16_t EN, uint16_t VDDIO, int WS=1);
        void initSensor();
        bool readData(float &omg, float &temp);
        bool checkDRDY();
        void disableCalibrationMode();
        
    private:
        u_int8_t CS;
        u_int8_t DRDY;
        u_int8_t EN;
        u_int8_t VDDIO;
        int WS = 1;
        int counter = 0;
        float sum_omg = 0;
        // float sum_temp = 0;
        std::queue<float> vec_omg;
        // std::queue<float> vec_temp;
        
        void select();
        void deselect();
        void readOutput(uint8_t address, uint8_t Buffer_Sensor[], uint8_t Buffer_Size);
        uint32_t readReg(uint32_t regAddr);
        void writeReg(uint32_t Data, uint32_t Address);
};

#endif