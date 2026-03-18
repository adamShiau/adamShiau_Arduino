#ifndef MYI2CSENSOR_H
#define MYI2CSENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include "myMessage.h"


#ifdef ARDUINO_ARDUINO_NANO33BLE
#include "BMI270-Sensor-API/bmi270.h"
#include "src/BMM150-Sensor-API/bmm150.h"
// #define ENABLE_GUESTURE_SENSOR
#define INT16_to_G   (8192.0f)                   // 2^16 / (2 * 4) = 8192
#define INT16_to_DPS   (16.384f * (2000/500))   // 2^16 / (2 * 2000) = 16.384
#define GRAVITY 9.7895f
#define INT_PIN_BMI270 p11

typedef enum {
    BOSCH_ACCELEROMETER_ONLY,
    BOSCH_MAGNETOMETER_ONLY,
    BOSCH_ACCEL_AND_MAGN
} CfgBoshSensor_t;
  
struct dev_info {
    TwoWire* _wire;
    uint8_t dev_addr;
};

class BoschSensorClass {
    public:
        BoschSensorClass(TwoWire& wire = Wire);
        ~BoschSensorClass() {}
        int begin(CfgBoshSensor_t cfg = BOSCH_ACCEL_AND_MAGN);
        void debug(Stream&);
        #ifdef __MBED__
        void onInterrupt(mbed::Callback<void()>);
        void setInterruptPin(PinName irq_pin) {
        BMI270_INT1 = irq_pin;
        }
        void setInterruptPin(pin_size_t irq_pin) {
        BMI270_INT1 = digitalPinToPinName(irq_pin);
        }
        PinName BMI270_INT1 = NC;
        #endif

        // IMU
        virtual int getIMUData(float (&omg)[3], float (&acc)[3]);
        virtual int accelerationAvailable(); // Number of samples in the FIFO.
        virtual float accelerationSampleRate(); // Sampling rate of the sensor.
        virtual int gyroscopeAvailable(); // Number of samples in the FIFO.
        virtual float gyroscopeSampleRate(); // Sampling rate of the sensor.
        
        // Magnetometer
        virtual int getMAGData(float (&mag)[3]); // Results are in uT (micro Tesla).
        virtual int magneticFieldAvailable(); // Number of samples in the FIFO.
        virtual float magneticFieldSampleRate(); // Sampling rate of the sensor.
        

    protected:
        // can be modified by subclassing for finer configuration
        virtual int8_t configure_sensor(struct bmm150_dev *dev);
        virtual int8_t configure_sensor(struct bmi2_dev *dev);

    private:
        static int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
        static int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
        static void bmi2_delay_us(uint32_t period, void *intf_ptr);
        void interrupt_handler();
        void print_rslt(int8_t rslt);

    private:
        TwoWire* _wire;
        Stream* _debug = nullptr;
        #ifdef __MBED__
        mbed::Callback<void(void)> _cb;
        #endif
        bool _initialized = false;
        int _interrupts = 0;
        struct dev_info accel_gyro_dev_info;
        struct dev_info mag_dev_info;
        struct bmi2_dev bmi2;
        struct bmm150_dev bmm1;
        uint16_t _int_status;
};


// Barometer
#define LPS22HB_ADDRESS  0x5C
#define LPS22HB_WHO_AM_I_REG        0x0f
#define LPS22HB_CTRL2_REG           0x11
#define LPS22HB_STATUS_REG          0x27
#define LPS22HB_PRESS_OUT_XL_REG    0x28
#define LPS22HB_PRESS_OUT_L_REG     0x29
#define LPS22HB_PRESS_OUT_H_REG     0x2a
#define LPS22HB_TEMP_OUT_L_REG      0x2b
#define LPS22HB_TEMP_OUT_H_REG      0x2c

  
class LPS22HBClass {
    public:
        LPS22HBClass(TwoWire& wire);
        int begin();
        void end();
        bool getBARData(float &bar);
        bool getTEMPData(float &temp);

    private:
        int i2cRead(uint8_t reg);
        int i2cWrite(uint8_t reg, uint8_t val);
        int counter = 0;

    private:
        TwoWire* _wire;
        bool _initialized;
};


// Gesture and color sensor 
#define APDS9960_ADDR 0x39

enum {
    GESTURE_NONE = -1,
    GESTURE_UP = 0,
    GESTURE_DOWN = 1,
    GESTURE_LEFT = 2,
    GESTURE_RIGHT = 3
};
  
class APDS9960 {
    public:
        APDS9960(TwoWire &wire, int intPin);
        virtual ~APDS9960();
    
        bool begin();
        void end();
        int gestureAvailable();
        int readGesture();
        int colorAvailable();
        bool readColor(int& r, int& g, int& b);
        bool readColor(int& r, int& g, int& b, int& c);
        int proximityAvailable();
        int readProximity();
        void setGestureSensitivity(uint8_t sensitivity);
        void setInterruptPin(int pin);
        bool setLEDBoost(uint8_t boost);
        int getInterrputPin();
    
    private:
        bool setGestureIntEnable(bool en);
        bool setGestureMode(bool en);
        int gestureFIFOAvailable();
        int handleGesture();
    
        bool enablePower();
        bool disablePower();
        bool enableColor();
        bool disableColor();
        bool enableProximity();
        bool disableProximity();
        bool enableWait();
        bool disableWait();
        bool enableGesture();
        bool disableGesture();
    
    private:
        TwoWire& _wire;
        int _intPin;
    
        bool _gestureEnabled;
        bool _proximityEnabled;
        bool _colorEnabled;
        bool _gestureIn;
        int _gestureDirectionX;
        int _gestureDirectionY;
        int _gestureDirInX;
        int _gestureDirInY;
        int _gestureSensitivity;
        int _detectedGesture;
    
        bool write(uint8_t val);
        bool write(uint8_t reg, uint8_t val);
        bool read(uint8_t reg, uint8_t *val);
        size_t readBlock(uint8_t reg, uint8_t *val, unsigned int len);
    
    private:
        #define REG(name, addr) \
            bool get##name(uint8_t *val) { return read(addr,  val); } \
            bool set##name(uint8_t val)  { return write(addr, val); } \
            size_t read##name(uint8_t *val, uint8_t len) { return readBlock(addr, val, len); }
            REG(ENABLE,     0x80)
            REG(ATIME,      0x81)
            REG(WTIME,      0x83)
            REG(AILTL,      0x84)
            REG(AILTH,      0x85)
            REG(AIHTL,      0x86)
            REG(AIHTH,      0x87)
            REG(PILT,       0x89)
            REG(PIHT,       0x8B)
            REG(PERS,       0x8C)
            REG(CONFIG1,    0x8D)
            REG(PPULSE,     0x8E)
            REG(CONTROL,    0x8F)
            REG(CONFIG2,    0x90)
            REG(ID,         0x92)
            REG(STATUS,     0x93)
            REG(CDATAL,     0x94)
            REG(CDATAH,     0x95)
            REG(RDATAL,     0x96)
            REG(RDATAH,     0x97)
            REG(GDATAL,     0x98)
            REG(GDATAH,     0x99)
            REG(BDATAL,     0x9A)
            REG(BDATAH,     0x9B)
            REG(PDATA,      0x9C)
            REG(POFFSET_UR, 0x9D)
            REG(POFFSET_DL, 0x9E)
            REG(CONFIG3,    0x9F)
            REG(GPENTH,     0xA0)
            REG(GEXTH,      0xA1)
            REG(GCONF1,     0xA2)
            REG(GCONF2,     0xA3)
            REG(GOFFSET_U,  0xA4)
            REG(GOFFSET_D,  0xA5)
            REG(GPULSE,     0xA6)
            REG(GOFFSET_L,  0xA7)
            REG(GOFFSET_R,  0xA9)
            REG(GCONF3,     0xAA)
            REG(GCONF4,     0xAB)
            REG(GFLVL,      0xAE)
            REG(GSTATUS,    0xAF)
            REG(IFORCE,     0xE4)
            REG(PICLEAR,    0xE5)
            REG(CICLEAR,    0xE6)
            REG(AICLEAR,    0xE7)
            REG(GFIFO_U,    0xFC)
            REG(GFIFO_D,    0xFD)
            REG(GFIFO_L,    0xFE)
            REG(GFIFO_R,    0xFF)
};

extern BoschSensorClass sensor;
extern LPS22HBClass baro;
#ifdef ENABLE_GUESTURE_SENSOR
APDS9960 gesture;
#endif


#elif (WIRE_INTERFACES_COUNT > 0)
#include "STM32duino_LSM6DS3/LSM6DS3Sensor.h"

class Nano33IOT_IMU{
    private:
        LSM6DS3Sensor AccGyr;
    public:
        Nano33IOT_IMU();
        bool begin();
        void getIMUData(float (&omg)[3], float (&acc)[3]);
};

extern Nano33IOT_IMU sensor;
#endif

#endif

