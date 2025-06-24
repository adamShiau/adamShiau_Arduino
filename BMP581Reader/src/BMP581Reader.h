#ifndef BMP581READER_H
#define BMP581READER_H

#include <Arduino.h>
#include <Wire.h>
#include "bmp5_api/bmp5.h"


class BMP5_Sensor{
    public:
        BMP5_Sensor(TwoWire& wire = Wire);
        ~BMP5_Sensor(){};
        bool begin();
        bool setODR(uint8_t odr);
        bool getSensorData(float *bar, float* temp);

    protected:
        TwoWire *wire;
        bmp5_dev dev;
        bmp5_osr_odr_press_config osr_odr_press_cfg = {0,0,0,0};
        
        static BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
        static BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
        static void bmp5_delay_us(uint32_t period_us, void *intf_ptr) {delayMicroseconds(period_us);}
        void bmp5_error_codes_print_result(const char api_name[], int8_t rslt);

        int8_t initSensor();
};


#endif