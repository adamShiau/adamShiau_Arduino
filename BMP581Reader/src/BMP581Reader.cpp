#include "BMP581Reader.h"



BMP5_Sensor::BMP5_Sensor(TwoWire& wire){
    this->wire = &wire;
}


BMP5_INTF_RET_TYPE BMP5_Sensor::bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr){
    TwoWire *wire = (TwoWire*) intf_ptr;
    wire->beginTransmission(BMP5_I2C_ADDR_SEC);
    wire->write(reg_addr);
    if (wire->endTransmission()) return BMP5_E_COM_FAIL;

    wire->requestFrom(BMP5_I2C_ADDR_SEC, length);
    for(uint32_t i = 0; i < length && wire->available(); i++)
    {
        reg_data[i] = wire->read();
    }
    return BMP5_OK;
}

BMP5_INTF_RET_TYPE BMP5_Sensor::bmp5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr){
    if (length <= 0) return BMP5_E_COM_FAIL;

    TwoWire *wire = (TwoWire*) intf_ptr;
    wire->beginTransmission(BMP5_I2C_ADDR_SEC);
    wire->write(reg_addr);
    for(uint32_t i = 0; i < length; i++)
    {
        wire->write(reg_data[i]);
    }
    if (wire->endTransmission()) return BMP5_E_COM_FAIL;

    return BMP5_OK;
}

bool BMP5_Sensor::begin(){
  dev.intf = BMP5_I2C_INTF;
  dev.read = bmp5_i2c_read;
  dev.write = bmp5_i2c_write;
  dev.delay_us = bmp5_delay_us;
  dev.intf_ptr = wire;
  osr_odr_press_cfg = {0,0,0,0};

  return initSensor() == BMP5_OK;
}

bool BMP5_Sensor::setODR(uint8_t odr)
{
    // Set the output data rate
    switch (odr)
    {
    case 100:
        osr_odr_press_cfg.odr = BMP5_ODR_100_2_HZ;
        break;
    case 50:
        osr_odr_press_cfg.odr = BMP5_ODR_50_HZ;
        break;
    case 20:
        osr_odr_press_cfg.odr = BMP5_ODR_20_HZ;
        break;
    case 10:
        osr_odr_press_cfg.odr = BMP5_ODR_10_HZ;
        break;
    case 5:
        osr_odr_press_cfg.odr = BMP5_ODR_05_HZ;
        break;
    case 2:
        osr_odr_press_cfg.odr = BMP5_ODR_02_HZ;
        break;
    case 1:
        osr_odr_press_cfg.odr = BMP5_ODR_01_HZ;
        break;
    default:
        return false;
    }
    int8_t rslt = bmp5_set_osr_odr_press_config(&osr_odr_press_cfg, &dev);
    bmp5_error_codes_print_result("bmp5_set_osr_odr_press_config", rslt);
    return (rslt == BMP5_OK);
}

void BMP5_Sensor::bmp5_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMP5_OK)
    {
        Serial.print(api_name);
        Serial.print('\t');

        if (rslt == BMP5_E_NULL_PTR)
        {
            Serial.print("Error [");
            Serial.print(rslt);
            Serial.println("] : Null pointer");
        }
        else if (rslt == BMP5_E_COM_FAIL)
        {
            Serial.print("Error [");
            Serial.print(rslt);
            Serial.println("] : Communication failure");
        }
        else if (rslt == BMP5_E_DEV_NOT_FOUND)
        {
            Serial.print("Error [");
            Serial.print(rslt);
            Serial.println("] : Device not found");
        }
        else if (rslt == BMP5_E_INVALID_CHIP_ID)
        {
            Serial.print("Error [");
            Serial.print(rslt);
            Serial.println("] : Invalid chip id");
        }
        else if (rslt == BMP5_E_POWER_UP)
        {
            Serial.print("Error [");
            Serial.print(rslt);
            Serial.println("] : Power up error");
        }
        else if (rslt == BMP5_E_POR_SOFTRESET)
        {
            Serial.print("Error [");
            Serial.print(rslt);
            Serial.println("] : Power-on reset/softreset failure");
        }
        else if (rslt == BMP5_E_INVALID_POWERMODE)
        {
            Serial.print("Error [");
            Serial.print(rslt);
            Serial.println("] : Invalid powermode");
        }
        else
        {
            // For more error codes refer "*_defs.h"
            Serial.print("Error [");
            Serial.print(rslt);
            Serial.println("] : Unknown error code");
        }
    }
}

int8_t BMP5_Sensor::initSensor(){
    int8_t rslt;
    struct bmp5_iir_config set_iir_cfg;
    struct bmp5_int_source_select int_source_select;
    
    // reset BMP581
    rslt = bmp5_soft_reset(&dev);
    bmp5_error_codes_print_result("bmp5_soft_reset", rslt);
    if (rslt != BMP5_OK) return rslt;

    // initial BMP581
    rslt = bmp5_init(&dev);
    bmp5_error_codes_print_result("bmp5_init", rslt);
    if (rslt != BMP5_OK) return rslt;

    // set BMP581 power mode
    rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &dev);
    bmp5_error_codes_print_result("bmp5_set_power_mode1", rslt);
    if (rslt != BMP5_OK) return rslt;

    // enable pressure data
    osr_odr_press_cfg.press_en = BMP5_ENABLE;
    rslt = bmp5_set_osr_odr_press_config(&osr_odr_press_cfg, &dev);
    bmp5_error_codes_print_result("bmp5_set_osr_odr_press_config", rslt);

    return rslt;
}


bool BMP5_Sensor::getSensorData(float* pressure, float* temperature)
{
    // Variable to track errors returned by API calls
    int8_t err = BMP5_OK;

    // Get sensor data
    bmp5_sensor_data data;
    err = bmp5_get_sensor_data(&data, &osr_odr_press_cfg, &dev);
    if(err != BMP5_OK)
    {
        return err;
    }

    // Copy data to output variables
    *pressure = data.pressure;
    *temperature = data.temperature;

    return BMP5_OK;
}