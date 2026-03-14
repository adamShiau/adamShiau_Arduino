#include "myI2CSensor.h"



#ifdef ARDUINO_ARDUINO_NANO33BLE
#ifdef __MBED__
#include "mbed_events.h"
#include "mbed_shared_queues.h"
#include "drivers/InterruptIn.h"

static events::EventQueue queue(10 * EVENTS_EVENT_SIZE);
#endif
BoschSensorClass::BoschSensorClass(TwoWire& wire)
{
  _wire = &wire;
  #ifdef TARGET_ARDUINO_NANO33BLE
  BMI270_INT1 = p11;
  #endif
}

void BoschSensorClass::debug(Stream& stream)
{
  _debug = &stream;
}
#ifdef __MBED__
void BoschSensorClass::onInterrupt(mbed::Callback<void(void)> cb)
{
  if (BMI270_INT1 == NC) {
    return;
  }
  static mbed::InterruptIn irq(BMI270_INT1, PullDown);
  static rtos::Thread event_t(osPriorityHigh, 768, nullptr, "events");
  _cb = cb;
  event_t.start(callback(&queue, &events::EventQueue::dispatch_forever));
  irq.rise(mbed::callback(this, &BoschSensorClass::interrupt_handler));
}
#endif
int BoschSensorClass::begin(CfgBoshSensor_t cfg) {
  _wire->begin();

  bmi2.chip_id = BMI2_I2C_PRIM_ADDR;
  bmi2.read = bmi2_i2c_read;
  bmi2.write = bmi2_i2c_write;
  bmi2.delay_us = bmi2_delay_us;
  bmi2.intf = BMI2_I2C_INTF;
  bmi2.intf_ptr = &accel_gyro_dev_info;
  bmi2.read_write_len = 30; // Limitation of the Wire library
  bmi2.config_file_ptr = NULL; // Use the default BMI270 config file

  accel_gyro_dev_info._wire = _wire;
  accel_gyro_dev_info.dev_addr = bmi2.chip_id;

  bmm1.chip_id = BMM150_DEFAULT_I2C_ADDRESS;
  bmm1.read = bmi2_i2c_read;
  bmm1.write = bmi2_i2c_write;
  bmm1.delay_us = bmi2_delay_us;
  bmm1.intf = BMM150_I2C_INTF;
  bmm1.intf_ptr = &mag_dev_info;

  mag_dev_info._wire = _wire;
  mag_dev_info.dev_addr = bmm1.chip_id;

  int8_t result = 0;

  if(cfg != BOSCH_MAGNETOMETER_ONLY) {

    result  |= bmi270_init(&bmi2);
    print_rslt(result);

    result  |= configure_sensor(&bmi2);
    print_rslt(result);
  }

  if(cfg != BOSCH_ACCELEROMETER_ONLY) {

    result |= bmm150_init(&bmm1);
    print_rslt(result);

    result = configure_sensor(&bmm1);
    print_rslt(result);
  }

  _initialized = (result == 0);

  return _initialized;
}

// Accelerometer
int BoschSensorClass::accelerationAvailable() {
  uint16_t status;
  bmi2_get_int_status(&status, &bmi2);
  int ret = ((status | _int_status) & BMI2_ACC_DRDY_INT_MASK);
  _int_status = status;
  _int_status &= ~BMI2_ACC_DRDY_INT_MASK;
  return ret;
}

float BoschSensorClass::accelerationSampleRate() {
  struct bmi2_sens_config sens_cfg;
  sens_cfg.type = BMI2_ACCEL;
  bmi2_get_sensor_config(&sens_cfg, 1, &bmi2);
  return (1 << sens_cfg.cfg.acc.odr) * 0.39;
}

// Gyroscope
int BoschSensorClass::gyroscopeAvailable() {
  uint16_t status;
  bmi2_get_int_status(&status, &bmi2);
  int ret = ((status | _int_status) & BMI2_GYR_DRDY_INT_MASK);
  _int_status = status;
  _int_status &= ~BMI2_GYR_DRDY_INT_MASK;
  return ret;
}

float BoschSensorClass::gyroscopeSampleRate() {
  struct bmi2_sens_config sens_cfg;
  sens_cfg.type = BMI2_GYRO;
  bmi2_get_sensor_config(&sens_cfg, 1, &bmi2);
  return (1 << sens_cfg.cfg.gyr.odr) * 0.39;
}

int BoschSensorClass::getIMUData(float (&omg)[3], float (&acc)[3]) {
  struct bmi2_sens_data sensor_data;
  auto ret = bmi2_get_sensor_data(&sensor_data, &bmi2);
  static const float bias_gyro[3] = {-0.078090, -0.571135, -0.029022};
  static const float bias_accl[3] = {-0.000193, -0.016808, -0.005163};

  omg[0] = sensor_data.gyr.x / INT16_to_DPS - bias_gyro[0];
  omg[1] = sensor_data.gyr.y / INT16_to_DPS - bias_gyro[1];
  omg[2] = sensor_data.gyr.z / INT16_to_DPS - bias_gyro[2];

  acc[0] = sensor_data.acc.x / INT16_to_G * GRAVITY - bias_accl[0];
  acc[1] = sensor_data.acc.y / INT16_to_G * GRAVITY - bias_accl[1];
  acc[2] = sensor_data.acc.z / INT16_to_G * GRAVITY - bias_accl[2];
  return (ret == 0);
}

// Magnetometer
int BoschSensorClass::getMAGData(float (&mag)[3]) {
    if (magneticFieldAvailable()){
        struct bmm150_mag_data mag_data;
        int const rc = bmm150_read_mag_data(&mag_data, &bmm1);
        mag[1] = mag_data.x;
        mag[0] = mag_data.y;
        mag[2] = -mag_data.z;
    
        if (rc == BMM150_OK)
            return 1;
        else
            return 0;
    }
    return 0;
  }

int BoschSensorClass::magneticFieldAvailable() {
  bmm150_get_interrupt_status(&bmm1);
  return bmm1.int_status & BMM150_INT_ASSERTED_DRDY;
}

float BoschSensorClass::magneticFieldSampleRate() {
  struct bmm150_settings settings;
  bmm150_get_sensor_settings(&settings, &bmm1);
  switch (settings.data_rate) {
    case BMM150_DATA_RATE_10HZ:
      return 10;
    case BMM150_DATA_RATE_02HZ:
      return 2;
    case BMM150_DATA_RATE_06HZ:
      return 6;
    case BMM150_DATA_RATE_08HZ:
      return 8;
    case BMM150_DATA_RATE_15HZ:
      return 15;
    case BMM150_DATA_RATE_20HZ:
      return 20;
    case BMM150_DATA_RATE_25HZ:
      return 25;
    case BMM150_DATA_RATE_30HZ:
      return 30;
  }
  return 0;
}

int8_t BoschSensorClass::configure_sensor(struct bmi2_dev *dev)
{
  int8_t rslt;
  uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

  struct bmi2_int_pin_config int_pin_cfg;
  int_pin_cfg.pin_type = BMI2_INT1;
  int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
  int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
  int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
  int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
  int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

  struct bmi2_sens_config sens_cfg[2];
  sens_cfg[0].type = BMI2_ACCEL;
  sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
  sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
  sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
  sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
  sens_cfg[1].type = BMI2_GYRO;
  sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
  sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
  sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_500;
  sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

  rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_sensor_enable(sens_list, 2, dev);
  if (rslt != BMI2_OK)
    return rslt;

  return rslt;
}

int8_t BoschSensorClass::configure_sensor(struct bmm150_dev *dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;
    struct bmm150_settings settings;

    /* Set powermode as normal mode */
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, dev);

    if (rslt == BMM150_OK)
    {
        settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;
        rslt = bmm150_set_presetmode(&settings, dev);

        if (rslt == BMM150_OK)
        {
            /* Map the data interrupt pin */
            settings.int_settings.drdy_pin_en = 0x01;
            //rslt = bmm150_set_sensor_settings(BMM150_SEL_DRDY_PIN_EN, &settings, dev);
        }
    }
    return rslt;
}

int8_t BoschSensorClass::bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  if ((reg_data == NULL) || (len == 0) || (len > 32)) {
    return -1;
  }
  uint8_t bytes_received;

  struct dev_info* dev_info = (struct dev_info*)intf_ptr;
  uint8_t dev_id = dev_info->dev_addr;

  dev_info->_wire->beginTransmission(dev_id);
  dev_info->_wire->write(reg_addr);
  if (dev_info->_wire->endTransmission() == 0) {
    bytes_received = dev_info->_wire->requestFrom(dev_id, len);
    // Optionally, throw an error if bytes_received != len
    for (uint16_t i = 0; i < bytes_received; i++)
    {
      reg_data[i] = dev_info->_wire->read();
    }
  } else {
    return -1;
  }

  return 0;
}

int8_t BoschSensorClass::bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  if ((reg_data == NULL) || (len == 0) || (len > 32)) {
    return -1;
  }

  struct dev_info* dev_info = (struct dev_info*)intf_ptr;
  uint8_t dev_id = dev_info->dev_addr;
  dev_info->_wire->beginTransmission(dev_id);
  dev_info->_wire->write(reg_addr);
  for (uint16_t i = 0; i < len; i++)
  {
    dev_info->_wire->write(reg_data[i]);
  }
  if (dev_info->_wire->endTransmission() != 0) {
    return -1;
  }

  return 0;
}

void BoschSensorClass::bmi2_delay_us(uint32_t period, void *intf_ptr)
{
  delayMicroseconds(period);
}
#ifdef __MBED__
void BoschSensorClass::interrupt_handler()
{
  if (_initialized && _cb) {
    queue.call(_cb);
  }
}
#endif
static void panic_led_trap(void)
{
#if !defined(LED_BUILTIN)
  static int const LED_BUILTIN = 2;
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  while (1)
  {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
}

void BoschSensorClass::print_rslt(int8_t rslt)
{
  if (!_debug) {
    return;
  }
  switch (rslt)
  {
    case BMI2_OK: return; /* Do nothing */ break;
    case BMI2_E_NULL_PTR:
      _debug->println("Error [" + String(rslt) + "] : Null pointer");
      panic_led_trap();
      break;
    case BMI2_E_COM_FAIL:
      _debug->println("Error [" + String(rslt) + "] : Communication failure");
      panic_led_trap();
      break;
    case BMI2_E_DEV_NOT_FOUND:
      _debug->println("Error [" + String(rslt) + "] : Device not found");
      panic_led_trap();
      break;
    case BMI2_E_OUT_OF_RANGE:
      _debug->println("Error [" + String(rslt) + "] : Out of range");
      panic_led_trap();
      break;
    case BMI2_E_ACC_INVALID_CFG:
      _debug->println("Error [" + String(rslt) + "] : Invalid accel configuration");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_INVALID_CFG:
      _debug->println("Error [" + String(rslt) + "] : Invalid gyro configuration");
      panic_led_trap();
      break;
    case BMI2_E_ACC_GYR_INVALID_CFG:
      _debug->println("Error [" + String(rslt) + "] : Invalid accel/gyro configuration");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_SENSOR:
      _debug->println("Error [" + String(rslt) + "] : Invalid sensor");
      panic_led_trap();
      break;
    case BMI2_E_CONFIG_LOAD:
      _debug->println("Error [" + String(rslt) + "] : Configuration loading error");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_PAGE:
      _debug->println("Error [" + String(rslt) + "] : Invalid page ");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_FEAT_BIT:
      _debug->println("Error [" + String(rslt) + "] : Invalid feature bit");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_INT_PIN:
      _debug->println("Error [" + String(rslt) + "] : Invalid interrupt pin");
      panic_led_trap();
      break;
    case BMI2_E_SET_APS_FAIL:
      _debug->println("Error [" + String(rslt) + "] : Setting advanced power mode failed");
      panic_led_trap();
      break;
    case BMI2_E_AUX_INVALID_CFG:
      _debug->println("Error [" + String(rslt) + "] : Invalid auxiliary configuration");
      panic_led_trap();
      break;
    case BMI2_E_AUX_BUSY:
      _debug->println("Error [" + String(rslt) + "] : Auxiliary busy");
      panic_led_trap();
      break;
    case BMI2_E_SELF_TEST_FAIL:
      _debug->println("Error [" + String(rslt) + "] : Self test failed");
      panic_led_trap();
      break;
    case BMI2_E_REMAP_ERROR:
      _debug->println("Error [" + String(rslt) + "] : Remapping error");
      panic_led_trap();
      break;
    case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
      _debug->println("Error [" + String(rslt) + "] : Gyro user gain update failed");
      panic_led_trap();
      break;
    case BMI2_E_SELF_TEST_NOT_DONE:
      _debug->println("Error [" + String(rslt) + "] : Self test not done");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_INPUT:
      _debug->println("Error [" + String(rslt) + "] : Invalid input");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_STATUS:
      _debug->println("Error [" + String(rslt) + "] : Invalid status");
      panic_led_trap();
      break;
    case BMI2_E_CRT_ERROR:
      _debug->println("Error [" + String(rslt) + "] : CRT error");
      panic_led_trap();
      break;
    case BMI2_E_ST_ALREADY_RUNNING:
      _debug->println("Error [" + String(rslt) + "] : Self test already running");
      panic_led_trap();
      break;
    case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
      _debug->println("Error [" + String(rslt) + "] : CRT ready for DL fail abort");
      panic_led_trap();
      break;
    case BMI2_E_DL_ERROR:
      _debug->println("Error [" + String(rslt) + "] : DL error");
      panic_led_trap();
      break;
    case BMI2_E_PRECON_ERROR:
      _debug->println("Error [" + String(rslt) + "] : PRECON error");
      panic_led_trap();
      break;
    case BMI2_E_ABORT_ERROR:
      _debug->println("Error [" + String(rslt) + "] : Abort error");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_SELF_TEST_ERROR:
      _debug->println("Error [" + String(rslt) + "] : Gyro self test error");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_SELF_TEST_TIMEOUT:
      _debug->println("Error [" + String(rslt) + "] : Gyro self test timeout");
      panic_led_trap();
      break;
    case BMI2_E_WRITE_CYCLE_ONGOING:
      _debug->println("Error [" + String(rslt) + "] : Write cycle ongoing");
      panic_led_trap();
      break;
    case BMI2_E_WRITE_CYCLE_TIMEOUT:
      _debug->println("Error [" + String(rslt) + "] : Write cycle timeout");
      panic_led_trap();
      break;
    case BMI2_E_ST_NOT_RUNING:
      _debug->println("Error [" + String(rslt) + "] : Self test not running");
      panic_led_trap();
      break;
    case BMI2_E_DATA_RDY_INT_FAILED:
      _debug->println("Error [" + String(rslt) + "] : Data ready interrupt failed");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_FOC_POSITION:
      _debug->println("Error [" + String(rslt) + "] : Invalid FOC position");
      panic_led_trap();
      break;
    default:
      _debug->println("Error [" + String(rslt) + "] : Unknown error code");
      panic_led_trap();
      break;
  }
}



// Barometer
LPS22HBClass::LPS22HBClass(TwoWire& wire) :
  _wire(&wire),
  _initialized(false)
{}

int LPS22HBClass::begin()
{
  _wire->begin();
  if (i2cRead(LPS22HB_WHO_AM_I_REG) != 0xb1) {
    end();
    return 0;
  }

  _initialized = true;
  return 1;
}

void LPS22HBClass::end()
{
  #if defined(WIRE_HAS_END) && WIRE_HAS_END
  _wire->end();
  #endif
  _initialized = false;
}

bool LPS22HBClass::getBARData(float &bar)
{
  if (_initialized == true) {
    // trigger one shot
    i2cWrite(LPS22HB_CTRL2_REG, 0x01);

    // wait for ONE_SHOT bit to be cleared by the hardware
    if ((i2cRead(LPS22HB_CTRL2_REG) & 0x01) == 0 || counter >= 10) {
        bar = (i2cRead(LPS22HB_PRESS_OUT_XL_REG) |
        (i2cRead(LPS22HB_PRESS_OUT_L_REG) << 8) |
        (i2cRead(LPS22HB_PRESS_OUT_H_REG) << 16)) / 40960.0;
        counter = 0;
        return true;
    }
  }
  counter ++;
  return false;
}

bool LPS22HBClass::getTEMPData(float &temp)
{
  temp = (i2cRead(LPS22HB_TEMP_OUT_L_REG) << 0) | 
          (i2cRead(LPS22HB_TEMP_OUT_H_REG) << 8);
  temp /= 100;
  return true;
}

int LPS22HBClass::i2cRead(uint8_t reg)
{
  _wire->beginTransmission(LPS22HB_ADDRESS);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) {
    return -1;
  }

  if (_wire->requestFrom(LPS22HB_ADDRESS, 1) != 1) {
    return -1;
  }
  
  return _wire->read();
}

int LPS22HBClass::i2cWrite(uint8_t reg, uint8_t val)
{
  _wire->beginTransmission(LPS22HB_ADDRESS);
  _wire->write(reg);
  _wire->write(val);
  if (_wire->endTransmission() != 0) {
    return 0;
  }

  return 1;
}



// Gesture and color sensor 
APDS9960::APDS9960(TwoWire& wire, int intPin) :
  _wire(wire),
  _intPin(intPin),
  _gestureEnabled(false),
  _proximityEnabled(false),
  _colorEnabled(false),
  _gestureIn(false),
  _gestureDirectionX(0),
  _gestureDirectionY(0),
  _gestureDirInX(0),
  _gestureDirInY(0),
  _gestureSensitivity(20),
  _detectedGesture(GESTURE_NONE)
{}

APDS9960::~APDS9960(){}

bool APDS9960::begin() {
  _wire.begin();
    
  // Check ID register
  uint8_t id;
  if (!getID(&id)) return false;
  if (id!=0xAB) return false;
    
  // Disable everything
  if (!setENABLE(0x00)) return false;
  if (!setWTIME(0xFF)) return false;
  if (!setGPULSE(0x8F)) return false; // 16us, 16 pulses // default is: 0x40 = 8us, 1 pulse
  if (!setPPULSE(0x8F)) return false; // 16us, 16 pulses // default is: 0x40 = 8us, 1 pulse
  if (!setGestureIntEnable(true)) return false;
  if (!setGestureMode(true)) return false;
  if (!enablePower()) return false;
  if (!enableWait()) return false;
  // set ADC integration time to 10 ms
  if (!setATIME(256 - (10 / 2.78))) return false;
  // set ADC gain 4x (0x00 => 1x, 0x01 => 4x, 0x02 => 16x, 0x03 => 64x)
  if (!setCONTROL(0x02)) return false;
  delay(10);
  // enable power
  if (!enablePower()) return false;

  if (_intPin > -1) {
    pinMode(_intPin, INPUT);
  }

  return true;
}

void APDS9960::end() {
  // Disable everything
  setENABLE(0x00);

  _gestureEnabled = false;

  _wire.end();
}

// Sets the LED current boost value:
// 0=100%, 1=150%, 2=200%, 3=300%
bool APDS9960::setLEDBoost(uint8_t boost) {
  uint8_t r;
  if (!getCONFIG2(&r)) return false;
  r &= 0b11001111;
  r |= (boost << 4) & 0b00110000;
  return setCONFIG2(r);
}

void APDS9960::setGestureSensitivity(uint8_t sensitivity) {
  if (sensitivity > 100) sensitivity = 100;
  _gestureSensitivity = 100 - sensitivity;
}

void APDS9960::setInterruptPin(int pin) {
  _intPin = pin;
}

bool APDS9960::setGestureIntEnable(bool en) {
    uint8_t r;
    if (!getGCONF4(&r)) return false;
    if (en) {
      r |= 0b00000010;
    } else {
      r &= 0b11111101;
    }
    return setGCONF4(r);
}

bool APDS9960::setGestureMode(bool en)
{
    uint8_t r;
    if (!getGCONF4(&r)) return false;
    if (en) {
      r |= 0b00000001;
    } else {
      r &= 0b11111110;
    }
    return setGCONF4(r);
}

int APDS9960::getInterrputPin(){
  return _intPin;
}

bool APDS9960::enablePower() {
  uint8_t r;
  if (!getENABLE(&r)) return false;
  if ((r & 0b00000001) != 0) return true;
  r |= 0b00000001;
  return setENABLE(r);
}

bool APDS9960::disablePower() {
  uint8_t r;
  if (!getENABLE(&r)) return false;
  if ((r & 0b00000001) == 0) return true;
  r &= 0b11111110;
  return setENABLE(r);
}

bool APDS9960::enableColor() {
  uint8_t r;
  if (!getENABLE(&r)) return false;
  if ((r & 0b00000010) != 0) {
    _colorEnabled = true;
    return true;
  }
  r |= 0b00000010;
  bool res = setENABLE(r);
  _colorEnabled = res;
  return res;
}

bool APDS9960::disableColor() {
  uint8_t r;
  if (!getENABLE(&r)) return false;
  if ((r & 0b00000010) == 0) {
    _colorEnabled = false;
    return true;
  }
  r &= 0b11111101;
  bool res = setENABLE(r);
  _colorEnabled = !res; // (res == true) if successfully disabled
  return res;
}

bool APDS9960::enableProximity() {
  uint8_t r;
  if (!getENABLE(&r)) return false;
  if ((r & 0b00000100) != 0) {
    _proximityEnabled = true;
    return true;
  }
  r |= 0b00000100;
  bool res = setENABLE(r);
  _proximityEnabled = res;
  return res;
}

bool APDS9960::disableProximity() {
  uint8_t r;
  if (!getENABLE(&r)) return false;
  if ((r & 0b00000100) == 0) {
    _proximityEnabled = false;
    return true;
  }
  r &= 0b11111011;
  bool res = setENABLE(r);
  _proximityEnabled = !res; // (res == true) if successfully disabled
  return res;
}

bool APDS9960::enableWait() {
  uint8_t r;
  if (!getENABLE(&r)) return false;
  if ((r & 0b00001000) != 0) return true;
  r |= 0b00001000;
  return setENABLE(r);
}

bool APDS9960::disableWait() {
  uint8_t r;
  if (!getENABLE(&r)) return false;
  if ((r & 0b00001000) == 0) return true;
  r &= 0b11110111;
  return setENABLE(r);
}

bool APDS9960::enableGesture() {
  uint8_t r;
  if (!getENABLE(&r)) return false;
  if ((r & 0b01000000) != 0) {
    _gestureEnabled = true;
    return true;
  }
  r |= 0b01000000;
  bool res = setENABLE(r);
  _gestureEnabled = res;
  return res;
}

bool APDS9960::disableGesture() {
  uint8_t r;
  if (!getENABLE(&r)) return false;
  if ((r & 0b01000000) == 0) {
    _gestureEnabled = false;
    return true;
  }
  r &= 0b10111111;
  bool res = setENABLE(r);
  _gestureEnabled = !res; // (res == true) if successfully disabled
  return res;
}

bool APDS9960::write(uint8_t val) {
  _wire.beginTransmission(APDS9960_ADDR);
  _wire.write(val);
  return _wire.endTransmission() == 0;
}

bool APDS9960::write(uint8_t reg, uint8_t val) {
  _wire.beginTransmission(APDS9960_ADDR);
  _wire.write(reg);
  _wire.write(val);
  return _wire.endTransmission() == 0;
}

bool APDS9960::read(uint8_t reg, uint8_t *val) {
  if (!write(reg)) return false;
  _wire.requestFrom(APDS9960_ADDR, 1);
  if (!_wire.available()) return false;
  *val = _wire.read();
  return true;
}

size_t APDS9960::readBlock(uint8_t reg, uint8_t *val, unsigned int len) {
    size_t i = 0;
    if (!write(reg)) return 0;
    _wire.requestFrom((uint8_t)APDS9960_ADDR, len);
    while (_wire.available()) {
      if (i == len) return 0;
      val[i++] = _wire.read();
    }
    return i;
}

int APDS9960::gestureFIFOAvailable() {
  uint8_t r;
  if (!getGSTATUS(&r)) return -1;
  if ((r & 0x01) == 0x00) return -2;
  if (!getGFLVL(&r)) return -3;
  return r;
}

int APDS9960::handleGesture() {
  const int gestureThreshold = 30;
  while (true) {
    int available = gestureFIFOAvailable();
    if (available <= 0) return 0;

    uint8_t fifo_data[128];
    uint8_t bytes_read = readGFIFO_U(fifo_data, available * 4);
    if (bytes_read == 0) return 0;

    for (int i = 0; i+3 < bytes_read; i+=4) {
      uint8_t u,d,l,r;
      u = fifo_data[i];
      d = fifo_data[i+1];
      l = fifo_data[i+2];
      r = fifo_data[i+3];
      // Serial.print(u);
      // Serial.print(",");
      // Serial.print(d);
      // Serial.print(",");
      // Serial.print(l);
      // Serial.print(",");
      // Serial.println(r);

      if (u<gestureThreshold && d<gestureThreshold && l<gestureThreshold && r<gestureThreshold) {
        _gestureIn = true;
        if (_gestureDirInX != 0 || _gestureDirInY != 0) {
          int totalX = _gestureDirInX - _gestureDirectionX;
          int totalY = _gestureDirInY - _gestureDirectionY;
          // Serial.print("OUT ");
          // Serial.print(totalX);
          // Serial.print(",");
          // Serial.println(totalY);
          if (totalX < -_gestureSensitivity) { _detectedGesture = GESTURE_LEFT; }
          if (totalX > _gestureSensitivity) { _detectedGesture = GESTURE_RIGHT; }
          if (totalY < -_gestureSensitivity) { _detectedGesture = GESTURE_DOWN; }
          if (totalY > _gestureSensitivity) { _detectedGesture = GESTURE_UP; }
          _gestureDirectionX = 0;
          _gestureDirectionY = 0;
          _gestureDirInX = 0;
          _gestureDirInY = 0;
        }
        continue;
      }

      _gestureDirectionX = r - l;
      _gestureDirectionY = u - d;
      if (_gestureIn) {
        _gestureIn = false;
        _gestureDirInX = _gestureDirectionX;
        _gestureDirInY = _gestureDirectionY;
        // Serial.print("IN ");
        // Serial.print(_gestureDirInX);
        // Serial.print(",");
        // Serial.print(_gestureDirInY);
        // Serial.print(" ");
      }
    }
  }
}

int APDS9960::gestureAvailable() {
  if (!_gestureEnabled) enableGesture();

  if (_intPin > -1) {
    if (digitalRead(_intPin) != LOW) {
      return 0;
    }
  } else if (gestureFIFOAvailable() <= 0) {
    return 0;
  }

  handleGesture();
  if (_proximityEnabled) {
    setGestureMode(false);
  }
  return (_detectedGesture == GESTURE_NONE) ? 0 : 1;
}

int APDS9960::readGesture() {
  int gesture = _detectedGesture;
  _detectedGesture = GESTURE_NONE;
  return gesture;
}

int APDS9960::colorAvailable() {
  uint8_t r;
  enableColor();

  if (!getSTATUS(&r)) {
    return 0;
  }

  if (r & 0b00000001) {
    return 1;
  }
  return 0;
}

bool APDS9960::readColor(int& r, int& g, int& b) {
  int c;
  return readColor(r, g, b, c);
}

bool APDS9960::readColor(int& r, int& g, int& b, int& c) {
  uint16_t colors[4];
  if (!readCDATAL((uint8_t *)colors, sizeof(colors))) {
    r = -1;
    g = -1;
    b = -1;
    c = -1;
    return false;
  }

  c = colors[0];
  r = colors[1];
  g = colors[2];
  b = colors[3];
  disableColor();

  return true;
}

int APDS9960::proximityAvailable() {
  uint8_t r;
  enableProximity();

  if (!getSTATUS(&r)) {
    return 0;
  }

  if (r & 0b00000010) {
    return 1;
  }

  return 0;
}

int APDS9960::readProximity() {
  uint8_t r;
  if (!getPDATA(&r)) {
    return -1;
  }

  disableProximity();

  return (255 - r);
}


// sensor

BoschSensorClass sensor(Wire1);
LPS22HBClass baro(Wire1);

#ifdef ENABLE_GUESTURE_SENSOR
APDS9960 gesture(Wire1, PIN_INT_APDS);
#endif

#elif (WIRE_INTERFACES_COUNT > 0)
Nano33IOT_IMU::Nano33IOT_IMU(): AccGyr(&Wire, LSM6DS3_ACC_GYRO_I2C_ADDRESS_LOW){}

bool Nano33IOT_IMU::begin(){
    // connect I2C
    Wire.begin();
    // connect to IMU
    LSM6DS3StatusTypeDef status = AccGyr.begin();
    AccGyr.Set_X_FS(16.0f);
    AccGyr.Enable_X();
    AccGyr.Enable_G();
    return (status == LSM6DS3_STATUS_OK);
}

void Nano33IOT_IMU::getIMUData(float (&omg)[3], float (&acc)[3]){
    int32_t int_acc[3];
    int32_t int_omg[3];
    
    //load gyro and accl data
    AccGyr.Get_X_Axes(int_acc);
    AccGyr.Get_G_Axes(int_omg);
    for (int i=0;i<3;i++){
      acc[i] = float(int_acc[i]) / 1000 * 9.8;
      omg[i] = float(int_omg[i]) / 1000;
    }
}

Nano33IOT_IMU sensor;
#endif
