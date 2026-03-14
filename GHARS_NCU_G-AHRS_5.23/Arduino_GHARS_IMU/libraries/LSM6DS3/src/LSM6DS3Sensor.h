
/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LSM6DS3Sensor_H__
#define __LSM6DS3Sensor_H__


/* Includes ------------------------------------------------------------------*/

#include "Wire.h"
#include "SPI.h"
#include "lsm6ds3_reg.h""
#include "Arduino.h"

/* Defines -------------------------------------------------------------------*/

#define LSM6DS3_ACC_SENSITIVITY_FS_2G   0.000061f
#define LSM6DS3_ACC_SENSITIVITY_FS_4G   0.000122f
#define LSM6DS3_ACC_SENSITIVITY_FS_8G   0.000244f
#define LSM6DS3_ACC_SENSITIVITY_FS_16G  0.000488f

#define LSM6DS3_GYRO_SENSITIVITY_FS_125DPS    0.004375f
#define LSM6DS3_GYRO_SENSITIVITY_FS_250DPS    0.008750f
#define LSM6DS3_GYRO_SENSITIVITY_FS_500DPS	0.017500f
#define LSM6DS3_GYRO_SENSITIVITY_FS_1000DPS	0.035000f
#define LSM6DS3_GYRO_SENSITIVITY_FS_2000DPS	0.070000f


/* Typedefs ------------------------------------------------------------------*/

typedef enum
{
  LSM6DS3_OK = 0,
  LSM6DS3_ERROR =-1
} LSM6DS3StatusTypeDef;


/* Class Declaration ---------------------------------------------------------*/
   
/**
 * Abstract class of an LSM6DS3 Inertial Measurement Unit (IMU) 3 axes
 * sensor.
 */
class LSM6DS3Sensor
{
  public:
    LSM6DS3Sensor(TwoWire *i2c, uint8_t address=LSM6DS3_I2C_ADD_H);
    LSM6DS3Sensor(SPIClass *spi, int cs_pin, uint32_t spi_speed=2000000);
    LSM6DS3StatusTypeDef begin();
    LSM6DS3StatusTypeDef end();
    LSM6DS3StatusTypeDef ReadID(uint8_t *Id);
    LSM6DS3StatusTypeDef Enable_X();
    LSM6DS3StatusTypeDef Disable_X();
    LSM6DS3StatusTypeDef Get_X_Sensitivity(float *Sensitivity);
    LSM6DS3StatusTypeDef Get_X_ODR(float *Odr);
    LSM6DS3StatusTypeDef Set_X_ODR(float Odr);
    LSM6DS3StatusTypeDef Get_X_FS(int32_t *FullScale);
    LSM6DS3StatusTypeDef Set_X_FS(int32_t FullScale);
    LSM6DS3StatusTypeDef Get_X_AxesRaw(int16_t *Value);
    LSM6DS3StatusTypeDef Get_X_Axes(int32_t *Acceleration);
    LSM6DS3StatusTypeDef Get_X_DRDY_Status(uint8_t *Status);
	LSM6DS3StatusTypeDef readAcceleration(unsigned char *);
	void print_AccelerationData(unsigned char *, unsigned int, unsigned int&);

    
    LSM6DS3StatusTypeDef Enable_G();
    LSM6DS3StatusTypeDef Disable_G();
    LSM6DS3StatusTypeDef Get_G_Sensitivity(float *Sensitivity);
    LSM6DS3StatusTypeDef Get_G_ODR(float *Odr);
    LSM6DS3StatusTypeDef Set_G_ODR(float Odr);
    LSM6DS3StatusTypeDef Get_G_FS(int32_t *FullScale);
    LSM6DS3StatusTypeDef Set_G_FS(int32_t FullScale);
    LSM6DS3StatusTypeDef Get_G_AxesRaw(int16_t *Value);
    LSM6DS3StatusTypeDef Get_G_Axes(int32_t *AngularRate);
    LSM6DS3StatusTypeDef Get_G_DRDY_Status(uint8_t *Status);
	LSM6DS3StatusTypeDef readGyroscope(unsigned char []);
    
    LSM6DS3StatusTypeDef Read_Reg(uint8_t reg, uint8_t *Data);
    LSM6DS3StatusTypeDef Write_Reg(uint8_t reg, uint8_t Data);
    
    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {        
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr | 0x80);
        /* Read the data */
        for (uint16_t i=0; i<NumByteToRead; i++) {
          *(pBuffer+i) = dev_spi->transfer(0x00);
        }
         
        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;
      }
		
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        dev_i2c->endTransmission(false);

        dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (uint8_t) NumByteToRead);

        int i=0;
        while (dev_i2c->available()) {
          pBuffer[i] = dev_i2c->read();
          i++;
        }

        return 0;
      }

      return 1;
    }
    
    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {  
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr);
        /* Write the data */
        for (uint16_t i=0; i<NumByteToWrite; i++) {
          dev_spi->transfer(pBuffer[i]);
        }

        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;                    
      }
  
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));

        dev_i2c->write(RegisterAddr);
        for (uint16_t i = 0 ; i < NumByteToWrite ; i++) {
          dev_i2c->write(pBuffer[i]);
        }

        dev_i2c->endTransmission(true);

        return 0;
      }

      return 1;
    }

  private:
  
    LSM6DS3StatusTypeDef Set_X_ODR_When_Enabled(float Odr);
    LSM6DS3StatusTypeDef Set_X_ODR_When_Disabled(float Odr);
    LSM6DS3StatusTypeDef Set_G_ODR_When_Enabled(float Odr);
    LSM6DS3StatusTypeDef Set_G_ODR_When_Disabled(float Odr);

    /* Helper classes. */
    TwoWire *dev_i2c;
    SPIClass *dev_spi;
    
    /* Configuration */
    uint8_t address;
    int cs_pin;
    uint32_t spi_speed;
    
    lsm6d3s_odr_xl_t acc_odr;
    lsm6d3s_odr_g_t gyro_odr;

    
    uint8_t acc_is_enabled;
    uint8_t gyro_is_enabled;
       
    lsm6d3s_ctx_t reg_ctx; 
     
};

#ifdef __cplusplus
 extern "C" {
#endif
int32_t LSM6DS3_io_write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
int32_t LSM6DS3_io_read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
#ifdef __cplusplus
  }
#endif

#endif
