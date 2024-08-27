#ifndef MYI2C_H
#define MYI2C_H
/** define all of the I2C resource used in AFI
 * 11/23/2023
*/


#include <Wire.h>
#define I2C_STANDARD_MODE   100000
#define I2C_FAST_MODE       400000
#define I2C_FAST_MODE_PLUS  1000000
TwoWire myWire(&sercom0, 27, 20);
void SERCOM0_Handler()
{
  myWire.onService();
}

#include "adxl357_I2C.h"
Adxl357_I2C adxl357_i2c(myWire);

#include <SparkFun_ADS122C04_ADC_Arduino_Library.h> 
SFE_ADS122C04 mySensor, mySensor_temp;

void myI2C_init(void)
{
    myWire.begin();
    myWire.setClock(I2C_FAST_MODE);
    pinPeripheral(27, PIO_SERCOM);
    pinPeripheral(20, PIO_SERCOM);

    /**XLM550 */
    if (mySensor.begin(0x40, myWire) == false) //SIG: 0x40, TEMP: 0x41
    {
      Serial.println(F("Qwiic PT100 not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
    }

    if (mySensor_temp.begin(0x41, Wire) == false) //SIG: 0x40, TEMP: 0x41
    {
      Serial.println(F("Qwiic PT100 not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
    }

    mySensor.setGain(ADS122C04_GAIN_1); // Set the gain to 1
    mySensor.enablePGA(ADS122C04_PGA_DISABLED); // Disable the Programmable Gain Amplifier
    mySensor.setDataRate(ADS122C04_DATA_RATE_1000SPS); // Set the data rate (samples per second) to 20
    mySensor.setOperatingMode(ADS122C04_OP_MODE_TURBO); // Turbo mode
    mySensor.setConversionMode(ADS122C04_CONVERSION_MODE_SINGLE_SHOT);
    mySensor.setVoltageReference(ADS122C04_VREF_INTERNAL); // Use the internal 2.048V reference
    mySensor.enableInternalTempSensor(ADS122C04_TEMP_SENSOR_OFF); // Disable the temperature sensor
    mySensor.setDataCounter(ADS122C04_DCNT_DISABLE); // Disable the data counter (Note: the library does not currently support the data count)
    mySensor.setDataIntegrityCheck(ADS122C04_CRC_DISABLED); // Disable CRC checking (Note: the library does not currently support data integrity checking)
    mySensor.setBurnOutCurrent(ADS122C04_BURN_OUT_CURRENT_OFF); // Disable the burn-out current
    mySensor.setIDACcurrent(ADS122C04_IDAC_CURRENT_OFF); // Disable the IDAC current
    mySensor.setIDAC1mux(ADS122C04_IDAC1_DISABLED); // Disable IDAC1
    mySensor.setIDAC2mux(ADS122C04_IDAC2_DISABLED); // Disable IDAC2
    mySensor.enableDebugging(Serial); //Enable debug messages on Serial
    mySensor.printADS122C04config(); //Print the configuration
    mySensor.disableDebugging(); //Enable debug messages on Serial

    mySensor_temp.setGain(ADS122C04_GAIN_1); // Set the gain to 1
    mySensor_temp.enablePGA(ADS122C04_PGA_DISABLED); // Disable the Programmable Gain Amplifier
    mySensor_temp.setDataRate(ADS122C04_DATA_RATE_1000SPS); // Set the data rate (samples per second) to 20
    mySensor_temp.setOperatingMode(ADS122C04_OP_MODE_TURBO); // Turbo mode
    mySensor_temp.setConversionMode(ADS122C04_CONVERSION_MODE_SINGLE_SHOT);
    mySensor_temp.setVoltageReference(ADS122C04_VREF_INTERNAL); // Use the internal 2.048V reference
    mySensor_temp.enableInternalTempSensor(ADS122C04_TEMP_SENSOR_OFF); // Disable the temperature sensor
    mySensor_temp.setDataCounter(ADS122C04_DCNT_DISABLE); // Disable the data counter (Note: the library does not currently support the data count)
    mySensor_temp.setDataIntegrityCheck(ADS122C04_CRC_DISABLED); // Disable CRC checking (Note: the library does not currently support data integrity checking)
    mySensor_temp.setBurnOutCurrent(ADS122C04_BURN_OUT_CURRENT_OFF); // Disable the burn-out current
    mySensor_temp.setIDACcurrent(ADS122C04_IDAC_CURRENT_OFF); // Disable the IDAC current
    mySensor_temp.setIDAC1mux(ADS122C04_IDAC1_DISABLED); // Disable IDAC1
    mySensor_temp.setIDAC2mux(ADS122C04_IDAC2_DISABLED); // Disable IDAC2
    mySensor_temp.enableDebugging(Serial); //Enable debug messages on Serial
    mySensor_temp.printADS122C04config(); //Print the configuration
    mySensor_temp.disableDebugging(); //Enable debug messages on Serial

    /**ADXL357*/
    // adxl357_i2c.init();
}





#endif