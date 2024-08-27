/*
  Using the Qwiic PT100
  By: Paul Clark (PaulZC)
  Date: May 5th, 2020

  When the ADS122C04 is initialised by .begin, it is configured for raw mode (which disables the IDAC).
  If you want to manually configure the chip, you can. This example demonstrates how.

  The IDAC current source is disabled, the gain is set to 1 and the internal 2.048V reference is selected.
  The conversion is started manually using .start.
  DRDY is checked manually using .checkDataReady.
  The ADC result is read using .readADC.

  readADC returns a uint32_t. The ADC data is returned in the least-significant 24-bits.

  Hardware Connections:
  Plug a Qwiic cable into the PT100 and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h>

#include <SparkFun_ADS122C04_ADC_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_ADS122C0

#define I2C_STANDARD_MODE   100000
#define I2C_FAST_MODE       400000
#define I2C_FAST_MODE_PLUS  1000000

SFE_ADS122C04 mySensor, mySensor_temp;
uint32_t sum_time=0, i=0;
uint32_t t0;

void setup(void)
{
  Serial.begin(230400);
  while (!Serial)
    ; //Wait for user to open terminal
  Serial.println(F("XLM550 ACCL read Example"));

  Wire.begin();
  // Wire.setClock(I2C_FAST_MODE);
  Wire.setClock(720000);

  if (mySensor.begin(0x40, Wire) == false) //SIG: 0x40, TEMP: 0x41
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

  // The ADS122C04 will now be configured for raw mode.
  // We can override the ADC mode using these commands:

  // mySensor.setInputMultiplexer(ADS122C04_MUX_AIN0_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGZ
  // mySensor.setInputMultiplexer(ADS122C04_MUX_AIN2_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGX
  // mySensor.setInputMultiplexer(ADS122C04_MUX_AIN3_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGY

  // mySensor.setInputMultiplexer(ADS122C04_MUX_AIN0_AVSS); // Route AINP and AINN to AIN0 and AVSS, TEMPY
  // mySensor.setInputMultiplexer(ADS122C04_MUX_AIN1_AVSS); // Route AINP and AINN to AIN0 and AVSS, TEMPZ
  // mySensor.setInputMultiplexer(ADS122C04_MUX_AIN3_AVSS); // Route AINP and AINN to AIN0 and AVSS, TEMPX

  mySensor.setGain(ADS122C04_GAIN_1); // Set the gain to 1
  mySensor.enablePGA(ADS122C04_PGA_DISABLED); // Disable the Programmable Gain Amplifier
  mySensor.setDataRate(ADS122C04_DATA_RATE_1000SPS); // Set the data rate (samples per second) to 20
  mySensor.setOperatingMode(ADS122C04_OP_MODE_TURBO); // Turbo mode
  mySensor.setConversionMode(ADS122C04_CONVERSION_MODE_SINGLE_SHOT);
  // mySensor.setConversionMode(ADS122C04_CONVERSION_MODE_CONTINUOUS);
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

  // mySensor.start();

}

void loop()
{
  /***
   mySensor.start(); // Start the conversion

  unsigned long start_time = millis(); // Record the start time so we can timeout
  boolean drdy = false; // DRDY (1 == new data is ready)

  // Wait for DRDY to go valid (by reading Config Register 2)
  // (You could read the DRDY pin instead, especially if you are using continuous conversion mode.)
  while((drdy == false) && (millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT)))
  {
    delay(5); // Don't pound the I2C bus too hard
    drdy = mySensor.checkDataReady(); // Read DRDY from Config Register 2
  }

  // Check if we timed out
  if (drdy == false)
  {
    Serial.println(F("checkDataReady timed out"));
    return;
  }

  // Read the raw (signed) ADC data
  // The ADC data is returned in the least-significant 24-bits
  uint32_t raw_ADC_data = mySensor.readADC();

  Serial.print(raw_ADC_data);
  Serial.print(", ");
  Serial.println(millis());

  delay(250); //Don't pound the I2C bus too hard
   */
  // Serial.println(read_SIG());

  // i++;
  // sum_time += read_SIG();
  // if(i==256) {
  //   Serial.println(sum_time>>8);
  //   sum_time = 0;
  //   i=0;
  // }
  uint32_t acc[3], acc2[3];
  t0 = millis();
  read_SIG_int_all_single(acc);
  read_TEMP_int_all_single(acc2);
  // Serial.print(",");
  Serial.println(millis()-t0);
}

void read_SIG_int_all_single(uint32_t acc[3])
{
  boolean drdy = false;
  mySensor.start(); // Start the conversion
  mySensor.setInputMultiplexer(ADS122C04_MUX_AIN2_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGX
  while(!drdy) {drdy = mySensor.checkDataReady();} //wait data ready
  // drdy = false;
  acc[0] = mySensor.readADC();
  drdy = false;

  mySensor.start(); // Start the conversion
  mySensor.setInputMultiplexer(ADS122C04_MUX_AIN3_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGY
  while(!drdy) {drdy = mySensor.checkDataReady();} //wait data ready
  // drdy = false;
  acc[1] = mySensor.readADC();
  drdy = false;

  mySensor.start(); // Start the conversion
  mySensor.setInputMultiplexer(ADS122C04_MUX_AIN0_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGZ
  while(!drdy) {drdy = mySensor.checkDataReady();} //wait data ready
  // drdy = false;
  acc[2] = mySensor.readADC();

  Serial.print(acc[0]);
  Serial.print(",");
  Serial.print(acc[1]);
  Serial.print(",");
  Serial.print(acc[2]);
  Serial.print(",");
}

void read_TEMP_int_all_single(uint32_t acc[3])
{
  boolean drdy = false;
  mySensor_temp.start(); // Start the conversion
  mySensor_temp.setInputMultiplexer(ADS122C04_MUX_AIN3_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGX
  while(!drdy) {drdy = mySensor_temp.checkDataReady();} //wait data ready
  // drdy = false;
  acc[0] = mySensor_temp.readADC();
  drdy = false;

  mySensor_temp.start(); // Start the conversion
  mySensor_temp.setInputMultiplexer(ADS122C04_MUX_AIN0_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGY
  while(!drdy) {drdy = mySensor_temp.checkDataReady();} //wait data ready
  // drdy = false;
  acc[1] = mySensor_temp.readADC();
  drdy = false;

  mySensor_temp.start(); // Start the conversion
  mySensor_temp.setInputMultiplexer(ADS122C04_MUX_AIN1_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGZ
  while(!drdy) {drdy = mySensor_temp.checkDataReady();} //wait data ready
  // drdy = false;
  acc[2] = mySensor_temp.readADC();

  Serial.print(acc[0]);
  Serial.print(",");
  Serial.print(acc[1]);
  Serial.print(",");
  Serial.print(acc[2]);
  Serial.print(",");
}

uint32_t read_SIG_X()
{
  boolean drdy = false; // DRDY (1 == new data is ready)
  uint32_t raw_ADC_data;

  mySensor.setInputMultiplexer(ADS122C04_MUX_AIN2_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGX
  mySensor.start(); // Start the conversion
  while(!drdy) {drdy = mySensor.checkDataReady();} //wait data ready
  raw_ADC_data = mySensor.readADC();
  return raw_ADC_data;
}

uint32_t read_SIG_Y()
{
  boolean drdy = false; // DRDY (1 == new data is ready)
  uint32_t raw_ADC_data;

  mySensor.setInputMultiplexer(ADS122C04_MUX_AIN3_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGX
  mySensor.start(); // Start the conversion
  while(!drdy) {drdy = mySensor.checkDataReady();} //wait data ready
  raw_ADC_data = mySensor.readADC();
  return raw_ADC_data;
}


uint32_t read_SIG_Z()
{
  boolean drdy = false; // DRDY (1 == new data is ready)
  uint32_t raw_ADC_data;

  mySensor.setInputMultiplexer(ADS122C04_MUX_AIN0_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGX
  // mySensor.start(); // Start the conversion
  while(!drdy) {drdy = mySensor.checkDataReady();} //wait data ready
  raw_ADC_data = mySensor.readADC();
  return raw_ADC_data;
}


