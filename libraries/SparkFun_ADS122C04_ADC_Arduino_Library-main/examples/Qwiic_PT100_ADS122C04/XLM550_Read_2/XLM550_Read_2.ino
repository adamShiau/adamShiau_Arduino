#include <Wire.h>
// #include <Arduino.h>
#include "wiring_private.h"

#include <SparkFun_ADS122C04_ADC_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_ADS122C0

#define I2C_STANDARD_MODE   100000
#define I2C_FAST_MODE       400000
#define I2C_FAST_MODE_PLUS  1000000

#define SFA_X 2.24042E-05
#define SFB_X -80.75709061

#define SFA_Y 2.23992E-05
#define SFB_Y -80.74025289

#define SFA_Z 2.29606E-05
#define SFB_Z -82.76352775

#define SFA_T 4.44701E-05
#define SFB_T -273.15

TwoWire myWire(&sercom0, 27, 20);
void SERCOM0_Handler()
{
  myWire.onService();
}



SFE_ADS122C04 mySensor, mySensor_temp;
uint32_t sum_time=0, i=0;
uint32_t t0;

void setup(void)
{
  Serial.begin(230400);
  while (!Serial)
    ; //Wait for user to open terminal
  Serial.println(F("XLM550 ACCL read Example"));

  // Wire.begin();
  // Wire.setClock(I2C_FAST_MODE);
  // Wire.setClock(720000);
  myWire.begin();
  myWire.setClock(I2C_FAST_MODE);
  pinPeripheral(27, PIO_SERCOM);
  pinPeripheral(20, PIO_SERCOM);

  if (mySensor.begin(0x40, myWire) == false) //SIG: 0x40, TEMP: 0x41
  {
    Serial.println(F("Qwiic PT100 not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  if (mySensor_temp.begin(0x41, myWire) == false) //SIG: 0x40, TEMP: 0x41
  {
    Serial.println(F("Qwiic PT100 not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  // The ADS122C04 will now be configured for raw mode.
  // We can override the ADC mode using these commands:

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

  // mySensor.start();

}

void loop()
{
  float acc[3], temp[3];

  t0 = micros();
  mySensor.start(); // Start the conversion
  mySensor.setInputMultiplexer(ADS122C04_MUX_AIN2_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGX
  // mySensor.read1AData(&accx);
  XLM550_readData_f(acc, temp);
  // read_SIG_int_all_single(acc);
  // read_TEMP_int_all_single(acc2);
  // Serial.print(",");
  // read_SIG_X();

  Serial.println(micros()-t0);
}

void XLM550_readData_f(float acc[3], float temp[3])
{
  uint32_t acc_int[3], temp_int[3];

  mySensor.start(); // Start the SIG conversion
  mySensor_temp.start(); // Start the TEMP conversion
  mySensor.setInputMultiplexer(ADS122C04_MUX_AIN2_AVSS); // SIGX
  mySensor_temp.setInputMultiplexer(ADS122C04_MUX_AIN3_AVSS); // TEMPX
  
  while(!mySensor.checkDataReady()){} //wait SIG data ready
  acc_int[0] = mySensor.readADC(); //read Sig_X data
  mySensor.start(); // Start the SIG conversion
  mySensor.setInputMultiplexer(ADS122C04_MUX_AIN3_AVSS); // SIGY

  while(!mySensor_temp.checkDataReady()){} //wait TEMP data ready
  temp_int[0] = mySensor_temp.readADC();//read Temp_X data
  mySensor_temp.start(); // Start the SIG conversion
  mySensor_temp.setInputMultiplexer(ADS122C04_MUX_AIN0_AVSS); // SIGY

  while(!mySensor.checkDataReady()){} //wait SIG data ready
  acc_int[1] = mySensor.readADC(); //read Sig_Y data
  mySensor.start(); // Start the SIG conversion
  mySensor.setInputMultiplexer(ADS122C04_MUX_AIN0_AVSS); // SIGZ

  while(!mySensor_temp.checkDataReady()){} //wait TEMP data ready
  temp_int[1] = mySensor_temp.readADC();//read Temp_Y data
  mySensor_temp.start(); // Start the SIG conversion
  mySensor_temp.setInputMultiplexer(ADS122C04_MUX_AIN1_AVSS); // SIGZ

  while(!mySensor.checkDataReady()){} //wait SIG data ready
  acc_int[2] = mySensor.readADC(); //read Sig_Z data

  while(!mySensor_temp.checkDataReady()){} //wait TEMP data ready
  temp_int[2] = mySensor_temp.readADC();//read Temp_Z data

  acc[0] = (float)acc_int[0]*SFA_X + SFB_X;
  acc[1] = (float)acc_int[1]*SFA_Y + SFB_Y;
  acc[2] = (float)acc_int[2]*SFA_Z + SFB_Z; 
  temp[0] = (float)temp_int[0]*SFA_T + SFB_T;
  temp[1] = (float)temp_int[1]*SFA_T + SFB_T;
  temp[2] = (float)temp_int[2]*SFA_T + SFB_T;

  // Serial.print(acc_int[0]);
  // Serial.print(", ");
  // Serial.print(acc_int[1]);
  // Serial.print(", ");
  // Serial.print(acc_int[2]);
  // Serial.print(", ");
  // Serial.print(temp_int[0]);
  // Serial.print(", ");
  // Serial.print(temp_int[1]);
  // Serial.print(", ");
  // Serial.print(temp_int[2]);
  // Serial.print(", ");

  Serial.print(acc[0], 5);
  Serial.print(", ");
  Serial.print(acc[1], 5);
  Serial.print(", ");
  Serial.print(acc[2], 5);
  Serial.print(", ");
  Serial.print(temp[0], 5);
  Serial.print(", ");
  Serial.print(temp[1], 5);
  Serial.print(", ");
  Serial.print(temp[2], 5);
  Serial.print(", ");

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
  // mySensor_temp.start(); // Start the conversion
  // mySensor_temp.setInputMultiplexer(ADS122C04_MUX_AIN3_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGX
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
  // mySensor_temp.start(); // Start the conversion
  // mySensor_temp.setInputMultiplexer(ADS122C04_MUX_AIN3_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGX
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

  mySensor.start(); // Start the conversion
  mySensor.setInputMultiplexer(ADS122C04_MUX_AIN2_AVSS); // Route AINP and AINN to AIN0 and AVSS, SIGX
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


