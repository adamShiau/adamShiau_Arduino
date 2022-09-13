#include "adxl355_SPI.h"
#include <Arduino.h>
#include "wiring_private.h"


#define SENS_8G 0.0000156
#define SENS_4G 0.0000078
#define SENS_2G 0.0000039


// Memory register addresses:
const int XDATA3 = 0x08;
const int XDATA2 = 0x09;
const int XDATA1 = 0x0A;
const int YDATA3 = 0x0B;
const int YDATA2 = 0x0C;
const int YDATA1 = 0x0D;
const int ZDATA3 = 0x0E;
const int ZDATA2 = 0x0F;
const int ZDATA1 = 0x10;
const int RANGE = 0x2C;
const int POWER_CTL = 0x2D;
const int SYNC = 0x2B;
const int RST = 0x2F;
const int INTERRUPT = 0x2A;

//analog device ID register
const int DEVID_AD = 0x00;

//status register
const int STATUS = 0x04;

//FIFO register
const int FIFO = 0x05;

//filter setting register
const int FILTER = 0x28;

// Device values
const int RANGE_2G = 0x01;
const int RANGE_4G = 0x02;
const int RANGE_8G = 0x03;
const int MEASURE_MODE = 0x06; // Only accelerometer

// SPI
#include <SPI.h>
#define SPI_CLOCK_8M 8000000
#define CHIP_SELECT_PIN 2
/*** SPIClass SPI (sercom, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_MOSI, PAD_SPI_TX, PAD_SPI_RX);***/
SPIClass mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);

Adxl355_SPI adxl355_spi(mySPI, CHIP_SELECT_PIN);
// Operations
//const int READ_BYTE = 0x01;
//const int WRITE_BYTE = 0x00;

// Pins used for the connection with the sensor
//const int CHIP_SELECT_PIN = 10;

void setup() {
//  Serial.begin(115200);
  

  mySPI.begin();
  mySPI.beginTransaction(SPISettings(SPI_CLOCK_8M, MSBFIRST, SPI_MODE0));
  pinPeripheral(3, PIO_SERCOM_ALT);
  pinPeripheral(22, PIO_SERCOM_ALT);
  pinPeripheral(23, PIO_SERCOM_ALT);
  adxl355_spi.init();
}
unsigned int t_old=0, t_new;
void loop() {
  byte acc[9];
  
//  adxl355.printRegAll();
  adxl355_spi.readData(acc);
  print_adxl355Data_spi(acc);
  adxl355_spi.printRegAll();
  delay(4);
}

void print_adxl355Data_spi(byte *temp_a)
{
  int accX, accY, accZ;
  
  accX = temp_a[0]<<12 | temp_a[1]<<4 | temp_a[2]>>4;
  if((accX>>19) == 1) accX = accX - (1<<20);
  accY = temp_a[3]<<12 | temp_a[4]<<4 | temp_a[5]>>4;
  if((accY>>19) == 1) accY = accY - (1<<20);
  accZ = temp_a[6]<<12 | temp_a[7]<<4 | temp_a[8]>>4;
  if((accZ>>19) == 1) accZ = accZ - (1<<20);
  
  t_new = micros();
  Serial.print(t_new - t_old);
  Serial.print('\t');
  Serial.print((float)accX*SENS_8G);
  Serial.print('\t');
  Serial.print((float)accY*SENS_8G);
  Serial.print('\t');
  Serial.println((float)accZ*SENS_8G);
  t_old = t_new;
}

/* 
 * Read multiple registries
 */
//void readMultipleData(int *addresses, int dataSize, int *readedData) {
//  digitalWrite(CHIP_SELECT_PIN, LOW);
//  for(int i = 0; i < dataSize; i = i + 1) {
//    byte dataToSend = (addresses[i] << 1) | READ_BYTE;
//    mySPI.transfer(dataToSend);
//    readedData[i] = mySPI.transfer(0x00);
//  }
//  digitalWrite(CHIP_SELECT_PIN, HIGH);
//}
