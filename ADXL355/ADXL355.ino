#include <SPI.h>

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

// Operations
const int READ_BYTE = 0x01;
const int WRITE_BYTE = 0x00;

// Pins used for the connection with the sensor
const int CHIP_SELECT_PIN = 10;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  

  // Initalize the  data ready and chip select pins:
  pinMode(CHIP_SELECT_PIN, OUTPUT);

  //Configure ADXL355:
  writeRegister(RST, 0x52);
  delay(100);
//  writeRegister(INTERRUPT, 0);
//  delay(100);
  writeRegister(RANGE, RANGE_8G); // 2G
  delay(100);
  writeRegister(FILTER, 0b101); //ODR 250Hz
  delay(100);
  writeRegister(SYNC, 0b000); 
  delay(100);
  writeRegister(POWER_CTL, 0x06); // Enable measure mode

  // Give the sensor time to set up:
//  delay(100);
//  Serial.print("RANGE: ");
//    Serial.println(readRegistry(RANGE),BIN);
//    delay(100);
//    Serial.print("SYNC: ");
//    Serial.println(readRegistry(SYNC),BIN);
//    delay(100);
//    Serial.print("FILTER: ");
//    Serial.println(readRegistry(FILTER),BIN);
//    delay(1000);
}
unsigned int t_old=0, t_new;
void loop() {
  byte temp1, temp2, temp3;
  int accX, accY, accZ; 
  

//  Serial.print("RANGE: ");
//    Serial.println(readRegistry(RANGE),BIN);
    Serial.print("SYNC: ");
    Serial.println(readRegistry(SYNC),BIN);
    Serial.print("FILTER: ");
    Serial.println(readRegistry(FILTER),BIN);
//    Serial.print("CTL: ");
//    Serial.println(readRegistry(POWER_CTL),BIN);
//    Serial.print("INTERRUPT: ");
//    Serial.println(readRegistry(INTERRUPT),BIN);
//  delay(10);
  if((readRegistry(STATUS)&0x01) == 1)
  {
  	temp1 = readRegistry(XDATA3);
  	temp2 = readRegistry(XDATA2);
  	temp3 = readRegistry(XDATA1);
  	accX = temp1<<12 | temp2<<4 | temp3>>4;
  	if((accX>>19) == 1) accX = accX - 1048576;
  
  	temp1 = readRegistry(YDATA3);
  	temp2 = readRegistry(YDATA2);
  	temp3 = readRegistry(YDATA1);
  	accY = temp1<<12 | temp2<<4 | temp3>>4;
  	if((accY>>19) == 1) accY = accY - 1048576;
  
  	temp1 = readRegistry(ZDATA3);
  	temp2 = readRegistry(ZDATA2);
  	temp3 = readRegistry(ZDATA1);
  	accZ = temp1<<12 | temp2<<4 | temp3>>4;
  	if((accZ>>19) == 1) accZ = accZ - 1048576;
    t_new = millis();
  	Serial.println(t_new - t_old);
//  	Serial.print(", ");
//  	Serial.print((float)accX*SENS_8G);
//  	Serial.print(", ");
//  	Serial.print((float)accY*SENS_8G);
//  	Serial.print(", ");
//  	Serial.println((float)accZ*SENS_8G);
//
//    Serial.print("RANGE: ");
//    Serial.println(readRegistry(RANGE),BIN);
//    Serial.print("SYNC: ");
//    Serial.println(readRegistry(SYNC),BIN);
//    Serial.print("FILTER: ");
//    Serial.println(readRegistry(FILTER),BIN);
    t_old = t_new;
//    Serial.print(micros());
//    Serial.print(", ");
//    Serial.println(readRegistry(FIFO),HEX);
//    delay(100);
  }
}

/* 
 * Write registry in specific device address
 */
void writeRegister(byte thisRegister, byte thisValue) {
  byte dataToSend = (thisRegister << 1) | WRITE_BYTE;
  digitalWrite(CHIP_SELECT_PIN, LOW);
  SPI.transfer(dataToSend);
  SPI.transfer(thisValue);
  digitalWrite(CHIP_SELECT_PIN, HIGH);
}

/* 
 * Read registry in specific device address
 */
unsigned int readRegistry(byte thisRegister) {
  unsigned int result = 0;
  byte dataToSend = (thisRegister << 1) | READ_BYTE;

  digitalWrite(CHIP_SELECT_PIN, LOW);
  SPI.transfer(dataToSend);
  result = SPI.transfer(0x00);
  digitalWrite(CHIP_SELECT_PIN, HIGH);
  return result;
}

/* 
 * Read multiple registries
 */
void readMultipleData(int *addresses, int dataSize, int *readedData) {
  digitalWrite(CHIP_SELECT_PIN, LOW);
  for(int i = 0; i < dataSize; i = i + 1) {
    byte dataToSend = (addresses[i] << 1) | READ_BYTE;
    SPI.transfer(dataToSend);
    readedData[i] = SPI.transfer(0x00);
  }
  digitalWrite(CHIP_SELECT_PIN, HIGH);
}
