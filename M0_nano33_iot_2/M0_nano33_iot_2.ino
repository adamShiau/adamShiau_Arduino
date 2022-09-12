#include <Arduino.h>
#include "wiring_private.h"
#include "adxl355_SPI.h"
#include "adxl355_I2C.h"
#include "pig_v2.h"

#define SENS_8G 0.0000156
#define SENS_4G 0.0000078
#define SENS_2G 0.0000039
/***
SERCOM0: I2C     (PA08, PA09) [sda: D2, scl: D3]
SERCOM1: serial3 (PA16, PA17) [tx : D11, rx: D13]
SERCOM2: serial2 (PA14, PA15) [tx : D4,  rx: D5]
SERCOM3: serial4 (PA18, PA19) [tx : D10, rx: D12]
// temp. test//
SERCOM3: serial4 (PA18, PA21) [tx : D10, rx: D7]
SERCOM4: SPI     (PB10, PB11, PA12, PA13) [ss: ICSP4, miso: ICSP3, mosi: ICSP1, sck:]
// temp. test//
SERCOM4: SPI     (PB10, PB11, PA12, PB9) [ss: ICSP4, miso: ICSP3, mosi: ICSP1, sck:A2]
SERCOM5: serial1 (PB22, PB23) [tx:, rx:]
  
***/

Uart Serial2 (&sercom2, 25, 24, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM2_Handler()
{
    Serial2.IrqHandler();
}
PIG pig_ser2(Serial2);

Uart Serial3 (&sercom1, 13, 11, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM1_Handler()
{
    Serial3.IrqHandler();
}
PIG pig_ser3(Serial3);

//Uart Serial4 (&sercom3, 12, 8, SERCOM_RX_PAD_3, UART_TX_PAD_2);
Uart Serial4 (&sercom3, 10, 8, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM3_Handler()
{
  Serial4.IrqHandler();
}
PIG pig_ser4(Serial4);

// I2C
#include <Wire.h>
#define ADXL355_ADDR     0x1D  //Adxl355 I2C address
#define I2C_STANDARD_MODE   100000
#define I2C_FAST_MODE     400000
#define I2C_FAST_MODE_PLUS     1000000
//#define I2C_HIGH_SPEED_MODE    3400000 //can not work
#define TEST_ADDR      0xAB
/*** TwoWire Wire(&sercom, PIN_WIRE_SDA, PIN_WIRE_SCL);***/
TwoWire myWire(&sercom0, 27, 20);
void SERCOM0_Handler()
{
  myWire.onService();
}
Adxl355_I2C adxl355_i2c(myWire);

// SPI
#include <SPI.h>
#define SPI_CLOCK_8M 8000000
/*** SPIClass SPI (sercom, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_MOSI, PAD_SPI_TX, PAD_SPI_RX);***/
//SPIClass mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
// temp. test//
SPIClass mySPI(&sercom4, 3, 19, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
#define CHIP_SELECT_PIN 2
Adxl355_SPI adxl355_spi(mySPI, CHIP_SELECT_PIN);
// Operations
const int READ_BYTE = 0x01;
const int WRITE_BYTE = 0x00;

unsigned int t_old=0, t_new;
int cnt = 0;

void setup() {
  // put your setup code here, to run once:


pinPeripheral(24, PIO_SERCOM);
pinPeripheral(25, PIO_SERCOM);

pinPeripheral(11, PIO_SERCOM);
pinPeripheral(13, PIO_SERCOM);

//pinPeripheral(12, PIO_SERCOM_ALT);
pinPeripheral(10, PIO_SERCOM_ALT);
pinPeripheral(8, PIO_SERCOM_ALT);

Serial.begin(921600);
Serial1.begin(921600);
Serial2.begin(921600); 
Serial3.begin(921600);
Serial4.begin(921600);

//I2C
myWire.begin();
myWire.setClock(I2C_FAST_MODE_PLUS);
pinPeripheral(27, PIO_SERCOM);
pinPeripheral(20, PIO_SERCOM);
adxl355_i2c.init();

//SPI
pinMode(CHIP_SELECT_PIN, OUTPUT);
digitalWrite(CHIP_SELECT_PIN, HIGH);
mySPI.begin();
mySPI.beginTransaction(SPISettings(SPI_CLOCK_8M, MSBFIRST, SPI_MODE0));
pinPeripheral(3, PIO_SERCOM_ALT);
pinPeripheral(22, PIO_SERCOM_ALT);
// temp. test//
pinPeripheral(19, PIO_SERCOM_ALT);
//pinPeripheral(23, PIO_SERCOM_ALT);
adxl355_spi.init();
}

void loop() {
  byte acc_i2c[9], acc_spi[9], header_ser2[2], header_ser3[2], header_ser4[2];
  byte fog_ser2[14], fog_ser3[14], fog_ser4[14];
  
//Serial.println(cnt);
//serialPrint(Serial1, cnt);
//serialPrint(Serial2, cnt);
//serialPrint(Serial3, cnt);
//serialPrint(Serial4, cnt);

pig_ser2.readData(header_ser2, fog_ser2);
pig_ser3.readData(header_ser3, fog_ser3);
pig_ser4.readData(header_ser4, fog_ser4);
//print_ser_ava(Serial2, "Ser2");
//print_ser_ava(Serial3, "Ser3");
//print_ser_ava(Serial4, "Ser4");
adxl355_spi.readData(acc_spi);
adxl355_i2c.readData(acc_i2c);
print_adxl355Data_spi(acc_spi);
print_adxl355Data_i2c(acc_i2c);
//SPIWriteData(TEST_ADDR, cnt);
//I2CWriteData(TEST_ADDR, 0x45);


cnt++;
//delay(4);
}

void print_ser_ava(Stream &ser, char *c)
{
  Serial.print(c);
  Serial.print(": ");
  Serial.println(ser.available());
}

void serialPrint(Stream &ser, int cnt)
{
  ser.write(cnt>>24);
  ser.write(cnt>>16);
  ser.write(cnt>>8);
  ser.write(cnt);
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
  Serial.print("SPI ");
  Serial.print('\t');
  Serial.print(t_new - t_old);
  Serial.print('\t');
  Serial.print((float)accX*SENS_8G);
  Serial.print('\t');
  Serial.print((float)accY*SENS_8G);
  Serial.print('\t');
  Serial.println((float)accZ*SENS_8G);
  t_old = t_new;
}

void print_adxl355Data_i2c(byte *temp_a)
{
  int accX, accY, accZ;
  
  accX = temp_a[0]<<12 | temp_a[1]<<4 | temp_a[2]>>4;
  if((accX>>19) == 1) accX = accX - (1<<20);
  accY = temp_a[3]<<12 | temp_a[4]<<4 | temp_a[5]>>4;
  if((accY>>19) == 1) accY = accY - (1<<20);
  accZ = temp_a[6]<<12 | temp_a[7]<<4 | temp_a[8]>>4;
  if((accZ>>19) == 1) accZ = accZ - (1<<20);
  
  t_new = micros();
  Serial.print("I2C ");
  Serial.print('\t');
  Serial.print(t_new - t_old);
  Serial.print('\t');
  Serial.print((float)accX*SENS_8G);
  Serial.print('\t');
  Serial.print((float)accY*SENS_8G);
  Serial.print('\t');
  Serial.println((float)accZ*SENS_8G);
  t_old = t_new;
}

void I2CWriteData(unsigned char addr, unsigned char val)
{
//  Serial.println(0);
  myWire.beginTransmission(ADXL355_ADDR);
//  Serial.println(1);
  myWire.write(addr);
//  Serial.println(2);
  myWire.write(val);
//  Serial.println(3);
  myWire.endTransmission();
}

void SPIWriteData(byte thisRegister, int thisValue) {
  byte dataToSend = (thisRegister << 1) | WRITE_BYTE;
  digitalWrite(CHIP_SELECT_PIN, LOW);
  mySPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  mySPI.transfer(thisValue >> 24);
  mySPI.transfer(thisValue >> 16);
  mySPI.transfer(thisValue >> 8);
  mySPI.transfer(thisValue);
  digitalWrite(CHIP_SELECT_PIN, HIGH);
//  mySPI.endTransaction();
}
