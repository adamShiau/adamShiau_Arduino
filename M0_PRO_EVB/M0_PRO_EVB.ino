#include <Arduino.h>
#include "wiring_private.h"
#include "adxl355_SPI.h"
#include "adxl355_I2C.h"
#include "pig_v2.h"

#define SENS_8G 0.0000156
#define SENS_4G 0.0000078
#define SENS_2G 0.0000039
/***
SERCOM0: I2C     (PA08, PA09) [sda, scl]
SERCOM1: serial3 (PA17, PA18) [rx, tx]
SERCOM2: serial2 (PA15, PA14) [rx, tx]
SERCOM3: serial4 (PA21, PA20) [rx, tx]
SERCOM4: SPI     (PB10, PB11, PA12, PA13) [ss, miso, mosi, sck]
SERCOM5: serial1 (PB23, PB22) [rx, tx]
  
***/
// interrupt for EXT_SYNC, EXTT
#define PIG_SYNC 29 //PA22

//
// PWM
#include <SAMD21turboPWM.h>
#define PWM100 5
#define PWM200 7
#define PWM250 11
TurboPWM  pwm;
//
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
#define SPI_CLOCK_1M 1000000
/*** SPIClass SPI (sercom, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_MOSI, PAD_SPI_TX, PAD_SPI_RX);***/
SPIClass mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);

#define CHIP_SELECT_PIN 2
Adxl355_SPI adxl355_spi(mySPI, CHIP_SELECT_PIN);
// Operations
//const int READ_BYTE = 0x01;
//const int WRITE_BYTE = 0x00;

// UART

//SERCOM2: serial2 (PA14, PA15) [tx : D4,  rx: D5]
Uart Serial2 (&sercom2, 25, 24, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM2_Handler()
{
    Serial2.IrqHandler();
}
PIG pig_ser2(Serial2);

//SERCOM1: serial3 (PA16, PA17) [tx : D11, rx: D13]
//Uart Serial3 (&sercom1, 13, 11, SERCOM_RX_PAD_1, UART_TX_PAD_0);
Uart Serial3 (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);
void SERCOM1_Handler()
{
    Serial3.IrqHandler();
}
PIG pig_ser3(Serial3);

//SERCOM3: serial4 (PA20, PA21) [tx : D6, rx: D7]
Uart Serial4 (&sercom3, 10, 9, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM3_Handler()
{
  Serial4.IrqHandler();
}
PIG pig_ser4(Serial4);

unsigned int t_old=0, t_new;
int cnt = 0;

void setup() {
//  int br = 230400;
  int br = 460800;
//int br = 921600;
  // put your setup code here, to run once:
analogWriteResolution(10);
analogReadResolution(12);

// EXTT
//attachInterrupt(26, ISR_EXTT, RISING); // EXTT = PA27, EXTINT[15]
attachInterrupt(30, ISR_EXTT, RISING); // EXTT = PA23, EXTINT[7]
pinMode(PIG_SYNC, OUTPUT); 
digitalWrite(PIG_SYNC, LOW);
//

Serial.begin(br);
Serial1.begin(br);
Serial2.begin(br); 
Serial3.begin(br);
Serial4.begin(br);

pinPeripheral(24, PIO_SERCOM);
pinPeripheral(25, PIO_SERCOM);

pinPeripheral(8, PIO_SERCOM);
pinPeripheral(13, PIO_SERCOM);
//
pinPeripheral(10, PIO_SERCOM_ALT);
pinPeripheral(9, PIO_SERCOM_ALT);


//I2C
myWire.begin();
myWire.setClock(I2C_FAST_MODE_PLUS);
pinPeripheral(27, PIO_SERCOM);
pinPeripheral(20, PIO_SERCOM);
adxl355_i2c.init();

//SPI
mySPI.begin();
mySPI.beginTransaction(SPISettings(SPI_CLOCK_8M, MSBFIRST, SPI_MODE0));
pinPeripheral(3, PIO_SERCOM_ALT);
pinPeripheral(22, PIO_SERCOM_ALT);
pinPeripheral(23, PIO_SERCOM_ALT);
adxl355_spi.init();

/*** pwm ***/

  pwm.setClockDivider(2, false); //48MHz/4 = 12MHz
  pwm.timer(2, 2, 24000, false); //12M/2/24000 = 250Hz
  pwm.timer(1, 2, 30000, false); //12M/2/60000 = 100Hz
  pwm.timer(0, 2, 30000, false); //12M/2/30000 = 200Hz
  
  pwm.analogWrite(PWM100, 500);  
  pwm.analogWrite(PWM200, 500);  
  pwm.analogWrite(PWM250, 500);
}

void loop() {
  int adc1, adc2, adc3;
  byte acc_i2c[9], acc_spi[9], header_ser2[2], header_ser3[2], header_ser4[2];
  byte fog_ser2[14], fog_ser3[14], fog_ser4[14];


  adc1 = analogRead(A4);
  adc2 = analogRead(A5);
  adc3 = analogRead(A3);

  Serial.print(adc1);
  Serial.print(", ");
  Serial.print(adc2);
  Serial.print(", ");
  Serial.println(adc3);
  analogWrite(A0, cnt);


pig_ser2.readData(header_ser2, fog_ser2);
Serial2.write(fog_ser2, 14);
pig_ser3.readData(header_ser3, fog_ser3);
Serial3.write(fog_ser3, 14);
pig_ser4.readData(header_ser4, fog_ser4);
Serial4.write(fog_ser4, 14);
Serial1.write(fog_ser2, 14);
Serial1.write(fog_ser3, 14);
Serial1.write(fog_ser4, 14);

adxl355_i2c.readData(acc_i2c);
adxl355_spi.readData(acc_spi);
//adxl355_spi.printRegAll();

//print_adxl355Data_spi(acc_spi);
//print_adxl355Data_i2c(acc_i2c);

//serialPrint(Serial2, cnt);
//serialPrint(Serial3, cnt);
//serialPrint(Serial4, cnt);
//SPIWriteData(TEST_ADDR, cnt);
//I2CWriteData(TEST_ADDR, 0x45);
//Serial.println(cnt);

cnt++;
//delay(4);
//EIC->CONFIG[1].reg = 0x10000000; //interrupt condition = RISE
digitalWrite(PIG_SYNC, LOW);
EIC->CONFIG[0].reg = 0x10000000; //interrupt condition = RISE
}

void ISR_EXTT()
{
//  EIC->CONFIG[1].reg = 0; //interrupt condition = NONE
//Serial.println("HI");
//  EIC->CONFIG[0].reg = 0; //interrupt condition = NONE
  digitalWrite(PIG_SYNC, HIGH);
//  loopdelay(50); // cannot use delay in ISR
//  digitalWrite(PIG_SYNC, LOW);
  EIC->CONFIG[0].reg = 0; //interrupt condition = NONE
//  EIC->CONFIG[1].reg = 0x100;
}

void loopdelay(int dly)
{
  for(int i=0; i<dly; i++) {
    Serial.println(i);
  }
}

void writeBack(Stream &ser)
{
   byte data[16];
  if (ser.available()>=16)
  {
    Serial.println(ser.available());
    ser.readBytes(data, 16);
    ser.write(data, 16);
//    Serial1.write(data, 16);
  }
  
}

void send_serial_data(Stream &ser, int cnt)
{
  ser.write(0xAB);
  ser.write(0xBA);
  ser.write(cnt>>24);
  ser.write(cnt>>23);
  ser.write(cnt>>22);
  ser.write(cnt>>21);
  ser.write(cnt>>20);
  ser.write(cnt>>19);
  ser.write(cnt>>18);
  ser.write(cnt>>17);
  ser.write(cnt>>24);
  ser.write(cnt>>16);
  ser.write(cnt>>8);
  ser.write(cnt);
  ser.write(0xFE);
  ser.write(0xFF);
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
//  byte dataToSend = (thisRegister << 1) | WRITE_BYTE;
  digitalWrite(CHIP_SELECT_PIN, LOW);
  mySPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  mySPI.transfer(thisValue >> 24);
  mySPI.transfer(thisValue >> 16);
  mySPI.transfer(thisValue >> 8);
  mySPI.transfer(thisValue);
  digitalWrite(CHIP_SELECT_PIN, HIGH);
}
