#include <Arduino.h>
#include "wiring_private.h"


/***
SERCOM0: I2C     (PA08, PA09) [sda: D2, scl: D3]
SERCOM1: serial3 (PA16, PA17) [tx : D11, rx: D13]
SERCOM2: serial2 (PA14, PA15) [tx : D4,  rx: D5]
SERCOM3: serial4 (PA18, PA19) [tx : D10, rx: D12]
SERCOM4: SPI     (PB10, PB11, PA12, PA13) [ss: ICSP4, miso: ICSP3, mosi:, sck:]
SERCOM5: serial1 (PB22, PB23) [tx:, rx:]
  
***/

Uart Serial2 (&sercom2, 25, 24, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM2_Handler()
{
    Serial2.IrqHandler();
}

Uart Serial3 (&sercom1, 13, 11, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM1_Handler()
{
    Serial3.IrqHandler();
}

Uart Serial4 (&sercom3, 12, 8, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM3_Handler()
{
  Serial4.IrqHandler();
}

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

// SPI
#include <SPI.h>
#define SPI_CLOCK_8M 8000000
/*** SPIClass SPI (sercom, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_MOSI, PAD_SPI_TX, PAD_SPI_RX);***/
SPIClass mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
#define CHIP_SELECT_PIN 2
// Operations
const int READ_BYTE = 0x01;
const int WRITE_BYTE = 0x00;


int cnt = 0;

void setup() {
  // put your setup code here, to run once:


pinPeripheral(24, PIO_SERCOM);
pinPeripheral(25, PIO_SERCOM);

pinPeripheral(11, PIO_SERCOM);
pinPeripheral(13, PIO_SERCOM);

pinPeripheral(12, PIO_SERCOM_ALT);
pinPeripheral(8, PIO_SERCOM_ALT);

Serial.begin(921600);
Serial1.begin(921600);
Serial2.begin(921600); 
Serial3.begin(921600);
Serial4.begin(921600);

//I2C
//Wire.begin();
//Wire.setClock(I2C_FAST_MODE_PLUS);
myWire.begin();
myWire.setClock(I2C_FAST_MODE_PLUS);
pinPeripheral(27, PIO_SERCOM);
pinPeripheral(20, PIO_SERCOM);

//SPI
pinMode(CHIP_SELECT_PIN, OUTPUT);
digitalWrite(CHIP_SELECT_PIN, HIGH);
mySPI.begin();
mySPI.beginTransaction(SPISettings(SPI_CLOCK_8M, MSBFIRST, SPI_MODE0));
pinPeripheral(3, PIO_SERCOM_ALT);
pinPeripheral(22, PIO_SERCOM_ALT);
pinPeripheral(23, PIO_SERCOM_ALT);

}

void loop() {
  
Serial.println(cnt);


serialPrint(Serial1, cnt);
serialPrint(Serial2, cnt);
serialPrint(Serial3, cnt);
serialPrint(Serial4, cnt);
SPIWriteData(TEST_ADDR, cnt);
I2CWriteData(TEST_ADDR, 0x45);


cnt++;
delay(4);
}

void serialPrint(Stream &ser, int cnt)
{
  ser.write(cnt>>24);
  ser.write(cnt>>16);
  ser.write(cnt>>8);
  ser.write(cnt);
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
