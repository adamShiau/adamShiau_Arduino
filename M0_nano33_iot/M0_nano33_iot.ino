#include <Arduino.h>
#include "wiring_private.h"

// I2C
#include <Wire.h>
#define ADXL355_ADDR     0x1D  //Adxl355 I2C address
#define I2C_STANDARD_MODE   100000
#define I2C_FAST_MODE     400000
#define TEST_ADDR      0xAB
//SCL: PB9, A2
//SDA: PB8, A1

// SPI
#include <SPI.h>
#define CHIP_SELECT_PIN 10
// Operations
const int READ_BYTE = 0x01;
const int WRITE_BYTE = 0x00;
//SS: PA21, D7
//MOSI: PA16, D11 
//MISO: PA19, D12
//SCK: PA17, D13


/***
SERCOM0: serial2 (PA05, PA04)
SERCOM1: serial3 (PA19, PA18)
SERCOM2: serial4 (PA51, PA14)
SERCOM3: SPI     (PA16, PA17, PA20)
SERCOM4: I2C     (PB08, PB09)
SERCOM5: serial1 (PB23, PB22)
  
***/

Uart Serial2 (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0); // rx: PA5, A4; tx: PA4, A3

// Attach the interrupt handler to the SERCOM
void SERCOM0_Handler()
{
    Serial2.IrqHandler();
}

//Uart Serial3 (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2); // rx: PA17, D13; tx: PA18, D10
Uart Serial3 (&sercom1, 12, 8, SERCOM_RX_PAD_3, UART_TX_PAD_2); // rx: PA19, D12; tx: PA18, D10

// Attach the interrupt handler to the SERCOM
void SERCOM1_Handler()
{
    Serial3.IrqHandler();
}

Uart Serial4 (&sercom2, 25, 24, SERCOM_RX_PAD_3, UART_TX_PAD_2);// rx: PA15, D5; tx: PA14, D4
void SERCOM2_Handler()
{
  Serial4.IrqHandler();
}

int cnt = 0;

void setup() {
  // put your setup code here, to run once:

pinPeripheral(24, PIO_SERCOM);
pinPeripheral(25, PIO_SERCOM);

pinPeripheral(5, PIO_SERCOM_ALT);
pinPeripheral(6, PIO_SERCOM_ALT);

pinPeripheral(12, PIO_SERCOM);
pinPeripheral(8, PIO_SERCOM);

Serial.begin(921600);
Serial1.begin(921600);
Serial2.begin(921600); 
Serial3.begin(921600);
Serial4.begin(921600);

//I2C
Wire.begin();
Wire.setClock(I2C_FAST_MODE);
//SPI
pinMode(CHIP_SELECT_PIN, OUTPUT);
digitalWrite(CHIP_SELECT_PIN, HIGH);
//SPI.begin();
//SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
}

void loop() {
  
Serial.println(cnt);

serialPrint(Serial1, cnt);
serialPrint(Serial2, cnt);
serialPrint(Serial3, cnt);
serialPrint(Serial4, cnt);
//I2CWriteData(TEST_ADDR, 0x45);
//SPIWriteData(TEST_ADDR, 0x45);

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
  Serial.println(1);
  Wire.beginTransmission(ADXL355_ADDR);
  Serial.println(2);
  Wire.write(addr);
  Serial.println(3);
  Wire.write(val);
  Serial.println(4);
  Wire.endTransmission();
}

//void SPIWriteData(byte thisRegister, byte thisValue) {
//  byte dataToSend = (thisRegister << 1) | WRITE_BYTE;
//  digitalWrite(CHIP_SELECT_PIN, LOW);
//  SPI.transfer(dataToSend);
//  SPI.transfer(thisValue);
//  digitalWrite(CHIP_SELECT_PIN, HIGH);
//}
