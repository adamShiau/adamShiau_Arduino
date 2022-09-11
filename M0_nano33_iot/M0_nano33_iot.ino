#include <Arduino.h>
#include "wiring_private.h"

// I2C
#include <Wire.h>
#define ADXL355_ADDR     0x1D  //Adxl355 I2C address
#define I2C_STANDARD_MODE   100000
#define I2C_FAST_MODE     400000
#define I2C_FAST_MODE_PLUS     1000000
//#define I2C_HIGH_SPEED_MODE    3400000 //can not work
#define TEST_ADDR      0xAB
//SCL: PB9, A2
//SDA: PB8, A1

// SPI
#include <SPI.h>
#define SPI_CLOCK_8M 8000000
/*** SPIClass SPI (sercom, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_MOSI, PAD_SPI_TX, PAD_SPI_RX);***/
SPIClass mySPI(&sercom3, 9, 13, 11, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_2);
#define CHIP_SELECT_PIN 10
// Operations
const int READ_BYTE = 0x01;
const int WRITE_BYTE = 0x00;
//SS: PA21, D7
//MOSI: PA16, D11 
//MISO: PA20, D6
//SCK: PA17, D13


/***
SERCOM0: serial2 (PA05, PA04) [rx:A4, tx:A3]
SERCOM1: serial3 (PA19, PA18) [rx:D12, tx:D10]
SERCOM2: serial4 (PA15, PA14) [rx:D5, tx:D4]
SERCOM3: SPI     (PA20, PA17, PA16, PA21) [miso:D6, sck:D13, mosi:D11, ss:D7]
SERCOM4: I2C     (PB08, PB09) [sda:A1, scl:A2]
SERCOM5: serial1 (PB23, PB22) 
  
***/

Uart Serial2 (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0); // rx: PA5, A4; tx: PA4, A3

// Attach the interrupt handler to the SERCOM
void SERCOM0_Handler()
{
    Serial2.IrqHandler();
}

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

GCLK->GENCTRL.reg = //GCLK_GENCTRL_OE |            // Test: enable GCLK output (on a selected pin)
                      GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                      GCLK_GENCTRL_GENEN |         // Enable GCLK0
                      GCLK_GENCTRL_SRC_XOSC32K |   // Set the external 32.768kHz clock source (XOSC32K)
                      GCLK_GENCTRL_ID(0);          // Select GCLK0
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization  
//  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(2) |         // Divide the 48MHz clock source by divisor 24: 48MHz/24=2MHz
//                   GCLK_GENDIV_ID(0);            // Select Generic Clock (GCLK) 0
  
  // Test: enable the GCLK0 output on D2 (PA14)
//  PORT->Group[g_APinDescription[24].ulPort].PINCFG[g_APinDescription[24].ulPin].bit.PMUXEN = 1;
//  PORT->Group[g_APinDescription[24].ulPort].PMUX[g_APinDescription[24].ulPin >> 1].reg |= PORT_PMUX_PMUXE_H;

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
Wire.setClock(I2C_FAST_MODE_PLUS);
//SPI
pinMode(CHIP_SELECT_PIN, OUTPUT);
digitalWrite(CHIP_SELECT_PIN, HIGH);
mySPI.begin();
mySPI.beginTransaction(SPISettings(SPI_CLOCK_8M, MSBFIRST, SPI_MODE0));
pinPeripheral(11, PIO_SERCOM_ALT);
pinPeripheral(13, PIO_SERCOM_ALT);
pinPeripheral(9, PIO_SERCOM_ALT);

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
  Wire.beginTransmission(ADXL355_ADDR);
  Wire.write(addr);
  Wire.write(val);
  Wire.endTransmission();
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
