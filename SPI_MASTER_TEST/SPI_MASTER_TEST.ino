
#include "wiring_private.h"

// SPI
#include <SPI.h>
#define SPI_CLOCK_8M 8000000
#define SPI_CLOCK_1M 1000000
/*** SPIClass SPI (sercom, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_MOSI, PAD_SPI_TX, PAD_SPI_RX);***/
// SPIClass mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
// SPIClassSAMD mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);


#define CHIP_SELECT_PIN 10

void setup() {
  pinMode(CHIP_SELECT_PIN, OUTPUT);
  digitalWrite(CHIP_SELECT_PIN, HIGH);
	Serial.begin(230400); //debug
	Serial1.begin(230400); //to PC

  
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_CLOCK_1M, MSBFIRST, SPI_MODE0));
}

void loop() {
  SPI_WRITE();
  delay(100);
	
}

void SPI_WRITE()
{
  digitalWrite(CHIP_SELECT_PIN, LOW);
  SPI.transfer(0xAB);
  SPI.transfer(0xAA);
  digitalWrite(CHIP_SELECT_PIN, HIGH);
}

