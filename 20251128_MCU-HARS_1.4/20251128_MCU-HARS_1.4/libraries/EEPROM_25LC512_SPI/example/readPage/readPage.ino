
#include <EEPROM_25LC512_SPI.h>
#include "wiring_private.h"

#define ADC_VIN     18

#define EEPROM_ADDR_LOG_LEN 128
#define EEPROM_ADDR_LOG1 1024
#define EEPROM_ADDR_LOG2 EEPROM_ADDR_LOG1+EEPROM_ADDR_LOG_LEN


// SPI
#include <SPI.h>
#define SPI_CLOCK_8M 8000000
#define SPI_CLOCK_1M 1000000
/*** SPIClass SPI (sercom, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_MOSI, PAD_SPI_TX, PAD_SPI_RX);***/
// SPIClass mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
SPIClassSAMD mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
#define CHIP_SELECT_PIN 2

// EEPROMM
EEPROM_25LC512_SPI eeprom(mySPI, CHIP_SELECT_PIN);

typedef union
{
  float float_val;
  uint8_t bin_val[4];
  int int_val;
}
my_float_t;

my_float_t my_f;
my_float_t *my_f_r = (my_float_t *) malloc(4);
 
byte data_w1[EEPROM_ADDR_LOG_LEN], data_w2[EEPROM_ADDR_LOG_LEN];
byte data_r[EEPROM_ADDR_LOG_LEN];

void setup() {
  // analogReadResolution(12); //set resolution
  Serial.begin(230400);

   //SPI
  mySPI.begin();
  mySPI.beginTransaction(SPISettings(SPI_CLOCK_8M, MSBFIRST, SPI_MODE0));
  pinPeripheral(3, PIO_SERCOM_ALT);
  pinPeripheral(22, PIO_SERCOM_ALT);
  pinPeripheral(23, PIO_SERCOM_ALT);

  eeprom.init();

  // for(int i=0;i<EEPROM_ADDR_LOG_LEN;i++) {
  //   data_w1[i] = i;
  // }
  // eeprom.Info_Write(EEPROM_ADDR_LOG1, data_w1, EEPROM_ADDR_LOG_LEN);
  // eeprom.Info_Write(EEPROM_ADDR_LOG2, data_w2, EEPROM_ADDR_LOG_LEN);
}

void loop() {

  eeprom.Info_Read(EEPROM_ADDR_LOG1,data_r, EEPROM_ADDR_LOG_LEN);


  for(int i=0; i<EEPROM_ADDR_LOG_LEN; i++) {
    Serial.print(data_r[i]);
    Serial.print(", ");
  }
  Serial.println("");

  delay(1000);

}

void readVin()
{
  my_f.int_val = analogRead(ADC_VIN);
  Serial.println((float)my_f.int_val/338.205);
}