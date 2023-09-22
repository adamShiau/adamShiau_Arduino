
#include <EEPROM_25LC512_SPI.h>
#include "wiring_private.h"

#define EEPROM_START_ADDR 1000 
#define EEPROM_END_ADDR EEPROM_START_ADDR+1700

/***ADC MUX*/
#define ADCMUX_S1 21
#define ADCMUX_S0 15

#define ADC_VIN     18

// SPI
#include <SPI.h>
#define SPI_CLOCK_8M 8000000
#define SPI_CLOCK_1M 1000000
/*** SPIClass SPI (sercom, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_MOSI, PAD_SPI_TX, PAD_SPI_RX);***/
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
 


void setup() {

  byte c[1700], data_r[1700];
  const byte numrw = 1700;
 
  Serial.begin(230400);

  analogReadResolution(12); //set resolution

    /***ADC MUX Setting*/
  pinMode(ADCMUX_S1, OUTPUT);
  pinMode(ADCMUX_S0, OUTPUT);

   //SPI
  mySPI.begin();
  mySPI.beginTransaction(SPISettings(SPI_CLOCK_8M, MSBFIRST, SPI_MODE0));
  pinPeripheral(3, PIO_SERCOM_ALT);
  pinPeripheral(22, PIO_SERCOM_ALT);
  pinPeripheral(23, PIO_SERCOM_ALT);

  eeprom.init();

  for(int i=0; i<numrw; i++) data_w[i] = i;

  Info_Write(EEPROM_START_ADDR, data_w, numrw);

  Info_Read(EEPROM_START_ADDR, data_r, numrw);

  
}

void loop() {
  // unsigned char data1, data2, data3, data4;

  // eeprom.Read(1, &data1);
  // eeprom.Read(2, &data2);
  // eeprom.Read(3, &data3);
  // eeprom.Read(4, &data4);
  // eeprom.Parameter_Read(1,my_f.bin_val);

  // Serial.print(data1);
  // Serial.print(", ");
  // Serial.print(data2);
  // Serial.print(", ");
  // Serial.print(data3);
  // Serial.print(", ");
  // Serial.print(data4);
  // Serial.println("");
  // Serial.println(my_f.int_val);

  for(i=0; i<numrw; i++){
    Serial.print("i: ");
    Serial.print(i);
    Serial.print(", ");
    Serial.println(data_r[i]);
  }
  readVin();

  delay(300);

}

void readVin()
{
  my_f.int_val = analogRead(ADC_VIN);
  Serial.println((float)my_f.int_val/338.205);
}