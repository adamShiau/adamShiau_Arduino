
#include <EEPROM_25LC512_SPI.h>
#include "wiring_private.h"

/**Instruction set*/
// #define EEPROM_25LC512_READ  0b00000011
// #define EEPROM_25LC512_WRITE 0b00000010
// #define EEPROM_25LC512_WREN  0b00000110
// #define EEPROM_25LC512_WRDI  0b00000100
// #define EEPROM_25LC512_RDSR  0b00000101
// #define EEPROM_25LC512_WRSR  0b00000001
// #define EEPROM_25LC512_PE    0b01000010
// #define EEPROM_25LC512_SE    0b11011000
// #define EEPROM_25LC512_CE    0b11000111
// #define EEPROM_25LC512_PDID  0b10101011
// #define EEPROM_25LC512_DPD   0b10111001

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
// my_float_t my_f_r;
my_float_t *my_f_r = (my_float_t *) malloc(4);
 


void setup() {

  // pinMode(CHIP_SELECT_PIN, OUTPUT);
	// digitalWrite(CHIP_SELECT_PIN, HIGH);
  
  Serial.begin(230400);

   //SPI
  mySPI.begin();
  mySPI.beginTransaction(SPISettings(SPI_CLOCK_8M, MSBFIRST, SPI_MODE0));
  pinPeripheral(3, PIO_SERCOM_ALT);
  pinPeripheral(22, PIO_SERCOM_ALT);
  pinPeripheral(23, PIO_SERCOM_ALT);

  eeprom.init();

  // p_SPIWriteData(EEPROM_25LC512_WRITE, 1, 0x01);
  // p_SPIWriteData(EEPROM_25LC512_WRITE, 2, 0x02);
  // p_SPIWriteData(EEPROM_25LC512_WRITE, 3, 0x03);
  // p_SPIWriteData(EEPROM_25LC512_WRITE, 4, 0x04);

  eeprom.Write(0, 0x01);
  eeprom.Write(1, 0x02);
  eeprom.Write(2, 0x03);
  eeprom.Write(3, 0x04);
  eeprom.Parameter_Write(1, 32769);
}

void loop() {
  unsigned char data1, data2, data3, data4;
  // int d1;
  // Serial.println("hi");
  // p_SPIReadData(EEPROM_25LC512_READ, 1);
  // p_SPIReadData(EEPROM_25LC512_READ, 2);
  // p_SPIReadData(EEPROM_25LC512_READ, 3);
  // p_SPIReadData(EEPROM_25LC512_READ, 4);
  eeprom.Read(0, &data1);
  eeprom.Read(1, &data2);
  eeprom.Read(2, &data3);
  eeprom.Read(3, &data4);
  eeprom.Parameter_Read(1,my_f.bin_val);

  Serial.print(data1);
  Serial.print(", ");
  Serial.print(data2);
  Serial.print(", ");
  Serial.print(data3);
  Serial.print(", ");
  Serial.print(data4);
  Serial.println("");
  Serial.println(my_f.int_val);

  delay(300);

}

void p_SPIReadData(unsigned char ins, unsigned int addr)
{
  unsigned char data;
	
	digitalWrite(CHIP_SELECT_PIN, LOW);
	mySPI.transfer(ins);
  mySPI.transfer(addr>>8);
  mySPI.transfer(addr & 0xff);
	// mySPI.transfer(0x00);
  data = mySPI.transfer(0x00);
	digitalWrite(CHIP_SELECT_PIN, HIGH);
	Serial.println(data, HEX);
	// return data;
}

void p_SPIWriteData(unsigned char ins, unsigned int addr, unsigned char data)
{
  writeEN();
	digitalWrite(CHIP_SELECT_PIN, LOW);
	mySPI.transfer(ins);
	// mySPI.transfer(addr>>16);
  mySPI.transfer(addr>>8);
  mySPI.transfer(addr & 0xff);
  mySPI.transfer(data);
	digitalWrite(CHIP_SELECT_PIN, HIGH);
  delay(5);
}

void writeEN()
{
  digitalWrite(CHIP_SELECT_PIN, LOW);
	mySPI.transfer(EEPROM_25LC512_WREN);
	digitalWrite(CHIP_SELECT_PIN, HIGH);
}