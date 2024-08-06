#ifndef MYI2C_H
#define MYI2C_H
/** define all of the I2C resource used in GP1Z
 * 11/23/2023
*/


#include <Wire.h>
#define ADXL355_ADDR     0x1D  //Adxl355 I2C address
#define I2C_STANDARD_MODE   100000
#define I2C_FAST_MODE       400000
#define I2C_FAST_MODE_PLUS  1000000
TwoWire myWire(&sercom0, 27, 20);
void SERCOM0_Handler()
{
  myWire.onService();
}

void myI2C_init(void)
{
    myWire.begin();
    myWire.setClock(I2C_FAST_MODE);
    pinPeripheral(27, PIO_SERCOM);
    pinPeripheral(20, PIO_SERCOM);
}

#include <EEPROM_24AA32A_I2C.h>
EEPROM_24AA32A_I2C eeprom = EEPROM_24AA32A_I2C(myWire);

#endif