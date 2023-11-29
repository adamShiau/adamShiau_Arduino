#ifndef MYI2C_H
#define MYI2C_H
/** define all of the I2C resource used in AFI
 * 11/23/2023
*/


#include <Wire.h>
#define I2C_STANDARD_MODE   100000
#define I2C_FAST_MODE       400000
#define I2C_FAST_MODE_PLUS  1000000
TwoWire myWire(&sercom0, 27, 20);
void SERCOM0_Handler()
{
  myWire.onService();
}

#include "adxl357_I2C.h"
Adxl357_I2C adxl357_i2c(myWire);

void myI2C_init(void)
{
    myWire.begin();
    myWire.setClock(I2C_FAST_MODE);
    pinPeripheral(27, PIO_SERCOM);
    pinPeripheral(20, PIO_SERCOM);

    /**ADXL357*/
    adxl357_i2c.init();
}

#endif