#line 1 "C:\\Users\\user\\Documents\\Arduino\\MCU-GP-28-3-PD\\mySPI.h"
#ifndef MYSPI_H
#define MYSPI_H
/** define all of the SPI resource used in GP1Z
 * 11/23/2023
*/
#include <SPI.h>
#define SPI_CLOCK_8M 8000000
#define SPI_CLOCK_1M 1000000
SPIClassSAMD mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);


#include <ASM330LHHSensor.h>
#define CHIP_SELECT_PIN 2
ASM330LHHSensor IMU(&mySPI, CHIP_SELECT_PIN);


void mySPI_init(void)
{
    mySPI.begin();
    pinPeripheral(3, PIO_SERCOM_ALT);
    pinPeripheral(22, PIO_SERCOM_ALT);
    pinPeripheral(23, PIO_SERCOM_ALT);
}

#endif