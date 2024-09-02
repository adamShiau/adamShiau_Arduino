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
#include <EEPROM_25LC512_SPI.h>
#define CHIP_SELECT_PIN 2
ASM330LHHSensor IMU(&mySPI, CHIP_SELECT_PIN);

// EEPROMM
EEPROM_25LC512_SPI eeprom = EEPROM_25LC512_SPI(mySPI, CHIP_SELECT_PIN);


void mySPI_init(void)
{
    Serial.println("mySPI_init");
    mySPI.begin();
    pinPeripheral(3, PIO_SERCOM_ALT);
    pinPeripheral(22, PIO_SERCOM_ALT);
    pinPeripheral(23, PIO_SERCOM_ALT);
    eeprom.init();
    Serial.println("end og mySPI_init");
}

#endif