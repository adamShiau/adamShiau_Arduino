#ifndef EEPROM_25LC512_SPI_H
#define EEPROM_25LC512_SPI_H

#include <Arduino.h>
#include <SPI.h>

using namespace std;

/**Instruction set*/
#define EEPROM_25LC512_READ  0b00000011
#define EEPROM_25LC512_WRITE 0b00000010
#define EEPROM_25LC512_WREN  0b00000110
#define EEPROM_25LC512_WRDI  0b00000100
#define EEPROM_25LC512_RDSR  0b00000101
#define EEPROM_25LC512_WRSR  0b00000001
#define EEPROM_25LC512_PE    0b01000010
#define EEPROM_25LC512_SE    0b11011000
#define EEPROM_25LC512_CE    0b11000111
#define EEPROM_25LC512_PDID  0b10101011
#define EEPROM_25LC512_DPD   0b10111001

class EEPROM_25LC512_SPI {
    
    public:
		EEPROM_25LC512_SPI(SPIClass &, char);
    void init();       
    void Read(unsigned int eeaddress, unsigned char* buf);
    void Write(unsigned int eeaddress, char data);
		void Parameter_Read(unsigned int eeaddress, unsigned char* buf);
		void Parameter_Write(unsigned int eeaddress, int value);
   	
    private:
		SPIClass &mySPI;
        void writeEN();
        char _ss;
        int eeprom = 0x57;
};

#endif