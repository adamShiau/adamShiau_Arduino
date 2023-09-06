#ifndef EEPROM_25LC512_SPI_H
#define EEPROM_25LC512_SPI_H

#include <Arduino.h>
#include <Wire.h>

using namespace std;

class EEPROM_25LC512_SPI {
    
    public:
        EEPROM_25LC512_I2C(int eeprom_addres, TwoWire &);
		EEPROM_25LC512_I2C(TwoWire &);
        void begin();
        void write(unsigned int eeaddress, char* data);
        void read(unsigned int eeaddress, unsigned char* data, unsigned int num_chars);
       
        
		void Parameter_Read(unsigned int eeaddress, unsigned char* buf);
		void Parameter_Write(unsigned int eeaddress, int value);
        void Read(unsigned int eeaddress, unsigned char* buf);
        void Write(unsigned int eeaddress, char data);
		
    private:
		TwoWire &myWire;
        int eeprom = 0x57;
};

#endif