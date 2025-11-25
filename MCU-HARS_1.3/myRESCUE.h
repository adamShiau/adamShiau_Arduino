#ifndef MYRESCUE_H
#define MYRESCUE_H

/***
 * This library is used to enter the rescue mode in setup() before entering the loop().
 * After powering on, send cmd = [AB BA FF FF FF FF FF FF 55 56] to enter the rescue mode.
 * Refer to the cmd list below to execute the predefined functions.
 * After finishing the predefined funciton, follow the instructions to power cycle or send the escape cmd to leave rescue mode.
 * 
 * 0. Escape rescue mode:
 *      [AB BA FF FF FF FF 02 FF 55 56]
 * 1. Reset system clock to internal clock:
 *      [AB BA FF FF FF FF 03 FF 55 56]
 * 2. Reset auto RST Function:
 *      [AB BA FF FF FF FF 04 FF 55 56]
 * 3. Disable external WDT function:
 *      [AB BA FF FF FF FF 05 FF 55 56]
 ***/

#include "IMU_PIG_DEFINE.h"
#include "EEPROM_MANAGE.h" 
#include "wiring_private.h"
#include "myI2C.h" 
#include "myUART.h"

#define RESCURE_VERSION "28-1"

#define VALID_RESCUE_CMD    0xFF
#define VALID_RESCUE_VAL    0xFFFFFFFF 
#define ESCAPE_CMD          0xFE

#define RESCUR_ENTER            0
#define RESCUR_WAITCMD          1     
#define RESCUR_ESCAPE           2 
#define RESCUR_RESET_SYSCLK     3 
#define RESCUR_RESET_AUTORST    4
#define RESCUR_EXT_WDT          5  

extern int uart_value;  // define in myUART.h
extern byte uart_cmd;   // define in myUART.h

uint8_t sm = RESCUR_ENTER;
bool rescue_flag = false;

// void rescue_mode(byte cmd, int val)

extern void write_fog_parameter_to_eeprom(int& , unsigned int , int );
extern void read_fog_parameter_from_eeprom(int&, unsigned int);

void update_SM()
{
    if(uart_cmd==VALID_RESCUE_CMD) {
        uart_cmd = ESCAPE_CMD;
        sm = (uint8_t)uart_value;
        Serial.print("update_SM: ");
        Serial.println(sm, HEX);
    }
}
void catch_rescue_flag()
{
    Serial.println(uart_cmd, HEX);
    Serial.println(uart_value, HEX);
    if(uart_cmd==VALID_RESCUE_CMD && uart_value==VALID_RESCUE_VAL) {
        uart_cmd = ESCAPE_CMD;
        rescue_flag = true;
    }
}

void rescue_mode()
{
    catch_rescue_flag();
    
    while(rescue_flag)
    {
        switch(sm) 
        {
            case RESCUR_ENTER: {
                // Serial.println("------enter rescue_mode-------");
                // Serial1.println("------enter rescue_mode-------");
                msg_out("\n------enter rescue_mode-------");
                sm = RESCUR_WAITCMD;
                break;
            }  
            case RESCUR_RESET_SYSCLK: {
                msg_out("\nenter RESCUR_RESET_SYSCLK:");
                msg_out("reset sys clock to internal");
                write_fog_parameter_to_eeprom(EEPROM_SYSCLK, EEPROM_ADDR_CLOCK, INTERNAL_CLK);
                msg_out("power cycling to reset the sys clock.");
                sm = RESCUR_WAITCMD;
                break;
            }
            case RESCUR_RESET_AUTORST: {
                msg_out("\nenter RESCUR_RESET_AUTORST:");
                msg_out("clear auto reset flag");
                eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
                eeprom.Parameter_Write(EEPROM_ADDR_SELECT_FN, SEL_DEFAULT);
                eeprom.Parameter_Write(EEPROM_ADDR_OUTPUT_FN, MODE_RST);
                eeprom.Parameter_Write(EEPROM_ADDR_REG_VALUE, 4); // STOP
                msg_out("power cycling or send RESCUR_ESCAPE cmd to exit.");
                sm = RESCUR_WAITCMD;
                break;
            }
            case RESCUR_EXT_WDT: {
                msg_out("\nenter RESCUR_EXT_WDT:");
                msg_out("disable external WDT");
                write_fog_parameter_to_eeprom(EEPROM_EXTWDT, EEPROM_ADDR_EXTWDT, SET_EXTWDT_DISABLE);
                msg_out("power cycling or send RESCUR_ESCAPE cmd to exit.");
                sm = RESCUR_WAITCMD;
                break;
            }
            case RESCUR_WAITCMD: {
                // Serial.println(sm);
                // Serial.println(uart_value);
                update_SM();
                delay(100);
                break;
            }
            case RESCUR_ESCAPE: {
                // Serial.println("------escape rescue_mode-------");
                // Serial1.println("------escape rescue_mode-------");
                msg_out("\n------escape rescue_mode-------");
                rescue_flag = false;
                break;
            }
            default: {
                // Serial.println("SM default"); 
                // delay(500);
                sm = RESCUR_WAITCMD;
                break;
            }
        }
    }
}

#endif