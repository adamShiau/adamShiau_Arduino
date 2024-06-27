#ifndef MYRESCUE_H
#define MYRESCUE_H

#include "IMU_PIG_DEFINE.h"
#include "EEPROM_MANAGE.h" 
#include "wiring_private.h"
#include "myI2C.h" 

#define VALID_RESCUE_CMD   0xFF
#define VALID_RESCUE_VAL   0xFFFFFFFF 
#define ESCAPE_CMD  0xFE

#define RESCUR_ENTER    0
#define RESCUR_WAITCMD  1  
#define RESCUR_ESCAPE   2 

extern int uart_value;
extern byte uart_cmd;

uint8_t sm = RESCUR_ENTER;
bool rescue_flag = false;

// void rescue_mode(byte cmd, int val)

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
    if(uart_cmd==VALID_RESCUE_CMD & uart_value==VALID_RESCUE_VAL) {
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
                Serial.println("------enter rescue_mode-------");
                Serial1.println("------enter rescue_mode-------");
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
                Serial.println("------escape rescue_mode-------");
                Serial1.println("------escape rescue_mode-------");
                rescue_flag = false;
                break;
            }

            default: break;
        }
    }
}

#endif