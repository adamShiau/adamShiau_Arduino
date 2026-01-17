#ifndef __MEMORY_MANAGE_H
#define __MEMORY_MANAGE_H

#include <Arduino.h>
#include "command_id.h"

#define PAR_LEN 40 // 定義陣列大小
#define MIS_LEN 30
#define CFG_LEN 6

#define CONTAINER_TO_CMD_OFFSET     8   
#define MIS_CONTAINER_TO_CMD_OFFSET     48  
#define CFG_CONTAINER_TO_CMD_OFFSET     72  

typedef union{
    float	float_val;
    int32_t  int_val;
    uint8_t  bin_val[4];
}data_t;

typedef enum {
    TYPE_INT,   // 0, 代表 data 為整數
    TYPE_FLOAT  // 1, 代表 data 為浮點數
} type_t;

typedef struct
{
    type_t type;
    data_t data;
}mem_unit_t;


typedef struct {    // for parameter container, container size defined by PAR_LEN
    uint8_t sn[13];      // serial number，12 words
    mem_unit_t paramX[PAR_LEN];    
    mem_unit_t paramY[PAR_LEN];    
    mem_unit_t paramZ[PAR_LEN];  
    mem_unit_t misalignment[MIS_LEN];  
    mem_unit_t config[CFG_LEN];
} fog_parameter_t;


/*** float value define*/
#define FLOAT_660 0x44250000






#endif/* __MEMORY_MANAGE_H */