#line 1 "C:\\Users\\user\\Documents\\Arduino\\MCU-GP-28-4-PD\\myCALIBRATION.h"
#ifndef MYCALIBRATION_H
#define MYCALIBRATION_H

#define MAX_STR_LENGTH 20
#define PARAMETER_CNT 24
// #define MAX_TOTAL_LENGTH (MAX_STR_LENGTH * PARAMETER_CNT)

typedef struct {
    char str[MAX_STR_LENGTH];
} DumpParameter;

#endif