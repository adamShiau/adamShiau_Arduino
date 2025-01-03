#line 1 "C:\\Users\\user\\Documents\\Arduino\\MCU-GP-28-4-PD\\myPWM.h"
#ifndef MYPWM_H
#define MYPWM_H


#include <SAMD21turboPWM.h>
#define PWM100 7
// #define PWM200 5
// #define PWM250 11
// #define PWM_FIX 1

TurboPWM  pwm;
void pwm_init(void)
{
    pwm.setClockDivider(2, false); //48MHz/4 = 12MHz
    // pwm.timer(2, 2, int(24000*PWM_FIX), false); //12M/2/24000 = 250Hz
    pwm.timer(1, 2, int(60000), false); //12M/2/60000 = 100Hz
    // pwm.timer(0, 2, int(30000*PWM_FIX), false); //12M/2/30000 = 200Hz
    
    pwm.analogWrite(PWM100, 500);  
    // pwm.analogWrite(PWM200, 500);  
    // pwm.analogWrite(PWM250, 500);
}

#endif