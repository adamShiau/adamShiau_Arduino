#line 1 "C:\\Users\\user\\Documents\\Arduino\\MCU-GP-28-3-PD\\myCLK.h"
#ifndef MYCLK_H
#define MYCLK_H

#define INTERNAL_CLK     0
#define EXTERNAL_CLK     1
#define EXTERNAL_CRYSTAL 2

extern void msg_out(char *msg);

void XOSC32K_CLK_SET()
{
  /* Set the correct number of wait states for 48 MHz @ 3.3v */
NVMCTRL->CTRLB.bit.RWS = 1;
SYSCTRL->XOSC32K.reg =
  /* Crystal oscillators can take a long time to startup. This
      waits the maximum amount of time (4 seconds). This can be
      reduced depending on your crystal oscillator. */
  SYSCTRL_XOSC32K_STARTUP(0x5) |
  SYSCTRL_XOSC32K_EN32K;

/* This has to be a separate write as per datasheet section 17.6.3 */
SYSCTRL->XOSC32K.bit.ENABLE = 1;

/* Wait for the external crystal to be ready */
while(!SYSCTRL->PCLKSR.bit.XOSC32KRDY);
/* Configure GCLK1's divider - in this case, no division - so just divide by one */
GCLK->GENDIV.reg =
    GCLK_GENDIV_ID(1) |
    GCLK_GENDIV_DIV(1);

/* Setup GCLK1 using the external 32.768 kHz oscillator */
GCLK->GENCTRL.reg =
GCLK_GENCTRL_ID(1) |
GCLK_GENCTRL_SRC_XOSC32K |
/* Improve the duty cycle. */
GCLK_GENCTRL_IDC |
GCLK_GENCTRL_GENEN;

/* Wait for the write to complete */
while(GCLK->STATUS.bit.SYNCBUSY);

GCLK->CLKCTRL.reg =
    GCLK_CLKCTRL_ID_DFLL48 |
    GCLK_CLKCTRL_GEN_GCLK1 |
    GCLK_CLKCTRL_CLKEN;

/* This works around a quirk in the hardware (errata 1.2.1) -
   the DFLLCTRL register must be manually reset to this value before
   configuration. */
while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
while(!SYSCTRL->PCLKSR.bit.DFLLRDY);

/* Set up the multiplier. This tells the DFLL to multiply the 32.768 kHz
   reference clock to 48 MHz */
SYSCTRL->DFLLMUL.reg =
    /* This value is output frequency / reference clock frequency,
       so 48 MHz / 32.768 kHz = 1465*/
    // SYSCTRL_DFLLMUL_MUL(48000) |
    SYSCTRL_DFLLMUL_MUL(1465) |
    /* The coarse and fine step are used by the DFLL to lock
       on to the target frequency. These are set to half
       of the maximum value. Lower values mean less overshoot,
       whereas higher values typically result in some overshoot but
       faster locking. */
    SYSCTRL_DFLLMUL_FSTEP(511) | // max value: 1023
    SYSCTRL_DFLLMUL_CSTEP(31);  // max value: 63

/* Wait for the write to finish */
while(!SYSCTRL->PCLKSR.bit.DFLLRDY);

uint32_t coarse = (*((uint32_t *)FUSES_DFLL48M_COARSE_CAL_ADDR) & FUSES_DFLL48M_COARSE_CAL_Msk) >> FUSES_DFLL48M_COARSE_CAL_Pos;

SYSCTRL->DFLLVAL.bit.COARSE = coarse;

/* Wait for the write to finish */
while(!SYSCTRL->PCLKSR.bit.DFLLRDY);

SYSCTRL->DFLLCTRL.reg |=
    /* Closed loop mode */
    SYSCTRL_DFLLCTRL_MODE |
    /* Wait for the frequency to be locked before outputting the clock */
    SYSCTRL_DFLLCTRL_WAITLOCK |
    /* Enable it */
    SYSCTRL_DFLLCTRL_ENABLE;

/* Wait for the frequency to lock */
while (!SYSCTRL->PCLKSR.bit.DFLLLCKC || !SYSCTRL->PCLKSR.bit.DFLLLCKF) {}

/* Setup GCLK0 using the DFLL @ 48 MHz */
GCLK->GENCTRL.reg =
    GCLK_GENCTRL_ID(0) |
    GCLK_GENCTRL_SRC_DFLL48M |
    /* Improve the duty cycle. */
    GCLK_GENCTRL_IDC |
    GCLK_GENCTRL_GENEN;

/* Wait for the write to complete */
while(GCLK->STATUS.bit.SYNCBUSY);
}


void XOSC32K_SET()
{
  /* Set the correct number of wait states for 48 MHz @ 3.3v */
NVMCTRL->CTRLB.bit.RWS = 1;
SYSCTRL->XOSC32K.reg =
  /* Crystal oscillators can take a long time to startup. This
      waits the maximum amount of time (4 seconds). This can be
      reduced depending on your crystal oscillator. */
  SYSCTRL_XOSC32K_STARTUP(0x5) |
  SYSCTRL_XOSC32K_EN32K |
  SYSCTRL_XOSC32K_XTALEN;

/* This has to be a separate write as per datasheet section 17.6.3 */
SYSCTRL->XOSC32K.bit.ENABLE = 1;

/* Wait for the external crystal to be ready */
while(!SYSCTRL->PCLKSR.bit.XOSC32KRDY);
/* Configure GCLK1's divider - in this case, no division - so just divide by one */
GCLK->GENDIV.reg =
    GCLK_GENDIV_ID(1) |
    GCLK_GENDIV_DIV(1);

/* Setup GCLK1 using the external 32.768 kHz oscillator */
GCLK->GENCTRL.reg =
GCLK_GENCTRL_ID(1) |
GCLK_GENCTRL_SRC_XOSC32K |
/* Improve the duty cycle. */
GCLK_GENCTRL_IDC |
GCLK_GENCTRL_GENEN;

/* Wait for the write to complete */
while(GCLK->STATUS.bit.SYNCBUSY);

GCLK->CLKCTRL.reg =
    GCLK_CLKCTRL_ID_DFLL48 |
    GCLK_CLKCTRL_GEN_GCLK1 |
    GCLK_CLKCTRL_CLKEN;

/* This works around a quirk in the hardware (errata 1.2.1) -
   the DFLLCTRL register must be manually reset to this value before
   configuration. */
while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
while(!SYSCTRL->PCLKSR.bit.DFLLRDY);

/* Set up the multiplier. This tells the DFLL to multiply the 32.768 kHz
   reference clock to 48 MHz */
SYSCTRL->DFLLMUL.reg =
    /* This value is output frequency / reference clock frequency,
       so 48 MHz / 32.768 kHz = 1465*/
    SYSCTRL_DFLLMUL_MUL(1465) |
    /* The coarse and fine step are used by the DFLL to lock
       on to the target frequency. These are set to half
       of the maximum value. Lower values mean less overshoot,
       whereas higher values typically result in some overshoot but
       faster locking. */
    SYSCTRL_DFLLMUL_FSTEP(511) | // max value: 1023
    SYSCTRL_DFLLMUL_CSTEP(31);  // max value: 63

/* Wait for the write to finish */
while(!SYSCTRL->PCLKSR.bit.DFLLRDY);

uint32_t coarse = (*((uint32_t *)FUSES_DFLL48M_COARSE_CAL_ADDR) & FUSES_DFLL48M_COARSE_CAL_Msk) >> FUSES_DFLL48M_COARSE_CAL_Pos;

SYSCTRL->DFLLVAL.bit.COARSE = coarse;

/* Wait for the write to finish */
while(!SYSCTRL->PCLKSR.bit.DFLLRDY);

SYSCTRL->DFLLCTRL.reg |=
    /* Closed loop mode */
    SYSCTRL_DFLLCTRL_MODE |
    /* Wait for the frequency to be locked before outputting the clock */
    SYSCTRL_DFLLCTRL_WAITLOCK |
    /* Enable it */
    SYSCTRL_DFLLCTRL_ENABLE;

/* Wait for the frequency to lock */
while (!SYSCTRL->PCLKSR.bit.DFLLLCKC || !SYSCTRL->PCLKSR.bit.DFLLLCKF) {}

/* Setup GCLK0 using the DFLL @ 48 MHz */
GCLK->GENCTRL.reg =
    GCLK_GENCTRL_ID(0) |
    GCLK_GENCTRL_SRC_DFLL48M |
    /* Improve the duty cycle. */
    GCLK_GENCTRL_IDC |
    GCLK_GENCTRL_GENEN;

/* Wait for the write to complete */
while(GCLK->STATUS.bit.SYNCBUSY);
}


uint8_t clock_status;

void set_system_clk(uint8_t sel)
{
    if(sel == 0){
        clock_status = 0;
        // Serial.println("Set internal clock");
        // Serial1.println("Set internal clock");
        msg_out("Set internal clock");
    }
    else if(sel == 1){
        clock_status = 1;
        msg_out("Set external clock");
        XOSC32K_CLK_SET();
        // Serial.println("Set external clock");
        // Serial1.println("Set external clock");
    }
    else if(sel == 2){
        clock_status = 2;
        msg_out("Set external crystal");
        XOSC32K_SET();
        // Serial.println("Set external crystal");
        // Serial1.println("Set external crystal");
    } 
    else{
        Serial.print("value:");
        Serial.println(sel);
        msg_out("System clock setting value invalid");
    }
}

void print_clock_configuration()
{
    if(clock_status == 0){
        Serial.println("Set internal clock");
        Serial1.println("Set internal clock");
    }
    else if(clock_status == 1){
        Serial.println("Set external clock");
        Serial1.println("Set external clock");
    }
    else if(clock_status == 2){
        Serial.println("Set external crystal");
        Serial1.println("Set external crystal");
    } 
}

void print_clock_configuration(int sel)
{
    if(sel == 0){
        Serial.println("Set internal clock");
        Serial1.println("Set internal clock");
    }
    else if(sel == 1){
        Serial.println("Set external clock");
        Serial1.println("Set external clock");
    }
    else if(sel == 2){
        Serial.println("Set external crystal");
        Serial1.println("Set external crystal");
    } 
}

#endif