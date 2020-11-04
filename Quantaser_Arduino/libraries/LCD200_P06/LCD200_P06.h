#define checkcurrent 65335 //200*3.0518 ~ 396.7uA; 65535-130= 65405
#define OpenVth 100 // the Vmonb should larger then 100*5/65535 ~ 7.62mV
#define VFSHORT 50 // 50*5/1024 ~ 0.22V 
#define RATIO 327.675 // 1mA = 327.675 steps of DAC
#define DACSTEP 980 // increasing 3mA per step
#define DACSTEPTIME 10 // delay 10ms per step
#define DOTPOSITION 11  // display dot position
#define CUR100POSITION 8 // define 100mA cursor position
#define CUR10POSITION 9 //define 10mmA cursor position
#define CUR1POSITION 10 // define 1mA curosr position


#define ADCADDRESS 0x14 // I2C address for LTC2451
#define IOEXPADDRESS 0x20 //I2C address for PCF8574
#define BEEP 2500 // BeeP frequency 2500 Hz
#define BEEPLONG 1000 // Beep during 1000ms
#define BEEPSHORT 200 // Beep during 200ms
#define BEEPSTOP 1000 //Beep duration 1s
#define TURNOFFSTEP 50 // 20ms* 50 steps total turn off time is 1s
#define TURNOFFTIME 20
#define DEBOUNCETIME 5 // debounce time in ms in the interrupt
#define ADDLIM 0 // current limit value address in EERPOM
#define ADDVTH1 1 // forward voltage threshold voltage 1 value address in EERPOM
#define ADDVTH2 2 // forward voltage threshold voltage 2 value address in EEPROM

#define ENC2_A 0 //rotary encoder 2A
#define ENC2_SW 1 //rotary encoeder switch for changing cursor
#define ENC2_B 2 //INT0 for rotory encoder 2B
#define PWR_OFF 3 // Vcc switch
#define LD_EN 4 // on/off the LD current
#define PZT 5 // PD5 for buzzle driver (excnage with LCDSW pin)
#define ENDAC 6  //PD6 for control AD5541
#define VFC1 7 // VFC1 swich
#define LDSW 8 //PB0 for Vcc on/off
#define VFC2 9 // VFC3 switch
#define VFC3 10 // VFC3 switch 
#define VLD A1 // PC1 (ADC1) for LD voltage read
#define V_SENS A2 // V+ voltage detection
#define LCDSW A7 // page switch
