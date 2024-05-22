#include "afi.h"

// #define TESTMODE
/***
SERCOM0: I2C     (PA08, PA09) [sda, scl]
SERCOM1: serial3 (PA17, PA18) [rx, tx]
SERCOM2: serial2 (PA15, PA14) [rx, tx]
SERCOM3: serial4 (PA21, PA20) [rx, tx]
SERCOM4: SPI     (PB10, PB11, PA12, PA13) [ss, miso, mosi, sck]
SERCOM5: serial1 (PB23, PB22) [rx, tx]
  
***/
// interrupt for EXT_SYNC to FPGA
#define PIG_SYNC 29 //PA22
#define EXTT 5 //PA05
// RST to FPGA nConfig
#define nCONFIG 12
//MCU LED
#define MCU_LED A2
/***ADC MUX*/
#define ADCMUX_S1 21
#define ADCMUX_S0 15

/*** global var***/
// int pin_scl_mux = 17;
// bool trig_status[2] = {0, 0};
unsigned int t_new, t_old=0;

unsigned long gps_init_time = 0;
unsigned int gps_date=0, gps_time=0;
bool gps_valid = 0;


my_time_t mcu_time;
my_float_t my_f;
my_misalignment_cali_t misalignment_cali_coe;

unsigned char fog_op_status;

#define WDI 26        //PA27
#define EXT_WDT_EN 28 //PA28

void setup() {

  pinMode(WDI, OUTPUT);
  pinMode(EXT_WDT_EN, OUTPUT);
  // digitalWrite(EXT_WDT_EN, LOW);

  // PORT->Group[0].DIRSET.reg = (uint32_t)(1<<28) ; //PA28, OUTPUT
  // PORT->Group[0].DIRSET.reg = (uint32_t)(1<<27) ; //PA27, OUTPUT
}

void loop() {
  // PORT->Group[0].OUTSET.reg |= 1ul << 27;
  // PORT->Group[0].OUTSET.reg |= 1ul << 28;
  digitalWrite(WDI, HIGH);
  digitalWrite(EXT_WDT_EN, LOW);
  delay(500);
  // PORT->Group[0].OUTCLR.reg |= 1ul << 27;
  // PORT->Group[0].OUTCLR.reg |= 1ul << 28;
  digitalWrite(WDI, LOW);
  digitalWrite(EXT_WDT_EN, HIGH);
  delay(500);
}
