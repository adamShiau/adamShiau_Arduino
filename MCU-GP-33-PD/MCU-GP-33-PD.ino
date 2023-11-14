#include <ASM330LHHSensor.h>
#include "EEPROM_MANAGE.h"
#include "pig_v2.h"
#include "adxl357_I2C.h"
#include "IMU_PIG_DEFINE.h"
#include "wiring_private.h"
#include "crcCalculator.h"
#include "uartRT.h"
#include <TinyGPSPlus.h>
// #include <EEPROM_24AA32A_I2C.h>
#include <EEPROM_25LC512_SPI.h>

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
// RST to FPGA nConfig
#define nCONFIG 12
//MCU LED
#define MCU_LED A2
/***ADC MUX*/
#define ADCMUX_S1 21
#define ADCMUX_S0 15

//PWM
#include <SAMD21turboPWM.h>
#define PWM100 7
#define PWM200 5
#define PWM250 11
#define PWM_FIX 1
TurboPWM  pwm;


// I2C
#include <Wire.h>
#define ADXL355_ADDR     0x1D  //Adxl355 I2C address
#define I2C_STANDARD_MODE   100000
#define I2C_FAST_MODE     400000
#define I2C_FAST_MODE_PLUS     1000000
//#define I2C_HIGH_SPEED_MODE    3400000 //can not work
#define TEST_ADDR      0xAB
/*** TwoWire Wire(&sercom, PIN_WIRE_SDA, PIN_WIRE_SCL);***/
TwoWire myWire(&sercom0, 27, 20);
void SERCOM0_Handler()
{
  myWire.onService();
}
// Adxl355_I2C adxl355_i2c(myWire); not use in IMU_V4 PCB
Adxl357_I2C adxl357_i2c(myWire);

/*** global var***/
// int pin_scl_mux = 17;
// bool trig_status[2] = {0, 0};
unsigned int t_new, t_old=0;

unsigned long gps_init_time = 0;
unsigned int gps_date=0, gps_time=0;
bool gps_valid = 0;

typedef union
{
  float float_val;
  uint8_t bin_val[4];
  uint32_t ulong_val;
}
my_time_t;

my_time_t mcu_time;

typedef union
{
  float float_val[3];
  uint8_t bin_val[12];
  int int_val[3];
}
my_acc_t;

typedef union
{
  float float_val;
  uint8_t bin_val[4];
  int int_val;
}
my_float_t;

my_float_t my_f;


unsigned char fog_op_status;

// SPI
#include <SPI.h>
#define SPI_CLOCK_8M 8000000
#define SPI_CLOCK_1M 1000000
/*** SPIClass SPI (sercom, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_MOSI, PAD_SPI_TX, PAD_SPI_RX);***/
// SPIClass mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
SPIClassSAMD mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
#define CHIP_SELECT_PIN 2

// EEPROMM
EEPROM_25LC512_SPI eeprom = EEPROM_25LC512_SPI(mySPI, CHIP_SELECT_PIN);

// ASM330LHHClass IMU(mySPI, CHIP_SELECT_PIN, SPI_CLOCK_8M);
ASM330LHHSensor IMU(&mySPI, CHIP_SELECT_PIN);

// cmd read from GUI
uint8_t myCmd_header[] = {0xAB, 0xBA};
uint8_t myCmd_trailer[] = {0x55, 0x56};
uint16_t myCmd_try_cnt;
const uint8_t myCmd_sizeofheader = sizeof(myCmd_header);
const uint8_t myCmd_sizeoftrailer = sizeof(myCmd_trailer);
uartRT myCmd(Serial1, 6);


// UART
//SERCOM2: serial2 (PA14, PA15) [tx,  rx]
Uart Serial2 (&sercom2, 25, 24, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM2_Handler()
{
    Serial2.IrqHandler();
}
// PIG pig_ser2(Serial2);
uint8_t header[] = {0xAB, 0xBA};
uint8_t trailer[] = {0x55};
uint16_t try_cnt;
const uint8_t sizeofheader = sizeof(header);
const uint8_t sizeoftrailer = sizeof(trailer);

//SERCOM1: serial3 (PA17, PA18) [rx, tx]
Uart Serial3 (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);
void SERCOM1_Handler()
{
    Serial3.IrqHandler();
}
// PIG pig_ser3(Serial3);

//SERCOM3: serial4 (PA21, PA20) [rx, tx]
Uart Serial4 (&sercom3, 10, 9, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM3_Handler()
{
  Serial4.IrqHandler();
}
PIG sp13(Serial2, 14); //ch1, z
PIG sp14(Serial3, 14); //ch2, x
PIG sp9(Serial4, 14);  //ch3, y

uartRT SP13_Read(Serial2, 14);
uartRT SP14_Read(Serial3, 14);
uartRT SP9_Read(Serial4, 14);

/** Move Serial1 definition from variant.cpp to here*/
// Uart Serial1( &sercom5, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

// void SERCOM5_Handler()
// {
//   Serial1.IrqHandler();
// }

// bool g_sp9_ready = false, g_sp13_ready = false, g_sp14_ready = false;
byte reg_fog_sp9[14] = {0}, reg_fog_sp13[14] = {0}, reg_fog_sp14[14] = {0};
byte *reg_fog;


/*** serial data from PC***/
byte rx_cnt = 0, cmd, fog_channel;
int value;
bool cmd_complete;

/*** output mode flag***/
byte mux_flag;

/*** acq fog flag***/
bool run_fog_flag;

/*** select running fn flag***/
byte select_fn;
// bool fn_lock;

/*** Control register to control the functionality  of each optput function***/
unsigned int CtrlReg=-1;

/*** KVH HEADER ***/
const unsigned char KVH_HEADER[4] = {0xFE, 0x81, 0xFF, 0x55};
const unsigned char PIG_HEADER[2] = {0xAB, 0xBA};

typedef void (*fn_ptr) (byte &, unsigned int, byte);
fn_ptr output_fn;

// Adxl355 adxl355(pin_scl_mux);
crcCal myCRC;

//SYNC OUT
bool sync_status = 0;

//ISR_FLAG
volatile bool ISR_Coming = false;
volatile bool ISR_PEDGE;

int t_adc = millis();

/*** * Watch dog  * **/
static void   WDTsync() {
  while (WDT->STATUS.bit.SYNCBUSY == 1); //Just wait till WDT is free
}
int WDT_CNT=0;
int tt0=0, tt1, tt2, tt3;
unsigned long data_cnt = 0;

unsigned int MCU_cnt = 0;

// The TinyGPSPlus object
// TinyGPSPlus gps;
unsigned int t_previous = 0;

void setup() {

  /*** pwm ***/

  pwm.setClockDivider(2, false); //48MHz/4 = 12MHz
  pwm.timer(2, 2, int(24000*PWM_FIX), false); //12M/2/24000 = 250Hz
  pwm.timer(1, 2, int(60000*PWM_FIX), false); //12M/2/60000 = 100Hz
  pwm.timer(0, 2, int(30000*PWM_FIX), false); //12M/2/30000 = 200Hz
  
  pwm.analogWrite(PWM100, 500);  
  pwm.analogWrite(PWM200, 500);  
  pwm.analogWrite(PWM250, 500);
  
    // EXTT
    /*** for IMU_V4  : EXTT = PA27, Variant pin = 26, EXINT[15]
     *   for PIG MCU : EXTT = PA27, Variant pin = 26, EXINT[15]
     *  ****/
  attachInterrupt(26, ISR_EXTT, CHANGE);

  disableWDT();

/*** see datasheet p353. 
 *  SENSEn register table:
 * -----------------------------
 *  SENSEn[2:0] |   Name    |   Description   |
 *  ----------------------------------------------------
 *       0x0    |   NONE    | No detection
 *  ----------------------------------------------------
 *       0x1    |   RISE    |  Rising-edge detection
 *  -----------------------------------------------------
 *       0x2    |   FALL    |  Falling-edge detection
 *  -----------------------------------------------------
 *       0x3    |   BOTH    |  Both-edges detection
 *  -----------------------------------------------------
 * ***/
// set interrupt mode to None
  /***----- for PIG MCU & IMU_V4 EXINT[15]----- ***/
  EIC->CONFIG[1].bit.SENSE7 = 0;  // set ISR no NONE



  /*** ADC setting***/
  analogReadResolution(12); //set resolution
  pinMode(ADC_ASE_TACT, INPUT);

  /***ADC MUX Setting*/
  pinMode(ADCMUX_S1, OUTPUT);
  pinMode(ADCMUX_S0, OUTPUT);

  pinMode(PIG_SYNC, OUTPUT); 
  digitalWrite(PIG_SYNC, sync_status);

  pinMode(MCU_LED, OUTPUT);
  digitalWrite(MCU_LED, HIGH);
  
  pinMode(nCONFIG, OUTPUT);
  

  
	Serial.begin(230400); //debug
	Serial1.begin(230400); //to PC
  Serial2.begin(115200); //fog
  Serial3.begin(115200);
  Serial4.begin(115200);

  pinPeripheral(24, PIO_SERCOM);
  pinPeripheral(25, PIO_SERCOM);

  pinPeripheral(8, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);
  //
  pinPeripheral(10, PIO_SERCOM_ALT);
  pinPeripheral(9, PIO_SERCOM_ALT);

  //I2C
  myWire.begin();
  myWire.setClock(I2C_FAST_MODE); 
  pinPeripheral(27, PIO_SERCOM);
  pinPeripheral(20, PIO_SERCOM);

  //SPI
  mySPI.begin();
  mySPI.beginTransaction(SPISettings(SPI_CLOCK_8M, MSBFIRST, SPI_MODE0));
  pinPeripheral(3, PIO_SERCOM_ALT);
  pinPeripheral(22, PIO_SERCOM_ALT);
  pinPeripheral(23, PIO_SERCOM_ALT);

  /**EEPROM*/
  eeprom.init();

  //Re-configure FPGA when system reset on MCU
  digitalWrite(nCONFIG, LOW);
  delay(50);
  digitalWrite(nCONFIG, HIGH);
  delay(500);

  #ifdef GP1Z 
    Wait_FPGA_Wakeup(2);
  #endif

  #ifdef AFI 
    Wait_FPGA_Wakeup(1);
    Wait_FPGA_Wakeup(2);
    Wait_FPGA_Wakeup(3);
  #endif
  Blink_MCU_LED();

  parameter_init();
  Blink_MCU_LED();

  /**ADXL357*/
  adxl357_i2c.init();


	/*** var initialization***/
	cmd_complete = 0;
	mux_flag = MUX_ESCAPE; 		//default set mux_flag to 2
	select_fn = SEL_DEFAULT; 	//default set select_fn to 128
	// select_fn = SEL_IMU;
	run_fog_flag = 0;
	output_fn = temp_idle;


  /***read eeprom current status*/
  eeprom.Read(EEPROM_ADDR_FOG_STATUS, &fog_op_status);

  /***write EEPROM DVT test value*/
  eeprom.Parameter_Write(EEPROM_ADDR_DVT_TEST_1, 0xABAAABAA);
  eeprom.Parameter_Write(EEPROM_ADDR_DVT_TEST_2, 0xFFFF0000);

	


  // tt1 = millis();
  if(fog_op_status==1) // disconnected last time, send cmd again
  {
    Serial.println("AUTO RST");
    eeprom.Parameter_Read(EEPROM_ADDR_SELECT_FN, my_f.bin_val);
    select_fn = my_f.int_val;
    eeprom.Parameter_Read(EEPROM_ADDR_OUTPUT_FN, my_f.bin_val);
    output_fn = (fn_ptr)my_f.int_val; 
    eeprom.Parameter_Read(EEPROM_ADDR_REG_VALUE, my_f.bin_val);
    value = my_f.int_val;
    fog_channel = 2;
    setupWDT(11);
  }


}

void loop() {

	getCmdValue(cmd, value, fog_channel, cmd_complete);
	cmd_mux(cmd_complete, cmd, mux_flag);
	parameter_setting(mux_flag, cmd, value, fog_channel);
	output_mode_setting(mux_flag, cmd, select_fn);
	output_fn(select_fn, value, fog_channel);
  // readAdc();
}

void printAdd(char name[], void* addr)
{
	Serial.print(name);
	Serial.print(": ");
	Serial.println((unsigned int)addr, HEX);
}

void print_adxl355Data(byte *temp_a)
{
	int accX, accY, accZ;
	
	accX = temp_a[0]<<12 | temp_a[1]<<4 | temp_a[2]>>4;
	if((accX>>19) == 1) accX = accX - (1<<20);
	accY = temp_a[3]<<12 | temp_a[4]<<4 | temp_a[5]>>4;
	if((accY>>19) == 1) accY = accY - (1<<20);
	accZ = temp_a[6]<<12 | temp_a[7]<<4 | temp_a[8]>>4;
	if((accZ>>19) == 1) accZ = accZ - (1<<20);
	
	t_new = micros();
	Serial.print(t_new - t_old);
	Serial.print('\t');
	Serial.print((float)accX*ADXL355_8G);
	Serial.print('\t');
	Serial.print((float)accY*ADXL355_8G);
	Serial.print('\t');
	Serial.println((float)accZ*ADXL355_8G);
	t_old = t_new;
}

void printVal_1(char name[], int val)
{
	Serial1.print(name);
	Serial1.print(": ");
	Serial1.println(val);
}

void printVal_0(char name[], int val)
{
	Serial.print(name);
	Serial.print(": ");
	Serial.println(val);
}

void printVal_0(char name[])
{
	Serial.println(name);
}

void getCmdValue(byte &uart_cmd, int &uart_value, byte &fog_ch, bool &uart_complete)
{
  byte *cmd;

    cmd = myCmd.readData(myCmd_header, myCmd_sizeofheader, &myCmd_try_cnt, myCmd_trailer, myCmd_sizeoftrailer);

    if(cmd){
      uart_cmd = cmd[0];
      uart_value = cmd[1]<<24 | cmd[2]<<16 | cmd[3]<<8 | cmd[4];
      fog_ch = cmd[5];
      uart_complete = 1;
      // printVal_0("uart_cmd", uart_cmd);
      // printVal_0("uart_value", uart_value);
      Serial.print("cmd, value, ch: ");
      Serial.print(uart_cmd);
      Serial.print(", ");
      Serial.print(uart_value);
      Serial.print(", ");
      Serial.println(fog_ch);
      // eeprom.Parameter_Write(EEPROM_ADDR_REG_VALUE, uart_value);
    }
}

void cmd_mux(bool &cmd_complete, byte cmd, byte &mux_flag)
{
	if(cmd_complete)
	{
		cmd_complete = 0;
		if(cmd >7) mux_flag = MUX_PARAMETER; 
		else mux_flag = MUX_OUTPUT;
	}
}

void parameter_setting(byte &mux_flag, byte cmd, int value, byte fog_ch) 
{
	if(mux_flag == MUX_PARAMETER)
	{
    PIG *sp;
    Stream *SER;
    eeprom_obj *eeprom_ptr;

    if(fog_ch==1){
      sp = &sp13;
      SER = &Serial2;
      eeprom_ptr = &eeprom_z;
    }
    else if(fog_ch==2){
      sp = &sp14;
      SER = &Serial3;
      eeprom_ptr = &eeprom_x;
    } 
    else if(fog_ch=3){
      sp = &sp9;
      SER = &Serial4;
      eeprom_ptr = &eeprom_y;
    } 

		mux_flag = MUX_ESCAPE;
		switch(cmd) {
      Serial.print("ch: ");
      Serial.println(fog_ch);
      case CMD_FOG_MOD_FREQ: {
        if(value != eeprom_ptr->EEPROM_Mod_freq){
          Serial.println("FOG_MOD_FREQ changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Mod_freq, eeprom_ptr->EEPROM_ADDR_MOD_FREQ, value);
          sp->updateParameter(myCmd_header, MOD_FREQ_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Mod_freq, 0xCC);
        }
        break;}
			case CMD_FOG_MOD_AMP_H: {
       if(value != eeprom_ptr->EEPROM_Amp_H){
          Serial.println("FOG_MOD_AMP_H changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Amp_H, eeprom_ptr->EEPROM_ADDR_MOD_AMP_H, value);
          sp->updateParameter(myCmd_header, MOD_AMP_H_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Amp_H, 0xCC);
        }
        break;}
			case CMD_FOG_MOD_AMP_L: {
        if(value != eeprom_ptr->EEPROM_Amp_L){
          Serial.println("FOG_MOD_AMP_L changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Amp_L, eeprom_ptr->EEPROM_ADDR_MOD_AMP_L, value);
          sp->updateParameter(myCmd_header, MOD_AMP_L_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Amp_L, 0xCC);
        }
        break;}
			case CMD_FOG_ERR_OFFSET: {
        if(value != eeprom_ptr->EEPROM_Err_offset){
          Serial.println("FOG_ERR_OFFSET changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Err_offset, eeprom_ptr->EEPROM_ADDR_ERR_OFFSET, value);
          sp->updateParameter(myCmd_header, ERR_OFFSET_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Err_offset, 0xCC);
        }
        break;}
			case CMD_FOG_POLARITY: {
        if(value != eeprom_ptr->EEPROM_Polarity){
          Serial.println("FOG_POLARITY changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Polarity, eeprom_ptr->EEPROM_ADDR_POLARITY, value);
          sp->updateParameter(myCmd_header, POLARITY_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Polarity, 0xCC);
        }
        break;}
			case CMD_FOG_WAIT_CNT:{
        if(value != eeprom_ptr->EEPROM_Wait_cnt){
          Serial.println("FOG_WAIT_CNT changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Wait_cnt, eeprom_ptr->EEPROM_ADDR_WAIT_CNT, value);
          sp->updateParameter(myCmd_header, WAIT_CNT_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Wait_cnt, 0xCC);
        }
        break;}
			case CMD_FOG_ERR_TH: {
        if(value != eeprom_ptr->EEPROM_Err_th){
          Serial.println("FOG_ERR_TH changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Err_th, eeprom_ptr->EEPROM_ADDR_ERR_TH, value);
          sp->updateParameter(myCmd_header, ERR_TH_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Err_th, 0xCC);
        }
        break;}
			case CMD_FOG_ERR_AVG: {
        if(value != eeprom_ptr->EEPROM_Err_avg){
          Serial.println("FOG_ERR_AVG changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Err_avg, eeprom_ptr->EEPROM_ADDR_ERR_AVG, value);
          sp->updateParameter(myCmd_header, ERR_AVG_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Err_avg, 0xCC);
        }
        break;}
			case CMD_FOG_TIMER_RST: {
        sp->updateParameter(myCmd_header, TIMER_RST_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_GAIN1: {
       if(value != eeprom_ptr->EEPROM_Gain1){
          Serial.println("FOG_GAIN1 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Gain1, eeprom_ptr->EEPROM_ADDR_GAIN1, value);
          sp->updateParameter(myCmd_header, GAIN1_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Gain1, 0xCC);
        }
        break;}
			case CMD_FOG_GAIN2: {
       if(value != eeprom_ptr->EEPROM_Gain2){
          Serial.println("FOG_GAIN2 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Gain2, eeprom_ptr->EEPROM_ADDR_GAIN2, value);
          sp->updateParameter(myCmd_header, GAIN2_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Gain2, 0xCC);
        }
        break;}
			case CMD_FOG_FB_ON: {
       if(value != eeprom_ptr->EEPROM_FB_ON){
          Serial.println("FOG_FB_ON changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_FB_ON, eeprom_ptr->EEPROM_ADDR_FB_ON, value);
          sp->updateParameter(myCmd_header, FB_ON_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_FB_ON, 0xCC);
        }
        break;}
			case CMD_FOG_CONST_STEP: {
        if(value != eeprom_ptr->EEPROM_Const_step){
          Serial.println("FOG_CONST_STEP changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Const_step, eeprom_ptr->EEPROM_ADDR_CONST_STEP, value);
          sp->updateParameter(myCmd_header, CONST_STEP_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Const_step, 0xCC);
        }
        break;}
			case CMD_FOG_FPGA_Q: {
       if(value != eeprom_ptr->EEPROM_Fpga_Q){
          Serial.println("FOG_FPGA_Q changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Fpga_Q, eeprom_ptr->EEPROM_ADDR_FPGA_Q, value);
          sp->updateParameter(myCmd_header, FPGA_Q_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Fpga_Q, 0xCC);
        }
        break;}
			case CMD_FOG_FPGA_R: {
       if(value != eeprom_ptr->EEPROM_Fpga_R){
          Serial.println("FOG_FPGA_R changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Fpga_R, eeprom_ptr->EEPROM_ADDR_FPGA_R, value);
          sp->updateParameter(myCmd_header, FPGA_R_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Fpga_R, 0xCC);
        }
        break;}
			case CMD_FOG_DAC_GAIN: {
       if(value != eeprom_ptr->EEPROM_DAC_gain){
          Serial.println("FOG_DAC_GAIN changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_DAC_gain, eeprom_ptr->EEPROM_ADDR_DAC_GAIN, value);
          sp->updateParameter(myCmd_header, DAC_GAIN_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_DAC_gain, 0xCC);
        }
        break;}
			case CMD_FOG_INT_DELAY: {
        if(value != eeprom_ptr->EEPROM_Data_delay){
          Serial.println("FOG_INT_DELAY changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Data_delay, eeprom_ptr->EEPROM_ADDR_DATA_DELAY, value);
          sp->updateParameter(myCmd_header, DATA_INT_DELAY_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Data_delay, 0xCC);
        }
        break;}
      case CMD_FOG_OUT_START: {
        sp->updateParameter(myCmd_header, DATA_OUT_START_ADDR, myCmd_trailer, value, 0xCC);
        break;}
      
      case CMD_FOG_SF0: {
        if(value != eeprom_ptr->EEPROM_SF0){
          Serial.println("FOG_SF0 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF0, eeprom_ptr->EEPROM_ADDR_SF_0, value);
          sp->updateParameter(myCmd_header, SF0_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF0, 0xCC);
        }
      break;}
      case CMD_FOG_SF1: {
        if(value != eeprom_ptr->EEPROM_SF1){
          Serial.println("FOG_SF1 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF1, eeprom_ptr->EEPROM_ADDR_SF_1, value);
          sp->updateParameter(myCmd_header, SF1_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF1, 0xCC);
        }
      break;}
      case CMD_FOG_SF2: {
        if(value != eeprom_ptr->EEPROM_SF2){
          Serial.println("FOG_SF2 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF2, eeprom_ptr->EEPROM_ADDR_SF_2, value);
          sp->updateParameter(myCmd_header, SF2_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF2, 0xCC);
        }
      break;}
      case CMD_FOG_SF3: {
        if(value != eeprom_ptr->EEPROM_SF3){
          Serial.println("FOG_SF3 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF3, eeprom_ptr->EEPROM_ADDR_SF_3, value);
          sp->updateParameter(myCmd_header, SF3_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF3, 0xCC);
        }
      break;}
      case CMD_FOG_SF4: {
        if(value != eeprom_ptr->EEPROM_SF4){
          Serial.println("FOG_SF4 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF4, eeprom_ptr->EEPROM_ADDR_SF_4, value);
          sp->updateParameter(myCmd_header, SF4_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF4, 0xCC);
        }
      break;}
      case CMD_FOG_SF5: {
        if(value != eeprom_ptr->EEPROM_SF5){
          Serial.println("FOG_SF5 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF5, eeprom_ptr->EEPROM_ADDR_SF_5, value);
          sp->updateParameter(myCmd_header, SF5_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF5, 0xCC);
        }
      break;}
      case CMD_FOG_SF6: {
        if(value != eeprom_ptr->EEPROM_SF6){
          Serial.println("FOG_SF6 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF6, eeprom_ptr->EEPROM_ADDR_SF_6, value);
          sp->updateParameter(myCmd_header, SF6_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF6, 0xCC);
        }
      break;}
      case CMD_FOG_SF7: {
        if(value != eeprom_ptr->EEPROM_SF7){
          Serial.println("FOG_SF7 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF7, eeprom_ptr->EEPROM_ADDR_SF_7, value);
          sp->updateParameter(myCmd_header, SF7_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF7, 0xCC);
        }
      break;}
      case CMD_FOG_SF8: {
        if(value != eeprom_ptr->EEPROM_SF8){
          Serial.println("FOG_SF8 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF8, eeprom_ptr->EEPROM_ADDR_SF_8, value);
          sp->updateParameter(myCmd_header, SF8_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF8, 0xCC);
        }
      break;}
      case CMD_FOG_SF9: {
        if(value != eeprom_ptr->EEPROM_SF9){
          Serial.println("FOG_SF9 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF9, eeprom_ptr->EEPROM_ADDR_SF_9, value);
          sp->updateParameter(myCmd_header, SF9_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF9, 0xCC);
        }
      break;}
      case CMD_FOG_SFB: {
        if(value != eeprom_ptr->EEPROM_SFB){
          Serial.println("FOG_SFB changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SFB, eeprom_ptr->EEPROM_ADDR_SFB, value);
          sp->updateParameter(myCmd_header, SFB_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SFB, 0xCC);
        }
      break;}
      case CMD_FOG_CUTOFF: {
        if(value != eeprom_ptr->EEPROM_CUTOFF){
          Serial.println("FOG_CUTOFF changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_CUTOFF, eeprom_ptr->EEPROM_ADDR_CUTOFF, value);
          sp->updateParameter(myCmd_header, CUTOFF_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_CUTOFF, 0xCC);
        }
      break;}
      case CMD_FOG_TMIN: {
        if(value != eeprom_ptr->EEPROM_TMIN){
          Serial.println("FOG_T_MIN changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_TMIN, eeprom_ptr->EEPROM_ADDR_TMIN, value);
          sp->updateParameter(myCmd_header, TMIN_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_TMIN, 0xCC);
        }
      break;}
      case CMD_FOG_TMAX: {
        // Serial.println(value, HEX);
        // Serial.println(eeprom_ptr->EEPROM_TMAX, HEX);
        if(value != eeprom_ptr->EEPROM_TMAX){
          Serial.println("FOG_T_MAX changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_TMAX, eeprom_ptr->EEPROM_ADDR_TMAX, value);
          sp->updateParameter(myCmd_header, TMAX_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_TMAX, 0xCC);
        }
      break;}

      case CMD_CONFI_BAUDRATE: {
        if(value != EEPROM_BAUDRATE){
          Serial.println("Baudrate changed!");
          write_fog_parameter_to_eeprom(EEPROM_BAUDRATE, EEPROM_ADDR_BAUDRATE, value);
          update_baudrate(EEPROM_BAUDRATE);
        }
      break;}

      case CMD_CONFI_DATARATE: {
        if(value != EEPROM_DATARATE){
          Serial.println("Datarate changed!");
          write_fog_parameter_to_eeprom(EEPROM_DATARATE, EEPROM_ADDR_DATARATE, value);
          update_datarate(EEPROM_DATARATE);
        }
      break;}

      case CMD_FPGA_VERSION: {
        String fpga_version;
        for(int i=0; i<255; i++) SER->read();//clear serial buffer
        sp->updateParameter(myCmd_header, FPGA_VERSION_ADDR, myCmd_trailer, value, 0xCC);
        while(!SER->available());
        if(SER->available()) fpga_version = SER->readStringUntil('\n');
        Serial.print(MCU_VERSION);
        Serial1.print(MCU_VERSION);
        Serial.print(',');
        Serial1.print(',');
        Serial.println(fpga_version);
        Serial1.println(fpga_version);

        break;
      }

      case CMD_DUMP_PARAMETERS: {
        String fog_parameter;
        for(int i=0; i<255; i++) SER->read();//clear serial buffer
        sp->updateParameter(myCmd_header, FPGA_DUMP_PARAMETERS_ADDR, myCmd_trailer, value, 0xCC);
        while(!SER->available()){delay(1);};
        if(SER->available())
         {
          fog_parameter = SER->readStringUntil('\n');
          Serial.println(fog_parameter);
          Serial1.println(fog_parameter);
         }  

        break;
      }
			default: break;
		}
    
	}
}

void output_mode_setting(byte &mux_flag, byte mode, byte &select_fn)
{
	if(mux_flag == MUX_OUTPUT)
	{
		mux_flag = MUX_ESCAPE;
		switch(mode) {
			case MODE_RST: {
				output_fn = fn_rst;
				select_fn = SEL_RST;
				break;
			}
			case MODE_FOG: {
				output_fn = acq_fog;
				select_fn = SEL_FOG_1;
				break;
			}
			case MODE_IMU: {
				output_fn = acq_imu; 
				select_fn = SEL_IMU;
				break;
			}
			case MODE_FOG_HP_TEST: {
				output_fn = acq_HP_test; 
				select_fn = SEL_HP_TEST;
				break;
			}
			case MODE_NMEA: {
				output_fn = acq_nmea;
				select_fn = SEL_NMEA;
				break;
            }
      case MODE_FOG_PARAMETER: {
          output_fn = acq_fog_parameter;
          select_fn = SEL_FOG_PARA;
          break;
      }

      default: break;
      }

      eeprom.Parameter_Write(EEPROM_ADDR_SELECT_FN, select_fn);

      eeprom.Parameter_Write(EEPROM_ADDR_OUTPUT_FN, (int)output_fn);

      eeprom.Parameter_Write(EEPROM_ADDR_REG_VALUE, value);
      // eeprom.Parameter_Read(EEPROM_ADDR_REG_VALUE, my_f.bin_val);
      // Serial.print("output_mode_setting - value: ");
      // Serial.print((int)value);
      // Serial.print(", ");
      // Serial.println(my_f.int_val);

	}
  
}

void temp_idle(byte &select_fn, unsigned int CTRLREG, byte ch)
{
	clear_SEL_EN(select_fn);
	// delay(100);
}

void fn_rst(byte &select_fn, unsigned int CTRLREG, byte ch)
{
	if(select_fn&SEL_RST) {
		switch(CTRLREG) {
			case REFILL_SERIAL1: {
				for(int i=0; i<256; i++) Serial1.read();
				break;
			}
			default: break;
		}
		
	}
	clear_SEL_EN(select_fn);
}


void acq_fog_parameter(byte &select_fn, unsigned int value, byte ch)
{
  byte *fog;
	uint8_t CRC32[4];
  String fpga_version;
	
	if(select_fn&SEL_FOG_PARA)
	{
    Serial.print("Enter fog parameter mode, channel: ");
    Serial.println(ch);
    CtrlReg = value;
    if(ch==1) run_fog_flag = sp13.setSyncMode(CtrlReg);
    else if(ch==2) run_fog_flag = sp14.setSyncMode(CtrlReg);
    else if(ch==3) run_fog_flag = sp9.setSyncMode(CtrlReg);

    

    switch(CtrlReg){
      case INT_SYNC:
        data_cnt = 0;
        Serial.println("Enter INT_SYNC mode");
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
      break;

      case EXT_SYNC:
        data_cnt = 0;
        Serial.println("Enter EXT_SYNC mode");
        Serial.println("Set EXTT to RISING");

        EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
      break;

      case STOP_SYNC:
        data_cnt = 0;
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
        disableWDT();
      break;

      default:
      break;
    }
    t_previous = millis();
	}
  if(run_fog_flag) {
	    t_new = micros();
      
           if(ch==1) fog = sp13.readData(header, sizeofheader, &try_cnt);
      else if(ch==2) fog = sp14.readData(header, sizeofheader, &try_cnt);
      else if(ch==3) fog = sp9.readData(header, sizeofheader, &try_cnt);
      
      if(fog) reg_fog = fog;

      if(ISR_PEDGE)
      {
        uint8_t* imu_data = (uint8_t*)malloc(18+4); // KVH_HEADER:4 + pig:14
        data_cnt++;
        mcu_time.ulong_val = millis() - t_previous;
        
        ISR_PEDGE = false;
        memcpy(imu_data, KVH_HEADER, 4);
        memcpy(imu_data+4, reg_fog, 14);
        memcpy(imu_data+18, mcu_time.bin_val, 4);
        myCRC.crc_32(imu_data, 22, CRC32);
        free(imu_data);

        #ifdef UART_RS422_CMD
        if(data_cnt >= DELAY_CNT)
        {
          Serial1.write(KVH_HEADER, 4);
          Serial1.write(reg_fog, 14);
          Serial1.write(mcu_time.bin_val, 4);
          Serial1.write(CRC32, 4);
        }
       #endif
        
      }
	    
      t_old = t_new;
      resetWDT();
        
	}
	clear_SEL_EN(select_fn);	
}


void acq_fog(byte &select_fn, unsigned int value, byte ch)
{
  byte *fog;
	uint8_t CRC32[4];
  my_float_t pd_temp;
	
	if(select_fn&SEL_FOG_1)
	{
    Serial.print("Enter acq_fog mode, fog channel: ");
    Serial.println(ch);
    CtrlReg = value;
    if(ch==1) run_fog_flag = sp13.setSyncMode(CtrlReg);
    else if(ch==2) run_fog_flag = sp14.setSyncMode(CtrlReg);
    else if(ch==3) run_fog_flag = sp9.setSyncMode(CtrlReg);

    

    switch(CtrlReg){
      case INT_SYNC:
        data_cnt = 0;
        Serial.println("Enter INT_SYNC mode");
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
      break;

      case EXT_SYNC:
        Serial.println("Enter EXT_SYNC mode");
        Serial.println("Set EXTT to RISING");
        data_cnt = 0;
        EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
      break;

      case STOP_SYNC:
        data_cnt = 0;
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
        disableWDT();
      break;

      default:
      break;
    }
    t_previous = millis();
	}


	if(run_fog_flag) {
	    t_new = micros();
      
           if(ch==1) fog = sp13.readData(header, sizeofheader, &try_cnt);
      else if(ch==2) fog = sp14.readData(header, sizeofheader, &try_cnt);
      else if(ch==3) fog = sp9.readData(header, sizeofheader, &try_cnt);
      
      if(fog) reg_fog = fog;
      pd_temp.float_val = convert_PDtemp(reg_fog[12], reg_fog[13]);
      if(ISR_PEDGE)
      {
        uint8_t* imu_data = (uint8_t*)malloc(16); // KVH_HEADER:4 + pig:14
        data_cnt++;
        mcu_time.ulong_val = millis() - t_previous;
        
        ISR_PEDGE = false;
        memcpy(imu_data, KVH_HEADER, 4);
        memcpy(imu_data+4, reg_fog+8, 4); //fog
        memcpy(imu_data+8, pd_temp.bin_val, 4);
        memcpy(imu_data+12, mcu_time.bin_val, 4);
        myCRC.crc_32(imu_data, 16, CRC32);
        free(imu_data);

        #ifdef UART_RS422_CMD
        if(data_cnt >= DELAY_CNT)
        {
          Serial1.write(KVH_HEADER, 4);
          Serial1.write(reg_fog+8, 4);
          Serial1.write(pd_temp.bin_val, 4);
          Serial1.write(mcu_time.bin_val, 4);
          Serial1.write(CRC32, 4);
        #endif
        }
        
      }
	    
      t_old = t_new;
      resetWDT();
        
	}
	clear_SEL_EN(select_fn);	
}


void acq_imu(byte &select_fn, unsigned int value, byte ch)
{
  my_acc_t my_memsXLM, my_memsGYRO;
  my_float_t pd_temp;

  byte *fog;
	uint8_t CRC32[4];

  if(select_fn&SEL_IMU)
	{
    Serial.print("Enter acq_imu mode, channel: ");
    Serial.println(ch);
    CtrlReg = value;

    if(ch==1) run_fog_flag = sp13.setSyncMode(CtrlReg);
    else if(ch==2) run_fog_flag = sp14.setSyncMode(CtrlReg);
    else if(ch==3) run_fog_flag = sp9.setSyncMode(CtrlReg);

    switch(CtrlReg){
      case INT_SYNC:
        data_cnt = 0;
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        
      break;
      case EXT_SYNC:
        data_cnt = 0;
        Serial.println("Enter EXT_SYNC mode");
        Serial.println("Set EXTT to CHANGE");

        EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);

      break;
      case STOP_SYNC:
        data_cnt = 0;
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
        disableWDT();
      break;

      default:
      break;
    }
    t_previous = millis();
	}

	if(run_fog_flag) {
        t_new = micros();

    fog = sp14.readData(header, sizeofheader, &try_cnt);
    if(fog) reg_fog = fog;
    pd_temp.float_val = convert_PDtemp(reg_fog[12], reg_fog[13]);

    if(ISR_PEDGE)
    {
      uint8_t* imu_data = (uint8_t*)malloc(36); // KVH_HEADER:4 + adxl355:9 + nano33_w:6 + nano33_a:6 + pig:14

      data_cnt++;

      mcu_time.ulong_val = millis() - t_previous;

      ISR_PEDGE = false;

      IMU.Get_X_Axes_f(my_memsXLM.float_val);
      IMU.Get_G_Axes_f(my_memsGYRO.float_val);

      memcpy(imu_data, KVH_HEADER, 4);
      memcpy(imu_data+4, my_memsGYRO.bin_val, 8);//wx, wy
      memcpy(imu_data+12, reg_fog+8, 4); //wz
      memcpy(imu_data+16, my_memsXLM.bin_val, 12);//ax, ay, az
      memcpy(imu_data+28, pd_temp.bin_val, 4);
      memcpy(imu_data+32, mcu_time.bin_val, 4);
      myCRC.crc_32(imu_data, 36, CRC32);

      free(imu_data);
      
      #ifdef UART_RS422_CMD
      if(data_cnt >= DELAY_CNT)
      {
        Serial1.write(KVH_HEADER, 4);
        Serial1.write(my_memsGYRO.bin_val, 8);
        Serial1.write(reg_fog+8, 4);
        Serial1.write(my_memsXLM.bin_val, 12);
        Serial1.write(pd_temp.bin_val, 4);
        Serial1.write(mcu_time.bin_val, 4);
        Serial1.write(CRC32, 4);
      }
      #endif   
    }
    t_old = t_new;    
    resetWDT();
	}
	clear_SEL_EN(select_fn);
}

void acq_nmea(byte &select_fn, unsigned int value, byte ch)
{
  byte *fog;
	uint8_t CRC32[4];
  String fpga_version;
	
	if(select_fn&SEL_NMEA)
	{
    Serial.print("Enter fog nmea mode, channel: ");
    Serial.println(ch);
    CtrlReg = value;
    if(ch==1) run_fog_flag = sp13.setSyncMode(CtrlReg);
    else if(ch==2) run_fog_flag = sp14.setSyncMode(CtrlReg);
    else if(ch==3) run_fog_flag = sp9.setSyncMode(CtrlReg);

    

    switch(CtrlReg){
      case INT_SYNC:
        data_cnt = 0;
        Serial.println("Enter INT_SYNC mode");
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
      break;

      case EXT_SYNC:
        Serial.println("Enter EXT_SYNC mode");
        Serial.println("Set EXTT to RISING");
        data_cnt = 0;
        EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
      break;

      case NMEA_MODE:
        Serial.println("Enter EXT_SYNC NMEA_MODE mode");
        Serial.println("Set EXTT to RISING");
        data_cnt = 0;
        EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
      break;

      case STOP_SYNC:
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
        data_cnt = 0;
        disableWDT();
      break;

      default:
      break;
    }
    t_previous = millis();
	}
  if(run_fog_flag) {
	    t_new = micros();
      
           if(ch==1) fog = sp13.readData(header, sizeofheader, &try_cnt);
      else if(ch==2) fog = sp14.readData(header, sizeofheader, &try_cnt);
      else if(ch==3) fog = sp9.readData(header, sizeofheader, &try_cnt);
      
      if(fog) reg_fog = fog;

      if(ISR_PEDGE)
      {

        ISR_PEDGE = false;
        data_cnt++;
        #ifdef UART_RS422_CMD
        if(data_cnt >= DELAY_CNT){
          Serial1.write(reg_fog, 14);
          Serial1.println("");
        }
        
       #endif 
      }
      resetWDT();
	}
	clear_SEL_EN(select_fn);	
}

void acq_HP_test(byte &select_fn, unsigned int value, byte ch)
{
  byte *fog, eeprom_var[4], mcu_var[4], adc_var[8];
	uint8_t CRC32[4];

	
	if(select_fn&SEL_HP_TEST)
	{
    Serial.print("fog channel: ");
    Serial.println(ch);
    Serial.println("select SEL_HP_TEST\n");
    CtrlReg = value;
    if(ch==1) run_fog_flag = sp13.setSyncMode(CtrlReg);
    else if(ch==2) run_fog_flag = sp14.setSyncMode(CtrlReg);
    else if(ch==3) run_fog_flag = sp9.setSyncMode(CtrlReg);

    switch(CtrlReg){
      case INT_SYNC:
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        
      break;
      case EXT_SYNC:
        Serial.println("Enter EXT_SYNC mode");
        Serial.println("Set EXTT to RISING");
        Serial.println("Write SYNC to LOW\n");

        EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);

      break;
      case STOP_SYNC:
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
        MCU_cnt = 0;
        disableWDT();
      break;

      case HP_TEST:
        Serial.println("Enter HP_TEST mode");
        Serial.println("Set EXTT to CHANGE");
        
        EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);

      break;

      default:
      break;
    }
	}

	// trig_status[0] = digitalRead(SYS_TRIG);

	if(run_fog_flag) {
	    t_new = micros();
      
           if(ch==1) fog = sp13.readData(header, sizeofheader, &try_cnt);
      else if(ch==2) fog = sp14.readData(header, sizeofheader, &try_cnt);
      else if(ch==3) fog = sp9.readData(header, sizeofheader, &try_cnt);
      
      if(fog)
      {
        uint8_t* imu_data = (uint8_t*)malloc(34); // KVH_HEADER:4 + pig:14

        readEEPROM(eeprom_var); //4
        readADC(adc_var);
        MCU_counter(mcu_var);
        // MCU_cnt++;//4
        
        memcpy(imu_data, KVH_HEADER, 4);
        memcpy(imu_data+4, fog, 14);
        memcpy(imu_data+18, eeprom_var, 4);
        memcpy(imu_data+22, mcu_var, 4);
        memcpy(imu_data+26, adc_var, 8);
        myCRC.crc_32(imu_data, 34, CRC32);
        free(imu_data);

        #ifdef UART_RS422_CMD
        Serial1.write(KVH_HEADER, 4);
        Serial1.write(fog, 14);
        Serial1.write(eeprom_var, 4);
        Serial1.write(mcu_var, 4);
        Serial1.write(adc_var, 8);
        Serial1.write(CRC32, 4);
       #endif
        
      }
	    
      t_old = t_new;
      resetWDT();
        
	}
	clear_SEL_EN(select_fn);	
}


// pd_temp.float_val = (float)reg_fog[12] + (float)(reg_fog[13]>>7)*0.5 ;
//       pd_temp.float_val = convert_PDtemp(reg_fog[12], reg_fog[13]);

float convert_PDtemp(byte dataH, byte dataL)
{
  return (int8_t)dataH + (dataL>>7)*0.5;
}

void convertGyro(byte data[6])
{
	int fog=0;

	fog = data[0]<<24 | data[1]<<16 | data[2]<<8 | data[3];
  // fog = fog
    Serial.print(data[0], HEX);
    Serial.print("\t");
    Serial.print(data[1], HEX);
    Serial.print("\t");
    Serial.print(data[2], HEX);
    Serial.print("\t");
    Serial.print(data[3], HEX);
    Serial.print("\t");
    Serial.print(data[4], HEX);
    Serial.print("\t");
    Serial.print(data[5], HEX);
    Serial.print("\t");
	Serial.println(fog);
};


void print_nano33XlmData(byte *data)
{
	int ax, ay, az;
	t_new = micros();
	ax = data[0]<<8 | data[1];
	ay = data[2]<<8 | data[3];
	az = data[4]<<8 | data[5];
	Serial.print(t_new - t_old);
	Serial.print('\t');
	Serial.print((float)ax*NANO33_XLM);
	Serial.print('\t');
	Serial.print((float)ay*NANO33_XLM);
	Serial.print('\t');
	Serial.println((float)az*NANO33_XLM);
	t_old = t_new;
}

void clear_SEL_EN(byte &select_fn)
{
	select_fn = SEL_DEFAULT;
}




void ISR_EXTT()
{
  // Serial.println(millis());
  sync_status = !sync_status;
  digitalWrite(PIG_SYNC, sync_status);
  ISR_Coming = !ISR_Coming;
  if(ISR_Coming == true) ISR_PEDGE = true;

  // EIC->CONFIG[1].bit.SENSE7 = 0; ////set interrupt condition to NONE
  }


void readEEPROM(byte data[4])
{
    // Serial1.println("\n\nEEPROM Value: ");
    eeprom.Parameter_Read(EEPROM_ADDR_DVT_TEST_1,my_f.bin_val);
    data[0] = my_f.bin_val[2];
    data[1] = my_f.bin_val[1];
    // Serial.println(" ");
    // Serial.println(my_f.bin_val[2], HEX);
    // Serial.println(my_f.bin_val[1], HEX);
    eeprom.Parameter_Read(EEPROM_ADDR_DVT_TEST_2,my_f.bin_val);
    data[2] = my_f.bin_val[2];
    data[3] = my_f.bin_val[1];
    // Serial.println(my_f.bin_val[2], HEX);
    // Serial.println(my_f.bin_val[1], HEX);
}

void MCU_counter(byte data[4])
{
  MCU_cnt++;
  data[0] = MCU_cnt >> 24;
  data[1] = MCU_cnt >> 16;
  data[2] = MCU_cnt >> 8;
  data[3] = MCU_cnt;
}

void readADC(byte data[8])
{

  // Serial1.print("\n\nVIN: ");
  // Serial1.println((float)analogRead(ADC_VIN)/0.27*ADC_CONV);
  // Serial1.print("ASE_TACT: ");
  // Serial1.println((float)analogRead(ADC_ASE_TACT)*ADC_CONV);
  // Serial1.print("ASE_IACT: ");
  // Serial1.println((float)analogRead(ADC_ASE_IACT)*ADC_CONV);
  // Serial1.print("PD_DC: ");
  // Serial1.println((float)analogRead(ADC_PD_DC)*ADC_CONV);

  my_f.int_val = analogRead(ADC_VIN);
  data[0] = my_f.bin_val[1];
  data[1] = my_f.bin_val[0];
  my_f.int_val = analogRead(ADC_ASE_TACT);
  data[2] = my_f.bin_val[1];
  data[3] = my_f.bin_val[0];
  my_f.int_val = analogRead(ADC_ASE_IACT);
  data[4] = my_f.bin_val[1];
  data[5] = my_f.bin_val[0];
  my_f.int_val = analogRead(ADC_PD_DC);
  data[6] = my_f.bin_val[1];
  data[7] = my_f.bin_val[0];
  // Serial.print(my_f.bin_val[1], HEX);
  // Serial.print(" ");
  // Serial.println(my_f.bin_val[0], HEX);
  
}

//============= resetWDT ===================================================== 
void resetWDT() {
  // reset the WDT watchdog timer.
  // this must be called before the WDT resets the system
  WDT->CLEAR.reg= 0xA5; // reset the WDT
  WDTsync(); 
  // Serial.println("resetWDT");
}

//============= systemReset ================================================== 
void systemReset() {
  // use the WDT watchdog timer to force a system reset.
  // WDT MUST be running for this to work
  WDT->CLEAR.reg= 0x00; // system reset via WDT
  WDTsync(); 
}

//============= setupWDT =====================================================
void setupWDT( uint8_t period) {
  // initialize the WDT watchdog timer

  WDT->CTRL.reg = 0; // disable watchdog
  WDTsync(); // sync is required

  WDT->CONFIG.reg = min(period,11); // see Table 17-5 Timeout Period (valid values 0-11)

  WDT->CTRL.reg = WDT_CTRL_ENABLE; //enable watchdog
  WDTsync(); 
  Serial.println("setupWDT");
}

//============= disable WDT =====================================================
void disableWDT() {
  WDT->CTRL.reg = 0; // disable watchdog
  WDTsync(); // sync is required
  Serial.println("disableWDT");
}

void parameter_init(void)
{
  // delay(2000);
  eeprom.Parameter_Read(EEPROM_ADDR_PARAMETER_EXIST,my_f.bin_val);
  EEPROM_Parameter_exist = my_f.int_val;
  /***fog parameter is empty,  write initial fog data*/
  if(EEPROM_Parameter_exist != EEPROM_PARAMETER_EXIST){
    eeprom.Parameter_Write(EEPROM_ADDR_PARAMETER_EXIST, EEPROM_PARAMETER_EXIST);
    Serial.println("EEPROM FOG parameter not exist!");
    /***output configuration*/
    Serial.println("Start setting output configuration.");
    write_fog_parameter_to_eeprom(EEPROM_BAUDRATE, EEPROM_ADDR_BAUDRATE, BAUDRATE_INIT);
    write_fog_parameter_to_eeprom(EEPROM_DATARATE, EEPROM_ADDR_DATARATE, DATARATE_INIT);
    set_output_configuration_init();
    /***end of output configuration*/

    /***fog parameters*/
    Serial.println("Start writing initial fog data.");
    eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
    #ifdef GP1Z 
      write_fog_parameter_to_eeprom_all(2);
      update_fpga_fog_parameter_init(100, 2);
    #endif
    #ifdef AFI
      write_fog_parameter_to_eeprom_all(1);
      write_fog_parameter_to_eeprom_all(2);
      write_fog_parameter_to_eeprom_all(3);
      update_fpga_fog_parameter_init(100, 1);
      update_fpga_fog_parameter_init(100, 2);
      update_fpga_fog_parameter_init(100, 3);
    #endif
    /***end of fog parameters*/

  }
  else{
    Serial.println("EEPROM FOG parameter exist!");
    /***fog parameters*/
    Serial.println("Start reading fog parameter from eeprom.");
    #ifdef GP1Z 
      read_fog_parameter_from_eeprom_all(2);
      update_fpga_fog_parameter_init(100, 2);
    #endif
    #ifdef AFI
      read_fog_parameter_from_eeprom_all(1);
      read_fog_parameter_from_eeprom_all(2);
      read_fog_parameter_from_eeprom_all(3);
      update_fpga_fog_parameter_init(100, 1);
      update_fpga_fog_parameter_init(100, 2);
      update_fpga_fog_parameter_init(100, 3);
    #endif
    /***end of fog parameters*/

    /***output configuration*/
    Serial.println("Start reading output configuration from eeprom.");
    read_fog_parameter_from_eeprom(EEPROM_BAUDRATE, EEPROM_ADDR_BAUDRATE);
    read_fog_parameter_from_eeprom(EEPROM_DATARATE, EEPROM_ADDR_DATARATE);
    report_current_output_configuration();
    set_output_configuration_init();
    /***end of output configuration*/
  }
}

void report_current_output_configuration()
{
  Serial1.begin(9600);
  switch(EEPROM_BAUDRATE)
  {
    case SET_BAUDRATE_230400: {
      Serial1.println("Baudrate set to 230400");
      Serial.println("Baudrate set to 230400");
      break;
    }
    case SET_BAUDRATE_115200: {
      Serial1.println("Baudrate set to 115200");
      Serial.println("Baudrate set to 115200");
      break;
    }
    case SET_BAUDRATE_9600: {
      Serial1.println("Baudrate set to 9600");
      Serial.println("Baudrate set to 9600");
      break;
    }
    case SET_BAUDRATE_4800: {
      Serial1.println("Baudrate set to 4800");
      Serial.println("Baudrate set to 4800");
      break;
    }
    default: {
      Serial1.println("Baudrate set to 230400");
      Serial.println("Baudrate set to 230400");
      break;
    }
  }

  switch(EEPROM_DATARATE)
  {
    case SET_DATARATE_100: {
      Serial1.println("Data rate set to 100 Hz");
      Serial.println("Data rate set to 100 Hz");
      break;
    }
    case SET_DATARATE_10: {
      Serial1.println("Data rate set to 10 Hz");
      Serial.println("Data rate set to 10 Hz");
      break;
    }
    default: {
      Serial1.println("Data rate set to 100 Hz");
      Serial.println("Data rate set to 100 Hz");
      break;
    }
  }
  delay(100);
}

void write_fog_parameter_to_eeprom_all(byte fog_ch)
{
  eeprom_obj *eeprom_obj_ptr;

  if(fog_ch==1) eeprom_obj_ptr = &eeprom_z;
  else if(fog_ch==2) eeprom_obj_ptr = &eeprom_x;
  else if(fog_ch==3) eeprom_obj_ptr = &eeprom_y;
  Serial.print("write_fog_parameter_to_eeprom_all, ch");
  Serial.println(fog_ch);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Mod_freq, eeprom_obj_ptr->EEPROM_ADDR_MOD_FREQ, MOD_FREQ_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Wait_cnt, eeprom_obj_ptr->EEPROM_ADDR_WAIT_CNT, WAIT_CNT_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Err_avg, eeprom_obj_ptr->EEPROM_ADDR_ERR_AVG, ERR_AVG_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Amp_H, eeprom_obj_ptr->EEPROM_ADDR_MOD_AMP_H, MOD_AMP_H_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Amp_L, eeprom_obj_ptr->EEPROM_ADDR_MOD_AMP_L, MOD_AMP_L_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Err_th, eeprom_obj_ptr->EEPROM_ADDR_ERR_TH, ERR_TH_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Err_offset, eeprom_obj_ptr->EEPROM_ADDR_ERR_OFFSET, ERR_OFFSET_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Polarity, eeprom_obj_ptr->EEPROM_ADDR_POLARITY, POLARITY_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Const_step, eeprom_obj_ptr->EEPROM_ADDR_CONST_STEP, CONST_STEP_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Fpga_Q, eeprom_obj_ptr->EEPROM_ADDR_FPGA_Q, FPGA_Q_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Fpga_R, eeprom_obj_ptr->EEPROM_ADDR_FPGA_R, FPGA_R_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Gain1, eeprom_obj_ptr->EEPROM_ADDR_GAIN1, GAIN1_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Gain2, eeprom_obj_ptr->EEPROM_ADDR_GAIN2, GAIN2_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_FB_ON, eeprom_obj_ptr->EEPROM_ADDR_FB_ON, FB_ON_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_DAC_gain, eeprom_obj_ptr->EEPROM_ADDR_DAC_GAIN, DAC_GAIN_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Data_delay, eeprom_obj_ptr->EEPROM_ADDR_DATA_DELAY, DATA_INT_DELAY_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF0, eeprom_obj_ptr->EEPROM_ADDR_SF_0, SF_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF1, eeprom_obj_ptr->EEPROM_ADDR_SF_1, SF_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF2, eeprom_obj_ptr->EEPROM_ADDR_SF_2, SF_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF3, eeprom_obj_ptr->EEPROM_ADDR_SF_3, SF_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF4, eeprom_obj_ptr->EEPROM_ADDR_SF_4, SF_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF5, eeprom_obj_ptr->EEPROM_ADDR_SF_5, SF_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF6, eeprom_obj_ptr->EEPROM_ADDR_SF_6, SF_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF7, eeprom_obj_ptr->EEPROM_ADDR_SF_7, SF_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF8, eeprom_obj_ptr-> EEPROM_ADDR_SF_8, SF_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF9, eeprom_obj_ptr->EEPROM_ADDR_SF_9, SF_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SFB, eeprom_obj_ptr->EEPROM_ADDR_SFB, SFB_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_CUTOFF, eeprom_obj_ptr->EEPROM_ADDR_CUTOFF, CUTOFF_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_TMIN, eeprom_obj_ptr->EEPROM_ADDR_TMIN, MINUS20);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_TMAX, eeprom_obj_ptr->EEPROM_ADDR_TMAX, PLUS80);

}


void write_fog_parameter_to_eeprom(int& eeprom_var, unsigned int eeprom_addr, int value)
{
  /**copy to eeprom variable*/
  eeprom_var = value;
  /**write to eeprom address*/
  eeprom.Parameter_Write(eeprom_addr, value);
}

void read_fog_parameter_from_eeprom_all(byte fog_ch)
{
  eeprom_obj *eeprom_obj_ptr;

  if(fog_ch==1) eeprom_obj_ptr = &eeprom_z;
  else if(fog_ch==2) eeprom_obj_ptr = &eeprom_x;
  else if(fog_ch==3) eeprom_obj_ptr = &eeprom_y;

  Serial.print("read_fog_parameter_from_eeprom_all, ch");
  Serial.println(fog_ch);

  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_Mod_freq, eeprom_obj_ptr->EEPROM_ADDR_MOD_FREQ);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_Wait_cnt, eeprom_obj_ptr->EEPROM_ADDR_WAIT_CNT);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_Err_avg, eeprom_obj_ptr->EEPROM_ADDR_ERR_AVG);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_Amp_H, eeprom_obj_ptr->EEPROM_ADDR_MOD_AMP_H);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_Amp_L, eeprom_obj_ptr->EEPROM_ADDR_MOD_AMP_L);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_Err_th, eeprom_obj_ptr->EEPROM_ADDR_ERR_TH);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_Err_offset, eeprom_obj_ptr->EEPROM_ADDR_ERR_OFFSET);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_Polarity, eeprom_obj_ptr->EEPROM_ADDR_POLARITY);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_Const_step, eeprom_obj_ptr->EEPROM_ADDR_CONST_STEP);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_Fpga_Q, eeprom_obj_ptr->EEPROM_ADDR_FPGA_Q);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_Fpga_R, eeprom_obj_ptr->EEPROM_ADDR_FPGA_R);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_Gain1, eeprom_obj_ptr->EEPROM_ADDR_GAIN1);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_Gain2, eeprom_obj_ptr->EEPROM_ADDR_GAIN2);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_FB_ON, eeprom_obj_ptr->EEPROM_ADDR_FB_ON);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_DAC_gain, eeprom_obj_ptr->EEPROM_ADDR_DAC_GAIN);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_Data_delay, eeprom_obj_ptr->EEPROM_ADDR_DATA_DELAY);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SF0, eeprom_obj_ptr->EEPROM_ADDR_SF_0);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SF1, eeprom_obj_ptr->EEPROM_ADDR_SF_1);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SF2, eeprom_obj_ptr->EEPROM_ADDR_SF_2);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SF3, eeprom_obj_ptr->EEPROM_ADDR_SF_3);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SF4, eeprom_obj_ptr->EEPROM_ADDR_SF_4);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SF5, eeprom_obj_ptr->EEPROM_ADDR_SF_5);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SF6, eeprom_obj_ptr->EEPROM_ADDR_SF_6);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SF7, eeprom_obj_ptr->EEPROM_ADDR_SF_7);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SF8, eeprom_obj_ptr->EEPROM_ADDR_SF_8);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SF9, eeprom_obj_ptr->EEPROM_ADDR_SF_9);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SFB, eeprom_obj_ptr->EEPROM_ADDR_SFB);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_CUTOFF, eeprom_obj_ptr->EEPROM_ADDR_CUTOFF);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_TMIN, eeprom_obj_ptr->EEPROM_ADDR_TMIN);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_TMAX, eeprom_obj_ptr->EEPROM_ADDR_TMAX);

}

void read_fog_parameter_from_eeprom(int& eeprom_var, unsigned int eeprom_addr)
{
  /**read from eeprom address*/
  eeprom.Parameter_Read(eeprom_addr, my_f.bin_val);
  /**copy to eeprom variable*/
  eeprom_var = my_f.int_val;
  // Serial.print(eeprom_addr);
  // Serial.print(", ");
  // Serial.println(eeprom_var);
}

void update_baudrate(byte eeprom_var)
{
  switch(eeprom_var)
  {
    case SET_BAUDRATE_230400: {
      Serial1.println("Baudrate set to 230400");
      Serial.println("Baudrate set to 230400");
      delay(100);
      Serial1.begin(230400);
      break;
    }
    case SET_BAUDRATE_115200: {
      Serial1.println("Baudrate set to 115200");
      Serial.println("Baudrate set to 115200");
      delay(100);
      Serial1.begin(115200);
      break;
    }
    case SET_BAUDRATE_9600: {
      Serial1.println("Baudrate set to 9600");
      Serial.println("Baudrate set to 9600");
      delay(100);
      Serial1.begin(9600);
      break;
    }
    case SET_BAUDRATE_4800: {
      Serial1.println("Baudrate set to 4800");
      Serial.println("Baudrate set to 4800");
      delay(100);
      Serial1.begin(4800);
      break;
    }
    default:{
      Serial1.println("Baudrate set to 230400");
      Serial.println("Baudrate set to 230400");
      delay(100);
      Serial1.begin(230400);
      break;
    }
  }
}

void update_datarate(byte eeprom_var)
{
  switch(eeprom_var)
  {
    case SET_DATARATE_100: {
      pwm.timer(1, 2, int(60000*PWM_FIX), false); //12M/2/60000 = 100Hz
      Serial.println("Data rate set to 100 Hz");
      Serial1.println("Data rate set to 100 Hz");
      delay(100);
      break;
    }
    case SET_DATARATE_10: {
      pwm.timer(1, 2, int(600000*PWM_FIX), false); //12M/2/600000 = 10Hz
      Serial.println("Data rate set to 10 Hz");
      Serial1.println("Data rate set to 10 Hz");
      delay(100);
      break;
    }
    default:{
      pwm.timer(1, 2, int(60000*PWM_FIX), false); //12M/2/60000 = 100Hz
      Serial.println("Data rate set to 100 Hz");
      Serial1.println("Data rate set to 100 Hz");
      delay(100);
      break;
    }
  }
}

void set_output_configuration_init()
{
  Serial.println("Setting output configuration!");
  update_baudrate(EEPROM_BAUDRATE);
  update_datarate(EEPROM_DATARATE);
  Serial.println("Setting output configuration done");
}

void update_fpga_fog_parameter_init(int dly_time, unsigned char fog_ch)
{
  Serial.print("Setting SP initail parameters, ch");
  Serial.println(fog_ch);

  PIG *sp;
  if(fog_ch==1) sp=&sp13;
  else if(fog_ch==2) sp=&sp14;
  else if(fog_ch=3) sp=&sp9;

  eeprom_obj *eeprom_ptr;

  if(fog_ch==1) eeprom_ptr = &eeprom_z;
  else if(fog_ch==2) eeprom_ptr = &eeprom_x;
  else if(fog_ch==3) eeprom_ptr = &eeprom_y;

  sp->sendCmd(myCmd_header, MOD_FREQ_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Mod_freq);
  delay(dly_time);
  sp->sendCmd(myCmd_header, WAIT_CNT_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Wait_cnt);
  delay(dly_time);
  sp->sendCmd(myCmd_header, ERR_AVG_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Err_avg);
  delay(dly_time);
  sp->sendCmd(myCmd_header, MOD_AMP_H_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Amp_H);
  delay(dly_time);
  sp->sendCmd(myCmd_header, MOD_AMP_L_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Amp_L);
  delay(dly_time);
  sp->sendCmd(myCmd_header, ERR_TH_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Err_th);
  delay(dly_time);
  sp->sendCmd(myCmd_header, ERR_OFFSET_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Err_offset);
  delay(dly_time);
  sp->sendCmd(myCmd_header, POLARITY_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Polarity);
  delay(dly_time);
  sp->sendCmd(myCmd_header, CONST_STEP_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Const_step);
  delay(dly_time);
  sp->sendCmd(myCmd_header, FPGA_Q_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Fpga_Q);
  delay(dly_time);
  sp->sendCmd(myCmd_header, FPGA_R_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Fpga_R);
  delay(dly_time);
  sp->sendCmd(myCmd_header, GAIN1_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Gain1);
  delay(dly_time);
  sp->sendCmd(myCmd_header, GAIN2_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Gain2);
  delay(dly_time);
  sp->sendCmd(myCmd_header, FB_ON_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_FB_ON);
  delay(dly_time);
  sp->sendCmd(myCmd_header, FB_ON_ADDR, myCmd_trailer, 0);
  delay(dly_time);
  sp->sendCmd(myCmd_header, FB_ON_ADDR, myCmd_trailer,eeprom_ptr-> EEPROM_FB_ON);
  delay(dly_time);
  sp->sendCmd(myCmd_header, DAC_GAIN_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_DAC_gain);
  delay(dly_time);
  sp->sendCmd(myCmd_header, DATA_INT_DELAY_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Data_delay);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SF0_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF0);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SF1_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF1);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SF2_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF2);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SF3_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF3);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SF4_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF4);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SF5_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF5);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SF6_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF6);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SF7_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF7);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SF8_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF8);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SF9_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SF9);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SFB_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SFB);
  delay(dly_time);
  sp->sendCmd(myCmd_header, CUTOFF_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_CUTOFF);
  delay(dly_time);
  sp->sendCmd(myCmd_header, TMIN_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_TMIN);
  delay(dly_time);
  sp->sendCmd(myCmd_header, TMAX_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_TMAX);
  delay(dly_time);
  Serial.println("Setting SP parameters done");
}

void Blink_MCU_LED()
{
  bool A=0;
  for(int i=0; i<10; i++){
    digitalWrite(MCU_LED, A);
    delay(100);
    A = !A;
  }
   
   delay(100);
}

void Wait_FPGA_Wakeup(byte fog_ch)
{
  PIG *sp;
  Stream *SER;
  byte times=0;

  if(fog_ch==1){
    sp = &sp13;
    SER = &Serial2;
  }
  else if(fog_ch==2){
    sp = &sp14;
    SER = &Serial3;
  } 
  else if(fog_ch=3){
    sp = &sp9;
    SER = &Serial4;
  } 
  byte flag = 0;

  for(int i=0; i<255; i++) SER->read();//clear serial buffer
  sp->updateParameter(myCmd_header, FPGA_WAKEUP_ADDR, myCmd_trailer, 5, 0xCC);
  delay(10);
  flag = (SER->readStringUntil('\n'))[0];
  int t0=millis();
  while(!flag){
    if((millis()-t0)>500){
      times++;
      flag = (SER->readStringUntil('\n'))[0];
      for(int i=0; i<255; i++) SER->read();//clear serial buffer
      sp->updateParameter(myCmd_header, FPGA_WAKEUP_ADDR, myCmd_trailer, 5, 0xCC);
      Serial.print("ch: ");
      Serial.print(fog_ch);
      Serial.print(", FPGA Sleeping: ");
      Serial.println(times);
      t0 = millis();
    }
  } 
  Serial.print("ch: ");
  Serial.print(fog_ch);
  Serial.println(", FPGA Wakeup! ");
}

