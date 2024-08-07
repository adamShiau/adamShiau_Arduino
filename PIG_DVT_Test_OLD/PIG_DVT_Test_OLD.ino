// #include "adxl355.h"
// #include <ASM330LHH.h>
#include <ASM330LHHSensor.h>
#include "pig_v2.h"
#include "IMU_PIG_DEFINE.h"
#include "wiring_private.h"
#include "crcCalculator.h"
#include "uartRT.h"
// #include "SparrowParaDefine.h"
// #include "Sparrow_read.h"
#include <TinyGPSPlus.h>
#include <EEPROM_24AA32A_I2C.h>

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

// #SP init parameter
#define MOD_FREQ_INIT 104
#define WAIT_CNT_INIT 60
#define ERR_AVG_INIT 4
#define MOD_AMP_H_INIT 9492
#define MOD_AMP_L_INIT -9492
#define ERR_TH_INIT 64
#define ERR_OFFSET_INIT 0
#define POLARITY_INIT 1
#define CONST_STEP_INIT 16384
#define FPGA_Q_INIT 10
#define FPGA_R_INIT 104
#define GAIN1_INIT 5
#define GAIN2_INIT 7
#define FB_ON_INIT 1
#define DAC_GAIN_INIT 70
#define DATA_INT_DELAY_ADDR 2220

//PWM
#include <SAMD21turboPWM.h>
#define PWM100 7
#define PWM200 5
#define PWM250 11
#define PWM_FIX 0.981
// #define PWM_FIX 0.978
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

/*** global var***/
// int pin_scl_mux = 17;
// bool trig_status[2] = {0, 0};
unsigned int t_new, t_old=0;

unsigned long gps_init_time = 0;
unsigned int gps_date=0, gps_time=0;
bool gps_valid = 0;

// EEPROMM
EEPROM_24AA32A_I2C eeprom = EEPROM_24AA32A_I2C(myWire);

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
PIG sp13(Serial2); //SP13
PIG sp14(Serial3); //SP14
PIG sp9(Serial4); //SP14


/*** serial data from PC***/
byte rx_cnt = 0, cmd, fog_channel;
unsigned int value;
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

int t_adc = millis(), t_eeprom = millis();

/*** * Watch dog  * **/
static void   WDTsync() {
  while (WDT->STATUS.bit.SYNCBUSY == 1); //Just wait till WDT is free
}
int WDT_CNT=0;
int tt0=0, tt1, tt2, tt3;

// The TinyGPSPlus object
TinyGPSPlus gps;

// Uart mySerial13 (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);
// void SERCOM1_Handler()
// {
//     mySerial13.IrqHandler();
// }

// #ifdef ENABLE_SRS200
//     #define SRS200_SIZE 15
// #endif

// unsigned int tt1=0, tt2=0;

void setup() {

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

  pinMode(PIG_SYNC, OUTPUT); 
  digitalWrite(PIG_SYNC, sync_status);

  pinMode(MCU_LED, OUTPUT);
  digitalWrite(MCU_LED, HIGH);
  
  pinMode(nCONFIG, OUTPUT);
  

  
	Serial.begin(230400); //debug
	// Serial1.begin(230400); //to PC
  // Serial2.begin(115200); //fog
  // Serial3.begin(115200);
  // Serial4.begin(115200);

  // pinPeripheral(24, PIO_SERCOM);
  // pinPeripheral(25, PIO_SERCOM);

  // pinPeripheral(8, PIO_SERCOM);
  // pinPeripheral(13, PIO_SERCOM);

  // pinPeripheral(10, PIO_SERCOM_ALT);
  // pinPeripheral(9, PIO_SERCOM_ALT);

  //I2C
  myWire.begin();
  myWire.setClock(I2C_FAST_MODE);
  pinPeripheral(27, PIO_SERCOM);
  pinPeripheral(20, PIO_SERCOM);

  //SPI
  mySPI.begin();
  pinPeripheral(3, PIO_SERCOM_ALT);
  pinPeripheral(22, PIO_SERCOM_ALT);
  pinPeripheral(23, PIO_SERCOM_ALT);

  //Re-configure FPGA when system reset on MCU
  // digitalWrite(nCONFIG, LOW);
  // delay(50);
  // digitalWrite(nCONFIG, HIGH);
  // delay(500);

  // set_parameter_init();

   

  //ADC
  //  pinPeripheral(6, PIN_ATTR_ANALOG);
	
	// if (!IMU.begin()) {
  //   Serial.println("Failed to initialize IMU!");
  //   while (1);
  // }
  IMU.begin();
  IMU.Enable_X();
  IMU.Enable_G();
  IMU.Set_X_ODR(104.0);
  IMU.Set_X_FS(4);  
  IMU.Set_G_ODR(104.0);
  IMU.Set_G_FS(250); 


  
    

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
eeprom.Parameter_Write(EEPROM_ADDR_DVT_TEST_1, 12345);
eeprom.Parameter_Write(EEPROM_ADDR_DVT_TEST_2, 67890);
eeprom.Parameter_Write(EEPROM_ADDR_DVT_TEST_3, 246810);
  
	
/*** pwm ***/

  pwm.setClockDivider(2, false); //48MHz/4 = 12MHz
  pwm.timer(2, 2, int(24000*PWM_FIX), false); //12M/2/24000 = 250Hz
  pwm.timer(1, 2, int(60000*PWM_FIX), false); //12M/2/60000 = 100Hz
  pwm.timer(0, 2, int(30000*PWM_FIX), false); //12M/2/30000 = 200Hz
  
  pwm.analogWrite(PWM100, 500);  
  pwm.analogWrite(PWM200, 500);  
  pwm.analogWrite(PWM250, 500);

  // tt1 = millis();
  if(fog_op_status==1) // disconnected last time, send cmd again
  {
    // delay(100);
    Serial.println("AUTO RST");
    // output_fn = acq_imu2;
    // select_fn = SEL_IMU;
    output_fn = acq_fog2;
    select_fn = SEL_FOG_1;
    value = 2;
    fog_channel = 2;
    setupWDT(11);
  }
  BlinkMCULed();
}

void loop() {
  
	getCmdValue(cmd, value, fog_channel, cmd_complete);
	cmd_mux(cmd_complete, cmd, mux_flag);
	parameter_setting(mux_flag, cmd, value, fog_channel);
	output_mode_setting(mux_flag, cmd, select_fn);
	output_fn(select_fn, value, fog_channel);
  // tt2 = millis();
  // if((tt2-tt1)>=500)
  // {
  //   Serial.print("\nfog status: ");
  //   Serial.println(fog_op_status);
  //   tt1 = tt2;
  // }
  
  // readADC();
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

void getCmdValue(byte &uart_cmd, unsigned int &uart_value, byte &fog_ch, bool &uart_complete)
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

void set_parameter_init()
{
  Serial.println("Setting SP initail parameters!");
  sp14.sendCmd(myCmd_header, MOD_FREQ_ADDR, myCmd_trailer, MOD_FREQ_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, WAIT_CNT_ADDR, myCmd_trailer, WAIT_CNT_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, ERR_AVG_ADDR, myCmd_trailer, ERR_AVG_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, MOD_AMP_H_ADDR, myCmd_trailer, MOD_AMP_H_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, MOD_AMP_L_ADDR, myCmd_trailer, MOD_AMP_L_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, ERR_TH_ADDR, myCmd_trailer, ERR_TH_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, ERR_OFFSET_ADDR, myCmd_trailer, ERR_OFFSET_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, POLARITY_ADDR, myCmd_trailer, POLARITY_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, CONST_STEP_ADDR, myCmd_trailer, CONST_STEP_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, FPGA_Q_ADDR, myCmd_trailer, FPGA_Q_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, FPGA_R_ADDR, myCmd_trailer, FPGA_R_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, GAIN1_ADDR, myCmd_trailer, GAIN1_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, GAIN2_ADDR, myCmd_trailer, GAIN2_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, FB_ON_ADDR, myCmd_trailer, FB_ON_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, FB_ON_ADDR, myCmd_trailer, 0);
  delay(100);
  sp14.sendCmd(myCmd_header, FB_ON_ADDR, myCmd_trailer, FB_ON_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, DAC_GAIN_ADDR, myCmd_trailer, DAC_GAIN_INIT);
  delay(100);
  sp14.sendCmd(myCmd_header, DATA_INT_DELAY_ADDR, myCmd_trailer, DATA_INT_DELAY_ADDR);
  Serial.println("Setting SP parameters done");
}

void parameter_setting(byte &mux_flag, byte cmd, unsigned int value, byte fog_ch) 
{
	if(mux_flag == MUX_PARAMETER)
	{
		mux_flag = MUX_ESCAPE;
		switch(cmd) {
      case CMD_FOG_MOD_FREQ: {
        Serial.println(1);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, MOD_FREQ_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, MOD_FREQ_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, MOD_FREQ_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_MOD_AMP_H: {
        Serial.println(2);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, MOD_AMP_H_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, MOD_AMP_H_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, MOD_AMP_H_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_MOD_AMP_L: {
        Serial.println(3);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, MOD_AMP_L_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, MOD_AMP_L_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, MOD_AMP_L_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_ERR_OFFSET: {
        Serial.println(4);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, ERR_OFFSET_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, ERR_OFFSET_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, ERR_OFFSET_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_POLARITY: {
        Serial.println(5);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, POLARITY_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, POLARITY_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, POLARITY_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_WAIT_CNT:{
        Serial.println(6);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, WAIT_CNT_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, WAIT_CNT_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, WAIT_CNT_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_ERR_TH: {
        Serial.println(7);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, ERR_TH_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, ERR_TH_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, ERR_TH_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_ERR_AVG: {
        Serial.println(8);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, ERR_AVG_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, ERR_AVG_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, ERR_AVG_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_TIMER_RST: {
        Serial.println(9);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, TIMER_RST_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, TIMER_RST_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, TIMER_RST_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_GAIN1: {
        Serial.println(10);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, GAIN1_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, GAIN1_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, GAIN1_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_GAIN2: {
        Serial.println(11);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, GAIN2_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, GAIN2_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, GAIN2_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_FB_ON: {
        Serial.println(12);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, FB_ON_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, FB_ON_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, FB_ON_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_CONST_STEP: {
        Serial.println(13);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, CONST_STEP_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, CONST_STEP_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, CONST_STEP_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_FPGA_Q: {
        Serial.println(14);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, FPGA_Q_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, FPGA_Q_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, FPGA_Q_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_FPGA_R: {
        Serial.println(15);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, FPGA_R_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, FPGA_R_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, FPGA_R_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_DAC_GAIN: {
        Serial.println(16);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, DAC_GAIN_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, DAC_GAIN_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, DAC_GAIN_ADDR, myCmd_trailer, value, 0xCC);
        break;}
			case CMD_FOG_INT_DELAY: {
        Serial.println(17);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, DATA_INT_DELAY_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, DATA_INT_DELAY_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, DATA_INT_DELAY_ADDR, myCmd_trailer, value, 0xCC);
        break;}
      case CMD_FOG_OUT_START: {
        Serial.println(18);
        if(fog_ch==1)       sp13.updateParameter(myCmd_header, DATA_OUT_START_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==2)  sp14.updateParameter(myCmd_header, DATA_OUT_START_ADDR, myCmd_trailer, value, 0xCC);
        else if(fog_ch==3)  sp9.updateParameter(myCmd_header, DATA_OUT_START_ADDR, myCmd_trailer, value, 0xCC);
        break;}
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
				output_fn = acq_fog2;
				select_fn = SEL_FOG_1;
				break;
			}
			case MODE_IMU: {
				output_fn = acq_imu2; 
				select_fn = SEL_IMU;
				break;
			}
			case MODE_IMU_GPS: {
				output_fn = acq_imu_gps; 
				select_fn = SEL_IMU;
				break;
			}
			case MODE_EQ: {
				output_fn = test_mems_imu;
				select_fn = SEL_EQ;
				break;
            }
      case MODE_IMU_MEMS: {
          output_fn = acq_imu_mems;
          select_fn = SEL_IMU_MEMS;
          break;
      }
      case MODE_IMU_MEMS_GPS: {
          output_fn = acq_imu_mems_gps;
          select_fn = SEL_IMU_MEMS;
          break;
      }
      default: break;
      }
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

void test_mems_imu(byte &select_fn, unsigned int value, byte ch)
{
  int32_t accelerometer[3] = {};
  int32_t gyroscope[3] = {};

  if(select_fn&SEL_EQ)
	{
    
    if(value==1 || value==2) {
      Serial1.println("--------------------------------------");
      Serial1.println("-------ENTER READ MEMS IMU TESTING---------");
      Serial1.println("--------------------------------------");
      resetWDT();
    }
    else if(value==4) {
      Serial1.println("--------------------------------------");
      Serial1.println("-------ESCAPE READ MEMS IMU TESTING---------");
      Serial1.println("--------------------------------------");
      disableWDT();
    }
  }

  if(value==1) {
    IMU.Get_X_Axes(accelerometer);
    resetWDT();
  }
  else if(value==2) {
    IMU.Get_G_Axes(gyroscope);
    resetWDT();
  }
  
	clear_SEL_EN(select_fn);	
}

void acq_fog2(byte &select_fn, unsigned int value, byte ch)
{
  if(select_fn&SEL_FOG_1)
	{
    if(value==1 || value==2) {
      Serial1.println("--------------------------------------");
      Serial1.println("-------ENTER READ ADC TESTING---------");
      Serial1.println("--------------------------------------");
      resetWDT();
    }
    else if(value==4) {
      Serial1.println("--------------------------------------");
      Serial1.println("-------ESCAPE READ ADC TESTING---------");
      Serial1.println("--------------------------------------");
      disableWDT();
    }
  }

  if(value==1 || value==2) {
    readADC();
    resetWDT();
  }
  
  
  
	clear_SEL_EN(select_fn);	
}


void acq_imu2(byte &select_fn, unsigned int value, byte ch)
{
  
  if(select_fn&SEL_IMU)
	{
    
    if(value==1 || value==2) {
      Serial1.println("--------------------------------------");
      Serial1.println("-------ENTER READ MEMORY TESTING---------");
      Serial1.println("--------------------------------------");
      resetWDT();
    }
    else if(value==4) {
      Serial1.println("--------------------------------------");
      Serial1.println("-------ESCAPE READ MEMORY IMU TESTING---------");
      Serial1.println("--------------------------------------");
      disableWDT();
    }
  }

  if(value==1 || value==2) {
    readEEPROM();
    resetWDT();
  }

  
	clear_SEL_EN(select_fn);	
}


void acq_imu_gps(byte &select_fn, unsigned int CTRLREG, byte ch)
{
  // byte *fog1, *fog2, *fog3;
  byte *fog;
	uint8_t CRC32[4];

	
	if(select_fn&SEL_IMU_GPS)
	{
    Serial.print("fog channel: ");
    Serial.println(ch);
    Serial.println("select acq_imu_gps\n");
    CtrlReg = value;
    if(ch==1) run_fog_flag = sp13.setSyncMode(CtrlReg);
    else if(ch==2) run_fog_flag = sp14.setSyncMode(CtrlReg);
    else if(ch==3) run_fog_flag = sp9.setSyncMode(CtrlReg);

    switch(CtrlReg){
      case INT_SYNC:
        Serial.println("Set INT_SYNC mode");
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        break;
      case EXT_SYNC:
        Serial.println("Set EXT_SYNC mode, Set EXTT to CHANGE");
        EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Change

        break;
      case STOP_SYNC:
        Serial.println("Set STOP_SYNC mode");
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
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
        uint8_t* imu_data = (uint8_t*)malloc(18); // KVH_HEADER:4 + pig:14
        
        memcpy(imu_data, KVH_HEADER, 4);
        memcpy(imu_data+4, fog, 14);
        myCRC.crc_32(imu_data, 18, CRC32);
        free(imu_data);

        #ifdef UART_RS422_CMD
        Serial1.write(KVH_HEADER, 4);
        Serial1.write(fog, 14);
        Serial1.write(CRC32, 4);
       #endif
        
      }
	    
        t_old = t_new;
        
        
	}
	clear_SEL_EN(select_fn);	
}

void acq_imu_mems(byte &select_fn, unsigned int CTRLREG, byte ch)
{
	byte nano33_w[6], nano33_a[6];
    byte adxl355_a[9]={0,0,0,0,0,0,0,0,0};
    uint8_t CRC8, CRC32[4];

  
    if(select_fn&SEL_IMU_MEMS)
    {
        if(CTRLREG == INT_SYNC || CTRLREG == EXT_SYNC) run_fog_flag = 1;
        else if(CTRLREG == STOP_SYNC) run_fog_flag = 0;
    }
  
	// trig_status[0] = digitalRead(SYS_TRIG);
	if(run_fog_flag)
	{

        uint8_t* imu_data = (uint8_t*)malloc(25); // 9+4+6+6
        // adxl355.readData(adxl355_a);
        IMU.readGyroscope(nano33_w);
        IMU.readAcceleration(nano33_a);

        memcpy(imu_data, KVH_HEADER, 4);
        memcpy(imu_data+4, adxl355_a, 9);
        memcpy(imu_data+13, nano33_w, 6);
        memcpy(imu_data+19, nano33_a, 6);
        myCRC.crc_32(imu_data, 25, CRC32);
        free(imu_data);

        // #ifdef UART_SERIAL_5_CMD
        //     mySerial5.write(KVH_HEADER, 4);
        //     mySerial5.write(adxl355_a, 9);
        //     mySerial5.write(nano33_w, 6);
        //     mySerial5.write(nano33_a, 6);
        //      mySerial5.write(CRC32, 4);
        // #endif
        #ifdef UART_USB_CMD
            Serial.write(KVH_HEADER, 4);
            Serial.write(adxl355_a, 9);
            Serial.write(nano33_w, 6);
            Serial.write(nano33_a, 6);
            Serial.write(CRC32, 4);
        #endif
        #ifdef UART_RS422_CMD
            Serial1.write(KVH_HEADER, 4);
            Serial1.write(adxl355_a, 9);
            Serial1.write(nano33_w, 6);
            Serial1.write(nano33_a, 6);
            Serial1.write(CRC32, 4);
        #endif
	}
    /*--end of if-condition--*/
	// trig_status[1] = trig_status[0];
	clear_SEL_EN(select_fn);
}

void acq_imu_mems_gps(byte &select_fn, unsigned int CTRLREG, byte ch)
{
	byte nano33_w[6], nano33_a[6];
    byte adxl355_a[9]={0,0,0,0,0,0,0,0,0};
	byte gps_data[9];
    uint8_t CRC32[4];

    if(select_fn&SEL_IMU_MEMS)
    {
        if(CTRLREG == INT_SYNC || CTRLREG == EXT_SYNC) run_fog_flag = 1;
        else if(CTRLREG == STOP_SYNC) run_fog_flag = 0;
    }
  
	// trig_status[0] = digitalRead(SYS_TRIG);
	if(run_fog_flag)
	{

        uint8_t* imu_data = (uint8_t*)malloc(32); // 9+4+6+6+9
        // adxl355.readData(adxl355_a);
        IMU.readGyroscope(nano33_w);
        IMU.readAcceleration(nano33_a);
		    getGPStimeData(gps_data);

        memcpy(imu_data, KVH_HEADER, 4);
        memcpy(imu_data+4, adxl355_a, 9);
        memcpy(imu_data+13, nano33_w, 6);
        memcpy(imu_data+19, nano33_a, 6);
		    memcpy(imu_data+25, gps_data, 9);
        myCRC.crc_32(imu_data, 34, CRC32);
        free(imu_data);

        // #ifdef UART_SERIAL_5_CMD
        //     mySerial5.write(KVH_HEADER, 4);
        //     mySerial5.write(adxl355_a, 9);
        //     mySerial5.write(nano33_w, 6);
        //     mySerial5.write(nano33_a, 6);
        //      mySerial5.write(CRC32, 4);
        // #endif
        #ifdef UART_USB_CMD
            Serial.write(KVH_HEADER, 4);
            Serial.write(adxl355_a, 9);
            Serial.write(nano33_w, 6);
            Serial.write(nano33_a, 6);
            Serial.write(CRC32, 4);
        #endif
        #ifdef UART_RS422_CMD
            Serial1.write(KVH_HEADER, 4);
            Serial1.write(adxl355_a, 9);
            Serial1.write(nano33_w, 6);
            Serial1.write(nano33_a, 6);
			Serial1.write(gps_data, 9);
            Serial1.write(CRC32, 4);
        #endif
        if (gps_valid == 1) gps_valid = 0;
	}
    /*--end of if-condition--*/
	
	// trig_status[1] = trig_status[0];
	clear_SEL_EN(select_fn);
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
  // digitalWrite(PIG_SYNC, LOW);
}

#ifdef ENABLE_SRS200
void readSRS200Data(unsigned char header[2], unsigned char data[SRS200_SIZE])
{
	// checkHeader(header);
	mySerial5.readBytes(data, SRS200_SIZE);
	// printSRSdata(data);
}

void printSRSdata(unsigned char data[10])
{
	long omega;

	omega = (long)(data[3]<<24|data[2]<<16|data[1]<<8|data[0]);
	t_new = micros();
	// Serial.print(t_new - t_old);
	// Serial.print('\t');
	// Serial.print(data[3]);
	// Serial.print('\t');
	// Serial.print(data[2]);
	// Serial.print('\t');
	// Serial.print(data[1]);
	// Serial.print('\t');
	// Serial.print(data[0]);
	// Serial.print('\t');
	if((data[3]>>7) == 1)
		omega = omega - (long)(1<<32);
	Serial.println(omega);
	t_old = t_new;
}
#endif



void sendCmd(unsigned char addr, unsigned int value)
{
	Serial1.write(addr);
	Serial1.write(value>>24 & 0xFF);
	Serial1.write(value>>16 & 0xFF);
	Serial1.write(value>>8 & 0xFF);
	Serial1.write(value & 0xFF);
	delay(1);
}

void updateGPS(unsigned int ms)
{
  if(( millis()-gps_init_time ) >= ms)
  {
    gps_init_time = millis();

    displayGPSInfo();
	// Serial.print(gps_date);
	// Serial.print(", ");
	// Serial.println(gps_time);
  }
}

void getGPStimeData(byte data[9])
{
	// gps_valid = 1;
	data[0] = gps_date>>24;
  data[1] = gps_date>>16;
  data[2] = gps_date>>8;
	data[3] = gps_date;
	data[4] = gps_time >> 24;
	data[5] = gps_time >> 16;
	data[6] = gps_time >> 8;
	data[7] = gps_time;
	data[8] = gps_valid;
}

void displayGPSInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.print(" ");
  Serial.print(gps_valid);
  Serial.println();
}


void ISR_EXTT()
{
  // Serial.println(millis());
  sync_status = !sync_status;
  digitalWrite(PIG_SYNC, sync_status);
  // EIC->CONFIG[1].bit.SENSE7 = 0; ////set interrupt condition to NONE
  }

void readADC()
{
  int t_now = millis();
  if((t_now - t_adc) >= 500){
    t_adc = t_now;
    Serial1.print("\n\nVIN: ");
    Serial1.println((float)analogRead(ADC_VIN)/0.27*ADC_CONV);
    Serial1.print("ASE_TACT: ");
    Serial1.println((float)analogRead(ADC_ASE_TACT)*ADC_CONV);
    Serial1.print("ASE_IACT: ");
    Serial1.println((float)analogRead(ADC_ASE_IACT)*ADC_CONV);
    Serial1.print("PD_DC: ");
    Serial1.println((float)analogRead(ADC_PD_DC)*ADC_CONV);
  }
}

void readEEPROM()
{
  byte data[4];
  int t_now = millis();
  if((t_now - t_eeprom) >= 1000){
    t_eeprom = t_now;

    Serial1.println("\n\nEEPROM Value: ");
    eeprom.Parameter_Read(EEPROM_ADDR_DVT_TEST_1,my_f.bin_val);
    Serial1.println(my_f.int_val);
    eeprom.Parameter_Read(EEPROM_ADDR_DVT_TEST_2,my_f.bin_val);
    Serial1.println(my_f.int_val);
    eeprom.Parameter_Read(EEPROM_ADDR_DVT_TEST_3,my_f.bin_val);
    Serial1.println(my_f.int_val);
  }
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

void BlinkMCULed() {
  digitalWrite(MCU_LED, HIGH);
  delay(500);
  digitalWrite(MCU_LED, LOW);
  delay(500);
  digitalWrite(MCU_LED, HIGH);
  delay(500);
  digitalWrite(MCU_LED, LOW);
  delay(500);
  digitalWrite(MCU_LED, HIGH);
  delay(500);
  digitalWrite(MCU_LED, LOW);
  delay(500);
}

