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


byte reg_fog_x[16] = {0}, reg_fog_y[16] = {0}, reg_fog_z[16] = {0};
byte *reg_fog;


/*** serial data from PC***/
byte rx_cnt = 0, cmd, fog_channel;
int value;


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

unsigned int t_start;

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

// auto rst fn flag
byte rst_fn_flag = MODE_RST;

int WDT_CNT=0;
int tt0=0, tt1, tt2, tt3;
unsigned long data_cnt = 0;

unsigned int MCU_cnt = 0;

unsigned int t_previous = 0;

DumpParameter my_cali_para[PARAMETER_CNT];

void my_parameter_f(const char *parameter_name, float input_value, DumpParameter *output_data) 
{
  snprintf(output_data->str, MAX_STR_LENGTH, "\"%s\":%.10f", parameter_name, input_value);
  Serial.println(output_data->str);
  Serial1.print(output_data->str);
}

void setup() {

  XOSC32K_CLK_SET();

  pwm_init();
  myWDT_init(); //disable all WDT
  


    // EXTT
    /*** for IMU_V4  : EXTT = PA27, Variant pin = 26, EXINT[15]
     *   for PIG MCU : EXTT = PA27, Variant pin = 26, EXINT[15]
     *  ****/
  attachInterrupt(EXTT, ISR_EXTT, CHANGE);

  // disableWDT();

    // EXT WDT
  // pinMode(WDI, OUTPUT);
  // pinMode(EXT_WDT_EN, OUTPUT);
  // disable_EXT_WDT(EXT_WDT_EN);
  

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
  EIC->CONFIG[0].bit.SENSE5 = 0;  // set ISR no NONE



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
  
  myUART_init();
  myI2C_init();
  mySPI_init();

  //Re-configure FPGA when system reset on MCU
  digitalWrite(nCONFIG, LOW);
  delay(50);
  digitalWrite(nCONFIG, HIGH);
  delay(500);

  #ifdef GP1Z 
    Wait_FPGA_Wakeup(2);
  #endif

  #ifdef AFI 
    // Wait_FPGA_Wakeup(1);
    // Wait_FPGA_Wakeup(2);
    // Wait_FPGA_Wakeup(3);
    Serial1.println("AFI parameters initializing...... ");
  #endif
  Blink_MCU_LED();

  parameter_init();
  Blink_MCU_LED();

	/*** var initialization***/
	mux_flag = MUX_ESCAPE; 		//default set mux_flag to 2
	select_fn = SEL_DEFAULT; 	//default set select_fn to 128
	run_fog_flag = 0;
	output_fn = temp_idle;


  /***read eeprom current status*/
  eeprom.Read(EEPROM_ADDR_FOG_STATUS, &fog_op_status);
  Serial.print("fog_op_status: ");
  Serial.println(fog_op_status);

  if(fog_op_status==1) // disconnected last time, send cmd again
    {
      Serial.println("\n-------AUTO RST---------");
      Serial1.println("\n-------AUTO RST---------");

      eeprom.Parameter_Read(EEPROM_ADDR_OUTPUT_FN, my_f.bin_val);// read output function index from eeprom
      rst_fn_flag = my_f.int_val; 
      // rst_fn_flag = 10; //test output fn output of range 
      verify_output_fn(rst_fn_flag);// check if output function is valid
      PRINT_OUTPUT_MODE(rst_fn_flag);
      
      eeprom.Parameter_Read(EEPROM_ADDR_REG_VALUE, my_f.bin_val);//read reg value of output function
      uart_value = my_f.int_val;
      // value = 10; //test fn reg output of range 
      verify_output_fn_reg_value(uart_value);
      PRINT_OUTPUT_REG(uart_value);

      eeprom.Parameter_Read(EEPROM_ADDR_SELECT_FN, my_f.bin_val);
      select_fn = my_f.int_val;
      // select_fn = 10; //test select_fn output of range 
      verify_select_fn(select_fn);
      PRINT_SELECT_FN(select_fn);

      fog_channel = 2;
    }
    PRINT_MUX_FLAG(mux_flag);
    printVersion();
    t_start = millis();
  }

void loop() {
	// getCmdValue(cmd, value, fog_channel, cmd_complete);
	cmd_mux(cmd_complete, uart_cmd, mux_flag);
	parameter_setting(mux_flag, uart_cmd, uart_value, fog_ch);
	output_mode_setting(mux_flag, uart_cmd, select_fn);
	output_fn(select_fn, uart_value, fog_ch);
  // delay(100);
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
        Serial.print(fog_ch);
        Serial.print(", ");
        Serial.println(eeprom_ptr->EEPROM_ADDR_MOD_FREQ, HEX);
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

      case CMD_CALI_AX: {
        if(value != EEPROM_CALI_AX){
          Serial.println("CALI_AX changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_AX, EEPROM_ADDR_CALI_AX, value);
          misalignment_cali_coe._d.ax = EEPROM_CALI_AX;
        }
      break;}
      case CMD_CALI_AY: {
        if(value != EEPROM_CALI_AY){
          Serial.println("CALI_AY changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_AY, EEPROM_ADDR_CALI_AY, value);
          misalignment_cali_coe._d.ay = EEPROM_CALI_AY;
        }
      break;}
      case CMD_CALI_AZ: {
        if(value != EEPROM_CALI_AZ){
          Serial.println("CALI_AZ changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_AZ, EEPROM_ADDR_CALI_AZ, value);
          misalignment_cali_coe._d.az = EEPROM_CALI_AZ;
        }
      break;}
      case CMD_CALI_A11: {
        if(value != EEPROM_CALI_A11){
          Serial.println("CALI_A11 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A11, EEPROM_ADDR_CALI_A11, value);
          misalignment_cali_coe._d.a11 = EEPROM_CALI_A11;
        }
      break;}
      case CMD_CALI_A12: {
        if(value != EEPROM_CALI_A12){
          Serial.println("CALI_A12 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A12, EEPROM_ADDR_CALI_A12, value);
          misalignment_cali_coe._d.a12 = EEPROM_CALI_A12;
        }
      break;}
      case CMD_CALI_A13: {
        if(value != EEPROM_CALI_A13){
          Serial.println("CALI_A13 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A13, EEPROM_ADDR_CALI_A13, value);
          misalignment_cali_coe._d.a13 = EEPROM_CALI_A13;
        }
      break;}
      case CMD_CALI_A21: {
        if(value != EEPROM_CALI_A21){
          Serial.println("CALI_A21 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A21, EEPROM_ADDR_CALI_A21, value);
          misalignment_cali_coe._d.a21 = EEPROM_CALI_A21;
        }
      break;}
      case CMD_CALI_A22: {
        if(value != EEPROM_CALI_A22){
          Serial.println("CALI_A22 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A22, EEPROM_ADDR_CALI_A22, value);
          misalignment_cali_coe._d.a22 = EEPROM_CALI_A22;
        }
      break;}
      case CMD_CALI_A23: {
        if(value != EEPROM_CALI_A23){
          Serial.println("CALI_A23 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A23, EEPROM_ADDR_CALI_A23, value);
          misalignment_cali_coe._d.a23 = EEPROM_CALI_A23;
        }
      break;}
      case CMD_CALI_A31: {
        if(value != EEPROM_CALI_A31){
          Serial.println("CALI_A31 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A31, EEPROM_ADDR_CALI_A31, value);
          misalignment_cali_coe._d.a31 = EEPROM_CALI_A31;
        }
      break;}
      case CMD_CALI_A32: {
        if(value != EEPROM_CALI_A32){
          Serial.println("CALI_A32 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A32, EEPROM_ADDR_CALI_A32, value);
          misalignment_cali_coe._d.a32 = EEPROM_CALI_A32;
        }
      break;}
      case CMD_CALI_A33: {
        if(value != EEPROM_CALI_A33){
          Serial.println("CALI_A33 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A33, EEPROM_ADDR_CALI_A33, value);
          misalignment_cali_coe._d.a33 = EEPROM_CALI_A33;
        }
      break;}

      case CMD_CALI_GX: {
        if(value != EEPROM_CALI_GX){
          Serial.println("CALI_GX changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_GX, EEPROM_ADDR_CALI_GX, value);
          misalignment_cali_coe._d.gx = EEPROM_CALI_GX;
        }
      break;}
      case CMD_CALI_GY: {
        if(value != EEPROM_CALI_GY){
          Serial.println("CALI_GY changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_GY, EEPROM_ADDR_CALI_GY, value);
          misalignment_cali_coe._d.gy = EEPROM_CALI_GY;
        }
      break;}
      case CMD_CALI_GZ: {
        if(value != EEPROM_CALI_GZ){
          Serial.println("CALI_GZ changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_GZ, EEPROM_ADDR_CALI_GZ, value);
          misalignment_cali_coe._d.gz = EEPROM_CALI_GZ;
        }
      break;}
      case CMD_CALI_G11: {
        if(value != EEPROM_CALI_G11){
          Serial.println("CALI_G11 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G11, EEPROM_ADDR_CALI_G11, value);
          misalignment_cali_coe._d.g11 = EEPROM_CALI_G11;
        }
      break;}
      case CMD_CALI_G12: {
        if(value != EEPROM_CALI_G12){
          Serial.println("CALI_G12 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G12, EEPROM_ADDR_CALI_G12, value);
          misalignment_cali_coe._d.g12 = EEPROM_CALI_G12;
        }
      break;}
      case CMD_CALI_G13: {
        if(value != EEPROM_CALI_G13){
          Serial.println("CALI_G13 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G13, EEPROM_ADDR_CALI_G13, value);
          misalignment_cali_coe._d.g13 = EEPROM_CALI_G13;
        }
      break;}
      case CMD_CALI_G21: {
        if(value != EEPROM_CALI_G21){
          Serial.println("CALI_G21 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G21, EEPROM_ADDR_CALI_G21, value);
          misalignment_cali_coe._d.g21 = EEPROM_CALI_G21;
        }
      break;}
      case CMD_CALI_G22: {
        if(value != EEPROM_CALI_G22){
          Serial.println("CALI_G22 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G22, EEPROM_ADDR_CALI_G22, value);
          misalignment_cali_coe._d.g22 = EEPROM_CALI_G22;
        }
      break;}
      case CMD_CALI_G23: {
        if(value != EEPROM_CALI_G23){
          Serial.println("CALI_G23 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G23, EEPROM_ADDR_CALI_G23, value);
          misalignment_cali_coe._d.g23 = EEPROM_CALI_G23;
        }
      break;}
      case CMD_CALI_G31: {
        if(value != EEPROM_CALI_G31){
          Serial.println("CALI_G31 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G31, EEPROM_ADDR_CALI_G31, value);
          misalignment_cali_coe._d.g31 = EEPROM_CALI_G31;
        }
      break;}
      case CMD_CALI_G32: {
        if(value != EEPROM_CALI_G32){
          Serial.println("CALI_G32 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G32, EEPROM_ADDR_CALI_G32, value);
          misalignment_cali_coe._d.g32 = EEPROM_CALI_G32;
        }
      break;}
      case CMD_CALI_G33: {
        if(value != EEPROM_CALI_G33){
          Serial.println("CALI_G33 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G33, EEPROM_ADDR_CALI_G33, value);
          misalignment_cali_coe._d.g33 = EEPROM_CALI_G33;
        }
      break;}
      case CMD_BIAS_COMP_T1: {
        if(value != eeprom_ptr->EEPROM_BIAS_COMP_T1){
          Serial.println("BIAS_COMP_T1 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_BIAS_COMP_T1, eeprom_ptr->EEPROM_ADDR_BIAS_COMP_T1, value);
          sp->updateParameter(myCmd_header, BIAS_COMP_T1_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_BIAS_COMP_T1, 0xCC);
        }
        // Serial.println(eeprom_ptr->EEPROM_ADDR_BIAS_COMP_T1);
        // eeprom.Parameter_Read(eeprom_ptr->EEPROM_ADDR_BIAS_COMP_T1, my_f.bin_val);
        // Serial.println(my_f.float_val,10);
      break;}
      case CMD_BIAS_COMP_T2: {
        if(value != eeprom_ptr->EEPROM_BIAS_COMP_T2){
          Serial.println("BIAS_COMP_T2 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_BIAS_COMP_T2, eeprom_ptr->EEPROM_ADDR_BIAS_COMP_T2, value);
          sp->updateParameter(myCmd_header, BIAS_COMP_T2_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_BIAS_COMP_T2, 0xCC);
        }
        // Serial.println(eeprom_ptr->EEPROM_ADDR_BIAS_COMP_T2);
        // eeprom.Parameter_Read(eeprom_ptr->EEPROM_ADDR_BIAS_COMP_T2, my_f.bin_val);
        // Serial.println(my_f.float_val,10);
      break;}
      case CMD_SFB_1_SLOPE: {
        if(value != eeprom_ptr->EEPROM_SFB_1_SLOPE){
          Serial.println("SFB_1_SLOPE changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SFB_1_SLOPE, eeprom_ptr->EEPROM_ADDR_SFB_1_SLOPE, value);
          sp->updateParameter(myCmd_header, SFB_1_SLOPE_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SFB_1_SLOPE, 0xCC);
        }
      break;}
      case CMD_SFB_1_OFFSET: {
        if(value != eeprom_ptr->EEPROM_SFB_1_OFFSET){
          Serial.println("SFB_1_OFFSET changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SFB_1_OFFSET, eeprom_ptr->EEPROM_ADDR_SFB_1_OFFSET, value);
          sp->updateParameter(myCmd_header, SFB_1_OFFSET_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SFB_1_OFFSET, 0xCC);
        }
      break;}
      case CMD_SFB_2_SLOPE: {
        if(value != eeprom_ptr->EEPROM_SFB_2_SLOPE){
          Serial.println("SFB_2_SLOPE changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SFB_2_SLOPE, eeprom_ptr->EEPROM_ADDR_SFB_2_SLOPE, value);
          sp->updateParameter(myCmd_header, SFB_2_SLOPE_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SFB_2_SLOPE, 0xCC);
        }
      break;}
      case CMD_SFB_2_OFFSET: {
        if(value != eeprom_ptr->EEPROM_SFB_2_OFFSET){
          Serial.println("SFB_2_OFFSET changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SFB_2_OFFSET, eeprom_ptr->EEPROM_ADDR_SFB_2_OFFSET, value);
          sp->updateParameter(myCmd_header, SFB_2_OFFSET_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SFB_2_OFFSET, 0xCC);
        }
      break;}
      case CMD_SFB_3_SLOPE: {
        if(value != eeprom_ptr->EEPROM_SFB_3_SLOPE){
          Serial.println("SFB_3_SLOPE changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SFB_3_SLOPE, eeprom_ptr->EEPROM_ADDR_SFB_3_SLOPE, value);
          sp->updateParameter(myCmd_header, SFB_3_SLOPE_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SFB_3_SLOPE, 0xCC);
        }
      break;}
      case CMD_SFB_3_OFFSET: {
        if(value != eeprom_ptr->EEPROM_SFB_3_OFFSET){
          Serial.println("SFB_3_OFFSET changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SFB_3_OFFSET, eeprom_ptr->EEPROM_ADDR_SFB_3_OFFSET, value);
          sp->updateParameter(myCmd_header, SFB_3_OFFSET_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SFB_3_OFFSET, 0xCC);
        }
      break;}

      case CMD_FPGA_VERSION: {
        String fpga_version;
        for(int i=0; i<255; i++) SER->read();//clear serial buffer
        sp->updateParameter(myCmd_header, FPGA_VERSION_ADDR, myCmd_trailer, value, 0xCC);
        while(!SER->available());
        if(SER->available()) fpga_version = SER->readStringUntil('\n');
        Serial.print("CH: ");
        Serial.println(fog_ch);
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
      case CMD_DUMP_CALI_PARAMETERS: {
        Serial1.print('{');
        my_parameter_f("AX", misalignment_cali_coe._f.ax, &my_cali_para[0]);
        Serial1.print(", ");
        my_parameter_f("AY", misalignment_cali_coe._f.ay, &my_cali_para[1]);
        Serial1.print(", ");
        my_parameter_f("AZ", misalignment_cali_coe._f.az, &my_cali_para[2]);
        Serial1.print(", ");
        my_parameter_f("A11", misalignment_cali_coe._f.a11, &my_cali_para[3]);
        Serial1.print(", ");
        my_parameter_f("A12", misalignment_cali_coe._f.a12, &my_cali_para[4]);
        Serial1.print(", ");
        my_parameter_f("A13", misalignment_cali_coe._f.a13, &my_cali_para[5]);
        Serial1.print(", ");
        my_parameter_f("A21", misalignment_cali_coe._f.a21, &my_cali_para[6]);
        Serial1.print(", ");
        my_parameter_f("A22", misalignment_cali_coe._f.a22, &my_cali_para[7]);
        Serial1.print(", ");
        my_parameter_f("A23", misalignment_cali_coe._f.a23, &my_cali_para[8]);
        Serial1.print(", ");
        my_parameter_f("A31", misalignment_cali_coe._f.a31, &my_cali_para[9]);
        Serial1.print(", ");
        my_parameter_f("A32", misalignment_cali_coe._f.a32, &my_cali_para[10]);
        Serial1.print(", ");
        my_parameter_f("A33", misalignment_cali_coe._f.a33, &my_cali_para[11]);
        Serial1.print(", ");
        my_parameter_f("GX", misalignment_cali_coe._f.gx, &my_cali_para[12]);
        Serial1.print(", ");
        my_parameter_f("GY", misalignment_cali_coe._f.gy, &my_cali_para[13]);
        Serial1.print(", ");
        my_parameter_f("GZ", misalignment_cali_coe._f.gz, &my_cali_para[14]);
        Serial1.print(", ");
        my_parameter_f("G11", misalignment_cali_coe._f.g11, &my_cali_para[15]);
        Serial1.print(", ");
        my_parameter_f("G12", misalignment_cali_coe._f.g12, &my_cali_para[16]);
        Serial1.print(", ");
        my_parameter_f("G13", misalignment_cali_coe._f.g13, &my_cali_para[17]);
        Serial1.print(", ");
        my_parameter_f("G21", misalignment_cali_coe._f.g21, &my_cali_para[18]);
        Serial1.print(", ");
        my_parameter_f("G22", misalignment_cali_coe._f.g22, &my_cali_para[19]);
        Serial1.print(", ");
        my_parameter_f("G23", misalignment_cali_coe._f.g23, &my_cali_para[20]);
        Serial1.print(", ");
        my_parameter_f("G31", misalignment_cali_coe._f.g31, &my_cali_para[21]);
        Serial1.print(", ");
        my_parameter_f("G32", misalignment_cali_coe._f.g32, &my_cali_para[22]);
        Serial1.print(", ");
        my_parameter_f("G33", misalignment_cali_coe._f.g33, &my_cali_para[23]);
        Serial1.println("}");
        
        break;
      }

      case CMD_PD_MON_SW: {
        if(value != eeprom_ptr->EEPROM_Wait_cnt){
          Serial.println("FOG_WAIT_CNT changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Wait_cnt, eeprom_ptr->EEPROM_ADDR_WAIT_CNT, value);
          sp->updateParameter(myCmd_header, WAIT_CNT_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_Wait_cnt, 0xCC);
        }
        if(value == 1){ //CH1
          digitalWrite(ADCMUX_S1, LOW);
          digitalWrite(ADCMUX_S0, HIGH);
        }
        else if(value == 2){ //CH2
          digitalWrite(ADCMUX_S1, LOW);
          digitalWrite(ADCMUX_S0, LOW);
        } 
        else if(value == 3){ //CH3
          digitalWrite(ADCMUX_S1, HIGH);
          digitalWrite(ADCMUX_S0, LOW);
        } 
        Serial.print("PD monitor switch to: ");
        Serial.println(value);
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
        rst_fn_flag = MODE_RST;
				break;
			}
			case MODE_FOG: {
				output_fn = acq_fog;
				select_fn = SEL_FOG_1;
        rst_fn_flag = MODE_FOG;
				break;
			}
			case MODE_IMU: {
				output_fn = acq_imu; 
				select_fn = SEL_IMU;
        rst_fn_flag = MODE_IMU;
				break;
			}
			case MODE_FOG_HP_TEST: {
				output_fn = acq_HP_test; 
				select_fn = SEL_HP_TEST;
        rst_fn_flag = MODE_FOG_HP_TEST;
				break;
			}
			case MODE_NMEA: {
				output_fn = acq_nmea;
				select_fn = SEL_NMEA;
        rst_fn_flag = MODE_NMEA;
				break;
      }
      case MODE_FOG_PARAMETER: {
        output_fn = acq_fog_parameter;
        select_fn = SEL_FOG_PARA;
        rst_fn_flag = MODE_FOG_PARAMETER;
        break;
      }
      case MODE_AFI: {
        output_fn = acq_afi;
        select_fn = SEL_AFI;
        rst_fn_flag = MODE_AFI;
        break;
      }
      default: break;
    }

      eeprom.Parameter_Write(EEPROM_ADDR_SELECT_FN, select_fn);
      eeprom.Parameter_Write(EEPROM_ADDR_OUTPUT_FN, rst_fn_flag);
      eeprom.Parameter_Write(EEPROM_ADDR_REG_VALUE, uart_value);
	}
  if(fog_op_status==1) // for auto reset
  {
    fog_op_status=0;
    Serial.println("AUTO RST select function");
		switch(rst_fn_flag) {
			case MODE_RST: {
				output_fn = fn_rst;
				break;
			}
			case MODE_FOG: {
				output_fn = acq_fog;
        Serial.println("MODE_FOG");
				break;
			}
			case MODE_IMU: {
				output_fn = acq_imu; 
				break;
			}
			case MODE_FOG_HP_TEST: {
				output_fn = acq_HP_test; 
				break;
			}
			case MODE_NMEA: {
				output_fn = acq_nmea;
				break;
            }
      case MODE_FOG_PARAMETER: {
          output_fn = acq_fog_parameter;
          break;
      }
      case MODE_AFI: {
          output_fn = acq_afi;
          break;
      }
      
      default: break;
      }
      eeprom.Parameter_Write(EEPROM_ADDR_SELECT_FN, select_fn);
      eeprom.Parameter_Write(EEPROM_ADDR_OUTPUT_FN, rst_fn_flag);
      eeprom.Parameter_Write(EEPROM_ADDR_REG_VALUE, uart_value);
	}
  
}


void temp_idle(byte &select_fn, unsigned int CTRLREG, byte ch)
{
	clear_SEL_EN(select_fn);
  if((millis() - t_start)>=5000) 
  {
    t_start = millis();
    Serial.println("IDLE");
    Serial1.println("IDLE");
  }
}

void fn_rst(byte &select_fn, unsigned int CTRLREG, byte ch)
{
  Serial.println("Enter fn_rst!");
	if(select_fn&SEL_RST) {
		switch(CTRLREG) {
			case REFILL_SERIAL1: {
				for(int i=0; i<256; i++) Serial1.read();
				break;
			}
			default: break;
		}
  Serial.println("Set fog_op_status to 0");
  eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
  Serial.println("Set fn_rst to temp_idle");
  output_fn = temp_idle; 
	}
	clear_SEL_EN(select_fn);
}



static void acq_fog_parameter(byte &select_fn, unsigned int value, byte ch)
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
        EIC->CONFIG[0].bit.SENSE5 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
      break;

      case EXT_SYNC:
        data_cnt = 0;
        Serial.println("Enter EXT_SYNC mode");
        Serial.println("Set EXTT to RISING");

        EIC->CONFIG[0].bit.SENSE5 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
      break;

      case STOP_SYNC:
        data_cnt = 0;
        EIC->CONFIG[0].bit.SENSE5 = 0; //set interrupt condition to None
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
        uint8_t* imu_data = (uint8_t*)malloc(20+4); // KVH_HEADER:4 + pig:16
        data_cnt++;
        mcu_time.ulong_val = millis() - t_previous;
        
        ISR_PEDGE = false;
        memcpy(imu_data, KVH_HEADER, 4);
        memcpy(imu_data+4, reg_fog, 16);
        memcpy(imu_data+20, mcu_time.bin_val, 4);
        myCRC.crc_32(imu_data, 24, CRC32);
        free(imu_data);

        #ifdef UART_RS422_CMD
        if(data_cnt >= DELAY_CNT)
        {
          Serial1.write(KVH_HEADER, 4);
          Serial1.write(reg_fog, 16);
          Serial1.write(mcu_time.bin_val, 4);
          Serial1.write(CRC32, 4);
        }
       #endif
      resetWDT();
      reset_EXT_WDI(WDI);
      }
	    
      t_old = t_new;
      
        
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
        EIC->CONFIG[0].bit.SENSE5 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);
      break;

      case EXT_SYNC:
        Serial.println("Enter EXT_SYNC mode");
        // Serial.println("Set EXTT to RISING");
        data_cnt = 0;
        EIC->CONFIG[0].bit.SENSE5 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);
      break;

      case STOP_SYNC:
        data_cnt = 0;
        EIC->CONFIG[0].bit.SENSE5 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
        disableWDT();
        disable_EXT_WDT(EXT_WDT_EN);
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
        resetWDT();
        reset_EXT_WDI(WDI);
      }
	    
      t_old = t_new;
      
        
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
        EIC->CONFIG[0].bit.SENSE5 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        
      break;
      case EXT_SYNC:
        data_cnt = 0;
        Serial.println("Enter EXT_SYNC mode");
        Serial.println("Set EXTT to CHANGE");

        EIC->CONFIG[0].bit.SENSE5 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);

      break;
      case STOP_SYNC:
        data_cnt = 0;
        EIC->CONFIG[0].bit.SENSE5 = 0; //set interrupt condition to None
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
      Serial.println(Serial3.available());
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
      resetWDT();
    }
    t_old = t_new;    
    
	}
	clear_SEL_EN(select_fn);
}

void acq_afi(byte &select_fn, unsigned int value, byte ch)
{
  byte *fog_x, *fog_y, *fog_z;
  my_acc_t my_ADXL357, ADXL357_cali;
	uint8_t CRC32[4];
  my_float_t pd_temp_x, pd_temp_y, pd_temp_z;
	
	if(select_fn&SEL_AFI)
	{
    Serial.println("Enter acq_afi mode: ");
    CtrlReg = value;

    run_fog_flag = sp13.setSyncMode(CtrlReg) && sp14.setSyncMode(CtrlReg) && sp9.setSyncMode(CtrlReg);
    delay(10);
    run_fog_flag = sp13.setSyncMode(CtrlReg) && sp14.setSyncMode(CtrlReg) && sp9.setSyncMode(CtrlReg);
    // run_fog_flag = sp14.setSyncMode(CtrlReg);
    Serial.print("AFI run_fog_flag: ");
    Serial.println(run_fog_flag);

    switch(CtrlReg){
      case INT_SYNC:
        data_cnt = 0;
        Serial.println("Enter INT_SYNC mode");
        EIC->CONFIG[0].bit.SENSE5 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);
      break;

      case EXT_SYNC:
        Serial.println("Enter EXT_SYNC mode");
        Serial.println("Set EXTT to RISING");
        data_cnt = 0;
        EIC->CONFIG[0].bit.SENSE5 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);
      break;

      case STOP_SYNC:
        data_cnt = 0;
        EIC->CONFIG[0].bit.SENSE5 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
        disableWDT();
        disable_EXT_WDT(EXT_WDT_EN);
      break;

      default:
      break;
    }
    t_previous = millis();
	}


	if(run_fog_flag) {
	    t_new = micros();
      
      fog_x = SP14_Read.readData(header, sizeofheader, &try_cnt, nullptr, 1, 0);
      fog_y = SP9_Read.readData_2(header, sizeofheader, &try_cnt, nullptr, 1, 0);
      fog_z = SP13_Read.readData_3(header, sizeofheader, &try_cnt, nullptr, 1, 0);

      if(fog_x) memcpy(reg_fog_x, fog_x, sizeof(reg_fog_x));
      if(fog_y) memcpy(reg_fog_y, fog_y, sizeof(reg_fog_y));
      if(fog_z) memcpy(reg_fog_z, fog_z, sizeof(reg_fog_z));
    
      // pd_temp_x.float_val = convert_PDtemp(reg_fog_x);
      // pd_temp_y.float_val = convert_PDtemp(reg_fog_y);
      // pd_temp_z.float_val = convert_PDtemp(reg_fog_z);
      // pd_temp_x.float_val = convert_PDtemp(reg_fog_x[12], reg_fog_x[13]);
      // pd_temp_y.float_val = convert_PDtemp(reg_fog_y[12], reg_fog_y[13]);
      // pd_temp_z.float_val = convert_PDtemp(reg_fog_z[12], reg_fog_z[13]);
      // pd_temp_x.float_val = 31.2;
      // pd_temp_y.float_val = 31.3;
      // pd_temp_z.float_val = 31.4;
      if(ISR_PEDGE)
      {
        adxl357_i2c.readData_f(my_ADXL357.float_val);
        acc_cali(ADXL357_cali.float_val, my_ADXL357.float_val);
        gyro_cali(reg_fog_x, reg_fog_y, reg_fog_z);
        uint8_t* imu_data = (uint8_t*)malloc(44); // KVH_HEADER:4 + wx:4 + wy:4 +wz:4 +ax:4 +ay:4 +az:4 +Tz:4 +Ty:4 +Tz:4 + time:4
        data_cnt++;
        mcu_time.ulong_val = millis() - t_previous;
        
        ISR_PEDGE = false;
        memcpy(imu_data, KVH_HEADER, 4);
        memcpy(imu_data+ 4, reg_fog_x+8, 4); //fog_x
        memcpy(imu_data+ 8, reg_fog_y+8, 4); //fog_y
        memcpy(imu_data+12, reg_fog_z+8, 4); //fog_z
        memcpy(imu_data+16, ADXL357_cali.bin_val, 12); //ax, ay, az
        // memcpy(imu_data+16, my_ADXL357.bin_val, 12); //ax, ay, az
        memcpy(imu_data+28, reg_fog_x+12, 4); //Temp_x
        memcpy(imu_data+32, reg_fog_y+12, 4); //Temp_y
        memcpy(imu_data+36, reg_fog_z+12, 4); //Temp_z
        memcpy(imu_data+40, mcu_time.bin_val, 4);
        myCRC.crc_32(imu_data, 44, CRC32);
        free(imu_data);

        #ifdef UART_RS422_CMD
        if(data_cnt >= DELAY_CNT)
        {
          Serial1.write(KVH_HEADER, 4);
          Serial1.write(reg_fog_x+8, 4);
          Serial1.write(reg_fog_y+8, 4);
          Serial1.write(reg_fog_z+8, 4);
          Serial1.write(ADXL357_cali.bin_val, 12);
          // Serial1.write(my_ADXL357.bin_val, 12);
          Serial1.write(reg_fog_x+12, 4);
          Serial1.write(reg_fog_y+12, 4);
          Serial1.write(reg_fog_z+12, 4);
          Serial1.write(mcu_time.bin_val, 4);
          Serial1.write(CRC32, 4);
        #endif
        }
        resetWDT();
        reset_EXT_WDI(WDI);
      }
      t_old = t_new;
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
        EIC->CONFIG[0].bit.SENSE5 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
      break;

      case EXT_SYNC:
        Serial.println("Enter EXT_SYNC mode");
        Serial.println("Set EXTT to RISING");
        data_cnt = 0;
        EIC->CONFIG[0].bit.SENSE5 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
      break;

      case NMEA_MODE:
        Serial.println("Enter EXT_SYNC NMEA_MODE mode");
        Serial.println("Set EXTT to RISING");
        data_cnt = 0;
        EIC->CONFIG[0].bit.SENSE5 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
      break;

      case STOP_SYNC:
        EIC->CONFIG[0].bit.SENSE5 = 0; //set interrupt condition to None
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
        EIC->CONFIG[0].bit.SENSE5 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        
      break;
      case EXT_SYNC:
        Serial.println("Enter EXT_SYNC mode");
        Serial.println("Set EXTT to RISING");
        Serial.println("Write SYNC to LOW\n");

        EIC->CONFIG[0].bit.SENSE5 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);

      break;
      case STOP_SYNC:
        EIC->CONFIG[0].bit.SENSE5 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
        MCU_cnt = 0;
        disableWDT();
      break;

      case HP_TEST:
        Serial.println("Enter HP_TEST mode");
        Serial.println("Set EXTT to CHANGE");
        
        EIC->CONFIG[0].bit.SENSE5 = 3; ////set interrupt condition to Both
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
  return (float)dataH + (float)(dataL>>7)*0.5;
}

float convert_PDtemp(byte* fog)
{
  return (float)(*(fog+12));
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

  // EIC->CONFIG[0].bit.SENSE5 = 0; ////set interrupt condition to NONE
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


void parameter_init(void)
{
  // delay(2000);
  eeprom.Parameter_Read(EEPROM_ADDR_PARAMETER_EXIST,my_f.bin_val);
  EEPROM_Parameter_exist = my_f.int_val;

  /***fog parameter is empty,  write initial fog data*/
  if(EEPROM_Parameter_exist != EEPROM_PARAMETER_EXIST){
    eeprom.Parameter_Write(EEPROM_ADDR_PARAMETER_EXIST, EEPROM_PARAMETER_EXIST);
    Serial.println("\n*****EEPROM FOG parameter not exist!********");

    /***output configuration*/
    Serial.println("\nStart setting output configuration.");
    write_fog_parameter_to_eeprom(EEPROM_BAUDRATE, EEPROM_ADDR_BAUDRATE, BAUDRATE_INIT);
    write_fog_parameter_to_eeprom(EEPROM_DATARATE, EEPROM_ADDR_DATARATE, DATARATE_INIT);
    set_output_configuration_init();
    Serial.println("Setting output configuration done.");
    /***end of output configuration*/

    /*** IMU misalignment calibration*/
    Serial.println("\nStart setting IMU misalignment calibration.");
    write_fog_parameter_to_eeprom(EEPROM_CALI_AX, EEPROM_ADDR_CALI_AX, CALI_AX_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_AY, EEPROM_ADDR_CALI_AY, CALI_AY_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_AZ, EEPROM_ADDR_CALI_AZ, CALI_AZ_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_A11, EEPROM_ADDR_CALI_A11, CALI_A11_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_A12, EEPROM_ADDR_CALI_A12, CALI_A12_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_A13, EEPROM_ADDR_CALI_A13, CALI_A13_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_A21, EEPROM_ADDR_CALI_A21, CALI_A21_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_A22, EEPROM_ADDR_CALI_A22, CALI_A22_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_A23, EEPROM_ADDR_CALI_A23, CALI_A23_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_A31, EEPROM_ADDR_CALI_A31, CALI_A31_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_A32, EEPROM_ADDR_CALI_A32, CALI_A32_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_A33, EEPROM_ADDR_CALI_A33, CALI_A33_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_GX, EEPROM_ADDR_CALI_GX, CALI_GX_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_GY, EEPROM_ADDR_CALI_GY, CALI_GY_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_GZ, EEPROM_ADDR_CALI_GZ, CALI_GZ_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_G11, EEPROM_ADDR_CALI_G11, CALI_G11_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_G12, EEPROM_ADDR_CALI_G12, CALI_G12_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_G13, EEPROM_ADDR_CALI_G13, CALI_G13_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_G21, EEPROM_ADDR_CALI_G21, CALI_G21_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_G22, EEPROM_ADDR_CALI_G22, CALI_G22_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_G23, EEPROM_ADDR_CALI_G23, CALI_G23_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_G31, EEPROM_ADDR_CALI_G31, CALI_G31_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_G32, EEPROM_ADDR_CALI_G32, CALI_G32_INIT);
    write_fog_parameter_to_eeprom(EEPROM_CALI_G33, EEPROM_ADDR_CALI_G33, CALI_G33_INIT);
    misalignment_cali_coe._d.ax = CALI_AX_INIT;
    misalignment_cali_coe._d.ay = CALI_AY_INIT;
    misalignment_cali_coe._d.az = CALI_AZ_INIT;
    misalignment_cali_coe._d.a11 = CALI_A11_INIT;
    misalignment_cali_coe._d.a12 = CALI_A12_INIT;
    misalignment_cali_coe._d.a13 = CALI_A13_INIT;
    misalignment_cali_coe._d.a21 = CALI_A21_INIT;
    misalignment_cali_coe._d.a22 = CALI_A22_INIT;
    misalignment_cali_coe._d.a23 = CALI_A23_INIT;
    misalignment_cali_coe._d.a31 = CALI_A31_INIT;
    misalignment_cali_coe._d.a32 = CALI_A32_INIT;
    misalignment_cali_coe._d.a33 = CALI_A33_INIT;
    misalignment_cali_coe._d.gx = CALI_GX_INIT;
    misalignment_cali_coe._d.gy = CALI_GY_INIT;
    misalignment_cali_coe._d.gz = CALI_GZ_INIT;
    misalignment_cali_coe._d.g11 = CALI_G11_INIT;
    misalignment_cali_coe._d.g12 = CALI_G12_INIT;
    misalignment_cali_coe._d.g13 = CALI_G13_INIT;
    misalignment_cali_coe._d.g21 = CALI_G21_INIT;
    misalignment_cali_coe._d.g22 = CALI_G22_INIT;
    misalignment_cali_coe._d.g23 = CALI_G23_INIT;
    misalignment_cali_coe._d.g31 = CALI_G31_INIT;
    misalignment_cali_coe._d.g32 = CALI_G32_INIT;
    misalignment_cali_coe._d.g33 = CALI_G33_INIT;
    // my_parameter_f("AX", sf_b, &my_para[35]);
    Serial.println("Setting IMU misalignment calibration done.");
    /***end of IMU misalignment calibration*/

    /***fog parameters*/
    Serial.println("Start writing initial fog data.");
    // eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0); // change to 0 12/18
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

    /*** IMU misalignment calibration*/
    read_misalignment_calibration_from_eeprom();
    /***end of IMU misalignment calibration*/

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
  // BIAS temp. compensation
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_BIAS_COMP_T1, eeprom_obj_ptr->EEPROM_ADDR_BIAS_COMP_T1, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_BIAS_COMP_T2, eeprom_obj_ptr->EEPROM_ADDR_BIAS_COMP_T2, FLOAT_40);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SFB_1_SLOPE, eeprom_obj_ptr->EEPROM_ADDR_SFB_1_SLOPE, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SFB_1_OFFSET, eeprom_obj_ptr->EEPROM_ADDR_SFB_1_OFFSET, FLOAT_1);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SFB_2_SLOPE, eeprom_obj_ptr->EEPROM_ADDR_SFB_2_SLOPE, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SFB_2_OFFSET, eeprom_obj_ptr->EEPROM_ADDR_SFB_2_OFFSET, FLOAT_1);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SFB_3_SLOPE, eeprom_obj_ptr->EEPROM_ADDR_SFB_3_SLOPE, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SFB_3_OFFSET, eeprom_obj_ptr->EEPROM_ADDR_SFB_3_OFFSET, FLOAT_1);

}


void write_fog_parameter_to_eeprom(int& eeprom_var, unsigned int eeprom_addr, int value)
{
  /**copy to eeprom variable*/
  eeprom_var = value;
  /**write to eeprom address*/
  eeprom.Parameter_Write(eeprom_addr, value);
}

void read_misalignment_calibration_from_eeprom()
{
  Serial.println("\nread_misalignment_calibration_from_eeprom");
  read_fog_parameter_from_eeprom(EEPROM_CALI_AX, EEPROM_ADDR_CALI_AX);
  read_fog_parameter_from_eeprom(EEPROM_CALI_AY, EEPROM_ADDR_CALI_AY);
  read_fog_parameter_from_eeprom(EEPROM_CALI_AZ, EEPROM_ADDR_CALI_AZ);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A11, EEPROM_ADDR_CALI_A11);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A12, EEPROM_ADDR_CALI_A12);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A13, EEPROM_ADDR_CALI_A13);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A21, EEPROM_ADDR_CALI_A21);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A22, EEPROM_ADDR_CALI_A22);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A23, EEPROM_ADDR_CALI_A23);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A31, EEPROM_ADDR_CALI_A31);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A32, EEPROM_ADDR_CALI_A32);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A33, EEPROM_ADDR_CALI_A33);
  read_fog_parameter_from_eeprom(EEPROM_CALI_GX, EEPROM_ADDR_CALI_GX);
  read_fog_parameter_from_eeprom(EEPROM_CALI_GY, EEPROM_ADDR_CALI_GY);
  read_fog_parameter_from_eeprom(EEPROM_CALI_GZ, EEPROM_ADDR_CALI_GZ);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G11, EEPROM_ADDR_CALI_G11);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G12, EEPROM_ADDR_CALI_G12);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G13, EEPROM_ADDR_CALI_G13);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G21, EEPROM_ADDR_CALI_G21);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G22, EEPROM_ADDR_CALI_G22);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G23, EEPROM_ADDR_CALI_G23);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G31, EEPROM_ADDR_CALI_G31);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G32, EEPROM_ADDR_CALI_G32);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G33, EEPROM_ADDR_CALI_G33);
  misalignment_cali_coe._d.ax = EEPROM_CALI_AX;
  misalignment_cali_coe._d.ay = EEPROM_CALI_AY;
  misalignment_cali_coe._d.az = EEPROM_CALI_AZ;
  misalignment_cali_coe._d.a11 = EEPROM_CALI_A11;
  misalignment_cali_coe._d.a12 = EEPROM_CALI_A12;
  misalignment_cali_coe._d.a13 = EEPROM_CALI_A13;
  misalignment_cali_coe._d.a21 = EEPROM_CALI_A21;
  misalignment_cali_coe._d.a22 = EEPROM_CALI_A22;
  misalignment_cali_coe._d.a23 = EEPROM_CALI_A23;
  misalignment_cali_coe._d.a31 = EEPROM_CALI_A31;
  misalignment_cali_coe._d.a32 = EEPROM_CALI_A32;
  misalignment_cali_coe._d.a33 = EEPROM_CALI_A33;
  misalignment_cali_coe._d.gx = EEPROM_CALI_GX;
  misalignment_cali_coe._d.gy = EEPROM_CALI_GY;
  misalignment_cali_coe._d.gz = EEPROM_CALI_GZ;
  misalignment_cali_coe._d.g11 = EEPROM_CALI_G11;
  misalignment_cali_coe._d.g12 = EEPROM_CALI_G12;
  misalignment_cali_coe._d.g13 = EEPROM_CALI_G13;
  misalignment_cali_coe._d.g21 = EEPROM_CALI_G21;
  misalignment_cali_coe._d.g22 = EEPROM_CALI_G22;
  misalignment_cali_coe._d.g23 = EEPROM_CALI_G23;
  misalignment_cali_coe._d.g31 = EEPROM_CALI_G31;
  misalignment_cali_coe._d.g32 = EEPROM_CALI_G32;
  misalignment_cali_coe._d.g33 = EEPROM_CALI_G33;
  Serial.println("read_misalignment_calibration_from_eeprom done");
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
    // BIAS temp. compensation
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_BIAS_COMP_T1, eeprom_obj_ptr->EEPROM_ADDR_BIAS_COMP_T1);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_BIAS_COMP_T2, eeprom_obj_ptr->EEPROM_ADDR_BIAS_COMP_T2);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SFB_1_SLOPE, eeprom_obj_ptr->EEPROM_ADDR_SFB_1_SLOPE);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SFB_1_OFFSET, eeprom_obj_ptr->EEPROM_ADDR_SFB_1_OFFSET);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SFB_2_SLOPE, eeprom_obj_ptr->EEPROM_ADDR_SFB_2_SLOPE);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SFB_2_OFFSET, eeprom_obj_ptr->EEPROM_ADDR_SFB_2_OFFSET);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SFB_3_SLOPE, eeprom_obj_ptr->EEPROM_ADDR_SFB_3_SLOPE);
  read_fog_parameter_from_eeprom(eeprom_obj_ptr->EEPROM_SFB_3_OFFSET, eeprom_obj_ptr->EEPROM_ADDR_SFB_3_OFFSET);

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
    case SET_DATARATE_400: {
      pwm.timer(1, 2, int(15000*PWM_FIX), false); //12M/2/15000 = 400Hz
      pwm.analogWrite(PWM100, 500);  
      Serial.println("Data rate set to 400 Hz");
      Serial1.println("Data rate set to 400 Hz");
      delay(100);
      break;
    }
    case SET_DATARATE_200: {
      pwm.timer(1, 2, int(30000*PWM_FIX), false); //12M/2/30000 = 200Hz
      pwm.analogWrite(PWM100, 500);  
      Serial.println("Data rate set to 200 Hz");
      Serial1.println("Data rate set to 200 Hz");
      delay(100);
      break;
    }
    case SET_DATARATE_100: {
      pwm.timer(1, 2, int(60000*PWM_FIX), false); //12M/2/60000 = 100Hz
      pwm.analogWrite(PWM100, 500);  
      Serial.println("Data rate set to 100 Hz");
      Serial1.println("Data rate set to 100 Hz");
      delay(100);
      break;
    }
    case SET_DATARATE_10: {
      pwm.timer(1, 2, int(600000*PWM_FIX), false); //12M/2/600000 = 10Hz
      pwm.analogWrite(PWM100, 500);  
      Serial.println("Data rate set to 10 Hz");
      Serial1.println("Data rate set to 10 Hz");
      delay(100);
      break;
    }
    default:{
      pwm.timer(1, 2, int(60000*PWM_FIX), false); //12M/2/60000 = 100Hz
      pwm.analogWrite(PWM100, 500);  
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
  //  Bias temp. compensation
  sp->sendCmd(myCmd_header, BIAS_COMP_T1_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_BIAS_COMP_T1);
  delay(dly_time);
  sp->sendCmd(myCmd_header, BIAS_COMP_T2_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_BIAS_COMP_T2);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SFB_1_SLOPE_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SFB_1_SLOPE);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SFB_1_OFFSET_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SFB_1_OFFSET);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SFB_2_SLOPE_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SFB_2_SLOPE);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SFB_2_OFFSET_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SFB_2_OFFSET);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SFB_3_SLOPE_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SFB_3_SLOPE);
  delay(dly_time);
  sp->sendCmd(myCmd_header, SFB_3_OFFSET_ADDR, myCmd_trailer, eeprom_ptr->EEPROM_SFB_3_OFFSET);
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
  // byte flag = 0;

  for(int i=0; i<255; i++) SER->read();//clear serial buffer
  sp->updateParameter(myCmd_header, FPGA_WAKEUP_ADDR, myCmd_trailer, 5, 0xCC);
  delay(10);
  // flag = (SER->readStringUntil('\n'))[0];
  int t0=millis();
  while(!fog_woke_flag){
    if((millis()-t0)>500){
      times++;
      // flag = (SER->readStringUntil('\n'))[0];
      for(int i=0; i<255; i++) SER->read();//clear serial buffer
      sp->updateParameter(myCmd_header, FPGA_WAKEUP_ADDR, myCmd_trailer, 5, 0xCC);
      Serial.print("ch: ");
      Serial.print(fog_ch);
      Serial.print(", FPGA Sleeping: ");
      Serial.println(times);
      Serial1.print("ch: ");
      Serial1.print(fog_ch);
      Serial1.print(", FPGA Sleeping: ");
      Serial1.println(times);
      manual_Wait_FPGA_Wakeup_escape(fog_ch);
      t0 = millis();
    }
  } 
  Serial.print("ch: ");
  Serial.print(fog_ch);
  Serial.println(", FPGA Wakeup! ");
}

void manual_Wait_FPGA_Wakeup_escape(byte fog_ch)
{
  // flag = (Serial1.readStringUntil('\n'))[0];
  if(fog_woke_flag){
    digitalWrite(MCU_LED, HIGH);
    delay(100);
    digitalWrite(MCU_LED, LOW);
    delay(100);
    digitalWrite(MCU_LED, HIGH);
    delay(100);
    digitalWrite(MCU_LED, LOW);
    Serial1.print("ch");
    Serial1.print(fog_ch);
    Serial1.println(" woke up!");
  }
  else{
    digitalWrite(MCU_LED, HIGH);
    delay(100);
    digitalWrite(MCU_LED, LOW);
  }
}

void acc_cali(float acc_cli[3], float acc[3])
{
  acc_cli[0] = misalignment_cali_coe._f.a11 * acc[0] + 
               misalignment_cali_coe._f.a12 * acc[1] + 
               misalignment_cali_coe._f.a13 * acc[2] + 
               misalignment_cali_coe._f.ax;
  acc_cli[1] = misalignment_cali_coe._f.a21 * acc[0] + 
               misalignment_cali_coe._f.a22 * acc[1] + 
               misalignment_cali_coe._f.a23 * acc[2] + 
               misalignment_cali_coe._f.ay;
  acc_cli[2] = misalignment_cali_coe._f.a31 * acc[0] + 
               misalignment_cali_coe._f.a32 * acc[1] + 
               misalignment_cali_coe._f.a33 * acc[2] + 
               misalignment_cali_coe._f.az;
} 

void gyro_cali(byte gyro_clix[14], byte gyro_cliy[14], byte gyro_cliz[14])
{
  my_float_t x_f, y_f, z_f;
  my_float_t x_cli, y_cli, z_cli;

  x_f.bin_val[0] = gyro_clix[11];
  x_f.bin_val[1] = gyro_clix[10];
  x_f.bin_val[2] = gyro_clix[9];
  x_f.bin_val[3] = gyro_clix[8];

  y_f.bin_val[0] = gyro_cliy[11];
  y_f.bin_val[1] = gyro_cliy[10];
  y_f.bin_val[2] = gyro_cliy[9];
  y_f.bin_val[3] = gyro_cliy[8];

  z_f.bin_val[0] = gyro_cliz[11];
  z_f.bin_val[1] = gyro_cliz[10];
  z_f.bin_val[2] = gyro_cliz[9];
  z_f.bin_val[3] = gyro_cliz[8];

  x_cli.float_val = misalignment_cali_coe._f.g11 * x_f.float_val + 
                    misalignment_cali_coe._f.g12 * y_f.float_val + 
                    misalignment_cali_coe._f.g13 * z_f.float_val + 
                    misalignment_cali_coe._f.gx;
  y_cli.float_val = misalignment_cali_coe._f.g21 * x_f.float_val + 
                    misalignment_cali_coe._f.g22 * y_f.float_val + 
                    misalignment_cali_coe._f.g23 * z_f.float_val + 
                    misalignment_cali_coe._f.gy;
  z_cli.float_val = misalignment_cali_coe._f.g31 * x_f.float_val + 
                    misalignment_cali_coe._f.g32 * y_f.float_val + 
                    misalignment_cali_coe._f.g33 * z_f.float_val + 
                    misalignment_cali_coe._f.gz;

  gyro_clix[11] = x_cli.bin_val[0];
  gyro_clix[10] = x_cli.bin_val[1];
  gyro_clix[9] = x_cli.bin_val[2];
  gyro_clix[8] = x_cli.bin_val[3];

  gyro_cliy[11] = y_cli.bin_val[0];
  gyro_cliy[10] = y_cli.bin_val[1];
  gyro_cliy[9] = y_cli.bin_val[2];
  gyro_cliy[8] = y_cli.bin_val[3];

  gyro_cliz[11] = z_cli.bin_val[0];
  gyro_cliz[10] = z_cli.bin_val[1];
  gyro_cliz[9] = z_cli.bin_val[2];
  gyro_cliz[8] = z_cli.bin_val[3];
} 

  void printVersion()
  {
    Serial.print("Version:");
    Serial.println(MCU_VERSION);
    Serial1.print("Version:");
    Serial1.println(MCU_VERSION);
  }

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

void verify_output_fn(byte in)
{
  Serial.println("\nverifying output fn........" );
  Serial.print("Input function index: ");
  Serial.println(in);

  Serial1.println("\nverifying output fn........" );
  Serial1.println("Input function index: ");
  Serial1.println(in);
  if(!( in==MODE_RST |in==MODE_FOG | in==MODE_IMU | in==MODE_FOG_HP_TEST |
    in==MODE_AFI | in==MODE_NMEA | in==MODE_FOG_PARAMETER ))
  {
    Serial.println("verify output fn: fail");
    Serial.println("output of function range, go to reset!");

    Serial1.println("verify output fn: fail");
    Serial1.println("output of function range, go to reset!");
    select_fn = SEL_RST;
    rst_fn_flag = MODE_RST; 
    value = STOP_SYNC;
  } 
  else {
    Serial.println("verify output fn: pass");
    Serial1.println("verify output fn: pass");
  }
}

void verify_output_fn_reg_value(int in)
{
  Serial.println("\nverifying output fn reg value........" );
  Serial.print("Input reg value: ");

  Serial1.println("\nverifying output fn reg value........" );
  Serial1.print("Input reg value: ");

  Serial.println(in);
  if(!( in==INT_SYNC |in==EXT_SYNC | in==STOP_SYNC | in==NMEA_MODE |
    in==HP_TEST ))
    {
      Serial.println("verify output fn reg value: fail");
      Serial.println("output of reg value range, go to reset!");

      Serial1.println("verify output fn reg value: fail");
      Serial1.println("output of reg value range, go to reset!");

      select_fn = SEL_RST;
      rst_fn_flag = MODE_RST; 
      value = STOP_SYNC;
    } 
  else {
    Serial.println("verify output fn: pass");
    Serial1.println("verify output fn: pass");
  }
}

void verify_select_fn(int in)
{
  Serial.println("\nverifying select fn........" );
  Serial.print("Input select_fn: ");
  Serial.println(in);

  Serial1.println("\nverifying select fn........" );
  Serial1.print("Input select_fn: ");
  Serial1.println(in);
  if(!( in==SEL_DEFAULT |in==SEL_RST | in==SEL_FOG_1 | in==SEL_FOG_2 | in==SEL_AFI |
    in==SEL_FOG_3 |in==SEL_IMU |in==SEL_NMEA |in==SEL_FOG_PARA |in==SEL_HP_TEST))
    {
      Serial.println("verify select_fn: fail");
      Serial.println("output of select_fn range, go to reset!");

      Serial1.println("verify select_fn: fail");
      Serial1.println("output of select_fn range, go to reset!");
      select_fn = SEL_RST;
      rst_fn_flag = MODE_RST; 
      value = STOP_SYNC;
    } 
  else {
    Serial.println("verify select_fn: pass");
    Serial1.println("verify select_fn: pass");
  }
}