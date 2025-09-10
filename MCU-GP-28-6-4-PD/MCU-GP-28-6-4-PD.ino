#include "gp1z.h"


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
/*** MCU LED***/
#define MCU_LED A2


/*** Attitude calculation*/
Navigation::ComplementaryFilter my_cpf;

unsigned short count = 0;
unsigned long pre_time = 0;
/***End of Attitude calculation*/

/*** global var***/
// int pin_scl_mux = 17;
// bool trig_status[2] = {0, 0};
unsigned int t_new, t_old=0;

my_time_t mcu_time;
my_float_t my_f;
my_misalignment_cali_t misalignment_cali_coe;
my_attitude_cali_t attitude_cali_coe;

unsigned char fog_op_status, rst_fog_para_flag=0;


/*** serial data from PC***/
byte rx_cnt = 0, cmd;
extern byte fog_ch;
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
const unsigned char MARS_PD_TEMP[4] = {0x00, 0x00, 0x00, 0x00};
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

// attitude calculation filter
Madgwick ahrs_attitude;

int WDT_CNT=0;
int tt0=0, tt1, tt2, tt3;
unsigned long data_cnt = 0;

unsigned int MCU_cnt = 0;


byte *reg_fog;
byte *t_reg_fog;

unsigned int t_previous = 0;
unsigned int t_start;

// auto rst fn flag
byte rst_fn_flag = MODE_RST;

DumpParameter my_cali_para[PARAMETER_CNT];

void my_parameter_f(const char *parameter_name, float input_value, DumpParameter *output_data) 
{
  snprintf(output_data->str, MAX_STR_LENGTH, "\"%s\":%.10f", parameter_name, input_value);
  Serial.println(output_data->str);
  Serial1.print(output_data->str);
}

void my_parameter_f_silent(const char *parameter_name, float input_value, DumpParameter *output_data) 
{
  snprintf(output_data->str, MAX_STR_LENGTH, "\"%s\":%.10f", parameter_name, input_value);
}


void setup() {
  // delay(5000);

  // set_system_clk(EXTERNAL_CRYSTAL);
  // Blink_MCU_LED();

  //attitude calculation 
  ahrs_attitude.begin(100.0f); // sample rate
  // 旋轉至NED座標系
  const float Rcs[9] = { 0,-1,0,  -1,0,0,  0,0,-1 };
  // const float Rcs[9] = { 1,0,0,  0,1,0,  0,0,1 };
  ahrs_attitude.setSensorToCaseMatrix(Rcs);

  // 設定 Local 世界座標為 NED：
  ahrs_attitude.setLocalFrameNED(true);

  ahrs_attitude.setGyroBiasLearning(true);
  ahrs_attitude.setDynamicAccelWeight(true);
  ahrs_attitude.setGimbalLockGuard(true, 89.0f);

  /*** pwm ***/
    pwm_init();
    myWDT_init(); //disable all WDT
  
    // EXTT
    /*** for IMU_V4  : EXTT = PA27, Variant pin = 26, EXINT[15]
     *   for PIG MCU : EXTT = PA27, Variant pin = 26, EXINT[15]
     *  ****/
  pinMode(26, INPUT_PULLUP);
  attachInterrupt(26, ISR_EXTT, CHANGE);

  // disableWDT();
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

  myUART_init();
  myI2C_init();
  mySPI_init();

  //Re-configure FPGA when system reset on MCU
  digitalWrite(nCONFIG, LOW);
  delay(50);
  digitalWrite(nCONFIG, HIGH);
  delay(500);

  byte FPGA_wakeup_flag = 0; 
  // Wait_FPGA_Wakeup(FPGA_wakeup_flag, 2);
  Blink_MCU_LED();

  rescue_mode();
  parameter_init();
  Blink_MCU_LED();
	
  IMU.init(); //setting MEMS IMU parameters 
  
    
	/*** var initialization***/
	mux_flag = MUX_ESCAPE; 
	select_fn = SEL_DEFAULT; 	
	run_fog_flag = 0;
	output_fn = temp_idle;

  /***read eeprom current status*/
  eeprom.Read(EEPROM_ADDR_FOG_STATUS, &fog_op_status);

  Serial.print("fog_op_status: ");
  Serial.println(fog_op_status);
 if(fog_op_status==1) // disconnected last time, send cmd again
  {
    Serial.println("\n-------AUTO RST---------");

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

    fog_ch = 2;
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

      case CMD_CONFI_CLK: {
        if(value != EEPROM_SYSCLK){
          Serial.println("system clock selection changed!");
          print_clock_configuration(value);
          write_fog_parameter_to_eeprom(EEPROM_SYSCLK, EEPROM_ADDR_CLOCK, value);
        }
      break;}

      case CMD_EXT_WDT: {
        if(value != EEPROM_EXTWDT){
          Serial.println("system clock selection changed!");
          msg_out("EXT WDT setup changed: ");
          print_ext_WDT_configuration(value);
          write_fog_parameter_to_eeprom(EEPROM_EXTWDT, EEPROM_ADDR_EXTWDT, value);
        }
      break;}

      case CMD_RST_FOG: {
        if(value == 0xFF) {
          msg_out("Sure to reset fog to initial value? ");
          msg_out("don't forget to save current parameters.");
          rst_fog_para_flag = 1;
        }
        else if(value == 0xFE) {
          if(rst_fog_para_flag == 1){
            msg_out("resetting fog parameters, please wait...");
            write_fog_parameter_to_eeprom_all(2);
            update_fpga_fog_parameter_init(100, 2);
            update_imu_misalignment_init();
            rst_fog_para_flag = 0;
          }
          else{
            msg_out("Resetting fog parameters process invalid, ");
            msg_out("request first.");
          }
        }
        else{
          msg_out("Resetting fog parameters error.");
        }
        break;
      }
      case CMD_THRESHOLD_WX: {
        if(value != EEPROM_THRESHOLD_WX){
          Serial.println("THRESHOLD_WX changed!");
          write_fog_parameter_to_eeprom(EEPROM_THRESHOLD_WX, EEPROM_ADDR_THRESHOLD_WX, value);
          attitude_cali_coe._d.std_wx = EEPROM_THRESHOLD_WX;
        }
      break;}

      case CMD_THRESHOLD_WY: {
        if(value != EEPROM_THRESHOLD_WY){
          Serial.println("THRESHOLD_WY changed!");
          write_fog_parameter_to_eeprom(EEPROM_THRESHOLD_WY, EEPROM_ADDR_THRESHOLD_WY, value);
          attitude_cali_coe._d.std_wy = EEPROM_THRESHOLD_WY;
        }
      break;}

      case CMD_THRESHOLD_WZ: {
        if(value != EEPROM_THRESHOLD_WZ){
          Serial.println("THRESHOLD_WZ changed!");
          write_fog_parameter_to_eeprom(EEPROM_THRESHOLD_WZ, EEPROM_ADDR_THRESHOLD_WZ, value);
          attitude_cali_coe._d.std_wz = EEPROM_THRESHOLD_WZ;
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

      /***
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
       */
      case CMD_DUMP_PARAMETERS: {
        String fog_parameter;
        for(int i=0; i<255; i++) SER->read();//clear serial buffer
        sp->updateParameter(myCmd_header, FPGA_DUMP_PARAMETERS_ADDR, myCmd_trailer, value, 0xCC);
        while(!SER->available()){delay(1);};

        if (SER->available()) {
          fog_parameter = SER->readStringUntil('\n');
          fog_parameter.trim();  // 去掉空白與換行

          // 確保 JSON 合法（開始是 {，結束是 }）
          if (fog_parameter.startsWith("{") && fog_parameter.endsWith("}")) {
              // 移除結尾 '}'
              fog_parameter.remove(fog_parameter.length() - 1);
          }

          // 加入你要的參數
          DumpParameter my_atti_para[3];
          my_parameter_f_silent("STD_Wx", attitude_cali_coe._f.std_wx, &my_atti_para[0]);
          my_parameter_f_silent("STD_Wy", attitude_cali_coe._f.std_wy, &my_atti_para[1]);
          my_parameter_f_silent("STD_Wz", attitude_cali_coe._f.std_wz, &my_atti_para[2]);

          // 加上逗號與三個參數
          fog_parameter += ",";
          fog_parameter += my_atti_para[0].str;
          fog_parameter += ",";
          fog_parameter += my_atti_para[1].str;
          fog_parameter += ",";
          fog_parameter += my_atti_para[2].str;
          fog_parameter += "}";  // 關閉 JSON

          // 輸出到 Serial 與 Serial1
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
      case MODE_ATT_NMEA: {
				output_fn = acq_att_nmea;
				select_fn = SEL_ATT_NMEA;
        rst_fn_flag = MODE_ATT_NMEA;
				break;
            }
      case MODE_FOG_PARAMETER: {
          output_fn = acq_fog_parameter;
          select_fn = SEL_FOG_PARA;
          rst_fn_flag = MODE_FOG_PARAMETER;
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
      case MODE_ATT_NMEA: {
				output_fn = acq_att_nmea;
				break;
            }
      case MODE_FOG_PARAMETER: {
          output_fn = acq_fog_parameter;
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
    // Serial.println("IDLE");
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


void acq_fog_parameter(byte &select_fn, unsigned int value, byte ch)
{
  byte *fog;
	uint8_t CRC32[4];
	
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
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);
      break;

      case EXT_SYNC:
        data_cnt = 0;
        Serial.println("Enter EXT_SYNC mode");
        Serial.println("Set EXTT to RISING");
        EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);
      break;

      case STOP_SYNC:
        reset_SYNC();
        data_cnt = 0;
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
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
  byte cali_fog[4]={0};
	uint8_t CRC32[4];
  my_float_t pd_temp;
  static my_float_t myfog_GYRO;
  static my_acc_t my_GYRO, my_GYRO_cali;
	
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
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);
      break;

      case STOP_SYNC:
        data_cnt = 0;
        reset_SYNC();
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
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
      
      if(fog) {
        reg_fog = fog;
        myfog_GYRO.bin_val[0] = reg_fog[11];
        myfog_GYRO.bin_val[1] = reg_fog[10];
        myfog_GYRO.bin_val[2] = reg_fog[9];
        myfog_GYRO.bin_val[3] = reg_fog[8];
        my_GYRO.float_val[0] = 0.0; 
        my_GYRO.float_val[1] = 0.0;
        my_GYRO.float_val[2] = myfog_GYRO.float_val;
      }

      if(ISR_PEDGE)
      {
        uint8_t* imu_data = (uint8_t*)malloc(16); // KVH_HEADER:4 + pig:14
        data_cnt++;
        mcu_time.ulong_val = millis() - t_previous;
        
        ISR_PEDGE = false;

        /*** ------mis-alignment calibration gyro raw data -----***/
        gyro_cali(my_GYRO_cali.float_val, my_GYRO.float_val); // Add this to align all modes in the same way to adjust the bias offset.
        cali_fog[0] = my_GYRO_cali.bin_val[11];
        cali_fog[1] = my_GYRO_cali.bin_val[10];
        cali_fog[2] = my_GYRO_cali.bin_val[9];
        cali_fog[3] = my_GYRO_cali.bin_val[8];

        memcpy(imu_data, KVH_HEADER, 4);
        memcpy(imu_data+4, cali_fog, 4); //fog
        memcpy(imu_data+8, reg_fog+12, 4);// PD temp
        memcpy(imu_data+12, mcu_time.bin_val, 4);
        myCRC.crc_32(imu_data, 16, CRC32);
        free(imu_data);

        #ifdef UART_RS422_CMD
        if(data_cnt >= DELAY_CNT)
        {
          Serial1.write(KVH_HEADER, 4);
          Serial1.write(cali_fog, 4);
          Serial1.write(reg_fog+12, 4);
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
  my_acc_t my_memsGYRO;
  my_float_t pd_temp;
  static my_float_t myfog_GYRO;
  static my_acc_t my_memsXLM, my_memsXLM_cali;
  static my_acc_t my_GYRO, my_GYRO_cali, my_att;
  static float yaw0 = 0.0f;
  // my_acc_t my_GYRO_case_frame, my_memsXLM_case_frame;

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
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);
        
      break;
      case EXT_SYNC:
        data_cnt = 0;
        Serial.println("Enter EXT_SYNC mode");
        Serial.println("Set EXTT to CHANGE");

        // float y = ahrs_attitude.getLocalCaseYaw(); 
        yaw0 = ahrs_attitude.getLocalCaseYaw();

        EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);
      break;

      case EXT_SYNC2:
        data_cnt = 0;
        Serial.println("Enter EXT_SYNC2 mode");
        Serial.println("Set EXTT to CHANGE");
        my_cpf.startLC();
        EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);
      break;

      case STOP_SYNC:
        reset_SYNC();
        data_cnt = 0;
        ahrs_attitude.resetAttitude(true);
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
        my_cpf.resetEuler(0,0,0);
        disableWDT();
        disable_EXT_WDT(EXT_WDT_EN);
      break;

      default:
      break;
    }
    t_previous = millis();
	}

	if(run_fog_flag) {
    fog = sp14.readData(header, sizeofheader, &try_cnt);
    if(fog) {
      reg_fog = fog;
      myfog_GYRO.bin_val[0] = reg_fog[11];
      myfog_GYRO.bin_val[1] = reg_fog[10];
      myfog_GYRO.bin_val[2] = reg_fog[9];
      myfog_GYRO.bin_val[3] = reg_fog[8];
    }

    if(ISR_PEDGE)
    {
      uint8_t* imu_data = (uint8_t*)malloc(36+12); // KVH_HEADER:4 + adxl355:9 + nano33_w:6 + nano33_a:6 + pig:14
      data_cnt++;
      mcu_time.ulong_val = millis() - t_previous;

      ISR_PEDGE = false;

      /*** get sensor raw data*/
      // IMU.Get_X_Axes_g_f(my_memsXLM.float_val);// get mems XLM data in m/s^2
      /*** ------get xlm raw data -----***/
      IMU.Get_X_Axes_f(my_memsXLM.float_val);// get mems XLM data in g
      /*** ------mis-alignment calibration xlm raw data -----***/
      acc_cali(my_memsXLM_cali.float_val, my_memsXLM.float_val);

      /*** ------get gyro raw data -----***/
      IMU.Get_G_Axes_f(my_memsGYRO.float_val);// get mems GYRO data in degree/s
      my_GYRO.float_val[0] = my_memsGYRO.float_val[0]; 
      my_GYRO.float_val[1] = my_memsGYRO.float_val[1];
      // my_GYRO.float_val[2] = my_memsGYRO.float_val[2];
      my_GYRO.float_val[2] = myfog_GYRO.float_val;
      // my_GYRO.float_val[2] = myfog_GYRO.float_val * DEG_TO_RAD;
      /*** ------mis-alignment calibration gyro raw data -----***/
      gyro_cali(my_GYRO_cali.float_val, my_GYRO.float_val);
      // my_GYRO_cali.float_val[0]*=-1; my_GYRO_cali.float_val[1]*=-1; my_GYRO_cali.float_val[2]*=-1;

      // --- 座標旋轉至輸出IMU顯示正確 --- 
      my_acc_t my_GYRO_case_frame, my_memsXLM_case_frame;
      rotate2NED(my_GYRO_case_frame.float_val, my_GYRO_cali.float_val);
      rotate2NED(my_memsXLM_case_frame.float_val, my_memsXLM_cali.float_val);

      print_imu_data(false, my_memsXLM_cali.float_val, my_GYRO_cali.float_val);

      memcpy(imu_data, KVH_HEADER, 4);
      // memcpy(imu_data+4, my_GYRO_cali.bin_val, 12);//wx, wy, wz
      // memcpy(imu_data+16, my_memsXLM_cali.bin_val, 12);//ax, ay, az
      memcpy(imu_data+4, my_GYRO_case_frame.bin_val, 12);//wx, wy, wz
      memcpy(imu_data+16, my_memsXLM_case_frame.bin_val, 12);//ax, ay, az
      // memcpy(imu_data+28, MARS_PD_TEMP, 4);// PD temp
      memcpy(imu_data+28, reg_fog+12, 4);// PD temp
      memcpy(imu_data+32, mcu_time.bin_val, 4);
      memcpy(imu_data+36, my_att.bin_val, 12);
      myCRC.crc_32(imu_data, 48, CRC32);

      free(imu_data);
      
      #ifdef UART_RS422_CMD
      if(data_cnt >= DELAY_CNT)
      {
        Serial1.write(KVH_HEADER, 4);
        // Serial1.write(my_GYRO_cali.bin_val, 12);   //wx, wy, wz
        // Serial1.write(my_memsXLM_cali.bin_val, 12);//ax, ay, az
        Serial1.write(my_GYRO_case_frame.bin_val, 12);   //wx, wy, wz
        Serial1.write(my_memsXLM_case_frame.bin_val, 12);//ax, ay, az
        // Serial1.write(MARS_PD_TEMP, 4);         // PD temp
        Serial1.write(reg_fog+12, 4);         // PD temp
        Serial1.write(mcu_time.bin_val, 4);
        Serial1.write(my_att.bin_val, 12);
        Serial1.write(CRC32, 4);
      }
      #endif  
      resetWDT(); 
      reset_EXT_WDI(WDI); 

      my_acc_t my_GYRO_att_calculate;
      if(abs(my_GYRO_cali.float_val[0]) > 0.38) my_GYRO_att_calculate.float_val[0] = my_GYRO_cali.float_val[0];
      else my_GYRO_att_calculate.float_val[0] = 0.0f;
      if(abs(my_GYRO_cali.float_val[1]) > 0.38) my_GYRO_att_calculate.float_val[1] = my_GYRO_cali.float_val[1];
      else my_GYRO_att_calculate.float_val[1] = 0.0f;
      if(abs(my_GYRO_cali.float_val[2]) > 0.025) my_GYRO_att_calculate.float_val[2] = my_GYRO_cali.float_val[2];
      else my_GYRO_att_calculate.float_val[2] = 0.0f;
      
      ahrs_attitude.updateIMU(my_GYRO_att_calculate.float_val[0], my_GYRO_att_calculate.float_val[1], my_GYRO_att_calculate.float_val[2],
         my_memsXLM_cali.float_val[0], my_memsXLM_cali.float_val[1], my_memsXLM_cali.float_val[2]);
      // ahrs_attitude.updateIMU(my_GYRO_cali.float_val[0], my_GYRO_cali.float_val[1], my_GYRO_cali.float_val[2],
      //    my_memsXLM_cali.float_val[0], my_memsXLM_cali.float_val[1], my_memsXLM_cali.float_val[2]);

      float r = ahrs_attitude.getLocalCaseRoll();
      float p = ahrs_attitude.getLocalCasePitch();
      float y = ahrs_attitude.getLocalCaseYaw();
      my_att.float_val[0] =  p; // pitch
      my_att.float_val[1] =  r; // roll
      my_att.float_val[2] =  wrapDeg(y - yaw0); // yaw 減去零位，並 wrap 到 [-180,180]
    }
	}
	clear_SEL_EN(select_fn);
}

void acq_att_nmea(byte &select_fn, unsigned int value, byte ch)
{
  my_acc_t my_memsGYRO;
  my_float_t pd_temp;
  static my_float_t myfog_GYRO;
  static my_acc_t my_memsXLM, my_memsXLM_cali;
  static my_acc_t my_GYRO, my_GYRO_cali, my_att;
  char nmeaSentence[50];

  byte *fog;
	uint8_t CRC32[4];

  if(select_fn&SEL_IMU)
	{
    Serial.print("Enter acq_att_nmea mode, channel: ");
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
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);
        
      break;
      case EXT_SYNC:
        data_cnt = 0;
        Serial.println("Enter EXT_SYNC mode");
        Serial.println("Set EXTT to CHANGE");

        EIC->CONFIG[1].bit.SENSE7 = 3; 
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);

      break;
      case EXT_SYNC2:
        data_cnt = 0;
        Serial.println("Enter EXT_SYNC2 mode");
        Serial.println("Set EXTT to CHANGE");
        my_cpf.startLC();
        EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);
      break;
      case STOP_SYNC:
        reset_SYNC();
        data_cnt = 0;
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
        my_cpf.resetEuler(0,0,0);
        disableWDT();
        disable_EXT_WDT(EXT_WDT_EN);
      break;

      default:
      break;
    }
    t_previous = millis();
	}

	if(run_fog_flag) {
    fog = sp14.readData(header, sizeofheader, &try_cnt);
    if(fog) {
      reg_fog = fog;
      myfog_GYRO.bin_val[0] = reg_fog[11];
      myfog_GYRO.bin_val[1] = reg_fog[10];
      myfog_GYRO.bin_val[2] = reg_fog[9];
      myfog_GYRO.bin_val[3] = reg_fog[8];
    }

    if(ISR_PEDGE)
    {
      data_cnt++;
      mcu_time.ulong_val = millis() - t_previous;

      ISR_PEDGE = false;

      /*** get sensor raw data*/
      // IMU.Get_X_Axes_g_f(my_memsXLM.float_val);// get mems XLM data in m/s^2
      /*** ------get xlm raw data -----***/
      IMU.Get_X_Axes_f(my_memsXLM.float_val);// get mems XLM data in g
      /*** ------mis-alignment calibration xlm raw data -----***/
      acc_cali(my_memsXLM_cali.float_val, my_memsXLM.float_val);

      /*** ------get gyro raw data -----***/
      // IMU.Get_G_Axes_rps_f(my_memsGYRO.float_val);// get mems GYRO data in radian/s
      // my_GYRO.float_val[0] = my_memsGYRO.float_val[0]; 
      // my_GYRO.float_val[1] = my_memsGYRO.float_val[1];
      // my_GYRO.float_val[2] = myfog_GYRO.float_val * DEG_TO_RAD;

      IMU.Get_G_Axes_f(my_memsGYRO.float_val);// get mems GYRO data in degree/s
      my_GYRO.float_val[0] = my_memsGYRO.float_val[0]; 
      my_GYRO.float_val[1] = my_memsGYRO.float_val[1];
      my_GYRO.float_val[2] = myfog_GYRO.float_val;

      /*** ------mis-alignment calibration gyro raw data -----***/
      gyro_cali(my_GYRO_cali.float_val, my_GYRO.float_val);

      // sprintf(nmeaSentence, "SEN,%06.2f,%+06.2f,%+07.2f", my_att.float_val[2], my_att.float_val[1], my_att.float_val[0]);
      sprintf(nmeaSentence, "SEN,%06.2f,%+06.2f,%+07.2f", 360 - my_att.float_val[2], my_att.float_val[0], my_att.float_val[1]);
      byte checksum = 0;
      for (int i = 0; i < strlen(nmeaSentence); i++) {
        checksum ^= nmeaSentence[i];
      }
      char nmeaOutput[70]; // 50 (nmeaSentence) + 3 ($, *, \r\n) + 2 (checksum) + 1 (null terminator)
      

      #ifdef UART_RS422_CMD
      if(data_cnt >= DELAY_CNT)
      {
        sprintf(nmeaOutput, "$%s*%02X\r\n", nmeaSentence, checksum);
        Serial1.print(nmeaOutput);
      }
      #endif  
      resetWDT(); 
      reset_EXT_WDI(WDI);

      my_cpf.run(float(mcu_time.ulong_val)*1e-3, my_GYRO_cali.float_val, my_memsXLM_cali.float_val);
      my_cpf.getEularAngle(my_att.float_val); //raw data -> att: pitch, roll, yaw 
      /*** set yaw value range */
      if(my_att.float_val[2] >= 360.0) my_att.float_val[2] -= 360.0;
      else if(my_att.float_val[2] < 0.0) my_att.float_val[2] += 360.0;
      /*** set pitch value range */

      /*** set roll value range */
    }
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
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);
      break;

      case EXT_SYNC:
        Serial.println("Enter EXT_SYNC mode");
        Serial.println("Set EXTT to RISING");
        data_cnt = 0;
        EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);
      break;

      case NMEA_MODE:
        Serial.println("Enter EXT_SYNC NMEA_MODE mode");
        Serial.println("Set EXTT to RISING");
        data_cnt = 0;
        EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        setupWDT(11);
        enable_EXT_WDT(EXT_WDT_EN);
        reset_EXT_WDI(WDI);
      break;

      case STOP_SYNC:
        reset_SYNC();
        EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
        data_cnt = 0;
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
       resetWDT();
       reset_EXT_WDI(WDI);
      }
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
        reset_SYNC();
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
        resetWDT();
      }
	    
      t_old = t_new;
      
        
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


void parameter_init(void)
{
  eeprom.Parameter_Read(EEPROM_ADDR_PARAMETER_EXIST,my_f.bin_val);
  EEPROM_Parameter_exist = my_f.int_val;

  /***fog parameter is empty,  write initial fog data*/
  if(EEPROM_Parameter_exist != EEPROM_PARAMETER_EXIST){
    eeprom.Parameter_Write(EEPROM_ADDR_PARAMETER_EXIST, EEPROM_PARAMETER_EXIST);
    Serial.println("\n*****EEPROM FOG parameter does not exist!********");

    /***output configuration*/
    Serial.println("\nStart setting output configuration.");
    write_fog_parameter_to_eeprom(EEPROM_BAUDRATE, EEPROM_ADDR_BAUDRATE, BAUDRATE_INIT);
    write_fog_parameter_to_eeprom(EEPROM_DATARATE, EEPROM_ADDR_DATARATE, DATARATE_INIT);
    write_fog_parameter_to_eeprom(  EEPROM_SYSCLK, EEPROM_ADDR_CLOCK,    SYSCLOCK_INIT);
    write_fog_parameter_to_eeprom(  EEPROM_EXTWDT, EEPROM_ADDR_EXTWDT,     EXTWDT_INIT);
    set_output_configuration_init();
    Serial.println("\nSetting output configuration done.");
    /***end of output configuration*/

    /*** IMU misalignment calibration*/
    update_imu_misalignment_init();
    /***end of IMU misalignment calibration*/

    /*** AHRS attitude calibration*/
    update_ahrs_attitude_cali_init();
    /***end of AHRS attitude calibration*/

    /***fog parameters*/
    Serial.println("\nStart writing initial fog data.");
    // eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0); // change to 0 12/18

    write_fog_parameter_to_eeprom_all(2);
    update_fpga_fog_parameter_init(100, 2);

    /***end of fog parameters*/

  }
  else{
    Serial.println("\n*****EEPROM FOG parameter exist!*****");
    /***fog parameters*/
    Serial.println("\nStart reading fog parameter from eeprom.");
    read_fog_parameter_from_eeprom_all(2);
    update_fpga_fog_parameter_init(100, 2);
    /***end of fog parameters*/

    /***output configuration*/
    Serial.println("\nStart reading output configuration from eeprom.");
    read_fog_parameter_from_eeprom(EEPROM_BAUDRATE, EEPROM_ADDR_BAUDRATE);
    read_fog_parameter_from_eeprom(EEPROM_DATARATE, EEPROM_ADDR_DATARATE);
    read_fog_parameter_from_eeprom(EEPROM_SYSCLK,   EEPROM_ADDR_CLOCK);
    read_fog_parameter_from_eeprom(EEPROM_EXTWDT,   EEPROM_ADDR_EXTWDT);
    report_current_output_configuration();
    set_output_configuration_init();
    /***end of output configuration*/

    /*** IMU misalignment calibration*/
    read_misalignment_calibration_from_eeprom();
    /***end of IMU misalignment calibration*/

    /*** AHRS attitude calibration*/
    read_ahrs_attitude_calibration_from_eeprom();
    /***end of AHRS attitude calibration*/

    set_ahrs_attitude_cali_init();

  }
}

void set_ahrs_attitude_cali_init()
{
  Serial.println("\nStart setting AHRS attitude calibration.");

  /*** End of Kalman Filter Initialize***/
  Serial.print("STD_Wx:");
  Serial.println(attitude_cali_coe._f.std_wx);
  Serial.print("STD_Wy:");
  Serial.println(attitude_cali_coe._f.std_wy);
  Serial.print("STD_Wz:");
  Serial.println(attitude_cali_coe._f.std_wz);

  /*** Kalman Filter Initialize ***/
  my_cpf.setIMUError(AR_1A_UY, 100);
  // my_cpf.setThresholdBySTD();
  my_cpf.setThreshold(attitude_cali_coe._f.std_wx, 
                      attitude_cali_coe._f.std_wy, 
                      attitude_cali_coe._f.std_wz);

  Serial.println("End of setting AHRS attitude calibration.");
}

void update_imu_misalignment_init()
{
  // Serial.println("\nStart setting IMU misalignment calibration.");
  msg_out("\nStart setting IMU misalignment calibration.");
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
  // Serial.println("\nSetting IMU misalignment calibration done.");
  msg_out("\nSetting IMU misalignment calibration done.");
}

void update_ahrs_attitude_cali_init()
{
  msg_out("\nStart setting ahrs cali.");
  write_fog_parameter_to_eeprom(EEPROM_THRESHOLD_WX, EEPROM_ADDR_THRESHOLD_WX, ATTI_THRESHOLD_WX_INIT);
  write_fog_parameter_to_eeprom(EEPROM_THRESHOLD_WY, EEPROM_ADDR_THRESHOLD_WY, ATTI_THRESHOLD_WY_INIT);
  write_fog_parameter_to_eeprom(EEPROM_THRESHOLD_WZ, EEPROM_ADDR_THRESHOLD_WZ, ATTI_THRESHOLD_WZ_INIT);

  attitude_cali_coe._d.std_wx = ATTI_THRESHOLD_WX_INIT;
  attitude_cali_coe._d.std_wy = ATTI_THRESHOLD_WY_INIT;
  attitude_cali_coe._d.std_wz = ATTI_THRESHOLD_WZ_INIT;

  msg_out("\nSetting ahrs cali done.");
}

void report_current_output_configuration()
{
  Serial1.begin(9600);
  Serial.println("report_current_output_configuration");
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
    case SET_DATARATE_400: {
      // pwm.timer(1, 2, int(15000*PWM_FIX), false); //12M/2/15000 = 400Hz
      // pwm.analogWrite(PWM100, 500);  
      
      Serial.println("Data rate set to 400 Hz");
      Serial1.println("Data rate set to 400 Hz");
      // delay(100);
      break;
    }
    case SET_DATARATE_200: {
      // pwm.timer(1, 2, int(30000*PWM_FIX), false); //12M/2/30000 = 200Hz
      // pwm.analogWrite(PWM100, 500);  

      Serial.println("Data rate set to 200 Hz");
      Serial1.println("Data rate set to 200 Hz");
      // delay(100);
      break;
    }
    case SET_DATARATE_100: {
      // pwm.timer(1, 2, int(60000*PWM_FIX), false); //12M/2/60000 = 100Hz
      // pwm.analogWrite(PWM100, 500);  

      Serial.println("Data rate set to 100 Hz");
      Serial1.println("Data rate set to 100 Hz");
      // delay(100);
      break;
    }
    case SET_DATARATE_10: {
      // pwm.timer(1, 2, int(600000*PWM_FIX), false); //12M/2/600000 = 10Hz
      // pwm.analogWrite(PWM100, 500);  

      Serial.println("Data rate set to 10 Hz");
      Serial1.println("Data rate set to 10 Hz");
      // delay(100);
      break;
    }
    default:{
      // pwm.timer(1, 2, int(60000*PWM_FIX), false); //12M/2/60000 = 100Hz
      // pwm.analogWrite(PWM100, 500);  

      Serial.println("Data rate set to 100 Hz");
      Serial1.println("Data rate set to 100 Hz");
      // delay(100);
      break;
    }
  }

print_clock_configuration(EEPROM_SYSCLK);
print_ext_WDT_configuration(EEPROM_EXTWDT);
}


void write_fog_parameter_to_eeprom_all(byte fog_ch)
{
  eeprom_obj *eeprom_obj_ptr;

  if(fog_ch==1) eeprom_obj_ptr = &eeprom_z;
  else if(fog_ch==2) eeprom_obj_ptr = &eeprom_x;
  else if(fog_ch==3) eeprom_obj_ptr = &eeprom_y;
  msg_out("write_fog_parameter_to_eeprom_all, ch");
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
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF0, eeprom_obj_ptr->EEPROM_ADDR_SF_0, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF1, eeprom_obj_ptr->EEPROM_ADDR_SF_1, SF_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF2, eeprom_obj_ptr->EEPROM_ADDR_SF_2, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF3, eeprom_obj_ptr->EEPROM_ADDR_SF_3, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF4, eeprom_obj_ptr->EEPROM_ADDR_SF_4, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF5, eeprom_obj_ptr->EEPROM_ADDR_SF_5, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF6, eeprom_obj_ptr->EEPROM_ADDR_SF_6, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF7, eeprom_obj_ptr->EEPROM_ADDR_SF_7, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF8, eeprom_obj_ptr-> EEPROM_ADDR_SF_8, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF9, eeprom_obj_ptr->EEPROM_ADDR_SF_9, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SFB, eeprom_obj_ptr->EEPROM_ADDR_SFB, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_CUTOFF, eeprom_obj_ptr->EEPROM_ADDR_CUTOFF, CUTOFF_INIT);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_TMIN, eeprom_obj_ptr->EEPROM_ADDR_TMIN, MINUS20);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_TMAX, eeprom_obj_ptr->EEPROM_ADDR_TMAX, PLUS80);
  // BIAS temp. compensation
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_BIAS_COMP_T1, eeprom_obj_ptr->EEPROM_ADDR_BIAS_COMP_T1, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_BIAS_COMP_T2, eeprom_obj_ptr->EEPROM_ADDR_BIAS_COMP_T2, FLOAT_40);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SFB_1_SLOPE, eeprom_obj_ptr->EEPROM_ADDR_SFB_1_SLOPE, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SFB_1_OFFSET, eeprom_obj_ptr->EEPROM_ADDR_SFB_1_OFFSET, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SFB_2_SLOPE, eeprom_obj_ptr->EEPROM_ADDR_SFB_2_SLOPE, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SFB_2_OFFSET, eeprom_obj_ptr->EEPROM_ADDR_SFB_2_OFFSET, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SFB_3_SLOPE, eeprom_obj_ptr->EEPROM_ADDR_SFB_3_SLOPE, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SFB_3_OFFSET, eeprom_obj_ptr->EEPROM_ADDR_SFB_3_OFFSET, 0);

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
  // Serial.println("\nread_misalignment_calibration_from_eeprom");
  msg_out("\nread_misalignment_calibration_from_eeprom");
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
  // Serial.println("read_misalignment_calibration_from_eeprom done");
  msg_out("read_misalignment_calibration_from_eeprom done");
}

void read_ahrs_attitude_calibration_from_eeprom()
{
  // Serial.println("\nread_misalignment_calibration_from_eeprom");
  msg_out("\nread_ahrs_attitude_calibration_from_eeprom");
  read_fog_parameter_from_eeprom(EEPROM_THRESHOLD_WX, EEPROM_ADDR_THRESHOLD_WX);
  read_fog_parameter_from_eeprom(EEPROM_THRESHOLD_WY, EEPROM_ADDR_THRESHOLD_WY);
  read_fog_parameter_from_eeprom(EEPROM_THRESHOLD_WZ, EEPROM_ADDR_THRESHOLD_WZ);
  attitude_cali_coe._d.std_wx = EEPROM_THRESHOLD_WX;
  attitude_cali_coe._d.std_wy = EEPROM_THRESHOLD_WY;
  attitude_cali_coe._d.std_wz = EEPROM_THRESHOLD_WZ;
  Serial.print("STD_Wx:");
  Serial.println(attitude_cali_coe._f.std_wx);
  Serial.print("STD_Wy:");
  Serial.println(attitude_cali_coe._f.std_wy);
  Serial.print("STD_Wz:");
  Serial.println(attitude_cali_coe._f.std_wz);
  // Serial.println("read_misalignment_calibration_from_eeprom done");
  msg_out("read_ahrs_attitude_calibration_from_eeprom done");
}

void read_fog_parameter_from_eeprom_all(byte fog_ch)
{
  eeprom_obj *eeprom_obj_ptr;

  if(fog_ch==1) eeprom_obj_ptr = &eeprom_z;
  else if(fog_ch==2) eeprom_obj_ptr = &eeprom_x;
  else if(fog_ch==3) eeprom_obj_ptr = &eeprom_y;

  msg_out("read_fog_parameter_from_eeprom_all, ch");
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
  Serial.println("update_datarate");
  switch(eeprom_var)
  {
    case SET_DATARATE_400: {
      pwm.timer(1, 2, int(15000), false); //12M/2/15000 = 400Hz
      pwm.analogWrite(PWM100, 500);  
      Serial.println("Data rate set to 400 Hz");
      Serial1.println("Data rate set to 400 Hz");
      delay(100);
      break;
    }
    case SET_DATARATE_200: {
      pwm.timer(1, 2, int(30000), false); //12M/2/30000 = 200Hz
      pwm.analogWrite(PWM100, 500);  
      Serial.println("Data rate set to 200 Hz");
      Serial1.println("Data rate set to 200 Hz");
      delay(100);
      break;
    }
    case SET_DATARATE_100: {
      pwm.timer(1, 2, int(60000), false); //12M/2/60000 = 100Hz
      pwm.analogWrite(PWM100, 500);  
      Serial.println("Data rate set to 100 Hz");
      Serial1.println("Data rate set to 100 Hz");
      delay(100);
      break;
    }
    case SET_DATARATE_10: {
      pwm.timer(1, 2, int(600000), false); //12M/2/600000 = 10Hz
      // pwm.timer(1, 2, int(375000), false); //12M/2/375000 = 16Hz
      pwm.analogWrite(PWM100, 500);  
      Serial.println("Data rate set to 10 Hz");
      Serial1.println("Data rate set to 10 Hz");
      delay(100);
      break;
    }
    default:{
      pwm.timer(1, 2, int(60000), false); //12M/2/60000 = 100Hz
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
  msg_out("\nSetting output configuration!");
  update_baudrate(EEPROM_BAUDRATE);
  update_datarate(EEPROM_DATARATE);
  set_system_clk(EEPROM_SYSCLK);
  // print_ext_WDT_configuration(EEPROM_EXTWDT);
  msg_out("Setting output configuration done");
}


void update_fpga_fog_parameter_init(int dly_time, unsigned char fog_ch)
{
  // Serial.print("Setting SP initail parameters, ch");
  // Serial.println(fog_ch);
  msg_out("Updating FOG initail parameters, ch");
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

  // Serial.println("Setting SP parameters done");
  msg_out("Setting SP parameters done");
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


void Wait_FPGA_Wakeup(byte &flag, byte fog_ch)
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
  else if(fog_ch==3){
    sp = &sp9;
    SER = &Serial4;
  } 

  for(int i=0; i<255; i++) SER->read();//clear serial buffer
  sp->updateParameter(myCmd_header, FPGA_WAKEUP_ADDR, myCmd_trailer, 5, 0xCC);
  delay(10);
  flag = (SER->readStringUntil('\n'))[0];
  int t0=millis();
  while(!flag){

    if((millis()-t0)>500){
      times++;
      for(int i=0; i<255; i++) SER->read();//clear serial buffer
      sp->updateParameter(myCmd_header, FPGA_WAKEUP_ADDR, myCmd_trailer, 5, 0xCC);
      delay(10);
      // Serial.print("flag: ");
      // Serial.println(flag);
      // Serial.print("fog_woke_flag: ");
      // Serial.println(fog_woke_flag);
      flag = (SER->readStringUntil('\n'))[0];
      if(fog_woke_flag) flag = 1; //manual wake up
      // Serial.print("FPGA Sleeping: ");
      // Serial1.print("FPGA Sleeping: ");
      msg_out("FPGA Sleeping: ");
      Serial.println(times);
      Serial1.println(times);
      t0 = millis();
    }
  } 
}

void reset_SYNC()
{
  sync_status = 0;
  digitalWrite(PIG_SYNC, sync_status);
}

 void printVersion()
  {
    Serial.print("\nVersion:");
    Serial.println(MCU_VERSION);
  }

void verify_output_fn(byte in)
{
  Serial.println("\nverifying output fn........" );
  Serial.print("Input function index: ");
  Serial.println(in);
  if(!( in==MODE_RST |in==MODE_FOG | in==MODE_IMU | in==MODE_FOG_HP_TEST |
    in==MODE_NMEA | in==MODE_FOG_PARAMETER | in==MODE_ATT_NMEA ))
  {
    Serial.println("verify output fn: fail");
    Serial.println("output of function range, go to reset!");
    select_fn = SEL_RST;
    rst_fn_flag = MODE_RST; 
    value = STOP_SYNC;
  } 
  else {
    Serial.println("verify output fn: pass");
  }
}

void verify_output_fn_reg_value(int in)
{
  Serial.println("\nverifying output fn reg value........" );
  Serial.print("Input reg value: ");
  Serial.println(in);
  if(!( in==INT_SYNC |in==EXT_SYNC | in==STOP_SYNC | in==NMEA_MODE |
    in==HP_TEST ))
    {
      Serial.println("verify output fn reg value: fail");
      Serial.println("output of reg value range, go to reset!");
      select_fn = SEL_RST;
      rst_fn_flag = MODE_RST; 
      value = STOP_SYNC;
    } 
  else {
    Serial.println("verify output fn: pass");
  }
}

void verify_select_fn(int in)
{
  Serial.println("\nverifying select fn........" );
  Serial.print("Input select_fn: ");
  Serial.println(in);
  if(!( in==SEL_DEFAULT |in==SEL_RST | in==SEL_FOG_1 | in==SEL_FOG_2 | in==SEL_ATT_NMEA |
    in==SEL_FOG_3 |in==SEL_IMU |in==SEL_NMEA |in==SEL_FOG_PARA |in==SEL_HP_TEST ))
    {
      Serial.println("verify select_fn: fail");
      Serial.println("output of select_fn range, go to reset!");
      select_fn = SEL_RST;
      rst_fn_flag = MODE_RST; 
      value = STOP_SYNC;
    } 
  else {
    Serial.println("verify select_fn: pass");
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

void gyro_cali(float gyro_cli[3], float gyro[3])
{
  gyro_cli[0] = misalignment_cali_coe._f.g11 * gyro[0] + 
                misalignment_cali_coe._f.g12 * gyro[1] + 
                misalignment_cali_coe._f.g13 * gyro[2] + 
                misalignment_cali_coe._f.gx;
  gyro_cli[1] = misalignment_cali_coe._f.g21 * gyro[0] + 
                misalignment_cali_coe._f.g22 * gyro[1] + 
                misalignment_cali_coe._f.g23 * gyro[2] + 
                misalignment_cali_coe._f.gy;
  gyro_cli[2] = misalignment_cali_coe._f.g31 * gyro[0] + 
                misalignment_cali_coe._f.g32 * gyro[1] + 
                misalignment_cali_coe._f.g33 * gyro[2] + 
                misalignment_cali_coe._f.gz;
} 

void print_imu_data(bool on, float acc[3], float gyro[3])
{
  if(on) 
  { 
    Serial.print(acc[0]);
    Serial.print(", ");
    Serial.print(acc[1]);
    Serial.print(", ");
    Serial.print(acc[2]);
    Serial.print(", ");
    Serial.print(gyro[0]);
    Serial.print(", ");
    Serial.print(gyro[1]);
    Serial.print(", ");
    Serial.println(gyro[2]);
  }
}

void print_ext_WDT_configuration(int sel)
{
    if(sel == 0){
        msg_out("disable external WDT");
    }
    else if(sel == 1){
        msg_out("enable external WDT");
    }
    else{
        msg_out("input out of range");
    } 
}

static inline void rotate2NED(float v_new[3], float v_old[3]) {
  v_new[0] = -v_old[1];  // X -> -Y
  v_new[1] = -v_old[0];  // Y -> -X
  v_new[2] = -v_old[2];   // Z -> -Z

  // v_new[0] = v_old[0];  
  // v_new[1] = v_old[1];  
  // v_new[2] = v_old[2];   
}

static inline float wrapDeg(float a){
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}