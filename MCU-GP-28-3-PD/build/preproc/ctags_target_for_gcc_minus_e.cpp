# 1 "C:\\Users\\user\\Documents\\Arduino\\MCU-GP-28-3-PD\\MCU-GP-28-3-PD.ino"
# 2 "C:\\Users\\user\\Documents\\Arduino\\MCU-GP-28-3-PD\\MCU-GP-28-3-PD.ino" 2


/***

SERCOM0: I2C     (PA08, PA09) [sda, scl]

SERCOM1: serial3 (PA17, PA18) [rx, tx]

SERCOM2: serial2 (PA15, PA14) [rx, tx]

SERCOM3: serial4 (PA21, PA20) [rx, tx]

SERCOM4: SPI     (PB10, PB11, PA12, PA13) [ss, miso, mosi, sck]

SERCOM5: serial1 (PB23, PB22) [rx, tx]

  

***/
# 13 "C:\\Users\\user\\Documents\\Arduino\\MCU-GP-28-3-PD\\MCU-GP-28-3-PD.ino"
// interrupt for EXT_SYNC to FPGA

// RST to FPGA nConfig

/*** MCU LED***/



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

int WDT_CNT=0;
int tt0=0, tt1, tt2, tt3;
unsigned long data_cnt = 0;

unsigned int MCU_cnt = 0;


byte *reg_fog;
byte *t_reg_fog;

unsigned int t_previous = 0;
unsigned int t_start;

// auto rst fn flag
byte rst_fn_flag = 0;

DumpParameter my_cali_para[24];

void my_parameter_f(const char *parameter_name, float input_value, DumpParameter *output_data)
{
  snprintf(output_data->str, 20, "\"%s\":%.10f", parameter_name, input_value);
  SerialUSB.println(output_data->str);
  Serial1.print(output_data->str);
}

void setup() {
  // delay(5000);

  // set_system_clk(EXTERNAL_CRYSTAL);
  // Blink_MCU_LED();

  /*** pwm ***/
    pwm_init();
    myWDT_init(); //disable all WDT

    // EXTT
    /*** for IMU_V4  : EXTT = PA27, Variant pin = 26, EXINT[15]

     *   for PIG MCU : EXTT = PA27, Variant pin = 26, EXINT[15]

     *  ****/
# 115 "C:\\Users\\user\\Documents\\Arduino\\MCU-GP-28-3-PD\\MCU-GP-28-3-PD.ino"
  pinMode(26, INPUT_PULLUP);
  attachInterrupt(26, ISR_EXTT, CHANGE);

  // disableWDT();
// set interrupt mode to None
  /***----- for PIG MCU & IMU_V4 EXINT[15]----- ***/
  ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 0; // set ISR no NONE



  /*** ADC setting***/
  analogReadResolution(12); //set resolution
  pinMode(6, INPUT);

  pinMode(29 /*PA22*/, OUTPUT);
  digitalWrite(29 /*PA22*/, sync_status);

  pinMode(A2, OUTPUT);
  digitalWrite(A2, HIGH);

  pinMode(12, OUTPUT);

  myUART_init();
  myI2C_init();
  mySPI_init();

  //Re-configure FPGA when system reset on MCU
  digitalWrite(12, LOW);
  delay(50);
  digitalWrite(12, HIGH);
  delay(500);

  byte FPGA_wakeup_flag = 0;
  Wait_FPGA_Wakeup(FPGA_wakeup_flag, 2);
  Blink_MCU_LED();

  rescue_mode();
  parameter_init();
  Blink_MCU_LED();

  IMU.init(); //setting MEMS IMU parameters 


 /*** var initialization***/
 mux_flag = 2;
 select_fn = 0;
 run_fog_flag = 0;
 output_fn = temp_idle;

  /***read eeprom current status*/
  eeprom.Read(4095, &fog_op_status);

  SerialUSB.print("fog_op_status: ");
  SerialUSB.println(fog_op_status);
 if(fog_op_status==1) // disconnected last time, send cmd again
  {
    SerialUSB.println("\n-------AUTO RST---------");

    eeprom.Parameter_Read(4090, my_f.bin_val);// read output function index from eeprom
    rst_fn_flag = my_f.int_val;
    // rst_fn_flag = 10; //test output fn output of range 
    verify_output_fn(rst_fn_flag);// check if output function is valid
    SerialUSB.println("\nprint OUTPUT_MODE"); if (rst_fn_flag == 0) { SerialUSB.println("---->>>MODE_RST"); } else if (rst_fn_flag == 1) { SerialUSB.println("---->>>MODE_FOG"); } else if (rst_fn_flag == 2) { SerialUSB.println("---->>>MODE_IMU"); } else if (rst_fn_flag == 3) { SerialUSB.println("---->>>MODE_FOG_HP_TEST"); } else if (rst_fn_flag == 4) { SerialUSB.println("---->>>MODE_NMEA"); } else if (rst_fn_flag == 5) { SerialUSB.println("---->>>MODE_ATT_NMEA"); } else if (rst_fn_flag == 6) { SerialUSB.println("---->>>MODE_FOG_PARAMETER"); }else { SerialUSB.println("---->>>OUTPUT_MODE out of range!"); };

    eeprom.Parameter_Read(4089, my_f.bin_val);//read reg value of output function
    uart_value = my_f.int_val;
    // value = 10; //test fn reg output of range 
    verify_output_fn_reg_value(uart_value);
    SerialUSB.println("\nprint OUTPUT_REG"); switch(uart_value) { case 1: SerialUSB.println("---->>>INT_SYNC"); break; case 2: SerialUSB.println("---->>>EXT_SYNC"); break; case 4: SerialUSB.println("---->>>STOP_SYNC"); break; case 5: SerialUSB.println("---->>>NMEA_MODE"); break; case 3: SerialUSB.println("---->>>HP_TEST"); break; default: SerialUSB.println("---->>>Reg value out of range!"); break; };

    eeprom.Parameter_Read(4091, my_f.bin_val);
    select_fn = my_f.int_val;
    // select_fn = 10; //test select_fn output of range 
    verify_select_fn(select_fn);
    SerialUSB.println("\nprint SELECT_FN"); if (select_fn == 0) { SerialUSB.println("---->>>SELECT SEL_DEFAULT"); } else if (select_fn == 1) { SerialUSB.println("---->>>SELECT SEL_RST"); } else if (select_fn == 2) { SerialUSB.println("---->>>SELECT SEL_FOG_1"); } else if (select_fn == 3) { SerialUSB.println("---->>>SELECT SEL_FOG_2"); } else if (select_fn == 4) { SerialUSB.println("---->>>SELECT SEL_FOG_3"); } else if (select_fn == 5) { SerialUSB.println("---->>>SELECT SEL_IMU"); } else if (select_fn == 6) { SerialUSB.println("---->>>SELECT SEL_NMEA"); } else if (select_fn == 9) { SerialUSB.println("---->>>SELECT SEL_ATT_NMEA"); } else if (select_fn == 7) { SerialUSB.println("---->>>SELECT SEL_FOG_PARA"); } else if (select_fn == 8) { SerialUSB.println("---->>>SELECT SEL_HP_TEST"); } else { SerialUSB.println("---->>>SELECT_FN out of range!"); };

    fog_ch = 2;
  }
  SerialUSB.println("\nprint mux_flag"); if(mux_flag == 0) SerialUSB.println("---->>>MUX_OUTPUT"); else if(mux_flag == 1) SerialUSB.println("---->>>MUX_PARAMETER"); else if(mux_flag == 2) SerialUSB.println("---->>>MUX_ESCAPE");;
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
 SerialUSB.print(name);
 SerialUSB.print(": ");
 SerialUSB.println((unsigned int)addr, 16);
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
 SerialUSB.print(t_new - t_old);
 SerialUSB.print('\t');
 SerialUSB.print((float)accX*0.0000156);
 SerialUSB.print('\t');
 SerialUSB.print((float)accY*0.0000156);
 SerialUSB.print('\t');
 SerialUSB.println((float)accZ*0.0000156);
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
 SerialUSB.print(name);
 SerialUSB.print(": ");
 SerialUSB.println(val);
}

void printVal_0(char name[])
{
 SerialUSB.println(name);
}

void cmd_mux(bool &cmd_complete, byte cmd, byte &mux_flag)
{
 if(cmd_complete)
 {
  cmd_complete = 0;
  if(cmd >7) mux_flag = 1;
  else mux_flag = 0;
 }
}

void parameter_setting(byte &mux_flag, byte cmd, int value, byte fog_ch)
{
 if(mux_flag == 1)
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

  mux_flag = 2;
  switch(cmd) {
      SerialUSB.print("ch: ");
      SerialUSB.println(fog_ch);
      case 8: {
        if(value != eeprom_ptr->EEPROM_Mod_freq){
          SerialUSB.println("FOG_MOD_FREQ changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Mod_freq, eeprom_ptr->EEPROM_ADDR_MOD_FREQ, value);
          sp->updateParameter(myCmd_header, 0, myCmd_trailer, eeprom_ptr->EEPROM_Mod_freq, 0xCC);
        }
        SerialUSB.print(fog_ch);
        SerialUSB.print(", ");
        SerialUSB.println(eeprom_ptr->EEPROM_ADDR_MOD_FREQ, 16);
        break;}
   case 9: {
       if(value != eeprom_ptr->EEPROM_Amp_H){
          SerialUSB.println("FOG_MOD_AMP_H changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Amp_H, eeprom_ptr->EEPROM_ADDR_MOD_AMP_H, value);
          sp->updateParameter(myCmd_header, 1, myCmd_trailer, eeprom_ptr->EEPROM_Amp_H, 0xCC);
        }
        break;}
   case 10: {
        if(value != eeprom_ptr->EEPROM_Amp_L){
          SerialUSB.println("FOG_MOD_AMP_L changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Amp_L, eeprom_ptr->EEPROM_ADDR_MOD_AMP_L, value);
          sp->updateParameter(myCmd_header, 2, myCmd_trailer, eeprom_ptr->EEPROM_Amp_L, 0xCC);
        }
        break;}
   case 11: {
        if(value != eeprom_ptr->EEPROM_Err_offset){
          SerialUSB.println("FOG_ERR_OFFSET changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Err_offset, eeprom_ptr->EEPROM_ADDR_ERR_OFFSET, value);
          sp->updateParameter(myCmd_header, 3, myCmd_trailer, eeprom_ptr->EEPROM_Err_offset, 0xCC);
        }
        break;}
   case 12: {
        if(value != eeprom_ptr->EEPROM_Polarity){
          SerialUSB.println("FOG_POLARITY changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Polarity, eeprom_ptr->EEPROM_ADDR_POLARITY, value);
          sp->updateParameter(myCmd_header, 4, myCmd_trailer, eeprom_ptr->EEPROM_Polarity, 0xCC);
        }
        break;}
   case 13:{
        if(value != eeprom_ptr->EEPROM_Wait_cnt){
          SerialUSB.println("FOG_WAIT_CNT changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Wait_cnt, eeprom_ptr->EEPROM_ADDR_WAIT_CNT, value);
          sp->updateParameter(myCmd_header, 5, myCmd_trailer, eeprom_ptr->EEPROM_Wait_cnt, 0xCC);
        }
        break;}
   case 14: {
        if(value != eeprom_ptr->EEPROM_Err_th){
          SerialUSB.println("FOG_ERR_TH changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Err_th, eeprom_ptr->EEPROM_ADDR_ERR_TH, value);
          sp->updateParameter(myCmd_header, 6, myCmd_trailer, eeprom_ptr->EEPROM_Err_th, 0xCC);
        }
        break;}
   case 15: {
        if(value != eeprom_ptr->EEPROM_Err_avg){
          SerialUSB.println("FOG_ERR_AVG changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Err_avg, eeprom_ptr->EEPROM_ADDR_ERR_AVG, value);
          sp->updateParameter(myCmd_header, 7, myCmd_trailer, eeprom_ptr->EEPROM_Err_avg, 0xCC);
        }
        break;}
   case 16: {
        sp->updateParameter(myCmd_header, 8, myCmd_trailer, value, 0xCC);
        break;}
   case 17: {
       if(value != eeprom_ptr->EEPROM_Gain1){
          SerialUSB.println("FOG_GAIN1 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Gain1, eeprom_ptr->EEPROM_ADDR_GAIN1, value);
          sp->updateParameter(myCmd_header, 9, myCmd_trailer, eeprom_ptr->EEPROM_Gain1, 0xCC);
        }
        break;}
   case 18: {
       if(value != eeprom_ptr->EEPROM_Gain2){
          SerialUSB.println("FOG_GAIN2 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Gain2, eeprom_ptr->EEPROM_ADDR_GAIN2, value);
          sp->updateParameter(myCmd_header, 10, myCmd_trailer, eeprom_ptr->EEPROM_Gain2, 0xCC);
        }
        break;}
   case 19: {
       if(value != eeprom_ptr->EEPROM_FB_ON){
          SerialUSB.println("FOG_FB_ON changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_FB_ON, eeprom_ptr->EEPROM_ADDR_FB_ON, value);
          sp->updateParameter(myCmd_header, 11, myCmd_trailer, eeprom_ptr->EEPROM_FB_ON, 0xCC);
        }
        break;}
   case 20: {
        if(value != eeprom_ptr->EEPROM_Const_step){
          SerialUSB.println("FOG_CONST_STEP changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Const_step, eeprom_ptr->EEPROM_ADDR_CONST_STEP, value);
          sp->updateParameter(myCmd_header, 12, myCmd_trailer, eeprom_ptr->EEPROM_Const_step, 0xCC);
        }
        break;}
   case 21: {
       if(value != eeprom_ptr->EEPROM_Fpga_Q){
          SerialUSB.println("FOG_FPGA_Q changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Fpga_Q, eeprom_ptr->EEPROM_ADDR_FPGA_Q, value);
          sp->updateParameter(myCmd_header, 13, myCmd_trailer, eeprom_ptr->EEPROM_Fpga_Q, 0xCC);
        }
        break;}
   case 22: {
       if(value != eeprom_ptr->EEPROM_Fpga_R){
          SerialUSB.println("FOG_FPGA_R changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Fpga_R, eeprom_ptr->EEPROM_ADDR_FPGA_R, value);
          sp->updateParameter(myCmd_header, 14, myCmd_trailer, eeprom_ptr->EEPROM_Fpga_R, 0xCC);
        }
        break;}
   case 23: {
       if(value != eeprom_ptr->EEPROM_DAC_gain){
          SerialUSB.println("FOG_DAC_GAIN changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_DAC_gain, eeprom_ptr->EEPROM_ADDR_DAC_GAIN, value);
          sp->updateParameter(myCmd_header, 50, myCmd_trailer, eeprom_ptr->EEPROM_DAC_gain, 0xCC);
        }
        break;}
   case 24: {
        if(value != eeprom_ptr->EEPROM_Data_delay){
          SerialUSB.println("FOG_INT_DELAY changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_Data_delay, eeprom_ptr->EEPROM_ADDR_DATA_DELAY, value);
          sp->updateParameter(myCmd_header, 98, myCmd_trailer, eeprom_ptr->EEPROM_Data_delay, 0xCC);
        }
        break;}
      case 25: {
        sp->updateParameter(myCmd_header, 99, myCmd_trailer, value, 0xCC);
        break;}

      case 26: {
        if(value != eeprom_ptr->EEPROM_SF0){
          SerialUSB.println("FOG_SF0 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF0, eeprom_ptr->EEPROM_ADDR_SF_0, value);
          sp->updateParameter(myCmd_header, 15, myCmd_trailer, eeprom_ptr->EEPROM_SF0, 0xCC);
        }
      break;}
      case 27: {
        if(value != eeprom_ptr->EEPROM_SF1){
          SerialUSB.println("FOG_SF1 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF1, eeprom_ptr->EEPROM_ADDR_SF_1, value);
          sp->updateParameter(myCmd_header, 16, myCmd_trailer, eeprom_ptr->EEPROM_SF1, 0xCC);
        }
      break;}
      case 28: {
        if(value != eeprom_ptr->EEPROM_SF2){
          SerialUSB.println("FOG_SF2 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF2, eeprom_ptr->EEPROM_ADDR_SF_2, value);
          sp->updateParameter(myCmd_header, 17, myCmd_trailer, eeprom_ptr->EEPROM_SF2, 0xCC);
        }
      break;}
      case 29: {
        if(value != eeprom_ptr->EEPROM_SF3){
          SerialUSB.println("FOG_SF3 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF3, eeprom_ptr->EEPROM_ADDR_SF_3, value);
          sp->updateParameter(myCmd_header, 18, myCmd_trailer, eeprom_ptr->EEPROM_SF3, 0xCC);
        }
      break;}
      case 30: {
        if(value != eeprom_ptr->EEPROM_SF4){
          SerialUSB.println("FOG_SF4 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF4, eeprom_ptr->EEPROM_ADDR_SF_4, value);
          sp->updateParameter(myCmd_header, 19, myCmd_trailer, eeprom_ptr->EEPROM_SF4, 0xCC);
        }
      break;}
      case 31: {
        if(value != eeprom_ptr->EEPROM_SF5){
          SerialUSB.println("FOG_SF5 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF5, eeprom_ptr->EEPROM_ADDR_SF_5, value);
          sp->updateParameter(myCmd_header, 20, myCmd_trailer, eeprom_ptr->EEPROM_SF5, 0xCC);
        }
      break;}
      case 32: {
        if(value != eeprom_ptr->EEPROM_SF6){
          SerialUSB.println("FOG_SF6 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF6, eeprom_ptr->EEPROM_ADDR_SF_6, value);
          sp->updateParameter(myCmd_header, 21, myCmd_trailer, eeprom_ptr->EEPROM_SF6, 0xCC);
        }
      break;}
      case 33: {
        if(value != eeprom_ptr->EEPROM_SF7){
          SerialUSB.println("FOG_SF7 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF7, eeprom_ptr->EEPROM_ADDR_SF_7, value);
          sp->updateParameter(myCmd_header, 22, myCmd_trailer, eeprom_ptr->EEPROM_SF7, 0xCC);
        }
      break;}
      case 34: {
        if(value != eeprom_ptr->EEPROM_SF8){
          SerialUSB.println("FOG_SF8 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF8, eeprom_ptr->EEPROM_ADDR_SF_8, value);
          sp->updateParameter(myCmd_header, 23, myCmd_trailer, eeprom_ptr->EEPROM_SF8, 0xCC);
        }
      break;}
      case 35: {
        if(value != eeprom_ptr->EEPROM_SF9){
          SerialUSB.println("FOG_SF9 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SF9, eeprom_ptr->EEPROM_ADDR_SF_9, value);
          sp->updateParameter(myCmd_header, 24, myCmd_trailer, eeprom_ptr->EEPROM_SF9, 0xCC);
        }
      break;}
      case 38: {
        if(value != eeprom_ptr->EEPROM_SFB){
          SerialUSB.println("FOG_SFB changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SFB, eeprom_ptr->EEPROM_ADDR_SFB, value);
          sp->updateParameter(myCmd_header, 27, myCmd_trailer, eeprom_ptr->EEPROM_SFB, 0xCC);
        }
      break;}
      case 39: {
        if(value != eeprom_ptr->EEPROM_CUTOFF){
          SerialUSB.println("FOG_CUTOFF changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_CUTOFF, eeprom_ptr->EEPROM_ADDR_CUTOFF, value);
          sp->updateParameter(myCmd_header, 28, myCmd_trailer, eeprom_ptr->EEPROM_CUTOFF, 0xCC);
        }
      break;}
      case 36: {
        if(value != eeprom_ptr->EEPROM_TMIN){
          SerialUSB.println("FOG_T_MIN changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_TMIN, eeprom_ptr->EEPROM_ADDR_TMIN, value);
          sp->updateParameter(myCmd_header, 25, myCmd_trailer, eeprom_ptr->EEPROM_TMIN, 0xCC);
        }
      break;}
      case 37: {
        // Serial.println(value, HEX);
        // Serial.println(eeprom_ptr->EEPROM_TMAX, HEX);
        if(value != eeprom_ptr->EEPROM_TMAX){
          SerialUSB.println("FOG_T_MAX changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_TMAX, eeprom_ptr->EEPROM_ADDR_TMAX, value);
          sp->updateParameter(myCmd_header, 26, myCmd_trailer, eeprom_ptr->EEPROM_TMAX, 0xCC);
        }
      break;}

      case 103 /*0x67*/: {
        if(value != EEPROM_BAUDRATE){
          SerialUSB.println("Baudrate changed!");
          write_fog_parameter_to_eeprom(EEPROM_BAUDRATE, 194 /*194 ~ 209, len = 16*/, value);
          update_baudrate(EEPROM_BAUDRATE);
        }
      break;}

      case 104 /*0x68*/: {
        if(value != EEPROM_DATARATE){
          SerialUSB.println("Datarate changed!");
          write_fog_parameter_to_eeprom(EEPROM_DATARATE, 194 /*194 ~ 209, len = 16*/+1, value);
          update_datarate(EEPROM_DATARATE);
        }
      break;}

      case 130 /*82*/: {
        if(value != EEPROM_SYSCLK){
          SerialUSB.println("system clock selection changed!");
          print_clock_configuration(value);
          write_fog_parameter_to_eeprom(EEPROM_SYSCLK, 194 /*194 ~ 209, len = 16*/+2, value);
        }
      break;}

      case 131 /*83*/: {
        if(value != EEPROM_EXTWDT){
          SerialUSB.println("system clock selection changed!");
          msg_out("EXT WDT setup changed: ");
          print_ext_WDT_configuration(value);
          write_fog_parameter_to_eeprom(EEPROM_EXTWDT, 194 /*194 ~ 209, len = 16*/+3, value);
        }
      break;}

      case 132 /*84*/: {
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

      case 105: {
        if(value != EEPROM_CALI_AX){
          SerialUSB.println("CALI_AX changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_AX, 210 /*210 ~ 241, len = 32*/, value);
          misalignment_cali_coe._d.ax = EEPROM_CALI_AX;
        }
      break;}
      case 106: {
        if(value != EEPROM_CALI_AY){
          SerialUSB.println("CALI_AY changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_AY, 210 /*210 ~ 241, len = 32*/+1, value);
          misalignment_cali_coe._d.ay = EEPROM_CALI_AY;
        }
      break;}
      case 107: {
        if(value != EEPROM_CALI_AZ){
          SerialUSB.println("CALI_AZ changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_AZ, 210 /*210 ~ 241, len = 32*/+2, value);
          misalignment_cali_coe._d.az = EEPROM_CALI_AZ;
        }
      break;}
      case 108: {
        if(value != EEPROM_CALI_A11){
          SerialUSB.println("CALI_A11 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A11, 210 /*210 ~ 241, len = 32*/+3, value);
          misalignment_cali_coe._d.a11 = EEPROM_CALI_A11;
        }
      break;}
      case 109: {
        if(value != EEPROM_CALI_A12){
          SerialUSB.println("CALI_A12 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A12, 210 /*210 ~ 241, len = 32*/+4, value);
          misalignment_cali_coe._d.a12 = EEPROM_CALI_A12;
        }
      break;}
      case 110: {
        if(value != EEPROM_CALI_A13){
          SerialUSB.println("CALI_A13 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A13, 210 /*210 ~ 241, len = 32*/+5, value);
          misalignment_cali_coe._d.a13 = EEPROM_CALI_A13;
        }
      break;}
      case 111: {
        if(value != EEPROM_CALI_A21){
          SerialUSB.println("CALI_A21 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A21, 210 /*210 ~ 241, len = 32*/+6, value);
          misalignment_cali_coe._d.a21 = EEPROM_CALI_A21;
        }
      break;}
      case 112: {
        if(value != EEPROM_CALI_A22){
          SerialUSB.println("CALI_A22 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A22, 210 /*210 ~ 241, len = 32*/+7, value);
          misalignment_cali_coe._d.a22 = EEPROM_CALI_A22;
        }
      break;}
      case 113: {
        if(value != EEPROM_CALI_A23){
          SerialUSB.println("CALI_A23 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A23, 210 /*210 ~ 241, len = 32*/+8, value);
          misalignment_cali_coe._d.a23 = EEPROM_CALI_A23;
        }
      break;}
      case 114: {
        if(value != EEPROM_CALI_A31){
          SerialUSB.println("CALI_A31 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A31, 210 /*210 ~ 241, len = 32*/+9, value);
          misalignment_cali_coe._d.a31 = EEPROM_CALI_A31;
        }
      break;}
      case 115: {
        if(value != EEPROM_CALI_A32){
          SerialUSB.println("CALI_A32 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A32, 210 /*210 ~ 241, len = 32*/+10, value);
          misalignment_cali_coe._d.a32 = EEPROM_CALI_A32;
        }
      break;}
      case 116: {
        if(value != EEPROM_CALI_A33){
          SerialUSB.println("CALI_A33 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_A33, 210 /*210 ~ 241, len = 32*/+11, value);
          misalignment_cali_coe._d.a33 = EEPROM_CALI_A33;
        }
      break;}

      case 117: {
        if(value != EEPROM_CALI_GX){
          SerialUSB.println("CALI_GX changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_GX, 210 /*210 ~ 241, len = 32*/+12, value);
          misalignment_cali_coe._d.gx = EEPROM_CALI_GX;
        }
      break;}
      case 118: {
        if(value != EEPROM_CALI_GY){
          SerialUSB.println("CALI_GY changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_GY, 210 /*210 ~ 241, len = 32*/+13, value);
          misalignment_cali_coe._d.gy = EEPROM_CALI_GY;
        }
      break;}
      case 119: {
        if(value != EEPROM_CALI_GZ){
          SerialUSB.println("CALI_GZ changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_GZ, 210 /*210 ~ 241, len = 32*/+14, value);
          misalignment_cali_coe._d.gz = EEPROM_CALI_GZ;
        }
      break;}
      case 120: {
        if(value != EEPROM_CALI_G11){
          SerialUSB.println("CALI_G11 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G11, 210 /*210 ~ 241, len = 32*/+15, value);
          misalignment_cali_coe._d.g11 = EEPROM_CALI_G11;
        }
      break;}
      case 121: {
        if(value != EEPROM_CALI_G12){
          SerialUSB.println("CALI_G12 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G12, 210 /*210 ~ 241, len = 32*/+16, value);
          misalignment_cali_coe._d.g12 = EEPROM_CALI_G12;
        }
      break;}
      case 122: {
        if(value != EEPROM_CALI_G13){
          SerialUSB.println("CALI_G13 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G13, 210 /*210 ~ 241, len = 32*/+17, value);
          misalignment_cali_coe._d.g13 = EEPROM_CALI_G13;
        }
      break;}
      case 123: {
        if(value != EEPROM_CALI_G21){
          SerialUSB.println("CALI_G21 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G21, 210 /*210 ~ 241, len = 32*/+18, value);
          misalignment_cali_coe._d.g21 = EEPROM_CALI_G21;
        }
      break;}
      case 124: {
        if(value != EEPROM_CALI_G22){
          SerialUSB.println("CALI_G22 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G22, 210 /*210 ~ 241, len = 32*/+19, value);
          misalignment_cali_coe._d.g22 = EEPROM_CALI_G22;
        }
      break;}
      case 125: {
        if(value != EEPROM_CALI_G23){
          SerialUSB.println("CALI_G23 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G23, 210 /*210 ~ 241, len = 32*/+20, value);
          misalignment_cali_coe._d.g23 = EEPROM_CALI_G23;
        }
      break;}
      case 126: {
        if(value != EEPROM_CALI_G31){
          SerialUSB.println("CALI_G31 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G31, 210 /*210 ~ 241, len = 32*/+21, value);
          misalignment_cali_coe._d.g31 = EEPROM_CALI_G31;
        }
      break;}
      case 127: {
        if(value != EEPROM_CALI_G32){
          SerialUSB.println("CALI_G32 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G32, 210 /*210 ~ 241, len = 32*/+22, value);
          misalignment_cali_coe._d.g32 = EEPROM_CALI_G32;
        }
      break;}
      case 128: {
        if(value != EEPROM_CALI_G33){
          SerialUSB.println("CALI_G33 changed!");
          write_fog_parameter_to_eeprom(EEPROM_CALI_G33, 210 /*210 ~ 241, len = 32*/+23, value);
          misalignment_cali_coe._d.g33 = EEPROM_CALI_G33;
        }
      break;}
      case 40 /*28*/: {
        if(value != eeprom_ptr->EEPROM_BIAS_COMP_T1){
          SerialUSB.println("BIAS_COMP_T1 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_BIAS_COMP_T1, eeprom_ptr->EEPROM_ADDR_BIAS_COMP_T1, value);
          sp->updateParameter(myCmd_header, 29, myCmd_trailer, eeprom_ptr->EEPROM_BIAS_COMP_T1, 0xCC);
        }
        // Serial.println(eeprom_ptr->EEPROM_ADDR_BIAS_COMP_T1);
        // eeprom.Parameter_Read(eeprom_ptr->EEPROM_ADDR_BIAS_COMP_T1, my_f.bin_val);
        // Serial.println(my_f.float_val,10);
      break;}
      case 41 /*29*/: {
        if(value != eeprom_ptr->EEPROM_BIAS_COMP_T2){
          SerialUSB.println("BIAS_COMP_T2 changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_BIAS_COMP_T2, eeprom_ptr->EEPROM_ADDR_BIAS_COMP_T2, value);
          sp->updateParameter(myCmd_header, 30, myCmd_trailer, eeprom_ptr->EEPROM_BIAS_COMP_T2, 0xCC);
        }
        // Serial.println(eeprom_ptr->EEPROM_ADDR_BIAS_COMP_T2);
        // eeprom.Parameter_Read(eeprom_ptr->EEPROM_ADDR_BIAS_COMP_T2, my_f.bin_val);
        // Serial.println(my_f.float_val,10);
      break;}
      case 42 /*2A*/: {
        if(value != eeprom_ptr->EEPROM_SFB_1_SLOPE){
          SerialUSB.println("SFB_1_SLOPE changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SFB_1_SLOPE, eeprom_ptr->EEPROM_ADDR_SFB_1_SLOPE, value);
          sp->updateParameter(myCmd_header, 31, myCmd_trailer, eeprom_ptr->EEPROM_SFB_1_SLOPE, 0xCC);
        }
      break;}
      case 43 /*2B*/: {
        if(value != eeprom_ptr->EEPROM_SFB_1_OFFSET){
          SerialUSB.println("SFB_1_OFFSET changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SFB_1_OFFSET, eeprom_ptr->EEPROM_ADDR_SFB_1_OFFSET, value);
          sp->updateParameter(myCmd_header, 32, myCmd_trailer, eeprom_ptr->EEPROM_SFB_1_OFFSET, 0xCC);
        }
      break;}
      case 44 /*2C*/: {
        if(value != eeprom_ptr->EEPROM_SFB_2_SLOPE){
          SerialUSB.println("SFB_2_SLOPE changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SFB_2_SLOPE, eeprom_ptr->EEPROM_ADDR_SFB_2_SLOPE, value);
          sp->updateParameter(myCmd_header, 33, myCmd_trailer, eeprom_ptr->EEPROM_SFB_2_SLOPE, 0xCC);
        }
      break;}
      case 45 /*2D*/: {
        if(value != eeprom_ptr->EEPROM_SFB_2_OFFSET){
          SerialUSB.println("SFB_2_OFFSET changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SFB_2_OFFSET, eeprom_ptr->EEPROM_ADDR_SFB_2_OFFSET, value);
          sp->updateParameter(myCmd_header, 34, myCmd_trailer, eeprom_ptr->EEPROM_SFB_2_OFFSET, 0xCC);
        }
      break;}
      case 46 /*2E*/: {
        if(value != eeprom_ptr->EEPROM_SFB_3_SLOPE){
          SerialUSB.println("SFB_3_SLOPE changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SFB_3_SLOPE, eeprom_ptr->EEPROM_ADDR_SFB_3_SLOPE, value);
          sp->updateParameter(myCmd_header, 35, myCmd_trailer, eeprom_ptr->EEPROM_SFB_3_SLOPE, 0xCC);
        }
      break;}
      case 47 /*2F*/: {
        if(value != eeprom_ptr->EEPROM_SFB_3_OFFSET){
          SerialUSB.println("SFB_3_OFFSET changed!");
          write_fog_parameter_to_eeprom(eeprom_ptr->EEPROM_SFB_3_OFFSET, eeprom_ptr->EEPROM_ADDR_SFB_3_OFFSET, value);
          sp->updateParameter(myCmd_header, 36, myCmd_trailer, eeprom_ptr->EEPROM_SFB_3_OFFSET, 0xCC);
        }
      break;}

      case 101 /*0x65*/: {
        String fpga_version;
        for(int i=0; i<255; i++) SER->read();//clear serial buffer
        sp->updateParameter(myCmd_header, 101, myCmd_trailer, value, 0xCC);
        while(!SER->available());
        if(SER->available()) fpga_version = SER->readStringUntil('\n');
        SerialUSB.print("CH: ");
        SerialUSB.println(fog_ch);
        SerialUSB.print("MCU-GP-28-2-PD");
        Serial1.print("MCU-GP-28-2-PD");
        SerialUSB.print(',');
        Serial1.print(',');
        SerialUSB.println(fpga_version);
        Serial1.println(fpga_version);
        break;
      }

      case 102 /*0x66*/: {
        String fog_parameter;
        for(int i=0; i<255; i++) SER->read();//clear serial buffer
        sp->updateParameter(myCmd_header, 102, myCmd_trailer, value, 0xCC);
        while(!SER->available()){delay(1);};
        if(SER->available())
         {
          fog_parameter = SER->readStringUntil('\n');
          SerialUSB.println(fog_parameter);
          Serial1.println(fog_parameter);
         }

        break;
      }
      case 129 /*81*/: {
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
 if(mux_flag == 0)
 {
  mux_flag = 2;
  switch(mode) {
   case 0: {
    output_fn = fn_rst;
    select_fn = 1;
        rst_fn_flag = 0;
    break;
   }
   case 1: {
    output_fn = acq_fog;
    select_fn = 2;
        rst_fn_flag = 1;
    break;
   }
   case 2: {
    output_fn = acq_imu;
    select_fn = 5;
        rst_fn_flag = 2;
    break;
   }
   case 3: {
    output_fn = acq_HP_test;
    select_fn = 8;
        rst_fn_flag = 3;
    break;
   }
   case 4: {
    output_fn = acq_nmea;
    select_fn = 6;
        rst_fn_flag = 4;
    break;
            }
      case 5: {
    output_fn = acq_att_nmea;
    select_fn = 9;
        rst_fn_flag = 5;
    break;
            }
      case 6: {
          output_fn = acq_fog_parameter;
          select_fn = 7;
          rst_fn_flag = 6;
          break;
      }

      default: break;
      }
      eeprom.Parameter_Write(4091, select_fn);
      eeprom.Parameter_Write(4090, rst_fn_flag);
      eeprom.Parameter_Write(4089, uart_value);
 }

  if(fog_op_status==1) // for auto reset
  {
    fog_op_status=0;
    SerialUSB.println("AUTO RST select function");
  switch(rst_fn_flag) {
   case 0: {
    output_fn = fn_rst;
    break;
   }
   case 1: {
    output_fn = acq_fog;
        SerialUSB.println("MODE_FOG");
    break;
   }
   case 2: {
    output_fn = acq_imu;
    break;
   }
   case 3: {
    output_fn = acq_HP_test;
    break;
   }
   case 4: {
    output_fn = acq_nmea;
    break;
            }
      case 5: {
    output_fn = acq_att_nmea;
    break;
            }
      case 6: {
          output_fn = acq_fog_parameter;
          break;
      }
      default: break;
      }
      eeprom.Parameter_Write(4091, select_fn);
      eeprom.Parameter_Write(4090, rst_fn_flag);
      eeprom.Parameter_Write(4089, uart_value);
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
  SerialUSB.println("Enter fn_rst!");
 if(select_fn&1) {
  switch(CTRLREG) {
   case 1: {
    for(int i=0; i<256; i++) Serial1.read();
    break;
   }
   default: break;
  }
  SerialUSB.println("Set fog_op_status to 0");
  eeprom.Write(4095, 0);
  SerialUSB.println("Set fn_rst to temp_idle");
  output_fn = temp_idle;
 }
 clear_SEL_EN(select_fn);
}


void acq_fog_parameter(byte &select_fn, unsigned int value, byte ch)
{
  byte *fog;
 uint8_t CRC32[4];

 if(select_fn&7)
 {
    SerialUSB.print("Enter fog parameter mode, channel: ");
    SerialUSB.println(ch);
    CtrlReg = value;
    if(ch==1) run_fog_flag = sp13.setSyncMode(CtrlReg);
    else if(ch==2) run_fog_flag = sp14.setSyncMode(CtrlReg);
    else if(ch==3) run_fog_flag = sp9.setSyncMode(CtrlReg);



    switch(CtrlReg){
      case 1:
        data_cnt = 0;
        SerialUSB.println("Enter INT_SYNC mode");
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(4095, 1);
        setupWDT(11);
        enable_EXT_WDT(4);
        reset_EXT_WDI(5);
      break;

      case 2:
        data_cnt = 0;
        SerialUSB.println("Enter EXT_SYNC mode");
        SerialUSB.println("Set EXTT to RISING");
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(4095, 1);
        setupWDT(11);
        enable_EXT_WDT(4);
        reset_EXT_WDI(5);
      break;

      case 4:
        reset_SYNC();
        data_cnt = 0;
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(4095, 0);
        disableWDT();
        disable_EXT_WDT(4);
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


        if(data_cnt >= 5)
        {
          Serial1.write(KVH_HEADER, 4);
          Serial1.write(reg_fog, 16);
          Serial1.write(mcu_time.bin_val, 4);
          Serial1.write(CRC32, 4);
        }

      resetWDT();
      reset_EXT_WDI(5);
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

 if(select_fn&2)
 {
    SerialUSB.print("Enter acq_fog mode, fog channel: ");
    SerialUSB.println(ch);
    CtrlReg = value;
    if(ch==1) run_fog_flag = sp13.setSyncMode(CtrlReg);
    else if(ch==2) run_fog_flag = sp14.setSyncMode(CtrlReg);
    else if(ch==3) run_fog_flag = sp9.setSyncMode(CtrlReg);

    switch(CtrlReg){
      case 1:
        data_cnt = 0;
        SerialUSB.println("Enter INT_SYNC mode");
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(4095, 1);
        setupWDT(11);
      break;

      case 2:
        SerialUSB.println("Enter EXT_SYNC mode");
        SerialUSB.println("Set EXTT to RISING");
        data_cnt = 0;
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(4095, 1);
        setupWDT(11);
        enable_EXT_WDT(4);
        reset_EXT_WDI(5);
      break;

      case 4:
        data_cnt = 0;
        reset_SYNC();
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(4095, 0);
        disableWDT();
        disable_EXT_WDT(4);
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


        if(data_cnt >= 5)
        {
          Serial1.write(KVH_HEADER, 4);
          Serial1.write(cali_fog, 4);
          Serial1.write(reg_fog+12, 4);
          Serial1.write(mcu_time.bin_val, 4);
          Serial1.write(CRC32, 4);

        }
        resetWDT();
        reset_EXT_WDI(5);
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

  byte *fog;
 uint8_t CRC32[4];

  if(select_fn&5)
 {
    SerialUSB.print("Enter acq_imu mode, channel: ");
    SerialUSB.println(ch);
    CtrlReg = value;

    if(ch==1) run_fog_flag = sp13.setSyncMode(CtrlReg);
    else if(ch==2) run_fog_flag = sp14.setSyncMode(CtrlReg);
    else if(ch==3) run_fog_flag = sp9.setSyncMode(CtrlReg);

    switch(CtrlReg){
      case 1:
        data_cnt = 0;
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(4095, 1);
        setupWDT(11);
        enable_EXT_WDT(4);
        reset_EXT_WDI(5);

      break;
      case 2:
        data_cnt = 0;
        SerialUSB.println("Enter EXT_SYNC mode");
        SerialUSB.println("Set EXTT to CHANGE");

        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(4095, 1);
        setupWDT(11);
        enable_EXT_WDT(4);
        reset_EXT_WDI(5);

      break;
      case 4:
        reset_SYNC();
        data_cnt = 0;
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(4095, 0);
        my_cpf.resetEuler(0,0,0);
        disableWDT();
        disable_EXT_WDT(4);
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
      my_GYRO.float_val[2] = myfog_GYRO.float_val;
      // my_GYRO.float_val[2] = myfog_GYRO.float_val * DEG_TO_RAD;
      /*** ------mis-alignment calibration gyro raw data -----***/
      gyro_cali(my_GYRO_cali.float_val, my_GYRO.float_val);
      print_imu_data(false, my_memsXLM_cali.float_val, my_GYRO_cali.float_val);

      memcpy(imu_data, KVH_HEADER, 4);
      memcpy(imu_data+4, my_GYRO_cali.bin_val, 12);//wx, wy, wz
      memcpy(imu_data+16, my_memsXLM_cali.bin_val, 12);//ax, ay, az
      memcpy(imu_data+28, reg_fog+12, 4);// PD temp
      memcpy(imu_data+32, mcu_time.bin_val, 4);
      memcpy(imu_data+36, my_att.bin_val, 12);
      myCRC.crc_32(imu_data, 48, CRC32);

      free(imu_data);


      if(data_cnt >= 5)
      {
        Serial1.write(KVH_HEADER, 4);
        Serial1.write(my_GYRO_cali.bin_val, 12); //wx, wy, wz
        Serial1.write(my_memsXLM_cali.bin_val, 12);//ax, ay, az
        Serial1.write(reg_fog+12, 4); // PD temp
        Serial1.write(mcu_time.bin_val, 4);
        Serial1.write(my_att.bin_val, 12);
        Serial1.write(CRC32, 4);
      }

      resetWDT();
      reset_EXT_WDI(5);

      my_cpf.run(float(mcu_time.ulong_val) * 1e-3, my_GYRO_cali.float_val, my_memsXLM_cali.float_val);
      my_cpf.getEularAngle(my_att.float_val); //raw data -> att, pitch, row, yaw 
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

  if(select_fn&5)
 {
    SerialUSB.print("Enter acq_att_nmea mode, channel: ");
    SerialUSB.println(ch);
    CtrlReg = value;

    if(ch==1) run_fog_flag = sp13.setSyncMode(CtrlReg);
    else if(ch==2) run_fog_flag = sp14.setSyncMode(CtrlReg);
    else if(ch==3) run_fog_flag = sp9.setSyncMode(CtrlReg);

    switch(CtrlReg){
      case 1:
        data_cnt = 0;
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(4095, 1);
        setupWDT(11);
        enable_EXT_WDT(4);
        reset_EXT_WDI(5);

      break;
      case 2:
        data_cnt = 0;
        SerialUSB.println("Enter EXT_SYNC mode");
        SerialUSB.println("Set EXTT to CHANGE");

        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 3;
        eeprom.Write(4095, 1);
        setupWDT(11);
        enable_EXT_WDT(4);
        reset_EXT_WDI(5);

      break;
      case 4:
        reset_SYNC();
        data_cnt = 0;
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(4095, 0);
        my_cpf.resetEuler(0,0,0);
        disableWDT();
        disable_EXT_WDT(4);
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



      if(data_cnt >= 5)
      {
        sprintf(nmeaOutput, "$%s*%02X\r\n", nmeaSentence, checksum);
        Serial1.print(nmeaOutput);
      }

      resetWDT();
      reset_EXT_WDI(5);

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

 if(select_fn&6)
 {
    SerialUSB.print("Enter fog nmea mode, channel: ");
    SerialUSB.println(ch);
    CtrlReg = value;
    if(ch==1) run_fog_flag = sp13.setSyncMode(CtrlReg);
    else if(ch==2) run_fog_flag = sp14.setSyncMode(CtrlReg);
    else if(ch==3) run_fog_flag = sp9.setSyncMode(CtrlReg);

    switch(CtrlReg){
      case 1:
        data_cnt = 0;
        SerialUSB.println("Enter INT_SYNC mode");
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(4095, 1);
        setupWDT(11);
        enable_EXT_WDT(4);
        reset_EXT_WDI(5);
      break;

      case 2:
        SerialUSB.println("Enter EXT_SYNC mode");
        SerialUSB.println("Set EXTT to RISING");
        data_cnt = 0;
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(4095, 1);
        setupWDT(11);
        enable_EXT_WDT(4);
        reset_EXT_WDI(5);
      break;

      case 5:
        SerialUSB.println("Enter EXT_SYNC NMEA_MODE mode");
        SerialUSB.println("Set EXTT to RISING");
        data_cnt = 0;
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(4095, 1);
        setupWDT(11);
        enable_EXT_WDT(4);
        reset_EXT_WDI(5);
      break;

      case 4:
        reset_SYNC();
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(4095, 0);
        data_cnt = 0;
        disableWDT();
        disable_EXT_WDT(4);
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

        if(data_cnt >= 5){
          Serial1.write(reg_fog, 14);
          Serial1.println("");
        }


       resetWDT();
       reset_EXT_WDI(5);
      }
 }
 clear_SEL_EN(select_fn);
}

void acq_HP_test(byte &select_fn, unsigned int value, byte ch)
{
  byte *fog, eeprom_var[4], mcu_var[4], adc_var[8];
 uint8_t CRC32[4];


 if(select_fn&8)
 {
    SerialUSB.print("fog channel: ");
    SerialUSB.println(ch);
    SerialUSB.println("select SEL_HP_TEST\n");
    CtrlReg = value;
    if(ch==1) run_fog_flag = sp13.setSyncMode(CtrlReg);
    else if(ch==2) run_fog_flag = sp14.setSyncMode(CtrlReg);
    else if(ch==3) run_fog_flag = sp9.setSyncMode(CtrlReg);

    switch(CtrlReg){
      case 1:
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(4095, 1);
        setupWDT(11);

      break;
      case 2:
        SerialUSB.println("Enter EXT_SYNC mode");
        SerialUSB.println("Set EXTT to RISING");
        SerialUSB.println("Write SYNC to LOW\n");

        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(4095, 1);
        setupWDT(11);

      break;
      case 4:
        reset_SYNC();
        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
        eeprom.Write(4095, 0);
        MCU_cnt = 0;
        disableWDT();
      break;

      case 3:
        SerialUSB.println("Enter HP_TEST mode");
        SerialUSB.println("Set EXTT to CHANGE");

        ((Eic *)0x40001800UL) /**< \brief (EIC) APB Base Address */->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
        eeprom.Write(4095, 1);
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


        Serial1.write(KVH_HEADER, 4);
        Serial1.write(fog, 14);
        Serial1.write(eeprom_var, 4);
        Serial1.write(mcu_var, 4);
        Serial1.write(adc_var, 8);
        Serial1.write(CRC32, 4);

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
    SerialUSB.print(data[0], 16);
    SerialUSB.print("\t");
    SerialUSB.print(data[1], 16);
    SerialUSB.print("\t");
    SerialUSB.print(data[2], 16);
    SerialUSB.print("\t");
    SerialUSB.print(data[3], 16);
    SerialUSB.print("\t");
    SerialUSB.print(data[4], 16);
    SerialUSB.print("\t");
    SerialUSB.print(data[5], 16);
    SerialUSB.print("\t");
 SerialUSB.println(fog);
};


void print_nano33XlmData(byte *data)
{
 int ax, ay, az;
 t_new = micros();
 ax = data[0]<<8 | data[1];
 ay = data[2]<<8 | data[3];
 az = data[4]<<8 | data[5];
 SerialUSB.print(t_new - t_old);
 SerialUSB.print('\t');
 SerialUSB.print((float)ax*0.000122 /* +/- 4g, 4/32768*/);
 SerialUSB.print('\t');
 SerialUSB.print((float)ay*0.000122 /* +/- 4g, 4/32768*/);
 SerialUSB.print('\t');
 SerialUSB.println((float)az*0.000122 /* +/- 4g, 4/32768*/);
 t_old = t_new;
}

void clear_SEL_EN(byte &select_fn)
{
 select_fn = 0;
}

void ISR_EXTT()
{
  // Serial.println(millis());
  sync_status = !sync_status;
  digitalWrite(29 /*PA22*/, sync_status);
  ISR_Coming = !ISR_Coming;
  if(ISR_Coming == true) ISR_PEDGE = true;

  // EIC->CONFIG[1].bit.SENSE7 = 0; ////set interrupt condition to NONE
  }


void readEEPROM(byte data[4])
{
    // Serial1.println("\n\nEEPROM Value: ");
    eeprom.Parameter_Read(4092,my_f.bin_val);
    data[0] = my_f.bin_val[2];
    data[1] = my_f.bin_val[1];
    // Serial.println(" ");
    // Serial.println(my_f.bin_val[2], HEX);
    // Serial.println(my_f.bin_val[1], HEX);
    eeprom.Parameter_Read(4093,my_f.bin_val);
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

  my_f.int_val = analogRead(18);
  data[0] = my_f.bin_val[1];
  data[1] = my_f.bin_val[0];
  my_f.int_val = analogRead(6);
  data[2] = my_f.bin_val[1];
  data[3] = my_f.bin_val[0];
  my_f.int_val = analogRead(17);
  data[4] = my_f.bin_val[1];
  data[5] = my_f.bin_val[0];
  my_f.int_val = analogRead(19);
  data[6] = my_f.bin_val[1];
  data[7] = my_f.bin_val[0];
  // Serial.print(my_f.bin_val[1], HEX);
  // Serial.print(" ");
  // Serial.println(my_f.bin_val[0], HEX);

}


void parameter_init(void)
{
  eeprom.Parameter_Read(1,my_f.bin_val);
  EEPROM_Parameter_exist = my_f.int_val;

  /***fog parameter is empty,  write initial fog data*/
  if(EEPROM_Parameter_exist != 0xAB){
    eeprom.Parameter_Write(1, 0xAB);
    SerialUSB.println("\n*****EEPROM FOG parameter does not exist!********");

    /***output configuration*/
    SerialUSB.println("\nStart setting output configuration.");
    write_fog_parameter_to_eeprom(EEPROM_BAUDRATE, 194 /*194 ~ 209, len = 16*/, 0);
    write_fog_parameter_to_eeprom(EEPROM_DATARATE, 194 /*194 ~ 209, len = 16*/+1, 1);
    write_fog_parameter_to_eeprom( EEPROM_SYSCLK, 194 /*194 ~ 209, len = 16*/+2, 0);
    write_fog_parameter_to_eeprom( EEPROM_EXTWDT, 194 /*194 ~ 209, len = 16*/+3, 1);
    set_output_configuration_init();
    SerialUSB.println("\nSetting output configuration done.");
    /***end of output configuration*/

    /*** IMU misalignment calibration*/
    update_imu_misalignment_init();

    /***end of IMU misalignment calibration*/

    /***fog parameters*/
    SerialUSB.println("\nStart writing initial fog data.");
    // eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0); // change to 0 12/18

    write_fog_parameter_to_eeprom_all(2);
    update_fpga_fog_parameter_init(100, 2);

    /***end of fog parameters*/

  }
  else{
    SerialUSB.println("\n*****EEPROM FOG parameter exist!*****");
    /***fog parameters*/
    SerialUSB.println("\nStart reading fog parameter from eeprom.");
    read_fog_parameter_from_eeprom_all(2);
    update_fpga_fog_parameter_init(100, 2);
    /***end of fog parameters*/

    /***output configuration*/
    SerialUSB.println("\nStart reading output configuration from eeprom.");
    read_fog_parameter_from_eeprom(EEPROM_BAUDRATE, 194 /*194 ~ 209, len = 16*/);
    read_fog_parameter_from_eeprom(EEPROM_DATARATE, 194 /*194 ~ 209, len = 16*/+1);
    read_fog_parameter_from_eeprom(EEPROM_SYSCLK, 194 /*194 ~ 209, len = 16*/+2);
    read_fog_parameter_from_eeprom(EEPROM_EXTWDT, 194 /*194 ~ 209, len = 16*/+3);
    report_current_output_configuration();
    set_output_configuration_init();
    /***end of output configuration*/

    /*** IMU misalignment calibration*/
    read_misalignment_calibration_from_eeprom();
    /***end of IMU misalignment calibration*/

  }
}

void update_imu_misalignment_init()
{
  // Serial.println("\nStart setting IMU misalignment calibration.");
  msg_out("\nStart setting IMU misalignment calibration.");
  write_fog_parameter_to_eeprom(EEPROM_CALI_AX, 210 /*210 ~ 241, len = 32*/, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_AY, 210 /*210 ~ 241, len = 32*/+1, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_AZ, 210 /*210 ~ 241, len = 32*/+2, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_A11, 210 /*210 ~ 241, len = 32*/+3, 0x3f800000);
  write_fog_parameter_to_eeprom(EEPROM_CALI_A12, 210 /*210 ~ 241, len = 32*/+4, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_A13, 210 /*210 ~ 241, len = 32*/+5, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_A21, 210 /*210 ~ 241, len = 32*/+6, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_A22, 210 /*210 ~ 241, len = 32*/+7, 0x3f800000);
  write_fog_parameter_to_eeprom(EEPROM_CALI_A23, 210 /*210 ~ 241, len = 32*/+8, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_A31, 210 /*210 ~ 241, len = 32*/+9, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_A32, 210 /*210 ~ 241, len = 32*/+10, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_A33, 210 /*210 ~ 241, len = 32*/+11, 0x3f800000);
  write_fog_parameter_to_eeprom(EEPROM_CALI_GX, 210 /*210 ~ 241, len = 32*/+12, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_GY, 210 /*210 ~ 241, len = 32*/+13, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_GZ, 210 /*210 ~ 241, len = 32*/+14, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_G11, 210 /*210 ~ 241, len = 32*/+15, 0x3f800000);
  write_fog_parameter_to_eeprom(EEPROM_CALI_G12, 210 /*210 ~ 241, len = 32*/+16, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_G13, 210 /*210 ~ 241, len = 32*/+17, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_G21, 210 /*210 ~ 241, len = 32*/+18, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_G22, 210 /*210 ~ 241, len = 32*/+19, 0x3f800000);
  write_fog_parameter_to_eeprom(EEPROM_CALI_G23, 210 /*210 ~ 241, len = 32*/+20, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_G31, 210 /*210 ~ 241, len = 32*/+21, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_G32, 210 /*210 ~ 241, len = 32*/+22, 0);
  write_fog_parameter_to_eeprom(EEPROM_CALI_G33, 210 /*210 ~ 241, len = 32*/+23, 0x3f800000);
  misalignment_cali_coe._d.ax = 0;
  misalignment_cali_coe._d.ay = 0;
  misalignment_cali_coe._d.az = 0;
  misalignment_cali_coe._d.a11 = 0x3f800000;
  misalignment_cali_coe._d.a12 = 0;
  misalignment_cali_coe._d.a13 = 0;
  misalignment_cali_coe._d.a21 = 0;
  misalignment_cali_coe._d.a22 = 0x3f800000;
  misalignment_cali_coe._d.a23 = 0;
  misalignment_cali_coe._d.a31 = 0;
  misalignment_cali_coe._d.a32 = 0;
  misalignment_cali_coe._d.a33 = 0x3f800000;
  misalignment_cali_coe._d.gx = 0;
  misalignment_cali_coe._d.gy = 0;
  misalignment_cali_coe._d.gz = 0;
  misalignment_cali_coe._d.g11 = 0x3f800000;
  misalignment_cali_coe._d.g12 = 0;
  misalignment_cali_coe._d.g13 = 0;
  misalignment_cali_coe._d.g21 = 0;
  misalignment_cali_coe._d.g22 = 0x3f800000;
  misalignment_cali_coe._d.g23 = 0;
  misalignment_cali_coe._d.g31 = 0;
  misalignment_cali_coe._d.g32 = 0;
  misalignment_cali_coe._d.g33 = 0x3f800000;
  // Serial.println("\nSetting IMU misalignment calibration done.");
  msg_out("\nSetting IMU misalignment calibration done.");
}

void report_current_output_configuration()
{
  Serial1.begin(9600);
  SerialUSB.println("report_current_output_configuration");
  switch(EEPROM_BAUDRATE)
  {
    case 0: {
      Serial1.println("Baudrate set to 230400");
      SerialUSB.println("Baudrate set to 230400");
      break;
    }
    case 1: {
      Serial1.println("Baudrate set to 115200");
      SerialUSB.println("Baudrate set to 115200");
      break;
    }
    case 2: {
      Serial1.println("Baudrate set to 9600");
      SerialUSB.println("Baudrate set to 9600");
      break;
    }
    case 3: {
      Serial1.println("Baudrate set to 4800");
      SerialUSB.println("Baudrate set to 4800");
      break;
    }
    default: {
      Serial1.println("Baudrate set to 230400");
      SerialUSB.println("Baudrate set to 230400");
      break;
    }
  }

  switch(EEPROM_DATARATE)
  {
    case 3: {
      // pwm.timer(1, 2, int(15000*PWM_FIX), false); //12M/2/15000 = 400Hz
      // pwm.analogWrite(PWM100, 500);  

      /*** Kalman Filter Initialize ***/
      my_cpf.setIMUError(AR_1C_UY, 400);
      /*** End of Kalman Filter Initialize***/

      SerialUSB.println("Data rate set to 400 Hz");
      Serial1.println("Data rate set to 400 Hz");
      // delay(100);
      break;
    }
    case 2: {
      // pwm.timer(1, 2, int(30000*PWM_FIX), false); //12M/2/30000 = 200Hz
      // pwm.analogWrite(PWM100, 500);  

      /*** Kalman Filter Initialize ***/
      my_cpf.setIMUError(AR_1C_UY, 200);
      /*** End of Kalman Filter Initialize***/

      SerialUSB.println("Data rate set to 200 Hz");
      Serial1.println("Data rate set to 200 Hz");
      // delay(100);
      break;
    }
    case 1: {
      // pwm.timer(1, 2, int(60000*PWM_FIX), false); //12M/2/60000 = 100Hz
      // pwm.analogWrite(PWM100, 500);  

      /*** Kalman Filter Initialize ***/
      my_cpf.setIMUError(AR_1C_UY, 100);
      /*** End of Kalman Filter Initialize***/

      SerialUSB.println("Data rate set to 100 Hz");
      Serial1.println("Data rate set to 100 Hz");
      // delay(100);
      break;
    }
    case 0: {
      // pwm.timer(1, 2, int(600000*PWM_FIX), false); //12M/2/600000 = 10Hz
      // pwm.analogWrite(PWM100, 500);  

      /*** Kalman Filter Initialize ***/
      my_cpf.setIMUError(AR_1C_UY, 10);
      /*** End of Kalman Filter Initialize***/

      SerialUSB.println("Data rate set to 10 Hz");
      Serial1.println("Data rate set to 10 Hz");
      // delay(100);
      break;
    }
    default:{
      // pwm.timer(1, 2, int(60000*PWM_FIX), false); //12M/2/60000 = 100Hz
      // pwm.analogWrite(PWM100, 500);  

      /*** Kalman Filter Initialize ***/
      my_cpf.setIMUError(AR_1C_UY, 100);
      /*** End of Kalman Filter Initialize***/

      SerialUSB.println("Data rate set to 100 Hz");
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
  SerialUSB.println(fog_ch);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Mod_freq, eeprom_obj_ptr->EEPROM_ADDR_MOD_FREQ, 100);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Wait_cnt, eeprom_obj_ptr->EEPROM_ADDR_WAIT_CNT, 20);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Err_avg, eeprom_obj_ptr->EEPROM_ADDR_ERR_AVG, 6);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Amp_H, eeprom_obj_ptr->EEPROM_ADDR_MOD_AMP_H, 9100);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Amp_L, eeprom_obj_ptr->EEPROM_ADDR_MOD_AMP_L, -9100);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Err_th, eeprom_obj_ptr->EEPROM_ADDR_ERR_TH, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Err_offset, eeprom_obj_ptr->EEPROM_ADDR_ERR_OFFSET, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Polarity, eeprom_obj_ptr->EEPROM_ADDR_POLARITY, 1);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Const_step, eeprom_obj_ptr->EEPROM_ADDR_CONST_STEP, 16384);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Fpga_Q, eeprom_obj_ptr->EEPROM_ADDR_FPGA_Q, 1);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Fpga_R, eeprom_obj_ptr->EEPROM_ADDR_FPGA_R, 6);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Gain1, eeprom_obj_ptr->EEPROM_ADDR_GAIN1, 5);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Gain2, eeprom_obj_ptr->EEPROM_ADDR_GAIN2, 7);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_FB_ON, eeprom_obj_ptr->EEPROM_ADDR_FB_ON, 1);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_DAC_gain, eeprom_obj_ptr->EEPROM_ADDR_DAC_GAIN, 77);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_Data_delay, eeprom_obj_ptr->EEPROM_ADDR_DATA_DELAY, 2000);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF0, eeprom_obj_ptr->EEPROM_ADDR_SF_0, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF1, eeprom_obj_ptr->EEPROM_ADDR_SF_1, 0x38d1b717);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF2, eeprom_obj_ptr->EEPROM_ADDR_SF_2, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF3, eeprom_obj_ptr->EEPROM_ADDR_SF_3, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF4, eeprom_obj_ptr->EEPROM_ADDR_SF_4, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF5, eeprom_obj_ptr->EEPROM_ADDR_SF_5, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF6, eeprom_obj_ptr->EEPROM_ADDR_SF_6, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF7, eeprom_obj_ptr->EEPROM_ADDR_SF_7, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF8, eeprom_obj_ptr-> EEPROM_ADDR_SF_8, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SF9, eeprom_obj_ptr->EEPROM_ADDR_SF_9, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_SFB, eeprom_obj_ptr->EEPROM_ADDR_SFB, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_CUTOFF, eeprom_obj_ptr->EEPROM_ADDR_CUTOFF, 0x44228000);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_TMIN, eeprom_obj_ptr->EEPROM_ADDR_TMIN, 0xC1A00000);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_TMAX, eeprom_obj_ptr->EEPROM_ADDR_TMAX, 0x42A00000);
  // BIAS temp. compensation
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_BIAS_COMP_T1, eeprom_obj_ptr->EEPROM_ADDR_BIAS_COMP_T1, 0);
  write_fog_parameter_to_eeprom(eeprom_obj_ptr->EEPROM_BIAS_COMP_T2, eeprom_obj_ptr->EEPROM_ADDR_BIAS_COMP_T2, 0x42200000);
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
  read_fog_parameter_from_eeprom(EEPROM_CALI_AX, 210 /*210 ~ 241, len = 32*/);
  read_fog_parameter_from_eeprom(EEPROM_CALI_AY, 210 /*210 ~ 241, len = 32*/+1);
  read_fog_parameter_from_eeprom(EEPROM_CALI_AZ, 210 /*210 ~ 241, len = 32*/+2);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A11, 210 /*210 ~ 241, len = 32*/+3);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A12, 210 /*210 ~ 241, len = 32*/+4);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A13, 210 /*210 ~ 241, len = 32*/+5);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A21, 210 /*210 ~ 241, len = 32*/+6);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A22, 210 /*210 ~ 241, len = 32*/+7);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A23, 210 /*210 ~ 241, len = 32*/+8);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A31, 210 /*210 ~ 241, len = 32*/+9);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A32, 210 /*210 ~ 241, len = 32*/+10);
  read_fog_parameter_from_eeprom(EEPROM_CALI_A33, 210 /*210 ~ 241, len = 32*/+11);
  read_fog_parameter_from_eeprom(EEPROM_CALI_GX, 210 /*210 ~ 241, len = 32*/+12);
  read_fog_parameter_from_eeprom(EEPROM_CALI_GY, 210 /*210 ~ 241, len = 32*/+13);
  read_fog_parameter_from_eeprom(EEPROM_CALI_GZ, 210 /*210 ~ 241, len = 32*/+14);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G11, 210 /*210 ~ 241, len = 32*/+15);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G12, 210 /*210 ~ 241, len = 32*/+16);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G13, 210 /*210 ~ 241, len = 32*/+17);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G21, 210 /*210 ~ 241, len = 32*/+18);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G22, 210 /*210 ~ 241, len = 32*/+19);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G23, 210 /*210 ~ 241, len = 32*/+20);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G31, 210 /*210 ~ 241, len = 32*/+21);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G32, 210 /*210 ~ 241, len = 32*/+22);
  read_fog_parameter_from_eeprom(EEPROM_CALI_G33, 210 /*210 ~ 241, len = 32*/+23);
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


void read_fog_parameter_from_eeprom_all(byte fog_ch)
{
  eeprom_obj *eeprom_obj_ptr;

  if(fog_ch==1) eeprom_obj_ptr = &eeprom_z;
  else if(fog_ch==2) eeprom_obj_ptr = &eeprom_x;
  else if(fog_ch==3) eeprom_obj_ptr = &eeprom_y;

  msg_out("read_fog_parameter_from_eeprom_all, ch");
  SerialUSB.println(fog_ch);

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
    case 0: {
      Serial1.println("Baudrate set to 230400");
      SerialUSB.println("Baudrate set to 230400");
      delay(100);
      Serial1.begin(230400);
      break;
    }
    case 1: {
      Serial1.println("Baudrate set to 115200");
      SerialUSB.println("Baudrate set to 115200");
      delay(100);
      Serial1.begin(115200);
      break;
    }
    case 2: {
      Serial1.println("Baudrate set to 9600");
      SerialUSB.println("Baudrate set to 9600");
      delay(100);
      Serial1.begin(9600);
      break;
    }
    case 3: {
      Serial1.println("Baudrate set to 4800");
      SerialUSB.println("Baudrate set to 4800");
      delay(100);
      Serial1.begin(4800);
      break;
    }
    default:{
      Serial1.println("Baudrate set to 230400");
      SerialUSB.println("Baudrate set to 230400");
      delay(100);
      Serial1.begin(230400);
      break;
    }
  }
}

void update_datarate(byte eeprom_var)
{
  SerialUSB.println("update_datarate");
  switch(eeprom_var)
  {
    case 3: {
      pwm.timer(1, 2, int(15000), false); //12M/2/15000 = 400Hz
      pwm.analogWrite(7, 500);
      SerialUSB.println("Data rate set to 400 Hz");
      Serial1.println("Data rate set to 400 Hz");
      delay(100);
      break;
    }
    case 2: {
      pwm.timer(1, 2, int(30000), false); //12M/2/30000 = 200Hz
      pwm.analogWrite(7, 500);
      SerialUSB.println("Data rate set to 200 Hz");
      Serial1.println("Data rate set to 200 Hz");
      delay(100);
      break;
    }
    case 1: {
      pwm.timer(1, 2, int(60000), false); //12M/2/60000 = 100Hz
      pwm.analogWrite(7, 500);
      SerialUSB.println("Data rate set to 100 Hz");
      Serial1.println("Data rate set to 100 Hz");
      delay(100);
      break;
    }
    case 0: {
      pwm.timer(1, 2, int(600000), false); //12M/2/600000 = 10Hz
      // pwm.timer(1, 2, int(375000), false); //12M/2/375000 = 16Hz
      pwm.analogWrite(7, 500);
      SerialUSB.println("Data rate set to 10 Hz");
      Serial1.println("Data rate set to 10 Hz");
      delay(100);
      break;
    }
    default:{
      pwm.timer(1, 2, int(60000), false); //12M/2/60000 = 100Hz
      pwm.analogWrite(7, 500);
      SerialUSB.println("Data rate set to 100 Hz");
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
  SerialUSB.println(fog_ch);

  PIG *sp;
  if(fog_ch==1) sp=&sp13;
  else if(fog_ch==2) sp=&sp14;
  else if(fog_ch=3) sp=&sp9;

  eeprom_obj *eeprom_ptr;

  if(fog_ch==1) eeprom_ptr = &eeprom_z;
  else if(fog_ch==2) eeprom_ptr = &eeprom_x;
  else if(fog_ch==3) eeprom_ptr = &eeprom_y;

  sp->sendCmd(myCmd_header, 0, myCmd_trailer, eeprom_ptr->EEPROM_Mod_freq);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 5, myCmd_trailer, eeprom_ptr->EEPROM_Wait_cnt);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 7, myCmd_trailer, eeprom_ptr->EEPROM_Err_avg);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 1, myCmd_trailer, eeprom_ptr->EEPROM_Amp_H);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 2, myCmd_trailer, eeprom_ptr->EEPROM_Amp_L);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 6, myCmd_trailer, eeprom_ptr->EEPROM_Err_th);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 3, myCmd_trailer, eeprom_ptr->EEPROM_Err_offset);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 4, myCmd_trailer, eeprom_ptr->EEPROM_Polarity);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 12, myCmd_trailer, eeprom_ptr->EEPROM_Const_step);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 13, myCmd_trailer, eeprom_ptr->EEPROM_Fpga_Q);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 14, myCmd_trailer, eeprom_ptr->EEPROM_Fpga_R);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 9, myCmd_trailer, eeprom_ptr->EEPROM_Gain1);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 10, myCmd_trailer, eeprom_ptr->EEPROM_Gain2);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 11, myCmd_trailer, eeprom_ptr->EEPROM_FB_ON);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 11, myCmd_trailer, 0);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 11, myCmd_trailer,eeprom_ptr-> EEPROM_FB_ON);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 50, myCmd_trailer, eeprom_ptr->EEPROM_DAC_gain);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 98, myCmd_trailer, eeprom_ptr->EEPROM_Data_delay);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 15, myCmd_trailer, eeprom_ptr->EEPROM_SF0);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 16, myCmd_trailer, eeprom_ptr->EEPROM_SF1);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 17, myCmd_trailer, eeprom_ptr->EEPROM_SF2);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 18, myCmd_trailer, eeprom_ptr->EEPROM_SF3);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 19, myCmd_trailer, eeprom_ptr->EEPROM_SF4);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 20, myCmd_trailer, eeprom_ptr->EEPROM_SF5);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 21, myCmd_trailer, eeprom_ptr->EEPROM_SF6);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 22, myCmd_trailer, eeprom_ptr->EEPROM_SF7);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 23, myCmd_trailer, eeprom_ptr->EEPROM_SF8);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 24, myCmd_trailer, eeprom_ptr->EEPROM_SF9);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 27, myCmd_trailer, eeprom_ptr->EEPROM_SFB);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 28, myCmd_trailer, eeprom_ptr->EEPROM_CUTOFF);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 25, myCmd_trailer, eeprom_ptr->EEPROM_TMIN);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 26, myCmd_trailer, eeprom_ptr->EEPROM_TMAX);
  delay(dly_time);
  //  Bias temp. compensation
  sp->sendCmd(myCmd_header, 29, myCmd_trailer, eeprom_ptr->EEPROM_BIAS_COMP_T1);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 30, myCmd_trailer, eeprom_ptr->EEPROM_BIAS_COMP_T2);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 31, myCmd_trailer, eeprom_ptr->EEPROM_SFB_1_SLOPE);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 32, myCmd_trailer, eeprom_ptr->EEPROM_SFB_1_OFFSET);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 33, myCmd_trailer, eeprom_ptr->EEPROM_SFB_2_SLOPE);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 34, myCmd_trailer, eeprom_ptr->EEPROM_SFB_2_OFFSET);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 35, myCmd_trailer, eeprom_ptr->EEPROM_SFB_3_SLOPE);
  delay(dly_time);
  sp->sendCmd(myCmd_header, 36, myCmd_trailer, eeprom_ptr->EEPROM_SFB_3_OFFSET);
  delay(dly_time);

  // Serial.println("Setting SP parameters done");
  msg_out("Setting SP parameters done");
}


void Blink_MCU_LED()
{
  bool A=0;
  for(int i=0; i<10; i++){
    digitalWrite(A2, A);
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
  sp->updateParameter(myCmd_header, 100, myCmd_trailer, 5, 0xCC);
  delay(10);
  flag = (SER->readStringUntil('\n'))[0];
  int t0=millis();
  while(!flag){

    if((millis()-t0)>500){
      times++;
      for(int i=0; i<255; i++) SER->read();//clear serial buffer
      sp->updateParameter(myCmd_header, 100, myCmd_trailer, 5, 0xCC);
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
      SerialUSB.println(times);
      Serial1.println(times);
      t0 = millis();
    }
  }
}

void reset_SYNC()
{
  sync_status = 0;
  digitalWrite(29 /*PA22*/, sync_status);
}

 void printVersion()
  {
    SerialUSB.print("\nVersion:");
    SerialUSB.println("MCU-GP-28-2-PD");
  }

void verify_output_fn(byte in)
{
  SerialUSB.println("\nverifying output fn........" );
  SerialUSB.print("Input function index: ");
  SerialUSB.println(in);
  if(!( in==0 |in==1 | in==2 | in==3 |
    in==4 | in==6 | in==5 ))
  {
    SerialUSB.println("verify output fn: fail");
    SerialUSB.println("output of function range, go to reset!");
    select_fn = 1;
    rst_fn_flag = 0;
    value = 4;
  }
  else {
    SerialUSB.println("verify output fn: pass");
  }
}

void verify_output_fn_reg_value(int in)
{
  SerialUSB.println("\nverifying output fn reg value........" );
  SerialUSB.print("Input reg value: ");
  SerialUSB.println(in);
  if(!( in==1 |in==2 | in==4 | in==5 |
    in==3 ))
    {
      SerialUSB.println("verify output fn reg value: fail");
      SerialUSB.println("output of reg value range, go to reset!");
      select_fn = 1;
      rst_fn_flag = 0;
      value = 4;
    }
  else {
    SerialUSB.println("verify output fn: pass");
  }
}

void verify_select_fn(int in)
{
  SerialUSB.println("\nverifying select fn........" );
  SerialUSB.print("Input select_fn: ");
  SerialUSB.println(in);
  if(!( in==0 |in==1 | in==2 | in==3 | in==9 |
    in==4 |in==5 |in==6 |in==7 |in==8 ))
    {
      SerialUSB.println("verify select_fn: fail");
      SerialUSB.println("output of select_fn range, go to reset!");
      select_fn = 1;
      rst_fn_flag = 0;
      value = 4;
    }
  else {
    SerialUSB.println("verify select_fn: pass");
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
    SerialUSB.print(acc[0]);
    SerialUSB.print(", ");
    SerialUSB.print(acc[1]);
    SerialUSB.print(", ");
    SerialUSB.print(acc[2]);
    SerialUSB.print(", ");
    SerialUSB.print(gyro[0]);
    SerialUSB.print(", ");
    SerialUSB.print(gyro[1]);
    SerialUSB.print(", ");
    SerialUSB.println(gyro[2]);
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
