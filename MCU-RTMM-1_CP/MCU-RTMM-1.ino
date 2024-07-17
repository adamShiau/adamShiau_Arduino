#include "rtmm.h"

/***
 * This library is created on 7/16/2024 by Adam,
 * 
 */


// pin definition
#define RL_1 12 // left  relay, D12
#define RL_2 A0 // right relay, A0'

#define LEFT_RLY  RL_1
#define RIGHT_RLY RL_2




/*** global var***/
// int pin_scl_mux = 17;
// bool trig_status[2] = {0, 0};
unsigned int t_new, t_old=0;



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

void setup() {

    myWDT_init(); //disable all WDT
   /*** pwm ***/
    pwm_init();

  /*** ADC setting***/
  analogReadResolution(12); //set resolution
  pinMode(ADC_ASE_TACT, INPUT);

  pinMode(PIG_SYNC, OUTPUT); 
  digitalWrite(PIG_SYNC, sync_status);

  pinMode(MCU_LED, OUTPUT);
  digitalWrite(MCU_LED, HIGH);
  
  pinMode(nCONFIG, OUTPUT);

  myUART_init();
  // myI2C_init();
  // mySPI_init();

  //Re-configure FPGA when system reset on MCU
  // digitalWrite(nCONFIG, LOW);
  // delay(50);
  // digitalWrite(nCONFIG, HIGH);
  // delay(500);

  // byte FPGA_wakeup_flag = 0; 
  // Wait_FPGA_Wakeup(FPGA_wakeup_flag, 2);
  // Blink_MCU_LED();

  // rescue_mode();
  // parameter_init();
  // Blink_MCU_LED();
	
  IMU.init(); //setting MEMS IMU parameters 
  
    
	/*** var initialization***/
	mux_flag = MUX_ESCAPE; 
	select_fn = SEL_DEFAULT; 	
	run_fog_flag = 0;
	output_fn = temp_idle;

  /***read eeprom current status*/
//   eeprom.Read(EEPROM_ADDR_FOG_STATUS, &fog_op_status);

//   Serial.print("fog_op_status: ");
//   Serial.println(fog_op_status);
//  if(fog_op_status==1) // disconnected last time, send cmd again
//   {
//     Serial.println("\n-------AUTO RST---------");

//     eeprom.Parameter_Read(EEPROM_ADDR_OUTPUT_FN, my_f.bin_val);// read output function index from eeprom
//     rst_fn_flag = my_f.int_val; 
//     // rst_fn_flag = 10; //test output fn output of range 
//     verify_output_fn(rst_fn_flag);// check if output function is valid
//     PRINT_OUTPUT_MODE(rst_fn_flag);
    
//     eeprom.Parameter_Read(EEPROM_ADDR_REG_VALUE, my_f.bin_val);//read reg value of output function
//     uart_value = my_f.int_val;
//     // value = 10; //test fn reg output of range 
//     verify_output_fn_reg_value(uart_value);
//     PRINT_OUTPUT_REG(uart_value);

//     eeprom.Parameter_Read(EEPROM_ADDR_SELECT_FN, my_f.bin_val);
//     select_fn = my_f.int_val;
//     // select_fn = 10; //test select_fn output of range 
//     verify_select_fn(select_fn);
//     PRINT_SELECT_FN(select_fn);

//     fog_ch = 2;
//   }
//   PRINT_MUX_FLAG(mux_flag);
  printVersion();
  t_start = millis();
}

void loop() {

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

		mux_flag = MUX_ESCAPE;
		switch(cmd) {

      case CMD_FPGA_VERSION: {
        msg_out(MCU_VERSION);
        // Serial.print(MCU_VERSION);
        // Serial1.print(MCU_VERSION);
        // Serial.print(',');
        // Serial1.print(',');
        // Serial.println(fpga_version);
        // Serial1.println(fpga_version);
        break;
      }
      case CMD_LEFT_RLY: {
        msg_out("Select CMD_LEFT_RLY");

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
			case MODE_IMU: {
				output_fn = acq_imu; 
				select_fn = SEL_IMU;
        rst_fn_flag = MODE_IMU;
				break;
			}

      default: break;
      }
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

void acq_imu(byte &select_fn, unsigned int value, byte ch)
{
  my_acc_t my_memsGYRO;
  my_float_t pd_temp;
  static my_float_t myfog_GYRO;
  static my_acc_t my_memsXLM, my_memsXLM_cali;
  static my_acc_t my_GYRO, my_GYRO_cali, my_att;
  float att_dt_f;
  static uint32_t att_dt;

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
        my_ekf.setInitOri(0,0,0);
        LC.reset();
        disableWDT();
        disable_EXT_WDT(EXT_WDT_EN);
      break;

      default:
      break;
    }
    t_previous = millis();
    att_dt = micros();
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
      IMU.Get_G_Axes_rps_f(my_memsGYRO.float_val);// get mems GYRO data in radian/s
      my_GYRO.float_val[0] = my_memsGYRO.float_val[0]; 
      my_GYRO.float_val[1] = my_memsGYRO.float_val[1];
      my_GYRO.float_val[2] = myfog_GYRO.float_val * DEG_TO_RAD;
      /*** ------mis-alignment calibration gyro raw data -----***/
      gyro_cali(my_GYRO_cali.float_val, my_GYRO.float_val);
      if(data_cnt >= DELAY_CNT)
      {
        LC.update(my_GYRO_cali.float_val); // substract gyro bias offset
      }
      print_imu_data(false, my_memsXLM_cali.float_val, my_GYRO_cali.float_val);


      memcpy(imu_data, KVH_HEADER, 4);
      memcpy(imu_data+4, my_GYRO_cali.bin_val, 12);//wx, wy, wz
      memcpy(imu_data+16, my_memsXLM_cali.bin_val, 12);//ax, ay, az
      memcpy(imu_data+28, reg_fog+12, 4);// PD temp
      memcpy(imu_data+32, mcu_time.bin_val, 4);
      memcpy(imu_data+36, my_att.bin_val, 12);
      myCRC.crc_32(imu_data, 48, CRC32);

      free(imu_data);
      
      #ifdef UART_RS422_CMD
      if(data_cnt >= DELAY_CNT)
      {
        Serial1.write(KVH_HEADER, 4);
        Serial1.write(my_GYRO_cali.bin_val, 12);   //wx, wy, wz
        Serial1.write(my_memsXLM_cali.bin_val, 12);//ax, ay, az
        Serial1.write(reg_fog+12, 4);         // PD temp
        Serial1.write(mcu_time.bin_val, 4);
        Serial1.write(my_att.bin_val, 12);
        Serial1.write(CRC32, 4);
      }
      #endif  
      resetWDT(); 
      reset_EXT_WDI(WDI);

      att_dt_f = (float)(micros()-att_dt)*1e-6; // unit:sec
      att_dt = micros();
      my_ekf.run(att_dt_f, my_GYRO_cali.float_val, my_memsXLM_cali.float_val);
      my_ekf.getEularAngle(my_att.float_val); //raw data -> att, pitch, row, yaw 
    }
	}
	clear_SEL_EN(select_fn);
}




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