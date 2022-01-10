#include "adxl355.h"
#include <Arduino_LSM6DS3.h>
#include "pig_v2.h"
#include "IMU_PIG_DEFINE.h"


/*** global var***/
int pin_scl_mux = 12;
bool trig_status[2] = {0, 0};
unsigned int t_new, t_old=0;

/*** serial data from PC***/
byte rx_cnt = 0, cmd;
unsigned int value;
bool cmd_complete;

/*** output mode flag***/
byte mux_flag;

/*** acq fog flag***/
bool run_fog_flag;

/*** select running fn flag***/
byte select_fn;
bool fn_lock;


// typedef void (*fn_ptr) (byte &, unsigned int);
// fn_ptr output_fn;
Adxl355 adxl355(pin_scl_mux);
PIG pig_v2;

void temp_idle(byte &, unsigned int );
void fn_rst(byte &, unsigned int );
void acq_fog(byte &, unsigned int );
void acq_imu(byte &, unsigned int );

void setup() {
	Serial1.begin(115200);
	IMU.begin();
	adxl355.init();
	pinMode(SYS_TRIG, INPUT);
	/*** var initialization***/
	cmd_complete = 0;
	mux_flag = MUX_ESCAPE;
	select_fn = SEL_DEFAULT;
	run_fog_flag = 0;
	fn_lock = 1;
}

void loop() {
	byte *a_adxl355, acc[9];
	int wx_nano33, wy_nano33, wz_nano33;
	int ax_nano33, ay_nano33, az_nano33;
	
	getCmdValue(cmd, value, cmd_complete);
	cmd_mux(cmd_complete, cmd, mux_flag);
	parameter_setting(mux_flag, cmd, value);
	output_mode_setting(mux_flag, cmd, select_fn, fn_lock);
	output_fn(cmd, value, fn_lock);
}

void print_nano33GyroData(int wx, int wy, int wz)
{
	t_new = micros();
	Serial.print(t_new - t_old);
	Serial.print('\t');
	Serial.print((float)wx*NANO33_GYRO);
	Serial.print('\t');
	Serial.print((float)wy*NANO33_GYRO);
	Serial.print('\t');
	Serial.println((float)wz*NANO33_GYRO);
	t_old = t_new;
}

void print_nano33XlmData(int ax, int ay, int az)
{
	t_new = micros();
	Serial.print(t_new - t_old);
	Serial.print('\t');
	Serial.print((float)ax*NANO33_XLM);
	Serial.print('\t');
	Serial.print((float)ay*NANO33_XLM);
	Serial.print('\t');
	Serial.println((float)az*NANO33_XLM);
	t_old = t_new;
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

void getCmdValue(byte &uart_cmd, unsigned int &uart_value, bool &uart_complete)
{
	if (Serial.available()){
		switch(rx_cnt) {
			case 0: {
				uart_cmd = Serial.read();
				rx_cnt = 1;
				break;
			}
			case 1: {
				uart_value = Serial.read()<<24;
				rx_cnt = 2;
				break;
			}
			case 2: {
				uart_value |= Serial.read()<<16;
				rx_cnt = 3;
				break;
			}
			case 3: {
				uart_value |= Serial.read()<<8;
				rx_cnt = 4;
				break;
			}
			case 4: {
				uart_value |= Serial.read();
				rx_cnt = 0;
				uart_complete = 1;
				// printVal_0("rx_cmd: ", uart_cmd);
				// printVal_0("rx_val: ", uart_value);
				break;
			}
		}
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

void parameter_setting(byte &mux_flag, byte cmd, unsigned int value) 
{
	if(mux_flag == MUX_PARAMETER)
	{
		mux_flag = MUX_ESCAPE;
		switch(cmd) {
			case CMD_FOG_MOD_FREQ: {pig_v2.sendCmd(MOD_FREQ_ADDR, value);break;}
			case CMD_FOG_MOD_AMP_H: {pig_v2.sendCmd(MOD_AMP_H_ADDR, value);break;}
			case CMD_FOG_MOD_AMP_L: {pig_v2.sendCmd(MOD_AMP_L_ADDR, value);break;}
			case CMD_FOG_ERR_OFFSET: {pig_v2.sendCmd(ERR_OFFSET_ADDR, value);break;}
			case CMD_FOG_POLARITY: {pig_v2.sendCmd(POLARITY_ADDR, value);break;}
			case CMD_FOG_WAIT_CNT:{pig_v2.sendCmd(WAIT_CNT_ADDR, value);break;}
			case CMD_FOG_ERR_TH: {pig_v2.sendCmd(ERR_TH_ADDR, value);break;}
			case CMD_FOG_ERR_AVG: {pig_v2.sendCmd(ERR_AVG_ADDR, value);break;}
			case CMD_FOG_TIMER_RST: {pig_v2.sendCmd(TIMER_RST_ADDR, value);break;}
			case CMD_FOG_GAIN1: {pig_v2.sendCmd(GAIN1_ADDR, value);break;}
			case CMD_FOG_GAIN2: {pig_v2.sendCmd(GAIN2_ADDR, value);break;}
			case CMD_FOG_FB_ON: {pig_v2.sendCmd(FB_ON_ADDR, value);break;}
			case CMD_FOG_CONST_STEP: {pig_v2.sendCmd(CONST_STEP_ADDR, value);break;}
			case CMD_FOG_FPGA_Q: {pig_v2.sendCmd(FPGA_Q_ADDR, value);break;}
			case CMD_FOG_FPGA_R: {pig_v2.sendCmd(FPGA_R_ADDR, value);break;}
			case CMD_FOG_DAC_GAIN: {pig_v2.sendCmd(DAC_GAIN_ADDR, value);break;}
			case CMD_FOG_INT_DELAY: {pig_v2.sendCmd(DATA_INT_DELAY_ADDR, value);break;}
			case CMD_FOG_OUT_START: {pig_v2.sendCmd(DATA_OUT_START_ADDR, value);break;}
			default: break;
		}
	}
}

void output_mode_setting(byte &mux_flag, byte mode, byte &select_fn, bool &fn_lock)
{
	if(mux_flag == MUX_OUTPUT)
	{
		mux_flag = MUX_ESCAPE;
		switch(mode) {
			case MODE_RST: {
				// output_fn = fn_rst;
				// fn = fn_rst;
				select_fn = SEL_RST;
				// printVal_0("output_mode_setting: ", MODE_RST);
				break;
			}
			case MODE_FOG: {
				// output_fn = acq_fog;
				select_fn = SEL_FOG_1;
				// printVal_0("output_mode_setting: ", MODE_FOG);
				
				break;
			}
			case MODE_IMU: {
				// output_fn = acq_imu;
				select_fn = SEL_IMU;
				// printVal_0("output_mode_setting: ", MODE_IMU);
				break;
			}
			case MODE_EQ: {
				// output_fn = temp_idle;
				select_fn = SEL_EQ;
				// printVal_0("output_mode_setting: ", MODE_EQ);
				break;
			}
			default: break;
		}
		
		fn_lock = 0;
	}
	// printVal_0("select_fn: ", select_fn);
}

void output_fn(byte select_fn, unsigned int CTRLREG, bool &fn_lock)
{
	switch(select_fn) {
		case MODE_RST: fn_rst(fn_lock, CTRLREG); break;
		case MODE_FOG: acq_fog(fn_lock, CTRLREG); break;
		case MODE_IMU: acq_imu(fn_lock, CTRLREG); break;
		case MODE_EQ:  temp_idle(fn_lock, CTRLREG); break;
	}
}

void temp_idle(bool &fn_lock, unsigned int CTRLREG)
{
	// clear_SEL_EN(select_fn);
	// printVal_0("run fn temp_idle ", 99);
}

void fn_rst(bool &fn_lock, unsigned int CTRLREG)
{
	if(!fn_lock) {
		fn_lock = 1;
		switch(CTRLREG) {
			case REFILL_SERIAL1: {
				for(int i=0; i<256; i++) Serial1.read();
				break;
			}
			default: break;
		}
		
	}
	// clear_SEL_EN(select_fn);
}

void acq_fog(bool &fn_lock, unsigned int CTRLREG)
{
	byte data[16]; 
	unsigned int time, PD_T;
	int err, step;
	
	if(!fn_lock)
	{
		fn_lock = 1;
		switch(CTRLREG) {
			case INT_SYNC: {
				pig_v2.sendCmd(DATA_OUT_START_ADDR, 1);
				run_fog_flag = 1;
				printVal_0("INT_SYNC ", 99);
				break;
			}
			case EXT_SYNC: {
				pig_v2.sendCmd(DATA_OUT_START_ADDR, 2);
				run_fog_flag = 1;
				printVal_0("EXT_SYNC ", 99);
				break;
			}
			case STOP_SYNC: {
				pig_v2.sendCmd(DATA_OUT_START_ADDR, 0);
				run_fog_flag = 0;
				printVal_0("STOP_SYNC ", 99);
				break;
			}
			default: break;
		}
		printVal_0("in acq_fog ", 99);
	}
	if(run_fog_flag){
		pig_v2.readData(data);
		Serial.write(CHECK_BYTE);
		Serial.write(CHECK_BYTE3);
		Serial.write(data, 16);
		Serial.write(CHECK_BYTE2);
		
	}
	// printVal_0("run fn acq_fog ", 99);
	// clear_SEL_EN(select_fn);	
}

void acq_imu(bool &fn_lock, unsigned int CTRLREG)
{
	byte acc[9];
	int wx_nano33, wy_nano33, wz_nano33;
	int ax_nano33, ay_nano33, az_nano33;
	
	trig_status[0] = digitalRead(SYS_TRIG);
	if(trig_status[0] & ~trig_status[1]) {
		adxl355.readData(acc);
		IMU.readGyroscope(wx_nano33, wy_nano33, wz_nano33);
		IMU.readAcceleration(ax_nano33, ay_nano33, az_nano33);
		acq_fog(fn_lock, CTRLREG);
		// print_adxl355Data(acc);
		// print_nano33GyroData(wx_nano33, wy_nano33, wz_nano33);
		// print_nano33XlmData(ax_nano33, ay_nano33, az_nano33);
	}
	trig_status[1] = trig_status[0];
	// clear_SEL_EN(select_fn);
}

// void clear_SEL_EN(byte &select_fn)
// {
	// select_fn = SEL_DEFAULT;
// }

// void sendCmd(byte addr, unsigned int value)
// {
	// Serial1.write(addr);
	// Serial1.write(value>>24 & 0xFF);
	// Serial1.write(value>>16 & 0xFF);
	// Serial1.write(value>>8 & 0xFF);
	// Serial1.write(value & 0xFF);
	// delay(1);
// }