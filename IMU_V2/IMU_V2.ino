#include "adxl355.h"
#include <Arduino_LSM6DS3.h>
#include "pig_v2.h"

/*** adxl355 conversion factor***/
#define ADXL355_8G 0.0000156
#define ADXL355_4G 0.0000078
#define ADXL355_2G 0.0000039

/*** Nano33 conversion facto***/
#define NANO33_XLM 0.000122 // +/- 4g, 4/32768
#define NANO33_GYRO 0.00763 // +/- 250dps, 250/32768

/*** trig pin***/
#define SYS_TRIG 14

/*** PIG CMD map: *********
0~7 for output mode setting,
8~255 for parameter setting
***************************/
#define MUX_OUTPUT		0
#define MUX_PARAMETER	1
#define MUX_ESCAPE		2  

#define MODE_RST 	0
#define MODE_FOG	1
#define MODE_IMU	2
#define MODE_EQ		3

#define CMD_FOG_MOD_FREQ	8
#define CMD_FOG_MOD_AMP_H	9
#define CMD_FOG_MOD_AMP_L	10
#define CMD_FOG_ERR_OFFSET	11
#define CMD_FOG_POLARITY	12
#define CMD_FOG_WAIT_CNT	13
#define CMD_FOG_ERR_TH		14
#define CMD_FOG_ERR_AVG		15
#define CMD_FOG_TIMER_RST	16
#define CMD_FOG_GAIN1		17
#define CMD_FOG_GAIN2		18
#define CMD_FOG_FB_ON		19
#define CMD_FOG_CONST_STEP	20
#define CMD_FOG_FPGA_Q		21
#define CMD_FOG_FPGA_R		22
#define CMD_FOG_DAC_GAIN	23
#define CMD_FOG_INT_DELAY	24
#define CMD_FOG_OUT_START	25

#define MOD_FREQ_ADDR			0
#define MOD_AMP_H_ADDR  		1
#define MOD_AMP_L_ADDR  		2
#define ERR_OFFSET_ADDR 		3
#define POLARITY_ADDR  			4
#define WAIT_CNT_ADDR  			5
#define ERR_TH_ADDR  			6
#define ERR_AVG_ADDR  			7
#define TIMER_RST_ADDR  		8
#define GAIN1_ADDR  			9
#define GAIN2_ADDR  			10
#define FB_ON_ADDR  			11
#define CONST_STEP_ADDR  		12
#define FPGA_Q_ADDR				13
#define FPGA_R_ADDR  			14
#define DAC_GAIN_ADDR  			50
#define DATA_INT_DELAY_ADDR 	98
#define DATA_OUT_START_ADDR		99
/*** PIG SYNC mode***/
#define INT_SYNC	1
#define EXT_SYNC 	2
#define STOP_SYNC 	3

#define CHECK_BYTE		170
#define CHECK_BYTE2		171
#define CHECK_BYTE3		172

/*** global var***/
int pin_scl_mux = 12;
bool trig_status[2] = {0, 0};
unsigned int t_new, t_old=0;

/*** serial data from PC***/
byte rx_cnt = 0, cmd;
unsigned int value;
bool cmd_complete;

/*** output mode flag***/
byte mux_flag = 0;
byte mode_flag = 0;

/*** acq fog flag***/
bool run_fog_flag = 0;
bool output_fn_lock = 1;

typedef void (*fn_ptr) (bool &, unsigned int &);
fn_ptr output_fn;
Adxl355 adxl355(pin_scl_mux);
PIG pig_v2;


void setup() {
	Serial1.begin(115200);
	IMU.begin();
	adxl355.init();
	pinMode(SYS_TRIG, INPUT);
	
}

void loop() {
	byte *a_adxl355, acc[9];
	int wx_nano33, wy_nano33, wz_nano33;
	int ax_nano33, ay_nano33, az_nano33;

	
	getCmdValue(cmd, value, cmd_complete);
	cmd_mux(cmd_complete, cmd, mux_flag, value);
	parameter_setting(mux_flag, cmd, value);
	output_mode_setting(mux_flag, cmd, output_fn_lock);
	output_fn(output_fn_lock, value);
	
	/***
	trig_status[0] = digitalRead(SYS_TRIG);
	if(trig_status[0] & ~trig_status[1]) {
		// adxl355.printRegAll();
		a_adxl355 = adxl355.readData(acc);
		IMU.readGyroscope(wx_nano33, wy_nano33, wz_nano33);
		IMU.readAcceleration(ax_nano33, ay_nano33, az_nano33);
		print_adxl355Data(a_adxl355);
		// print_nano33GyroData(wx_nano33, wy_nano33, wz_nano33);
		// print_nano33XlmData(ax_nano33, ay_nano33, az_nano33);
	}
	trig_status[1] = trig_status[0];
	***/
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
				break;
			}
		}
	}
}

void cmd_mux(bool &cmd_complete, byte cmd, byte &mux_flag, unsigned int val)
{
	if(cmd_complete)
	{
		cmd_complete = 0;
		if(cmd >7) mux_flag = MUX_PARAMETER; 
		else mux_flag = MUX_OUTPUT;
	}
}

void sendCmd(byte addr, unsigned int value)
{
	Serial1.write(addr);
	Serial1.write(value>>24 & 0xFF);
	Serial1.write(value>>16 & 0xFF);
	Serial1.write(value>>8 & 0xFF);
	Serial1.write(value & 0xFF);
	delay(1);
}

void parameter_setting(byte &mux_flag, byte cmd, unsigned int value) 
{
	if(mux_flag == MUX_PARAMETER)
	{
		mux_flag = MUX_ESCAPE;
		switch(cmd) {
			case CMD_FOG_MOD_FREQ: {sendCmd(MOD_FREQ_ADDR, value);break;}
			case CMD_FOG_MOD_AMP_H: {sendCmd(MOD_AMP_H_ADDR, value);break;}
			case CMD_FOG_MOD_AMP_L: {sendCmd(MOD_AMP_L_ADDR, value);break;}
			case CMD_FOG_ERR_OFFSET: {sendCmd(ERR_OFFSET_ADDR, value);break;}
			case CMD_FOG_POLARITY: {sendCmd(POLARITY_ADDR, value);break;}
			case CMD_FOG_WAIT_CNT:{sendCmd(WAIT_CNT_ADDR, value);break;}
			case CMD_FOG_ERR_TH: {sendCmd(ERR_TH_ADDR, value);break;}
			case CMD_FOG_ERR_AVG: {sendCmd(ERR_AVG_ADDR, value);break;}
			case CMD_FOG_TIMER_RST: {sendCmd(TIMER_RST_ADDR, value);break;}
			case CMD_FOG_GAIN1: {sendCmd(GAIN1_ADDR, value);break;}
			case CMD_FOG_GAIN2: {sendCmd(GAIN2_ADDR, value);break;}
			case CMD_FOG_FB_ON: {sendCmd(FB_ON_ADDR, value);break;}
			case CMD_FOG_CONST_STEP: {sendCmd(CONST_STEP_ADDR, value);break;}
			case CMD_FOG_FPGA_Q: {sendCmd(FPGA_Q_ADDR, value);break;}
			case CMD_FOG_FPGA_R: {sendCmd(FPGA_R_ADDR, value);break;}
			case CMD_FOG_DAC_GAIN: {sendCmd(DAC_GAIN_ADDR, value);break;}
			case CMD_FOG_INT_DELAY: {sendCmd(DATA_INT_DELAY_ADDR, value);break;}
			case CMD_FOG_OUT_START: {sendCmd(DATA_OUT_START_ADDR, value);break;}
			default: break;
		}
	}
}

void output_mode_setting(byte &mux_flag, byte mode, bool &output_fn_lock)
{
	if(mux_flag == MUX_OUTPUT)
	{
		mux_flag = MUX_ESCAPE;
		output_fn_lock = 0;
		switch(mode) {
			case MODE_RST: {
				output_fn = temp_idle;
				break;
			}
			case MODE_FOG: {
				output_fn = acq_fog;
				break;
			}
			case MODE_IMU: {
				output_fn = acq_imu;
				break;
			}
			case MODE_EQ: {
				output_fn = temp_idle;
				break;
			}
			default: break;
		}
	}
}

void temp_idle(bool &output_fn_lock, unsigned int &val)
{}

void acq_fog(bool &output_fn_lock, unsigned int &syncFlag)
{
	byte data[16]; 
	unsigned int time, PD_T;
	int err, step;
	
	if(output_fn_lock == 0)
	{
		output_fn_lock = 1;
		switch(syncFlag) {
			case INT_SYNC: {
				sendCmd(DATA_OUT_START_ADDR, 1);
				syncFlag = 0; 
				run_fog_flag = 1;
				break;
			}
			case EXT_SYNC: {
				sendCmd(DATA_OUT_START_ADDR, 2);
				syncFlag = 0;
				run_fog_flag = 1;
				break;
			}
			case STOP_SYNC: {
				sendCmd(DATA_OUT_START_ADDR, 0);
				syncFlag = 0;
				run_fog_flag = 0;
				break;
			}
			default: break;
		}
	}
	
	if(run_fog_flag) pig_v2.readData(data);
	Serial.write(CHECK_BYTE);
	Serial.write(CHECK_BYTE3);
	Serial.write(data, 16);
	Serial.write(CHECK_BYTE2);
}

void acq_imu(bool &output_fn_lock, unsigned int &syncFlag)
{
	byte acc[9];
	int wx_nano33, wy_nano33, wz_nano33;
	int ax_nano33, ay_nano33, az_nano33;
	
	trig_status[0] = digitalRead(SYS_TRIG);
	if(trig_status[0] & ~trig_status[1]) {
		adxl355.readData(acc);
		IMU.readGyroscope(wx_nano33, wy_nano33, wz_nano33);
		IMU.readAcceleration(ax_nano33, ay_nano33, az_nano33);
		acq_fog(output_fn_lock, syncFlag);
		// print_adxl355Data(acc);
		// print_nano33GyroData(wx_nano33, wy_nano33, wz_nano33);
		// print_nano33XlmData(ax_nano33, ay_nano33, az_nano33);
	}
	trig_status[1] = trig_status[0];
	
}