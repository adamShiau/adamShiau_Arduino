#include "adxl355.h"
#include <Arduino_LSM6DS3.h>
#include "pig_v2.h"
#include "IMU_PIG_DEFINE.h"

// #define TESTMODE

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
// bool fn_lock;


typedef void (*fn_ptr) (byte &, unsigned int);
fn_ptr output_fn;
Adxl355 adxl355(pin_scl_mux);
PIG pig_v2;


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
	output_fn = temp_idle;
}

void loop() {
	byte *a_adxl355, acc[9];
	int wx_nano33, wy_nano33, wz_nano33;
	int ax_nano33, ay_nano33, az_nano33;
	
	getCmdValue(cmd, value, cmd_complete);
	cmd_mux(cmd_complete, cmd, mux_flag);
	parameter_setting(mux_flag, cmd, value);
	output_mode_setting(mux_flag, cmd, select_fn);
	output_fn(select_fn, value);
	#ifdef TESTMODE
		delay(10);
	#endif
	
}

void printAdd(char name[], void* addr)
{
	Serial.print(name);
	Serial.print(": ");
	Serial.println((unsigned int)addr, HEX);
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
			case MODE_EQ: {
				output_fn = temp_idle;
				select_fn = SEL_EQ;
				break;
			}
			default: break;
		}
		
	}
}

void temp_idle(byte &select_fn, unsigned int CTRLREG)
{
	clear_SEL_EN(select_fn);
	delay(100);
}

void fn_rst(byte &select_fn, unsigned int CTRLREG)
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

void acq_fog(byte &select_fn, unsigned int CTRLREG)
{
	byte data[16]; 
	
	if(select_fn&SEL_FOG_1)
	{
		run_fog_flag = pig_v2.setSyncMode(CTRLREG);
	}
	if(run_fog_flag){
		#ifdef TESTMODE
			pig_v2.readFakeData(data);
		#else 
			pig_v2.readData(data);
		#endif
		Serial.write(CHECK_BYTE);
		Serial.write(CHECK_BYTE3);
		Serial.write(data, 16);
		Serial.write(CHECK_BYTE2);
	}
	clear_SEL_EN(select_fn);	
}

void acq_imu_fake(byte &select_fn, unsigned int CTRLREG)
{
	byte acc[9], data[16]; 
	int wx_nano33, wy_nano33, wz_nano33;
	int ax_nano33, ay_nano33, az_nano33;
	
	if(select_fn&SEL_IMU) 
	{
		run_fog_flag = pig_v2.setSyncMode(CTRLREG);
	}
	adxl355.readFakeData(acc);
	IMU.readFakeGyroscope(wx_nano33, wy_nano33, wz_nano33);
	IMU.readFakeAcceleration(ax_nano33, ay_nano33, az_nano33);
	if(run_fog_flag){
		pig_v2.readFakeData(data);
		Serial1.write(CHECK_BYTE);
		Serial1.write(CHECK_BYTE3);
		Serial1.write(data, 16);
		Serial1.write(acc, 9);
		Serial1.write(wx_nano33);
		Serial1.write(wy_nano33);
		Serial1.write(ax_nano33);
		Serial1.write(ay_nano33);
		Serial1.write(az_nano33);
		Serial1.write(CHECK_BYTE2);
	}
	
	clear_SEL_EN(select_fn);
}

void acq_imu(byte &select_fn, unsigned int CTRLREG)
{
	byte adxl355_a[9], fog[16], nano33_w[6], nano33_a[6] ;
	// int wx_nano33, wy_nano33, wz_nano33;
	// int ax_nano33, ay_nano33, az_nano33;
	
	if(select_fn&SEL_IMU) {
		run_fog_flag = pig_v2.setSyncMode(CTRLREG);
		// printVal_0("run_fog_flag", run_fog_flag);
	}
	trig_status[0] = digitalRead(SYS_TRIG);
	if((trig_status[0] & ~trig_status[1]) && run_fog_flag) {
		adxl355.readData(adxl355_a);
		IMU.readGyroscope(nano33_w);
		IMU.readAcceleration(nano33_a);
		pig_v2.readData(fog);
		
		Serial.write(CHECK_BYTE);
		Serial.write(CHECK_BYTE3);
		Serial.write(fog, 16);
		Serial.write(adxl355_a, 9);
		Serial.write(nano33_w, 6);
		Serial.write(nano33_a, 6);
		Serial.write(CHECK_BYTE2);
	}
	trig_status[1] = trig_status[0];
	clear_SEL_EN(select_fn);
}

void clear_SEL_EN(byte &select_fn)
{
	select_fn = SEL_DEFAULT;
}