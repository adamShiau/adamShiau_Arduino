/*** VERSION */
#define MCU_VERSION "MCU-GP-11-PD"

/*** adxl355 conversion factor***/
#define ADXL355_8G 0.0000156
#define ADXL355_4G 0.0000078
#define ADXL355_2G 0.0000039

/*** Nano33 conversion facto***/
#define NANO33_XLM 0.000122 // +/- 4g, 4/32768
#define NANO33_GYRO 0.00763 // +/- 250dps, 250/32768

/*** trig pin***/
#define SYS_TRIG 26

/*** PWM AS SINC***/
#define PWM_SYNC

/*** ADC PIN***/
#define ADC_CONV 3.3/4096.0
#define ADC_VIN     18
#define ADC_PD_DC   19
#define ADC_ASE_TACT 6
#define ADC_ASE_VPD 17

/*** IMU_PIG Serial CMD map: *********
0~7 for output mode setting,
8~100 for parameter setting
***************************/
#define MODE_RST 	        0
#define MODE_FOG	        1
#define MODE_IMU	        2
#define MODE_EQ		        4
#define MODE_FOG_HP_TEST	3
#define MODE_IMU_MEMS		5
#define MODE_VERSION_TEST   6
#define MODE_IMU_MEMS_GPS   7

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

#define CMD_FPGA_VERSION	101

#define MUX_OUTPUT		    0
#define MUX_PARAMETER	    1
#define MUX_ESCAPE		    2

#define MOD_FREQ_ADDR		0
#define MOD_AMP_H_ADDR  	1
#define MOD_AMP_L_ADDR  	2
#define ERR_OFFSET_ADDR 	3
#define POLARITY_ADDR  		4
#define WAIT_CNT_ADDR  		5
#define ERR_TH_ADDR  		6
#define ERR_AVG_ADDR  		7
#define TIMER_RST_ADDR  	8
#define GAIN1_ADDR  		9
#define GAIN2_ADDR  		10
#define FB_ON_ADDR  		11
#define CONST_STEP_ADDR  	12
#define FPGA_Q_ADDR			13
#define FPGA_R_ADDR  		14
#define DAC_GAIN_ADDR  		50
#define DATA_INT_DELAY_ADDR	98
#define DATA_OUT_START_ADDR	99

#define FPGA_VERSION_ADDR	101

/*** UART port***/
// #define UART_SERIAL_5_CMD
// #define UART_USB_CMD
#define UART_RS422_CMD

/*** ENABLE SRS200***/
// #define ENABLE_SRS200

/*** check byte***/
#define CHECK_BYTE		170
#define CHECK_BYTE2		171
#define CHECK_BYTE3		172

/*** SEL EN table***/
#define SEL_DEFAULT         0
#define SEL_RST			    1
#define SEL_FOG_1 		    2
#define SEL_FOG_2		    3
#define SEL_FOG_3 		    4
#define SEL_IMU 		    5
#define SEL_EQ	 		    6
#define SEL_IMU_MEMS	    7
#define SEL_HP_TEST 	    8
#define SEL_VERSION_TEST 	8

/*** MODE_RST CTRL REG***/
#define REFILL_SERIAL1 1
/*** MODE_FOG CTRL REG***/
#define INT_SYNC	1
#define EXT_SYNC 	1<<1
#define STOP_SYNC 	1<<2
#define VERSION     5
#define HP_TEST     3


/*** EEPROM ADDR*/
#define EEPROM_ADDR_FOG_STATUS 4095
#define EEPROM_ADDR_DVT_TEST_1 4092
#define EEPROM_ADDR_DVT_TEST_2 4093
#define EEPROM_ADDR_DVT_TEST_3 4094
#define EEPROM_ADDR_OUTPUT_FN  4090 
#define EEPROM_ADDR_SELECT_FN  4091
#define EEPROM_ADDR_REG_VALUE  4089

/*** ADC PIN***/
#define ADC_CONV 3.3/4096.0
#define ADC_VIN     18
#define ADC_PD_DC   19
#define ADC_ASE_TACT 6
#define ADC_ASE_IACT 17