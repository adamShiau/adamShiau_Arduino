/*** VERSION */
#define MCU_VERSION "MCU-GP-32-RD"

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
#define CMD_FOG_SF0     	26
#define CMD_FOG_SF1     	27
#define CMD_FOG_SF2     	28
#define CMD_FOG_SF3     	29
#define CMD_FOG_SF4     	30
#define CMD_FOG_SF5     	31
#define CMD_FOG_SF6     	32
#define CMD_FOG_SF7     	33
#define CMD_FOG_SF8     	34
#define CMD_FOG_SF9     	35
#define CMD_FOG_TMIN     	36
#define CMD_FOG_TMAX     	37
#define CMD_FPGA_VERSION	101
#define CMD_DUMP_PARAMETERS	102

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
#define SF0_ADDR  		    15
#define SF1_ADDR  		    16
#define SF2_ADDR  		    17
#define SF3_ADDR  		    18
#define SF4_ADDR  		    19
#define SF5_ADDR  		    20
#define SF6_ADDR  		    21
#define SF7_ADDR  		    22
#define SF8_ADDR  		    23
#define SF9_ADDR  		    24
#define TMIN_ADDR           25
#define TMAX_ADDR           26
#define DAC_GAIN_ADDR  		50
#define DATA_INT_DELAY_ADDR	98
#define DATA_OUT_START_ADDR	99
#define FPGA_WAKEUP_ADDR	100
#define FPGA_VERSION_ADDR	101
#define FPGA_DUMP_PARAMETERS_ADDR	102

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

#define EEPROM_PARAMETER_EXIST 0xAB

#define FLOAT_1 0x3f800000  
#define FLOAT_2 0x40000000  
#define FLOAT_3 0x40400000  
#define FLOAT_4 0x40800000  

#define MINUS20 0xC1A00000
#define PLUS60  0x42700000
#define PLUS80  0x42A00000

/***FOG INIT VALUE*/
#define MOD_FREQ_INIT       104
#define WAIT_CNT_INIT       20
#define ERR_AVG_INIT        6
#define MOD_AMP_H_INIT      9100
#define MOD_AMP_L_INIT      -9100
#define ERR_TH_INIT         0
#define ERR_OFFSET_INIT     0
#define POLARITY_INIT       1
#define CONST_STEP_INIT     16384
#define FPGA_Q_INIT         1
#define FPGA_R_INIT         6
#define GAIN1_INIT          5
#define GAIN2_INIT          7
#define FB_ON_INIT          1
#define DAC_GAIN_INIT       77
#define DATA_INT_DELAY_INIT 2000
#define SF_INIT             FLOAT_1   


/*** EEPROM ADDR*/
#define EEPROM_ADDR_FOG_STATUS 4095
#define EEPROM_ADDR_DVT_TEST_1 4092
#define EEPROM_ADDR_DVT_TEST_2 4093
#define EEPROM_ADDR_DVT_TEST_3 4094
#define EEPROM_ADDR_OUTPUT_FN  4090 
#define EEPROM_ADDR_SELECT_FN  4091
#define EEPROM_ADDR_REG_VALUE  4089
/** for fog parameter**/
#define EEPROM_ADDR_PARAMETER_EXIST 1
#define EEPROM_ADDR_MOD_FREQ        2
#define EEPROM_ADDR_WAIT_CNT        3
#define EEPROM_ADDR_ERR_AVG         4
#define EEPROM_ADDR_MOD_AMP_H       5
#define EEPROM_ADDR_MOD_AMP_L       6
#define EEPROM_ADDR_ERR_TH          7
#define EEPROM_ADDR_ERR_OFFSET      8
#define EEPROM_ADDR_POLARITY        9
#define EEPROM_ADDR_CONST_STEP      10
#define EEPROM_ADDR_FPGA_Q          11
#define EEPROM_ADDR_FPGA_R          12
#define EEPROM_ADDR_GAIN1           13
#define EEPROM_ADDR_GAIN2           14
#define EEPROM_ADDR_FB_ON           15
#define EEPROM_ADDR_DAC_GAIN        16
#define EEPROM_ADDR_DATA_DELAY      17
#define EEPROM_ADDR_SF_0            18
#define EEPROM_ADDR_SF_1            19
#define EEPROM_ADDR_SF_2            20
#define EEPROM_ADDR_SF_3            21
#define EEPROM_ADDR_SF_4            22
#define EEPROM_ADDR_SF_5            23
#define EEPROM_ADDR_SF_6            24
#define EEPROM_ADDR_SF_7            25
#define EEPROM_ADDR_SF_8            26
#define EEPROM_ADDR_SF_9            27
#define EEPROM_ADDR_TMIN            28
#define EEPROM_ADDR_TMAX            29


/**Global Variable for EEPROM*/
int EEPROM_Parameter_exist=0; 
int EEPROM_Gain1, EEPROM_Gain2, EEPROM_FB_ON, EEPROM_DAC_gain, EEPROM_Data_delay;
int EEPROM_Mod_freq, EEPROM_Wait_cnt, EEPROM_Err_avg, EEPROM_Polarity, EEPROM_Fpga_Q, EEPROM_Fpga_R;
int EEPROM_Amp_H, EEPROM_Amp_L, EEPROM_Err_offset, EEPROM_Err_th, EEPROM_Const_step;
int EEPROM_SF0, EEPROM_SF1, EEPROM_SF2, EEPROM_SF3, EEPROM_SF4;
int EEPROM_SF5, EEPROM_SF6, EEPROM_SF7, EEPROM_SF8, EEPROM_SF9; 
int EEPROM_TMIN, EEPROM_TMAX;

/*** ADC PIN***/
#define ADC_CONV 3.3/4096.0
#define ADC_VIN     18
#define ADC_PD_DC   6
#define ADC_ASE_TACT 19
#define ADC_ASE_IACT 17