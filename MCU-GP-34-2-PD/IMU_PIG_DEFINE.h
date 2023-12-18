/*** VERSION */
#define MCU_VERSION "MCU-GP-34-2-RD"

// #define GP1Z
#define AFI

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
#define MODE_FOG_HP_TEST	3
#define MODE_NMEA		    4
#define MODE_AFI		    5
#define MODE_FOG_PARAMETER  6

// MCU Parameters CMD Address
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
#define CMD_FOG_SFB     	38
#define CMD_FOG_CUTOFF     	39
#define CMD_FPGA_VERSION	101
#define CMD_DUMP_PARAMETERS	102
#define CMD_CONFI_BAUDRATE  103
#define CMD_CONFI_DATARATE  104
#define CMD_CALI_AX         105
#define CMD_CALI_AY         106
#define CMD_CALI_AZ         107
#define CMD_CALI_A11        108
#define CMD_CALI_A12        109
#define CMD_CALI_A13        110
#define CMD_CALI_A21        111
#define CMD_CALI_A22        112
#define CMD_CALI_A23        113
#define CMD_CALI_A31        114
#define CMD_CALI_A32        115
#define CMD_CALI_A33        116
#define CMD_CALI_GX         117
#define CMD_CALI_GY         118
#define CMD_CALI_GZ         119
#define CMD_CALI_G11        120
#define CMD_CALI_G12        121
#define CMD_CALI_G13        122
#define CMD_CALI_G21        123
#define CMD_CALI_G22        124
#define CMD_CALI_G23        125
#define CMD_CALI_G31        126
#define CMD_CALI_G32        127
#define CMD_CALI_G33        128
#define CMD_DUMP_CALI_PARAMETERS	129

#define MUX_OUTPUT		    0
#define MUX_PARAMETER	    1
#define MUX_ESCAPE		    2

// FPGA Parameters CMD Address
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
#define SFB_ADDR            27
#define CUTOFF_ADDR         28
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
#define DELAY_CNT 5
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
#define SEL_NMEA	 		6
#define SEL_FOG_PARA	    7
#define SEL_HP_TEST 	    8
#define SEL_AFI          	9

/*** MODE_RST CTRL REG***/
#define REFILL_SERIAL1 1
/*** MODE_FOG CTRL REG***/
#define INT_SYNC	1
#define EXT_SYNC 	1<<1
#define STOP_SYNC 	1<<2
#define NMEA_MODE   5
#define HP_TEST     3

/*** setting output parameter ***/
#define SET_BAUDRATE_230400 0
#define SET_BAUDRATE_115200 1
#define SET_BAUDRATE_9600   2
#define SET_BAUDRATE_4800   3
#define SET_DATARATE_100    0
#define SET_DATARATE_10     1

#define EEPROM_PARAMETER_EXIST 0xAA

#define FLOAT_1 0x3f800000  
#define FLOAT_2 0x40000000  
#define FLOAT_3 0x40400000  
#define FLOAT_4 0x40800000  
#define FLOAT_650 0x44228000

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
#define SFB_INIT            0
#define CUTOFF_INIT         FLOAT_650

/*** OUTPUT CONFIGURATION PARAMETERS*/
#define BAUDRATE_INIT SET_BAUDRATE_230400
#define DATARATE_INIT SET_DATARATE_100

/*** CALIBRATION PARAMETERS*/
#define CALI_AX_INIT 0
#define CALI_AY_INIT 0
#define CALI_AZ_INIT 0
#define CALI_A11_INIT FLOAT_1
#define CALI_A12_INIT 0
#define CALI_A13_INIT 0
#define CALI_A21_INIT 0
#define CALI_A22_INIT FLOAT_1
#define CALI_A23_INIT 0
#define CALI_A31_INIT 0
#define CALI_A32_INIT 0
#define CALI_A33_INIT FLOAT_1
#define CALI_GX_INIT 0
#define CALI_GY_INIT 0
#define CALI_GZ_INIT 0
#define CALI_G11_INIT FLOAT_1
#define CALI_G12_INIT 0
#define CALI_G13_INIT 0
#define CALI_G21_INIT 0
#define CALI_G22_INIT FLOAT_1
#define CALI_G23_INIT 0
#define CALI_G31_INIT 0
#define CALI_G32_INIT 0
#define CALI_G33_INIT FLOAT_1


/*** ADC PIN***/
#define ADC_CONV 3.3/4096.0
#define ADC_VIN     18
#define ADC_PD_DC   6
#define ADC_ASE_TACT 19
#define ADC_ASE_IACT 17

#define MAX_STR_LENGTH 20
#define PARAMETER_CNT 24
#define MAX_TOTAL_LENGTH (MAX_STR_LENGTH * PARAMETER_CNT)

typedef struct {
    char str[MAX_STR_LENGTH];
} DumpParameter;



