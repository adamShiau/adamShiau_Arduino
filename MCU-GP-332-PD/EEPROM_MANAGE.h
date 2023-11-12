#ifndef EEPROM_MANAGE_H
#define EEPROM_MANAGE_H

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
// z axis
#define EEPROM_ADDR_MOD_FREQ_Z        2
#define EEPROM_ADDR_WAIT_CNT_Z        3
#define EEPROM_ADDR_ERR_AVG_Z         4
#define EEPROM_ADDR_MOD_AMP_H_Z       5
#define EEPROM_ADDR_MOD_AMP_L_Z       6
#define EEPROM_ADDR_ERR_TH_Z          7
#define EEPROM_ADDR_ERR_OFFSET_Z      8
#define EEPROM_ADDR_POLARITY_Z        9
#define EEPROM_ADDR_CONST_STEP_Z      10
#define EEPROM_ADDR_FPGA_Q_Z          11
#define EEPROM_ADDR_FPGA_R_Z          12
#define EEPROM_ADDR_GAIN1_Z           13
#define EEPROM_ADDR_GAIN2_Z           14
#define EEPROM_ADDR_FB_ON_Z           15
#define EEPROM_ADDR_DAC_GAIN_Z        16
#define EEPROM_ADDR_DATA_DELAY_Z      17
#define EEPROM_ADDR_SF_0_Z            18
#define EEPROM_ADDR_SF_1_Z            19
#define EEPROM_ADDR_SF_2_Z            20
#define EEPROM_ADDR_SF_3_Z            21
#define EEPROM_ADDR_SF_4_Z            22
#define EEPROM_ADDR_SF_5_Z            23
#define EEPROM_ADDR_SF_6_Z            24
#define EEPROM_ADDR_SF_7_Z            25
#define EEPROM_ADDR_SF_8_Z            26
#define EEPROM_ADDR_SF_9_Z            27
#define EEPROM_ADDR_TMIN_Z            28
#define EEPROM_ADDR_TMAX_Z            29
#define EEPROM_ADDR_SFB_Z             30
#define EEPROM_ADDR_CUTOFF_Z          31
// x axis
#define EEPROM_ADDR_MOD_FREQ_X        32
#define EEPROM_ADDR_WAIT_CNT_X        33
#define EEPROM_ADDR_ERR_AVG_X         34
#define EEPROM_ADDR_MOD_AMP_H_X       35
#define EEPROM_ADDR_MOD_AMP_L_X       36
#define EEPROM_ADDR_ERR_TH_X          37
#define EEPROM_ADDR_ERR_OFFSET_X      38
#define EEPROM_ADDR_POLARITY_X        39
#define EEPROM_ADDR_CONST_STEP_X      40
#define EEPROM_ADDR_FPGA_Q_X          41
#define EEPROM_ADDR_FPGA_R_X          42
#define EEPROM_ADDR_GAIN1_X           43
#define EEPROM_ADDR_GAIN2_X           44
#define EEPROM_ADDR_FB_ON_X           45
#define EEPROM_ADDR_DAC_GAIN_X        46
#define EEPROM_ADDR_DATA_DELAY_X      47
#define EEPROM_ADDR_SF_0_X            48
#define EEPROM_ADDR_SF_1_X            49
#define EEPROM_ADDR_SF_2_X            50
#define EEPROM_ADDR_SF_3_X            51
#define EEPROM_ADDR_SF_4_X            52
#define EEPROM_ADDR_SF_5_X            53
#define EEPROM_ADDR_SF_6_X            54
#define EEPROM_ADDR_SF_7_X            55
#define EEPROM_ADDR_SF_8_X            56
#define EEPROM_ADDR_SF_9_X            57
#define EEPROM_ADDR_TMIN_X            58
#define EEPROM_ADDR_TMAX_X            59
#define EEPROM_ADDR_SFB_X             60
#define EEPROM_ADDR_CUTOFF_X          61
// y axis
#define EEPROM_ADDR_MOD_FREQ_Y        62
#define EEPROM_ADDR_WAIT_CNT_Y        63
#define EEPROM_ADDR_ERR_AVG_Y         64
#define EEPROM_ADDR_MOD_AMP_H_Y       65
#define EEPROM_ADDR_MOD_AMP_L_Y       66
#define EEPROM_ADDR_ERR_TH_Y          67
#define EEPROM_ADDR_ERR_OFFSET_Y      68
#define EEPROM_ADDR_POLARITY_Y        69
#define EEPROM_ADDR_CONST_STEP_Y      70
#define EEPROM_ADDR_FPGA_Q_Y          71
#define EEPROM_ADDR_FPGA_R_Y          72
#define EEPROM_ADDR_GAIN1_Y           73
#define EEPROM_ADDR_GAIN2_Y           74
#define EEPROM_ADDR_FB_ON_Y           75
#define EEPROM_ADDR_DAC_GAIN_Y        76
#define EEPROM_ADDR_DATA_DELAY_Y      77
#define EEPROM_ADDR_SF_0_Y            78
#define EEPROM_ADDR_SF_1_Y            79
#define EEPROM_ADDR_SF_2_Y            80
#define EEPROM_ADDR_SF_3_Y            81
#define EEPROM_ADDR_SF_4_Y            82
#define EEPROM_ADDR_SF_5_Y            83
#define EEPROM_ADDR_SF_6_Y            84
#define EEPROM_ADDR_SF_7_Y            85
#define EEPROM_ADDR_SF_8_Y            86
#define EEPROM_ADDR_SF_9_Y            87
#define EEPROM_ADDR_TMIN_Y            88
#define EEPROM_ADDR_TMAX_Y            89
#define EEPROM_ADDR_SFB_Y             90
#define EEPROM_ADDR_CUTOFF_Y          91
/** data output parameter**/
#define EEPROM_ADDR_BAUDRATE        92
#define EEPROM_ADDR_DATARATE        93

/**Global Variable for EEPROM*/
int EEPROM_Parameter_exist=0; 
//fog parameters
// int EEPROM_Gain1, EEPROM_Gain2, EEPROM_FB_ON, EEPROM_DAC_gain, EEPROM_Data_delay;
// int EEPROM_Mod_freq, EEPROM_Wait_cnt, EEPROM_Err_avg, EEPROM_Polarity, EEPROM_Fpga_Q, EEPROM_Fpga_R;
// int EEPROM_Amp_H, EEPROM_Amp_L, EEPROM_Err_offset, EEPROM_Err_th, EEPROM_Const_step;
// int EEPROM_SF0, EEPROM_SF1, EEPROM_SF2, EEPROM_SF3, EEPROM_SF4;
// int EEPROM_SF5, EEPROM_SF6, EEPROM_SF7, EEPROM_SF8, EEPROM_SF9; 
// int EEPROM_TMIN, EEPROM_TMAX, EEPROM_SFB, EEPROM_CUTOFF;
// //X axis
// int EEPROM_Gain1_X, EEPROM_Gain2_X, EEPROM_FB_ON_X, EEPROM_DAC_gain_X, EEPROM_Data_delay_X;
// int EEPROM_Mod_freq_X, EEPROM_Wait_cnt_X, EEPROM_Err_avg_X, EEPROM_Polarity_X, EEPROM_Fpga_Q_X, EEPROM_Fpga_R_X;
// int EEPROM_Amp_H_X, EEPROM_Amp_L_X, EEPROM_Err_offset_X, EEPROM_Err_th_X, EEPROM_Const_step_X;
// int EEPROM_SF0_X, EEPROM_SF1_X, EEPROM_SF2_X, EEPROM_SF3_X, EEPROM_SF4_X;
// int EEPROM_SF5_X, EEPROM_SF6_X, EEPROM_SF7_X, EEPROM_SF8_X, EEPROM_SF9_X; 
// int EEPROM_TMIN_X, EEPROM_TMAX_X, EEPROM_SFB_X, EEPROM_CUTOFF_X;
// //Y axis
// int EEPROM_Gain1_Y, EEPROM_Gain2_Y, EEPROM_FB_ON_Y, EEPROM_DAC_gain_Y, EEPROM_Data_delay_Y;
// int EEPROM_Mod_freq_Y, EEPROM_Wait_cnt_Y, EEPROM_Err_avg_Y, EEPROM_Polarity_Y, EEPROM_Fpga_Q_Y, EEPROM_Fpga_R_Y;
// int EEPROM_Amp_H_Y, EEPROM_Amp_L_Y, EEPROM_Err_offset_Y, EEPROM_Err_th_Y, EEPROM_Const_step_Y;
// int EEPROM_SF0_Y, EEPROM_SF1_Y, EEPROM_SF2_Y, EEPROM_SF3_Y, EEPROM_SF4_Y;
// int EEPROM_SF5_Y, EEPROM_SF6_Y, EEPROM_SF7_Y, EEPROM_SF8_Y, EEPROM_SF9_Y; 
// int EEPROM_TMIN_Y, EEPROM_TMAX_Y, EEPROM_SFB_Y, EEPROM_CUTOFF_Y;
//Output configuration
int EEPROM_BAUDRATE, EEPROM_DATARATE;

struct eeprom_obj{
    const uint32_t EEPROM_ADDR_MOD_FREQ        ;
    const uint32_t EEPROM_ADDR_WAIT_CNT        ;
    const uint32_t EEPROM_ADDR_ERR_AVG         ;
    const uint32_t EEPROM_ADDR_MOD_AMP_H       ;
    const uint32_t EEPROM_ADDR_MOD_AMP_L       ;
    const uint32_t EEPROM_ADDR_ERR_TH          ;
    const uint32_t EEPROM_ADDR_ERR_OFFSET      ;
    const uint32_t EEPROM_ADDR_POLARITY        ;
    const uint32_t EEPROM_ADDR_CONST_STEP      ;
    const uint32_t EEPROM_ADDR_FPGA_Q          ;
    const uint32_t EEPROM_ADDR_FPGA_R          ;
    const uint32_t EEPROM_ADDR_GAIN1           ;
    const uint32_t EEPROM_ADDR_GAIN2           ;
    const uint32_t EEPROM_ADDR_FB_ON           ;
    const uint32_t EEPROM_ADDR_DAC_GAIN        ;
    const uint32_t EEPROM_ADDR_DATA_DELAY      ;
    const uint32_t EEPROM_ADDR_SF_0            ;
    const uint32_t EEPROM_ADDR_SF_1            ;
    const uint32_t EEPROM_ADDR_SF_2            ;
    const uint32_t EEPROM_ADDR_SF_3            ;
    const uint32_t EEPROM_ADDR_SF_4            ;
    const uint32_t EEPROM_ADDR_SF_5            ;
    const uint32_t EEPROM_ADDR_SF_6            ;
    const uint32_t EEPROM_ADDR_SF_7            ;
    const uint32_t EEPROM_ADDR_SF_8            ;
    const uint32_t EEPROM_ADDR_SF_9            ;
    const uint32_t EEPROM_ADDR_TMIN            ;
    const uint32_t EEPROM_ADDR_TMAX            ;
    const uint32_t EEPROM_ADDR_SFB             ;
    const uint32_t EEPROM_ADDR_CUTOFF          ;
    int EEPROM_Gain1; int EEPROM_Gain2; int EEPROM_Polarity;
    int EEPROM_FB_ON; int EEPROM_DAC_gain; int EEPROM_Data_delay;
    int EEPROM_Mod_freq; int EEPROM_Wait_cnt; int EEPROM_Err_avg; 
    int EEPROM_Fpga_Q; int EEPROM_Fpga_R;
    int EEPROM_Amp_H; int  EEPROM_Amp_L; int EEPROM_Err_offset; int EEPROM_Err_th; int EEPROM_Const_step;
    int EEPROM_SF0; int EEPROM_SF1; int EEPROM_SF2; int EEPROM_SF3; int EEPROM_SF4;
    int EEPROM_SF5; int EEPROM_SF6; int EEPROM_SF7; int EEPROM_SF8; int EEPROM_SF9; 
    int EEPROM_TMIN; int EEPROM_TMAX; int EEPROM_SFB; int EEPROM_CUTOFF;
    
};


eeprom_obj eeprom_z = {
 EEPROM_ADDR_MOD_FREQ_Z        ,
 EEPROM_ADDR_WAIT_CNT_Z        ,
 EEPROM_ADDR_ERR_AVG_Z         ,
 EEPROM_ADDR_MOD_AMP_H_Z       ,
 EEPROM_ADDR_MOD_AMP_L_Z       ,
 EEPROM_ADDR_ERR_TH_Z          ,
 EEPROM_ADDR_ERR_OFFSET_Z      ,
 EEPROM_ADDR_POLARITY_Z        ,
 EEPROM_ADDR_CONST_STEP_Z      ,
 EEPROM_ADDR_FPGA_Q_Z          ,
 EEPROM_ADDR_FPGA_R_Z          ,
 EEPROM_ADDR_GAIN1_Z           ,
 EEPROM_ADDR_GAIN2_Z           ,
 EEPROM_ADDR_FB_ON_Z           ,
 EEPROM_ADDR_DAC_GAIN_Z        ,
 EEPROM_ADDR_DATA_DELAY_Z      ,
 EEPROM_ADDR_SF_0_Z            ,
 EEPROM_ADDR_SF_1_Z            ,
 EEPROM_ADDR_SF_2_Z            ,
 EEPROM_ADDR_SF_3_Z            ,
 EEPROM_ADDR_SF_4_Z            ,
 EEPROM_ADDR_SF_5_Z            ,
 EEPROM_ADDR_SF_6_Z            ,
 EEPROM_ADDR_SF_7_Z            ,
 EEPROM_ADDR_SF_8_Z            ,
 EEPROM_ADDR_SF_9_Z            ,
 EEPROM_ADDR_TMIN_Z            ,
 EEPROM_ADDR_TMAX_Z            ,
 EEPROM_ADDR_SFB_Z             ,
 EEPROM_ADDR_CUTOFF_Z          
};

eeprom_obj eeprom_x = {
 EEPROM_ADDR_MOD_FREQ_X        ,
 EEPROM_ADDR_WAIT_CNT_X        ,
 EEPROM_ADDR_ERR_AVG_X         ,
 EEPROM_ADDR_MOD_AMP_H_X       ,
 EEPROM_ADDR_MOD_AMP_L_X       ,
 EEPROM_ADDR_ERR_TH_X          ,
 EEPROM_ADDR_ERR_OFFSET_X      ,
 EEPROM_ADDR_POLARITY_X        ,
 EEPROM_ADDR_CONST_STEP_X      ,
 EEPROM_ADDR_FPGA_Q_X          ,
 EEPROM_ADDR_FPGA_R_X          ,
 EEPROM_ADDR_GAIN1_X           ,
 EEPROM_ADDR_GAIN2_X           ,
 EEPROM_ADDR_FB_ON_X           ,
 EEPROM_ADDR_DAC_GAIN_X        ,
 EEPROM_ADDR_DATA_DELAY_X      ,
 EEPROM_ADDR_SF_0_X            ,
 EEPROM_ADDR_SF_1_X            ,
 EEPROM_ADDR_SF_2_X            ,
 EEPROM_ADDR_SF_3_X            ,
 EEPROM_ADDR_SF_4_X            ,
 EEPROM_ADDR_SF_5_X            ,
 EEPROM_ADDR_SF_6_X            ,
 EEPROM_ADDR_SF_7_X            ,
 EEPROM_ADDR_SF_8_X            ,
 EEPROM_ADDR_SF_9_X            ,
 EEPROM_ADDR_TMIN_X            ,
 EEPROM_ADDR_TMAX_X            ,
 EEPROM_ADDR_SFB_X             ,
 EEPROM_ADDR_CUTOFF_X          
};

eeprom_obj eeprom_y = {
 EEPROM_ADDR_MOD_FREQ_Y        ,
 EEPROM_ADDR_WAIT_CNT_Y        ,
 EEPROM_ADDR_ERR_AVG_Y         ,
 EEPROM_ADDR_MOD_AMP_H_Y       ,
 EEPROM_ADDR_MOD_AMP_L_Y       ,
 EEPROM_ADDR_ERR_TH_Y          ,
 EEPROM_ADDR_ERR_OFFSET_Y      ,
 EEPROM_ADDR_POLARITY_Y        ,
 EEPROM_ADDR_CONST_STEP_Y      ,
 EEPROM_ADDR_FPGA_Q_Y          ,
 EEPROM_ADDR_FPGA_R_Y          ,
 EEPROM_ADDR_GAIN1_Y           ,
 EEPROM_ADDR_GAIN2_Y           ,
 EEPROM_ADDR_FB_ON_Y           ,
 EEPROM_ADDR_DAC_GAIN_Y        ,
 EEPROM_ADDR_DATA_DELAY_Y      ,
 EEPROM_ADDR_SF_0_Y            ,
 EEPROM_ADDR_SF_1_Y            ,
 EEPROM_ADDR_SF_2_Y            ,
 EEPROM_ADDR_SF_3_Y            ,
 EEPROM_ADDR_SF_4_Y            ,
 EEPROM_ADDR_SF_5_Y            ,
 EEPROM_ADDR_SF_6_Y            ,
 EEPROM_ADDR_SF_7_Y            ,
 EEPROM_ADDR_SF_8_Y            ,
 EEPROM_ADDR_SF_9_Y            ,
 EEPROM_ADDR_TMIN_Y            ,
 EEPROM_ADDR_TMAX_Y            ,
 EEPROM_ADDR_SFB_Y             ,
 EEPROM_ADDR_CUTOFF_Y          
};

#endif