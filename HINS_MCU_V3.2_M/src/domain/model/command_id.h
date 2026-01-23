#pragma once

/*** CMD 1 ~ 7 reserve for ooutput mode  ***/ 
enum {
    CMD_MOD_FREQ = 8,   
    CMD_MOD_AMP_H,      
    CMD_MOD_AMP_L,      //0x0A
    CMD_POLARITY,       //0x0B
    CMD_WAIT_CNT,       //0x0C
    CMD_ERR_AVG,        //0x0D
    CMD_GAIN1,          //0x0E
    CMD_CONST_STEP,     //0x0F
    CMD_FB_ON,          //0x10
    CMD_GAIN2,          //0x11
    CMD_ERR_OFFSET,     //0x12
    CMD_DAC_GAIN,       //0x13
    CMD_CUT_OFF,        //0x14
    CMD_OUT_TH,         //0x15
    CMD_OUT_TH_EN,      //0x16

    CMD_SF_COMP_T1 = 23,    //0x17
    CMD_SF_COMP_T2,         //0x18
    CMD_SF_1_SLOPE,         //0x19
    CMD_SF_1_OFFSET,        //0x1A
    CMD_SF_2_SLOPE,         //0x1B
    CMD_SF_2_OFFSET,        //0x1C
    CMD_SF_3_SLOPE,         //0x1D
    CMD_SF_3_OFFSET,        //0x1E
    CMD_BIAS_COMP_T1,       //0x1F
    CMD_BIAS_COMP_T2,       //0x20
    CMD_BIAS_1_SLOPE,       //0x21
    CMD_BIAS_1_OFFSET,      //0x22
    CMD_BIAS_2_SLOPE,       //0x23
    CMD_BIAS_2_OFFSET,      //0x24
    CMD_BIAS_3_SLOPE,       //0x25
    CMD_BIAS_3_OFFSET,      //0x26
    CMD_SF_SLOPE_XLM,       //0x27
    CMD_SF_OFFSET_XLM,      //0x28
    CMD_BIAS_SLOPE_XLM,     //0x29
    CMD_BIAS_OFFSET_XLM,    //0x2A

    CMD_MIS_AX = 48,            //0x30
    CMD_MIS_AY,
    CMD_MIS_AZ,
    CMD_MIS_A11,
    CMD_MIS_A12,
    CMD_MIS_A13,
    CMD_MIS_A21,
    CMD_MIS_A22,
    CMD_MIS_A23,
    CMD_MIS_A31,
    CMD_MIS_A32,
    CMD_MIS_A33,

    CMD_MIS_GX = 60,            //0x3C
    CMD_MIS_GY,
    CMD_MIS_GZ,
    CMD_MIS_G11,
    CMD_MIS_G12,
    CMD_MIS_G13,
    CMD_MIS_G21,
    CMD_MIS_G22,
    CMD_MIS_G23,
    CMD_MIS_G31,
    CMD_MIS_G32,
    CMD_MIS_G33,                //0x47

    CMD_CFG_DR = 72,            //0x48
    CMD_CFG_BR,                 //0x49

    CMD_DATA_OUT_START = 99,    //0x63
    CMD_HW_TIMER_RST = 100,     //0x64
    CMD_SYNC_CNT = 101,         //0x65
    CMD_DUMP_FOG = 102, //0x66
    CMD_WRITE_SN = 110,  //0x6E
    CMD_DUMP_MIS = 129,  //0x81
    CMD_DUMP_SN = 130,  //0x82
    CMD_DUMP_VERSION = 131,  //0x83
    CMD_DUMP_CONFIG = 132, //0x84

    /*** HINS CMD */

    CMD_HINS_PING     = 150, //0x96
    CMD_HINS_MIP      = 151, //0x97
    CMD_HINS_MIP_DATA = 152 //0x98


};