#ifndef DTC03_SLAVE_H
#define DTC03_SLAVE_H

#include <DTC03_MS.h>

//#include <avr/pgmspace.h>

//========================Other Library version Request =================
//Need to Use LTC1865 1.01
// ===================================================

// =====================DEBUGFLAG Table =================================
// #define DEBUGFLAG01 
//#define DEBUGFLAG02 3
//#define DEBUGFLAG03

#ifdef DTCDEBUG01
	#define DEBUGFLAG01 // Show initial setup information in Serial monitor
#else
#endif

#ifdef DTCDEBUG02				 // Disable VCC autodetection function and set VCC for H, M, L by 3, 2, 1
	#define DEBUGFLAG02 DTCDEBU02 // Disable VCC autodetection function and set VCC for H, M, L by 3, 2, 1
#else
#endif 

#ifdef DTCDEBUG03
	#define DEBUGFLAG03
#else
#endif

//====================end of DEBUGFLAG Table==============================*/

//=====================Freqently update define ==========================
// ========User can copy below define to main function ==================
#define RMEASUREVOUT 29000 //20161031, 55000
#define RMEASUREDELAY 3000 //20161031
#define RMEASUREAVGTIME 10
#define AVGTIME 64 // Note!!!! VACTAVGTIEM = 2 ^ VACTAVGPWR 
#define AVGPWR 6 	// Note!!!! VACTAVGTIEM = 2 ^ VACTAVGPWR
// new for autotune//
#define ATUNEAVGTIME 4
#define ATUNEAVGPWR 2

#define BCONSTOFFSET 3500

#define ILIMDACOUTSTART 500		// define the current limit start current 500mA
#define ILIMDACSTEP 50  		// define the current limit step current 50mA
#define BVALUE 3988 

//=================end of Frequently update define======================

//=======No EEPROM default parameter user can copy these define to main for debug======
#define NOEE_P 15
#define NOEE_KI 219
#define NOEE_LS 20
#define NOEE_KIINDEX 11 //20161103
#define NOEE_B 488
#define NOEE_SENS 0 //sensortype
#define NOEE_VSET 13524 //vset_lim =@Bconst=3988
#define NOEE_ILIM 32 // currntlimit,3A=50
//#define NOEE_VBEH1 215 // DACout max=215*
//#define NOEE_VBEH2	19 // DACout minus step=19*
//#define NOEE_VBEC1	215
#define NOEE_VBEH1 6 // R1, 0.6ohm
#define NOEE_VBEH2	1 // R2, 1ohm
#define NOEE_VBEC1	2 //Tpid offset 2000
#define NOEE_VBEC2	19
#define NOEE_FBC 22400
#define NOEE_DUMMY 104
#define NOEE_OFFSET 32400

// The address of EEPROM
#define EEADD_P 				0
//#define EEADD_KI 				1
//#define EEADD_LS				2
#define EEADD_currentlim 		3
#define EEADD_Vset_upper 		4
#define EEADD_Vset_lower 		5
#define EEADD_Sensor_type 		8
#define EEADD_B_upper 			9
#define EEADD_B_lower 			10
#define EEADD_VBE_H1 			11
#define EEADD_VBE_H2			12
#define EEADD_VBE_C1			13
#define EEADD_VBE_C2			14
#define EEADD_FBC_base_upper 	15
#define EEADD_FBC_base_lower 	16
#define EEADD_Vmodoffset_upper 	17
#define EEADD_Vmodoffset_lower 	18
#define EEADD_KIINDEX		    19 //20161101
#define EEADD_DUMMY             20
// Pin definition

#define FBSEL 3
#define NMOSC_IN 4
#define NMOSH_IN 5
#define CURRENT_LIM 6
#define LTC1865CONV 7
#define VCC1 8
#define VCC2 10
#define VCC3 1
#define DACC 9
#define ISENSE0 A0
#define TEMP_SENSOR A1
#define SENSOR_TYPE A2
#define TEC_VOLT A3

// define SLAVE ADDRESS

#define MUCSLAVE 0x07

// define VCC voltage state
#define VCCHIGH 3   // 8.3
#define VCCMEDIUM 2 // 6.4
#define VCCLOW 1 	// 4.8

// define MOS status
#define HEATING 1
#define COOLING 0

// define LTC1865 channel name
#define CHVACT 0
#define CHVMOD 1

// define port manipulation address
#define MOS_ON_OFF_STATUS_ADD (1 << NMOSC_IN)|(1<<NMOSH_IN)|(1<<FBSEL)

//define calculation ration parameter
#define CURRENTRatio 9.775 // code * 5000/1023/50/0.01= code * 9.997 (mA)
#define RTECRatio 2 // R=(vcode-vtec0)/(icode-itec0)/50/0.01 = (vcode-vtec0)/(icode-itec0)*2
#define RTHRatio 25665 //(Isen*R0)*65535/vref concert Isense*R0 to 16bit ADC code
#define T0INV 0.003354
#define V_NOAD590 30000 

//new for autotune//
#define AUTUNE_MV_STATUS 0
#define ATUNE_BIAS 50690
#define NOISEBAND 7
#define MAXLBACK 50
#define MAXPEAKS 5
#define OUTSTEP 1000
#define FINDBIASARRAY 15
#define RUNTIMELIMIT 1200000//1200000 : 20 min
#define SAMPLINGTINE 1000 
const unsigned char PS_16 = (1<<ADPS2);
const unsigned char PS_32 = (1<<ADPS2)|(1<<ADPS0);
const unsigned char PS_128 = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);


class AD5541;
class LT1865;
class PID;
class DTC03
{
public:
	AD5541 dacformos, dacforilim;
	PID pid, ipid;
	DTC03();
	unsigned int InitVactArray();
    void SetPinMode();
	void SetSPI();
	void ParamInit();
	void DynamicVcc();
	void SetVcc(unsigned char state);
    void SetMos(bool heating, unsigned int fb_value);
    void SetMosOff();
	float CalculateR(unsigned int fb_value, unsigned int stabletime, int ravgtime, int vavgtime);
	void ReadEEPROMnew();
	void ReadEEPROM();
	void CheckSensorType();
	void CheckTemp();
	
	void CurrentLimit();
	void I2CRequest();
	void I2CReceive();
	void SaveEEPROM();
	void ReadVoltage(bool);
    void ReadIsense();
    void ReadVpcb();
    float ReturnTemp(unsigned int, bool);
    unsigned int ReturnVset(float tset, bool type);
    void BuildUpArray(bool, bool, bool);
    void setVset();
     

	unsigned int g_vact, g_vact_MV, g_vmod, g_fbc_base, g_Rmeas, g_isense0, g_currentabs,g_itecread;//
    unsigned char g_p, g_ki, g_ls, g_currentlim, g_kiindex, g_tpidoffset;
    unsigned char g_r1, g_r2;;
	unsigned long g_vactavgsum, g_itecavgsum, g_vpcbavgsum;
	bool g_en_state, g_heating, g_errcode1, g_errcode2, g_sensortype, g_mod_status, g_wakeup;
    bool g_overshoot, g_atune_flag;
	unsigned int g_b_upper, g_b_lower,g_vset_limit, g_ilimdacout,g_vset_limitt, g_otp;
    unsigned int g_vmodoffset, g_i2ctest, g_Vtemp;//
    int g_iteclimitset;//
    
    
    // new for autotune//
    void input_bias(unsigned int &, uint8_t);
    void output_bias(unsigned int, bool);
    int autotune(float &, float &);
    void RelaySwitchTime(unsigned long *, int &, bool &);
    void RelayMethod(unsigned int &, unsigned int &, bool*, bool*, bool*, bool &, unsigned long*, int &, unsigned int &);
    void AtunSamplingTime();
    void lookbackloop (unsigned int &, unsigned int *, bool *, bool *);
    void peakrecord (unsigned int &, bool *, bool *, int *, int *, unsigned int *, int *, unsigned long *, unsigned long , unsigned long *, bool *);
    void parameter(int *, unsigned int *, unsigned long *, int *, unsigned long *);
    unsigned int FindBiasCurrent(float &, uint8_t &, unsigned int &, unsigned int (&)[FINDBIASARRAY], unsigned long &, unsigned long &, int &);
    uint8_t atunKiLs(float &);
    uint8_t atunKp(float &);
    // new for serial command//
    void CheckSerial();
    
    unsigned int is, js;
    //
    unsigned long g_autunAactavgsum, g_dbr_counter[2];
    unsigned int g_atuneVact_MV;
    bool g_atunDone, g_DBRflag, g_runTimeflag, g_dbrCounter_flag;
    uint8_t g_atune_kp, g_atune_ki, g_p_atune, g_T_atune, g_stableCode_atune;

private:
	int ReadVtec(int Avgtime);
	
    unsigned char  g_vactindex, g_currentindex, g_vpcbindex;
	LTC1865 ltc1865;

	unsigned int Vactarray[AVGTIME], Itecarray[AVGTIME], Vpcbarray[AVGTIME], t_master;
    float g_ilimgain;
    bool p_enterSetVFlag;
    //new for autotune//
    unsigned int p_noise_Mid, p_relayT, p_samplingTime, AtuneActArray[ATUNEAVGTIME];
    
    
   
};
#endif
