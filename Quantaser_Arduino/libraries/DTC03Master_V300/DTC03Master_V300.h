#include <Wire.h>
#include <openGLCD.h>
#include <DTC03_MS.h>
#include <EEPROM.h>
#include <Wire.h>
#include <fonts/SystemFont5x7.h>
#include <fonts/Iain5x7.h>
#include <fonts/fixed_bold10x15.h>
#include <avr/pgmspace.h>

//========Frequently update paramter===========
#define BCONSTOFFSET 3500
#define KILENGTH 16
#define VAVGTIMES 64	// Note!!!! VAVGTIMEs = 2 ^ VAVGPWR 
#define VAVGPWR 6		// Note!!!! VAVGTIMEs = 2 ^ VAVGPWR
#define IAVGTIMES 8
#define IAVGPWR 3
#define ILIMSTART 0.45
#define ILIMSTEP 0.05
#define DEBOUNCETIME 2 //debounceing time(ms) for ENC
#define COUNTRESETTIME 80
#define LONGPRESSTIME 1000
#define FACTORYMODETIME 20000
#define ENGCOUNTER 10
#define BLINKDELAY 350
#define CURRENTLIMMAX 50 //maximun of g_currentlim
#define BCONSTMAX 4500//maximun of bconst
#define HIGHLOWBOUNDRY 500 //
#define CURSORSTATE_STAYTIME 700
#define ACCUMULATE_TH 50
#define DEBOUNCE_WAIT ACCUMULATE_TH*4
#define MV_STATUS 1
#define MVTIME 16
#define MVTIME_POWER 4
//=================pin definition=========================
#define ENC_B 3
#define ENC_A 2
#define PUSHB A7
#define ENSW A6 

//-----------EEPROM ADDRESS---------
#define EEADD_VSET_UPPER	0
#define EEADD_VSET_LOWER	1
#define EEADD_BCONST_UPPER	2
#define EEADD_BCONST_LOWER	3
#define EEADD_MODSTATUS		4
#define EEADD_currentlim 	5
#define EEADD_FBC_UPPER		6
#define EEADD_FBC_LOWER		7
#define EEADD_P             8
#define EEADD_KIINDEX	    9
#define EEADD_TOTP_UPPER    10
#define EEADD_TOTP_LOWER    11
#define EEADD_R1            12
#define EEADD_R2            13
#define EEADD_TPIDOFF		14
#define EEADD_MODOFF_UPPER	15
#define EEADD_MODOFF_LOWER	16
#define EEADD_RMEAS_UPPER	17
#define EEADD_RMEAS_LOWER	18
#define EEADD_PAP			19
#define EEADD_TBIAS			20
#define EEADD_ATSTABLE		21
#define EEADD_DUMMY			100

//----------NOEE Default value------
#define NOEE_DUMMY 		104
#define NOEE_VSET		26214//25C
#define NOEE_ILIM		11 // currntlimit=0.45+0.05*11=1A
#define NOEE_P			10
#define NOEE_kiindex    1 //OFF
#define NOEE_BCONST		3988
#define NOEE_MODSTATUS  0
#define NOEE_R1			10
#define NOEE_R2			20
#define NOEE_TPIDOFF    2
#define NOEE_FBC       	45000
#define NOEE_MODOFF     32500
#define NOEE_RMEAS      55000
#define NOEE_TOTP		561 //120C
#define NOEE_PAP		10
#define NOEE_TBIAS		15
#define NOEE_ATSTABLE	10

//=====================BG print coordinate definition=========
#define TSET_COORD_X	0 
#define TSET_COORD_Y	0
#define TSET_COORD_X2	16
#define Text_SET		"SET"
#define TACT_COORD_X	0
#define TACT_COORD_Y	ROWPIXEL0507*3
#define TACT_COORD_X2	18
#define Text_ACT		"ACT"
#define ITEC_COORD_X	COLUMNPIXEL0507
#define ITEC_COORD_Y	ROWPIXEL0507*6
#define ITEC_COORD_X2	COLUMNPIXEL0507*6
#define Text_ITEC		"ITEC:"
#define ILIM_COORD_X	COLUMNPIXEL0507
#define ILIM_COORD_Y	ROWPIXEL0507*7
#define ILIM_COORD_X2	COLUMNPIXEL0507*7
#define Text_ILIM		"ILIM:"
#define P_COORD_X		COLUMNPIXEL0507*14
#define P_COORD_Y		ROWPIXEL0507*3
#define P_COORD_X2		COLUMNPIXEL0507*17
#define Text_P			"P: "
#define I_COORD_X		COLUMNPIXEL0507*14
#define I_COORD_Y		ROWPIXEL0507*4
#define I_COORD_X2		COLUMNPIXEL0507*16
#define Text_I			"I:"
#define BCONST_COORD_X 	COLUMNPIXEL0507*14
#define BCONST_COORD_Y 	ROWPIXEL0507*5
#define BCONST_COORD_X2	COLUMNPIXEL0507*16
#define Text_B			"B:"
#define VMOD_COORD_X 	COLUMNPIXEL0507*14
#define VMOD_COORD_Y 	ROWPIXEL0507*6
#define VMOD_COORD_X2 	COLUMNPIXEL0507*17
#define Text_MS			"MS:"
#define ATUNE_COORD_X 	COLUMNPIXEL0507*14
#define ATUNE_COORD_Y 	ROWPIXEL0507*7
#define ATUNE_COORD_X2 	COLUMNPIXEL0507*17
#define Text_AT		    "AT:"


//==================ENG BG print coordinate definition=========
#define R1_COORD_X       COLUMNPIXEL0507
#define R1_COORD_X2      COLUMNPIXEL0507*7  
#define R1_COORD_Y       0
#define Text_R1 		 "R1   :"
#define R2_COORD_X 		 COLUMNPIXEL0507
#define R2_COORD_X2 	 COLUMNPIXEL0507*7
#define R2_COORD_Y 		 ROWPIXEL0507
#define Text_R2 		 "R2   :"
#define TPIDOFF_COORD_X  COLUMNPIXEL0507
#define TPIDOFF_COORD_X2 COLUMNPIXEL0507*7
#define TPIDOFF_COORD_Y  ROWPIXEL0507*2
#define Text_pidOS 		 "pidOS:"
#define VFBC_COORD_X 	 COLUMNPIXEL0507
#define VFBC_COORD_X2 	 COLUMNPIXEL0507*7
#define VFBC_COORD_Y 	 ROWPIXEL0507*3
#define Text_Vfbc 		 "Vfbc :"
#define VMOD_COOED_X 	 COLUMNPIXEL0507
#define VMOD_COOED_X2 	 COLUMNPIXEL0507*7
#define VMOD_COOED_Y 	 ROWPIXEL0507*4
#define Text_Vmod 		 "Vmod  :"
#define RMEAS_COORD_X 	 COLUMNPIXEL0507
#define RMEAS_COORD_X2 	 COLUMNPIXEL0507*7
#define RMEAS_COORD_Y 	 ROWPIXEL0507*5
#define Text_Rmeas 		 "Rmeas:"
#define TOTP_COORD_X 	 COLUMNPIXEL0507
#define TOTP_COORD_X2 	 COLUMNPIXEL0507*9
#define TOTP_COORD_Y 	 ROWPIXEL0507*6
#define Text_Totp 		 "Totp :"
#define TPCB_COORD_X 	 COLUMNPIXEL0507
#define TPCB_COORD_X2 	 COLUMNPIXEL0507*9
#define TPCB_COORD_Y 	 ROWPIXEL0507*7
#define Text_Tpcb 		 "Tpcb :"
#define P_AT_COORD_X 	 COLUMNPIXEL0507*13
#define P_AT_COORD_X2 	 COLUMNPIXEL0507*18
#define P_AT_COORD_Y 	 ROWPIXEL0507*0
#define Text_PAT		 "P_AT:"
#define TBIAS_COORD_X 	 COLUMNPIXEL0507*13
#define TBIAS_COORD_X2 	 COLUMNPIXEL0507*18
#define TBIAS_COORD_Y 	 ROWPIXEL0507*1
#define Text_TAT		 "T_AT:"
#define ATSTABLE_COORD_X 	 COLUMNPIXEL0507*13
#define ATSTABLE_COORD_X2 	 COLUMNPIXEL0507*18
#define ATSTABLE_COORD_Y 	 ROWPIXEL0507*2
#define Text_SAT		     "S_AT:"


#define Test1_COORD_X    0
#define Test1_COORD_Y    ROWPIXEL0507*5
#define Test2_COORD_X    COLUMNPIXEL0507*9
#define Test2_COORD_Y    ROWPIXEL0507*2
#define Test3_COORD_X    0
#define Test3_COORD_Y    ROWPIXEL0507*5


// define GLCD parameter
#define COLUMNPIXEL1015 11 //column pixels of fixed_bold10x15
#define COLUMNPIXEL0507 6  //column pixels of SystemFont5x7
#define ROWPIXEL0507 8 //row pixels of SystemFont5x7


// define I2C parameter
#define I2CSENDDELAY 100 //delay100us
#define I2CREADDELAY 100 //delay100us

//define calculation parameter
#define T0INV 0.003354
#define RTHRatio 25665 
#define CURRENTRatio 0.00977	// 3A/307code=0.00977A/code 


class DTC03Master {
public:
	DTC03Master();
	void SetPinMode();
	void ParamInit();
	void WelcomeScreen();
	void I2CReadData(unsigned char i);
	void I2CWriteData(unsigned char com);
	void I2CReadAll();
	void VarrayInit();
	void IarrayInit();
	void BackGroundPrint();
	float ReturnTemp(unsigned int vact, bool type);
	void PrintTset();
	void PrintTact(float tact);
	void PrintItec(float itec);
	void PrintIlim();
	void PrintP();
	void PrintKi();
	void PrintB();
	void PrintModStatus();
	void Encoder();
	void CursorState();	
	void UpdateParam();
	unsigned int ReturnVset(float tset, bool type);
	void PrintFactaryMode();
	void CheckStatus();
	void PrintEngBG();
	void PrintNormalAll();
	void PrintEngAll();
	void PrintR1();
	void PrintR2();
	void PrintTpidoff();
	void PrintVfbc();
	void PrintVmod();
	void PrintRmeas();
	void PrintTotp();
	void PrintTpcb(float);
	void PrintEnable();
	void PrintAtune();
	void PrintAtuneDone();
	void PrintTestValue();
	void PrintP_Atune();
	void PrintTbias();
	void PrintATStable();
	
	void ShowCursor(unsigned char);
    void UpdateEnable();
    void blinkTsetCursor();
    void SaveEEPROM();
    void ReadEEPROM();
    void I2CWriteAll();
    void HoldCursortate();
    void vact_MV();
	
	//working variable-------------------
	unsigned int g_vact,g_vact_MV, g_vset, g_tpcb, g_otp, g_Rmeas, g_bconst, g_fbcbase, g_vmodoffset;
	unsigned char g_p, g_ki,g_ls,g_currentlim, g_tpidoff, g_r1, g_r2, g_kiindex, g_cursorstate, g_p_atune, g_T_atune, g_stableCode_atune;
	int g_itec;
	bool g_mod_status, g_atune_status, g_atunDone, g_DBRflag, g_runTimeflag, g_LCDlock_flag;	
    float g_tset;
    //------------------------------------
    bool g_sensortype, g_en_state, g_kpkiFromAT;
    int en_temp, test_at=0;
    
private:
	glcd lcd;
	int g_counter;
    unsigned int  g_icount , p_cursorStateCounter[3], p_temp, p_cursorStayTime;
    unsigned int p_tBlink, p_tcursorStateBounce, p_holdCursorTimer, p_vact_array[16]; 
	unsigned long  p_vact_MV_sum, p_mvindex;
	unsigned char g_iarrayindex, g_varrayindex, g_lastencoded, p_engmodeCounter, p_ee_change_state;
    bool g_errcode1, g_errcode2, g_flag, g_paramupdate, g_countersensor, g_testgo, p_tBlink_toggle, p_engModeFlag, p_blinkTsetCursorFlag, g_wakeup;
    bool p_ee_changed, p_HoldCursortateFlag, p_timerResetFlag, p_keyflag, p_atunProcess_flag;
    unsigned long g_tenc, p_loopindex;
	float g_tsetstep;


};
