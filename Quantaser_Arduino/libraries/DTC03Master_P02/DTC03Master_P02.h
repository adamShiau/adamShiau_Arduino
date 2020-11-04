#include <Wire.h>
#include <openGLCD.h>
#include <DTC03_MS.h>
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
#define ILIMSTART 0.5
#define ILIMSTEP 0.05
#define DEBOUNCETIME 2 //debounceing time(ms) for ENC
#define COUNTRESETTIME 80
#define LONGPRESSTIME 1000
#define FACTORYMODETIME 20000
#define ENGCOUNTER 12
#define BLINKDELAY 200
#define CURRENTLIMMAX 50 //maximun of g_currentlim
#define BCONSTMAX 4500//maximun of bconst
#define HIGHLOWBOUNDRY 500 //
#define TESTGOPERIOD 18000000 // testing period for one temperature in the unit of ms (18,000,000 = 5hrs)
//#define TESTGOPERIOD 60000 //testing period for one tempeature in the unit of ms (600,000 = 10 mins) used for functional test should be remove after function varification 
#define TESTGOSTEP 22.00 // testgo temperature step, usually ttstart = Tamb -12 so the second step will be ttstart + 24 = Tamb +12

//=================pin definition=========================
#define ENC_B 3
#define ENC_A 2
#define PUSHB A7
#define ENSW A6 

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
#define Text_P			"P:"
#define I_COORD_X		COLUMNPIXEL0507*14
#define I_COORD_Y		ROWPIXEL0507*4
#define I_COORD_X2		COLUMNPIXEL0507*16
#define Text_I			"I:"
#define BCONST_COORD_X 	COLUMNPIXEL0507*14
#define BCONST_COORD_Y 	ROWPIXEL0507*6
#define BCONST_COORD_X2	COLUMNPIXEL0507*16
#define Text_B			"B:"
#define VMOD_COORD_X 	COLUMNPIXEL0507*14
#define VMOD_COORD_Y 	ROWPIXEL0507*7
#define VMOD_COORD_X2 	COLUMNPIXEL0507*17
#define Text_MS			"MS:"


//==================ENG BG print coordinate definition=========
#define VR1_X 0
#define VR1_X2 42//
#define VR1_Y 0
#define VR2_X 0
#define VR2_X2 42//
#define VR2_Y 8
#define TPIDOFF_X 0
#define TPIDOFF_X2 42//
#define TPIDOFF_Y 16
#define VFBC_X 0
#define VFBC_X2 42
#define VFBC_Y 24
#define VMOD_X 0
#define VMOD_X2 42
#define VMOD_Y 32
#define TTSTART_X 0
#define TTSTART_X2 49
#define TTSTART_Y 40
#define TESTGO_X 0
#define TESTGO_X2 49
#define TESTGO_Y 48

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
//#define CURRENTRatio 9.775	// code * 5000/1023/50/0.01= code * 9.997 (mA)
#define CURRENTRatio 0.009997// code * 5/1023/50/0.01= code * 0.09997 (A)


class DTC03Master {
public:
	DTC03Master();
	void SetPinMode();
	void ParamInit();
	void WelcomeScreen();
	void I2CReadData(unsigned char i);
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
	void I2CWriteData(unsigned char com);
	void UpdateParam();
	unsigned int ReturnVset(float tset, bool type);
	void PrintFactaryMode();
	void CheckStatus();
	void PrintEngBG();
	void PrintR1();
	void PrintR2();
	void PrintTpidoff();
	void PrintTtstart();
	void PrintTestgo();
	void PrintVfbc();
	void PrintVmod();
	void ShowCursor();
    void UpdateEnable();//
	
	unsigned int g_vact, g_vset;
	bool g_sensortype, g_en_state, g_mod_status;
    unsigned long g_vactsum; 
	int g_itecsum;//
    float g_tset;
    unsigned char g_cursorstate;
private:
	glcd lcd;
	int g_counter;
    unsigned int g_bconst, g_fbcbase, Varray[VAVGTIMES], Iarray[IAVGTIMES], g_icount ,g_vmodoffset;
    unsigned char g_p, g_ki,g_ls,g_currentlim, g_tpidoff, g_vbec2, g_r1, g_r2, g_kiindex;
	unsigned char g_iarrayindex, g_varrayindex, g_lastencoded, g_engmode;
    bool g_errcode1, g_errcode2, g_flag, g_paramupdate, g_countersensor, g_testgo;
    unsigned long g_tenc;
	float g_tsetstep, g_ttstart;


};
