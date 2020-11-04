//------------------I2C Command and Address-----------
//						b15 b14 b13 b12 b11 b10 b09 b08 b07 b06 b05 b04 b03 b02 b01 b00
// Address: VSET 0x01 	-----------------------------VSET------------------------------
// Address: VACT 0x02   -----------------------------VACT------------------------------
// Address: 
#include <openGLCD.h>
#include <EEPROM.h>
#include <DTC03_MS.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <fonts/SystemFont5x7.h>
// #include <fonts/Iain5x7.h>
#include <fonts/fixed_bold10x15.h>


//------constantly change parameter---------
#define BVALUE 3988
#define ILIMINDEX 50 // currentl limit = 0.5+ 50*0.05 = 3
#define PGAIN	15 
#define DEFAULTR1 15	// 1.5 ohm
#define DEFAULTR2 30	// 3.0 ohm
#define DEFAULTTPIDOFF 2 // tpidoffset 2000
#define KI 176
#define LS 20

// define I2C parameter
#define I2CSENDDELAY 100 //delay100us
#define I2CREADDELAY 100 //delay100us

//=========request Events Mask ============
#define REQMSK_ENSTATE 		0x80 //B1000 0000
#define REQMSK_SENSTYPE		0x40 //B0100 0000
#define REQMSK_BUPPER		0x07 //B0000 0111

#define REQMSK_ITECU		0x03 //B0000 0011
#define REQMSK_ITECSIGN		0x04 //B0000 0100
#define REQMSK_ERR1		 	0x10 //B0001 0000
#define REQMSK_ERR2			0x20 //B0010 0000

//-------constant definition -----
#define T0INV 			0.003354
#define CURRENTRatio    0.009997// code * 5/1023/50/0.01= code * 0.09997 (A)
#define RTHRatio 		25665 
#define ANAREADVIL 		240
#define ANAREADVIH 		500
#define LONGPRESSTIME 	5000
#define PERIOD			100
#define MAXRATEINDEX	12
#define DEBOUNCETIME 	2
#define COUNTERINCRE	50
#define COUNTERSPEEDUP	200
#define FINETUNEAMP 3
#define SCANSAMPLERATE  15
#define ILIMSTART 0.5
#define ILIMSTEP 0.05

//------pin definition ----------------
#define ENC_A 2
#define ENC_B 3
#define SCANB 11 // MOSI
#define PUSHB A7
#define ENSW A6

//-----------EEPROM ADDRESS---------
#define EEADD_VSTART_UPPER	0
#define EEADD_VSTART_LOWER	1
#define EEADD_VEND_UPPER	2
#define EEADD_VEND_LOWER	3
#define EEADD_RATE_INDEX	4
#define EEADD_currentlim 	5
#define EEADD_FBC_UPPER		6
#define EEADD_FBC_LOWER		7
#define EEADD_P             8
#define EEADD_KIINDEX	    9
#define EEADD_TOTP_UPPER    10
#define EEADD_TOTP_LOWER    11
#define EEADD_R1            12
#define EEADD_R2            13
#define EEADD_DUMMY			100

//----------NOEE Default value------
#define NOEE_VSTART		32932
#define NOEE_VEND		9313
#define NOEE_RATE		1
#define NOEE_ILIM       50 // currntlimit,3A=50
#define NOEE_FBC		22400
#define NOEE_P          10
#define NOEE_KI			0
#define NOEE_LS			0
#define NOEE_kiindex    1
#define NOEE_TOTP       561 //120C
#define NOEE_R1         10
#define NOEE_R2         30
#define NOEE_DUMMY		104

#define TEMP_RISING 	1
#define TEMP_FALLING 	0



//-----------Font Parameters---------
#define ROWPIXEL0507 8 //row pixel of SystemFont5x7
#define ROWPIXELdef 12
#define COLUMEPIXEL0507 6  

//----------Print Coordinate ---------
#define TSTART_COORD_X 	COLUMEPIXEL0507
#define TSTART_COORD_Y 	0
#define TSTART_COORD_X2	COLUMEPIXEL0507*8
#define Text_Tstart		"Tstart:"

#define TEND_COORD_X	COLUMEPIXEL0507
#define TEND_COORD_Y	ROWPIXELdef*1
#define TEND_COORD_X2	COLUMEPIXEL0507*8
#define Text_Tstop		"Tstop :"

#define RATE_COORD_X	COLUMEPIXEL0507
#define RATE_COORD_Y	ROWPIXELdef*2
#define RATE_COORD_X2	COLUMEPIXEL0507*8
#define Text_RATE		"RATE  :     C/s"

#define TFINE_COORD_X	COLUMEPIXEL0507*12
#define TFINE_COORD_Y	ROWPIXELdef*3

#define EN_COORD_X		COLUMEPIXEL0507
#define EN_COORD_Y		56
#define EN_COORD_X2 	COLUMEPIXEL0507*6
#define Text_CTRL		"CTRL:"

#define SCAN_COORD_X	COLUMEPIXEL0507*14
#define SCAN_COORD_Y	56

#define TACT_COORD_X	6
#define TACT_COORD_Y	36





//-------ENG mode related-------

#define TS_X		COLUMEPIXEL0507
#define TS_Y		0
#define TS_X2		COLUMEPIXEL0507*4
#define Text_TS		"TS:"

#define TA_X		COLUMEPIXEL0507*12
#define TA_Y		0
#define TA_X2		COLUMEPIXEL0507*15
#define Text_TA		"TA:"

#define Ilim_X		COLUMEPIXEL0507
#define Ilim_Y		ROWPIXELdef*1
#define Ilim_X2		COLUMEPIXEL0507*6
#define Text_Ilim	"Ilim:"

#define Ic_X		COLUMEPIXEL0507*12
#define Ic_Y		ROWPIXELdef*1
#define Ic_X2		COLUMEPIXEL0507*15
#define Text_Ic		"Ic:"

#define P_X			COLUMEPIXEL0507
#define P_Y			ROWPIXELdef*2
#define P_X2 		COLUMEPIXEL0507*3
#define Text_P		"P:"

#define I_X			COLUMEPIXEL0507*7
#define I_Y			ROWPIXELdef*2
#define I_X2		COLUMEPIXEL0507*9
#define Text_I		"I:"

#define R1_X		COLUMEPIXEL0507*15
#define R1_Y		ROWPIXELdef*2
#define R1_X2 		COLUMEPIXEL0507*18
#define Text_R1		"R1:"

#define R2_X		COLUMEPIXEL0507
#define R2_Y		ROWPIXELdef*3
#define R2_X2 		COLUMEPIXEL0507*4
#define Text_R2		"R2:"

#define FBC_X		COLUMEPIXEL0507*11
#define FBC_Y		ROWPIXELdef*3
#define FBC_X2 	    COLUMEPIXEL0507*15
#define Text_FBC	"FBC:"

#define Totp_X		COLUMEPIXEL0507
#define Totp_Y		ROWPIXELdef*4
#define Totp_X2 	COLUMEPIXEL0507*6
#define Text_Totp	"Totp:"

#define Tpcb_X		COLUMEPIXEL0507*12
#define Tpcb_Y		ROWPIXELdef*4
#define Tpcb_X2 	COLUMEPIXEL0507*17
#define Text_Tpcb	"Tpcb:"


const PROGMEM unsigned char RateTable[]
{
	0,		// dummy index
	1,		//index =1,	0.01k/s
	2,		//index =2, 
	3,		//index =3, 
	4,		//index =4, 
	5,		//index =5, 
	6, 		//index =6, 
	7,		//index =7, 
	8,		//index =8, 
	9, 		//index =9, 
	10,		//       10, 0.1 
	20,		//       11, 0.2
	30,		//       12, 0.3
	40,		//       13, 0.4
	50,		//       14, 0.5
	60,		//       15, 0.6
	70,		//       16, 0.7
	80,		//       17, 0.8
	90,		//       18, 0.9
	100,	//       19, 1.0
	200,	//       20, 2.0
};

// Rate definition in the unit of degree/sec // in the unit of m degree/ 100ms 
// 0 	0 		0
// 1 	0.01 	1
// 2 	0.02 	2
// 3 	0.05 	5
// 4	0.1 	10
// 5 	0.2 	20
// 6	0.5 	50
// 7 	1 		100
// 8	2 		200



class DTC03SMaster{
public:
	DTC03SMaster();
	void SetPinMode();
	void ParamInit();
	void WelcomeScreen();
	void ReadEEPROM();
	void I2CReadData(unsigned char com);
	void I2CWriteData(unsigned char com);
	void I2CWriteAll();
	float ReturnTemp(unsigned int vact, bool type);
	unsigned int ReturnVset(float tset, bool type);
	void PrintBG();
	void PrintEngBG();
	
	void PrintTstart();
	void PrintTend();
	void PrintRate();
	void PrintScan();
	void PrintTnow();
	void PrintEnable();
	void PrintTact(float);
	void PrintItec(float);
	void PrintTpcb(float);
	void PrintVfbc();
	void PrintKi();
	void PrintP();
	void PrintIlim();
	void PrintR1();
	void PrintR2();
	void PrintTpidoff();
	void PrintTotp();
	
		
	void CheckStatus();
	void CalculateRate();
	void checkTnowStatus();
	void CheckScan();
	void UpdateEnable();
	void ShowCursor();
	void CursorState();
	void UpdateParam();
	void Encoder();
	void SaveEEPROM();
	void RuntestI2C();
	void setKpKiLs(float tin);
	void Overshoot_Cancelation(float);
	void checkOvershoot(float);
	bool g_en_state;
	int g_itec;
	unsigned int g_vact, g_fbcbase, g_tpcb, g_otp;
	unsigned long g_tloop;
    unsigned char g_currentlim, g_r1, g_r2, g_tpidoff; 
    bool g_errcode1, g_errcode2;


private:
	glcd lcd;
	bool g_scan, g_heater, g_paramterupdate, p_en[2], p_scan[2], p_tnow_flag[2], p_curstatus0flag, p_rateflag, p_EngFlag;
	bool p_ee_changed, p_enableFlag, p_overshoot_scan, p_overshoot_noscan, p_overshoot_cancel_Flag_scan, p_overshoot_cancel_Flag_noscan;
	char g_counter, g_counter2;
	unsigned char g_rateindex, g_trate, g_cursorstate,g_oldcursorstate, g_lastencoded, g_kiindex, g_p, p_ee_change_state;
	unsigned int  g_vstart, g_vset, g_vend, p_loopcount, p_trate, p_tlp;
	unsigned long loopindex, g_timer, g_tenc[3], g_tscan, g_tpush;
	float g_tstart, g_tend, g_tnow, g_tfine, p_rate;

};
