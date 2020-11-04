

// ---------Pin definition of LCD200_P06----------//

//#define ENC2_A 0 //rotary encoder 2A
//#define ENC2_SW 1 //rotary encoeder switch for changing cursor
#define SOFTSCLPIN 2 // SOFTI2C !! No need to set the pin mode SoftI2C will do this in his member function
#define PWR_OFF 3 // Vcc switch
#define LD_EN 4 // on/off the LD current
#define SOFTSDAPIN 8 // SDA for softI2C !! No need to set the pin mode SoftI2C will do this in his member function

#define ENDAC 6  //PD6 for control AD5541
#define VFC1 7 // VFC1 swich
#define VFC2 9 // VFC3 switch
#define VFC3 10 // VFC3 switch 
#define VLD A1 // PC1 (ADC1) for LD voltage read
#define V_SENS A2 // V+ voltage detection
//#define LCDSW A7 // page switch


//---------VCC setting parameter --------
#define VCCHIGH 0
#define VCCMEDIUM 1
#define VCCLOW 2

//----Current Parameter----
#define CHECKCURRENT 65335 // set the check current around 400uA for open short checking
#define OPENVTH 100 // vmon < 7.62mV while the current will be regards zero current and judge as Open
#define VFSHORT 50 // if under 400uA condition, vf < 0.22V the LD will be regards as short
#define I2CREADTIMEMAX 500 //maxmum I2C command read time is 500us
#define IOUTSTEP 980 // around 3mA perstep (980*5/65535/25=0.003A)
#define IOUTCOUNTERMAX 10 //if iout real can't equal to ioutseet for 10 loop times, it will be judge as output error, which means the dropout voltage is too large.
#define POWERGOOD 550 //V_SENS pin detection values for power good, if vpluse smaller thna this value, need to by pass the LD to avoid inrush current caused LT1028


//class SoftI2C;
class LTC2451;
class AD5541;

class LCD200
{
public:
	AD5541 ad5541;
	LTC2451 ltc2451;
	
	LCD200();
	void SetPinMode();
	void DACInit();
	void PWROnOff(bool);
	void SetVCC(unsigned char);
	void AnaBoardInit();
	void ResetFlag();
	bool OpenShortVfCheck();
	void PWRCheck();
	bool IoutSlow();
	void CheckOutputErr();
	void OnReceiveEvent();
	void OnRequestEvent();
	void ad5541Test();
	void readMonitor();
	
	bool g_checkflag, g_initfinished, g_com_lden;
	
//private:
	bool g_LDOpenFlag, g_LDShortFlag, g_OutErrFlag, g_AnyErrFlag;
	unsigned int g_dacout, g_vmon, g_dacoutslow;
	unsigned char g_outerrorcounter, g_vfth1, g_vfth2;

};
