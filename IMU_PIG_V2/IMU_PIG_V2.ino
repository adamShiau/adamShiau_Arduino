#include "adxl355.h"
#include <Arduino_LSM6DS3.h>
#include "pig_v2.h"
// #include "srs200.h"
#include "IMU_PIG_DEFINE.h"
#include "wiring_private.h"
#include "crcCalculator.h"
#include "SparrowParaDefine.h"
#include "Sparrow_read.h"
#include <TinyGPSPlus.h>

#ifdef PWM_SYNC
	#include <SAMD21turboPWM.h>
#endif
// #define TESTMODE

/*** global var***/
int pin_scl_mux = 17;
bool trig_status[2] = {0, 0};
unsigned int t_new, t_old=0;

unsigned long gps_init_time = 0;
unsigned int gps_date=0, gps_time=0;
bool gps_valid = 0;


/*** serial data from PC***/
byte rx_cnt = 0, cmd;
unsigned int value;
bool cmd_complete;

/*** output mode flag***/
byte mux_flag;

/*** acq fog flag***/
bool run_fog_flag;

/*** select running fn flag***/
byte select_fn;
// bool fn_lock;

/*** KVH HEADER ***/
const unsigned char KVH_HEADER[4] = {0xFE, 0x81, 0xFF, 0x55};
const unsigned char PIG_HEADER[2] = {0xAB, 0xBA};

typedef void (*fn_ptr) (byte &, unsigned int);
fn_ptr output_fn;

Adxl355 adxl355(pin_scl_mux);
crcCal myCRC;


Uart mySerial5 (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM0_Handler() 
{
	mySerial5.IrqHandler();
}

Sparrow_read sparrow(mySerial5);
PIG pig_v2(mySerial5);

// The TinyGPSPlus object
TinyGPSPlus gps;

Uart mySerial13 (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);
void SERCOM1_Handler()
{
    mySerial13.IrqHandler();
}

#ifdef ENABLE_SRS200
    #define SRS200_SIZE 15
#endif

TurboPWM  pwm;

void setup() {
	Serial.begin(230400);
	Serial1.begin(230400); //to PC

    mySerial5.begin(115200); //rx:p5, tx:p6
    // Reassign pins 5 and 6 to SERCOM alt
    pinPeripheral(5, PIO_SERCOM_ALT); //RX
    pinPeripheral(6, PIO_SERCOM_ALT); //TX

	mySerial13.begin(9600);//rx:p13, tx:p8
	// Reassign pins 13 and 8 to SERCOM (not alt this time)
	pinPeripheral(13, PIO_SERCOM);
	pinPeripheral(8, PIO_SERCOM);
	
	IMU.begin();
	adxl355.init();
	pinMode(SYS_TRIG, INPUT);
	/*** var initialization***/
	cmd_complete = 0;
	mux_flag = MUX_ESCAPE; 		//default set mux_flag to 2
	// select_fn = SEL_DEFAULT; 	//default set select_fn to 128
	select_fn = SEL_IMU;
	run_fog_flag = 0;
	output_fn = temp_idle;
	
	/*** pwm ***/

	/*** Set input clock divider and Turbo Mode (which uses a 96MHz instead of a 48Mhz input clock): ***/
	pwm.setClockDivider(200, false); // Main clock 48MHz divided by 200 => 240KHz
	
	/*** Initialise timer x, with prescaler, with steps (resolution), 
	with fast aka single-slope PWM (or not -> double-slope PWM): 
	For the Arduino Nano 33 IoT, you need to initialise timer 1 for pins 4 and 7, timer 0 for pins 5, 6, 8, and 12, 
	and timer 2 for pins 11 and 13;
	***/
	pwm.timer(2, 2, 600, false);   // Use timer 2 for pin 11, divide clock by 4, resolution 600, dual-slope PWM
	pwm.analogWrite(11, 500);        // PWM frequency = 120000/step/2, dutycycle is 500 / 1000 * 100% = 50%
	                                // current setup: 120000/240/2 = 250Hz
	                                // step = 600 for 100Hz
    /*--- for Sparrow demo ---*/
    delay(1000);
//     sparrow.gyroInitialize(50);
// 	sparrow.flushInputBuffer();
	Serial.print("initial buffer: ");
	Serial.println(mySerial5.available());
// 	sparrow.startRead(1);
}

void loop() {
	getCmdValue(cmd, value, cmd_complete);
	cmd_mux(cmd_complete, cmd, mux_flag);
	parameter_setting(mux_flag, cmd, value);
	output_mode_setting(mux_flag, cmd, select_fn);
	output_fn(select_fn, value);
  updateGPS(1000);
}

void printAdd(char name[], void* addr)
{
	Serial.print(name);
	Serial.print(": ");
	Serial.println((unsigned int)addr, HEX);
}

// void print_nano33GyroData(int wx, int wy, int wz)
// {
	// t_new = micros();
	// Serial.print(t_new - t_old);
	// Serial.print('\t');
	// Serial.print((float)wx*NANO33_GYRO);
	// Serial.print('\t');
	// Serial.print((float)wy*NANO33_GYRO);
	// Serial.print('\t');
	// Serial.println((float)wz*NANO33_GYRO);
	// t_old = t_new;
// }

// void print_nano33XlmData(int ax, int ay, int az)
// {
	// t_new = micros();
	// Serial.print(t_new - t_old);
	// Serial.print('\t');
	// Serial.print((float)ax*NANO33_XLM);
	// Serial.print('\t');
	// Serial.print((float)ay*NANO33_XLM);
	// Serial.print('\t');
	// Serial.println((float)az*NANO33_XLM);
	// t_old = t_new;
// }



void print_adxl355Data(byte *temp_a)
{
	int accX, accY, accZ;
	
	accX = temp_a[0]<<12 | temp_a[1]<<4 | temp_a[2]>>4;
	if((accX>>19) == 1) accX = accX - (1<<20);
	accY = temp_a[3]<<12 | temp_a[4]<<4 | temp_a[5]>>4;
	if((accY>>19) == 1) accY = accY - (1<<20);
	accZ = temp_a[6]<<12 | temp_a[7]<<4 | temp_a[8]>>4;
	if((accZ>>19) == 1) accZ = accZ - (1<<20);
	
	t_new = micros();
	Serial.print(t_new - t_old);
	Serial.print('\t');
	Serial.print((float)accX*ADXL355_8G);
	Serial.print('\t');
	Serial.print((float)accY*ADXL355_8G);
	Serial.print('\t');
	Serial.println((float)accZ*ADXL355_8G);
	t_old = t_new;
}

void printVal_1(char name[], int val)
{
	Serial1.print(name);
	Serial1.print(": ");
	Serial1.println(val);
}

void printVal_0(char name[], int val)
{
	Serial.print(name);
	Serial.print(": ");
	Serial.println(val);
}

void printVal_0(char name[])
{
	Serial.println(name);
}

void getCmdValue(byte &uart_cmd, unsigned int &uart_value, bool &uart_complete)
{
	byte cmd[5];
		
	#ifdef UART_SERIAL_5_CMD
    while (mySerial5.available()>0){
      mySerial5.readBytes((char*)cmd, 5);
    #endif

    #ifdef UART_RS422_CMD
    while (Serial1.available()>0){
      Serial1.readBytes((char*)cmd, 5);
    #endif

	#ifdef UART_USB_CMD
    while (Serial.available()>0){
      Serial.readBytes((char*)cmd, 5);
	#endif

		uart_cmd = cmd[0];
		uart_value = cmd[1]<<24 | cmd[2]<<16 | cmd[3]<<8 | cmd[4];
		uart_complete = 1;
		printVal_0("cmd", uart_cmd);
		printVal_0("rx", uart_value);
	}
}

void cmd_mux(bool &cmd_complete, byte cmd, byte &mux_flag)
{
	if(cmd_complete)
	{
		cmd_complete = 0;
		if(cmd >7) mux_flag = MUX_PARAMETER; 
		else mux_flag = MUX_OUTPUT;
		
	}
}

void parameter_setting(byte &mux_flag, byte cmd, unsigned int value) 
{
	if(mux_flag == MUX_PARAMETER)
	{
		mux_flag = MUX_ESCAPE;
		switch(cmd) {
			case CMD_FOG_MOD_FREQ: {pig_v2.sendCmd(MOD_FREQ_ADDR, value);break;}
			case CMD_FOG_MOD_AMP_H: {pig_v2.sendCmd(MOD_AMP_H_ADDR, value);break;}
			case CMD_FOG_MOD_AMP_L: {pig_v2.sendCmd(MOD_AMP_L_ADDR, value);break;}
			case CMD_FOG_ERR_OFFSET: {pig_v2.sendCmd(ERR_OFFSET_ADDR, value);break;}
			case CMD_FOG_POLARITY: {pig_v2.sendCmd(POLARITY_ADDR, value);break;}
			case CMD_FOG_WAIT_CNT:{pig_v2.sendCmd(WAIT_CNT_ADDR, value);break;}
			case CMD_FOG_ERR_TH: {pig_v2.sendCmd(ERR_TH_ADDR, value);break;}
			case CMD_FOG_ERR_AVG: {pig_v2.sendCmd(ERR_AVG_ADDR, value);break;}
			case CMD_FOG_TIMER_RST: {pig_v2.sendCmd(TIMER_RST_ADDR, value);break;}
			case CMD_FOG_GAIN1: {pig_v2.sendCmd(GAIN1_ADDR, value);break;}
			case CMD_FOG_GAIN2: {pig_v2.sendCmd(GAIN2_ADDR, value);break;}
			case CMD_FOG_FB_ON: {pig_v2.sendCmd(FB_ON_ADDR, value);break;}
			case CMD_FOG_CONST_STEP: {pig_v2.sendCmd(CONST_STEP_ADDR, value);break;}
			case CMD_FOG_FPGA_Q: {pig_v2.sendCmd(FPGA_Q_ADDR, value);break;}
			case CMD_FOG_FPGA_R: {pig_v2.sendCmd(FPGA_R_ADDR, value);break;}
			case CMD_FOG_DAC_GAIN: {pig_v2.sendCmd(DAC_GAIN_ADDR, value);break;}
			case CMD_FOG_INT_DELAY: {pig_v2.sendCmd(DATA_INT_DELAY_ADDR, value);break;}
			case CMD_FOG_OUT_START: {pig_v2.sendCmd(DATA_OUT_START_ADDR, value);break;}
			default: break;
		}
	}
}

void output_mode_setting(byte &mux_flag, byte mode, byte &select_fn)
{
	if(mux_flag == MUX_OUTPUT)
	{
		mux_flag = MUX_ESCAPE;
		switch(mode) {
			case MODE_RST: {
				output_fn = fn_rst;
				select_fn = SEL_RST;
				break;
			}
			case MODE_FOG: {
				output_fn = acq_fog;
				select_fn = SEL_FOG_1;
				break;
			}
			case MODE_IMU: {
				output_fn = acq_imu; 
				select_fn = SEL_IMU;
				break;
			}
			case MODE_IMU_FAKE: {
				output_fn = acq_imu_fake; 
				select_fn = SEL_IMU;
				break;
			}
			case MODE_EQ: {
				output_fn = temp_idle;
				select_fn = SEL_EQ;
				break;
            }
      case MODE_IMU_MEMS: {
          output_fn = acq_imu_mems;
          select_fn = SEL_IMU_MEMS;
          break;
      }
      case MODE_IMU_MEMS_GPS: {
          output_fn = acq_imu_mems_gps;
          select_fn = SEL_IMU_MEMS;
          break;
      }
      case MODE_SPARROW_DEMO: {
          output_fn = acq_imu_sparrow;
          select_fn = SEL_SPARROW_DEMO ;
          break;
      }
      default: break;
      }
		
		// printVal_0("in output_mode_setting");
		// printVal_0("mux_flag", mux_flag);
		// printVal_0("mode", mode);
		// printVal_0("select_fn", select_fn);
	}
}

void temp_idle(byte &select_fn, unsigned int CTRLREG)
{
	clear_SEL_EN(select_fn);
	delay(100);
}

void fn_rst(byte &select_fn, unsigned int CTRLREG)
{
	if(select_fn&SEL_RST) {
		switch(CTRLREG) {
			case REFILL_SERIAL1: {
				for(int i=0; i<256; i++) Serial1.read();
				break;
			}
			default: break;
		}
		
	}
	clear_SEL_EN(select_fn);
}


void acq_fog(byte &select_fn, unsigned int CTRLREG)
{
	byte header[2], fog[10];
	uint8_t CRC32[4];
	
	if(select_fn&SEL_FOG_1)
	{
		run_fog_flag = pig_v2.setSyncMode(CTRLREG);
	}

	trig_status[0] = digitalRead(SYS_TRIG);

	if((trig_status[0] & ~trig_status[1]) & run_fog_flag) {
	    t_new = micros();

	    uint8_t* imu_data = (uint8_t*)malloc(12); // header:2 + data:10
        pig_v2.readData(header, fog);
        memcpy(imu_data, PIG_HEADER, 2);
        memcpy(imu_data+2, fog, 10);
        myCRC.crc_32(imu_data, 12, CRC32);
        free(imu_data);

		#ifdef UART_SERIAL_5_CMD
        mySerial5.write(header, 2);
        mySerial5.write(fog, 10);
        mySerial5.write(CRC32, 4);
        #endif
        #ifdef UART_RS422_CMD
        Serial1.write(header, 2);
        Serial1.write(fog, 10);
        Serial1.write(CRC32, 4);
        #endif
//         Serial.println(t_new - t_old);
        t_old = t_new;
	}
	clear_SEL_EN(select_fn);	
}

void acq_imu(byte &select_fn, unsigned int CTRLREG)
{
	byte adxl355_a[9], header[2], fog[14], nano33_w[6], nano33_a[6];
	uint8_t CRC32[4];

	#ifdef ENABLE_SRS200
		byte srs200[SRS200_SIZE], header_srs200[2];
	#endif
	if(select_fn&SEL_IMU) {
		run_fog_flag = pig_v2.setSyncMode(CTRLREG);
	}
	trig_status[0] = digitalRead(SYS_TRIG);

	if((trig_status[0] & ~trig_status[1]) & run_fog_flag) {
        t_new = micros();

		#ifdef ENABLE_SRS200
			readSRS200Data(header_srs200, srs200);
		#endif

		uint8_t* imu_data = (uint8_t*)malloc(39); // KVH_HEADER:4 + adxl355:9 + nano33_w:6 + nano33_a:6 + pig:14

    pig_v2.readData(header, fog);
		adxl355.readData(adxl355_a);
		IMU.readGyroscope(nano33_w);
		IMU.readAcceleration(nano33_a);


		memcpy(imu_data, KVH_HEADER, 4);
        memcpy(imu_data+4, adxl355_a, 9);
        memcpy(imu_data+13, nano33_w, 6);
        memcpy(imu_data+19, nano33_a, 6);
        memcpy(imu_data+25, fog, 14);
        myCRC.crc_32(imu_data, 39, CRC32);
        free(imu_data);
// 		print_adxl355Data(adxl355_a);

		#ifdef UART_SERIAL_5_CMD
            mySerial5.write(KVH_HEADER, 4);
            mySerial5.write(adxl355_a, 9);
            mySerial5.write(nano33_w, 6);
            mySerial5.write(nano33_a, 6);
            mySerial5.write(fog, 14);
            mySerial5.write(CRC32, 4);
		#endif
        #ifdef UART_USB_CMD
            Serial.write(KVH_HEADER, 4);
            Serial.write(adxl355_a, 9);
            Serial.write(nano33_w, 6);
            Serial.write(nano33_a, 6);
            Serial.write(fog, 14);
            Serial.write(CRC32, 4);
		#endif
		#ifdef UART_RS422_CMD
            Serial1.write(KVH_HEADER, 4);
            Serial1.write(adxl355_a, 9);
            Serial1.write(nano33_w, 6);
            Serial1.write(nano33_a, 6);
            Serial1.write(fog, 14);
            Serial1.write(CRC32, 4);
		#endif
		#ifdef ENABLE_SRS200
			Serial.write(srs200, SRS200_SIZE);
		#endif
// 		Serial.println(t_new - t_old);
        t_old = t_new;
	}
	trig_status[1] = trig_status[0];
	clear_SEL_EN(select_fn);
}


void acq_imu_mems(byte &select_fn, unsigned int CTRLREG)
{
	byte nano33_w[6], nano33_a[6];
    byte adxl355_a[9]={0,0,0,0,0,0,0,0,0};
    uint8_t CRC8, CRC32[4];

  
    if(select_fn&SEL_IMU_MEMS)
    {
        if(CTRLREG == INT_SYNC || CTRLREG == EXT_SYNC) run_fog_flag = 1;
        else if(CTRLREG == STOP_SYNC) run_fog_flag = 0;
    }
  
	trig_status[0] = digitalRead(SYS_TRIG);
	if((trig_status[0] & ~trig_status[1] & run_fog_flag))
	{

        uint8_t* imu_data = (uint8_t*)malloc(25); // 9+4+6+6
        // adxl355.readData(adxl355_a);
        IMU.readGyroscope(nano33_w);
        IMU.readAcceleration(nano33_a);

        memcpy(imu_data, KVH_HEADER, 4);
        memcpy(imu_data+4, adxl355_a, 9);
        memcpy(imu_data+13, nano33_w, 6);
        memcpy(imu_data+19, nano33_a, 6);
        myCRC.crc_32(imu_data, 25, CRC32);
        free(imu_data);

        #ifdef UART_SERIAL_5_CMD
            mySerial5.write(KVH_HEADER, 4);
            mySerial5.write(adxl355_a, 9);
            mySerial5.write(nano33_w, 6);
            mySerial5.write(nano33_a, 6);
             mySerial5.write(CRC32, 4);
        #endif
        #ifdef UART_USB_CMD
            Serial.write(KVH_HEADER, 4);
            Serial.write(adxl355_a, 9);
            Serial.write(nano33_w, 6);
            Serial.write(nano33_a, 6);
            Serial.write(CRC32, 4);
        #endif
        #ifdef UART_RS422_CMD
            Serial1.write(KVH_HEADER, 4);
            Serial1.write(adxl355_a, 9);
            Serial1.write(nano33_w, 6);
            Serial1.write(nano33_a, 6);
            Serial1.write(CRC32, 4);
        #endif
	}
    /*--end of if-condition--*/
	trig_status[1] = trig_status[0];
	clear_SEL_EN(select_fn);
}

void acq_imu_mems_gps(byte &select_fn, unsigned int CTRLREG)
{
	byte nano33_w[6], nano33_a[6];
    byte adxl355_a[9]={0,0,0,0,0,0,0,0,0};
	byte gps_data[7];
    uint8_t CRC32[4];

    if(select_fn&SEL_IMU_MEMS)
    {
        if(CTRLREG == INT_SYNC || CTRLREG == EXT_SYNC) run_fog_flag = 1;
        else if(CTRLREG == STOP_SYNC) run_fog_flag = 0;
    }
  
	trig_status[0] = digitalRead(SYS_TRIG);
	if((trig_status[0] & ~trig_status[1] & run_fog_flag))
	{

        uint8_t* imu_data = (uint8_t*)malloc(32); // 9+4+6+6+7
        // adxl355.readData(adxl355_a);
        IMU.readGyroscope(nano33_w);
        IMU.readAcceleration(nano33_a);
		getGPStimeData(gps_data);

        memcpy(imu_data, KVH_HEADER, 4);
        memcpy(imu_data+4, adxl355_a, 9);
        memcpy(imu_data+13, nano33_w, 6);
        memcpy(imu_data+19, nano33_a, 6);
		memcpy(imu_data+25, gps_data, 7);
        myCRC.crc_32(imu_data, 32, CRC32);
        free(imu_data);

        #ifdef UART_SERIAL_5_CMD
            mySerial5.write(KVH_HEADER, 4);
            mySerial5.write(adxl355_a, 9);
            mySerial5.write(nano33_w, 6);
            mySerial5.write(nano33_a, 6);
             mySerial5.write(CRC32, 4);
        #endif
        #ifdef UART_USB_CMD
            Serial.write(KVH_HEADER, 4);
            Serial.write(adxl355_a, 9);
            Serial.write(nano33_w, 6);
            Serial.write(nano33_a, 6);
            Serial.write(CRC32, 4);
        #endif
        #ifdef UART_RS422_CMD
            Serial1.write(KVH_HEADER, 4);
            Serial1.write(adxl355_a, 9);
            Serial1.write(nano33_w, 6);
            Serial1.write(nano33_a, 6);
			Serial1.write(gps_data, 7);
            Serial1.write(CRC32, 4);
        #endif
        if (gps_valid == 1) gps_valid = 0;
	}
    /*--end of if-condition--*/
	
	trig_status[1] = trig_status[0];
	clear_SEL_EN(select_fn);
}

void acq_imu_sparrow(byte &select_fn, unsigned int CTRLREG)
{
    byte nano33_w[6], nano33_a[6], sparrow_w[6];
    byte adxl355_a[9];
    uint8_t CRC32[4];

    if(select_fn&SEL_SPARROW_DEMO)
    {
        switch(CTRLREG)
        {
            case INT_SYNC:
                run_fog_flag = 1;
                break;
            case EXT_SYNC:
                run_fog_flag = 1;
                break;
            case STOP_SYNC:
                run_fog_flag = 0;
                sparrow.setStop();
                break;
            case START_SPARROW:
                sparrow.startRead(1);
                break;
            default:
                break;
        }
//         if(CTRLREG == INT_SYNC || CTRLREG == EXT_SYNC) run_fog_flag = 1;
//         else if(CTRLREG == STOP_SYNC) run_fog_flag = 0;
//         sparrow.startRead(1);
    }

	trig_status[0] = digitalRead(SYS_TRIG);
	if((trig_status[0] & ~trig_status[1] & run_fog_flag))
	{

        uint8_t* imu_data = (uint8_t*)malloc(31); // 6+9+4+6+6
		adxl355.readData(adxl355_a);
		IMU.readGyroscope(nano33_w);
		IMU.readAcceleration(nano33_a);
		sparrow.readData(sparrow_w);
        // convertGyro(sparrow_w);

        memcpy(imu_data, KVH_HEADER, 4);
        memcpy(imu_data+4, adxl355_a, 9);
        memcpy(imu_data+13, nano33_w, 6);
        memcpy(imu_data+19, nano33_a, 6);
        memcpy(imu_data+25, sparrow_w, 6);
        myCRC.crc_32(imu_data, 31, CRC32);
		free(imu_data);

        #ifdef UART_SERIAL_5_CMD
            mySerial5.write(KVH_HEADER, 4);      
            mySerial5.write(adxl355_a, 9);
            mySerial5.write(nano33_w, 6);
            mySerial5.write(nano33_a, 6);
            mySerial5.write(sparrow_w, 6);
            mySerial5.write(CRC32, 4);
        #endif
        #ifdef UART_USB_CMD
            Serial.write(KVH_HEADER, 4); 
            Serial.write(adxl355_a, 9);
            Serial.write(nano33_w, 6);
            Serial.write(nano33_a, 6);
            Serial.write(sparrow_w, 6);
            Serial.write(CRC32, 4);
        #endif
        #ifdef UART_RS422_CMD
            t_new = micros();
            Serial1.write(KVH_HEADER, 4);
            Serial1.write(adxl355_a, 9);
            Serial1.write(nano33_w, 6);
            Serial1.write(nano33_a, 6);
            Serial1.write(sparrow_w, 6);
            Serial1.write(CRC32, 4);
            Serial.println(t_new - t_old);
            t_old = t_new;
        #endif
	}
    /*--end of if-condition--*/
	trig_status[1] = trig_status[0];
	clear_SEL_EN(select_fn);
}


void acq_imu_fake(byte &select_fn, unsigned int CTRLREG)
{
	byte adxl355_a[9], header[4], fog[17], nano33_w[6], nano33_a[6];

	if(select_fn&SEL_IMU)
	{
		run_fog_flag = pig_v2.setSyncMode(CTRLREG);
	}
	// adxl355.readFakeData(adxl355_a);
	// IMU.readFakeGyroscope(nano33_w);
	// IMU.readFakeAcceleration(nano33_a);
	if(run_fog_flag){
		// adxl355.readFakeData(adxl355_a);
    adxl355.readData(adxl355_a);
		// IMU.readFakeGyroscope(nano33_w);
		// IMU.readFakeAcceleration(nano33_a);
		IMU.readGyroscope(nano33_w);
		IMU.readAcceleration(nano33_a);
		pig_v2.readFakeDataCRC(header, fog);
    #ifdef UART_RS422
      Serial1.write(header, 4);
      Serial1.write(fog, 17);
      Serial1.write(adxl355_a, 9);
      Serial1.write(nano33_w, 6);
      Serial1.write(nano33_a, 6);
    #endif
		#ifdef UART_USB
      Serial.write(header, 4);
      Serial.write(fog, 17);
      Serial.write(adxl355_a, 9);
      Serial.write(nano33_w, 6);
      Serial.write(nano33_a, 6);
    #endif
		delay(10);
	}
	clear_SEL_EN(select_fn);
}

void convertGyro(byte data[6])
{
	int fog=0;

	fog = data[0]<<24 | data[1]<<16 | data[2]<<8 | data[3];
  // fog = fog
    Serial.print(data[0], HEX);
    Serial.print("\t");
    Serial.print(data[1], HEX);
    Serial.print("\t");
    Serial.print(data[2], HEX);
    Serial.print("\t");
    Serial.print(data[3], HEX);
    Serial.print("\t");
    Serial.print(data[4], HEX);
    Serial.print("\t");
    Serial.print(data[5], HEX);
    Serial.print("\t");
	Serial.println(fog);
};


void print_nano33XlmData(byte *data)
{
	int ax, ay, az;
	t_new = micros();
	ax = data[0]<<8 | data[1];
	ay = data[2]<<8 | data[3];
	az = data[4]<<8 | data[5];
	Serial.print(t_new - t_old);
	Serial.print('\t');
	Serial.print((float)ax*NANO33_XLM);
	Serial.print('\t');
	Serial.print((float)ay*NANO33_XLM);
	Serial.print('\t');
	Serial.println((float)az*NANO33_XLM);
	t_old = t_new;
}

void clear_SEL_EN(byte &select_fn)
{
	select_fn = SEL_DEFAULT;
}

#ifdef ENABLE_SRS200
void readSRS200Data(unsigned char header[2], unsigned char data[SRS200_SIZE])
{
	// checkHeader(header);
	mySerial5.readBytes(data, SRS200_SIZE);
	// printSRSdata(data);
}

void printSRSdata(unsigned char data[10])
{
	long omega;

	omega = (long)(data[3]<<24|data[2]<<16|data[1]<<8|data[0]);
	t_new = micros();
	// Serial.print(t_new - t_old);
	// Serial.print('\t');
	// Serial.print(data[3]);
	// Serial.print('\t');
	// Serial.print(data[2]);
	// Serial.print('\t');
	// Serial.print(data[1]);
	// Serial.print('\t');
	// Serial.print(data[0]);
	// Serial.print('\t');
	if((data[3]>>7) == 1)
		omega = omega - (long)(1<<32);
	Serial.println(omega);
	t_old = t_new;
}
#endif

#ifdef ENABLE_SRS200
unsigned char* checkHeader(unsigned char headerArr[2])
{
	unsigned char header[2], hold;
	
	mySerial5.readBytes(headerArr, 2);
	hold = 1;
	while(hold)
	{
		if(	(headerArr[0] == 0xC0) && 
			(headerArr[1] == 0xC0)
			){
				// Serial.print("A");
				// Serial.print(" ");
				// Serial.print(headerArr[0], HEX);
				// Serial.print(" ");
				// Serial.println(headerArr[1], HEX);
				hold = 0;
				return headerArr ;
			}
		else {
			// Serial.print("B");
			// Serial.print(" ");
			// Serial.print(headerArr[0], HEX);
			// Serial.print(" ");
			// Serial.println(headerArr[1], HEX);
			headerArr[0] = headerArr[1];
			headerArr[1] = mySerial5.read();
			delayMicroseconds(100);
		}
	}
}
#endif

void sendCmd(unsigned char addr, unsigned int value)
{
	Serial1.write(addr);
	Serial1.write(value>>24 & 0xFF);
	Serial1.write(value>>16 & 0xFF);
	Serial1.write(value>>8 & 0xFF);
	Serial1.write(value & 0xFF);
	delay(1);
}

void updateGPS(unsigned int ms)
{
  if(( millis()-gps_init_time ) >= ms)
  {
    gps_init_time = millis();
    while (mySerial13.available())
          gps.encode(mySerial13.read());
	gps_date = gps.date.value();
	gps_time = gps.time.value();
	gps_valid = gps.date.isValid() & gps.time.isValid();
  displayGPSInfo();
	// Serial.print(gps_date);
	// Serial.print(", ");
	// Serial.println(gps_time);
  }
}

void getGPStimeData(byte data[7])
{
	// gps_valid = 1;
	data[0] = gps_date>>8;
	data[1] = gps_date;
	data[2] = gps_time >> 24;
	data[3] = gps_time >> 16;
	data[4] = gps_time >> 8;
	data[5] = gps_time;
	data[6] = gps_valid;
}

void displayGPSInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.print(" ");
  Serial.print(gps_valid);
  Serial.println();
}