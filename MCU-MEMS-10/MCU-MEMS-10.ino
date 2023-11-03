#include <ASM330LHHSensor.h>
// #include "pig_v2.h"
#include "IMU_PIG_DEFINE.h"
#include "wiring_private.h"
#include "crcCalculator.h"
#include "uartRT.h"
// #include <TinyGPSPlus.h>
// #include <EEPROM_24AA32A_I2C.h>


// interrupt for EXT_SYNC to FPGA
#define PIG_SYNC 29 //PA22
// RST to FPGA nConfig
#define nCONFIG 12
//MCU LED
#define MCU_LED A2

//PWM
#include <SAMD21turboPWM.h>
#define PWM100 7
#define PWM200 5
#define PWM250 11
#define PWM_FIX 1
// #define PWM_FIX 0.978
TurboPWM  pwm;

#define SPI_SLAVE_DATA_SIZE 16

unsigned int t_new, t_old=0;


typedef union
{
  float float_val;
  uint8_t bin_val[4];
  uint32_t ulong_val;
}
my_time_t;

my_time_t mcu_time;

typedef union
{
  float float_val[3];
  uint8_t bin_val[12];
  int int_val[3];
}
my_acc_t;

typedef union
{
  float float_val[4];
  uint8_t bin_val[16];
  unsigned long ulong_val[4];
}
my_att_t;

typedef union
{
  float float_val;
  uint8_t bin_val[4];
  int int_val;
}
my_float_t;

my_float_t my_f;
unsigned char fog_op_status;

// SPI
#include <SPI.h>
#define SPI_CLOCK_8M 8000000
#define SPI_CLOCK_1M 1000000

// SPIClassSAMD mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);

#define CHIP_SELECT_PIN 10

ASM330LHHSensor IMU(&Wire, ASM330LHH_I2C_ADD_L);
// cmd read from GUI
uint8_t myCmd_header[] = {0xAB, 0xBA};
uint8_t myCmd_trailer[] = {0x55, 0x56};
uint16_t myCmd_try_cnt;
const uint8_t myCmd_sizeofheader = sizeof(myCmd_header);
const uint8_t myCmd_sizeoftrailer = sizeof(myCmd_trailer);
uartRT myCmd(Serial1, 6);


// UART
//SERCOM2: serial2 (PA14, PA15) [tx,  rx]
Uart Serial2 (&sercom2, 25, 24, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM2_Handler()
{
    Serial2.IrqHandler();
}
// PIG pig_ser2(Serial2);
uint8_t header[] = {0xAB, 0xBA};
uint8_t trailer[] = {0x55};
uint16_t try_cnt;
const uint8_t sizeofheader = sizeof(header);
const uint8_t sizeoftrailer = sizeof(trailer);

//SERCOM1: serial3 (PA17, PA18) [rx, tx]
Uart Serial3 (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);
void SERCOM1_Handler()
{
    Serial3.IrqHandler();
}
// PIG pig_ser3(Serial3);

//SERCOM3: serial4 (PA21, PA20) [rx, tx]
Uart Serial4 (&sercom3, 10, 9, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM3_Handler()
{
  Serial4.IrqHandler();
}
// PIG sp13(Serial2); //SP13
// PIG sp14(Serial3, 14); //SP14
// PIG sp9(Serial4); //SP14


/*** serial data from PC***/
byte rx_cnt = 0, cmd, fog_channel;
int value;
bool cmd_complete;

/*** output mode flag***/
byte mux_flag;

/*** acq fog flag***/
bool run_fog_flag;

/*** select running fn flag***/
byte select_fn;
// bool fn_lock;

/*** Control register to control the functionality  of each optput function***/
unsigned int CtrlReg=-1;

/*** KVH HEADER ***/
const unsigned char KVH_HEADER[4] = {0xFE, 0x81, 0xFF, 0x55};
const unsigned char PIG_HEADER[2] = {0xAB, 0xBA};

typedef void (*fn_ptr) (byte &, unsigned int, byte);
fn_ptr output_fn;

// Adxl355 adxl355(pin_scl_mux);
crcCal myCRC;

//SYNC OUT
bool sync_status = 0;

//ISR_FLAG
volatile bool ISR_Coming = false;
volatile bool ISR_PEDGE;

int t_adc = millis();


int tt0=0, tt1, tt2, tt3;

unsigned int MCU_cnt = 0;


byte *reg_fog;

unsigned int t_previous = 0;

void setup() {
  pinMode(CHIP_SELECT_PIN, OUTPUT);
  digitalWrite(CHIP_SELECT_PIN, HIGH);
  
    // EXTT
    /*** for IMU_V4  : EXTT = PA27, Variant pin = 26, EXINT[15]
     *   for PIG MCU : EXTT = PA27, Variant pin = 26, EXINT[15]
     *   for NANO33 iot: EXTT = PA21, Variant pin = 10, EXINT[5]
     *   for NANO33 iot: EXTT = PB03, Variant pin = 21, EXINT[3]
     *  ****/
  // attachInterrupt(10, ISR_EXTT, CHANGE);
  attachInterrupt(21, ISR_EXTT, CHANGE);

/*** see datasheet p353. 
 *  SENSEn register table:
 * -----------------------------
 *  SENSEn[2:0] |   Name    |   Description   |
 *  ----------------------------------------------------
 *       0x0    |   NONE    | No detection
 *  ----------------------------------------------------
 *       0x1    |   RISE    |  Rising-edge detection
 *  -----------------------------------------------------
 *       0x2    |   FALL    |  Falling-edge detection
 *  -----------------------------------------------------
 *       0x3    |   BOTH    |  Both-edges detection
 *  -----------------------------------------------------
 * ***/
// set interrupt mode to None
  /***----- for PIG MCU & IMU_V4 EXINT[15]----- ***/
  // EIC->CONFIG[1].bit.SENSE7 = 0;  // set ISR no NONE
  /*** for EXINT[5]*/
  // EIC->CONFIG[0].bit.SENSE5 = 0;  // set ISR no NONE
  EIC->CONFIG[0].bit.SENSE3 = 0;  // set ISR no NONE

  
	Serial.begin(230400); //debug
	Serial1.begin(230400); //to PC
  // Serial2.begin(115200); //fog
  // Serial3.begin(115200);
  // Serial4.begin(115200);

  // pinPeripheral(24, PIO_SERCOM);
  // pinPeripheral(25, PIO_SERCOM);

  // pinPeripheral(8, PIO_SERCOM);
  // pinPeripheral(13, PIO_SERCOM);
  // //
  // pinPeripheral(10, PIO_SERCOM_ALT);
  // pinPeripheral(9, PIO_SERCOM_ALT);

  //I2C
  Wire.begin();
  IMU.init(); //setting MEMS IMU parameters 
  // myWire.begin();
  // myWire.setClock(I2C_FAST_MODE);
  // pinPeripheral(27, PIO_SERCOM);
  // pinPeripheral(20, PIO_SERCOM);

  //SPI
  SPI.begin();
  // SPI.beginTransaction(SPISettings(SPI_CLOCK_1M, MSBFIRST, SPI_MODE0));
  
  // mySPI.begin();
  // pinPeripheral(3, PIO_SERCOM_ALT);
  // pinPeripheral(22, PIO_SERCOM_ALT);
  // pinPeripheral(23, PIO_SERCOM_ALT);



  // byte FPGA_wakeup_flag = 0; 
  // Wait_FPGA_Wakeup(FPGA_wakeup_flag, 2);
  // Blink_MCU_LED();

  // parameter_init();
  // Blink_MCU_LED();
	
  IMU.init(); //setting MEMS IMU parameters 
  
    
	/*** var initialization***/
	cmd_complete = 0;
	mux_flag = MUX_ESCAPE; 		//default set mux_flag to 2
	select_fn = SEL_DEFAULT; 	//default set select_fn to 128
	// select_fn = SEL_IMU;
	run_fog_flag = 0;
	output_fn = temp_idle;


	
/*** pwm ***/

  pwm.setClockDivider(2, false); //48MHz/4 = 12MHz
  pwm.timer(1, 2, int(60000*PWM_FIX), false); //12M/2/60000 = 100Hz
  pwm.analogWrite(PWM100, 500);


}

void loop() {

	getCmdValue(cmd, value, fog_channel, cmd_complete);
	cmd_mux(cmd_complete, cmd, mux_flag);
	parameter_setting(mux_flag, cmd, value, fog_channel);
	output_mode_setting(mux_flag, cmd, select_fn);
	output_fn(select_fn, value, fog_channel);
}

void printAdd(char name[], void* addr)
{
	Serial.print(name);
	Serial.print(": ");
	Serial.println((unsigned int)addr, HEX);
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

void getCmdValue(byte &uart_cmd, int &uart_value, byte &fog_ch, bool &uart_complete)
{
  byte *cmd;

    cmd = myCmd.readData(myCmd_header, myCmd_sizeofheader, &myCmd_try_cnt, myCmd_trailer, myCmd_sizeoftrailer);

    if(cmd){
      uart_cmd = cmd[0];
      uart_value = cmd[1]<<24 | cmd[2]<<16 | cmd[3]<<8 | cmd[4];
      fog_ch = cmd[5];
      uart_complete = 1;
      // printVal_0("uart_cmd", uart_cmd);
      // printVal_0("uart_value", uart_value);
      Serial.print("cmd, value, ch: ");
      Serial.print(uart_cmd);
      Serial.print(", ");
      Serial.print(uart_value);
      Serial.print(", ");
      Serial.println(fog_ch);
      // eeprom.Parameter_Write(EEPROM_ADDR_REG_VALUE, uart_value);
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


void parameter_setting(byte &mux_flag, byte cmd, int value, byte fog_ch) 
{
	if(mux_flag == MUX_PARAMETER)
	{
    
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
				output_fn = fn_rst;
				select_fn = SEL_FOG_1;
				break;
			}
			case MODE_IMU: {
				output_fn = acq_imu; 
				select_fn = SEL_IMU;
				break;
			}
			case MODE_FOG_HP_TEST: {
				output_fn = fn_rst; 
				select_fn = SEL_HP_TEST;
				break;
			}
			case MODE_EQ: {
				output_fn = fn_rst;
				select_fn = SEL_EQ;
				break;
            }
      case MODE_FOG_PARAMETER: {
          output_fn = fn_rst;
          select_fn = SEL_FOG_PARA;
          break;
      }

      default: break;
      }

	}
  
}

void temp_idle(byte &select_fn, unsigned int CTRLREG, byte ch)
{
	clear_SEL_EN(select_fn);
	// delay(100);
}

void fn_rst(byte &select_fn, unsigned int CTRLREG, byte ch)
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


void acq_imu(byte &select_fn, unsigned int value, byte ch)
{
  my_acc_t my_memsXLM, my_memsGYRO;
  my_float_t pd_temp;

  byte *fog;
	uint8_t CRC32[4];

  if(select_fn&SEL_IMU)
	{
    Serial.print("Enter acq_imu mode, channel: ");
    Serial.println(ch);
    CtrlReg = value;

    switch(CtrlReg){
      case INT_SYNC:
        // EIC->CONFIG[0].bit.SENSE5 = 0; //set interrupt condition to None
        EIC->CONFIG[0].bit.SENSE3 = 0; //set interrupt condition to None
        // eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        // setupWDT(11);
        
      break;
      case EXT_SYNC:
        Serial.println("Enter EXT_SYNC mode");
        Serial.println("Set EXTT to CHANGE");

        // EIC->CONFIG[0].bit.SENSE5 = 3; ////set interrupt condition to Both
        EIC->CONFIG[0].bit.SENSE3 = 3; ////set interrupt condition to Both
        run_fog_flag = true;
        // eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
        // setupWDT(11);

      break;
      case STOP_SYNC:
        // EIC->CONFIG[0].bit.SENSE5 = 0; //set interrupt condition to None
        EIC->CONFIG[0].bit.SENSE3 = 0; //set interrupt condition to None
        run_fog_flag = false;
        // eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
        // disableWDT();
      break;

      default:
      break;
    }
    t_previous = millis();
	}

	if(run_fog_flag) {
        t_new = micros();


    if(ISR_PEDGE)
    {
      uint8_t* imu_data = (uint8_t*)malloc(32); // KVH_HEADER:4 + adxl355:9 + nano33_w:6 + nano33_a:6 + pig:14
      mcu_time.ulong_val = millis() - t_previous;
      // uint8_t tt[8] = {0xFE, 0x81, 0xFF, 0x55, 0x01, 0x02 ,0x03, 0x04};
      uint8_t tt[4] = {0x1, 0x02 ,0x03, 0x4};

      ISR_PEDGE = false;

      IMU.Get_X_Axes_f(my_memsXLM.float_val);
      IMU.Get_G_Axes_f(my_memsGYRO.float_val);

      memcpy(imu_data, KVH_HEADER, 4);
      memcpy(imu_data+4, my_memsGYRO.bin_val, 12);//wx, wy, wz
      memcpy(imu_data+16, my_memsXLM.bin_val, 12);//ax, ay, az
      memcpy(imu_data+28, mcu_time.bin_val, 4);
      myCRC.crc_32(imu_data, 32, CRC32);

      
      #ifdef UART_RS422_CMD
        Serial1.write(KVH_HEADER, 4);
        Serial1.write(my_memsGYRO.bin_val, 12);
        Serial1.write(my_memsXLM.bin_val, 12);
        Serial1.write(mcu_time.bin_val, 4);
        Serial1.write(CRC32, 4);
      #endif   
      // getImuData(imu_data + 4);
      digitalWrite(CHIP_SELECT_PIN, LOW);
      SPI.transfer(imu_data, 32); // send imu data to slave
      getSlaveData(10); // get attitude data from slave
      digitalWrite(CHIP_SELECT_PIN, HIGH);
      free(imu_data);
    }
    t_old = t_new;    
    // resetWDT();
	}
	clear_SEL_EN(select_fn);
}

void getSlaveData(uint8_t dly)
{
  my_att_t my_altitude;

  delayMicroseconds(dly);
  SPI.transfer(98);
  for(int i=0; i<SPI_SLAVE_DATA_SIZE; i++)
  {
    delayMicroseconds(dly);
    my_altitude.bin_val[i] = SPI.transfer(0);
  }

  for(int i=0; i<3; i++){
    Serial.print(my_altitude.float_val[i]);
    Serial.print(", ");
  }
  Serial.println(my_altitude.ulong_val[3]);
}

void getImuData(uint8_t *data)
{
  my_acc_t my_memsXLM, my_memsGYRO;
  my_time_t my_time;

  memcpy(&my_memsGYRO, data, sizeof(my_memsGYRO));
  memcpy(&my_memsXLM, data + 12, sizeof(my_memsXLM));
  memcpy(&my_time, data + 24, sizeof(my_time));

  for(int i=0; i<3; i++){
    Serial.print(my_memsGYRO.float_val[i]);
    Serial.print(", ");
  }
  for(int i=0; i<3; i++){
    Serial.print(my_memsXLM.float_val[i]);
    Serial.print(", ");
  }
  Serial.println(my_time.ulong_val);
}


float convert_PDtemp(byte dataH, byte dataL)
{
  return (int8_t)dataH + (dataL>>7)*0.5;
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
  // digitalWrite(PIG_SYNC, LOW);
}


void sendCmd(unsigned char addr, unsigned int value)
{
	Serial1.write(addr);
	Serial1.write(value>>24 & 0xFF);
	Serial1.write(value>>16 & 0xFF);
	Serial1.write(value>>8 & 0xFF);
	Serial1.write(value & 0xFF);
	delay(1);
}

void ISR_EXTT()
{
  // Serial.println(ISR_PEDGE);
  ISR_Coming = !ISR_Coming;
  if(ISR_Coming == true) ISR_PEDGE = true;
  
}


