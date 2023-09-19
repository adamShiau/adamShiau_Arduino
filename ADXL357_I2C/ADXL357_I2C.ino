#include "adxl357_I2C.h"
#include <Arduino.h>
#include "wiring_private.h"


/*** trig pin***/
#define SYS_TRIG 14



int pin_scl_mux = 12;
bool trig_status[2] = {0, 0};
unsigned int t_new, t_old=0;

//PWM
#include <SAMD21turboPWM.h>
#define PWM100 7
#define PWM200 5
#define PWM250 11
#define PWM_FIX 0.981
// #define PWM_FIX 0.978
TurboPWM  pwm;

// I2C
#include <Wire.h>
#define ADXL357_ADDR     0x1D  //Adxl357 I2C address
#define I2C_STANDARD_MODE   100000
#define I2C_FAST_MODE     400000
#define I2C_FAST_MODE_PLUS     1000000
//#define I2C_HIGH_SPEED_MODE    3400000 //can not work
#define TEST_ADDR      0xAB
/*** TwoWire Wire(&sercom, PIN_WIRE_SDA, PIN_WIRE_SCL);***/
TwoWire myWire(&sercom0, 27, 20);
void SERCOM0_Handler()
{
  myWire.onService();
}

Adxl357_I2C adxl357_i2c(myWire);

void setup() {
	Serial.begin(230400);
	myWire.begin();
	myWire.setClock(I2C_FAST_MODE_PLUS);
	pinPeripheral(27, PIO_SERCOM);
	pinPeripheral(20, PIO_SERCOM);
	adxl357_i2c.init();
	pinMode(SYS_TRIG, INPUT);


/*** pwm ***/

  pwm.setClockDivider(2, false); //48MHz/4 = 12MHz
  pwm.timer(2, 2, int(24000*PWM_FIX), false); //12M/2/24000 = 250Hz
  pwm.timer(1, 2, int(60000*PWM_FIX), false); //12M/2/60000 = 100Hz
  pwm.timer(0, 2, int(30000*PWM_FIX), false); //12M/2/30000 = 200Hz
  
  pwm.analogWrite(PWM100, 500);  
  pwm.analogWrite(PWM200, 500);  
  pwm.analogWrite(PWM250, 500);

}

void loop() {
	byte acc[9];
	
//		 adxl357.printRegAll();
		adxl357_i2c.readData(acc);
		print_adxl357Data_i2c(acc);

}

void print_adxl357Data_i2c(byte *temp_a)
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
	Serial.print((float)accX*SENS_10G);
	Serial.print('\t');
	Serial.print((float)accY*SENS_10G);
	Serial.print('\t');
	Serial.println((float)accZ*SENS_10G);
	t_old = t_new;
}
