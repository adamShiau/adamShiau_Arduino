#include "adxl355.h"
#include <SAMD21turboPWM.h>

#define SENS_8G 0.0000156
#define SENS_4G 0.0000078
#define SENS_2G 0.0000039

/*** trig pin***/
#define SYS_TRIG 12


int pin_scl_mux = 17;
bool trig_status[2] = {0, 0};
unsigned int t_new, t_old=0;

Adxl355 adxl355(pin_scl_mux);
TurboPWM  pwm;

void setup() {
	adxl355.init();
	pinMode(SYS_TRIG, INPUT);

  /*** pwm ***/

	/*** Set input clock divider and Turbo Mode (which uses a 96MHz instead of a 48Mhz input clock): ***/
	pwm.setClockDivider(200, false); // Main clock 48MHz divided by 200 => 240KHz
	
	/*** Initialise timer x, with prescaler, with steps (resolution), 
	with fast aka single-slope PWM (or not -> double-slope PWM): 
	For the Arduino Nano 33 IoT, you need to initialise timer 1 for pins 4 and 7, timer 0 for pins 5, 6, 8, and 12, 
	and timer 2 for pins 11 and 13;
	***/
	pwm.timer(2, 2, 240, false);   // Use timer 2 for pin 11, divide clock by 4, resolution 600, dual-slope PWM
	pwm.analogWrite(11, 500);        // PWM frequency = 120000/step/2, dutycycle is 500 / 1000 * 100% = 50%
}

void loop() {
	byte acc[9];
	
	trig_status[0] = digitalRead(SYS_TRIG);
	if(trig_status[0] & ~trig_status[1]) {
		// adxl355.printRegAll();
		adxl355.readData(acc);
		print_adxl355Data(acc);
	}
	trig_status[1] = trig_status[0];
}

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
	Serial.print((float)accX*SENS_8G);
	Serial.print('\t');
	Serial.print((float)accY*SENS_8G);
	Serial.print('\t');
	Serial.println((float)accZ*SENS_8G);
	t_old = t_new;
}