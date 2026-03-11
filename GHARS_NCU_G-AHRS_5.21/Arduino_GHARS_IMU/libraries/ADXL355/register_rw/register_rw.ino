#include "adxl355.h"

#define SENS_8G 0.0000156
#define SENS_4G 0.0000078
#define SENS_2G 0.0000039

/*** trig pin***/
#define SYS_TRIG 14


int pin_scl_mux = 12;
bool trig_status[2] = {0, 0};
unsigned int t_new, t_old=0;

Adxl355 adxl355(pin_scl_mux);

void setup() {
	adxl355.init();
	pinMode(SYS_TRIG, INPUT);
}

void loop() {

adxl355.printRegAll();
delay(500);
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
