#include "adxl355.h"
#include <Arduino_LSM6DS3.h>

/*** adxl355 conversion factor***/
#define ADXL355_8G 0.0000156
#define ADXL355_4G 0.0000078
#define ADXL355_2G 0.0000039

/*** Nano33 conversion facto***/
#define NANO33_XLM 0.000122 // +/- 4g, 4/32768
#define NANO33_GYRO 0.00763 // +/- 250dps, 250/32768

/*** trig pin***/
#define SYS_TRIG 14


int pin_scl_mux = 12;
bool trig_status[2] = {0, 0};
unsigned int t_new, t_old=0;

Adxl355 adxl355(pin_scl_mux);

void setup() {
	IMU.begin();
	adxl355.init();
	pinMode(SYS_TRIG, INPUT);
}

void loop() {
	byte *a_adxl355, acc[9];
	int wx_nano33, wy_nano33, wz_nano33;
	int ax_nano33, ay_nano33, az_nano33;
	
	trig_status[0] = digitalRead(SYS_TRIG);
	if(trig_status[0] & ~trig_status[1]) {
		// adxl355.printRegAll();
		a_adxl355 = adxl355.readData(acc);
		IMU.readGyroscope(wx_nano33, wy_nano33, wz_nano33);
		IMU.readAcceleration(ax_nano33, ay_nano33, az_nano33);
		// print_adxl355Data(a_adxl355);
		// print_nano33GyroData(wx_nano33, wy_nano33, wz_nano33);
		// print_nano33XlmData(ax_nano33, ay_nano33, az_nano33);
	}
	trig_status[1] = trig_status[0];
}

void print_nano33GyroData(int wx, int wy, int wz)
{
	t_new = micros();
	Serial.print(t_new - t_old);
	Serial.print('\t');
	Serial.print((float)wx*NANO33_GYRO);
	Serial.print('\t');
	Serial.print((float)wy*NANO33_GYRO);
	Serial.print('\t');
	Serial.println((float)wz*NANO33_GYRO);
	t_old = t_new;
}

void print_nano33XlmData(int ax, int ay, int az)
{
	t_new = micros();
	Serial.print(t_new - t_old);
	Serial.print('\t');
	Serial.print((float)ax*NANO33_XLM);
	Serial.print('\t');
	Serial.print((float)ay*NANO33_XLM);
	Serial.print('\t');
	Serial.println((float)az*NANO33_XLM);
	t_old = t_new;
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
	Serial.print((float)accX*ADXL355_8G);
	Serial.print('\t');
	Serial.print((float)accY*ADXL355_8G);
	Serial.print('\t');
	Serial.println((float)accZ*ADXL355_8G);
	t_old = t_new;
}