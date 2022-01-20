/*
  Arduino LSM6DS3 - Simple Accelerometer

  This example reads the acceleration values from the LSM6DS3
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Uno WiFi Rev 2 or Arduino Nano 33 IoT

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include <Arduino_LSM6DS3.h>

#define SENS_AXLM_4G 0.000122
unsigned long t_new, t_old;
void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");
}

void loop() {
  byte data[6];

	if (IMU.accelerationAvailable()) {
	IMU.readAcceleration(data);
	print_axlmData(data);
	}
}

void print_axlmData(byte *temp_a)
{
	int ax, ay, az;
	
	ax = temp_a[1]<<8 | temp_a[0];
	if((ax>>15) == 1) ax = ax - (1<<16);
	ay = temp_a[3]<<8 | temp_a[2];
	if((ay>>15) == 1) ay = ay - (1<<16);
	az = temp_a[5]<<8 | temp_a[4];
	if((az>>15) == 1) az = az - (1<<16);
	
	t_new = micros();
	Serial.print(t_new - t_old);
	Serial.print('\t');
	// Serial.print(wx, HEX);
	Serial.print((float)ax*SENS_AXLM_4G);
	Serial.print('\t');
	// Serial.print(wy, HEX);
	Serial.print((float)ay*SENS_AXLM_4G);
	Serial.print('\t');
	// Serial.println(wz, HEX);
	Serial.println((float)az*SENS_AXLM_4G);
	t_old = t_new;
}