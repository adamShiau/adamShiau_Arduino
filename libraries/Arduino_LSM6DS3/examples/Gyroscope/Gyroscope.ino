/**
 * @file Gyroscope.ino
 * @author Adam
 * @brief 
 * @version 0.1
 * @date 2023-05-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <Arduino_LSM6DS3.h>

#define SENS_GYRO_250 0.00875

unsigned int t_new, t_old;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
  t_old = micros();
}

void loop() {
	byte data[6];
	int x, y, z;
	
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(data);
	IMU.print_GyroData(data, x, y, z, t_new, t_old);
    // print_gyroData(data);
	
	// IMU.readGyroscope(x, y, z);
	// print_gyroData(x, y, z);
  }
}

void print_gyroData(int x, int y, int z)
{
	t_new = micros();
	Serial.print(t_new - t_old);
	Serial.print('\t');
	// Serial.print((float)x*SENS_GYRO_250);
	Serial.print(x, HEX);
	Serial.print('\t');
	Serial.print(y, HEX);
	Serial.print('\t');
	Serial.println(z, HEX);
	t_old = t_new;
}

void print_gyroData(byte *temp_a)
{
	int wx, wy, wz;
	
	wx = temp_a[1]<<8 | temp_a[0];
	if((wx>>15) == 1) wx = wx - (1<<16);
	wy = temp_a[3]<<8 | temp_a[2];
	if((wy>>15) == 1) wy = wy - (1<<16);
	wz = temp_a[5]<<8 | temp_a[4];
	if((wz>>15) == 1) wz = wz - (1<<16);
	
	t_new = micros();
	Serial.print(t_new - t_old);
	Serial.print('\t');
	// Serial.print(wx, HEX);
	Serial.print((float)wx*SENS_GYRO_250);
	Serial.print('\t');
	// Serial.print(wy, HEX);
	Serial.print((float)wy*SENS_GYRO_250);
	Serial.print('\t');
	// Serial.println(wz, HEX);
	Serial.println((float)wz*SENS_GYRO_250);
	t_old = t_new;
}