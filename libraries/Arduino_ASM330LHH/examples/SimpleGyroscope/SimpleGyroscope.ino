/*
  Arduino LSM6DS3 - Simple Gyroscope

  This example reads the gyroscope values from the LSM6DS3
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Uno WiFi Rev 2 or Arduino Nano 33 IoT

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include <Arduino.h>
#include "wiring_private.h"
#include <ASM330LHH.h>

#define SENS_GYRO_250 0.00875

unsigned int t_new, t_old;

// SPI
SPIClassSAMD dev_spi(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
#define CHIP_SELECT_PIN 2
ASM330LHHClass IMU(dev_spi, CHIP_SELECT_PIN, -1);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  dev_spi.begin();
  pinPeripheral(3, PIO_SERCOM_ALT);
  pinPeripheral(22, PIO_SERCOM_ALT);
  pinPeripheral(23, PIO_SERCOM_ALT);

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