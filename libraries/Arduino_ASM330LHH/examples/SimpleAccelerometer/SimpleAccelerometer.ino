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

#include <Arduino.h>
#include "wiring_private.h"
#include <ASM330LHH.h>

#define SENS_AXLM_4G 0.000122
unsigned int t_new, t_old=0;

// SPI
#define SPI_CLOCK_8M 8000000
#define SPI_CLOCK_1M 1000000
SPIClassSAMD dev_spi(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
#define CHIP_SELECT_PIN 2
ASM330LHHClass IMU(dev_spi, CHIP_SELECT_PIN, SPI_CLOCK_8M);

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

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");
}

void loop() {
	byte data[6];
	int x, y, z;

	if (IMU.accelerationAvailable()) {
	IMU.readAcceleration(data);
	IMU.print_AccelerationData(data, x, y, z, t_new, t_old);
	// print_axlmData(data);
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