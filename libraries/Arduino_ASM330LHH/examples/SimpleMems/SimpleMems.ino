#include <Arduino.h>
#include "wiring_private.h"
#include <ASM330LHH.h>

unsigned int t_new, t_old=0;

// SPI
SPIClassSAMD dev_spi(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
#define CHIP_SELECT_PIN 2
ASM330LHHClass IMU(dev_spi, CHIP_SELECT_PIN, -1);

void setup() {

  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }
}

void loop() {
  byte nano33_w[6], nano33_a[6];
  int x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(nano33_w);
  }

  // if (IMU.accelerationAvailable()) {
	// IMU.readAcceleration(nano33_a);
  // }
}
