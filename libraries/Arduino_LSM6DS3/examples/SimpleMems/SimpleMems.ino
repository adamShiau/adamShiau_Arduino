#include <Arduino_LSM6DS3.h>

unsigned int t_new, t_old=0;
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
    // IMU.print_GyroData(nano33_w, x, y, z, t_new, t_old);
  }

  if (IMU.accelerationAvailable()) {
	IMU.readAcceleration(nano33_a);
  // IMU.print_AccelerationData(nano33_a, x, y, z, t_new, t_old);
  }
}
