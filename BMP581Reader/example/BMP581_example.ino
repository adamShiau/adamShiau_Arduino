#include "BMP581Reader.h"


BMP5_Sensor bar_sensor;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  Serial.println("init");
  Wire.begin();
  while(!bar_sensor.begin());
  bar_sensor.setODR(100);

  Serial.println("Start Measurement.");
  delay(1000);
}


void loop() { 
  float bar = 0, temp = 0;
  bar_sensor.getSensorData(&bar, &temp);
  Serial.print("Time: ");
  Serial.print(millis() / 1000.0f);
  Serial.print("   Pressure: ");
  Serial.print(bar);
  Serial.print("   Temperature: ");
  Serial.println(temp);
  delay(5);
}
