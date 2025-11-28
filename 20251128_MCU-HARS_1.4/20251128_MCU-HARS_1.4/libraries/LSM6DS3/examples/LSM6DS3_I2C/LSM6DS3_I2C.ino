#include <Arduino.h>
#include <ASM330LHHSensor.h>
#include "wiring_private.h"


// Components
ASM330LHHSensor IMU(&Wire, ASM330LHH_I2C_ADD_L);
typedef union
{
  float float_val[3];
  uint8_t bin_val[12];
  int int_val[3];
}
my_acc_t;

void setup() {

  delay(2000);
  // Initialize I2C bus.
  Wire.begin();
  IMU.init(); //setting MEMS IMU parameters 
  Serial.begin(115200);
  delay(5000);
  
}

void loop() {
  my_acc_t my_memsXLM, my_memsGYRO;
  
  IMU.Get_X_Axes_f(my_memsXLM.float_val);
  IMU.Get_G_Axes_f(my_memsGYRO.float_val);
}
