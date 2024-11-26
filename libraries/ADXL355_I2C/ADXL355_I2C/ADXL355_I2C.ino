#include <Wire.h>

#include "adxl357_I2C.h"

#define I2C_STANDARD_MODE   100000
#define I2C_FAST_MODE       400000
#define I2C_FAST_MODE_PLUS  1000000


#define ADXL357_ADDR 		0x1E

Adxl357_I2C adxl357_i2c(Wire);

typedef union
{
  float float_val[3];
  uint8_t bin_val[12];
  int int_val[3];
}my_acc_t;

my_acc_t my_ADXL357;

void setup() {
  // put your setup code here, to run once:
  delay(3000);
  Wire.begin();        // join i2c bus (address optional for master)
  Wire.setClock(400000);
  adxl357_i2c.init();
  Serial.begin(115200);  // start serial for output

  // Wire.beginTransmission(ADXL357_ADDR);
  // Wire.write(0x00);
  // Wire.endTransmission();

  // Wire.requestFrom(ADXL357_ADDR,1);
}

void loop() {
  // put your main code here, to run repeatedly:
  adxl357_i2c.readData_f(my_ADXL357.float_val);
  Serial.print(my_ADXL357.float_val[2]);
  Serial.print(", ");
  Serial.println(millis());

}
