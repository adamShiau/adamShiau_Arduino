// #incqlude "Arduino_adxl355.h"
#include "adxl355.h"

char pin_mux_sclen = 12;

adxl355 adxl355(12);

void setup() {
	adxl355.init();
}
void loop() {
	// adxl355.printRegAll();
	delay(100);
}