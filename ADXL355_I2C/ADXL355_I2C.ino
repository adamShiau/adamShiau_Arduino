#include "adxl355.h"
#include <Arduino.h>
#include "wiring_private.h"


#define SENS_8G 0.0000156
#define SENS_4G 0.0000078
#define SENS_2G 0.0000039

/*** trig pin***/
#define SYS_TRIG 14
/*** register address***/
#define DEVID_AD_ADDR	0x00
#define STATUS_ADDR 	0x04
#define FIFO_ADDR 		0x05
#define TEMP2_ADDR 		0x06
#define TEMP1_ADDR  	0x07
#define XDATA3_ADDR  	0x08
#define XDATA2_ADDR  	0x09
#define XDATA1_ADDR  	0x0A
#define YDATA3_ADDR  	0x0B
#define YDATA2_ADDR  	0x0C
#define YDATA1_ADDR  	0x0D
#define ZDATA3_ADDR  	0x0E
#define ZDATA2_ADDR  	0x0F
#define ZDATA1_ADDR  	0x10
#define FILTER_ADDR  	0x28
#define INTERRUPT_ADDR 	0x2A
#define SYNC_ADDR  		0x2B
#define RANGE_ADDR  	0x2C
#define POWER_CTL_ADDR 	0x2D
#define RST_ADDR  		0x2F

/*** range reg parameter***/
#define H_MODE		0x80
#define F_MODE		0x00
#define RANGE_2G 	0x01
#define RANGE_4G 	0x02
#define RANGE_8G 	0x03

#define SENS_8G 0.0000156
#define SENS_4G 0.0000078
#define SENS_2G 0.0000039
/*** reset parameter ***/
#define POR 		0x52
/*** ODR parameter ***/
#define ODR_4000	0b0000
#define ODR_2000 	0b0001
#define ODR_1000 	0b0010
#define ODR_500 	0b0011
#define ODR_250 	0b0100
#define ODR_125 	0b0101

/*** sync parameter ***/
#define INT_SYNC	0x00
#define EXT_SYNC	0x02

/*** status reg mask***/
#define DATA_RDY_MSK 	0x01
#define FIFO_FULL_MSK 	0x02
#define FIFO_OVR_MSK 	0x04

/*** power control parameter***/
#define MEASURE_MODE	0x00 
#define TEMP_OFF_MSK	0x02


int pin_scl_mux = 12;
bool trig_status[2] = {0, 0};
unsigned int t_new, t_old=0;


// I2C
#include <Wire.h>
#define ADXL355_ADDR     0x1D  //Adxl355 I2C address
#define I2C_STANDARD_MODE   100000
#define I2C_FAST_MODE     400000
#define I2C_FAST_MODE_PLUS     1000000
//#define I2C_HIGH_SPEED_MODE    3400000 //can not work
#define TEST_ADDR      0xAB
/*** TwoWire Wire(&sercom, PIN_WIRE_SDA, PIN_WIRE_SCL);***/
TwoWire myWire(&sercom0, 27, 20);
void SERCOM0_Handler()
{
  myWire.onService();
}

Adxl355 adxl355(myWire);

void setup() {
  myWire.begin();
  myWire.setClock(I2C_FAST_MODE_PLUS);
  pinPeripheral(27, PIO_SERCOM);
  pinPeripheral(20, PIO_SERCOM);
	adxl355.init();
//	adxl355.setRegVal(0x2B, 0); 
	pinMode(SYS_TRIG, INPUT);
}

void loop() {
	byte acc[9];
	
	// trig_status[0] = digitalRead(SYS_TRIG);
	// if(trig_status[0] & ~trig_status[1]) {
//		 adxl355.printRegAll();
		// adxl355.printRegVal("", STATUS_ADDR, HEX);
//   adxl355.testI2C();
		adxl355.readData(acc);
		print_adxl355Data(acc);
	// }
	// trig_status[1] = trig_status[0];
//	 delay(500);
}

void print_adxl355Data(byte *temp_a)
{
	int accX, accY, accZ;
	
	accX = temp_a[0]<<12 | temp_a[1]<<4 | temp_a[2]>>4;
	if((accX>>19) == 1) accX = accX - (1<<20);
	accY = temp_a[3]<<12 | temp_a[4]<<4 | temp_a[5]>>4;
	if((accY>>19) == 1) accY = accY - (1<<20);
	accZ = temp_a[6]<<12 | temp_a[7]<<4 | temp_a[8]>>4;
	if((accZ>>19) == 1) accZ = accZ - (1<<20);
	
	t_new = micros();
	Serial.print(t_new - t_old);
	Serial.print('\t');
	Serial.print((float)accX*SENS_8G);
	Serial.print('\t');
	Serial.print((float)accY*SENS_8G);
	Serial.print('\t');
	Serial.println((float)accZ*SENS_8G);
	t_old = t_new;
}
