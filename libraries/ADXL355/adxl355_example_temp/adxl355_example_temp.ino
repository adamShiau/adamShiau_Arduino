#include <Wire.h>


#define ADXL355_ADDR 0x1D	//I2C address
#define I2C_MUXEN  12		//I2C SCL mux

#define I2C_STANDARD_MODE 	100000
#define I2C_FAST_MODE 		400000

#define PRINT_ACC_DATA 1
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
/*** ODR ***/
#define ODR_4000	0b0000
#define ODR_2000 	0b0001
#define ODR_1000 	0b0010
#define ODR_500 	0b0011
#define ODR_250 	0b0100
#define ODR_125 	0b0101
/*** sync parameter ***/
#define INT_SYNC			0x00
#define EXT_SYNC			0x01
#define EXT_SYNC_INTFILTER	0x02

/*** status reg mask***/
#define DATA_RDY_MSK 	0x01
#define FIFO_FULL_MSK 	0x02
#define FIFO_OVR_MSK 	0x04


/*** range reg parameter***/
#define H_MODE		0x80
#define F_MODE		0x00
#define RANGE_2G 	0x01
#define RANGE_4G 	0x02
#define RANGE_8G 	0x03

#define SENS_8G 0.0000156
#define SENS_4G 0.0000078
#define SENS_2G 0.0000039
/*** power control parameter***/
#define MEASURE_MODE	0x00
#define TEMP_OFF_MSK	0x02
#define DRDY_OFF_MSK	0x04


/*** trig pin***/
#define SYS_TRIG 14

/*********************************************/

unsigned int t_old=0, t_new;
bool trig_status[2] = {0, 0};

void setup() {
	Wire.begin();
	Wire.setClock(I2C_FAST_MODE);
	Serial.begin(115200); // arduino monitor
	Serial1.begin(115200);
	pinMode(I2C_MUXEN, OUTPUT);

	SCL_MUX_ENABLE();
	setRegVal(RST_ADDR, 0x52);
	setRegVal(RANGE_ADDR, F_MODE | RANGE_8G);
	setRegVal(FILTER_ADDR, ODR_500);
	setRegVal(SYNC_ADDR, EXT_SYNC_INTFILTER);
	setRegVal(POWER_CTL_ADDR, TEMP_OFF_MSK| MEASURE_MODE);
	SCL_MUX_DISABLE();
	pinMode(SYS_TRIG, INPUT);
}
void loop() {
	int ax, ay, az;
	trig_status[0] = digitalRead(SYS_TRIG);
	// printRegAll();
	if(trig_status[0] & ~trig_status[1]) {
		readData(ax, ay, az);
	}
	trig_status[1] = trig_status[0];
}

void SCL_MUX_ENABLE()
{
	digitalWrite(I2C_MUXEN, 0);
}

void SCL_MUX_DISABLE()
{
	digitalWrite(I2C_MUXEN, 1);
	delayMicroseconds(20);
}

void printRegAll()
{
	printRegVal("DEV_ID", DEVID_AD_ADDR, HEX);
	printRegVal("SYNC", SYNC_ADDR, HEX);
	printRegVal("POWER_CTL", POWER_CTL_ADDR, HEX);
	printRegVal("ODR", FILTER_ADDR, BIN);
	printRegVal("INT MAP", INTERRUPT_ADDR, HEX);
}

void printRegVal(char name[], byte addr, byte rep)
{
	SCL_MUX_ENABLE();
	Serial.print(name);
	Serial.print(": ");
	Serial.println(I2CReadData(addr), rep);
	SCL_MUX_DISABLE();
}

void setRegVal(unsigned char addr, unsigned char val)
{
	I2CWriteData(addr, val);
	delay(100);
}

void readData(int &accX, int &accY, int &accZ) {
	byte temp_ax[3], temp_ay[3], temp_az[3];	
	
	SCL_MUX_ENABLE();
	while(!(I2CReadData(STATUS_ADDR) & DATA_RDY_MSK)) {}; //wait ADXL322 data available
	temp_ax[0] = I2CReadData(XDATA3_ADDR); 
	temp_ax[1] = I2CReadData(XDATA2_ADDR); 
	temp_ax[2] = I2CReadData(XDATA1_ADDR); 
	accX = temp_ax[0]<<12 | temp_ax[1]<<4 | temp_ax[2]>>4;
	if((accX>>19) == 1) accX = accX - (1<<20);

	temp_ay[0] = I2CReadData(YDATA3_ADDR); 
	temp_ay[1] = I2CReadData(YDATA2_ADDR); 
	temp_ay[2] = I2CReadData(YDATA1_ADDR); 
	accY = temp_ay[0]<<12 | temp_ay[1]<<4 | temp_ay[2]>>4;
	if((accY>>19) == 1) accY = accY - (1<<20);

	temp_az[0] = I2CReadData(ZDATA3_ADDR); 
	temp_az[1] = I2CReadData(ZDATA2_ADDR); 
	temp_az[2] = I2CReadData(ZDATA1_ADDR); 
	accZ = temp_az[0]<<12 | temp_az[1]<<4 | temp_az[2]>>4;
	if((accZ>>19) == 1) accZ = accZ - (1<<20);

	SCL_MUX_DISABLE();
	
  if(PRINT_ACC_DATA)
  {
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
  // Serial1.write(temp_ax[0]);
  // Serial1.write(temp_ax[1]);
  // Serial1.write(temp_ax[2]);
  // Serial1.write(temp_ay[0]);
  // Serial1.write(temp_ay[1]);
  // Serial1.write(temp_ay[2]);
  // Serial1.write(temp_az[0]);
  // Serial1.write(temp_az[1]);
  // Serial1.write(temp_az[2]);
}

void I2CWriteData(byte addr, byte val)
{
  Wire.beginTransmission(ADXL355_ADDR);
  Wire.write(addr);
  Wire.write(val);
  Wire.endTransmission();
}

byte I2CReadData(byte addr)
{
	byte data;
	
	Wire.beginTransmission(ADXL355_ADDR);
	Wire.write(addr);//
	Wire.endTransmission();
	Wire.requestFrom(ADXL355_ADDR,1);
	while (Wire.available()) data=Wire.read();
	return data;
}
