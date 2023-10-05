

// Includes
#include <Arduino.h>
#include "wiring_private.h"
#include <ASM330LHHSensor.h>

#define SerialPort Serial

// SPI
#include <SPI.h>
#define SPI_CLOCK_8M 8000000
#define SPI_CLOCK_1M 1000000
/*** SPIClass SPI (sercom, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_MOSI, PAD_SPI_TX, PAD_SPI_RX);***/
// SPIClass mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
SPIClassSAMD mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);


#define CHIP_SELECT_PIN 2
// ASM330LHHClass IMU(mySPI, CHIP_SELECT_PIN, SPI_CLOCK_8M);
ASM330LHHSensor IMU(&mySPI, CHIP_SELECT_PIN);


unsigned int t_new, t_old=0;

void setup() {
  // Led.
  // pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);
  delay(2000);
  //SPI
  mySPI.begin();
  pinPeripheral(3, PIO_SERCOM_ALT);
  pinPeripheral(22, PIO_SERCOM_ALT);
  pinPeripheral(23, PIO_SERCOM_ALT);
	
	// if (!IMU.begin()) {
  //   Serial.println("Failed to initialize IMU!");
  //   while (1);
  // }
  IMU.begin();
  IMU.Enable_X();
  IMU.Enable_G();
  IMU.Set_X_ODR(416.0);
  IMU.Set_X_FS(4);  
  IMU.Set_G_ODR(416.0);
  IMU.Set_G_FS(250); 
}

void loop() {
  
  byte nano33_w[6]={0,0,0,0,0,0};
  byte  nano33_a[6]={0,0,0,0,0,0};;

  IMU.Get_X_AxesRaw(nano33_a);
  IMU.Get_G_AxesRaw(nano33_w);

  Serial.print(nano33_a[0], HEX);
  Serial.print(nano33_a[1], HEX);
  Serial.println();

  delay(10);

 

}
