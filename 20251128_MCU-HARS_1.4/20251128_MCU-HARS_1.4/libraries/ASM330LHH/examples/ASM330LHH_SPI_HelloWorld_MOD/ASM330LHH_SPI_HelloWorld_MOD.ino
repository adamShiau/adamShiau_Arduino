/**
 ******************************************************************************
 * @file    ASM330LHH_SPI_HelloWorld.ino
 * @author  SRA
 * @version V1.0.0
 * @date    March 2020
 * @brief   Arduino test application for the STMicrolectronics STEVAL-MKI193V1
 *          adapter board via SPI.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
// In order to test this sketch you need to plug a STEVAL-MKI193V1 in the DIL24 adapter of the X-NUCLEO-IKS01A3
// In order to configure the X-NUCLEO-IKS01A3 DIL24 adapter in SPI mode you need to unsolder SB5, SB12, SB19 and SB23 and solder SB6, SB10, SB18 and SB22

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
  Serial.println("1");
  IMU.begin();
  Serial.println("2");
  IMU.Enable_X();
  IMU.Enable_G();
  IMU.Set_X_ODR(416.0);
  IMU.Set_X_FS(4);  
  IMU.Set_G_ODR(416.0);
  IMU.Set_G_FS(250); 
}

void loop() {
  // byte nano33_w[6]={};
  // byte  nano33_a[6]={};
  int ax, ay, az;

  float ODR, SEN;
  long FS;
  Serial.println("hi");
  // IMU.readGyroscope(nano33_w);
  // IMU.readAcceleration(nano33_a);
 
  IMU.Get_X_ODR(&ODR);
  IMU.Get_X_FS(&FS);
  IMU.Get_X_Sensitivity(&SEN);
  Serial.print(ODR);
  Serial.print(" ");
  Serial.print(FS);
  Serial.print(" ");
  Serial.println(SEN, 6);
  // ax = (nano33_a[0]<<8 | nano33_a[1]);
  // ay = (nano33_a[2]<<8 | nano33_a[3]);
  // az = (nano33_a[4]<<8 | nano33_a[5]);
  // Serial.print(ax);
  // Serial.print(", ");
  // Serial.print(ay);
  // Serial.print(", ");
  // Serial.println(az);
    // Serial.print(nano33_a[0],HEX);
    // Serial.print(", ");
    // Serial.print(nano33_a[1],HEX);
    // Serial.print(", ");
    // Serial.print(nano33_a[2],HEX);
    // Serial.print(", ");
    // Serial.print(nano33_a[3],HEX);
    // Serial.print(", ");
    // Serial.print(nano33_a[4],HEX);
    // Serial.print(", ");
    // Serial.print(nano33_a[5],HEX);
    // Serial.print(", ");
    // Serial.print(nano33_a[6],HEX);
    // Serial.print(", ");
    // Serial.print(nano33_a[7],HEX);
    // Serial.print(", ");
    // Serial.println(nano33_a[8],HEX);

  delay(10);

  /***Read accelerometer and gyroscope.***/
  int32_t accelerometer[3] = {};
  int32_t gyroscope[3] = {};
  IMU.Get_X_Axes(accelerometer);
  IMU.Get_G_Axes(gyroscope);

  /**
   * @brief Output data.
   * 
   */
  SerialPort.print("ASM330LHH: | Acc[mg]: ");
  SerialPort.print(accelerometer[0]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer[1]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer[2]);
  SerialPort.print(" | Gyr[mdps]: ");
  SerialPort.print(gyroscope[0]);
  SerialPort.print(" ");
  SerialPort.print(gyroscope[1]);
  SerialPort.print(" ");
  SerialPort.print(gyroscope[2]);
  SerialPort.println(" |");

}
