#include <Arduino.h>
#include "wiring_private.h"

//SERCOM2: serial2 (PA14, PA15) [tx : D4,  rx: D5]
Uart Serial2 (&sercom2, 25, 24, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM2_Handler()
{
    Serial2.IrqHandler();
}

//SERCOM1: serial3 (PA16, PA17) [tx : D11, rx: D13]
Uart Serial3 (&sercom1, 13, 11, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM1_Handler()
{
    Serial3.IrqHandler();
}

//SERCOM3: serial4 (PA20, PA21) [tx : D6, rx: D7]
Uart Serial4 (&sercom3, 10, 9, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM3_Handler()
{
  Serial4.IrqHandler();
}

int cnt=0;

void setup() {
  // put your setup code here, to run once:
Serial1.begin(921600);
Serial2.begin(921600); 
Serial3.begin(921600);
Serial4.begin(921600);

pinPeripheral(24, PIO_SERCOM);
pinPeripheral(25, PIO_SERCOM);

pinPeripheral(11, PIO_SERCOM);
pinPeripheral(13, PIO_SERCOM);

pinPeripheral(10, PIO_SERCOM_ALT);
pinPeripheral(9, PIO_SERCOM_ALT);
}

void loop() {
  // put your main code here, to run repeatedly:
//send_serial_data(Serial4    , cnt);
loopBack(Serial2);
loopBack(Serial3);
loopBack(Serial4);
cnt++;
//delay(100);
}

void send_serial_data(Stream &ser, int cnt)
{
  ser.write(0xAB);
  ser.write(0xBA);
  ser.write(0xA0);
  ser.write(0xA1);
  ser.write(0xA2);
  ser.write(0xA3);
  ser.write(0xA4);
  ser.write(0xA5);
  ser.write(0xA6);
  ser.write(0xA7);
  ser.write(0xA8);
  ser.write(0xA9);
  Serial.println();
}

void loopBack(Stream &ser)
{
  byte data[12];
  if (ser.available()>=12)
  {
    Serial.println(ser.available());
    ser.readBytes(data, 12);
//    ser.write(data, 12);
    Serial1.write(data, 12);
  }
  
}
