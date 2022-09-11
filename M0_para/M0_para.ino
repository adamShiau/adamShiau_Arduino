/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/

// the setup function runs once when you press reset or power the board
#include <Arduino.h>
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(115200);
//  GCLK->GENCTRL.reg = GCLK_GENCTRL_OE |            // Test: enable GCLK output (on a selected pin)
//                      GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
//                      GCLK_GENCTRL_GENEN |         // Enable GCLK0
//                      GCLK_GENCTRL_SRC_XOSC32K |   // Set the external 32.768kHz clock source (XOSC32K)
//                      GCLK_GENCTRL_ID(0);          // Select GCLK0
//  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization  
//  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(24) |         // Divide the 48MHz clock source by divisor 24: 48MHz/24=2MHz
//                   GCLK_GENDIV_ID(0);            // Select Generic Clock (GCLK) 0
  
  // Test: enable the GCLK0 output on D2 (PA14)
//  PORT->Group[g_APinDescription[1].ulPort].PINCFG[g_APinDescription[1].ulPin].bit.PMUXEN = 1;
//  PORT->Group[g_APinDescription[1].ulPort].PMUX[g_APinDescription[1].ulPin >> 1].reg |= PORT_PMUX_PMUXE_H;

  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |
                      GCLK_GENCTRL_GENEN |
//                      GCLK_GENCTRL_SRC_OSC8M |
                      GCLK_GENCTRL_SRC_XOSC32K |
                      GCLK_GENCTRL_ID(0);
               
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN(0) |
                      GCLK_CLKCTRL_ID_USB_Val;

                       
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  Serial.println("");
  Serial.print("GCLK addr: ");
  Serial.println((int)GCLK, HEX);
  Serial.print("CLKCTRL.reg: ");
  Serial.println((int)GCLK->CLKCTRL.reg, HEX);
  Serial.print("GENCTRL.reg: ");
  Serial.println((int)GCLK->GENCTRL.reg, HEX);
  Serial.print("GENDIV.reg: ");
  Serial.println((int)GCLK->GENDIV.reg, HEX);

delay(1000);
}
