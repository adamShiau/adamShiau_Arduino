#include <TinyGPSPlus.h>
#include "wiring_private.h"
// #include <SoftwareSerial.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
// static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

Uart mySerial13 (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);
void SERCOM1_Handler()
{
    mySerial13.IrqHandler();
}

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
// SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600);
  // ss.begin(GPSBaud);
  mySerial13.begin(9600);//rx:p13, tx:p8
	// Reassign pins 13 and 8 to SERCOM (not alt this time)
	pinPeripheral(13, PIO_SERCOM);
	pinPeripheral(8, PIO_SERCOM);

  Serial.println("DeviceExample.ino");
  Serial.println("A simple demonstration of TinyGPSPlus with an attached GPS module");
  Serial.print("Testing TinyGPSPlus library v. "); 
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println("by Mikal Hart");
  Serial.println();
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  // while (Serial1.available() > 0)
  // {
  //   // Serial.println(ss.available());
  //   if (gps.encode(Serial1.read()))
  //     displayInfo();
  // }
    
  displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
  smartDelay(1000);

}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (mySerial13.available())
      gps.encode(mySerial13.read());

  } while (millis() - start < ms);
}


void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}