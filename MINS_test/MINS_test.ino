#include "wiring_private.h"
#include "myUART.h"
// #include <Arduino.h>
#define LED 16

bool OUT = 1;


void setup() {
  myUART_init();
  pinMode(LED, OUTPUT);
}


void loop() { 
  // printNMEA(Serial3, Serial1, true);
  // printNMEA(Serial4, Serial1, true);
  Serial.println(123);
  Serial1.println(123);
  Serial2.println(123);
  Serial3.println(123);
  Serial4.println(123);
  digitalWrite(LED, HIGH);
  // print_serial1_rx();
  // print_serial2_rx();
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);

}

void print_serial1_rx(void) {
  if (Serial1.available()) {
    char receivedChar = Serial1.read();

    Serial.print("Received from Serial1: ");
    Serial.println(receivedChar);
  }
}

void print_serial4_rx(void) {
  if (Serial4.available()) {
    char receivedChar = Serial4.read();

    Serial.print("Received from Serial4: ");
    Serial.println(receivedChar);
  }
}

void print_serial2_rx(void) {
  if (Serial2.available()) {
    char receivedChar = Serial2.read();

    Serial.print("Received from Serial2: ");
    Serial.println(receivedChar);
  }
}


int readNMEA(HardwareSerial &serial_nmea, char *receivedChars, int buffer_size) {
  int index = 0;
  while (true) {
    while (serial_nmea.available()) {
      char incomingChar = serial_nmea.read();  // 讀取一個字元
      if (incomingChar == '$') {  // 遇到起始符號時，清空字串
        index = 0;
      }

      if (incomingChar == '\n' && index > 0) {  // 遇到換行符號時，結束字串處理
        if (receivedChars[index - 1] == '\r') {
          index -= 1;
        } 
        receivedChars[index] = '\0';
        return index;
      } 
      
      else {
          if (index < buffer_size - 1) {
              receivedChars[index++] = incomingChar;  // 存入字串
          }
          else{
            Serial.println("NMEA buffer overflow");
            return 0;
          }
      }
    }
  }
}



void printNMEA(HardwareSerial &serial_nmea, HardwareSerial &serial_output, bool debug){
  char buffer[256];
  int bytesRead = readNMEA(serial_nmea, buffer, 256);
  if (bytesRead){
    serial_output.println(buffer);
    if (debug) Serial.println(buffer);
  }
}