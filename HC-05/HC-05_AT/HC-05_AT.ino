/*** 
use Arduino Nano33 iot for configuration,
Serial1 (D0, D1) connect to HC-05  
arduino-Tx -> HC-05-RX
arduino-Rx -> HC-05-TX
***/
/***command 
1. 藍芽裝置配對顯示名稱
AT+NAME?
AT+NAME="BT_NAME"
配對密碼:1234

2. UART相關
AT+UART?
AT+UART=115200,0,0

3. 恢復出場設定
AT+ORGL

/***/
char val;

void setup() {
  Serial1.begin(38400);
  Serial.begin(9600);
}

void loop() {
	// transmit command to HC-05
  if(Serial.available()) {
    val = Serial.read();
    Serial1.print(val);
  }
	// receive respons from HC-05
  if(Serial1.available()) {
//    Serial.println(Serial1.available());
    val = Serial1.read();
    Serial.print(val);
  }
}
