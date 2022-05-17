#include "wiring_private.h"
#include "SparrowParaDefine.h"

#define UART_SERIAL_5

// #define readSPI_cmd  String("readSPI ")
// #define readTemp_cmd  "readTemp "
// #define ANGULAR_V  String("17 ")
// #define OPEN_V  "12  "

// #define Read_cmd  "1"
// #define Stop_cmd  "0"

#ifdef UART_SERIAL_5
Uart mySerial5 (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM0_Handler() 
{
	mySerial5.IrqHandler();
}
#endif

int cnt = 0;

void setup() {
  #ifdef UART_SERIAL_5
		mySerial5.begin(115200); //to Quantaser Sparrow
		// Reassign pins 5 and 6 to SERCOM alt
		pinPeripheral(5, PIO_SERCOM_ALT); //RX
		pinPeripheral(6, PIO_SERCOM_ALT); //TX
	#endif
  Serial.begin(230400);
  Serial1.begin(115200); // to PC
  startRead(1);

}

unsigned char data[6];

void loop() {
  if(cnt < 100){

    mySerial5.readBytes(data, 6);
    Serial1.write(data, 6);
    cnt++;
    delay(50);
  }
  if(cnt==100) setStop();
  // startRead(1);
  // delay(50);

}


void startRead(byte ch)
{
  String cmd;

  cmd = readSPI_cmd + ANGULAR_V + String(ch);
  mySerial5.println(cmd);
}

void setStop()
{
  String cmd;

  cmd =  readSPI_cmd + ANGULAR_V + Stop_cmd;
  mySerial5.println(cmd);
}

void setOutputMode()
{
  String cmd;

  cmd = setSPI_cmd + OUTPUT_ADD + CLOSE_MODE;
  mySerial5.println(cmd);

}


	// gyro.setModHigh()
	// gyro.setModLow()
	// gyro.setModFreq()
	// gyro.setPiVth()
	// gyro.setPolarity()
	// gyro.setIgnor()
	// gyro.setOffset()
	// gyro.setStepVth()
	// gyro.setAVG()
	// gyro.setGain1()
	// gyro.setGain2()


