#include <C12880.h>

C12880 spectro;
File myFile;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  spectro.SpectroInit();
#if 0   //sample code
  spectro.PulseClkAB(3);
  spectro.StartIntegAB();
  spectro.PulseClkAB(10000);
  spectro.StopIntegA();
  spectro.StopIntegB();
  spectro.PulseClkAB(87);
  spectro.ReadVedioAB(data);
#else
  spectro.RunDevice(10000,10000,NoPrint,myFile);
#endif
  //=======
  delay(10);
  spectro.RunDevice(20000,30000,WriteSerial,myFile);
#if DEBUG_MODE
  spectro.PrintData();
#endif
  //=======
  delay(10);

  if (!SD.begin(SD_CSPIN)) {
    Serial.println("initialization failed!");
    return;
  }
  myFile = SD.open("test.txt", FILE_WRITE);

  if (myFile) {
    spectro.RunDevice(30000,20000,WriteSD,myFile);
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  // close the file:
  myFile.close();

}

void loop() {
  // put your main code here, to run repeatedly:

}


