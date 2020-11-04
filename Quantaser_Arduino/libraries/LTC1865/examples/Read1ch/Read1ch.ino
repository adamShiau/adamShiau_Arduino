#include <LTC1865.h>
#include <SPI.h>

#define LTC1865CONV 7

LTC1865 ltc1865(LTC1865CONV);

void setup() {
  // put your setup code here, to run once:
  ltc1865.init(0);// ch0 is the first channel to read)

}

void loop() {
  // put your main code here, to run repeatedly:
 unsigned int data0;
 data0 = ltc1865.Read(0); // read the ch0 data and next channel is ch1
}
