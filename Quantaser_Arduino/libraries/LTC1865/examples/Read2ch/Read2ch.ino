#include <LTC1865.h>
#include <SPI.h>

#define LTC1865CONV 7

LTC1865 ltc1865;

void setup() {
  // put your setup code here, to run once:
  ltc1865.init(LTC1865CONV,0);// ch0 is the first channel to read)

}

void loop() {
  // put your main code here, to run repeatedly:
 unsigned int data0, data1;
 data0 = ltc1865.Read(1); // read the ch0 data and next channel is ch1
 data1 = ltc1865.Read(0); // read the ch1 data and next chanel is ch0
}
