#include <SercomSPISlave.h>

Sercom1SPISlave SPISlave; // to use a different SERCOM, change this line and find and replace all SERCOM1 with the SERCOM of your choice

// #define DEBUG // uncomment this line to print debug data to the serial bus
#define INTERRUPT2BUFFER // uncomment this line to copy the data received in the Data Received Complete interrupt to a buffer to be used in the main loop
//#define INTERRUPT2SERIAL // uncomment this line to print the data to the serial bus whenever the Data Received Complete interrupt is triggered

// initialize variables
byte buf[100]; // initialize a buffer of 1 byte

int att=0;
int cnt=0;


const uint8_t data_size_expected = 32, header_size = 4;
uint8_t bytes_received = 0;
uint8_t buffer[64]={0};
uint8_t expected_header[] = {0xFE, 0x81, 0xFF, 0x55};
bool rcv_complete = 0;

enum {
  EXPECTING_HEADER, 
  EXPECTING_PAYLOAD,
  EXPECTING_WRITE
	} state = EXPECTING_HEADER;

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial started");
  SPISlave.SercomInit(SPISlave.MOSI_Pins::PA16, SPISlave.SCK_Pins::PA17, SPISlave.SS_Pins::PA18, SPISlave.MISO_Pins::PA19);
  Serial.println("SERCOM1 SPI slave initialized");
}

void loop()
{
  att += 1;
  delay(100);
  if(cnt==99) {
    for(int i=0; i<100; i++){
      Serial.print(i);
      Serial.print(", ");
      Serial.println(buf[i], HEX);
    }
  }
}

void SERCOM1_Handler()
{
  uint8_t data = 0;

  #ifdef DEBUG
    Serial.println("In SPI Interrupt");
  #endif
  uint8_t interrupts = SERCOM1->SPI.INTFLAG.reg; // Read SPI interrupt register
  Serial.print(interrupts, BIN);

  // Data Register Empty interrupt
  if (interrupts & (1 << 0)) // 0001 = bit 0 = DRE // page 503
  {
    #ifdef DEBUG
      Serial.println("SPI Data Register Empty interrupt");
    #endif
    SERCOM1->SPI.DATA.reg = 0;
  }

      // Slave Select Low interrupt
  if (interrupts & (1 << 3)) // 1000 = bit 3 = SSL // page 503
  {
    #ifdef DEBUG
      Serial.println("SPI Slave Select Low interupt");
    #endif
    SERCOM1->SPI.INTFLAG.bit.SSL = 1; // Clear Slave Select Low interrupt
  }

  // Data Received Complete interrupt: this is where the data is received, which is used in the main loop
  if (interrupts & (1 << 2)) // 0100 = bit 2 = RXC // page 503
  {
    #ifdef DEBUG
      Serial.println("SPI Data Received Complete interrupt");
    #endif
    data = (uint8_t)SERCOM1->SPI.DATA.reg;
    SERCOM1->SPI.INTFLAG.bit.RXC = 1; // Clear Receive Complete interrupt
  }
  Serial.print(",");
  Serial.println(SERCOM1->SPI.INTFLAG.reg, BIN);

}

void SERCOM11_Handler()
/*
Reference: Atmel-42181G-SAM-D21_Datasheet section 26.8.6 on page 503
*/
{
  #ifdef DEBUG
    Serial.println("In SPI Interrupt");
  #endif
  uint8_t data = 0;
  // data = (uint8_t)SERCOM1->SPI.DATA.reg;
  uint8_t interrupts = SERCOM1->SPI.INTFLAG.reg; // Read SPI interrupt register
  #ifdef DEBUG
    Serial.print("Interrupt: "); Serial.println(interrupts);
  #endif
  

    // Slave Select Low interrupt
  if (interrupts & (1 << 3)) // 1000 = bit 3 = SSL // page 503
  {
    #ifdef DEBUG
      Serial.println("SPI Slave Select Low interupt");
    #endif
    SERCOM1->SPI.INTFLAG.bit.SSL = 1; // Clear Slave Select Low interrupt
  }

  // /***
  // Data Received Complete interrupt: this is where the data is received, which is used in the main loop
  if (interrupts & (1 << 2)) // 0100 = bit 2 = RXC // page 503
  {
    // noInterrupts();
    #ifdef DEBUG
      Serial.println("SPI Data Received Complete interrupt");
    #endif
    // data = SERCOM1->SPI.DATA.reg; // Read data register
    data = (uint8_t)SERCOM1->SPI.DATA.reg;
    SERCOM1->SPI.INTFLAG.bit.RXC = 1; // Clear Receive Complete interrupt
    buf[cnt++] = data;
    if(cnt==100) cnt=0;
    // Serial.println(data);
    // interrupts();
    
// /***
// switch(state) 
//     {
//       case EXPECTING_HEADER:{

//         Serial.print("EXPECTING_HEADER: ");
//         Serial.print(state);
//         Serial.print(", ");
//         Serial.println(data);

//         rcv_complete = 0;
//         if (data != expected_header[bytes_received++])
//         {
//           state = EXPECTING_HEADER;
//           bytes_received = 0;

//         }

//         if(bytes_received >= header_size)
//         {
//           Serial.println("EXPECTING_PAYLOAD: ");
//           state = EXPECTING_PAYLOAD;
//           bytes_received = 0;
//         }
//         break;
//       }
      
//       case EXPECTING_PAYLOAD:{ 

//         buffer[bytes_received++] = data;

//         if(bytes_received >= data_size_expected)
//         {
//           rcv_complete = 1;
//           bytes_received = 0;
//           state = EXPECTING_WRITE;
//         }
//         break;
//       }
//     }

    

  }
  
  // Data Transmit Complete interrupt
  if (interrupts & (1 << 1)) // 0010 = bit 1 = TXC // page 503
  {
    #ifdef DEBUG
      Serial.println("SPI Data Transmit Complete interrupt");
    #endif
    SERCOM1->SPI.INTFLAG.bit.TXC = 1; // Clear Transmit Complete interrupt
  }
  
  // Data Register Empty interrupt
  if (interrupts & (1 << 0)) // 0001 = bit 0 = DRE // page 503
  {
    #ifdef DEBUG
      Serial.println("SPI Data Register Empty interrupt");
    #endif
    SERCOM1->SPI.DATA.reg = att;
  }
  
  #ifdef INTERRUPT2BUFFER
    // Write data to buffer, to be used in main loop
    // buf[cnt++] = data;
    // if(cnt==100) cnt=0;
  #endif
  #ifdef INTERRUPT2SERIAL
    // Print data received during the Data Receive Complete interrupt
    char _data = data;
    Serial.print("DATA: ");
    Serial.println(_data); // Print received data
  #endif

}
