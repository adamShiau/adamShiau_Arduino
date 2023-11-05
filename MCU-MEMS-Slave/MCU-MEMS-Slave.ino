#include <SercomSPISlave.h>

Sercom1SPISlave SPISlave; // to use a different SERCOM, change this line and find and replace all SERCOM1 with the SERCOM of your choice

// #define DEBUG // uncomment this line to print debug data to the serial bus
#define INTERRUPT2BUFFER // uncomment this line to copy the data received in the Data Received Complete interrupt to a buffer to be used in the main loop
//#define INTERRUPT2SERIAL // uncomment this line to print the data to the serial bus whenever the Data Received Complete interrupt is triggered

#define SPI_SLAVE_DATA_SIZE 16

// initialize variables
byte buf[100]; // initialize a buffer of 1 byte

volatile int att=0;
int cnt=0;


const uint8_t data_size_expected = 28, header_size = 4;
uint8_t bytes_received = 0;
uint8_t imu_buffer[data_size_expected]={0}, imu_buffer_out[data_size_expected] = {0};
uint8_t expected_header[] = {0xFE, 0x81, 0xFF, 0x55};
// volatile bool  payload_complete = 0;
bool rcv_complete, payload_complete = 0;
volatile uint8_t bytes_output=0;
unsigned long t_pre = 0;

enum {
  EXPECTING_HEADER, 
  EXPECTING_PAYLOAD,
  EXPECTING_WRITE
	} FSM = EXPECTING_HEADER;

typedef union
{
  float float_val;
  uint8_t bin_val[4];
  uint32_t ulong_val;
} my_time_t;


typedef union
{
  float float_val[3];
  uint8_t bin_val[12];
  unsigned long ulong_val[3];
}my_att_t;



typedef struct 
{
  my_att_t my_attitude;
  my_time_t my_time;
}att_struc_t;

att_struc_t my_att_data;

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial started");
  SPISlave.SercomInit(SPISlave.MOSI_Pins::PA16, SPISlave.SCK_Pins::PA17, SPISlave.SS_Pins::PA18, SPISlave.MISO_Pins::PA19);
  Serial.println("SERCOM1 SPI slave initialized");
  my_att_data.my_attitude.float_val[0] = 123.45;
  my_att_data.my_attitude.float_val[1] = 130.56;
  my_att_data.my_attitude.float_val[2] = 20.89;
  my_att_data.my_time.ulong_val = 0;
  t_pre = millis();
}

void loop()
{
  //obtain latest imu data, imu_buffer_out, when rcv_complete == 1
  if(rcv_complete){
    rcv_complete = 0;
    getImuData(imu_buffer_out);
  }
  delay(100); //assume the attitude algorithm takes 100ms to compute
  update_attitude_data_here();
}

void update_attitude_data_here()
{
  my_att_data.my_time.ulong_val = millis() - t_pre;
}

void getImuData(uint8_t *data)
{
  my_att_t my_memsXLM, my_memsGYRO;
  my_time_t my_time;

  memcpy(&my_memsGYRO, data, sizeof(my_memsGYRO));
  memcpy(&my_memsXLM, data + 12, sizeof(my_memsXLM));
  memcpy(&my_time, data + 24, sizeof(my_time));

  for(int i=0; i<3; i++){
    Serial.print(my_memsGYRO.float_val[i]);
    Serial.print(", ");
  }
  for(int i=0; i<3; i++){
    Serial.print(my_memsXLM.float_val[i]);
    Serial.print(", ");
  }
  Serial.println(my_time.ulong_val);
}

void SERCOM1_Handler()
{
  uint8_t data = 0;

  uint8_t interrupts = SERCOM1->SPI.INTFLAG.reg; // Read SPI interrupt register

  // Data Received Complete interrupt
  if (interrupts & (1 << 2)) // 0100 = bit 2 = RXC
  {
    // read DATA register as RXC = 1
    data = (uint8_t)SERCOM1->SPI.DATA.reg;

    switch(FSM)
    {
      case EXPECTING_HEADER:{

        if (data != expected_header[bytes_received++])
        {
          FSM = EXPECTING_HEADER;
          bytes_received = 0;
        }

        if(bytes_received >= header_size)
        {
          FSM = EXPECTING_PAYLOAD;
          bytes_received = 0;
        }
        break;
      }

      case EXPECTING_PAYLOAD:{

        imu_buffer[bytes_received++] = data;

        if(bytes_received >= data_size_expected)
        {
          bytes_received = 0;
          FSM = EXPECTING_HEADER;
          for (int i = 0; i < data_size_expected; i++) imu_buffer_out[i] = imu_buffer[i];
          /***try*****
             * memcpy(imu_buffer_out, imu_buffer, sizeof(imu_buffer));
            */
          // Indicate the latest imu data is complete that can be used to compute.
          rcv_complete = 1;
          // Indicate the FSM: EXPECTING_PAYLOAD is complete and can output Slave data to Master.
          payload_complete = 1;
        }
        break;
      }
      default:{
        break;
      }
    } 
    SERCOM1->SPI.INTFLAG.bit.RXC = 1; // Clear Receive Complete interrupt
  }

  // Data Register Empty interrupt
  if (interrupts & (1 << 0)) // 0001 = bit 0 = DRE
  {
    if(payload_complete){
      //output data according to the defined data structure
      if(bytes_output < 12) SERCOM1->SPI.DATA.reg = my_att_data.my_attitude.bin_val[bytes_output++];
      else SERCOM1->SPI.DATA.reg = my_att_data.my_time.bin_val[bytes_output++ -12];
      if(bytes_output >=SPI_SLAVE_DATA_SIZE) {
        payload_complete = 0;
        bytes_output = 0;
      }
    }
    else{
      SERCOM1->SPI.DATA.reg = 99;
    }
    
  }

    // Data Transmit Complete interrupt
  if (interrupts & (1 << 1)) // 0010 = bit 1 = TXC // page 503
  {
    SERCOM1->SPI.INTFLAG.bit.TXC = 1; // Clear Transmit Complete interrupt
  }

      // Slave Select Low interrupt
  if (interrupts & (1 << 3)) // 1000 = bit 3 = SSL // page 503
  {
    SERCOM1->SPI.INTFLAG.bit.SSL = 1; // Clear Slave Select Low interrupt
  }
}


