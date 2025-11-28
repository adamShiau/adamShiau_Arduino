#ifndef MYMESSAGE
#define MYMESSAGE
#include <Arduino.h>


class MyCRC {
private:
    uint32_t WIDTH;
    uint32_t POLYNOMIAL;
    uint32_t REMAINDER;
    uint32_t crcFailCnt;
    int NUM_BYTE;

public:
    MyCRC(uint32_t width = 32, uint32_t polynomial = 0x04C11DB7, uint32_t initial_remainder = 0xFFFFFFFF);
    void calCRC(uint8_t *messagem, int num_byte);
    bool isCRCPass(uint8_t *message, int num_msg);
  };


typedef enum{
  INITIALIZING,
  IMU_MEASURING,
  CONFIGURING,
} SYSTEM_STATE;

typedef enum{
  OUT_MODE_MAVLINK, 
  OUT_MODE_ML_ODOM, 
  OUT_MODE_ML_VISO, 
  OUT_MODE_ML_GPS_RAW,
  OUT_MODE_ML_GPS_IN,
  OUT_MODE_NMEA,
  OUT_MODE_BIN,
  OUT_MODE_XBUS,
  OUT_MODE_STR,
  OUT_MODE_CONFIG,
  OUT_MODE_GNSS,
  OUT_MODE_VEC,
  OUT_MODE_NONE
} OutputMode;

const uint8_t HEADER[2] = {0xFA, 0xFF};
const byte HEADER_SIZE = 2;

uint8_t cal_xor_checksum(uint8_t *msg, int num_msg);
String cal_xor_checksum(String nmea_string);
bool xor_checksum(uint8_t *msg, int num_msg, uint8_t checksum);

void convert2Sign_8B(double *value, uint8_t *buf);
void convert2Sign_4B(uint32_t* value, uint8_t* buf);
void convert2Sign_2B(uint16_t* value, uint8_t* buf);
char *my_dtostrf(double val, signed char width, unsigned char prec, char *sout);
int appendValue2Str(char *buffer, int bufSize, int curIndex, float value, int decimalPlaces);
int appendValues2Str(char *buffer, int bufSize, int curIndex, const float *values, int numValues, int decimalPlaces);
int appendValue2Str(char *buffer, int bufSize, int curIndex, double value, int decimalPlaces);
int appendValues2Str(char *buffer, int bufSize, int curIndex, const double *values, int numValues, int decimalPlaces);
void LEDblink();
void blinkLED(SYSTEM_STATE sys_state);
void checkSTRCommand(OutputMode &output_mode, Stream &port=Serial);
String formatLatitude(float lat);
String formatLongitude(float lon);
String readCurrentBytes(Stream &port, uint16_t timeout_ms=10);
void write_big_endian(uint8_t *dst, uint8_t *src, size_t len);

typedef union{
  float float_val[3];
  uint8_t bin_val[12];
  unsigned long ulong_val[3];
} my_data_3f;

typedef union{
  float float_val[4];
  uint8_t bin_val[16];
  unsigned long ulong_val[4];
} my_data_4f;

typedef union{
  float float_val[2];
  uint8_t bin_val[8];
  unsigned long ulong_val[2];
} my_data_2f;

typedef union{
  double float_val[2];
  uint8_t bin_val[16];
  uint64_t ulong_val[2];
} my_data_2d;

typedef union{
  float float_val;
  uint8_t bin_val[4];
  unsigned long ulong_val;
} my_data_u4;

typedef union{
  double float_val;
  uint8_t bin_val[8];
  uint64_t ulong_val;
} my_data_u8;

typedef union{
  uint8_t bin_val[2];
  uint16_t ushort_val;
  int16_t short_val;
} my_data_u2;

typedef union{
  uint8_t bin_val;
  uint8_t ushort_val;
  int8_t short_val;
} my_data_u1;

typedef enum{
  EXPECTING_HEADER,
  EXPECTING_MID,
  EXPECTING_LEN,
  EXPECTING_PAYLOAD,
  EXPECTING_CHECKSUM,
  EXPECTING_TRAILER
} READING_DATA_STATE;

#endif