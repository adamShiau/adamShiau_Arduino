#ifndef MYUARTSENSOR_h
#define MYUARTSENSOR_h

#include <vector>
#include <sstream>
#include "myMessage.h"
#include "Xsens/XsensMessage.h"

#define LEN_XBUS 200


struct XsensDataset
{
  my_data_3f omg, acc, ori, mag, vel;
  my_data_4f qut;
  my_data_u4 XsensTime, temp, pressure, hei, status;
  my_data_2d latlon;
  my_data_u2 xsens_counter;
};


class XsensUart {
  public:
    XsensUart(Stream& serial);
    uint8_t getDataStatus() const;
    void parseData(my_data_u2* xsens_counter, my_data_u4* XsensTime=nullptr, 
                    my_data_3f* omg=nullptr, my_data_3f* acc=nullptr,
                    my_data_3f* mag=nullptr, my_data_u4* pressure=nullptr,
                    my_data_3f* vel=nullptr, my_data_2d* latlon=nullptr, 
                    my_data_u4* hei=nullptr, my_data_3f* ori=nullptr, 
                    my_data_4f* qut=nullptr, my_data_u4* status=nullptr, 
                    my_data_u4* temp=nullptr, UTC_TIME* utc_time=nullptr);
    void parseData(XsensDataset* data_set);
    bool ToMeasurementMode();
    bool ToConfigMode();
    void getDID();
    void getFW();
    void InitMT();

    void sendMessage(const uint8_t* command, size_t size);
    void getMeasures(XDI_MID EXPECTED_MID=NONE);
    void printXsensSerial();

    void IMURawMeas();
    void SensorRawMeas();
    void GNSSRawMeas();
    void TotalRawMeas();
    void AHRSData();
    void INSData();
    void INS_RAW_Data();
    void INS_PAV_QUT();
    void INS_UTC();
    
    void reset();
    void setDataRate(int data_rate);
    void setAngleUnitDeg(bool unit);    
    void setAlignmentRotation(float q0, float q1, float q2, float q3);
    void setFrameENU(bool frame);
    void setGnssReceiverSettings(int baudrate=115200, int update_rate=2, int talker_id=1);
    void setStringOutputType(int odr);
    void setPortConfig(uint8_t output_type, int baudrate=115200);
    void setOutputNMEA() {setPortConfig(5);}
    void setOutputXBUS() {setPortConfig(1);}
    void RestoreFactoryDef();
    
    void reqPortConfig();

    bool caliGyro(uint16_t sec, Stream *output_port=nullptr);
    void clearBuffer(uint32_t timeout_ms = 500);

    byte getFilterStatus(my_data_u4 status_world, bool print_result=false, HardwareSerial &output_port=Serial);
    byte getNoRotationUpdateStatus(my_data_u4 status_world);
    void getINSStatus(my_data_u4 status_world, uint8_t (&ins_status)[2]);
    void PrintMessage(HardwareSerial &output_port=Serial);

  private:
    Stream& serial_;
    uint8_t data_status = NO_DATA;
    bool angle_output_unit = true;            // false: radian, true: degree
    int length_msg = 0;
    int data_rate = 100;
    uint8_t frame_ENU_BIT = 0x0;              // 0x0: ENU, 0x4: NED,
    uint8_t mid = 0;
    uint8_t len = 43;
    uint8_t checksum = 0;
    uint8_t buffer[LEN_XBUS], buffer_out[LEN_XBUS];
    uint8_t mode = MEASUREMENTMODE;
    std::map<OutputPackets, std::vector<uint8_t>> packetMap = {};

    bool sendMSGcheckACK(uint8_t *buffer, uint8_t size, XDI_MID mid_ACK);
    uint8_t calCheckSum(uint8_t* xbusMessage, int nBytes);
    bool Xbus_verifyChecksum(const uint8_t* xbusMessage, int nBytes);
    void getCurrentMessage(uint8_t* all_msg);
    int getMessageLength();
    void initPacketMap();
    std::vector<uint8_t> genOutputConfigMSG(const std::vector<uint8_t> message);
    void sendOutputConfigMSG(const std::vector<uint8_t> message);
};

std::vector<uint8_t> vec_append(std::vector<uint8_t> vec1, std::vector<uint8_t> vec2);
int readNMEA(HardwareSerial &serial_nmea, char* buffer, int buffer_size=1024);
bool checkLOCOSYS_ACK(HardwareSerial &serial_nmea);
void printNMEA(HardwareSerial &serial_nmea=Serial1);
void printNMEA(HardwareSerial &serial_nmea, HardwareSerial &serial_output, bool debug);
void printNMEA(const char* filters[], int filterCount, HardwareSerial &serial_nmea, HardwareSerial &serial_output, bool debug);
void printNMEAWithModifiedTimestamp(HardwareSerial &serial_nmea, HardwareSerial &serial_output, bool debug);

#endif
