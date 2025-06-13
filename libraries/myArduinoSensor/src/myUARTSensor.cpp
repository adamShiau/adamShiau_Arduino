#include "myUARTSensor.h"

// #define test_mode

XsensUart::XsensUart(Stream& serial)
    : serial_(serial) {}

uint8_t XsensUart::getDataStatus() const{
  return data_status;
}
    
void XsensUart::getMeasures(XDI_MID EXPECTED_MID){
  static byte bytes_received = 0;
  data_status = NO_DATA;
  static READING_DATA_STATE state = EXPECTING_HEADER;

  while (serial_.available() > 0 && data_status == NO_DATA) {
    byte data = serial_.read();
    switch (state)
    {
      case EXPECTING_HEADER:{
        if (data != HEADER[bytes_received++])
        {
          state = EXPECTING_HEADER;
          bytes_received = 0;
        }

        if (bytes_received >= HEADER_SIZE)
        {
          state = EXPECTING_MID;
          bytes_received = 0;
        }
        break;
      }
      case EXPECTING_MID:{
        if (data == EXPECTED_MID || EXPECTED_MID == NONE || EXPECTED_MID == WARNING)
        {
          mid = data;
          state = EXPECTING_LEN;
        }
        else state = EXPECTING_HEADER;
        break;
      }
      case EXPECTING_LEN:{
        if (data <= LEN_XBUS) {
          len = data;
          if (len) state = EXPECTING_PAYLOAD;
          else state = EXPECTING_CHECKSUM;
        }
        else state = EXPECTING_HEADER;
        break;
      }
      case EXPECTING_PAYLOAD:{
        buffer_out[bytes_received++] = data;
        if (bytes_received >= len)
        {
          state = EXPECTING_CHECKSUM;
          bytes_received = 0;
        }
        break;
      }
      case EXPECTING_CHECKSUM:{
        checksum = data;
        length_msg = getMessageLength();
        uint8_t* all_message = (uint8_t*)malloc(length_msg);
        getCurrentMessage(all_message);

        if(Xbus_verifyChecksum(all_message, length_msg)){ data_status = DATA_OK; }
        state = EXPECTING_HEADER;
        free(all_message);

        break;
      }
      default: break;
    }
  }
}

void XsensUart::parseData(
  my_data_u2* xsens_counter, my_data_u4* XsensTime, 
  my_data_3f* omg, my_data_3f* acc, 
  my_data_3f* mag, my_data_u4* pressure,
  my_data_3f* vel, my_data_2d* latlon, 
  my_data_u4* hei, my_data_3f* ori, 
  my_data_4f* qut, my_data_u4* status, 
  my_data_u4* temp, UTC_TIME* utc_time){

  #ifdef test_mode
  if (data_status > 0 && mid == MTDATA2){
    Serial.print("LEN: ");Serial.print(len);
    Serial.print("  MID: ");Serial.println(mid, HEX);
    for (int i=0;i<len;i++){
      Serial.print(buffer_out[i], HEX);Serial.print(" ");
    }
    Serial.println();
  }
  #endif

  uint8_t trans_byte = 0;

  if (mid == WARNING){
    #ifdef test_mode
      Serial.print("Warning: ");
      while(trans_byte < len && data_status > 0){
        Serial.print(buffer_out[trans_byte++], HEX); Serial.print(" ");
      }
      Serial.println();
    #endif
    data_status = DATA_WARNING;
    return;
  }


  while(trans_byte < len && data_status > 0 && mid == MTDATA2){
    uint8_t data_length = 0;
    int data_type = buffer_out[trans_byte++]<<8 | buffer_out[trans_byte];        
    switch (data_type)
    {
      case XDI_StatusByte:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        byte status_byte = *(buffer_out + trans_byte);
        break;
      }

      case XDI_PacketCounter:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (xsens_counter != nullptr){
          convert2Sign_2B(&(xsens_counter->ushort_val), buffer_out + trans_byte);

          #ifdef test_mode
          Serial.print("counter DID:");
          Serial.print(data_type, HEX);
          Serial.print("  len:");
          Serial.print(data_length);
          Serial.print("  ");
          for (int i=0;i<data_length;i++){
            Serial.print(buffer_out[trans_byte+i], HEX);
            Serial.print(" ");
          }
          Serial.println(xsens_counter->ushort_val);
          #endif
        }
        break;
      }

      case XDI_SampleTimeFine:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (XsensTime != nullptr){
          convert2Sign_4B(&(XsensTime->ulong_val), buffer_out + trans_byte);

          #ifdef test_mode
          Serial.print("time DID:");
          Serial.print(data_type, HEX);
          Serial.print("  len:");
          Serial.print(data_length);
          Serial.print("  ");
          for (int i=0;i<data_length;i++){
            Serial.print(buffer_out[trans_byte+i], HEX);
            Serial.print(" ");
          }
          Serial.println(XsensTime->ulong_val);
          #endif
        }
        break;
      }

      case XDI_UtcTime:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (utc_time != nullptr){
          convert2Sign_4B(&(utc_time->ns.ulong_val), buffer_out + trans_byte);
          convert2Sign_2B(&(utc_time->year.ushort_val), buffer_out + trans_byte + 4);
          utc_time->month.ushort_val = *(buffer_out + trans_byte + 6);
          utc_time->day.ushort_val = *(buffer_out + trans_byte + 7);
          utc_time->hour.ushort_val = *(buffer_out + trans_byte + 8);
          utc_time->minute.ushort_val = *(buffer_out + trans_byte + 9);
          utc_time->second.ushort_val = *(buffer_out + trans_byte + 10);
        }
        break;
      }

      case XDI_Acceleration:
      { 
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (acc != nullptr){
          convert2Sign_4B(&(acc->ulong_val[0]), buffer_out + trans_byte);
          convert2Sign_4B(&(acc->ulong_val[1]), buffer_out + trans_byte + 4);
          convert2Sign_4B(&(acc->ulong_val[2]), buffer_out + trans_byte + 8);

          #ifdef test_mode
          Serial.print("ACCE DID : ");
          Serial.print(data_type, HEX);
          Serial.print("  len:");
          Serial.print(data_length);
          Serial.print("  ");
          for (int i=0;i<data_length;i++){
            Serial.print(buffer_out[trans_byte+i], HEX);
            Serial.print(" ");
          }
          for (int i=0;i<3;i++){
            Serial.print(acc->float_val[i]);
            Serial.print(" ");
          }
          Serial.println();
          #endif
        }
        break;
      }

      case XDI_AccelerationNED:
      { 
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (acc != nullptr){
          convert2Sign_4B(&(acc->ulong_val[0]), buffer_out + trans_byte);
          convert2Sign_4B(&(acc->ulong_val[1]), buffer_out + trans_byte + 4);
          convert2Sign_4B(&(acc->ulong_val[2]), buffer_out + trans_byte + 8);

          #ifdef test_mode
          Serial.print("ACCE DID : ");
          Serial.print(data_type, HEX);
          Serial.print("  len:");
          Serial.print(data_length);
          Serial.print("  ");
          for (int i=0;i<data_length;i++){
            Serial.print(buffer_out[trans_byte+i], HEX);
            Serial.print(" ");
          }
          for (int i=0;i<3;i++){
            Serial.print(acc->float_val[i]);
            Serial.print(" ");
          }
          Serial.println();
          #endif
        }
        break;
      }

      case XDI_RateOfTurn:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (omg != nullptr){
          convert2Sign_4B(&(omg->ulong_val[0]), buffer_out + trans_byte);
          convert2Sign_4B(&(omg->ulong_val[1]), buffer_out + trans_byte + 4);
          convert2Sign_4B(&(omg->ulong_val[2]), buffer_out + trans_byte + 8);

          if (angle_output_unit){
            omg->float_val[0] = degrees(omg->float_val[0]);
            omg->float_val[1] = degrees(omg->float_val[1]);
            omg->float_val[2] = degrees(omg->float_val[2]);
          }

          #ifdef test_mode
          Serial.print("Gyro DID : ");
          Serial.print(data_type, HEX);
          Serial.print("  len:");
          Serial.print(data_length);
          Serial.print("  ");
          for (int i=0;i<data_length;i++){
            Serial.print(buffer_out[trans_byte+i], HEX);
            Serial.print(" ");
          }
          for (int i=0;i<3;i++){
            Serial.print(omg->float_val[i]);
            Serial.print(" ");
          }
          Serial.println();
          #endif
        }
        break;
      }

      case XDI_RateOfTurnNED:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (omg != nullptr){
          convert2Sign_4B(&(omg->ulong_val[0]), buffer_out + trans_byte);
          convert2Sign_4B(&(omg->ulong_val[1]), buffer_out + trans_byte + 4);
          convert2Sign_4B(&(omg->ulong_val[2]), buffer_out + trans_byte + 8);
          if (angle_output_unit){
            omg->float_val[0] = degrees(omg->float_val[0]);
            omg->float_val[1] = degrees(omg->float_val[1]);
            omg->float_val[2] = degrees(omg->float_val[2]);
          }

          #ifdef test_mode
          Serial.print("Gyro DID : ");
          Serial.print(data_type, HEX);
          Serial.print("  len:");
          Serial.print(data_length);
          Serial.print("  ");
          for (int i=0;i<data_length;i++){
            Serial.print(buffer_out[trans_byte+i], HEX);
            Serial.print(" ");
          }
          for (int i=0;i<3;i++){
            Serial.print(omg->float_val[i]);
            Serial.print(" ");
          }
          Serial.println();
          #endif
        }
        break;
      }

      case XDI_MagneticField:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (mag != nullptr){
          convert2Sign_4B(&(mag->ulong_val[0]), buffer_out + trans_byte);
          convert2Sign_4B(&(mag->ulong_val[1]), buffer_out + trans_byte + 4);
          convert2Sign_4B(&(mag->ulong_val[2]), buffer_out + trans_byte + 8);
          
          #ifdef test_mode
          Serial.print("MAG DID : ");
          Serial.print(data_type, HEX);
          Serial.print("  len:");
          Serial.print(data_length);
          Serial.print("  ");
          for (int i=0;i<data_length;i++){
            Serial.print(buffer_out[trans_byte+i], HEX);
            Serial.print(" ");
          }
          for (int i=0;i<3;i++){
            Serial.print(mag->float_val[i]);
            Serial.print(" ");
          }
          Serial.println();
          #endif
        }
        break;
      }
      
      case XDI_MagneticFieldNED:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (mag != nullptr){
          convert2Sign_4B(&(mag->ulong_val[0]), buffer_out + trans_byte);
          convert2Sign_4B(&(mag->ulong_val[1]), buffer_out + trans_byte + 4);
          convert2Sign_4B(&(mag->ulong_val[2]), buffer_out + trans_byte + 8);
          
          #ifdef test_mode
          Serial.print("MAG DID : ");
          Serial.print(data_type, HEX);
          Serial.print("  len:");
          Serial.print(data_length);
          Serial.print("  ");
          for (int i=0;i<data_length;i++){
            Serial.print(buffer_out[trans_byte+i], HEX);
            Serial.print(" ");
          }
          for (int i=0;i<3;i++){
            Serial.print(mag->float_val[i]);
            Serial.print(" ");
          }
          Serial.println();
          #endif
        }
        break;
      }

      case XDI_BaroPressure:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (pressure != nullptr) convert2Sign_4B(&(pressure->ulong_val), buffer_out + trans_byte);
        break;
      }

      case XDI_BaroPressureNED:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (pressure != nullptr) convert2Sign_4B(&(pressure->ulong_val), buffer_out + trans_byte);
        break;
      }

      case XDI_VelocityXYZ:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (vel != nullptr){
          convert2Sign_4B(&(vel->ulong_val[0]), buffer_out + trans_byte);
          convert2Sign_4B(&(vel->ulong_val[1]), buffer_out + trans_byte + 4);
          convert2Sign_4B(&(vel->ulong_val[2]), buffer_out + trans_byte + 8);
        }
        break;
      }

      case XDI_VelocityXYZNED:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (vel != nullptr){
          convert2Sign_4B(&(vel->ulong_val[0]), buffer_out + trans_byte);
          convert2Sign_4B(&(vel->ulong_val[1]), buffer_out + trans_byte + 4);
          convert2Sign_4B(&(vel->ulong_val[2]), buffer_out + trans_byte + 8);
        }
        break;
      }

      // case XDI_LatLon:
      // {
      //   trans_byte++;
      //   data_length = buffer_out[trans_byte++];
      //   if (latlon != nullptr){
      //     convert2Sign_4B(&(latlon->ulong_val[0]), buffer_out + trans_byte);
      //     convert2Sign_4B(&(latlon->ulong_val[1]), buffer_out + trans_byte + 4);
      //   }
      //   break;
      // }

      case XDI_LatLon8B:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (latlon != nullptr){
          convert2Sign_8B(&(latlon->float_val[0]), buffer_out + trans_byte);
          convert2Sign_8B(&(latlon->float_val[1]), buffer_out + trans_byte + 8);
        }
        break;
      }

      case XDI_AltitudeEllipsoid:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (hei != nullptr) convert2Sign_4B(&(hei->ulong_val), buffer_out + trans_byte);
        break;
      }

      case XDI_StatusWord:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (status != nullptr) convert2Sign_4B(&(status->ulong_val), buffer_out + trans_byte);
        break;
      }

      case XDI_Temperature:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (temp != nullptr) convert2Sign_4B(&(temp->ulong_val), buffer_out + trans_byte);
        break;
      }

      case XDI_EulerAngles:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (ori != nullptr){
          convert2Sign_4B(&(ori->ulong_val[0]), buffer_out + trans_byte);
          convert2Sign_4B(&(ori->ulong_val[1]), buffer_out + trans_byte + 4);
          convert2Sign_4B(&(ori->ulong_val[2]), buffer_out + trans_byte + 8);

          if (!angle_output_unit){
            ori->float_val[0] = radians(ori->float_val[0]);
            ori->float_val[1] = radians(ori->float_val[1]);
            ori->float_val[2] = radians(ori->float_val[2]);
          }
        }
        break;
      }

      case XDI_EulerAnglesNED:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (ori != nullptr){
          convert2Sign_4B(&(ori->ulong_val[0]), buffer_out + trans_byte);
          convert2Sign_4B(&(ori->ulong_val[1]), buffer_out + trans_byte + 4);
          convert2Sign_4B(&(ori->ulong_val[2]), buffer_out + trans_byte + 8);

          if (!angle_output_unit){
            ori->float_val[0] = radians(ori->float_val[0]);
            ori->float_val[1] = radians(ori->float_val[1]);
            ori->float_val[2] = radians(ori->float_val[2]);
          }
        }
        break;
      }

      case XDI_Quaternion:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (qut != nullptr){
          convert2Sign_4B(&(qut->ulong_val[0]), buffer_out + trans_byte);
          convert2Sign_4B(&(qut->ulong_val[1]), buffer_out + trans_byte + 4);
          convert2Sign_4B(&(qut->ulong_val[2]), buffer_out + trans_byte + 8);
          convert2Sign_4B(&(qut->ulong_val[3]), buffer_out + trans_byte + 12);
        }
        break;
      }

      case XDI_QuaternionNED:
      {
        trans_byte++;
        data_length = buffer_out[trans_byte++];
        if (qut != nullptr){
          convert2Sign_4B(&(qut->ulong_val[0]), buffer_out + trans_byte);
          convert2Sign_4B(&(qut->ulong_val[1]), buffer_out + trans_byte + 4);
          convert2Sign_4B(&(qut->ulong_val[2]), buffer_out + trans_byte + 8);
          convert2Sign_4B(&(qut->ulong_val[3]), buffer_out + trans_byte + 12);
        }
        break;
      }

      default: break;
    }
    trans_byte += data_length;
    #ifdef test_mode
      if (data_length > 0) {Serial.print("trans_byte: ");Serial.println(trans_byte);}
    #endif
  }
}

void XsensUart::parseData(XsensDataset* data_set){
  parseData(&(data_set->xsens_counter), &(data_set->XsensTime),
            &(data_set->omg), &(data_set->acc),
            &(data_set->mag), &(data_set->pressure), 
            &(data_set->vel), &(data_set->latlon), 
            &(data_set->hei), &(data_set->ori), 
            &(data_set->qut), &(data_set->status), &(data_set->temp));
}

bool XsensUart::ToConfigMode(){
  Serial.print("Enter Config Mode...");
  uint8_t message[] = {GOTOCONFIG, 0x00, 0xD1};
  if (sendMSGcheckACK(message, sizeof(message), GOTOCONFIGACK)){
    mode = CONFIGMODE;
    return true;
  }
  mode = MEASUREMENTMODE;
  return false;
}

void XsensUart::InitMT(){
  Serial.print("Init Sensor...");
  uint8_t message[] = {INIT_MID, 0x00, 0xFF};
  sendMSGcheckACK(message, sizeof(message), INIT_MIDACK);
}

void XsensUart::getDID(){
  Serial.print("get Device ID...");
  uint8_t message[] = {REQDID, 0x00, 0x01};
  sendMSGcheckACK(message, sizeof(message), DEVICEID);
}

void XsensUart::getFW(){
  Serial.print("get Firmware...");
  uint8_t message[3] = {REQFWREV, 0x00, 0x01};
  message[2] = calCheckSum(message, 2);
  sendMSGcheckACK(message, sizeof(message), FIRMWAREREV);
}

bool XsensUart::ToMeasurementMode(){
  Serial.print("Enter Measurement Mode...");
  uint8_t message[] = {GOTOMEASUREMENT, 0x00, 0xF1};
  clearBuffer(10);
  if (sendMSGcheckACK(message, sizeof(message), GOTOMEASUREMENTACK)){
      mode = MEASUREMENTMODE;
      return true;
  }
  mode = CONFIGMODE;
  return false;
}

//Output: counter, timer, status, temperature, acc, omg
void XsensUart::IMURawMeas(){
  std::vector<uint8_t> msg = vec_append(packetMap[DEFAULT_OP], packetMap[IMU_OP]);
  sendOutputConfigMSG(msg);
}

//Output: counter, timer, status, temperature, acc, omg, mag, bar
void XsensUart::SensorRawMeas(){
  std::vector<uint8_t> msg = vec_append(packetMap[DEFAULT_OP], packetMap[IMU_OP]);
  msg = vec_append(msg, packetMap[MAG_OP]);
  msg = vec_append(msg, packetMap[BAR_OP]);
  sendOutputConfigMSG(msg);
}

//Output: counter, timer, status, temperature, acc, omg, mag, euler
void XsensUart::AHRSData(){
  std::vector<uint8_t> msg = vec_append(packetMap[DEFAULT_OP], packetMap[IMU_OP]);
  msg = vec_append(msg, packetMap[MAG_OP]);
  msg = vec_append(msg, packetMap[EULERs_OP]);
  sendOutputConfigMSG(msg);
}

//Output: counter, timer, status, GNSS measurement
void XsensUart::GNSSRawMeas(){
  std::vector<uint8_t> msg = vec_append(packetMap[DEFAULT2_OP], packetMap[GNSS_RAW_OP]);
  sendOutputConfigMSG(msg);
}

//Output: counter, timer, status, GNSS measurement
void XsensUart::TotalRawMeas(){
  std::vector<uint8_t> msg = vec_append(packetMap[DEFAULT_OP], packetMap[IMU_OP]);
  msg = vec_append(msg, packetMap[MAG_OP]);
  msg = vec_append(msg, packetMap[BAR_OP]);
  msg = vec_append(msg, packetMap[GNSS_RAW_OP]);
  sendOutputConfigMSG(msg);
}

//Output: counter, timer, status, temperature, pos, vel, euler
void XsensUart::INSData(){
  std::vector<uint8_t> msg = vec_append(packetMap[DEFAULT_OP], packetMap[EKF_OP]);
  msg = vec_append(msg, packetMap[EULERs_OP]);
  sendOutputConfigMSG(msg);
}

//Output: counter, timer, status, temperature, acc, omg, mag, bar, pos, vel, euler
void XsensUart::INS_RAW_Data(){
  std::vector<uint8_t> msg = vec_append(packetMap[DEFAULT_OP], packetMap[IMU_OP]);
  msg = vec_append(msg, packetMap[MAG_OP]);
  msg = vec_append(msg, packetMap[BAR_OP]);
  msg = vec_append(msg, packetMap[GNSS_RAW_OP]);
  msg = vec_append(msg, packetMap[EKF_OP]);
  msg = vec_append(msg, packetMap[EULERs_OP]);
  sendOutputConfigMSG(msg);
}

void XsensUart::INS_PAV_QUT(){
  std::vector<uint8_t> msg = vec_append(packetMap[DEFAULT_OP], packetMap[EKF_OP]);
  msg = vec_append(msg, packetMap[QUT_OP]);
  msg = vec_append(msg, packetMap[GYRO_OP]);
  sendOutputConfigMSG(msg);
}

void XsensUart::INS_UTC(){
  std::vector<uint8_t> msg = vec_append(packetMap[DEFAULT_OP], packetMap[EKF_OP]);
  msg = vec_append(msg, packetMap[EULERs_OP]);
  msg = vec_append(msg, packetMap[UTC_OP]);
  sendOutputConfigMSG(msg);
}

void XsensUart::sendMessage(const uint8_t* message, size_t size){
  serial_.write(HEADER, sizeof(HEADER));
  serial_.write(message, size);
  // Serial.print("Send: ");
  // Serial.print(HEADER[0], HEX); Serial.print(" ");
  // Serial.print(HEADER[1], HEX); Serial.print(" ");
  // for (int i=0;i<size;i++){
  //   Serial.print(message[i], HEX); Serial.print(" ");
  // }
  // Serial.println();
  delay(100);
}

void XsensUart::PrintMessage(HardwareSerial &output_port){
  int length = getMessageLength();
  uint8_t* msg = (uint8_t*)malloc(length);
  getCurrentMessage(msg);
  for (int i=0;i<length;i++){
    output_port.print(msg[i], HEX);
    output_port.print(" ");
  }
  output_port.println();
}

void XsensUart::getCurrentMessage(uint8_t* all_msg){
  uint8_t idx = 0;
  all_msg[idx++] = HEADER[0];
  all_msg[idx++] = HEADER[1];
  all_msg[idx++] = mid;
  all_msg[idx++] = len;
  for(int i=0;i<len;i++) { all_msg[idx++] = buffer_out[i]; }
  all_msg[idx] = checksum;
}

int XsensUart::getMessageLength(){
  return HEADER_SIZE + 1 + 1 + len + 1;
}

void XsensUart::initPacketMap(){
  uint8_t d1 = data_rate >> 8;
  uint8_t d2 = data_rate & 0xFF;
  packetMap = {
    { OutputPackets::DEFAULT_OP,   {static_cast<uint8_t>(0x10), static_cast<uint8_t>(0x20), static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0x10), static_cast<uint8_t>(0x60), static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xE0), static_cast<uint8_t>(0x20), static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0x08), static_cast<uint8_t>(0x10 | frame_ENU_BIT), d1, d2} },
    { OutputPackets::DEFAULT2_OP,  {static_cast<uint8_t>(0x10), static_cast<uint8_t>(0x20), static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0x10), static_cast<uint8_t>(0x60), static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xE0), static_cast<uint8_t>(0x20), static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xFF)} },
    { OutputPackets::IMU_OP,       {static_cast<uint8_t>(0x40), static_cast<uint8_t>(0x20), d1, d2, static_cast<uint8_t>(0x80), static_cast<uint8_t>(0x20), d1, d2} },
    { OutputPackets::MAG_OP,       {static_cast<uint8_t>(0xC0), static_cast<uint8_t>(0x20), d1, d2} },
    { OutputPackets::BAR_OP,       {static_cast<uint8_t>(0x30), static_cast<uint8_t>(0x10), d1, d2} },
    { OutputPackets::GNSS_RAW_OP,  {static_cast<uint8_t>(0x70), static_cast<uint8_t>(0x10 | frame_ENU_BIT), static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xFF)} },
    { OutputPackets::EULERs_OP,    {static_cast<uint8_t>(0x20), static_cast<uint8_t>(0x30 | frame_ENU_BIT), d1, d2} },
    { OutputPackets::QUT_OP,       {static_cast<uint8_t>(0x20), static_cast<uint8_t>(0x10 | frame_ENU_BIT), d1, d2} },
    { OutputPackets::EKF_OP,       {static_cast<uint8_t>(0x50), static_cast<uint8_t>(0x43 | frame_ENU_BIT), d1, d2, static_cast<uint8_t>(0x50), static_cast<uint8_t>(0x20 | frame_ENU_BIT), d1, d2, static_cast<uint8_t>(0xD0), static_cast<uint8_t>(0x10 | frame_ENU_BIT), d1, d2} },
    { OutputPackets::GYRO_OP,      {static_cast<uint8_t>(0x80), static_cast<uint8_t>(0x20), d1, d2} },
    { OutputPackets::UTC_OP,       {static_cast<uint8_t>(0x10), static_cast<uint8_t>(0x10), static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xFF)}} 
  };
  // Serial.print("Data Rate = ");Serial.println(data_rate);
  // Serial.print("Cord Frame = ");Serial.println(frame_ENU_BIT);
}

bool XsensUart::sendMSGcheckACK(uint8_t *buffer, uint8_t size, XDI_MID mid_ACK){
  initPacketMap();
  for (int i=0;i<5;i++) {
    sendMessage(buffer, size);
    getMeasures(mid_ACK);
    if(getDataStatus() == DATA_OK) {
      Serial.println("->Successed.");
      if (mid_ACK == RESET_MIDACK) delay(2000);
      return true;
    }
    while (serial_.available()) {serial_.read();}
    delay(100);
  }
  Serial.println("->Failed.");

  Serial.print("Send: ");
  Serial.print(HEADER[0], HEX); Serial.print(" ");
  Serial.print(HEADER[1], HEX); Serial.print(" ");
  for (int i=0;i<size;i++){
    Serial.print(buffer[i], HEX); Serial.print(" ");
  }
  Serial.println();

  return false;
}

// xbusMessage doesn't include header
uint8_t XsensUart::calCheckSum(uint8_t* xbusMessage, int nBytes)
{
	uint8_t checksum = -HEADER[1];
	for (int i = 0; i < nBytes; i++)
		checksum -= xbusMessage[i];
  return checksum;
}

bool XsensUart::Xbus_verifyChecksum(const uint8_t* xbusMessage, int nBytes)
{
	uint8_t checksum = 0;
	for (int n = 1; n < nBytes; n++)
		checksum += (xbusMessage[n] & 0xff);
	checksum &= 0xff;
	return (checksum == 0);
}

void XsensUart::printXsensSerial(){
  while(serial_.available()>0){
    Serial.print(serial_.read(), HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void XsensUart::reset(){
  Serial.print("Reset...");
  uint8_t message[] = {RESET_MID, 0x00, 0xC1};
  sendMSGcheckACK(message, sizeof(message), RESET_MIDACK);
}

void XsensUart::setDataRate(int data_rate){
  this->data_rate = data_rate;
  initPacketMap();
}

// false: radian, true: degree
void XsensUart::setAngleUnitDeg(bool unit){
  angle_output_unit = unit;
}

void XsensUart::setAlignmentRotation(float q0, float q1, float q2, float q3){
  my_data_4f qut;
  qut.float_val[0] = q0;
  qut.float_val[1] = q1;
  qut.float_val[2] = q2;
  qut.float_val[3] = q3;
  
  uint8_t total_data_len = 1 + 1 + (1 + 4*4) + 1;   // MID + num_data + data(1 + 4*4) + checksum
  uint8_t message[total_data_len];
  message[0] = SETALIGNMENTROTATION_MID;        // MID
  message[1] = total_data_len - 3;              // num_data 
  message[2] = 0;                               // rotation params: 0: Sensor Alignment, 1: Local Alignemtn
  for (int i=0;i<4;i++){
    for (int j=0;j<4;j++){
      message[i * 4 + j + 3] = qut.bin_val[i * 4 + (3 - j)];
    }
  }
  message[total_data_len - 1] = calCheckSum(message, total_data_len - 1);
  Serial.print("Set Alignment...");
  sendMSGcheckACK(message, sizeof(message), SETALIGNMENTROTATION_MIDACK);

  total_data_len = 1 + 1 + 1 + 1;               // MID + num_data + data(1 + 4*4) + checksum
  uint8_t req_message[total_data_len];
  req_message[0] = SETALIGNMENTROTATION_MID;    // MID of request is same as MID of setting
  req_message[1] = total_data_len - 3;          // num_data 
  req_message[2] = 0;                           // rotation params: 0: Sensor Alignment, 1: Local Alignemtn                
  req_message[total_data_len - 1] = calCheckSum(req_message, total_data_len - 1);
  Serial.print("REQ Alignment...");
  sendMSGcheckACK(req_message, sizeof(req_message), SETALIGNMENTROTATION_MIDACK);
}

void XsensUart::setFrameENU(bool frame){
  if (frame){ frame_ENU_BIT = 0x0; }
  else      { frame_ENU_BIT = 0x4; }
  initPacketMap();
}

void XsensUart::setGnssReceiverSettings(int baudrate, int update_rate, int talker_id){
  // Message includes MID, num_data, data (10 bytes), checksum  (total 13 bytes)
  // data byte 0 to 1: GNSS Receiver Type
  // data byte 2 to 3: Baud Rate
  // data byte 4 to 5: Update Rate (hz)
  // data byte 6 to 9: Talker ID (0 for GL, 1 for GN, 2 for GP)
  Serial.print("Config Gnss Receiver...");
  uint8_t message[] = {SETGNSSMID, 10, 0, 1, 0, 2, 0, update_rate, 0, 0, 0, talker_id, 99};
  message[12] = calCheckSum(message, 12);
  sendMSGcheckACK(message, sizeof(message), SETGNSSMIDACK);
}

// this cofiguration would take effect after rebooting the snesor
void XsensUart::setStringOutputType(int odr){
  // Message includes MID, num_data, data (6 bytes), checksum (total 9 bytes)
  // data byte 0 to 3: NMEA OUTPUT (Bitmask): 0050 includes HEHDT and GPGGA
  // data byte 4 to 5: Output Data Rate
  uint8_t message[] = {STRINGOUTPUTTYPE_MID, 6, 0, 0, 4, 0, 0, odr, 99};
  message[8] = calCheckSum(message, 8);
  Serial.print("Set String Output...");
  sendMSGcheckACK(message, sizeof(message), STRINGOUTPUTTYPE_MID);
}

// output_type 1 for xbus, 5 for NMEA
void XsensUart::setPortConfig(uint8_t output_type, int baudrate){
  // Message includes MID, num_data, data (12 bytes), checksum (total 15 bytes)
  // data byte 0 to 3: NMEA OUTPUT (Bitmask): 0050 includes HEHDT and GPGGA
  // data byte 4 to 5: Output Data Rate
  //baudrate: 115200 = 0x02
  uint8_t message[] = {SETPORTCONFIG_MID, 12, 
                      0, 1, 1, 2, 
                      0, output_type, 0, 2, 
                      0, 0, 0, 0, 99};

  message[14] = calCheckSum(message, 14);
  Serial.print("Config Port...");
  sendMSGcheckACK(message, sizeof(message), SETPORTCONFIG_MIDACK);
  init();
}

void XsensUart::RestoreFactoryDef(){
  uint8_t message[] = {RESTOREFACTORY_MID, 0, 99};
  message[2] = calCheckSum(message, 2);
  Serial.print("Restore Factory Setting...");
  sendMSGcheckACK(message, sizeof(message), NONE);
  init();
}

// output_type 1 for xbus, 5 for NMEA
void XsensUart::reqPortConfig(){
  // Message includes MID, num_data, data (12 bytes), checksum (total 15 bytes)
  // data byte 0 to 3: NMEA OUTPUT (Bitmask): 0050 includes HEHDT and GPGGA
  // data byte 4 to 5: Output Data Rate
  //baudrate: 115200 = 0x02
  uint8_t message[] = {SETPORTCONFIG_MID, 0, 99};

  message[2] = calCheckSum(message, 2);
  Serial.print("Req Port...");
  sendMSGcheckACK(message, sizeof(message), SETPORTCONFIG_MIDACK);

  int length = getMessageLength();
  uint8_t* msg = (uint8_t*)malloc(length);
  getCurrentMessage(msg);

  for (int i=0;i<length;i++){
    Serial.print(msg[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("Port Config: ");
  Serial.print(msg[8]);
  Serial.print(" ");
  if (msg[9] == 1) {
    Serial.print("1 (Xbus out) ");
  }
  else {
    Serial.print(msg[9]);
    Serial.print(" ");
  }

  Serial.print(msg[10]);
  Serial.print(" ");
  Serial.print(msg[11]);

  switch (msg[11])
  {
  case 0:
    Serial.println("(460800)");
    break;
  
  case 1:
    Serial.println("(230400)");
    break;

  case 2:
    Serial.println("(115200)");
    break;
    
  default:
    Serial.println();
    break;
  }

}

bool XsensUart::caliGyro(uint16_t sec, Stream *output_port){
  uint8_t message[] = {SETNOROTATION, 2, sec >> 8, sec & 0xFF, 99};
  message[4] = calCheckSum(message, 4);
  sendMSGcheckACK(message, sizeof(message), NONE);
}

void XsensUart::clearBuffer(uint32_t timeout_ms) {
  uint32_t st = millis();
  while (serial_.available()) {
    serial_.read();
    if (millis() - st > timeout_ms) {
      serial_.println("Encouter timeout when clearing serial buffer.");
      break;
    }
  }
}

// return filter mode (0: without GNSS, 1: coasting mode, 2: with GNSS)
byte XsensUart::getFilterStatus(my_data_u4 status_world, bool print_result, HardwareSerial &output_port){
  // output_port.print("Status World: ");
  // for (int i=0;i<4;i++){
  //   output_port.print(status_world.bin_val[i], BIN);
  //   output_port.print(" ");
  // }
  // output_port.print(',');
  
  byte filter_mode = ((status_world.bin_val[3] & 0b00000001) << 1) | ((status_world.bin_val[2] & 0b10000000) >> 7);

  if (print_result){
    output_port.print("Filter Mode: ");
    output_port.print(filter_mode);
    if (filter_mode == 0){
      output_port.print(" (Without GNSS)");
    } else if (filter_mode == 1){
      output_port.print(" (Coasting Mode)");
    } else if (filter_mode == 3){
      output_port.print(" (With GNSS)");
    }

    byte filter_valid = (status_world.bin_val[0] & 0b00000010) >> 1;
    output_port.print(", Filter Valid: ");
    output_port.print(filter_valid);

    byte GNSS_fix = (status_world.bin_val[0] & 0b00000100) >> 2;
    output_port.print(", GNSS Fix: ");
    output_port.println(GNSS_fix);
  }
  return filter_mode;
}

byte XsensUart::getNoRotationUpdateStatus(my_data_u4 status_world){
  return (status_world.bin_val[0] & 0b00011000) >> 3;
}

void XsensUart::getINSStatus(my_data_u4 status_world, uint8_t (&ins_status)[2]){
  byte filter_mode = ((status_world.bin_val[3] & 0b00000001) << 1) | ((status_world.bin_val[2] & 0b10000000) >> 7);
  memset(ins_status, 0, 2);
  if (filter_mode == 1){
    ins_status[0] |= 0b00000010;  // Coasting Mode
  } else if (filter_mode == 3){
    ins_status[0] |= 0b00000001;  // With GNSS
  }
  
  if ((status_world.bin_val[0] & 0b00000100) >> 2){
    ins_status[0] |= 0b00000100;  // GNSS Fix
  }
}

std::vector<uint8_t> XsensUart::genOutputConfigMSG(const std::vector<uint8_t> message){
  std::vector<uint8_t> bytes = {OUTPUTCONFIGURATION, 0};
  for (int i=0;i<message.size();i++){
    bytes.push_back(message[i]);
  }

  bytes[1] = message.size();
  bytes.push_back(calCheckSum(bytes.data(), bytes.size()));
  return bytes;
}

void XsensUart::sendOutputConfigMSG(const std::vector<uint8_t> message){
  std::vector<uint8_t> byte_msg = genOutputConfigMSG(message);
  Serial.print("Config Output Mode...");
  sendMSGcheckACK(byte_msg.data(), byte_msg.size(), OUTPUTCONFIGURATIONACK);
}


std::vector<uint8_t> vec_append(std::vector<uint8_t> vec1, std::vector<uint8_t> vec2){
  // 建立一個新的 vector 並預先保留足夠容量
  std::vector<uint8_t> combined;
  combined.reserve(vec1.size() + vec2.size());

  // 將 vec1 的所有元素插入 combined
  combined.insert(combined.end(), vec1.begin(), vec1.end());
  // 將 vec2 的所有元素插入 combined
  combined.insert(combined.end(), vec2.begin(), vec2.end());
  return combined;
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

bool checkLOCOSYS_ACK(HardwareSerial &serial_nmea){
  // serial_nmea.println("$PAIR003*39");      關閉GNSS
  // serial_nmea.println("$PAIR062,8,1*37");  開啟GST
  // serial_nmea.println("$PAIR062,2,1*3D");  開啟GSA
  // serial_nmea.println("$PAIR513*3D");      將目前設定寫入記憶體
  // serial_nmea.println("$PAIR002*38");      重新開啟GNSS

  ulong t0 = millis();
  while(true){
    char receivedChars[256];
    readNMEA(serial_nmea, receivedChars, 256);
    if (strstr(receivedChars, "$PAIR") != NULL)
    {
        Serial.println(receivedChars);
        return true;
    }

    if(millis() - t0 > 1000){
      Serial.println("Timeout");
      return false;
    }
  }
  return false;
}

void printNMEA(HardwareSerial &serial_nmea){
  char buffer[256];
  int bytesRead = readNMEA(serial_nmea, buffer, 256);
  if (bytesRead){
    Serial.println(buffer);
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

void printNMEA(const char* filters[], int num_filter, HardwareSerial &serial_nmea, HardwareSerial &serial_output, bool debug){
  char buffer[256];
  int bytesRead = readNMEA(serial_nmea, buffer, 256);
  if (bytesRead){
    for (int i = 0; i < num_filter; i++){
      if (strstr(buffer, filters[i]) != NULL){
        serial_output.println(buffer);
        if (debug) Serial.println(buffer);
        break;
      }
    }
  }
}

void modifyTimestampDecimals(char* nmeaMessage, int num_bytes) {
  char* commaPos = strchr(nmeaMessage, ',');
  if (commaPos != nullptr) {
    char* dotPos = strchr(commaPos + 1, '.');
    if (dotPos != nullptr && (dotPos - commaPos) > 1 && *(dotPos + 3) != ',') {
      num_bytes--;
      for (int i=(dotPos - nmeaMessage) + 3; i<num_bytes+1; i++){
        nmeaMessage[i] = nmeaMessage[i + 1];
      }
    }
  }

  // Recalculate checksum
  char* checksumPos = strrchr(nmeaMessage, '*');
  if (checksumPos != nullptr) {
    uint8_t checksum = 0;
    for (char* ptr = nmeaMessage + 1; ptr < checksumPos; ++ptr) {
      checksum ^= *ptr;
    }
    snprintf(checksumPos + 1, 3, "%02X", checksum);
  }
}

void printNMEAWithModifiedTimestamp(HardwareSerial &serial_nmea, HardwareSerial &serial_output, bool debug) {
  char buffer[256];
  int bytesRead = readNMEA(serial_nmea, buffer, 256);
  if (bytesRead) {
    if (strstr(buffer, "GGA") != NULL || strstr(buffer, "RMC") != NULL || strstr(buffer, "GST") != NULL) {
      modifyTimestampDecimals(buffer, bytesRead);
    }
    serial_output.println(buffer);
    if (debug) Serial.println(buffer);
  }
}
