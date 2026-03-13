# include "myMessage.h"


MyCRC::MyCRC(uint32_t width, uint32_t polynomial, uint32_t initial_remainder)
    : WIDTH(width), POLYNOMIAL(polynomial), REMAINDER(initial_remainder), crcFailCnt(0) {
    NUM_BYTE = (WIDTH + 7) / 8;
}

// Calculate CRC, num_msg includ the CRC bytes
void MyCRC::calCRC(uint8_t *message, int num_msg) {
    uint32_t TOPBIT = (1 << (WIDTH - 1));
    uint32_t mask = (1 << WIDTH) - 1;
    uint32_t remainder = REMAINDER;

    for (int n = 0; n < num_msg - NUM_BYTE; n++) {
        remainder ^= (message[n] << (WIDTH - 8));

        for (int i = 0; i < 8; i++) {
            if (remainder & TOPBIT) {
                remainder = ((remainder << 1) & mask) ^ POLYNOMIAL;
            } else {
                remainder = (remainder << 1) & mask;
            }
        }
    }

    for (int i = 0; i < NUM_BYTE; i++) {
        message[num_msg - NUM_BYTE + i] = (remainder >> (8 * (NUM_BYTE - 1 - i))) & 0xFF;
    }
}

// num_msg includ the CRC bytes
bool MyCRC::isCRCPass(uint8_t *message, int num_msg) {
    uint8_t temp_msg[num_msg + NUM_BYTE];
    for (int i = 0; i < num_msg; i++) { temp_msg[i] = message[i]; }
    calCRC(temp_msg, num_msg + NUM_BYTE);
    for (int i=0;i<NUM_BYTE;i++){
        if (temp_msg[num_msg + i] != 0) { return false; }
    }
    return true;
}

// num_msg "not" include the checksum
uint8_t cal_xor_checksum(uint8_t *msg, int num_msg){
    uint8_t xor_result = 0;
    for (int i=0;i<num_msg;i++){
        xor_result ^= msg[i];
    }
    return xor_result;
}

String cal_xor_checksum(String nmea_string) {
  byte checksum = 0;
  for (int i = 1; i < nmea_string.length(); i++) {
    checksum ^= nmea_string.charAt(i);
  }
  char buffer[3];
  sprintf(buffer, "%02X", checksum);
  return String(buffer);
}

// num_msg "not" include the checksum
bool xor_checksum(uint8_t *msg, int num_msg, uint8_t checksum){
    return cal_xor_checksum(msg, num_msg) == checksum;
}

void convert2Sign_8B(double *value, uint8_t *buf) {
    uint8_t temp[8];
    for (int i = 0; i < 8; i++) {
        temp[i] = buf[7 - i];  // 反轉順序
    }
    memcpy(value, temp, 8);  // 直接將 8 Byte 複製到 double
}

void convert2Sign_4B(uint32_t* value, uint8_t* buf){
    *value = *buf<<24 | *(buf+1)<<16 | *(buf+2)<<8 | *(buf+3);
}
  

void convert2Sign_2B(uint16_t* value, uint8_t* buf){
    *value = *buf<<8 | *(buf+1);
}

char *my_dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  asm(".global _printf_float");

  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}

int appendValue2Str(char *buffer, int bufSize, int curIndex, float value, int decimalPlaces) {
    if (buffer == nullptr || bufSize <= 0 || curIndex < 0 || curIndex >= bufSize) {
        return -1;
    }
    int remaining = bufSize - curIndex;

    char temp[32] = {0};
    if (isnan(value) || isinf(value)) {
        strncpy(temp, "-999999.0,", sizeof(temp) - 1);
    } else if (value > -999999 && value < 999999) {
        my_dtostrf(value, 0, decimalPlaces, temp);
    } else {
        strncpy(temp, "999999.0,", sizeof(temp) - 1);
    }

    // 計算要附加的字串長度
    int tempLen = strlen(temp);
    temp[tempLen++] = ',';  // 添加逗號作為分隔符
    temp[tempLen++] = '\0';
    if (tempLen >= remaining) {
        return -1; // 緩衝區空間不足
    }
    // 複製到目標緩衝區
    memcpy(buffer + curIndex, temp, tempLen);
    // 回傳index為最後一個字元，而不是'\0'
    return curIndex + tempLen - 1;
}


int appendValue2Str(char *buffer, int bufSize, int curIndex, double value, int decimalPlaces) {
    if (buffer == nullptr || bufSize <= 0 || curIndex < 0 || curIndex >= bufSize) {
        return -1; // 錯誤：無效的緩衝區或索引
    }
    int remaining = bufSize - curIndex;

    char temp[32] = {0};
    if (value > -999999 && value < 999999) {
        my_dtostrf(value, 0, decimalPlaces, temp);
    } else if (isnan(value)) {
        strncpy(temp, "nan", sizeof(temp) - 1);
    } 
    else if(isinf(value)){
        strncpy(temp, "inf", sizeof(temp) - 1);
    } else {
        strncpy(temp, "999999.0", sizeof(temp) - 1);
    }

    // 計算要附加的字串長度
    int tempLen = strlen(temp);
    temp[tempLen++] = ',';  // 添加逗號作為分隔符
    temp[tempLen++] = '\0';
    if (tempLen >= remaining) {
        return -1; // 緩衝區空間不足
    }
    // 複製到目標緩衝區
    memcpy(buffer + curIndex, temp, tempLen);
    // 回傳index為最後一個字元，而不是'\0'
    return curIndex + tempLen - 1;
}

// 將一個浮點數陣列依序格式化後附加到緩衝區中
int appendValues2Str(char *buffer, int bufSize, int curIndex, const float *values, int numValues, int decimalPlaces) {
    // 輸入檢查
    if (buffer == nullptr || values == nullptr || numValues < 0) {
        return -1;
    }
    // 依序處理每個數值
    for (int i = 0; i < numValues; i++) {
        curIndex = appendValue2Str(buffer, bufSize, curIndex, values[i], decimalPlaces);
        if (curIndex < 0) {
            return -1; // 寫入過程中發生錯誤
        }
    }
    return curIndex;
}


// 將一個浮點數陣列依序格式化後附加到緩衝區中
int appendValues2Str(char *buffer, int bufSize, int curIndex, const double *values, int numValues, int decimalPlaces) {
    // 輸入檢查
    if (buffer == nullptr || values == nullptr || numValues < 0) {
        return -1;
    }
    // 依序處理每個數值
    for (int i = 0; i < numValues; i++) {
        curIndex = appendValue2Str(buffer, bufSize, curIndex, values[i], decimalPlaces);
        if (curIndex < 0) {
            return -1; // 寫入過程中發生錯誤
        }
    }
    return curIndex;
}


void LEDblink(){
    static uint32_t pre_time = 0;
    static bool ledState = false;
    static bool is_pin_set = false;

    if (!is_pin_set){
        is_pin_set = true;
        pinMode(LED_BUILTIN, OUTPUT);
    }

    if (millis() - pre_time >= 1000){
        pre_time = millis();
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
    }
}


void blinkLED(SYSTEM_STATE sys_state) {
    static uint32_t pre_time = 0;
    static bool ledState = false;
    static bool is_pin_set = false;

    if (!is_pin_set){
        is_pin_set = true;
        pinMode(LED_BUILTIN, OUTPUT);
    }
  
    uint32_t current_time = millis();
    switch (sys_state)
    {
    case INITIALIZING:
        if (current_time - pre_time >= 100) {
            ledState = !ledState;
            digitalWrite(LED_BUILTIN, ledState);
            pre_time = current_time;
        }
        break;
    case IMU_MEASURING:
        if (current_time - pre_time >= 1000) {
            ledState = !ledState;
            digitalWrite(LED_BUILTIN, ledState);
            pre_time = current_time;
        }
        break;
    case CONFIGURING:
        if (current_time - pre_time >= 500) {
            ledState = true;
            digitalWrite(LED_BUILTIN, ledState);
            pre_time = current_time;
        }
        break;
    default:
        if (current_time - pre_time >= 100) {
            ledState = false;
            digitalWrite(LED_BUILTIN, ledState);
            pre_time = current_time;
        }
        break;
    }
}


void checkSTRCommand(OutputMode &output_mode, Stream &port){
    static const uint8_t MAX_COMMAND_LENGTH = 10;
    static char command_buffer[MAX_COMMAND_LENGTH + 1];
    static uint8_t buffer_index = 0;

    while (port.available()) {
        char c = port.read();
        if (c == '\r' || c == '\n'){continue;}      
    
        if (buffer_index < MAX_COMMAND_LENGTH){
            command_buffer[buffer_index++] = c;
            command_buffer[buffer_index] = '\0';
        } else {
            buffer_index = 0;
            command_buffer[0] = '\0';
        }
    
        if (buffer_index >= 3){
            if (strcmp(command_buffer, "STR") == 0) {
                port.println("Output mode: STR");
                output_mode = OUT_MODE_STR;
            } 
            else if (strcmp(command_buffer, "BIN") == 0) {
                port.println("Output mode: BIN");
                output_mode = OUT_MODE_BIN;
            } 
            else if (buffer_index >= 4 && strcmp(command_buffer, "XBUS") == 0) {
                port.println("Output mode: XBUS");
                output_mode = OUT_MODE_XBUS;
            } 
            else if (buffer_index >= 4 && strcmp(command_buffer, "GNSS") == 0) {
                port.println("Output mode: GNSS");
                output_mode = OUT_MODE_GNSS;
            } 
            else if (buffer_index >= 4 && strcmp(command_buffer, "NONE") == 0) {
                port.println("Output mode: None");
                output_mode = OUT_MODE_NONE;
            } 
            else if (buffer_index >= 6 && strcmp(command_buffer, "CONFIG") == 0) {
                port.println("Output mode: CONFIG");
                output_mode = OUT_MODE_CONFIG;
            } 
            else if (buffer_index >= 7 && strcmp(command_buffer, "MAVLINK") == 0) {
                port.println("Output mode: MAVLINK");
                output_mode = OUT_MODE_MAVLINK;
            } 
        }
        return;
    }
    
    if (buffer_index > 0){
        buffer_index = 0;
        command_buffer[0] = '\0';
    }    
}


String formatLatitude(float lat) {
  int degrees = (int)lat;
  float minutes = (lat - degrees) * 60.0;
  char buffer[10];
  sprintf(buffer, "%02d%07.4f", abs(degrees), minutes);
  return String(buffer);
}

String formatLongitude(float lon) {
  int degrees = (int)lon;
  float minutes = (lon - degrees) * 60.0;
  char buffer[11];
  sprintf(buffer, "%03d%07.4f", abs(degrees), minutes);
  return String(buffer);
}

String readCurrentBytes(Stream &port, uint16_t timeout_ms){
    String output;
    uint32_t start_time = millis();

    while (millis() - start_time < timeout_ms) {
        while (port.available()) {
            char c = (char)port.read();
            output += c;
            start_time = millis();
        }
    }

    return output;
}

void write_big_endian(uint8_t *dst, uint8_t *src, size_t len) {
    for (size_t i = 0; i < len; i++) {
        dst[i] = src[len - 1 - i];
    }
}

  