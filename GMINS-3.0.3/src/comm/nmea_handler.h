#pragma once

#include "../data/data_types.h"
#include <stdint.h>

enum class NMEAMessageType {
    UNKNOWN,
    GGA,
    RMC,
    GSV,
    GSA,
    VTG,
    PLSHD  // Custom GNSS heading message
};

class NMEAHandler {
public:
    NMEAHandler();
    ~NMEAHandler();
    
    bool initialize();
    
    size_t processIncomingData(const uint8_t* buffer, size_t length);
    bool parseGNSSData(const char* nmea_sentence, GNSSData& gnss_data);
    
    struct NMEAStats {
        uint32_t sentences_processed;
        uint32_t parse_errors;
        uint32_t checksum_errors;
        uint32_t gga_count;
        uint32_t rmc_count;
        uint32_t gsv_count;
        uint32_t plshd_count;
    };
    
    NMEAStats getStats() const { return stats_; }
    
    void setGNSSCallback(void (*callback)(const GNSSData&));
    
private:
    char receive_buffer_[512];
    size_t buffer_index_;
    bool sentence_in_progress_;
    
    NMEAStats stats_;
    void (*gnss_callback_)(const GNSSData&);
    
    NMEAMessageType identifyMessageType(const char* sentence);
    bool validateChecksum(const char* sentence);
    uint8_t calculateChecksum(const char* sentence);
    
    bool parseGGA(const char* sentence, GNSSData& gnss_data);
    bool parseRMC(const char* sentence, GNSSData& gnss_data);
    bool parseGSV(const char* sentence, GNSSData& gnss_data);
    bool parsePLSHD(const char* sentence, GNSSData& gnss_data);
    
    double parseLatLon(const char* str, char dir);
    float parseFloat(const char* str);
    int parseInt(const char* str);
    bool parseTime(const char* time_str, uint32_t& timestamp);
};