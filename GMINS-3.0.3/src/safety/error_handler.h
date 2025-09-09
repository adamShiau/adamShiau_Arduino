#pragma once

#include <stdint.h>

enum class ErrorSeverity {
    INFO,
    WARNING,
    ERROR,
    CRITICAL,
    FATAL
};

enum class ErrorCategory {
    SYSTEM,
    COMMUNICATION,
    SENSOR,
    FUSION,
    SAFETY,
    CONFIGURATION,
    RESOURCE
};

struct ErrorEntry {
    uint32_t timestamp;
    ErrorSeverity severity;
    ErrorCategory category;
    uint32_t error_code;
    char message[128];
    uint32_t occurrence_count;
};

struct ErrorHandlerConfig {
    uint32_t max_error_log_size;
    bool enable_auto_recovery;
    uint32_t recovery_timeout_ms;
    uint32_t max_recovery_attempts;
    bool enable_error_reporting;
};

class ErrorHandler {
public:
    ErrorHandler();
    ~ErrorHandler();
    
    bool initialize(const ErrorHandlerConfig& config);
    
    void reportError(ErrorSeverity severity, ErrorCategory category, 
                    uint32_t error_code, const char* message);
    
    bool handleError(uint32_t error_code);
    void clearErrors(ErrorCategory category = ErrorCategory::SYSTEM);
    
    uint32_t getErrorCount(ErrorSeverity min_severity = ErrorSeverity::WARNING) const;
    uint32_t getErrorCount(ErrorCategory category) const;
    
    const ErrorEntry* getLastError() const;
    const ErrorEntry* getErrorHistory(uint32_t index) const;
    uint32_t getHistorySize() const { return error_count_; }
    
    bool isInRecoveryMode() const { return recovery_in_progress_; }
    uint32_t getRecoveryAttempts() const { return recovery_attempts_; }
    
    struct ErrorStats {
        uint32_t total_errors;
        uint32_t critical_errors;
        uint32_t warnings;
        uint32_t recovery_attempts;
        uint32_t successful_recoveries;
        uint32_t last_error_time;
    };
    
    ErrorStats getStats() const;
    
private:
    ErrorHandlerConfig config_;
    ErrorEntry* error_log_;
    uint32_t error_count_;
    uint32_t log_write_index_;
    
    bool recovery_in_progress_;
    uint32_t recovery_attempts_;
    uint32_t recovery_start_time_;
    
    // Recovery strategies
    bool attemptCommunicationRecovery(uint32_t error_code);
    bool attemptSensorRecovery(uint32_t error_code);
    bool attemptFusionRecovery(uint32_t error_code);
    bool attemptSystemRecovery(uint32_t error_code);
    
    void logError(const ErrorEntry& entry);
    bool shouldAttemptRecovery(ErrorSeverity severity, ErrorCategory category);
    void updateRecoveryStatus(bool success);
};