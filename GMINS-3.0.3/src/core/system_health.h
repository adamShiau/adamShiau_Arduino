#pragma once

#include <stdint.h>

enum class HealthStatus {
    UNKNOWN,
    HEALTHY,
    WARNING,
    ERROR,
    CRITICAL
};

struct ModuleHealth {
    HealthStatus status;
    uint32_t last_update_time;
    uint32_t error_count;
    char last_error[64];
};

struct SystemHealthStats {
    ModuleHealth communication;
    ModuleHealth fusion;
    ModuleHealth safety;
    ModuleHealth gnss;
    ModuleHealth imu;
    
    uint32_t total_runtime;
    uint32_t last_heartbeat;
    float cpu_usage;
    uint32_t free_memory;
};

// 協議需要的簡化版本
struct SystemHealth {
    bool all_systems_ok;
    float cpu_temperature;
    uint32_t timestamp_us;
    uint8_t error_flags;
    
    // 兼容性字段（用於舊代碼）
    bool communication_ok;
    bool fusion_ok;
    bool safety_ok;
    bool protocol_ok;
    bool dfi_ok;
    uint32_t error_count;
    uint32_t warning_count;
    
    SystemHealth() : all_systems_ok(true), cpu_temperature(25.0f), 
                     timestamp_us(0), error_flags(0),
                     communication_ok(true), fusion_ok(true), safety_ok(true),
                     protocol_ok(true), dfi_ok(true), error_count(0), warning_count(0) {}
};

class SystemHealthMonitor {
public:
    SystemHealthMonitor();
    
    void updateModuleHealth(const char* module, HealthStatus status, const char* message = nullptr);
    HealthStatus getOverallHealth() const;
    SystemHealthStats getStats() const { return stats_; }
    
    bool isModuleHealthy(const char* module) const;
    void resetErrorCounts();
    
private:
    SystemHealthStats stats_;
    
    ModuleHealth* getModuleHealth(const char* module);
    void updateOverallStatus();
};