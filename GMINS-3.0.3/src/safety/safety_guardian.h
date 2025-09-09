#pragma once

#include "../data/data_types.h"
#include "../core/system_health.h"
#include "error_handler.h"

enum class SafetyLevel {
    NORMAL,
    CAUTION,
    WARNING,
    CRITICAL,
    EMERGENCY
};

enum class FaultType {
    NONE,
    COMMUNICATION_TIMEOUT,
    DATA_CORRUPTION,
    SENSOR_FAILURE,
    FUSION_DIVERGENCE,
    RESOURCE_EXHAUSTION,
    TIMING_VIOLATION,
    CONFIGURATION_ERROR
};

struct SafetyCheck {
    const char* name;
    bool (*check_function)(const SystemHealthStats&, const NavigationState&);
    SafetyLevel severity;
    uint32_t check_interval_ms;
    uint32_t last_check_time;
    bool is_enabled;
};

struct SafetyConfig {
    // Timing constraints
    uint32_t max_loop_time_ms;
    uint32_t max_sensor_timeout_ms;
    uint32_t max_fusion_lag_ms;
    
    // Resource limits
    uint32_t min_free_memory;
    float max_cpu_usage;
    
    // Data validation
    float max_position_jump;
    float max_velocity_change;
    float max_acceleration_magnitude;
    float max_angular_velocity;
    
    // System limits
    uint32_t max_error_count;
    uint32_t recovery_timeout_ms;
    bool enable_automatic_recovery;
};

class SafetyGuardian {
public:
    SafetyGuardian();
    ~SafetyGuardian();
    
    bool initialize(const SafetyConfig& config);
    
    SafetyLevel checkSystemSafety(const SystemHealthStats& health_stats, 
                                const NavigationState& nav_state);
    
    bool attemptRecovery(FaultType fault);
    void triggerSafeMode(const char* reason);
    
    FaultType getLastFault() const { return last_fault_; }
    SafetyLevel getCurrentLevel() const { return current_level_; }
    
    void updateConfig(const SafetyConfig& config);
    SafetyConfig getConfig() const { return config_; }
    
    struct SafetyStats {
        uint32_t checks_performed;
        uint32_t faults_detected;
        uint32_t recoveries_attempted;
        uint32_t recoveries_successful;
        uint32_t safe_mode_triggers;
        SafetyLevel max_level_reached;
    };
    
    SafetyStats getStats() const { return stats_; }
    
private:
    SafetyConfig config_;
    SafetyLevel current_level_;
    FaultType last_fault_;
    SafetyStats stats_;
    
    ErrorHandler error_handler_;
    
    // Safety check functions
    static bool checkCommunicationHealth(const SystemHealthStats& health, const NavigationState& nav);
    static bool checkDataIntegrity(const SystemHealthStats& health, const NavigationState& nav);
    static bool checkResourceUsage(const SystemHealthStats& health, const NavigationState& nav);
    static bool checkTimingConstraints(const SystemHealthStats& health, const NavigationState& nav);
    static bool checkSensorHealth(const SystemHealthStats& health, const NavigationState& nav);
    static bool checkFusionPerformance(const SystemHealthStats& health, const NavigationState& nav);
    
    SafetyCheck safety_checks_[6];
    
    SafetyLevel evaluateOverallSafety();
    void performScheduledChecks(const SystemHealthStats& health, const NavigationState& nav);
    void logSafetyEvent(SafetyLevel level, const char* message);
};