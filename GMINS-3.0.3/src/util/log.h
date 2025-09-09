#pragma once

// ===== 平台輸出 =====
#ifdef ARDUINO
  #include <Arduino.h>
  #include <Print.h>
  #include <stdarg.h>
  #include <stdio.h>

  // 預設輸出埠（可在包含前用 #define LOG_DEFAULT_PORT Serial1 覆寫）
  #ifndef LOG_DEFAULT_PORT
    #define LOG_DEFAULT_PORT Serial
  #endif

  // 可在執行期切換輸出埠：LOG_SET_PORT(Serial1);
  inline Print*& _log_port(){ static Print* p = &LOG_DEFAULT_PORT; return p; }
  inline void LOG_SET_PORT(Print& p){ _log_port() = &p; }

  // Arduino 平台：用 vsnprintf + Print 輸出（注意：部分 AVR 不支援 %f）
  namespace LogUtils {
    inline void logPrintF(const char* format, ...) {
      char buffer[256];
      va_list args;
      va_start(args, format);
      vsnprintf(buffer, sizeof(buffer), format, args);
      va_end(args);
      _log_port()->print(buffer);
    }
  }
  #define LOG_OUT(...)   LogUtils::logPrintF(__VA_ARGS__)
  #define LOG_NEWLINE()  _log_port()->println()
  #ifndef LOG_TIME_MS
    #define LOG_TIME_MS() millis()
  #endif

#else
  #include <stdio.h>
  #include <stdint.h>
  #include <time.h>
  #include <stdarg.h>
  inline void _host_printf(const char* fmt, ...){
    va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
  }
  #define LOG_OUT(...)   _host_printf(__VA_ARGS__)
  #define LOG_NEWLINE()  printf("\n")
  #ifndef LOG_TIME_MS
    #define LOG_TIME_MS() ((uint32_t)clock())
  #endif
#endif

// ===== 等級與來源 =====
enum LogLevel { LOG_DEBUG=0, LOG_INFO=1, LOG_WARN=2, LOG_ERROR=3, LOG_CRITICAL=4, LOG_NONE=99 };

#ifndef LOG_LEVEL_GLOBAL
  #define LOG_LEVEL_GLOBAL LOG_NONE          // 暫時關閉所有LOG避免死機
#endif
#ifdef LOG_LEVEL_LOCAL
  #define LOG_LEVEL_EFFECTIVE LOG_LEVEL_LOCAL
#else
  #define LOG_LEVEL_EFFECTIVE LOG_LEVEL_GLOBAL
#endif

#ifndef LOG_TAG
  #ifdef __FILE_NAME__
    #define LOG_TAG __FILE_NAME__
  #else
    #define LOG_TAG __FILE__
  #endif
#endif

// ===== 等級字串 =====
inline const char* logLevelStr(LogLevel lv){
  switch(lv){case LOG_DEBUG:return "D";case LOG_INFO:return "I";case LOG_WARN:return "W";
  case LOG_ERROR:return "E";case LOG_CRITICAL:return "C";default:return "-";}
}

// ===== 英文字母遮罩（精準挑 I/D/W/E/C 要不要顯示） =====
// 每個等級對應一個 bit
enum LogBit : uint32_t {
  LOG_BIT_DEBUG    = 1u << 0,
  LOG_BIT_INFO     = 1u << 1,
  LOG_BIT_WARN     = 1u << 2,
  LOG_BIT_ERROR    = 1u << 3,
  LOG_BIT_CRITICAL = 1u << 4,
};

// 全域預設遮罩（不含 DEBUG）：可在 log_config.h 或編譯旗標覆寫
#ifndef LOG_LEVEL_MASK_GLOBAL
  #define LOG_LEVEL_MASK_GLOBAL (LOG_BIT_INFO | LOG_BIT_WARN | LOG_BIT_ERROR | LOG_BIT_CRITICAL)
#endif
// 檔案在地遮罩：如未定義則用全域預設
#ifndef LOG_LEVEL_MASK_LOCAL
  #define LOG_LEVEL_MASK_LOCAL LOG_LEVEL_MASK_GLOBAL
#endif
#define LOG_LEVEL_MASK_EFFECTIVE LOG_LEVEL_MASK_LOCAL

// 把 LogLevel 轉 bit
#define _LEVEL_TO_BIT(lv) \
  ((lv)==LOG_DEBUG?LOG_BIT_DEBUG: \
  (lv)==LOG_INFO?LOG_BIT_INFO: \
  (lv)==LOG_WARN?LOG_BIT_WARN: \
  (lv)==LOG_ERROR?LOG_BIT_ERROR: LOG_BIT_CRITICAL)

// ===== 內核：編譯期 + 執行期雙重過濾 =====
#ifndef LOG_MIN_RUNTIME_LEVEL
  #define LOG_MIN_RUNTIME_LEVEL LOG_DEBUG   // 需要執行期關閉再改這個（或改成變數）
#endif

// 編譯期過濾：同時滿足「等級門檻」與「字母遮罩」
#define _LOG_COMPILED(level) ( \
  ((int)(level) >= (int)LOG_LEVEL_EFFECTIVE) && \
  ((LOG_LEVEL_MASK_EFFECTIVE & _LEVEL_TO_BIT((LogLevel)(level))) != 0) )

// 執行期門檻（不要就把條件改成 1）
#define _LOG_RUNTIME(level)  ((int)(level) >= (int)LOG_MIN_RUNTIME_LEVEL)

#define _LOG_PRINT(level, fmt, ...) \
  do{ if(_LOG_COMPILED(level) && _LOG_RUNTIME(level)){ \
    LOG_OUT("[%s][%s] " fmt, logLevelStr((LogLevel)level), LOG_TAG, ##__VA_ARGS__); LOG_NEWLINE(); \
  }}while(0)

// 快速等級宏
#define LOGD(fmt, ...) _LOG_PRINT(LOG_DEBUG, fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) _LOG_PRINT(LOG_INFO,  fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) _LOG_PRINT(LOG_WARN,  fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) _LOG_PRINT(LOG_ERROR, fmt, ##__VA_ARGS__)
#define LOGC(fmt, ...) _LOG_PRINT(LOG_CRITICAL, fmt, ##__VA_ARGS__)

// 兼容舊代碼的函式式巨集
#define LOG_DEBUG(fmt, ...) _LOG_PRINT(LOG_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)  _LOG_PRINT(LOG_INFO,  fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  _LOG_PRINT(LOG_WARN,  fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) _LOG_PRINT(LOG_ERROR, fmt, ##__VA_ARGS__)
#define LOG_ERR(fmt, ...)   _LOG_PRINT(LOG_ERROR, fmt, ##__VA_ARGS__)

// ===== TDD 測試驅動開發控制 =====
// 獨立的TDD調試輸出控制，不受LOG_LEVEL影響
#ifndef TDD_ENABLE
  #define TDD_ENABLE 1  // 重新啟用TDD輸出
#endif

#if TDD_ENABLE
  #define TDD_PRINT(fmt, ...) do{ LOG_OUT("[TDD] " fmt, ##__VA_ARGS__); LOG_NEWLINE(); }while(0)
  #define TDD_FREQ(name, interval_ms) \
    do{ static uint32_t __tdd_last=0; static uint16_t __tdd_count=0; \
        uint32_t __tdd_now=LOG_TIME_MS(); \
        if(__tdd_now < 40000) break; \
        __tdd_count++; \
        if(__tdd_now - __tdd_last >= (uint32_t)(interval_ms)){ \
          uint32_t __tdd_delta = __tdd_now - __tdd_last; \
          uint32_t __tdd_hz_x10 = (__tdd_count * 10000UL) / __tdd_delta; \
          TDD_PRINT("%s: %u.%u Hz", name, __tdd_hz_x10/10, __tdd_hz_x10%10); \
          __tdd_last=__tdd_now; __tdd_count=0; } \
    }while(0)
#else
  #define TDD_PRINT(fmt, ...) do{}while(0)
  #define TDD_FREQ(name, interval_ms) do{}while(0)
#endif

// ===== 工具：率限 / 心跳 / HEX =====
#define LOG_EVERY_MS(level, interval_ms, fmt, ...) \
  do{ static uint32_t __last=0; uint32_t __now=LOG_TIME_MS(); \
      if(__now-__last >= (uint32_t)(interval_ms)){ __last=__now; \
        _LOG_PRINT(level, fmt, ##__VA_ARGS__); } \
  }while(0)

// 心跳：預設走 LOG_INFO（受門檻/遮罩影響）
// 若你想「只要心跳、其他全關」，在某檔案先 #define HEARTBEAT_BYPASS_FILTER
#ifdef HEARTBEAT_ENABLE
  #ifdef HEARTBEAT_BYPASS_FILTER
    #define HEARTBEAT(period_ms) \
      do{ static uint32_t __hb_last=0; uint32_t __hb_now=LOG_TIME_MS(); \
          if(__hb_now-__hb_last >= (uint32_t)(period_ms)){ __hb_last=__hb_now; \
            LOG_OUT("[I][%s] %s", LOG_TAG, "\xE2\x99\xA5 heartbeat"); LOG_NEWLINE(); } \
      }while(0)
  #else
    #define HEARTBEAT(period_ms) LOG_EVERY_MS(LOG_INFO, period_ms, "\xE2\x99\xA5 heartbeat")
  #endif
#else
  #define HEARTBEAT(period_ms) do{}while(0)
#endif

#define LOG_HEX(level, data, len) \
  do{ if(_LOG_COMPILED(level) && _LOG_RUNTIME(level)){ \
      const uint8_t* _p=(const uint8_t*)(data); size_t _n=(size_t)(len); \
      for(size_t i=0;i<_n;i++){ if((i&0x0F)==0){ LOG_OUT("[%s][%s] %04u: ", \
        logLevelStr((LogLevel)level), LOG_TAG, (unsigned)i); } \
        LOG_OUT("%02X ", _p[i]); if((i&0x0F)==0x0F || i==_n-1) LOG_NEWLINE(); } \
  }}while(0)

// 原味輸出（不加前綴；走相同輸出埠）
#define LOG_RAW(fmt, ...) do{ LOG_OUT(fmt, ##__VA_ARGS__); LOG_NEWLINE(); }while(0)

// ===== 量測（scope 計時）=====
struct LogScopeTimer {
  const char* name; uint32_t t0;
  explicit LogScopeTimer(const char* n):name(n),t0(LOG_TIME_MS()){}
  ~LogScopeTimer(){ _LOG_PRINT(LOG_DEBUG, "[TIMER] %s: %lu ms", name, (unsigned long)(LOG_TIME_MS()-t0)); }
};
#define SCOPED_TIMER(name_literal) LogScopeTimer __timer__(name_literal)
