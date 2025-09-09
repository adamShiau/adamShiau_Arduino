#pragma once
// nav_publish_gate.h - Navigation 發佈守門員（解決多入口重複喚起）
// 特色：
// 1) 單一出入口：集中處理 IMU / GNSS / tick 多路觸發，避免回調重複呼叫
// 2) 策略可切換：預設 IMU-driven（GNSS 合流、不單獨觸發）；可改同步或固定頻率
// 3) 穩定節拍：min interval（抖動/上限）+ max latency（IMU 斷流保底）
// 4) 低成本：無動態配置、O(1)、適合 Arduino/MCU
//
// 用法摘要：
//   - 在 DFI 內建一個 NavPublishGate，綁定真正的 publish 回調（通常是 pm.sendNavigation(nav)）
//   - 把原先直接呼叫 nav_callback(nav) 改為：
//       gate.onImuUpdate(nav) / gate.onGnssUpdate(nav) / gate.onTick(nav)
//   - 如此一來，不論 updateIMU / updateGNSS / tick 呼叫次數多少，最後都由 gate 決定是否真的發佈一次
//
// 依賴：需要 Arduino.h 的 micros()。NavigationState 由你的專案提供。

#include <Arduino.h>

struct NavigationState;  // 前向宣告；實體結構由你的專案提供

// ====== 編譯期參數（可在編譯器或 config 標頭覆寫） ======
#ifndef NAVPG_POLICY_IMU_DRIVEN
// 1 = IMU 驅動（預設）：IMU 來才主動發佈；GNSS 只合流不單獨觸發；tick 作保底
// 0 = 其他策略（預留），例如同步/固定頻率（可在下方分支擴充）
#define NAVPG_POLICY_IMU_DRIVEN 1
#endif

#ifndef NAVPG_MIN_PUB_INTERVAL_US
// 兩次發佈的最小間隔（抖動上限）；想要 100 Hz ⇒ 設 10000
#define NAVPG_MIN_PUB_INTERVAL_US 10000UL
#endif

#ifndef NAVPG_MAX_LATENCY_US
// 最長延遲保險（IMU 斷流時，tick 可補發一次）
#define NAVPG_MAX_LATENCY_US 30000UL
#endif

#ifndef NAVPG_REQUIRE_GNSS_SINCE_LAST_PUB
// 1 = 要求自上次發佈以來 GNSS 也更新過才允許發佈（會把輸出頻率拉向 GNSS 頻率）
// 0 = 不強制；只要 IMU 有新樣本且達到最小間隔就可發佈（預設）
#define NAVPG_REQUIRE_GNSS_SINCE_LAST_PUB 0
#endif

#ifndef NAVPG_LOG_EVERY_MS
// 統計摘要列印間隔（0 = 不列印）
#define NAVPG_LOG_EVERY_MS 5000UL
#endif

// ====== 可選：態勢有效性的輕量檢查 ======
// 你的 NavigationState 內若已由 DFI 保證有效，可將此宏設為 0 跳過再檢查
#ifndef NAVPG_LIGHT_ATTITUDE_CHECK
#define NAVPG_LIGHT_ATTITUDE_CHECK 0  // DFI 已保證有效，跳過檢查
#endif

class NavPublishGate {
public:
  typedef void (*PublishFn)(const NavigationState&);

  enum class Source : uint8_t { IMU, GNSS, TICK };

  NavPublishGate()
  : publish_fn_(nullptr),
    imu_seq_(0), gnss_seq_(0),
    last_pub_imu_seq_(0), last_seen_gnss_seq_(0),
    last_pub_us_(0),
    in_publish_(false),
    stats_{}
  {}

  void bindPublishFn(PublishFn fn) { publish_fn_ = fn; }

  // 可選：動態覆寫參數
  void setMinIntervalUs(uint32_t us) { min_interval_us_ = us; }
  void setMaxLatencyUs(uint32_t us)  { max_latency_us_  = us; }

  // 三個入口：用 DFI 的 nav 快照呼叫（取代原本直接觸發 callback）
  inline void onImuUpdate(const NavigationState& nav)  { ++imu_seq_; maybePublish_(Source::IMU,  nav); }
  inline void onGnssUpdate(const NavigationState& nav) { ++gnss_seq_; maybePublish_(Source::GNSS, nav); }
  inline void onTick(const NavigationState& nav)       { maybePublish_(Source::TICK, nav); }

  // 每回合（例如主迴圈尾端）可呼叫一次，用於列印統計
  void tickStats() {
#if NAVPG_LOG_EVERY_MS
    const uint32_t now_ms = millis();
    if (now_ms - stats_.last_log_ms >= (uint32_t)NAVPG_LOG_EVERY_MS) {
      stats_.last_log_ms = now_ms;
      logStats_();
      // 週期重置
      stats_.pub = stats_.skip_same_imu = stats_.skip_min_interval = stats_.tick_rescue = 0;
    }
#endif
  }

private:
  // === gating 主邏輯 ===
  inline void maybePublish_(Source src, const NavigationState& nav) {
    const uint32_t now = micros();

#if NAVPG_POLICY_IMU_DRIVEN
    if (src == Source::IMU) {
      const bool new_imu = (imu_seq_ != last_pub_imu_seq_);
      const bool over_min = (now - last_pub_us_) >= min_interval_us_;
#if NAVPG_REQUIRE_GNSS_SINCE_LAST_PUB
      const bool gnss_ok = (gnss_seq_ != last_seen_gnss_seq_);
#else
      const bool gnss_ok = true;
#endif
      if (!new_imu) { stats_.skip_same_imu++; return; }
      if (!over_min) { stats_.skip_min_interval++; return; }
      if (!gnss_ok)  { stats_.skip_need_gnss++; return; }
      publishOnce_(nav, now);
      return;
    }
    if (src == Source::TICK) {
      const bool too_long = (now - last_pub_us_) >= max_latency_us_;
      if (too_long) { publishOnce_(nav, now); stats_.tick_rescue++; }
      return;
    }
    // Source::GNSS：IMU 驅動策略下不單獨觸發；僅合流
    (void)nav;
    return;
#else
    // 這裡可擴充其他策略（例如 IMU+GNSS 同步/固定頻率等）
    (void)src; (void)nav; (void)now;
    return;
#endif
  }

  inline void publishOnce_(const NavigationState& nav, uint32_t now_us) {
    if (!publish_fn_) return;
    if (in_publish_) return;      // reentrancy guard
#if NAVPG_LIGHT_ATTITUDE_CHECK
    // 輕量：要求態勢有效 + 時戳非零（若 DFI 已保證，可關此檢查）
    if (!attitudeValid_(nav)) return;
#endif
    in_publish_ = true;
    publish_fn_(nav);
    in_publish_ = false;

    last_pub_us_ = now_us;
    last_pub_imu_seq_ = imu_seq_;
    last_seen_gnss_seq_ = gnss_seq_;

    stats_.pub++;
  }

  inline bool attitudeValid_(const NavigationState& nav) const {
    // DFI 已保證有效，直接回 true
    (void)nav;
    return true;
  }

  // === 統計 & 列印（使用專案的 LOG 宏） ===
  void logStats_() {
#if defined(LOGI)
    LOGI("[NavPublishGate] pub=%u skip_same_imu=%u skip_min_interval=%u skip_need_gnss=%u tick_rescue=%u", 
         stats_.pub, stats_.skip_same_imu, stats_.skip_min_interval,
#if NAVPG_REQUIRE_GNSS_SINCE_LAST_PUB
         stats_.skip_need_gnss,
#else
         0u,
#endif
         stats_.tick_rescue);
#endif
  }

private:
  PublishFn publish_fn_;

  // 來源序號（只遞增，不回捲）
  uint32_t imu_seq_;
  uint32_t gnss_seq_;

  // 上次發佈時記錄
  uint32_t last_pub_imu_seq_;
  uint32_t last_seen_gnss_seq_;
  uint32_t last_pub_us_;

  bool in_publish_;

  uint32_t min_interval_us_ = NAVPG_MIN_PUB_INTERVAL_US;
  uint32_t max_latency_us_  = NAVPG_MAX_LATENCY_US;

  struct {
    uint32_t pub = 0;
    uint32_t skip_same_imu = 0;
    uint32_t skip_min_interval = 0;
    uint32_t skip_need_gnss = 0;  // 只有在 REQUIRE_GNSS=1 時會用到
    uint32_t tick_rescue = 0;
    uint32_t last_log_ms = 0;
  } stats_;
};