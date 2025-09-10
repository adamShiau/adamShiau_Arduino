#pragma once
#include <Arduino.h>

/**
 * 簡單的四元數 + 歐拉角結構
 */
struct Quatf { float w, x, y, z; };
struct RPYdeg { float roll, pitch, yaw; }; // 以度為單位

/**
 * Madgwick IMU-only 濾波器（含自適應 beta）
 * - 僅用陀螺儀 + 加速度（無磁力）
 * - 適合 Arduino Nano 33 IoT（SAMD21）
 * - 無動態記憶體、使用 float
 */
class MadgwickIMUAdaptive {
public:
  // ===== 可調參數（預設適合 ~100 Hz） =====
  float beta_base      = 0.033f;   // 基準修正增益（原作者常用）
  float beta_min_ratio = 0.25f;    // β 最低比例（避免完全關掉修正）
  float beta_max_ratio = 1.00f;    // β 最高比例
  float g0             = 9.80665f; // 重力常數 [m/s^2]
  float acc_tol_g      = 0.15f;    // |‖a‖-1g| 相對 1g 的容忍（±15%）
  float tilt_soft      = 0.25f;    // |vz| >= 此值 → 完全信任 acc
  float tilt_hard      = 0.05f;    // |vz| <= 此值 → 幾乎垂直，關閉 acc 修正
  float alpha_lp       = 0.20f;    // β 的低通係數（0~1；越大越跟隨目標）
  bool  accelMeasuresDown = false; // ★你的情況：Z↑ 靜止時 az≈+9.8 → 設 false（加速度量的是「向上」）

  // ===== 狀態 =====
  Quatf q;                 // 當前姿態四元數
  uint32_t t_prev_us;      // 上一次更新的 micros()
  float beta_eff_lp;       // 低通後的實際 β

  // ===== 生命週期 =====
  MadgwickIMUAdaptive();                        // 建構（預設 q=單位四元數）
  void reset(const Quatf& q0 = {1,0,0,0});      // 重置狀態（清空時間戳、β 回到基準）

  /**
   * 用一次加速度取樣快速初始化姿態：
   * - roll/pitch 由加速度（重力）決定
   * - yaw 設為 yaw0_deg（通常 0）
   * - Start 當下呼叫即可讓 R/P 對齊當前姿態
   */
  void initFromAccel(float ax_ms2, float ay_ms2, float az_ms2, float yaw0_deg = 0.0f);

  /**
   * 主要更新函式
   * @param gx_dps,gy_dps,gz_dps  陀螺儀（度/秒）
   * @param ax_ms2,ay_ms2,az_ms2  加速度（m/s^2）
   * @param t_now_us              當下 micros()
   * @param rpy_out               若不為 nullptr，輸出 roll/pitch/yaw（度）
   * @return                      更新後四元數
   */
  Quatf update(float gx_dps, float gy_dps, float gz_dps,
               float ax_ms2, float ay_ms2, float az_ms2,
               uint32_t t_now_us,
               RPYdeg* rpy_out = nullptr);

private:
  // 自適應 β：依 |‖a‖−1g| 與 |vz| 調整修正力
  float adaptiveBeta(float ax_ms2, float ay_ms2, float az_ms2);

  // 工具：四元數轉 ZYX 歐拉角（度）
  static RPYdeg quatToEulerZYXdeg(const Quatf& q);

  // 工具：四元數正規化
  static Quatf  quatNormalize(const Quatf& q);
};
