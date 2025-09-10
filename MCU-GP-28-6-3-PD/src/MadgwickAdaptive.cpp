#include "MadgwickAdaptive.h"
#include <math.h>

// === 內部小工具（不暴露） ===
namespace {
  // 夾限
  inline float clampf(float x, float lo, float hi) { return (x<lo)?lo:((x>hi)?hi:x); }

  // 快速 1/sqrt(x)（這裡直接用標準 sqrtf 也行）
  inline float invSqrtf(float x) { return 1.0f / sqrtf(x); }

  // 3D 向量正規化（若全零則回零）
  inline void normalize3(float& x, float& y, float& z) {
    const float n2 = x*x + y*y + z*z;
    if (n2 <= 0.0f) { x=y=z=0.0f; return; }
    const float invn = invSqrtf(n2);
    x*=invn; y*=invn; z*=invn;
  }
}

// ===== 建構 / 重置 =====
MadgwickIMUAdaptive::MadgwickIMUAdaptive() {
  q = {1.0f,0.0f,0.0f,0.0f};  // 單位四元數
  t_prev_us = 0;
  beta_eff_lp = beta_base;
}

void MadgwickIMUAdaptive::reset(const Quatf& q0) {
  q = q0;
  t_prev_us = 0;
  beta_eff_lp = beta_base;
}

// ===== Start 當下用加速度快速初始化（R/P 對齊，Yaw 指定）=====
void MadgwickIMUAdaptive::initFromAccel(float ax_ms2, float ay_ms2, float az_ms2, float yaw0_deg) {
  // 先把加速度正規化成方向（IMU 量到的是「向上」的比力；若要代表「向下」，請視 accelMeasuresDown 取反）
  float ax=ax_ms2, ay=ay_ms2, az=az_ms2;
  float n = sqrtf(ax*ax + ay*ay + az*az);
  if (n>0.0f) { ax/=n; ay/=n; az/=n; }
  if (!accelMeasuresDown) { ax=-ax; ay=-ay; az=-az; }  // 轉成「向下」

  // 由重力估計 pitch/roll（ZYX 定義；如有座標差異可在此調整符號/次序）
  float pitch = asinf(-ax);     // -ax 對應「向下」的 X 分量
  float roll  = atan2f(ay, az); // Y/Z 比例

  // yaw 指定為 yaw0_deg
  const float yaw  = yaw0_deg * (PI/180.0f);

  // 四元數合成：q = Rz(yaw) * Ry(pitch) * Rx(roll)
  const float cy = cosf(yaw*0.5f),  sy = sinf(yaw*0.5f);
  const float cp = cosf(pitch*0.5f),sp = sinf(pitch*0.5f);
  const float cr = cosf(roll*0.5f), sr = sinf(roll*0.5f);

  q.w = cr*cp*cy + sr*sp*sy;
  q.x = sr*cp*cy - cr*sp*sy;
  q.y = cr*sp*cy + sr*cp*sy;
  q.z = cr*cp*sy - sr*sp*cy;
  q = quatNormalize(q);

  // 初始化完也把時間戳清零，下一筆 update 重新計 dt
  t_prev_us = 0;
}

// ===== 工具：四元數 → ZYX 歐拉角（度）=====
RPYdeg MadgwickIMUAdaptive::quatToEulerZYXdeg(const Quatf& q) {
  const float w=q.w,x=q.x,y=q.y,z=q.z;

  // yaw (Z)
  const float siny_cosp = 2.0f*(w*z + x*y);
  const float cosy_cosp = 1.0f - 2.0f*(y*y + z*z);
  float yaw = atan2f(siny_cosp, cosy_cosp);

  // pitch (Y)
  float sinp = 2.0f*(w*y - z*x);
  float pitch;
  if      (sinp >=  1.0f) pitch =  PI*0.5f;   // 鉗制避免超域
  else if (sinp <= -1.0f) pitch = -PI*0.5f;
  else                    pitch = asinf(sinp);

  // roll (X)
  const float sinr_cosp = 2.0f*(w*x + y*z);
  const float cosr_cosp = 1.0f - 2.0f*(x*x + y*y);
  float roll = atan2f(sinr_cosp, cosr_cosp);

  const float RAD2DEG = 180.0f/PI;
  return { roll*RAD2DEG, pitch*RAD2DEG, yaw*RAD2DEG };
}

// ===== 工具：四元數正規化 =====
Quatf MadgwickIMUAdaptive::quatNormalize(const Quatf& qi) {
  const float n2 = qi.w*qi.w + qi.x*qi.x + qi.y*qi.y + qi.z*qi.z;
  if (n2<=0.0f) return {1,0,0,0};
  const float invn = invSqrtf(n2);
  return { qi.w*invn, qi.x*invn, qi.y*invn, qi.z*invn };
}

// ===== 自適應 β（依加速度大小與傾角退化）=====
float MadgwickIMUAdaptive::adaptiveBeta(float ax_ms2, float ay_ms2, float az_ms2) {
  // 1) 動態加速度因子：‖a‖ 偏離 1g 越多 → 信任 acc 越低（β 越小）
  const float anorm = sqrtf(ax_ms2*ax_ms2 + ay_ms2*ay_ms2 + az_ms2*az_ms2);
  const float dev_g = fabsf(anorm - g0) / (g0 * acc_tol_g);
  const float k_acc = 1.0f / (1.0f + dev_g); // 0..1

  // 2) 傾角退化因子：估計重力在 body-Z 的投影 |vz| 越小（接近 90°）→ β 越小
  const float x=q.x, y=q.y;
  float vz = 1.0f - 2.0f*(x*x + y*y);
  vz = fabsf(vz);

  float k_tilt;
  if      (vz >= tilt_soft) k_tilt = 1.0f;                               // 完全信任
  else if (vz <= tilt_hard) k_tilt = 0.0f;                               // 幾乎垂直，關閉 acc 修正
  else                      k_tilt = (vz - tilt_hard) / (tilt_soft - tilt_hard); // 線性過渡

  // 3) 合成並夾限
  float k = clampf(k_acc * k_tilt, beta_min_ratio, beta_max_ratio);
  const float beta_target = beta_base * k;

  // 4) 低通，避免 β 抖動
  beta_eff_lp = (1.0f - alpha_lp)*beta_eff_lp + alpha_lp*beta_target;
  return beta_eff_lp;
}

// ===== 主更新（gyro 積分 + 梯度修正）=====
Quatf MadgwickIMUAdaptive::update(float gx_dps, float gy_dps, float gz_dps,
                                  float ax_ms2, float ay_ms2, float az_ms2,
                                  uint32_t t_now_us,
                                  RPYdeg* rpy_out) {
  // --- 計算 dt（含防呆夾限，避免暫停後一步走太大） ---
  if (t_prev_us==0 || t_now_us<=t_prev_us) {
    t_prev_us = t_now_us;
    if (rpy_out) *rpy_out = quatToEulerZYXdeg(q);
    return q;
  }
  float dt = (t_now_us - t_prev_us) * 1e-6f;
  t_prev_us = t_now_us;
  if (dt <= 0.0f) dt = 1e-3f;     // 最小 1ms
  if (dt >  0.05f) dt = 0.05f;    // 最大 50ms（避免一次走太大）

  // --- 單位處理：dps → rad/s ---
  const float d2r = PI/180.0f;
  const float gx = gx_dps*d2r, gy = gy_dps*d2r, gz = gz_dps*d2r;

  // --- 加速度方向（normalize）---
  float axu=ax_ms2, ayu=ay_ms2, azu=az_ms2;
  normalize3(axu, ayu, azu);
  if (!accelMeasuresDown) { axu=-axu; ayu=-ayu; azu=-azu; }  // 轉成「向下」

  // --- 自適應 β ---
  const float beta_eff = adaptiveBeta(ax_ms2, ay_ms2, az_ms2);

  // --- 由 gyro 推進（四元數微分）---
  float qw=q.w, qx=q.x, qy=q.y, qz=q.z;
  float qw_dot = 0.5f * (-qx*gx - qy*gy - qz*gz);
  float qx_dot = 0.5f * ( qw*gx + qy*gz - qz*gy);
  float qy_dot = 0.5f * ( qw*gy - qx*gz + qz*gx);
  float qz_dot = 0.5f * ( qw*gz + qx*gy - qy*gx);

  // --- Madgwick 原式 IMU 梯度修正（s0..s3）---
  if (!(axu==0.0f && ayu==0.0f && azu==0.0f)) {
    const float _2q0 = 2.0f*qw, _2q1 = 2.0f*qx, _2q2 = 2.0f*qy, _2q3 = 2.0f*qz;
    const float _4q0 = 4.0f*qw, _4q1 = 4.0f*qx, _4q2 = 4.0f*qy;
    const float _8q1 = 8.0f*qx, _8q2 = 8.0f*qy;
    const float q0q0 = qw*qw, q1q1 = qx*qx, q2q2 = qy*qy, q3q3 = qz*qz;

    // 這四行請保持與參考實作一致，少項會造成發散
    float s0 = _4q0*q2q2 + _2q2*axu + _4q0*q1q1 - _2q1*ayu;
    float s1 = _4q1*q3q3 - _2q3*axu + 4.0f*q0q0*qx - _2q0*ayu - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*azu;
    float s2 = 4.0f*q0q0*qy + _2q0*axu + _4q2*q3q3 - _2q3*ayu - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*azu;
    float s3 = 4.0f*q1q1*qz - _2q1*axu + 4.0f*q2q2*qz - _2q2*ayu;

    // 正規化梯度，避免步長過大
    float sn = s0*s0 + s1*s1 + s2*s2 + s3*s3;
    if (sn > 0.0f) {
      float invsn = 1.0f / sqrtf(sn);
      s0*=invsn; s1*=invsn; s2*=invsn; s3*=invsn;

      // qDot -= beta * s
      qw_dot -= beta_eff * s0;
      qx_dot -= beta_eff * s1;
      qy_dot -= beta_eff * s2;
      qz_dot -= beta_eff * s3;
    }
  }

  // --- 積分 + 正規化 ---
  qw += qw_dot*dt; qx += qx_dot*dt; qy += qy_dot*dt; qz += qz_dot*dt;
  q = quatNormalize({qw,qx,qy,qz});

  if (rpy_out) *rpy_out = quatToEulerZYXdeg(q);
  return q;
}
