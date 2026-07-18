#ifndef ODOMETRY_KINEMATICS_H
#define ODOMETRY_KINEMATICS_H

// 纯数学层：无硬件依赖，固件与主机测试共用。
// 只做几何/运动学，不读任何全局状态。

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define ODOM_KIN_G 9.80665f

static inline bool Odom_IsFinite3(const float v[3])
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

// 四元数模长检查并归一化，异常返回 false。四元数顺序 [w,x,y,z]，表示 S->E 旋转。
static inline bool Odom_NormalizeQuaternion(float q[4])
{
    const float norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (!isfinite(norm_sq) || norm_sq < 0.25f || norm_sq > 2.25f)
    {
        return false;
    }
    const float inv = 1.0f / sqrtf(norm_sq);
    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
    return true;
}

// 用四元数把传感器系向量旋转到导航系 E。
static inline void Odom_RotateSensorToNav(const float q[4], const float s[3], float nav[3])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    nav[0] = (1.0f - 2.0f * (y * y + z * z)) * s[0] + 2.0f * (x * y - w * z) * s[1] + 2.0f * (x * z + w * y) * s[2];
    nav[1] = 2.0f * (x * y + w * z) * s[0] + (1.0f - 2.0f * (x * x + z * z)) * s[1] + 2.0f * (y * z - w * x) * s[2];
    nav[2] = 2.0f * (x * z - w * y) * s[0] + 2.0f * (y * z + w * x) * s[1] + (1.0f - 2.0f * (x * x + y * y)) * s[2];
}

// 麦轮正运动学：4 轮转速(rpm) 反推车体系速度分量(仍为 rpm 尺度)。
// 正解见 ChassisControl.c: FR=-(vx+vy+vw) FL=-vx+vy-vw BR=-(-vx+vy+vw) BL=vx+vy-vw
// 反解： vx=(BL+BR-FR-FL)/4  vy=(BL+FL-FR-BR)/4  vw=-(FR+FL+BR+BL)/4
static inline void Odom_WheelForwardKinematics(float fr, float fl, float br, float bl,
                                               float *vx_body, float *vy_body, float *vw_body)
{
    *vx_body = (bl + br - fr - fl) * 0.25f;
    *vy_body = (bl + fl - fr - br) * 0.25f;
    *vw_body = -(fr + fl + br + bl) * 0.25f;
}

// 车体系速度(vx_body=右, vy_body=前)按底盘航向 head 投影到导航系 E。
// 与"加速度用完整四元数旋转到 E"严格同系：
//   前向单位向量落在 (cos head, sin head)，右向落在 (sin head, -cos head)。
// head = 云台航向(atan2(fwd_nav_y, fwd_nav_x)) - 云台相对底盘角。
static inline void Odom_WheelBodyToNav(float vx_body, float vy_body, float head,
                                       float *vx_nav, float *vy_nav)
{
    const float c = cosf(head);
    const float s = sinf(head);
    *vx_nav = vy_body * c + vx_body * s;
    *vy_nav = vy_body * s - vx_body * c;
}

// 车体系速度(vx_body=右, vy_body=前)绕云台相对底盘角 rel 旋转到云台系。
// 云台系轴定义与车体系一致(VX=右, VY=前)。rel=0 时两系重合。
// 旋向 R(+rel) 已上车验证:小陀螺匀速直走时自转项约掉,VX/VY 为常数。
// (若某次改线后又见 2 倍频正弦,即旋向反,翻 sin 两项符号。)
static inline void Odom_WheelBodyToGimbal(float vx_body, float vy_body, float rel,
                                          float *vx_gimbal, float *vy_gimbal)
{
    const float c = cosf(rel);
    const float s = sinf(rel);
    *vx_gimbal =  vx_body * c - vy_body * s;
    *vy_gimbal = vx_body * s + vy_body * c;
}

// 从四元数取云台在导航系的航向角(把传感器前向 {0,1,0} 投到水平面)。
static inline bool Odom_QuaternionToYaw(const float q[4], float *yaw)
{
    const float forward_sensor[3] = {0.0f, 1.0f, 0.0f};
    float forward_nav[3];
    Odom_RotateSensorToNav(q, forward_sensor, forward_nav);
    const float h = sqrtf(forward_nav[0] * forward_nav[0] + forward_nav[1] * forward_nav[1]);
    if (!isfinite(h) || h < 0.1f)
    {
        return false;
    }
    *yaw = atan2f(forward_nav[1], forward_nav[0]);
    return isfinite(*yaw);
}

// 加速度(传感器系, m/s^2) 旋转到导航系并去除重力，输出水平线加速度。
static inline void Odom_AccelSensorToNavHorizontal(const float q[4], const float acc_s[3],
                                                   float gravity_sign, float accel_nav[2])
{
    float acc_nav[3];
    Odom_RotateSensorToNav(q, acc_s, acc_nav);
    acc_nav[2] -= gravity_sign * ODOM_KIN_G;
    accel_nav[0] = acc_nav[0];
    accel_nav[1] = acc_nav[1];
}

// 互补滤波单步：IMU 加速度高频预测 + 轮速低频拉回。
//   v_pred  = v_prev + a_nav*dt        (高频, 跟手/抗打滑)
//   v_fused = v_pred + k*(v_wheel-v_pred)  (低频锚, 不漂)
//   k = dt/(TAU+dt)
static inline float Odom_ComplementaryStep(float v_prev, float a_nav,
                                           float v_wheel, float dt, float k)
{
    const float v_pred = v_prev + a_nav * dt;
    return v_pred + k * (v_wheel - v_pred);
}

// ---- 二阶卡尔曼：状态 [pos, vel]，IMU 加速度作输入，轮速作观测 ----

// 每轴独立的 KF 状态（X/Y 共用同一参数，各持独立协方差）。
typedef struct {
    float p;     // 位置 (m)
    float v;     // 速度 (m/s)
    float P00;   // cov(p,p)
    float P01;   // cov(p,v) = cov(v,p)
    float P11;   // cov(v,v)
    uint8_t init; // 0=未初始化，首次调用前须先 Init
} OdomKf2_t;

// 初始化：位置归零，速度用当前轮速，P 对角初始化。
static inline void Odom_Kf2_Init(OdomKf2_t *kf, float v0, float P0_p, float P0_v)
{
    kf->p = 0.0f; kf->v = v0;
    kf->P00 = P0_p; kf->P01 = 0.0f; kf->P11 = P0_v;
    kf->init = 1U;
}

// 预测步：a=IMU 世界系加速度，dt=控制周期，Qp/Qv=过程噪声方差。
static inline void Odom_Kf2_Predict(OdomKf2_t *kf, float a, float dt,
                                    float Qp, float Qv)
{
    const float p_pred  = kf->p + kf->v * dt + 0.5f * a * dt * dt;
    const float v_pred  = kf->v + a * dt;
    const float P00_new = kf->P00 + 2.0f * dt * kf->P01 + dt * dt * kf->P11 + Qp;
    const float P01_new = kf->P01 + dt * kf->P11;
    const float P11_new = kf->P11 + Qv;
    kf->p = p_pred; kf->v = v_pred;
    kf->P00 = P00_new; kf->P01 = P01_new; kf->P11 = P11_new;
}

// 更新步：z_v=轮速世界系速度观测，Rv=测速噪声方差。H=[0,1]。
static inline void Odom_Kf2_Update(OdomKf2_t *kf, float z_v, float Rv)
{
    const float S = kf->P11 + Rv;
    if (S < 1e-9f) { return; }
    const float K0  = kf->P01 / S;
    const float K1  = kf->P11 / S;
    const float y   = z_v - kf->v;
    kf->p   += K0 * y;
    kf->v   += K1 * y;
    const float P00_new = kf->P00 - K0 * kf->P01;
    const float P01_new = kf->P01 - K0 * kf->P11;
    const float P11_new = (1.0f - K1) * kf->P11;
    kf->P00 = P00_new; kf->P01 = P01_new; kf->P11 = P11_new;
}

#endif // ODOMETRY_KINEMATICS_H
