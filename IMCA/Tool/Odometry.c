#include "Odometry.h"

#include "OdometryKinematics.h"

#ifndef ODOMETRY_HOST_TEST
#include "Motor.h"
#include "imu_data_decode.h"
#include "user_lib.h"
#endif

// ================= 可调常量：上车只改这里 =================
// rpm -> m/s 换算： (1/减速比) * (rpm->rad/s) * 轮半径
// Reduction_Ratio=14, RPM_To_Rads=0.104666667, CHASSIS_WHEEL_RADIUS=0.07
#define WHEEL_ODOM_SCALE (1.0f / 14.2f * 0.104666667f * 0.0775f)

// 四轮转速符号映射(电机安装方向)。上车若某轮反了，翻对应符号。
#define WHEEL_SIGN_FR (+1.0f)
#define WHEEL_SIGN_FL (+1.0f)
#define WHEEL_SIGN_BR (+1.0f)
#define WHEEL_SIGN_BL (+1.0f)

// 底盘绝对航向符号： chassis_yaw = gimbal_yaw - CHASSIS_YAW_REL_SIGN * relative
#define CHASSIS_YAW_REL_SIGN (+1.0f)

// 航向同源化：vw_body(rpm尺度)-> ωz(rad/s)。修小陀螺速度用。
// 起算 = WHEEL_ODOM_SCALE/(lx+ly)，lx+ly=半轴距+半轮距(m)。这里取 0.3 估值。
// 不准只影响暂态跟踪，低频纠偏保证稳态回到 chassis_yaw_cont。上车调这里。
#define ODOM_VW_TO_RADPS (WHEEL_ODOM_SCALE / 0.25f)
// 航向纠偏时间常数(s)：越大越信 vw 高频、暂态越同步；越小越快锚回绝对航向。
// chassis_yaw(ch6)已验证正确追踪自转，可以安全做低频锚。
#define ODOM_HEAD_TAU (0.5f)

#define ODOM_PI  (3.14159265358979f)
#define ODOM_2PI (6.28318530717959f)

// 四元数分量顺序 [w,x,y,z]
#define ODOM_QW 0U
#define ODOM_QX 1U
#define ODOM_QY 2U
#define ODOM_QZ 3U

// 重力去除：低通估计法(不依赖四元数约定)。
// 对传感器系加速度做低通得到"当前重力向量"，原始减低通=真实线性加速度。
// ALPHA = dt/(tau+dt)，控制周期约 1ms、时间常数 tau≈1s => ALPHA≈0.001。
// 越小越干净但收敛越慢，越大越跟手但会吃掉持续加速度。上车可调。
#define ODOM_GRAVITY_LPF_ALPHA (0.001f)
// 低通收敛判据：连续样本数达到后才认为重力估计可用。
#define ODOM_GRAVITY_WARMUP (2000U)

// 加速度计传感器轴符号(IMU 安装方向)。默认全 +1。
#define ODOM_IMU_ACC_SIGN_X (+1.0f)
#define ODOM_IMU_ACC_SIGN_Y (+1.0f)
#define ODOM_IMU_ACC_SIGN_Z (+1.0f)

// 加速度输出平滑低通截止(Hz 量级)。alpha越大截止越高、响应越快但噪声越大；
// 越小越干净但会模糊短促冲击。15Hz 起步，看不清就往上调。
// alpha ≈ dt/(1/(2π×fc)+dt)，1kHz 控制周期 fc=15Hz → α≈0.086
#define ODOM_ACCEL_SMOOTH_ALPHA (0.086f)

// 二阶卡尔曼调参（上车只改这里）
// Q_p 越大位置越灵活，Q_v 越大越信 IMU 加速度，R_v 越大越信轮速
#define ODOM_KF_Q_P  (0.0f)    // 位置过程噪声：位置只靠速度积分演化，无独立噪声
                                // 若加非零值会使 P00 快速发散(位置从未被直接观测)
#define ODOM_KF_Q_V  (1e-3f)   // 速度过程噪声方差
#define ODOM_KF_R_V  (1e-2f)   // 轮速测速噪声方差 (~0.1 m/s 标准差)
#define ODOM_KF_Q_V  (1e-3f)   // 速度过程噪声方差
#define ODOM_KF_R_V  (1e-2f)   // 轮速测速噪声方差 (~0.1 m/s 标准差)
#define ODOM_KF_P0_P (0.0f)    // 初始位置协方差：开机位置由用户定义为0，初始不确定度为0
#define ODOM_KF_P0_V (0.1f)    // 初始速度协方差
#define ODOM_FUSION_DT  (0.001f)  // CtrlTask 1kHz 固定周期

// 加速度旋转四元数共轭开关。重力已由 LPF 在传感器系去除，
// 此开关只影响线加速度的世界系方向，不会漏重力，可放心翻。
// =0 原始方向，=1 共轭。推车看 ch1/ch3 与 ch0/ch2 方向是否一致，不一致则翻。
#define ODOM_ACCEL_QUAT_CONJUGATE 0
// ========================================================

WheelOdom_t WheelOdom;

void WheelOdom_Init(void)
{
    for (unsigned i = 0; i < sizeof(WheelOdom); ++i)
    {
        ((unsigned char *)&WheelOdom)[i] = 0;
    }
}

#ifndef ODOMETRY_HOST_TEST
void WheelOdom_Update(float gimbal_relative_rad)
{
    // ---- 轮速链：4 轮 rpm -> 车体系速度(m/s) ----
    const float fr = WHEEL_SIGN_FR * (float)xMotorParam[WHEEL_FR].MeasureEco->speed_rpm;
    const float fl = WHEEL_SIGN_FL * (float)xMotorParam[WHEEL_FL].MeasureEco->speed_rpm;
    const float br = WHEEL_SIGN_BR * (float)xMotorParam[WHEEL_BR].MeasureEco->speed_rpm;
    const float bl = WHEEL_SIGN_BL * (float)xMotorParam[WHEEL_BL].MeasureEco->speed_rpm;

    float vx_body, vy_body, vw_body;
    Odom_WheelForwardKinematics(fr, fl, br, bl, &vx_body, &vy_body, &vw_body);
    vx_body *= WHEEL_ODOM_SCALE;
    vy_body *= WHEEL_ODOM_SCALE;
    WheelOdom.wheel_vx_body = vx_body;
    WheelOdom.wheel_vy_body = vy_body;
    // 车体系速度模长(旋转不变量)。边转边匀速走时应恒定；
    // 若它平 → 轮速没问题、摆动是航向跟踪(可修)；若它也摆 → 轮子打滑(物理)。
    WheelOdom.wheel_speed_body = sqrtf(vx_body * vx_body + vy_body * vy_body);

    // ---- 云台系速度：纯运动学，车体系速度绕相对角旋转到云台朝向 ----
    // 与四元数/yaw_valid 无关，放在四元数分支之前，云台角无效时也照常输出。
    Odom_WheelBodyToGimbal(vx_body, vy_body, gimbal_relative_rad,
                           &WheelOdom.wheel_vx_gimbal, &WheelOdom.wheel_vy_gimbal);

    // ---- 四元数：取云台航向，算底盘绝对航向 ----
    float q[4];
    q[0] = receive_imusol.quat[ODOM_QW];
    q[1] = receive_imusol.quat[ODOM_QX];
    q[2] = receive_imusol.quat[ODOM_QY];
    q[3] = receive_imusol.quat[ODOM_QZ];

    WheelOdom.imu_valid = Odom_NormalizeQuaternion(q) ? 1U : 0U;
    if (!WheelOdom.imu_valid)
    {
        WheelOdom.yaw_valid = 0U;
        return;
    }

    float gimbal_yaw;
    WheelOdom.yaw_valid = Odom_QuaternionToYaw(q, &gimbal_yaw) ? 1U : 0U;
    if (WheelOdom.yaw_valid)
    {
        WheelOdom.dbg_gimbal_yaw = gimbal_yaw;              // 调试
        WheelOdom.dbg_gimbal_rel = gimbal_relative_rad;    // 调试
        WheelOdom.chassis_yaw = gimbal_yaw - CHASSIS_YAW_REL_SIGN * gimbal_relative_rad;

        // 连续航向(unwrap)：累加逐周期增量，增量 wrap 到 [-π,π]。
        // 1kHz 下真实转角远小于 π，故增量归一化能安全跨越回绕跳变。
        if (!WheelOdom.yaw_cont_init)
        {
            WheelOdom.chassis_yaw_cont = WheelOdom.chassis_yaw;
            WheelOdom.head_sync = WheelOdom.chassis_yaw; // 首帧对齐到绝对航向
            WheelOdom.yaw_cont_init = 1U;
        }
        else
        {
            float d = WheelOdom.chassis_yaw - WheelOdom.chassis_yaw_prev;
            while (d > ODOM_PI)  { d -= ODOM_2PI; }
            while (d < -ODOM_PI) { d += ODOM_2PI; }
            WheelOdom.chassis_yaw_cont += d;

            // ---- 同源航向 head_sync：vw 高频积分(与 v_body 同源零延迟) + 低频纠偏 ----
            // 修小陀螺速度：v_body 与它用的航向同时刻同源，消除 ω·Δ 误差。
            const float wz = vw_body * ODOM_VW_TO_RADPS; // rad/s，同源
            WheelOdom.head_sync += wz * ODOM_FUSION_DT;  // 高频积分
            float e = WheelOdom.chassis_yaw_cont - WheelOdom.head_sync;
            while (e > ODOM_PI)  { e -= ODOM_2PI; }
            while (e < -ODOM_PI) { e += ODOM_2PI; }
            const float kh = ODOM_FUSION_DT / (ODOM_HEAD_TAU + ODOM_FUSION_DT);
            WheelOdom.head_sync += kh * e;               // 低频纠偏(不漂)
        }
        WheelOdom.chassis_yaw_prev = WheelOdom.chassis_yaw;

        // 轮速车体系速度 -> 世界系 E。直接用 chassis_yaw_cont。
        // head_sync(vw积分+慢纠偏TAU=0.5s)在小陀螺高转速时会有>1rad航向滞后，造成世界速度摆动。
        // chassis_yaw_cont 由 (gimbal_yaw - relative) 每周期直接算出，无积分漂移，无滞后。
        Odom_WheelBodyToNav(vx_body, vy_body, WheelOdom.chassis_yaw_cont,
                            &WheelOdom.wheel_vx_nav, &WheelOdom.wheel_vy_nav);
    }

    // ---- IMU 链：LPF 估计传感器系重力 → 相减得线加速度 → 四元数转世界系 ----
    // 不依赖四元数做去重力，云台任意姿态下均无重力泄漏。
    float acc_s[3];
    acc_s[0] = receive_imusol.acc[0] * ODOM_KIN_G * ODOM_IMU_ACC_SIGN_X;
    acc_s[1] = receive_imusol.acc[1] * ODOM_KIN_G * ODOM_IMU_ACC_SIGN_Y;
    acc_s[2] = receive_imusol.acc[2] * ODOM_KIN_G * ODOM_IMU_ACC_SIGN_Z;

    // 低通更新（在传感器系估计重力）
    for (int i = 0; i < 3; ++i)
    {
        WheelOdom.gravity_lpf[i] = (1.0f - ODOM_GRAVITY_LPF_ALPHA) * WheelOdom.gravity_lpf[i]
                                   + ODOM_GRAVITY_LPF_ALPHA * acc_s[i];
    }
    static uint32_t warmup_cnt;
    if (warmup_cnt < ODOM_GRAVITY_WARMUP) { ++warmup_cnt; }
    WheelOdom.gravity_ready = (warmup_cnt >= ODOM_GRAVITY_WARMUP) ? 1U : 0U;

    // 线加速度 = 原始 - 低通重力（传感器系）
    float a_linear[3];
    a_linear[0] = acc_s[0] - WheelOdom.gravity_lpf[0];
    a_linear[1] = acc_s[1] - WheelOdom.gravity_lpf[1];
    a_linear[2] = acc_s[2] - WheelOdom.gravity_lpf[2];

    // 四元数仅用于"方向旋转"，重力已去除，共轭翻错只影响方向，不漏重力。
    // ODOM_ACCEL_QUAT_CONJUGATE=0：原始方向；=1：共轭（若方向反了在这里调）
    float q_rot[4] = {q[0], q[1], q[2], q[3]};
#if ODOM_ACCEL_QUAT_CONJUGATE
    q_rot[1] = -q_rot[1]; q_rot[2] = -q_rot[2]; q_rot[3] = -q_rot[3];
#endif
    float a_nav[3];
    Odom_RotateSensorToNav(q_rot, a_linear, a_nav);
    // 输出一阶低通平滑(旧值即滤波状态)，压掉加速度计高频噪声
    WheelOdom.imu_ax_nav = (1.0f - ODOM_ACCEL_SMOOTH_ALPHA) * WheelOdom.imu_ax_nav
                           + ODOM_ACCEL_SMOOTH_ALPHA * a_nav[0];
    WheelOdom.imu_ay_nav = (1.0f - ODOM_ACCEL_SMOOTH_ALPHA) * WheelOdom.imu_ay_nav
                           + ODOM_ACCEL_SMOOTH_ALPHA * a_nav[1];

    // ---- 二阶卡尔曼融合：IMU 加速度预测 + 轮速观测，输出位置+速度 ----
    if (!WheelOdom.yaw_valid)
    {
        return; // 无航向无法转世界系，保持上一拍
    }
    // 重力未收敛时 a=0，等效纯轮速，位置连续积分不留空缺
    const float ax = WheelOdom.gravity_ready ? WheelOdom.imu_ax_nav : 0.0f;
    const float ay = WheelOdom.gravity_ready ? WheelOdom.imu_ay_nav : 0.0f;

    if (!WheelOdom.kf_x.init)
    {
        Odom_Kf2_Init(&WheelOdom.kf_x, WheelOdom.wheel_vx_nav,
                      ODOM_KF_P0_P, ODOM_KF_P0_V);
        Odom_Kf2_Init(&WheelOdom.kf_y, WheelOdom.wheel_vy_nav,
                      ODOM_KF_P0_P, ODOM_KF_P0_V);
    }

    Odom_Kf2_Predict(&WheelOdom.kf_x, ax, ODOM_FUSION_DT, ODOM_KF_Q_P, ODOM_KF_Q_V);
    Odom_Kf2_Predict(&WheelOdom.kf_y, ay, ODOM_FUSION_DT, ODOM_KF_Q_P, ODOM_KF_Q_V);
    Odom_Kf2_Update(&WheelOdom.kf_x, WheelOdom.wheel_vx_nav, ODOM_KF_R_V);
    Odom_Kf2_Update(&WheelOdom.kf_y, WheelOdom.wheel_vy_nav, ODOM_KF_R_V);

    // 输出：速度保持 fused_* 命名向后兼容，位置为新输出
    WheelOdom.fused_vx_nav = WheelOdom.kf_x.v;
    WheelOdom.fused_vy_nav = WheelOdom.kf_y.v;
    WheelOdom.pos_x_nav    = WheelOdom.kf_x.p;
    WheelOdom.pos_y_nav    = WheelOdom.kf_y.p;
}

#else
void WheelOdom_Update(float gimbal_relative_rad)
{
    (void)gimbal_relative_rad;
}
#endif
