//
// Created by Claude on 25-5-30.
//

#include "Odometry.h"
#include "OdometryKinematics.h"

#include <math.h>
#include <string.h>

#include "arm_math.h"
#include "kalman_filter.h"   // vaEstimateKF, Kalman_Filter_*
#include "Motor.h"           // xMotorParam[]
#include "user_lib.h"        // WHEEL_* 枚举, RPM_To_Rads, Reduction_Ratio, CHASSIS_WHEEL_RADIUS, PI, xRelativeAngle
#include "IMU_Task.h"        // IMU_Yaw
#include "imu_data_decode.h" // receive_imusol

/* ===================== 可调参数（上车后用 Vofa 标定）===================== */

#define ODOM_DT 0.001f      // 控制周期：CtrlTask 中 osDelay(1) => 1ms
#define ODOM_G 9.80665f     // 重力加速度，IMU acc[] 单位为 g，需转 m/s^2
#define DEG2RAD (PI / 180.0f)

// rotor rpm -> 轮面线速度 m/s ：rpm * (rad/s per rpm) / 减速比 * 轮半径
#define WHEEL_RPM_TO_MPS (RPM_To_Rads / Reduction_Ratio * CHASSIS_WHEEL_RADIUS)

// 底盘旋转半径 (lx+ly)，用于把逆解算的旋转分量换算成 rad/s。请实测后修改！
#define CHASSIS_ROTATE_RADIUS 0.2f

// 陀螺仪是否安装在云台上(随云台一起转)。1=云台(默认,小陀螺时云台不转) 0=底盘
#define ODOM_IMU_ON_GIMBAL 1

// —— 四元数去重力（IMU 在云台上、随云台 pitch/yaw，必须用姿态去掉重力分量）——
// quat 元素顺序：HiPNUC IMUSOL 包为 [w,x,y,z]。固件不同就改下标。
#define ODOM_QUAT_W 0
#define ODOM_QUAT_X 1
#define ODOM_QUAT_Y 2
#define ODOM_QUAT_Z 3
// quat 默认表示 body->world 旋转。若标定发现“静止时俯仰云台，acc_body 不为 0”，
// 把这里置 1（取共轭，按 world->body 处理）。
#define ODOM_QUAT_CONJ 0
// 加速度世界系与里程计世界系的航向对齐微调(rad)，一般 0；若推车方向与轮速对不上再调。
#define ODOM_ACC_YAW_OFFSET 0.0f

// —— 航向/相对角符号校准位 ——
#define ODOM_YAW_SIGN   (+1.0f)   // IMU 航向角正方向
#define ODOM_REL_SIGN   (+1.0f)   // 云台-底盘相对角正方向

// —— 卡尔曼噪声（相对大小决定信轮速还是信加速度，按需微调）——
#define ODOM_R_VEL  0.25f   // 轮速量测噪声方差（越大越不信轮速，抗打滑↑但滞后↑）
#define ODOM_R_ACC  1.50f   // 加速度量测噪声方差
#define ODOM_Q_VEL  0.005f  // 速度过程噪声
#define ODOM_Q_ACC  8.0f    // 加速度过程噪声（accel 变化快，给大一点）
#define ODOM_P_INIT 1.0f    // 初始协方差

/* ======================================================================= */

Odometry_t Odom;
KalmanFilter_t vaEstimateKF; // 满足 kalman_filter.h 中的 extern 声明

static float s_chassis_yaw0; // 起始底盘绝对航向(rad)，作为里程计世界系 0 航向
static uint8_t s_kf_ok = 0;  // KF 关键矩阵是否分配成功（堆不足时为 0，走兜底）

/**
 * @brief  初始化 4 状态 va 卡尔曼滤波器
 *         x = [vx_w, vy_w, ax_w, ay_w]^T   (世界系)
 *         z = [vx_wheel, vy_wheel, ax_imu, ay_imu]^T
 *         匀加速模型： v(k) = v(k-1) + a*dt
 */
static void Odom_KF_Init(void)
{
    Kalman_Filter_Init(&vaEstimateKF, 4, 0, 4);
    vaEstimateKF.UseAutoAdjustment = 0; // 每周期轮速与加速度都有效，不做动态调整

    static const float F_Init[16] = {
        1, 0, ODOM_DT, 0,
        0, 1, 0, ODOM_DT,
        0, 0, 1, 0,
        0, 0, 0, 1};
    static const float P_Init[16] = {
        ODOM_P_INIT, 0, 0, 0,
        0, ODOM_P_INIT, 0, 0,
        0, 0, ODOM_P_INIT, 0,
        0, 0, 0, ODOM_P_INIT};
    static const float Q_Init[16] = {
        ODOM_Q_VEL, 0, 0, 0,
        0, ODOM_Q_VEL, 0, 0,
        0, 0, ODOM_Q_ACC, 0,
        0, 0, 0, ODOM_Q_ACC};
    static const float R_Init[16] = {
        ODOM_R_VEL, 0, 0, 0,
        0, ODOM_R_VEL, 0, 0,
        0, 0, ODOM_R_ACC, 0,
        0, 0, 0, ODOM_R_ACC};
    // UseAutoAdjustment=0 时需手动给定 H（这里量测与状态一一对应，取单位阵）
    static const float H_Init[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1};
    static const float StateMinVar[4] = {0.001f, 0.001f, 0.01f, 0.01f};

    memcpy(vaEstimateKF.F_data, F_Init, sizeof(F_Init));
    memcpy(vaEstimateKF.P_data, P_Init, sizeof(P_Init));
    memcpy(vaEstimateKF.Q_data, Q_Init, sizeof(Q_Init));
    memcpy(vaEstimateKF.R_data, R_Init, sizeof(R_Init));
    memcpy(vaEstimateKF.H_data, H_Init, sizeof(H_Init));
    memcpy(vaEstimateKF.StateMinVariance, StateMinVar, sizeof(StateMinVar));

    // 校验关键矩阵（尤其算增益用的 K/S/temp）确实分配成功；任一为 NULL 说明堆不足，
    // 滤波会静默失效(增益恒0、速度冻结)，此时置 0 走兜底，不再信任 KF 输出。
    s_kf_ok = (vaEstimateKF.K_data != NULL && vaEstimateKF.S_data != NULL &&
               vaEstimateKF.temp_matrix_data != NULL && vaEstimateKF.temp_matrix_data1 != NULL &&
               vaEstimateKF.FilteredValue != NULL);
}

void Odometry_Init(void)
{
    memset(&Odom, 0, sizeof(Odom));
    s_chassis_yaw0 = 0.0f;
    Odom_KF_Init();
}

void Odometry_Reset(void)
{
    Odom.pos_x = 0.0f;
    Odom.pos_y = 0.0f;
    Odom.pos_gimbal_y = 0.0f;
    Odom.total_dist = 0.0f;
    Odom.inited = 0; // 下次 Update 时重新锁定起始航向
}

/**
 * @brief  麦轮逆解算：四轮转速(rpm) -> 底盘机体系速度(m/s, rad/s)
 *
 * 正解算(ChassisControl_Calculating)使用的关系：
 *   FR = -(vx + vy + vw)
 *   FL = -vx + vy - vw
 *   BR =  vx - vy - vw
 *   BL =  vx + vy - vw
 * 对其求逆得到（取冗余项平均，抗单轮噪声/打滑）：
 *   vx = (BL - FL + BR - FR) / 4
 *   vy = (FL - FR + BL - BR) / 4
 *   vw = -(FR + FL + BR + BL) / 4
 */
static void Odom_InverseSolve(float *vx_chassis, float *vy_chassis)
{
    // 轮面线速度 m/s（电机转子 rpm -> 轮面 m/s）
    const float fr = xMotorParam[WHEEL_FR].MeasureEco->speed_rpm * WHEEL_RPM_TO_MPS;
    const float fl = xMotorParam[WHEEL_FL].MeasureEco->speed_rpm * WHEEL_RPM_TO_MPS;
    const float br = xMotorParam[WHEEL_BR].MeasureEco->speed_rpm * WHEEL_RPM_TO_MPS;
    const float bl = xMotorParam[WHEEL_BL].MeasureEco->speed_rpm * WHEEL_RPM_TO_MPS;
    const OdomMecanumVelocity_t velocity = Odom_DecodeCalibratedMecanum(fr, fl, br, bl);

    *vx_chassis = velocity.vx_chassis;
    *vy_chassis = velocity.vy_chassis;
    Odom.wheel_wz = velocity.rotation_linear / CHASSIS_ROTATE_RADIUS;
}

/**
 * @brief  四元数去重力：把 IMU 机体加速度(含重力, 单位 g) 旋转到 IMU 世界系，
 *         重力整体落在世界系 Z 轴，因此输出的水平 X/Y 分量自动不含重力，
 *         且与云台 pitch/roll/yaw 姿态无关（姿态全部由四元数吸收）。
 * @param  ax_iw [out] IMU 世界系 X 向水平加速度 (m/s^2)
 * @param  ay_iw [out] IMU 世界系 Y 向水平加速度 (m/s^2)
 */
static void Odom_AccToWorldHorizontal(float *ax_iw, float *ay_iw)
{
    float w = receive_imusol.quat[ODOM_QUAT_W];
    float x = receive_imusol.quat[ODOM_QUAT_X];
    float y = receive_imusol.quat[ODOM_QUAT_Y];
    float z = receive_imusol.quat[ODOM_QUAT_Z];
#if ODOM_QUAT_CONJ
    x = -x;
    y = -y;
    z = -z; // 取共轭：按 world->body 处理
#endif
    // 机体系加速度 (m/s^2)，含重力
    float bx = receive_imusol.acc[0] * ODOM_G;
    float by = receive_imusol.acc[1] * ODOM_G;
    float bz = receive_imusol.acc[2] * ODOM_G;
    // 旋转矩阵 R(body->world) 的前两行（世界系 X、Y；第三行=Z 含重力，不需要）
    *ax_iw = (1.0f - 2.0f * (y * y + z * z)) * bx + (2.0f * (x * y - w * z)) * by + (2.0f * (x * z + w * y)) * bz;
    *ay_iw = (2.0f * (x * y + w * z)) * bx + (1.0f - 2.0f * (x * x + z * z)) * by + (2.0f * (y * z - w * x)) * bz;
}

void Odometry_Update(void)
{
    /* ---------- 1. 航向（世界系，以起始为 0） ---------- */
    float yaw_g_abs = ODOM_YAW_SIGN * IMU_Yaw.total_angle * DEG2RAD; // 云台绝对航向
#if ODOM_IMU_ON_GIMBAL
    float rel = ODOM_REL_SIGN * xRelativeAngle.receive_yaw_relative_rad; // 云台相对底盘
#else
    float rel = 0.0f;
#endif
    float yaw_c_abs = yaw_g_abs - rel; // 底盘绝对航向

    if (!Odom.inited)
    {
        s_chassis_yaw0 = yaw_c_abs;
        Odom.inited = 1;
    }
    Odom.yaw_chassis = yaw_c_abs - s_chassis_yaw0; // 底盘里程计航向
    Odom.yaw_gimbal = yaw_g_abs - s_chassis_yaw0;  // 云台里程计航向
    Odom.yaw_rate = ODOM_YAW_SIGN * IMU_Yaw.speed_gyro * DEG2RAD;

    /* ---------- 2. 逆解算：轮速 -> 底盘机体系速度 ---------- */
    float wheel_vx_chassis;
    float wheel_vy_chassis;
    Odom_InverseSolve(&wheel_vx_chassis, &wheel_vy_chassis);

    // 公开轮速使用云台坐标系；该矩阵是小陀螺指令变换的严格逆变换。
    const float sin_rel = arm_sin_f32(rel);
    const float cos_rel = arm_cos_f32(rel);
    Odom_ChassisToGimbal(wheel_vx_chassis,
                         wheel_vy_chassis,
                         sin_rel,
                         cos_rel,
                         &Odom.wheel_vx,
                         &Odom.wheel_vy);

    /* ---------- 3. 量测旋转到世界系 ---------- */
    // 世界系融合必须继续使用底盘机体系轮速和底盘航向。
    const float cc = arm_cos_f32(Odom.yaw_chassis);
    const float sc = arm_sin_f32(Odom.yaw_chassis);
    const float vx_w_meas = wheel_vx_chassis * cc - wheel_vy_chassis * sc;
    const float vy_w_meas = wheel_vx_chassis * sc + wheel_vy_chassis * cc;

    // 加速度：四元数去重力，得到 IMU 世界系水平加速度（与云台姿态无关）
    float ax_iw, ay_iw;
    Odom_AccToWorldHorizontal(&ax_iw, &ay_iw);
    // 再旋转到里程计世界系（与轮速同一世界系=底盘起始朝向）：
    // 起始底盘前向在 IMU 世界系朝向 s0，odom-world X=右、Y=前。
    float s0 = s_chassis_yaw0 + ODOM_ACC_YAW_OFFSET;
    float cs0 = arm_cos_f32(s0);
    float ss0 = arm_sin_f32(s0);
    float ax_w_meas = ax_iw * ss0 - ay_iw * cs0; // odom 世界系 X(右)向加速度
    float ay_w_meas = ax_iw * cs0 + ay_iw * ss0; // odom 世界系 Y(前)向加速度

    /* ---------- 4. 卡尔曼融合 ---------- */
    if (s_kf_ok)
    {
        vaEstimateKF.MeasuredVector[0] = vx_w_meas; // 世界系 vx (轮速)
        vaEstimateKF.MeasuredVector[1] = vy_w_meas; // 世界系 vy (轮速)
        vaEstimateKF.MeasuredVector[2] = ax_w_meas; // 世界系 ax (IMU)
        vaEstimateKF.MeasuredVector[3] = ay_w_meas; // 世界系 ay (IMU)
        Kalman_Filter_Update(&vaEstimateKF);

        Odom.vx_world = vaEstimateKF.FilteredValue[0];
        Odom.vy_world = vaEstimateKF.FilteredValue[1];
        Odom.ax_world = vaEstimateKF.FilteredValue[2];
        Odom.ay_world = vaEstimateKF.FilteredValue[3];
    }
    else
    {
        // 兜底：KF 分配失败时直接用世界系轮速/加速度量测（无融合），保证里程计仍可用
        Odom.vx_world = vx_w_meas;
        Odom.vy_world = vy_w_meas;
        Odom.ax_world = ax_w_meas;
        Odom.ay_world = ay_w_meas;
    }

    /* ---------- 5. 世界系积分得到位置 ---------- */
    Odom.pos_x += Odom.vx_world * ODOM_DT;
    Odom.pos_y += Odom.vy_world * ODOM_DT;
    Odom.speed = sqrtf(Odom.vx_world * Odom.vx_world + Odom.vy_world * Odom.vy_world);
    Odom.total_dist += Odom.speed * ODOM_DT;

    const float cg = arm_cos_f32(Odom.yaw_gimbal);
    const float sg = arm_sin_f32(Odom.yaw_gimbal);
    const float vy_gimbal_fused = Odom_WorldToGimbalForward(Odom.vx_world,
                                                            Odom.vy_world,
                                                            sg,
                                                            cg);
    Odom.pos_gimbal_y += vy_gimbal_fused * ODOM_DT;

    // 融合速度反旋转回机体系，便于与轮速直接对比
    Odom.vx_body = Odom.vx_world * cc + Odom.vy_world * sc;
    Odom.vy_body = -Odom.vx_world * sc + Odom.vy_world * cc;

    // 去重力后的机体系加速度（标定用）：静止时任意云台姿态都应≈0；
    // 推车前进 acc_body_y 应先正后负。pitch 云台时它不再变化即说明去重力成功。
    Odom.acc_body_x = ax_w_meas * cc + ay_w_meas * sc;
    Odom.acc_body_y = -ax_w_meas * sc + ay_w_meas * cc;
}
