#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>
#include "OdometryKinematics.h"

// 验证阶段：轮速与 IMU 加速度各自独立解算到世界系(导航系 E)，
// 只做对比观察，不做融合、不积分 IMU 速度。

typedef struct
{
    // 轮速链：世界系水平速度 (m/s)
    float wheel_vx_nav;
    float wheel_vy_nav;
    // 轮速链：车体系速度 (m/s，调试用)
    float wheel_vx_body;
    float wheel_vy_body;
    // 轮速链：云台系速度 (m/s，VX=右 VY=前，调试用)
    float wheel_vx_gimbal;
    float wheel_vy_gimbal;
    // 车体系速度模长 √(vx²+vy²)，小陀螺诊断用：应恒定=真实平动速度
    float wheel_speed_body;
    // IMU 链：世界系水平线加速度 (m/s^2，去重力，不积分)
    float imu_ax_nav;
    float imu_ay_nav;
    // 融合输出：世界系速度 (m/s)，KF 速度状态（向后兼容命名）
    float fused_vx_nav;
    float fused_vy_nav;
    // KF 输出：世界系位移 (m，开机归零)
    float pos_x_nav;
    float pos_y_nav;
    // 内部 KF 状态（X/Y 轴各一，含协方差）
    OdomKf2_t kf_x;
    OdomKf2_t kf_y;
    // 重力估计低通状态(传感器系, m/s^2)——不依赖四元数去重力
    float gravity_lpf[3];
    uint8_t gravity_ready; // 低通收敛标志
    // 公共量
    float chassis_yaw;        // 底盘航向原始值 (rad, 范围[-2π,2π], 给cos/sin用)
    float chassis_yaw_cont;   // 连续航向 (rad, 无回绕, 可积分/微分, 融合用)
    float chassis_yaw_prev;   // 上周期原始值(内部, unwrap用)
    float head_sync;          // 同源航向: vw高频积分+chassis_yaw_cont低频纠偏, 供速度重构
    float dbg_gimbal_yaw;     // 调试: 云台世界航向(四元数)
    float dbg_gimbal_rel;     // 调试: relative输入(应为chassis-gimbal机械角)
    uint8_t yaw_cont_init;    // 连续航向初始化标志
    uint8_t imu_valid;        // 四元数有效标志
    uint8_t yaw_valid;        // 航向可解算标志
} WheelOdom_t;

extern WheelOdom_t WheelOdom;

// gimbal_relative_rad: 云台相对底盘的 yaw 角(xRelativeAngle.receive_yaw_relative_rad)
void WheelOdom_Init(void);
void WheelOdom_Update(float gimbal_relative_rad);

#endif // ODOMETRY_H
