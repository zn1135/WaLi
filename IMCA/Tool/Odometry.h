//
// Created by Claude on 25-5-30.
//
// 麦克纳姆轮里程计：融合「轮速逆解算」与「IMU加速度」
//
// 数据来源：
//   - 四个底盘麦轮 (M3508) 编码器转速  xMotorParam[WHEEL_*].MeasureEco->speed_rpm
//   - 陀螺仪线加速度 / 航向            receive_imusol.acc[]  /  IMU_Yaw.total_angle
//
// 处理流程：
//   1. 逆解算：四轮转速  ->  底盘机体系速度 (vx, vy, wz)
//   2. 卡尔曼融合：轮速(低频准、打滑时跳变) + 加速度(高频准、积分漂移) -> 平滑速度
//   3. 旋转到世界系并积分，得到场地坐标 (pos_x, pos_y)
//

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

typedef struct
{
    /* ---- 逆解算轮速（云台坐标系，单位 m/s；wheel_wz 为底盘 rad/s）---- */
    float wheel_vx;     // 云台横向速度；右向为正（符号由 ODOM_VX_SIGN 标定）
    float wheel_vy;     // 云台纵向速度；沿云台前向为正（符号由 ODOM_VY_SIGN 标定）
    float wheel_wz;     // 底盘逆解偏航角速度 rad/s

    /* ---- 卡尔曼融合后的速度（世界系，单位 m/s 与 m/s^2）---- */
    float vx_world;
    float vy_world;
    float ax_world;
    float ay_world;

    /* ---- IMU 机体系加速度（融合输入，便于标定加速度轴/符号，单位 m/s^2）---- */
    float acc_body_x;   // 机体横向加速度（静止≈0；若常驻±9.8 说明该轴是垂直轴、轴选错）
    float acc_body_y;   // 机体纵向加速度

    /* ---- 融合速度反旋转回机体系（便于与轮速对比观察，单位 m/s）---- */
    float vx_body;
    float vy_body;
    float speed;        // 合速度大小 sqrt(vx^2+vy^2)  m/s

    /* ---- 里程计输出：世界系位置与云台前向位移（单位 m）---- */
    float pos_x;
    float pos_y;
    float pos_gimbal_y; // 沿当前云台前向的累计位移
    float total_dist;   // 累计路程 m

    /* ---- 航向（单位 rad，以里程计启动时底盘朝向为 0）---- */
    float yaw_chassis;  // 底盘航向
    float yaw_gimbal;   // 云台航向
    float yaw_rate;     // 偏航角速度（来自IMU陀螺）rad/s

    uint8_t inited;     // 是否已锁定起始航向
} Odometry_t;

extern Odometry_t Odom;

/**
 * @brief  里程计初始化（在 TotalInitTask 中调用一次）
 *         初始化 va 卡尔曼滤波器 vaEstimateKF 并清零状态
 */
void Odometry_Init(void);

/**
 * @brief  里程计周期更新（在 TotalControl 中每 1ms 调用一次）
 *         逆解算 -> 卡尔曼融合 -> 世界系积分
 */
void Odometry_Update(void);

/**
 * @brief  里程计复位：位置清零，并以当前底盘朝向为新的 0 航向
 */
void Odometry_Reset(void);

#endif // ODOMETRY_H
