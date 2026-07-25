//
// Created by zn1135 on 25-9-23.
//

#ifndef USER_LIB_H
#define USER_LIB_H

#include "Motor.h"

#define IMU_UPDATA       1 // use imu
#define INFANTRY         1
#define CHASSIS_MAXSPEED ((send_chassis_power_limit() == 35) ? (3500) : (6000)) // 底盘最大速度
// #define  CHASSIS_MAXSPEED 6000 //底盘最大速度
#define GPIO_DEBOUNCE_TICKS 10U // 1ms调用周期下，GPIO输入消抖阈值（建议5~10）
// 高度状态机滞回阈值（1ms调用周期）
#define HEIGHT_STATE_ENTER_TICKS 10U // Middle进入High/Low所需连续触发次数
#define HEIGHT_STATE_EXIT_TICKS  10U // High/Low退出到Middle所需连续释放次数
#define High_GPIO                GPIO_PIN_9
#define Low_GPIO                  GPIO_PIN_7
// 定义电机编号枚举，用于标识不同的电机位置
// WHEEL系列代表行走电机，RUD系列代表转向电机
enum {
    WHEEL_FR = 0, // 前右轮
    WHEEL_FL = 1, // 前左轮
    WHEEL_BR = 2, // 后右轮
    WHEEL_BL = 3, // 后左轮
    YAW      = 4,
    PITCH    = 5,
    Height   = 6,
    FRICTR   = 7,
    FRICTL   = 8,
    PLUCK    = 9,
    CAP      = 10
};

/*CAN2的电机回调ID*/
typedef enum {
    WHEEL_FR_ID = 0x201,
    WHEEL_FL_ID = 0x202,
    WHEEL_BR_ID = 0x203,
    WHEEL_BL_ID = 0x204,
} CAN1_Message_ID;
/*CAN3的电机回调ID*/
typedef enum {

    Height_ID = 0x205,
    FRICTR_ID = 0x206,
    FRICTL_ID = 0x207,
    PLUCK_ID  = 0x208,
} CAN2_Message_ID;

typedef enum {
    YAW_ID   = 0x01,
    PITCH_ID = 0x02,
    CAP_ID   = 0x108,
} CAN3_Message_ID;

// DM电机配置参数
#define P_MAX  3.141592f
#define P_MIN  -3.141592f

#define V_MAX  30
#define V_MIN  -30

#define T_MAX  10
#define T_MIN  -10

#define KP_MAX 500
#define KP_MIN 0

#define KD_MAX 5
#define KD_MIN 0

typedef enum {
    CHASSIS_BENCH = 0,
    CHASSIS_WHEEL,
    CHASSIS_SEPARA,
    CHASSIS_GYROSC,
    CHASSIS_FOLLOW,
} chassis_mode_t;

typedef enum {
    DEBUG_MODE = 0,
    MOUSESHOOT
} shoot_mode_t;

typedef enum {
    NORMAL_MODE = 0,
    UNBOUNCE_MODE,
    NOSHOOT_MODE,

} gimbal_mode_t;

typedef enum {
    RC = 0,
    PC,
    LOSE
} ctrl_mode_t;

typedef struct {
    chassis_mode_t ChassisMode;
    shoot_mode_t ShootMode;
    ctrl_mode_t CtrlMode;
    gimbal_mode_t GimbalMode;
} robot_mode_t;

typedef enum {
    High = 0,
    Low,
    Middle
} Height_mode_t;

typedef enum {
    VT_C = 0,
    VT_N,
    VT_S
} VT_switch_t;

typedef struct {
    float receive_yaw_relative_angle;
    float receive_yaw_relative_rad;
    int8_t gimbal_direction;
    uint8_t basic_region;
} relative_angle_t;

// 68.601784

// 相关的全局变量定义

#define GIMBAL_YAW_J          0.175701f  // kg*m^2
#define GIMBAL_YAW_B          0.057705f  // N*m*s/rad
#define GIMBAL_YAW_FC         0.220767f  // N*m
#define GIMBAL_YAW_TAU0       0.017746f  // N*m
#define GIMBAL_YAW_FC_SMOOTH  0.15f      // rad/s
#define Moment_of_Inertia     GIMBAL_YAW_J


#define Gimbal_FrontAngle     88.081459f
#define Gimbal_backAngle      88.081459f - 180.0f //
#define Moment_of_Inertia     0.074867862        // 0.01006421849
#define PI                    3.14159265358979f  /*圆周率*/
#define Eco_to_Degree         0.000095826f
#define wheel_const           1000.0f
#define Rad_To_Degree         57.3f
#define RPM_To_Rads           0.104666667f // rpm -> rad/s
#define Reduction_Ratio       19.2f        // 减速比
#define CHASSIS_WHEEL_RADIUS  0.07f        // 底盘轮子的半径，单位：m
#define joint_const           25.0f        // 关节电机转矩系数   0.24/6
#define Leg_High_Length       0.38f
#define CHASSIS_Yaw_IMU_FRONT 3.14f //-1.57f//
#define CHASSIS_Yaw_IMU_BACK  0.0f  // 1.57f//
float AngleLimit(float tar, float max_angle, float min_angle);
void TargetAngleProc(motor_param_t *ptr);
void FloatToUint8_t(float fNum, uint8_t *strBuf, int nLen);
void relative_angle_processing(motor_param_t *ptr);
void GetGimbalFrontBackOrientation(motor_param_t *ptr);
void MotorWatchdogMonitor();
void Getreceive_yaw_relative_angle(motor_param_t *ptr);
extern uint8_t RB_Color;
extern uint8_t RoboHP_flag;
extern uint8_t yaw_position[4];
extern uint8_t yaw_flag;
extern relative_angle_t xRelativeAngle;
extern uint8_t AUTO_Shoot_flag;
extern float Gimbal_FOLLOWANGLE;
extern uint8_t Motor_TimeOut_flag;

#endif // USER_LIB_H
