#ifndef __RAMP_H
#define __RAMP_H


#include "arm_math.h"
#include <stdio.h>
#include "stdbool.h"
#include "string.h"
#include "stdint.h"



// 控制周期设定 0.001s
#define GIMBAL_CONTROL_TIME 0.0015f
#define CHASSIS_CONTROL_TIME 0.0015f
/*************************************************************************************************/
// 值越大滤波系数越大，数值变化越平滑
#define MOUSE_X_NUM 0.2f  //0.0633333333f  //左右平移
#define MOUSE_Y_NUM 0.001f  //0.0799999999f  //前后运动

// 数值越大滤波系数就越大，数值变化越平缓
#define CHASSIS_ACCEL_X_NUM 0.0666666667f  //0.0633333333f  //左右平移
#define CHASSIS_ACCEL_Y_NUM 0.159999999f //1.8333333333f  //前后运动
#define CHASSIS_ACCEL_W_NUM 0.5333333333f  //0.1333333333f
#define CHASSIS_ACCEL_Y_PC_NUM 0.3333333333f  //0.1333333333f
#define CHASSIS_ACCEL_Y_SLIP_NUM 0.1333333333f  //0.1333333333f

/************************************************************************************************/
#define mat         arm_matrix_instance_f32 //float
#define mat_64      arm_matrix_instance_f64 //double
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32//浮点矩阵转置
#define mat_inv     arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64


// 基于一阶的低通滤波器
typedef  struct
{
    float input;        // 输入数据
    float out;          // 滤波后的输出值
    float num[1];       // 滤波参数
    float frame_period; // 滤波器采样时间 单位 s
} first_order_filter_type_t;

typedef  struct
{
    float input;        // 输入数据
    float out;          // 输出值
    float min_value;    // 限幅最小值
    float max_value;    // 限幅最大值
    float frame_period; // 时间戳
} ramp_function_source_t;

//typedef struct{
//
//    int32_t compare_value_up;
//    int32_t compare_value_dowm;
//    int sin_ramp_switch;
//
//}SinRampState;

typedef struct
{
	float x_last;
	float p_last;
	float out;
} one_kalman_filter_init_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
} kalman_filter_init_t;

extern one_kalman_filter_init_t kalman_pitch;
extern one_kalman_filter_init_t kalman_velocity;
extern one_kalman_filter_init_t kalman_left_fn;
extern one_kalman_filter_init_t kalman_right_fn;
extern one_kalman_filter_init_t kalman_position;

extern float matrix_value1;
extern float matrix_value2;



extern first_order_filter_type_t xChassisCmdSlowSetVx;
extern first_order_filter_type_t xChassisCmdSlowSetVy;

void   kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);



extern ramp_function_source_t chassis_vy_move;
// 一阶滤波初始化
void FirstOrderFilterInit(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
// 一阶滤波计算
void FirstOrderFilterCali(first_order_filter_type_t *first_order_filter_type, float input);

// 斜坡函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);

// 斜坡函数计算
void ramp_calc(ramp_function_source_t *ramp_source_type, float input);

void MouseRampInit(void);
void ChassisRampInit(void);
float MeanFilter(float newvalue);

void SlopeSlow(float *rec , float target , float slow_Inc);
float  KalmanFilter(one_kalman_filter_init_t * data,const float ResrcData,float ProcessNoise_Q,float MeasureNoise_R);


//int32_t SinRampCalc(SinRampState *sin_state, int32_t variation, uint8_t Sin_control_time_up, uint8_t Sin_control_time_down);

//void SinRampInit(SinRampState *sin_state);

//void SinTest(void);
extern first_order_filter_type_t mouse_x;
extern first_order_filter_type_t mouse_y;
#endif
