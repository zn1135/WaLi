//
// Created by zn1135 on 25-9-25.
//

#ifndef BIG_WHEELLEG_PID_H
#define BIG_WHEELLEG_PID_H

#include "main.h"
// #include "Motor.h"

enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,

    POSITION_PID,
    DELTA_PID,
};




/*PID结构体*/
typedef struct __pid_t
{
    float p;
    float i;
    float d;

    uint32_t pid_mode;          //PID模式，分位置式和增量式
    uint32_t MaxOutput;			//输出限幅
    uint32_t IntegralLimit;		//积分限幅


    float set[3];	//目标值，NOW, LAST, LLAST
    float get[3];	//测量值
    float err[3];	//误差


    float pout;		//P输出
    float iout;		//I输出
    float dout;		//D输出

    float pos_out;			//本次位置式输出，即 pos_out = pout + iout + dout
    float last_pos_out;		//上次位置式输出
    float delta_u;			//本次增量值
    float delta_out;		//本次增量式输出 = last_delta_out + delta_u
    float last_delta_out;   //上次增量式输出

    float max_err;              //最大偏差
    float deadband;				//err < deadband return


    void (*f_param_init)(struct __pid_t *pid,  /*定义PID参数初始化的函数指针*/
                         uint32_t pid_mode,
                         uint32_t maxOutput,
                         uint32_t integralLimit,
                         float p,
                         float i,
                         float d);
//    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);   //pid参数修改

}pid_t;



/**********************************************************************************************************************
 * @brief 初始化PID结构体
 *
 * 该函数用于初始化PID控制器的参数结构体，设置PID的工作模式、输出限制、积分限制以及PID三个参数(KP、KI、KD)。
 *
 * @param pid 指向要初始化的pid_t结构体的指针
 * @param mode PID工作模式，通常是POSITION_PID或DELTA_PID
 * @param maxout PID输出的最大值限制
 * @param intergral_limit 积分项的最大限制值，防止积分饱和
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 *********************************************************************************************************************/
void PID_struct_init(
        pid_t* pid,
        uint32_t mode,
        uint32_t maxout,
        uint32_t intergral_limit,

        float 	kp,
        float 	ki,
        float 	kd);
void abs_limit(float *a, float ABS_MAX);
void PidInit(void);
float pid_calc(pid_t* pid, float get, float set);

extern pid_t Automatic_aiming_pos;
extern pid_t Automatic_aiming_spd;
extern pid_t Chassis_Power;
extern pid_t Chassis_Rotate;  //底盘旋转PID结构体
float ABS(float num);
// void PidCalcEco(motor_param_t * ptr);
// void PidCalcImu(motor_param_t * ptr);

#endif //BIG_WHEELLEG_PID_H
