//
// Created by zn1135 on 25-9-25.
//

#include "pid.h"
#include "Motor.h"
#include "user_lib.h"

pid_t Chassis_Rotate;  //底盘旋转PID结构体

pid_t Automatic_aiming_pos;
pid_t Automatic_aiming_spd;


pid_t Chassis_Power;
/**
 * @brief 计算浮点数的绝对值
 * @param num 输入的浮点数
 * @return 返回num的绝对值
 */
float ABS(float num)
{
    float value = (num<0) ? -num : num;

    return value;
}

/**
 * @brief 限制浮点数的绝对值不超过指定的最大值
 * @param a 指向需要限制的浮点数的指针
 * @param ABS_MAX 限制的最大绝对值
 * @note 该函数会直接修改指针a指向的值
 */
void abs_limit(float *a, float ABS_MAX)
{
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

/**********************************************************************************************************************
* @brief   PID参数初始化
* @param   pid                  PID结构体指针
* @param   mode                 PID模式选择，0为位置式PID，1为增量式PID
* @param   maxout               输出限幅值
* @param   integral_limit       积分限幅值
* @param   kp                   比例系数
* @param   ki                   积分系数
* @param   kd                   微分系数
* @retval  None
**********************************************************************************************************************/
static void pid_param_init(
        pid_t *pid,
        uint32_t mode,
        uint32_t maxout,
        uint32_t integral_limit,
        float 	kp,
        float 	ki,
        float 	kd )
{

    if (pid == NULL) {
        return;
    }

    pid->pid_mode = mode;
    pid->MaxOutput = maxout;
    pid->IntegralLimit = integral_limit;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;

}

/**********************************************************************************************************************
 * @brief 初始化PID结构体
 *
 * 该函数用于初始化PID控制器的参数结构体，设置PID的工作模式、输出限制、积分限制以及PID三个参数(KP、KI、KD)。
 * 同时初始化函数指针，将pid_param_init函数地址赋给f_param_init指针。
 *
 * @param pid 指向要初始化的pid_t结构体的指针
 * @param mode PID工作模式，通常是POSITION_PID或其它模式
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
        float 	kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init; /*锟斤拷pid_param_init锟斤拷锟斤拷锟斤拷锟阶碉拷址锟斤拷值锟斤拷指锟斤拷锟斤拷锟絝_param_init*/
//    pid->f_pid_reset = pid_reset;

    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);

}

/**********************************************************************************************************************
 * @brief PID参数初始化函数
 *
 * 该函数用于初始化不同控制模式下各轴的PID控制参数。包括ECO模式和IMU模式下的位置环和速度环PID参数初始化。
 * 主要针对YAW轴和PITCH轴进行参数配置，其中YAW轴控制偏航方向，PITCH轴控制俯仰方向。
 *
 * 参数配置说明:



 */
void PidInit(void)
{
    PID_struct_init(&Chassis_Power, POSITION_PID, 1.0f, 0.0, 0.021f, 0.000f, 0.0f);

    PID_struct_init(&Automatic_aiming_pos, POSITION_PID, 5, 0.8, 0.8f, 0.060f, 1.00f);
    PID_struct_init(&Automatic_aiming_spd, POSITION_PID, 4, 1, 0.050f, 0.00f, 0.0f);


    PID_struct_init(&Chassis_Rotate, POSITION_PID, 5000.0, 10, 4000.0f, 0.00f, 10.0f);

    PID_struct_init(&xMotorPid[POS][ECO][YAW], POSITION_PID, 1000, 8, 12.0f, 0.00f, 2.0f);
    PID_struct_init(&xMotorPid[SPD][ECO][YAW], POSITION_PID, 10, 1, 1.22f, 0.004f, 2.0f);
    PID_struct_init(&xMotorPid[POS][IMU][YAW], POSITION_PID, 1000, 10, 8.0f, 0.0f, 0.2f);
    PID_struct_init(&xMotorPid[SPD][IMU][YAW], POSITION_PID, 10, 1, 0.08f, 0.00f, 0.0f);

    PID_struct_init(&xMotorPid[POS][ECO][PITCH], POSITION_PID, 10, 1, 10.0f, 0.000f, 0.1f);
    PID_struct_init(&xMotorPid[SPD][ECO][PITCH], POSITION_PID, 10, 1, 0.8f, 0.00f, 0.0f);
    PID_struct_init(&xMotorPid[POS][IMU][PITCH], POSITION_PID, 25, 1,20.0f, 0.00f, 0.0f);
    PID_struct_init(&xMotorPid[SPD][IMU][PITCH], POSITION_PID, 10, 1, 0.88f, 0.02f, 0.0f);

    PID_struct_init(&xMotorPid[SPD][ECO][Height], POSITION_PID, 16000, 500, 20.0f, 0.2f, 0.0f);

    PID_struct_init(&xMotorPid[SPD][ECO][WHEEL_FR], POSITION_PID, 10000, 500, 10.0f, 0.2f, 0.0f);
    PID_struct_init(&xMotorPid[SPD][ECO][WHEEL_FL], POSITION_PID, 10000, 500, 10.0f, 0.2f, 0.0f);
    PID_struct_init(&xMotorPid[SPD][ECO][WHEEL_BR], POSITION_PID, 10000, 500, 10.0f, 0.2f, 0.0f);
    PID_struct_init(&xMotorPid[SPD][ECO][WHEEL_BL], POSITION_PID, 10000, 500, 10.0f, 0.2f, 0.0f);

    PID_struct_init(&xMotorPid[SPD][ECO][FRICTR], POSITION_PID, 16384, 10000, 16.0f, 0.05f, 2.0f);
    PID_struct_init(&xMotorPid[SPD][ECO][FRICTL], POSITION_PID, 16384, 10000, 16.0f, 0.05f, 2.0f);


    PID_struct_init(&xMotorPid[POS][ECO][PLUCK], POSITION_PID, 10000, 8000, 0.38f, 0.0f, 0.0f);
    PID_struct_init(&xMotorPid[SPD][ECO][PLUCK], POSITION_PID, 15000, 1000, 12.0f, 0.0f, 0.0f);//10000, 6000, 8.0f, 1.9f, 3.0f



}

/**********************************************************************************************************************
 * @brief   calculate delta PID and position PID
 * @param   pid: PID结构体指针
 *          set: 目标值
 *          get: 实际测量值
 * @retval  PID计算输出结果
 * @others  PID教程参考: https://blog.csdn.net/as480133937/article/details/89508034
**********************************************************************************************************************/
float pid_calc(pid_t* pid, float get, float set)
{

    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	// 计算目标值与实际值之间的误差
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err)
        return 0;   // 超过最大误差范围时不进行PID计算
    if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
        return 0;  // 误差在死区范围内时不进行PID计算

    if(pid->pid_mode == POSITION_PID) // 位置式PID计算
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
        abs_limit(&(pid->iout), pid->IntegralLimit);  // 积分限幅
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);  // 输出限幅
        pid->last_pos_out = pid->pos_out;	// 更新上次输出值
    }
    else if(pid->pid_mode == DELTA_PID) // 增量式PID计算
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);

        abs_limit(&(pid->iout), pid->IntegralLimit); // 积分限幅
        pid->delta_u = pid->pout + pid->iout + pid->dout; // 计算增量值
        pid->delta_out = pid->last_delta_out + pid->delta_u; // 累加得到输出值
        abs_limit(&(pid->delta_out), pid->MaxOutput); // 输出限幅
        pid->last_delta_out = pid->delta_out;	// 更新上次输出值
    }

    // 更新历史数据
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out; // 返回PID计算结果

}


