#include "PowerLimit.h"
#include "CanBus_Task.h"
#include "../Judge/Driver_Judge.h"
#include "Judge_interact.h"
// #include "../Judge/CapInteract.h"
#include "CapInteract.h"
#include "Pid.h"
#include "ChassisControl.h"
#include "motor.h"
#include "RcTask.h"
#include "user_lib.h"

#define ABS(x) ((x) > 0 ? (x) : (-(x)))  // 绝对值宏

power_t power;

// model->k1 * ABS(model->real_current) * ABS(Chassis_Motor->MeasureEco->speed_rpm) +
//        model->k2 * ABS(Chassis_Motor->MeasureEco->speed_rpm) * ABS(Chassis_Motor->MeasureEco->speed_rpm) +
//        model->k3 * ABS(model->real_current) * ABS(model->real_current) +
//        model->a/4;
// k1, k2, k3, a, real_current, PowLim_IQ
// 电机功率模型参数（通过拟合得到）
model_t model[4] = {

    {2.513e-06, 2.861e-07,   2.747e-07, 9.28, 0, 0}, // 轮子 1
    {2.539e-06, 1.932e-07, 2.571e-07 , 9.37 , 0, 0},// 轮子 2
    {2.689e-06, 2.283e-07,  2.766e-07,  9.14, 0, 0},  // 轮子 3
    {2.399e-06, 2.415e-07, 2.915e-07, 10.47, 0, 0}  // 轮子 4
};

// // 超级电容功率拟合参数（MATLAB 拟合得到）
// cap_t cap_power = {0.0026, -0.0066, -0.13, -1.16, 84.79, 0};

// 简单滑动平均滤波，a 表示通道索引（0~3）
float averageFilter(uint8_t a, float in_data)
{
    float sum = 0.0f;
    static float data[4][20]; // 4 路，每路 10 点历史数据

    for (int i = 0; i < 19; i++)
    {
        data[a][i] = data[a][i + 1];
        sum += data[a][i];
    }

    data[a][19] = in_data;
    sum += data[a][19];

    return sum / 20.0f;
}

// 单个电机功率估算模型
double PowerMotorLimit(model_t *model, motor_param_t *Chassis_Motor)
{
    // 原始版本：k1 * I * w + k2 * w^2 + a
    // 这里使用测量值 MeasureEco 中的转速与电流
    return model->k1 * (model->real_current) * (Chassis_Motor->MeasureEco->speed_rpm) +
           model->k2 * (Chassis_Motor->MeasureEco->speed_rpm) * (Chassis_Motor->MeasureEco->speed_rpm) +
           model->k3 * (model->real_current) * (model->real_current) +
           model->a/4;
    // return model->k1 * ABS(model->real_current) * ABS(Chassis_Motor->MeasureEco->speed_rpm) +
    //    model->k2 * ABS(Chassis_Motor->MeasureEco->speed_rpm) * ABS(Chassis_Motor->MeasureEco->speed_rpm) +
    //    model->k3 * ABS(model->real_current) * ABS(model->real_current) +
    //    model->a/4;

}

// 以下为整车功率限制逻辑，依赖多个外部符号（Cap_Measure、xMotorParam 等），如需启用请补齐依赖
void PowertotalLimit(void) {
    // TODO: 如需启用功率限制逻辑，请解除此函数中的注释，并确保：
    // 1. WHEEL_RF / WHEEL_LF / WHEEL_LB / WHEEL_RB 等宏已在底盘模块中定义；
    // 2. xMotorParam、Cap_Measure、servo_state、KEY_CTRL、send_chassis_power_limit 等符号已在对应模块中声明/定义；

    power.motor = 0;
    // 电机侧功率累加
    int i = 0;
    for (int i = 0; i < 4; i++)
    {

        model[i].real_current = averageFilter(i, xMotorParam[i + WHEEL_FR].MeasureEco->real_current);
        power.motor += PowerMotorLimit(&model[i], &xMotorParam[i + WHEEL_FR]);
    }
    //
    // 超级电容功率与总功率
    // power.cap = PowerCapLimit(&Cap_Measure);
    power.total = 1.0f * power.motor + 0.0f * power.cap;
    //

    if ((Cap_Measure.cap_value / 10.0f >= 16.0f &&Cap_Measure.state_core == 161)||KEY_CTRL)
    {
        pid_calc(&Chassis_Power, power.total, send_chassis_power_limit() + 15.0f*Cap_Measure.cap_value/10.0f-5);
    }
    else
    {
        pid_calc(&Chassis_Power, power.total, send_chassis_power_limit()-5);
    }

    Chassis_Power.pos_out = (Chassis_Power.pos_out + 1.0f) / 3.0f;

    // 当电容电压较高且功率限制系数过低时，给一个下限保护
    // if (Cap_Measure.cap_value / 10.0f >= 16.0f &&
    //     Chassis_Power.pos_out < 0.30f &&
    //     Cap_Measure.state_core == 161)
    // {
    //     Chassis_Power.pos_out = 0.30f;
                                                                   // }
    xMotorParam[WHEEL_FR].CurrentOutput = xMotorParam[WHEEL_FR].PidParamSpdEco->pos_out * Chassis_Power.pos_out;
    xMotorParam[WHEEL_FL].CurrentOutput = xMotorParam[WHEEL_FL].PidParamSpdEco->pos_out * Chassis_Power.pos_out;
    xMotorParam[WHEEL_BL].CurrentOutput = xMotorParam[WHEEL_BL].PidParamSpdEco->pos_out * Chassis_Power.pos_out;
    xMotorParam[WHEEL_BR].CurrentOutput = xMotorParam[WHEEL_BR].PidParamSpdEco->pos_out * Chassis_Power.pos_out;
}
