#include "PowerLimit.h"
#include "../Judge/Driver_Judge.h"
#include "CanBus_Task.h"
#include "Judge_interact.h"
// #include "../Judge/CapInteract.h"
#include "CapInteract.h"
#include "ChassisControl.h"
#include "Pid.h"
#include "RcTask.h"
#include "motor.h"
#include "user_lib.h"

#define ABS(x) ((x) > 0 ? (x) : (-(x))) // ����ֵ��

power_t power;

// model->k1 * ABS(model->real_current) * ABS(Chassis_Motor->MeasureEco->speed_rpm) +
//        model->k2 * ABS(Chassis_Motor->MeasureEco->speed_rpm) * ABS(Chassis_Motor->MeasureEco->speed_rpm) +
//        model->k3 * ABS(model->real_current) * ABS(model->real_current) +
//        model->a/4;
// k1, k2, k3, a, real_current, PowLim_IQ
// �������ģ�Ͳ�����ͨ����ϵõ���
model_t model[4] = {

    {2.513e-06, 2.861e-07, 2.747e-07, 9.28, 0, 0}, // ���� 1
    {2.539e-06, 1.932e-07, 2.571e-07, 9.37, 0, 0}, // ���� 2
    {2.689e-06, 2.283e-07, 2.766e-07, 9.14, 0, 0}, // ���� 3
    {2.399e-06, 2.415e-07, 2.915e-07, 10.47, 0, 0} // ���� 4
};

// // �������ݹ�����ϲ�����MATLAB ��ϵõ���
// cap_t cap_power = {0.0026, -0.0066, -0.13, -1.16, 84.79, 0};

/**********************************************************************************************************************
 * @Func	 float averageFilter(uint8_t a, float in_data)
 * @Brief   简单滑动平均滤波器
 * @Param	 uint8_t a：通道编号(0~3)
 *          float in_data：输入数据
 * @Retval	 float：滤波后的平均值
 * @Note    每路通道保留20个历史数据点，返回滑动平均值
 *********************************************************************************************************************/
float averageFilter(uint8_t a, float in_data)
{
    float sum = 0.0f;
    static float data[4][20]; // 4 ·��ÿ· 10 ����ʷ����

    for (int i = 0; i < 19; i++)
    {
        data[a][i] = data[a][i + 1];
        sum += data[a][i];
    }

    data[a][19] = in_data;
    sum += data[a][19];

    return sum / 20.0f;
}

/**********************************************************************************************************************
 * @Func	 double PowerMotorLimit(model_t *model, motor_param_t *Chassis_Motor)
 * @Brief   根据电机功率模型计算单个电机功率
 * @Param	 model_t *model：功率模型参数结构体指针
 *          motor_param_t *Chassis_Motor：底盘电机参数结构体指针
 * @Retval	 double：计算得到的电机功率值
 * @Note    使用公式 k1*I*w + k2*w^2 + k3*I^2 + a/4 计算功率
 *********************************************************************************************************************/
double PowerMotorLimit(model_t *model, motor_param_t *Chassis_Motor)
{
    // ԭʼ�汾��k1 * I * w + k2 * w^2 + a
    // ����ʹ�ò���ֵ MeasureEco �е�ת�������
    return model->k1 * (model->real_current) * (Chassis_Motor->MeasureEco->speed_rpm) +
           model->k2 * (Chassis_Motor->MeasureEco->speed_rpm) * (Chassis_Motor->MeasureEco->speed_rpm) +
           model->k3 * (model->real_current) * (model->real_current) +
           model->a / 4;
    // return model->k1 * ABS(model->real_current) * ABS(Chassis_Motor->MeasureEco->speed_rpm) +
    //    model->k2 * ABS(Chassis_Motor->MeasureEco->speed_rpm) * ABS(Chassis_Motor->MeasureEco->speed_rpm) +
    //    model->k3 * ABS(model->real_current) * ABS(model->real_current) +
    //    model->a/4;
}

/**********************************************************************************************************************
  * @Func	 void PowertotalLimit(void)
  * @Brief   底盘功率总限幅控制
  * @Param	 无
  * @Retval	 无
  * @Note    累加四个底盘电机功率，通过PID控制与裁判系统功率上限做比较，
            对每个底盘的CurrentOutput做等比例限幅
*********************************************************************************************************************/
void PowertotalLimit(void)
{
    // TODO: �������ù��������߼��������˺����е�ע�ͣ���ȷ����
    // 1. WHEEL_RF / WHEEL_LF / WHEEL_LB / WHEEL_RB �Ⱥ����ڵ���ģ���ж��壻
    // 2. xMotorParam��Cap_Measure��servo_state��KEY_CTRL��send_chassis_power_limit �ȷ������ڶ�Ӧģ��������/���壻

    power.motor = 0;
    // ����๦���ۼ�
    int i = 0;
    for (int i = 0; i < 4; i++)
    {

        model[i].real_current = averageFilter(i, xMotorParam[i + WHEEL_FR].MeasureEco->real_current);
        power.motor += PowerMotorLimit(&model[i], &xMotorParam[i + WHEEL_FR]);
    }
    //
    // �������ݹ������ܹ���
    // power.cap = PowerCapLimit(&Cap_Measure);
    power.total = 1.0f * power.motor + 0.0f * power.cap;
    //

    if ((Cap_Measure.cap_value / 10.0f >= 16.0f && Cap_Measure.state_core == 161) || KEY_CTRL)
    {
        pid_calc(&Chassis_Power, power.total, send_chassis_power_limit() + 15.0f * Cap_Measure.cap_value / 10.0f - 5);
    }
    else
    {
        pid_calc(&Chassis_Power, power.total, send_chassis_power_limit() - 5);
    }

    Chassis_Power.pos_out = (Chassis_Power.pos_out + 1.0f) / 3.0f;

    // �����ݵ�ѹ�ϸ��ҹ�������ϵ������ʱ����һ�����ޱ���
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
