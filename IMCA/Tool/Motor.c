//
// Created by zn1135 on 25-9-24.
//

#include "Motor.h"
#include "IMU_Task.h"
#include "pid.h"
#include "user_lib.h"
volatile double TempCalc;
// 电机参数数组，用于存储所有电机的参数
motor_param_t xMotorParam[MOTOR_NUM];
moto_eco_measure_t xMotorEcoMeasure[MOTOR_NUM];
DM_J4310_measure_t xMotorDMMeasure[MOTOR_NUM];
moto_imu_measure_t xMotorImuMeasure[MOTOR_NUM];
pid_t xMotorPid[2][2][MOTOR_NUM];
/**
 * @brief 电机初始化函数，使用属性允许用户在应用层覆盖
 * @note   此函数当前为空实现，需要在具体应用中根据实际需求进行实现
 */
void MotorInit(void)
{

    xMotorParam[WHEEL_FR].ID = 0x201;
    xMotorParam[WHEEL_FR].Name = "WHEEL_FR";
    xMotorParam[WHEEL_FR].FeedbackMode = EcoFeedback;
    xMotorParam[WHEEL_FR].CloseLoopMode = SpdClsloop;
    xMotorParam[WHEEL_FR].MeasureEco = &xMotorEcoMeasure[WHEEL_FR];
    xMotorParam[WHEEL_FR].MeasureImu = &xMotorImuMeasure[WHEEL_FR];
    xMotorParam[WHEEL_FR].TotalAngleState = 0;
    xMotorParam[WHEEL_FR].PidParamPosEco = &xMotorPid[POS][ECO][WHEEL_FR];
    xMotorParam[WHEEL_FR].PidParamSpdEco = &xMotorPid[SPD][ECO][WHEEL_FR];
    xMotorParam[WHEEL_FR].PidParamPosImu = &xMotorPid[POS][IMU][WHEEL_FR];
    xMotorParam[WHEEL_FR].PidParamSpdImu = &xMotorPid[SPD][IMU][WHEEL_FR];
    xMotorParam[WHEEL_FR].MotorType = DJI;
    xMotorParam[WHEEL_FR].CanBus = Can1;

    xMotorParam[WHEEL_FL].ID = 0x202;
    xMotorParam[WHEEL_FL].Name = "WHEEL_FL";
    xMotorParam[WHEEL_FL].FeedbackMode = EcoFeedback;
    xMotorParam[WHEEL_FL].CloseLoopMode = SpdClsloop;
    xMotorParam[WHEEL_FL].MeasureEco = &xMotorEcoMeasure[WHEEL_FL];
    xMotorParam[WHEEL_FL].MeasureImu = &xMotorImuMeasure[WHEEL_FL];
    xMotorParam[WHEEL_FL].TotalAngleState = 0;
    xMotorParam[WHEEL_FL].PidParamPosEco = &xMotorPid[POS][ECO][WHEEL_FL];
    xMotorParam[WHEEL_FL].PidParamSpdEco = &xMotorPid[SPD][ECO][WHEEL_FL];
    xMotorParam[WHEEL_FL].PidParamPosImu = &xMotorPid[POS][IMU][WHEEL_FL];
    xMotorParam[WHEEL_FL].PidParamSpdImu = &xMotorPid[SPD][IMU][WHEEL_FL];
    xMotorParam[WHEEL_FL].MotorType = DJI;
    xMotorParam[WHEEL_FL].CanBus = Can1;

    xMotorParam[WHEEL_BR].ID = 0x203;
    xMotorParam[WHEEL_BR].Name = "WHEEL_BR";
    xMotorParam[WHEEL_BR].FeedbackMode = EcoFeedback;
    xMotorParam[WHEEL_BR].CloseLoopMode = SpdClsloop;
    xMotorParam[WHEEL_BR].MeasureEco = &xMotorEcoMeasure[WHEEL_BR];
    xMotorParam[WHEEL_BR].MeasureImu = &xMotorImuMeasure[WHEEL_BR];
    xMotorParam[WHEEL_BR].TotalAngleState = 0;
    xMotorParam[WHEEL_BR].PidParamPosEco = &xMotorPid[POS][ECO][WHEEL_BR];
    xMotorParam[WHEEL_BR].PidParamSpdEco = &xMotorPid[SPD][ECO][WHEEL_BR];
    xMotorParam[WHEEL_BR].PidParamPosImu = &xMotorPid[POS][IMU][WHEEL_BR];
    xMotorParam[WHEEL_BR].PidParamSpdImu = &xMotorPid[SPD][IMU][WHEEL_BR];
    xMotorParam[WHEEL_BR].MotorType = DJI;
    xMotorParam[WHEEL_BR].CanBus = Can1;

    xMotorParam[WHEEL_BL].ID = 0x204;
    xMotorParam[WHEEL_BL].Name = "WHEEL_BL";
    xMotorParam[WHEEL_BL].FeedbackMode = EcoFeedback;
    xMotorParam[WHEEL_BL].CloseLoopMode = SpdClsloop;
    xMotorParam[WHEEL_BL].MeasureEco = &xMotorEcoMeasure[WHEEL_BL];
    xMotorParam[WHEEL_BL].MeasureImu = &xMotorImuMeasure[WHEEL_BL];
    xMotorParam[WHEEL_BL].TotalAngleState = 0;
    xMotorParam[WHEEL_BL].PidParamPosEco = &xMotorPid[POS][ECO][WHEEL_BL];
    xMotorParam[WHEEL_BL].PidParamSpdEco = &xMotorPid[SPD][ECO][WHEEL_BL];
    xMotorParam[WHEEL_BL].PidParamPosImu = &xMotorPid[POS][IMU][WHEEL_BL];
    xMotorParam[WHEEL_BL].PidParamSpdImu = &xMotorPid[SPD][IMU][WHEEL_BL];
    xMotorParam[WHEEL_BL].MotorType = DJI;
    xMotorParam[WHEEL_BL].CanBus = Can1;

    xMotorParam[YAW].ID = 0x01;
    xMotorParam[YAW].Name = "YAW";
    xMotorParam[YAW].FeedbackMode = EcoFeedback;
    xMotorParam[YAW].CloseLoopMode = PosClsloop;
    xMotorParam[YAW].MeasureDM = &xMotorDMMeasure[YAW];
    xMotorParam[YAW].MeasureImu = &xMotorImuMeasure[YAW];
    xMotorParam[YAW].MeasureDM->mode = 1;
    xMotorParam[YAW].PidParamPosEco = &xMotorPid[POS][ECO][YAW];
    xMotorParam[YAW].PidParamSpdEco = &xMotorPid[SPD][ECO][YAW];
    xMotorParam[YAW].PidParamPosImu = &xMotorPid[POS][IMU][YAW];
    xMotorParam[YAW].PidParamSpdImu = &xMotorPid[SPD][IMU][YAW];
    xMotorParam[YAW].MotorType = DM;
    xMotorParam[YAW].CanBus = Can3;
    // xMotorParam[YAW].MeasureDM ->reverse_derection = 1;

    xMotorParam[PITCH].ID = 0x02;
    xMotorParam[PITCH].Name = "PITCH";
    xMotorParam[PITCH].FeedbackMode = EcoFeedback;
    xMotorParam[PITCH].CloseLoopMode = PosClsloop;
    xMotorParam[PITCH].MeasureDM = &xMotorDMMeasure[PITCH];
    xMotorParam[PITCH].MeasureImu = &xMotorImuMeasure[PITCH];
    xMotorParam[PITCH].MeasureDM->mode = 1;
    xMotorParam[PITCH].PidParamPosEco = &xMotorPid[POS][ECO][PITCH];
    xMotorParam[PITCH].PidParamSpdEco = &xMotorPid[SPD][ECO][PITCH];
    xMotorParam[PITCH].PidParamPosImu = &xMotorPid[POS][IMU][PITCH];
    xMotorParam[PITCH].PidParamSpdImu = &xMotorPid[SPD][IMU][PITCH];
    xMotorParam[PITCH].MotorType = DM;
    xMotorParam[PITCH].CanBus = Can3;

    xMotorParam[Height].ID = 0x205;
    xMotorParam[Height].Name = "Height";
    xMotorParam[Height].FeedbackMode = EcoFeedback;
    xMotorParam[Height].CloseLoopMode = SpdClsloop;
    xMotorParam[Height].MeasureEco = &xMotorEcoMeasure[Height];
    xMotorParam[Height].MeasureImu = &xMotorImuMeasure[Height];
    xMotorParam[Height].TotalAngleState = 1;
    xMotorParam[Height].PidParamPosEco = &xMotorPid[POS][ECO][Height];
    xMotorParam[Height].PidParamSpdEco = &xMotorPid[SPD][ECO][Height];
    xMotorParam[Height].PidParamPosImu = &xMotorPid[POS][IMU][Height];
    xMotorParam[Height].PidParamSpdImu = &xMotorPid[SPD][IMU][Height];
    xMotorParam[Height].MotorType = DJI;
    xMotorParam[Height].CanBus = Can2;

    xMotorParam[FRICTR].ID = 0x206;
    xMotorParam[FRICTR].Name = "FRICTR";
    xMotorParam[FRICTR].FeedbackMode = EcoFeedback;
    xMotorParam[FRICTR].CloseLoopMode = SpdClsloop;
    xMotorParam[FRICTR].MeasureEco = &xMotorEcoMeasure[FRICTR];
    xMotorParam[FRICTR].MeasureImu = &xMotorImuMeasure[FRICTR];
    xMotorParam[FRICTR].TotalAngleState = 1;
    xMotorParam[FRICTR].PidParamPosEco = &xMotorPid[POS][ECO][FRICTR];
    xMotorParam[FRICTR].PidParamSpdEco = &xMotorPid[SPD][ECO][FRICTR];
    xMotorParam[FRICTR].PidParamPosImu = &xMotorPid[POS][IMU][FRICTR];
    xMotorParam[FRICTR].PidParamSpdImu = &xMotorPid[SPD][IMU][FRICTR];
    xMotorParam[FRICTR].MotorType = DJI;
    xMotorParam[FRICTR].CanBus = Can2;

    xMotorParam[FRICTL].ID = 0x207;
    xMotorParam[FRICTL].Name = "FRICTL";
    xMotorParam[FRICTL].FeedbackMode = EcoFeedback;
    xMotorParam[FRICTL].CloseLoopMode = SpdClsloop;
    xMotorParam[FRICTL].MeasureEco = &xMotorEcoMeasure[FRICTL];
    xMotorParam[FRICTL].MeasureImu = &xMotorImuMeasure[FRICTL];
    xMotorParam[FRICTL].TotalAngleState = 1;
    xMotorParam[FRICTL].PidParamPosEco = &xMotorPid[POS][ECO][FRICTL];
    xMotorParam[FRICTL].PidParamSpdEco = &xMotorPid[SPD][ECO][FRICTL];
    xMotorParam[FRICTL].PidParamPosImu = &xMotorPid[POS][IMU][FRICTL];
    xMotorParam[FRICTL].PidParamSpdImu = &xMotorPid[SPD][IMU][FRICTL];
    xMotorParam[FRICTL].MotorType = DJI;
    xMotorParam[FRICTL].CanBus = Can2;

    xMotorParam[PLUCK].ID = 0x208;
    xMotorParam[PLUCK].Name = "PLUCK";
    xMotorParam[PLUCK].FeedbackMode = EcoFeedback;
    xMotorParam[PLUCK].CloseLoopMode = SpdClsloop;
    xMotorParam[PLUCK].MeasureEco = &xMotorEcoMeasure[PLUCK];
    xMotorParam[PLUCK].MeasureImu = &xMotorImuMeasure[PLUCK];
    xMotorParam[PLUCK].TotalAngleState = 1;
    xMotorParam[PLUCK].PidParamPosEco = &xMotorPid[POS][ECO][PLUCK];
    xMotorParam[PLUCK].PidParamSpdEco = &xMotorPid[SPD][ECO][PLUCK];
    xMotorParam[PLUCK].PidParamPosImu = &xMotorPid[POS][IMU][PLUCK];
    xMotorParam[PLUCK].PidParamSpdImu = &xMotorPid[SPD][IMU][PLUCK];
    xMotorParam[PLUCK].MotorType = DJI;
    xMotorParam[PLUCK].CanBus = Can2;
}
/*MIT模式下*/
/**********************************************************************************************************************
  * @brief  MIT命令采用浮点数据等比例转换成整数发送到驱动器，
            驱动器再将接收到的证书等比例转换成浮点数据。
  * @param
  *
  * @retval
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * @Func	 float uint_to_float(int x_int, float x_min, float x_max, int bits)
 * @Brief   将无符号整数转换为浮点数
 * @Param	 int x_int：输入的整数值
 *          float x_min：输出浮点数的最小值
 *          float x_max：输出浮点数的最大值
 *          int bits：输入整数的位数
 * @Retval	 float：转换后的浮点数值
 * @Note    该函数根据给定的范围和位数将整数线性映射到浮点数范围
 *********************************************************************************************************************/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**********************************************************************************************************************
 * @Func	 int float_to_uint(float x, float x_min, float x_max, int bits)
 * @Brief   将浮点数转换为无符号整数
 * @Param	 float x：输入的浮点数值
 *          float x_min：输入浮点数的最小值
 *          float x_max：输入浮点数的最大值
 *          int bits：输出整数的位数
 * @Retval	 int：转换后的整数值
 * @Note    该函数根据给定的范围和位数将浮点数线性映射到整数范围
 *********************************************************************************************************************/
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits///
    if (x < x_min)
        x = x_min;
    else if (x > x_max)
        x = x_max;
    // 不作越界处理会电机反转
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**********************************************************************************************************************
  * @Func	 void PidCalcIMU(motor_param_t * ptr)
  * @Brief   基于IMU反馈的电机PID计算
  * @Param	 motor_param_t *ptr：电机参数结构体指针
  * @Retval	 无
  * @Note    根据电机类型(DJI/DM)和闭环模式(位置/速度)进行位置-速度串级PID或速度单环PID计算，
            计算结果经反向处理后写入CurrentOutput
*********************************************************************************************************************/
void PidCalcIMU(motor_param_t *ptr)
{
    switch (ptr->MotorType)
    {
    case DJI: {
        if (!ptr->TotalAngleState)
        {
            TempCalc = pid_calc(ptr->PidParamPosImu, ptr->MeasureImu->angle, ptr->ImuTar);
            ptr->TempCalc = TempCalc;
            TempCalc = pid_calc(ptr->PidParamSpdImu, ptr->MeasureImu->palstance, TempCalc);
        }
        else if (ptr->TotalAngleState)
        {
            TempCalc = pid_calc(ptr->PidParamPosImu, ptr->MeasureImu->total_angle, ptr->ImuTar);
            ptr->TempCalc = TempCalc;
            TempCalc = pid_calc(ptr->PidParamSpdImu, ptr->MeasureImu->palstance, TempCalc);
        }
    }
    break;
    case DM: {
        if (ptr->MeasureDM->mode == 0)
        {
            if (!ptr->TotalAngleState)
            {
                TempCalc = pid_calc(ptr->PidParamPosImu, ptr->MeasureImu->angle, ptr->ImuTar);
                ptr->TempCalc = TempCalc;
            }
            else if (ptr->TotalAngleState)
            {
                TempCalc = pid_calc(ptr->PidParamPosImu, ptr->MeasureImu->total_angle, ptr->ImuTar);
                ptr->TempCalc = TempCalc;
            }
        }
        else if (ptr->MeasureDM->mode == 1)
        {
            if (ptr->CloseLoopMode == PosClsloop)
            {
                if (ptr->TotalAngleState == 0)
                {
                    TempCalc = pid_calc(ptr->PidParamPosImu, ptr->MeasureImu->angle, ptr->ImuTar);
                    ptr->TempCalc = TempCalc;
                    TempCalc = pid_calc(ptr->PidParamSpdImu, ptr->MeasureImu->palstance, TempCalc);
                }
                else if (ptr->TotalAngleState == 1)
                {
                    TempCalc = pid_calc(ptr->PidParamPosImu, ptr->MeasureImu->total_angle, ptr->ImuTar);
                    ptr->TempCalc = TempCalc;
                    TempCalc = pid_calc(ptr->PidParamSpdImu, ptr->MeasureImu->palstance, TempCalc);
                }

                else
                    ;
            }
            else if (ptr->CloseLoopMode == SpdClsloop)
            {
                TempCalc = pid_calc(ptr->PidParamSpdImu, ptr->MeasureImu->palstance, ptr->ImuTar);
            }
        }
    }
    break;
    }
    if (ptr->MeasureImu->reverse_derection == 1)
        ptr->CurrentOutput = -TempCalc;
    else
    {
        ptr->CurrentOutput = TempCalc;
    }
}

/**********************************************************************************************************************
  * @Func	 void PidCalcEco(motor_param_t * ptr)
  * @Brief   基于编码器反馈的电机PID计算
  * @Param	 motor_param_t *ptr：电机参数结构体指针
  * @Retval	 无
  * @Note    根据电机类型(DJI/DM)和闭环模式(位置/速度)进行位置-速度串级PID或速度单环PID计算，
            计算结果经反向处理后写入CurrentOutput
*********************************************************************************************************************/
void PidCalcEco(motor_param_t *ptr)
{
    switch (ptr->MotorType)
    {
    case DJI: {
        if (ptr->CloseLoopMode == PosClsloop)
        {
            if (!ptr->TotalAngleState)
            {
                TempCalc = pid_calc(ptr->PidParamPosEco, ptr->MeasureEco->angle, ptr->EcoTar);
                ptr->TempCalc = TempCalc;
                TempCalc = pid_calc(ptr->PidParamSpdEco, ptr->MeasureEco->speed_rpm, TempCalc);
            }
            else if (ptr->TotalAngleState)
            {
                TempCalc = pid_calc(ptr->PidParamPosEco, ptr->MeasureEco->total_angle, ptr->EcoTar);
                ptr->TempCalc = TempCalc;
                TempCalc = pid_calc(ptr->PidParamSpdEco, ptr->MeasureEco->speed_rpm, TempCalc);
            }
            /*FeedForward*/

            else
                ;
        }
        else
        {
            TempCalc = pid_calc(ptr->PidParamSpdEco, ptr->MeasureEco->speed_rpm, ptr->EcoTar);
            ptr->TempCalc = TempCalc;
        }

        ptr->CurrentOutput = TempCalc;
    }
    break;

    case DM: {
        if (ptr->MeasureDM->mode == 0)
        {
            if (ptr->CloseLoopMode == PosClsloop)
            {
                TempCalc = pid_calc(ptr->PidParamPosEco, ptr->MeasureDM->position, ptr->EcoTar);
                ptr->TempCalc = TempCalc;
                ptr->CurrentOutput = TempCalc;
            }
            else
            {
                ptr->CurrentOutput = ptr->EcoTar;
            }
        }
        else if (ptr->MeasureDM->mode == 1)
        {
            if (ptr->CloseLoopMode == PosClsloop)
            {
                TempCalc = pid_calc(ptr->PidParamPosEco, ptr->MeasureDM->position, ptr->EcoTar);
                ptr->TempCalc = TempCalc;
                TempCalc = pid_calc(ptr->PidParamSpdEco, ptr->MeasureDM->velocity, TempCalc);
            }
            else
            {
                TempCalc = pid_calc(ptr->PidParamSpdEco, ptr->MeasureDM->velocity, ptr->EcoTar);
                ptr->TempCalc = TempCalc;

                /*FeedForward*/
            }
            ptr->CurrentOutput = TempCalc;
        }
    }
    break;
    }
    if (ptr->MeasureDM->reverse_derection == 1)
        ptr->CurrentOutput = -TempCalc;
    else
        ptr->CurrentOutput = TempCalc;
}

/**********************************************************************************************************************
 * @Func	 void MotorCalc(motor_param_t * pxMotorParam)
 * @Brief   电机控制计算调度函数
 * @Param	 motor_param_t *pxMotorParam：电机参数结构体指针
 * @Retval	 无
 * @Note    根据反馈模式(EcoFeedback/IMUFeedback)选择调用对应的PID计算函数
 *********************************************************************************************************************/
void MotorCalc(motor_param_t *pxMotorParam)
{
    if (pxMotorParam->FeedbackMode == EcoFeedback)
    {
        PidCalcEco(pxMotorParam);
    }
    else
    {
        PidCalcIMU(pxMotorParam);
    }
}
