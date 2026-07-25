//
// Created by zn1135 on 25-11-29.
//

#include "GimbalControl.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "ControlTask.h"
#include "IMU_Task.h"
#include "Motor.h"
#include "Ramp.h"
#include "RcTask.h"
#include "ShootControl.h"
#include "Vision_interact.h"
#include "arm_math.h"
#include "main.h"
#include "user_lib.h"

#ifndef GIMBAL_YAW_ESO_ENABLE
#define GIMBAL_YAW_ESO_ENABLE 1
#endif

#define GIMBAL_YAW_ESO_BANDWIDTH 8.0f
#define GIMBAL_YAW_ESO_COMP_GAIN 0.5f
#define GIMBAL_YAW_ESO_COMP_LIMIT 1.5f
#define GIMBAL_YAW_ESO_COMP_LPF_HZ 5.0f
#define GIMBAL_YAW_ESO_DEFAULT_DT 0.006f
#define GIMBAL_YAW_ESO_MIN_DT 0.0005f
#define GIMBAL_YAW_ESO_MAX_DT 0.020f

float yaw_eso_z1;
float yaw_eso_z2;
float yaw_eso_z3;
float yaw_eso_comp_filt;
volatile float yaw_eso_tau_applied;

#if GIMBAL_YAW_ESO_ENABLE
static float yaw_eso_last_timestamp_ms;
static uint8_t yaw_eso_ready;
static uint8_t yaw_eso_timestamp_ready;

static void YawEsoReset(void)
{
    yaw_eso_z1 = 0.0f;
    yaw_eso_z2 = 0.0f;
    yaw_eso_z3 = 0.0f;
    yaw_eso_comp_filt = 0.0f;
    yaw_eso_tau_applied = 0.0f;
    yaw_eso_last_timestamp_ms = 0.0f;
    yaw_eso_ready = 0U;
    yaw_eso_timestamp_ready = 0U;
}

static float YawEsoGetDt(void)
{
    const float now_ms = DWT_GetMonotonicTimestampMs();
    float dt;

    if (!yaw_eso_timestamp_ready)
    {
        yaw_eso_last_timestamp_ms = now_ms;
        yaw_eso_timestamp_ready = 1U;
        return GIMBAL_YAW_ESO_DEFAULT_DT;
    }

    dt = (now_ms - yaw_eso_last_timestamp_ms) * 0.001f;
    yaw_eso_last_timestamp_ms = now_ms;
    if (!(dt >= GIMBAL_YAW_ESO_MIN_DT && dt <= GIMBAL_YAW_ESO_MAX_DT))
    {
        dt = GIMBAL_YAW_ESO_DEFAULT_DT;
    }
    return dt;
}

static void YawEsoUpdate(float theta_measured, float tau_applied, float dt)
{
    const float w0 = GIMBAL_YAW_ESO_BANDWIDTH;
    const float b0 = 1.0f / GIMBAL_YAW_J;
    const float damping = GIMBAL_YAW_B / GIMBAL_YAW_J;
    float error;

    if (!yaw_eso_ready)
    {
        yaw_eso_z1 = theta_measured;
        yaw_eso_z2 = 0.0f;
        yaw_eso_z3 = 0.0f;
        yaw_eso_comp_filt = 0.0f;
        yaw_eso_ready = 1U;
        return;
    }

    error = theta_measured - yaw_eso_z1;
    yaw_eso_z1 += dt * (yaw_eso_z2 + 3.0f * w0 * error);
    yaw_eso_z2 += dt * (-damping * yaw_eso_z2 + b0 * tau_applied +
                         yaw_eso_z3 + 3.0f * w0 * w0 * error);
    yaw_eso_z3 += dt * (w0 * w0 * w0 * error);

    if (!isfinite(yaw_eso_z1) || !isfinite(yaw_eso_z2) || !isfinite(yaw_eso_z3))
    {
        YawEsoReset();
    }
}

static float YawEsoCompensation(float dt)
{
    const float rc = 1.0f / (2.0f * PI * GIMBAL_YAW_ESO_COMP_LPF_HZ);
    const float alpha = dt / (dt + rc);
    float compensation;

    if (!yaw_eso_ready)
    {
        return 0.0f;
    }

    compensation = -GIMBAL_YAW_ESO_COMP_GAIN * yaw_eso_z3 * GIMBAL_YAW_J;
    yaw_eso_comp_filt += alpha * (compensation - yaw_eso_comp_filt);
    compensation = yaw_eso_comp_filt;
    abs_limit(&compensation, GIMBAL_YAW_ESO_COMP_LIMIT);
    return compensation;
}
#endif

void YawEsoRecordAppliedTorque(float torque)
{
    yaw_eso_tau_applied = torque;
}

float yaw_pos, yaw_acc, yaw_vel, yaw_output;
float pitch_pos, pitch_acc, pitch_vel, pitch_output; // 自瞄控制参数，暂时没用到，后续可以根据需要调整
float D_speed_yaw, gyrosc_speed;                     // 无用设计
float D_speed_pitch;                                 // 无用设计
float vision_pitch = 0;                              // 视觉系统提供的目标Pitch角度，应该也是没用的
float vision_yaw = 0;                                // 视觉系统提供的目标Yaw角度，应该也是没用的
uint16_t Stuck_Hight_cnt = 0;
uint8_t Stuck_Hight_flag = 0;
// 高低位动作标志：0-空闲，1-执行到High，2-执行到Low
uint8_t highlow_flag = 0;
int32_t High_Low_Start_angle = 0;
/**********************************************************************************************************************
 * @Func	 void GimbalInit(void)
 * @Brief   云台初始化函数
 * @Param	 无
 * @Retval	 无
 * @Note    将Pitch/YAW拉到安全初始位置，等待电机稳定后切换到IMU反馈闭环并校准角度偏移
 *********************************************************************************************************************/
void GimbalInit(void)
{
    static uint16_t Gimbal_Init_count = 0;
    // 上电先将Pitch/Yaw拉到安全初始目标位，等待电机稳定
    xMotorParam[PITCH].EcoTar = 0;
    MotorCalc(&xMotorParam[PITCH]);
    xMotorParam[YAW].EcoTar = Gimbal_FOLLOWANGLE;
    MotorCalc(&xMotorParam[YAW]);

    if (abs(xMotorParam[PITCH].MeasureDM->position) <= 0.05 && xMotorParam[PITCH].MeasureDM->DM_J4310_InitFlag && abs(xMotorParam[YAW].MeasureDM->position - xMotorParam[YAW].EcoTar) <= 0.05 && xMotorParam[YAW].MeasureDM->DM_J4310_InitFlag)
    {
        Gimbal_Init_count++;
        if (Gimbal_Init_count >= 200)
        {
            Gimbal_Init_Flag = 1;
            Gimbal_Init_count = 0;
            // 初始化完成后切到IMU闭环，并重置角度参考
            xMotorParam[YAW].FeedbackMode = ImuFeedback;
            xMotorParam[YAW].MeasureImu->offset_angle = xMotorParam[YAW].MeasureImu->angle;
            xMotorParam[PITCH].MeasureImu->offset_angle = xMotorParam[PITCH].MeasureImu->angle;
            xMotorParam[YAW].TotalAngleState = 1;
            xMotorParam[PITCH].TotalAngleState = 0;
            xMotorParam[YAW].MeasureImu->reverse_derection = 0;
            xMotorParam[PITCH].MeasureImu->reverse_derection = 1;
        }
    }
}
int znlook[10];

/**********************************************************************************************************************
 * @Func	 void GimbalControlSetValue(void)
 * @Brief   云台控制量给定
 * @Param	 无
 * @Retval	 无
 * @Note    处理RC/PC模式下的YAW/PITCH角度控制和自瞄逻辑，以及升降限位控制
 *********************************************************************************************************************/
void GimbalControlSetValue(void)
{

    static int AngleSaveFlag = 1, AutoSaveAngle = 1;
#if GIMBAL_YAW_ESO_ENABLE
    uint8_t yaw_eso_active = 0U;
#endif
    if (AngleSaveFlag)
    {
        AngleSaveFlag = 0;
        // 进入可控状态时锁定当前角度，避免模式切换时目标跳变
        xMotorParam[PITCH].EcoTar = xMotorParam[PITCH].MeasureDM->position;
        //		xMotorParam[PITCH].ImuTar = xMotorParam[PITCH].MeasureImu->angle;
        xMotorParam[YAW].ImuTar = xMotorParam[YAW].MeasureImu->total_angle;
    }
    if (xRobotMode.CtrlMode == LOSE)
    {
        // 遥控丢失后，下次恢复控制时重新保存目标角
        AngleSaveFlag = 1;
    }
    else if (xRobotMode.CtrlMode == RC)
    {
        // RC模式下：仅高位开放云台角度控制，低位回收到安全姿态
        if (Gimbal_Height_flag == High)
        {
            if (xMotorParam[YAW].FeedbackMode == EcoFeedback)
            {
                xMotorParam[YAW].ImuTar = xMotorParam[YAW].MeasureImu->total_angle;
                xMotorParam[YAW].FeedbackMode = ImuFeedback;
            }
            xMotorParam[PITCH].EcoTar -= RcDeadlineLimit(CH4, 25) * 0.000008;
            xMotorParam[YAW].ImuTar -= RcDeadlineLimit(CH3, 25) * 0.0008;
        }
        else
        {
            // xMotorParam[PITCH].EcoTar=-0f;
            if (xMotorParam[YAW].FeedbackMode == ImuFeedback)
            {
                xMotorParam[YAW].EcoTar = Gimbal_FrontAngle;
                xMotorParam[YAW].FeedbackMode = EcoFeedback;
            }
        }
        //				xMotorParam[PITCH].ImuTar += RcDeadlineLimit(CH4,25)*0.0005;
        if (CH5 < -500 && Gimbal_Height_flag != High && xRobotMode.ChassisMode == CHASSIS_FOLLOW)
        {
            xMotorParam[Height].EcoTar = -5000.0f;
        }
        else if (CH5 > 500 && Gimbal_Height_flag != Low && xRobotMode.ChassisMode == CHASSIS_FOLLOW)
        {
            xMotorParam[Height].EcoTar = 5000.0f;
            xMotorParam[PITCH].EcoTar = -0.23f;
        }
        else
        {
            xMotorParam[Height].EcoTar = 0.0f;
        }

        //		xMotorParam[YAW].EcoTar -= RcDeadlineLimit(CH3,25)*0.00002;
    }
    else if (xRobotMode.CtrlMode == PC)
    {

        // if (KEY_SHIFT&&KEY_X&&Gimbal_Height_flag!=High&&xRobotMode.ChassisMode ==CHASSIS_FOLLOW) {
        //     xMotorParam[Height].EcoTar=-15000.0f;
        // }else if(KEY_CTRL&&KEY_X&&Gimbal_Height_flag!=Low&&xRobotMode.ChassisMode ==CHASSIS_FOLLOW) {
        //     xMotorParam[Height].EcoTar=15000.0f;
        //     xRobotMode.ChassisMode =CHASSIS_FOLLOW;
        // }else {
        //     xMotorParam[Height].EcoTar=0.0f;
        // }
        if (xRobotMode.GimbalMode == NOSHOOT_MODE || xRobotMode.GimbalMode == NORMAL_MODE)
        {
            // 右键/拨杆/快捷键触发自瞄通道
            if ((mouse_right.deshake_key || (CH5 > 500) || KEY_V) && Gimbal_Height_flag == High)
            {
                // AUTO_Shoot_flag = 2;
                if (xMotorParam[PITCH].FeedbackMode == EcoFeedback)
                {
                    // Pitch从编码器环切入IMU环时，先对齐目标角
                    xMotorParam[PITCH].ImuTar = xMotorParam[PITCH].MeasureImu->angle;
                    //					xMotorParam[PITCH].EcoTar = xMotorParam[PITCH].MeasureDM->position;
                }
                xMotorParam[PITCH].FeedbackMode = ImuFeedback;
                if (xMotorParam[YAW].TotalAngleState == 1)
                {
                    // Yaw由总角模式切到相对角模式，先同步当前姿态
                    xMotorParam[YAW].ImuTar = xMotorParam[YAW].MeasureImu->angle;
                    xMotorParam[YAW].TotalAngleState = 0;
                }
                if (MiniPC_ReceivePacket.fireflag != 0)
                {
#if GIMBAL_YAW_ESO_ENABLE
                    float yaw_eso_compensation = 0.0f;
                    if (xMotorParam[YAW].MeasureDM->ERROR == 1)
                    {
                        const float yaw_eso_dt = YawEsoGetDt();
                        yaw_eso_active = 1U;
                        YawEsoUpdate(xMotorParam[YAW].MeasureImu->angle / 180.0f * PI,
                                     yaw_eso_tau_applied,
                                     yaw_eso_dt);
                        yaw_eso_compensation = YawEsoCompensation(yaw_eso_dt);
                    }
#endif

                    AUTO_Shoot_flag = 2;
                    xMotorParam[PITCH].ImuTar = -MiniPC_ReceivePacket.pitch;
                    vision_yaw = MiniPC_ReceivePacket.yaw;

                    yaw_pos = MiniPC_ReceivePacket.yaw;
                    xMotorParam[YAW].ImuTar = MiniPC_ReceivePacket.yaw;
                    // yaw_vel=MiniPC_ReceivePacket.yaw_vel;
                    TargetAngleProc(&xMotorParam[YAW]);

                    // 位置环+速度环+前馈组合，提高跟踪响应
                    pid_calc(&Automatic_aiming_pos, xMotorParam[YAW].MeasureImu->angle / 180 * PI, xMotorParam[YAW].ImuTar);
                    pid_calc(&Automatic_aiming_spd, xMotorParam[YAW].MeasureImu->palstance / 180 * PI, MiniPC_ReceivePacket.yaw_vel);
                    //     yaw_output=Moment_of_Inertia*(MiniPC_ReceivePacket.yaw_acc)+18.0f*(MiniPC_ReceivePacket.yaw-xMotorParam[YAW].MeasureImu->angle/(180/PI));

                    yaw_output = Automatic_aiming_pos.pos_out +
                                 GIMBAL_YAW_J * MiniPC_ReceivePacket.yaw_acc +
                                 GIMBAL_YAW_B * MiniPC_ReceivePacket.yaw_vel +
                                 GIMBAL_YAW_FC * tanhf(MiniPC_ReceivePacket.yaw_vel / GIMBAL_YAW_FC_SMOOTH) +
                                 GIMBAL_YAW_TAU0 +
                                 Automatic_aiming_spd.pos_out;
#if GIMBAL_YAW_ESO_ENABLE
                    yaw_output += yaw_eso_compensation;
#endif
                    abs_limit(&yaw_output, 10); // 限幅（现在是冗余设计）

                    /*记录角度标志位*/
                    AutoSaveAngle = 1;
                }
                else if (AutoSaveAngle == 1)
                {
                    // 自瞄目标丢失后，锁定当前角度，避免回弹
                    AUTO_Shoot_flag = 0;
                    xMotorParam[PITCH].ImuTar = xMotorParam[PITCH].MeasureImu->angle;
                    xMotorParam[YAW].ImuTar = xMotorParam[YAW].MeasureImu->angle;
                    // pid_calc(&Automatic_aiming_pos,xMotorParam[YAW].MeasureImu->angle,vision_yaw/PI*180);
                    // xMotorParam[YAW].CurrentOutput=0;
                    AutoSaveAngle = 0;
                }
            }
            else
            {
                AUTO_Shoot_flag = 0;
                // AutoSaveAngle=0;
                // if (AutoSaveAngle==1) {
                //
                //     // xMotorParam[YAW].CurrentOutput=0;
                //     AutoSaveAngle=0;
                // }
                if (xMotorParam[PITCH].FeedbackMode == ImuFeedback)
                {
                    xMotorParam[PITCH].EcoTar = xMotorParam[PITCH].MeasureDM->position;
                }
                xMotorParam[PITCH].FeedbackMode = EcoFeedback;
                // kalman_mouse_y.out 	 =	KalmanFilter(&kalman_mouse_y,remote_control.mouse.y,0.01f,0.001f);
                if (xMotorParam[YAW].TotalAngleState == 0)
                {
                    xMotorParam[YAW].ImuTar = xMotorParam[YAW].MeasureImu->total_angle;
                    memset(xMotorParam[YAW].PidParamPosImu->err, 0, 3);
                }
                xMotorParam[YAW].TotalAngleState = 1;
                // kalman_mouse_x.out   =  KalmanFilter(&kalman_mouse_x,remote_control.mouse.x,0.01f,0.001f);

                // 仅在底盘跟随模式下允许升降动作
                if (xRobotMode.ChassisMode == CHASSIS_FOLLOW)
                {
                    // SHIFT+X 触发上升，触发后保持输出直到检测到 High 到位
                    static uint16_t highlow_Init_cnt = 0;
                    if (ABS(xMotorParam[Height].MeasureEco->real_current) > 8000)
                    {
                        Stuck_Hight_cnt++;
                        if (Stuck_Hight_cnt > 50)
                        {
                            Stuck_Hight_flag = 1;
                            xMotorParam[Height].EcoTar = 0.0f;
                            Stuck_Hight_cnt = 0;
                            highlow_flag = 0;
                        }
                    }
                    else
                    {
                        Stuck_Hight_cnt = 0; // 堵转检测，当前电流超过8000mA则认为可能堵转，持续50次（约500ms）则触发堵转保护，停止升降并清除动作标志；如果电流恢复正常则重置计数
                    }
                    // 上升
                    if (KEY_SHIFT && KEY_X && Gimbal_Height_flag != High)
                    {
                        if (Stuck_Hight_flag == 0)
                        {
                            High_Low_Start_angle = xMotorParam[Height].MeasureEco->total_angle;
                        }
                        highlow_flag = 1;
                    }
                    else if (highlow_flag == 1)
                    {

                        if (ABS(High_Low_Start_angle - xMotorParam[Height].MeasureEco->total_angle) > 2200000)
                        {
                            // 如果长时间未检测到高位到达，强制停止升降，避免过流损伤
                            xMotorParam[Height].EcoTar = -5000.0f;
                        }
                        else
                        {
                            xMotorParam[Height].EcoTar = -20000.0f;
                        }
                        if (Gimbal_Height_flag == High)
                        {
                            Stuck_Hight_flag = 0;
                            xMotorParam[Height].EcoTar = 0.0f;
                            highlow_flag = 0;
                        }
                    }
                    // 下降
                    if (KEY_SHIFT && KEY_C && Gimbal_Height_flag != Low)
                    {
                        // xMotorParam[PITCH].EcoTar=0.0f;
                        if (Stuck_Hight_flag == 0)
                        {
                            High_Low_Start_angle = xMotorParam[Height].MeasureEco->total_angle;
                        }
                        xMotorParam[PITCH].EcoTar = -0.23f;
                        Gimbal_FOLLOWANGLE = Gimbal_FrontAngle;
                        // if (xMotorParam[YAW].FeedbackMode == ImuFeedback) {
                        //     xMotorParam[YAW].EcoTar    =Gimbal_FrontAngle;
                        //     xMotorParam[YAW].FeedbackMode = EcoFeedback;
                        //
                        // }
                        highlow_flag = 2;
                    }
                    else if (highlow_flag == 2)
                    {
                        if (ABS(xMotorParam[YAW].MeasureDM->position - Gimbal_FrontAngle) <= 1.0f)
                        {
                            highlow_Init_cnt++;
                            if (highlow_Init_cnt >= 50)
                            {
                                highlow_Init_cnt = 0;
                                if (ABS(High_Low_Start_angle - xMotorParam[Height].MeasureEco->total_angle) > 2100000)
                                {
                                    // 给一个较大的目标值，确保能快速下降到位，如果检测到已经接近低位了，则降低目标值，避免过冲
                                    xMotorParam[Height].EcoTar = 5000.0f;
                                }
                                else
                                {
                                    xMotorParam[Height].EcoTar = 20000.0f;
                                }
                            }
                        }
                        else
                        {
                            xMotorParam[Height].EcoTar = 0.0f;
                        }

                        if (Gimbal_Height_flag == Low)
                        {
                            // 到位后立即停止，清空动作标志
                            xMotorParam[PITCH].EcoTar = 0.05f;
                            xMotorParam[Height].EcoTar = 0.0f;
                            highlow_flag = 0;
                        }
                    }
                }

                // 仅在高位时开放正常云台角度控制；非高位时收回到安全状态
                // if (highlow_flag==0) {
                if (Gimbal_Height_flag == High)
                {
                    if (xMotorParam[YAW].FeedbackMode == EcoFeedback)
                    {
                        // 进入高位时切到IMU反馈，避免模式切换造成突变
                        xMotorParam[YAW].ImuTar = xMotorParam[YAW].MeasureImu->total_angle;
                        xMotorParam[YAW].FeedbackMode = ImuFeedback;
                    }
                    xMotorParam[YAW].ImuTar -= MOUSE_X * 50 * 0.00008f;
                    xMotorParam[PITCH].EcoTar -= MOUSE_Y * 50 * 0.0000008f;
                    // xMotorParam[PITCH].EcoTar -= RcDeadlineLimit(CH4,25)*0.000008;
                    // xMotorParam[YAW].ImuTar    -= RcDeadlineLimit(CH3,25)*0.0008;
                }
                else
                {
                    // 非高位时限制俯仰并切回Eco反馈
                    // xMotorParam[PITCH].EcoTar=0.0f;
                    if (xMotorParam[YAW].FeedbackMode == ImuFeedback)
                    {
                        xMotorParam[YAW].EcoTar = Gimbal_FrontAngle;
                        xMotorParam[YAW].FeedbackMode = EcoFeedback;
                    }

                    xMotorParam[YAW].ImuTar -= MOUSE_X * 50 * 0.00004f;
                }
                // }

                // if (KEY_A) {
                //     xMotorParam[YAW].ImuTar+=0.0005f;
                // }else if (KEY_D) {
                //     xMotorParam[YAW].ImuTar-=0.0005f;
                // }
            }
        }
        if (RC_UPPER_RIGHT_SW_DOWN)//test:增加了手动发弹测试的pitch和yaw运动
        {
            if (Gimbal_Height_flag == High)
            {
                xMotorParam[PITCH].EcoTar -= RcDeadlineLimit(CH2, 25) * 0.000008;
                xMotorParam[YAW].ImuTar -= RcDeadlineLimit(CH1, 25) * 0.0008;
            }
        }
    }
#if GIMBAL_YAW_ESO_ENABLE
    if (!yaw_eso_active)
    {
        YawEsoReset();
    }
#endif
    xMotorParam[PITCH].EcoTar = AngleLimit(xMotorParam[PITCH].EcoTar, 0.34, -0.65);
    xMotorParam[PITCH].ImuTar = AngleLimit(xMotorParam[PITCH].ImuTar, 18, -37);
}
