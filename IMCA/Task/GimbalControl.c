//
// Created by zn1135 on 25-11-29.
//

#include "GimbalControl.h"

#include <stdlib.h>
#include <string.h>

#include "arm_math.h"
#include "ControlTask.h"
#include "IMU_Task.h"
#include "main.h"
#include "user_lib.h"
#include "Motor.h"
#include "Ramp.h"
#include "RcTask.h"
#include "ShootControl.h"
#include "Vision_interact.h"

float yaw_pos,yaw_acc,yaw_vel,yaw_output;
float pitch_pos,pitch_acc,pitch_vel,pitch_output;
float D_speed_yaw,gyrosc_speed;
float D_speed_pitch;
float vision_pitch = 0;
float vision_yaw = 0;

// 高低位动作标志：0-空闲，1-执行到High，2-执行到Low
uint8_t highlow_flag = 0;

void GimbalInit(void)
{
    static uint16_t Gimbal_Init_count = 0;
    // 上电先将Pitch/Yaw拉到安全初始目标位，等待电机稳定
    xMotorParam[PITCH].EcoTar = 0;
    MotorCalc(&xMotorParam[PITCH]);
    xMotorParam[YAW].EcoTar = Gimbal_FOLLOWANGLE;
    MotorCalc(&xMotorParam[YAW]);

    if(abs(xMotorParam[PITCH].MeasureDM->position) <= 0.05 && xMotorParam[PITCH].MeasureDM->DM_J4310_InitFlag
        &&abs(xMotorParam[YAW].MeasureDM->position-xMotorParam[YAW].EcoTar) <= 0.05 && xMotorParam[YAW].MeasureDM->DM_J4310_InitFlag) {
        Gimbal_Init_count ++;
        if(Gimbal_Init_count >= 200)
        {
            Gimbal_Init_Flag = 1;
            Gimbal_Init_count = 0;
            // 初始化完成后切到IMU闭环，并重置角度参考
            xMotorParam[YAW].FeedbackMode = ImuFeedback;
            xMotorParam[YAW].MeasureImu ->offset_angle = xMotorParam[YAW].MeasureImu ->angle ;
            xMotorParam[PITCH].MeasureImu ->offset_angle = xMotorParam[PITCH].MeasureImu ->angle ;
            xMotorParam[YAW].TotalAngleState = 1;
            xMotorParam[PITCH].TotalAngleState = 0;
            xMotorParam[YAW].MeasureImu ->reverse_derection = 0;
            xMotorParam[PITCH].MeasureImu ->reverse_derection = 1;
        }
    }
}
int znlook[10];

void GimbalControlSetValue(void) {

    static int AngleSaveFlag = 1, AutoSaveAngle = 1;
    if(AngleSaveFlag)
    {
        AngleSaveFlag = 0;
        // 进入可控状态时锁定当前角度，避免模式切换时目标跳变
        xMotorParam[PITCH].EcoTar = xMotorParam[PITCH].MeasureDM->position;
        //		xMotorParam[PITCH].ImuTar = xMotorParam[PITCH].MeasureImu->angle;
        xMotorParam[YAW].ImuTar = xMotorParam[YAW].MeasureImu ->total_angle ;
    }
    if (xRobotMode.CtrlMode == LOSE)
    {
        // 遥控丢失后，下次恢复控制时重新保存目标角
        AngleSaveFlag = 1;
    } else if(xRobotMode.CtrlMode == RC)
    {
        // RC模式下：仅高位开放云台角度控制，低位回收到安全姿态
        if (Gimbal_Height_flag==High) {
            if (xMotorParam[YAW].FeedbackMode == EcoFeedback) {
                xMotorParam[YAW].ImuTar=xMotorParam[YAW].MeasureImu->total_angle;
                xMotorParam[YAW].FeedbackMode = ImuFeedback;
            }
                xMotorParam[PITCH].EcoTar -= RcDeadlineLimit(CH4,25)*0.000008;
                xMotorParam[YAW].ImuTar    -= RcDeadlineLimit(CH3,25)*0.0008;
        }else {
            xMotorParam[PITCH].EcoTar=0.0f;
            if (xMotorParam[YAW].FeedbackMode == ImuFeedback) {
                xMotorParam[YAW].EcoTar    =Gimbal_FrontAngle;
                xMotorParam[YAW].FeedbackMode = EcoFeedback;
            }

        }
        //				xMotorParam[PITCH].ImuTar += RcDeadlineLimit(CH4,25)*0.0005;

       if (CH5 < -500&&Gimbal_Height_flag!=High&&xRobotMode.ChassisMode ==CHASSIS_FOLLOW) {
           xMotorParam[Height].EcoTar=-5000.0f;
       }else if (CH5 >500&&Gimbal_Height_flag!=Low&&xRobotMode.ChassisMode ==CHASSIS_FOLLOW){
           xMotorParam[Height].EcoTar=5000.0f;
           xMotorParam[PITCH].EcoTar=-0.017f;
       }else{
           xMotorParam[Height].EcoTar=0.0f;
       }

        //		xMotorParam[YAW].EcoTar -= RcDeadlineLimit(CH3,25)*0.00002;
    }else if (xRobotMode.CtrlMode == PC) {


        // if (KEY_SHIFT&&KEY_X&&Gimbal_Height_flag!=High&&xRobotMode.ChassisMode ==CHASSIS_FOLLOW) {
        //     xMotorParam[Height].EcoTar=-15000.0f;
        // }else if(KEY_CTRL&&KEY_X&&Gimbal_Height_flag!=Low&&xRobotMode.ChassisMode ==CHASSIS_FOLLOW) {
        //     xMotorParam[Height].EcoTar=15000.0f;
        //     xRobotMode.ChassisMode =CHASSIS_FOLLOW;
        // }else {
        //     xMotorParam[Height].EcoTar=0.0f;
        // }
        if(xRobotMode.GimbalMode == NOSHOOT_MODE || xRobotMode.GimbalMode == NORMAL_MODE)
        {
            // 右键/拨杆/快捷键触发自瞄通道
            if(mouse_right.deshake_key||(CH5 >500)||KEY_V)
            {
                // AUTO_Shoot_flag = 2;
                if(xMotorParam[PITCH].FeedbackMode == EcoFeedback) {
                    // Pitch从编码器环切入IMU环时，先对齐目标角
                    xMotorParam[PITCH].ImuTar = xMotorParam[PITCH].MeasureImu->angle;
                    //					xMotorParam[PITCH].EcoTar = xMotorParam[PITCH].MeasureDM->position;
                }
                xMotorParam[PITCH].FeedbackMode = ImuFeedback;
                if (xMotorParam[YAW].TotalAngleState==1) {
                    // Yaw由总角模式切到相对角模式，先同步当前姿态
                    xMotorParam[YAW].ImuTar=xMotorParam[YAW].MeasureImu->angle;
                    xMotorParam[YAW].TotalAngleState=0;
                }
                if(MiniPC_ReceivePacket.fireflag != 0)
                {

                    AUTO_Shoot_flag = 2;
                    xMotorParam[PITCH].ImuTar =-MiniPC_ReceivePacket.pitch;
                    vision_yaw=MiniPC_ReceivePacket.yaw;

                    yaw_pos=MiniPC_ReceivePacket.yaw;
                    xMotorParam[YAW].ImuTar = MiniPC_ReceivePacket.yaw/PI*180;
                    // yaw_vel=MiniPC_ReceivePacket.yaw_vel;
                    TargetAngleProc(&xMotorParam[YAW]);

                    // 位置环+速度环+前馈组合，提高跟踪响应
                    pid_calc(&Automatic_aiming_pos,xMotorParam[YAW].MeasureImu->angle,xMotorParam[YAW].ImuTar);
                    pid_calc(&Automatic_aiming_spd,xMotorParam[YAW].MeasureImu->palstance, MiniPC_ReceivePacket.yaw_vel/PI*180);
                    //     yaw_output=Moment_of_Inertia*(MiniPC_ReceivePacket.yaw_acc)+18.0f*(MiniPC_ReceivePacket.yaw-xMotorParam[YAW].MeasureImu->angle/(180/PI));

                    yaw_output=Automatic_aiming_pos.pos_out+Moment_of_Inertia*(MiniPC_ReceivePacket.yaw_acc)+Automatic_aiming_spd.pos_out;
                    abs_limit(&yaw_output,10);

                    /*记录角度标志位*/
                    AutoSaveAngle = 1;

                }else if(AutoSaveAngle == 1)
                {
                    // 自瞄目标丢失后，锁定当前角度，避免回弹
                    AUTO_Shoot_flag = 0;
                    xMotorParam[PITCH].ImuTar= xMotorParam[PITCH].MeasureImu->angle;
                    xMotorParam[YAW].ImuTar= xMotorParam[YAW].MeasureImu->angle;
                    // pid_calc(&Automatic_aiming_pos,xMotorParam[YAW].MeasureImu->angle,vision_yaw/PI*180);
                    // xMotorParam[YAW].CurrentOutput=0;
                    AutoSaveAngle = 0;


                }
            }else
            {
                AUTO_Shoot_flag = 0;
                // AutoSaveAngle=0;
                // if (AutoSaveAngle==1) {
                //
                //     // xMotorParam[YAW].CurrentOutput=0;
                //     AutoSaveAngle=0;
                // }
                if(xMotorParam[PITCH].FeedbackMode == ImuFeedback) {
                    xMotorParam[PITCH].EcoTar = xMotorParam[PITCH].MeasureDM->position;
                }
                xMotorParam[PITCH].FeedbackMode = EcoFeedback;
                // kalman_mouse_y.out 	 =	KalmanFilter(&kalman_mouse_y,remote_control.mouse.y,0.01f,0.001f);
                if(xMotorParam[YAW].TotalAngleState == 0)
                {
                    xMotorParam[YAW].ImuTar= xMotorParam[YAW].MeasureImu->total_angle;
                    memset(xMotorParam[YAW].PidParamPosImu->err,0,3);
                }
                xMotorParam[YAW].TotalAngleState = 1;
                // kalman_mouse_x.out   =  KalmanFilter(&kalman_mouse_x,remote_control.mouse.x,0.01f,0.001f);



                // 仅在底盘跟随模式下允许升降动作
                if (xRobotMode.ChassisMode ==CHASSIS_FOLLOW) {
                    // SHIFT+X 触发上升，触发后保持输出直到检测到 High 到位
                    static uint16_t  highlow_Init_flag= 0;
                    if (KEY_SHIFT&&KEY_X&&Gimbal_Height_flag!=High) {

                        highlow_flag=1;
                    }else if (highlow_flag==1) {
                        // if (ABS(xMotorParam[YAW].MeasureDM->position-Gimbal_FrontAngle) <= 0.5f) {
                            xMotorParam[Height].EcoTar=-20000.0f;
                        // }else {
                        //     xMotorParam[Height].EcoTar=0.0f;
                        // }
                        if (Gimbal_Height_flag==High) {
                            // 到位后立即停止，清空动作标志                            xMotorParam[Height].EcoTar=0.0f;
                            xMotorParam[Height].EcoTar=0.0f;
                            highlow_flag=0;
                        }

                    }

                    if (KEY_SHIFT&&KEY_C&&Gimbal_Height_flag!=Low) {
                        xMotorParam[PITCH].EcoTar=0.0f;
                        Gimbal_FOLLOWANGLE=Gimbal_FrontAngle;
                        // if (xMotorParam[YAW].FeedbackMode == ImuFeedback) {
                        //     xMotorParam[YAW].EcoTar    =Gimbal_FrontAngle;
                        //     xMotorParam[YAW].FeedbackMode = EcoFeedback;
                        //
                        // }
                        highlow_flag=2;
                    }else if (highlow_flag==2) {
                        if (ABS(xMotorParam[YAW].MeasureDM->position-Gimbal_FrontAngle) <= 1.0f) {
                            highlow_Init_flag++;
                            if (highlow_Init_flag>=50) {
                                highlow_Init_flag=0;
                                xMotorParam[Height].EcoTar=20000.0f;
                            }
                        }else {
                            xMotorParam[Height].EcoTar=0.0f;
                        }
                        if (Gimbal_Height_flag==Low) {
                            // 到位后立即停止，清空动作标志
                            xMotorParam[Height].EcoTar=0.0f;
                            highlow_flag=0;
                        }
                    }
                }


                // 仅在高位时开放正常云台角度控制；非高位时收回到安全状态
                // if (highlow_flag==0) {
                    if (Gimbal_Height_flag==High) {
                        if (xMotorParam[YAW].FeedbackMode == EcoFeedback) {
                            // 进入高位时切到IMU反馈，避免模式切换造成突变
                            xMotorParam[YAW].ImuTar=xMotorParam[YAW].MeasureImu->total_angle;
                            xMotorParam[YAW].FeedbackMode = ImuFeedback;
                        }
                        xMotorParam[YAW].ImuTar   -= MOUSE_X *50*0.00004f;
                        xMotorParam[PITCH].EcoTar -= MOUSE_Y*50*0.0000008f;
                        // xMotorParam[PITCH].EcoTar -= RcDeadlineLimit(CH4,25)*0.000008;
                        // xMotorParam[YAW].ImuTar    -= RcDeadlineLimit(CH3,25)*0.0008;
                    }else {
                        // 非高位时限制俯仰并切回Eco反馈
                        xMotorParam[PITCH].EcoTar=0.0f;
                        if (xMotorParam[YAW].FeedbackMode == ImuFeedback) {
                            xMotorParam[YAW].EcoTar    =Gimbal_FrontAngle;
                            xMotorParam[YAW].FeedbackMode = EcoFeedback;

                        }

                        xMotorParam[YAW].ImuTar   -= MOUSE_X *50*0.00004f;
                    }
                // }

                // if (KEY_A) {
                //     xMotorParam[YAW].ImuTar+=0.0005f;
                // }else if (KEY_D) {
                //     xMotorParam[YAW].ImuTar-=0.0005f;
                // }
            }
        }
    }
    xMotorParam[PITCH].EcoTar = AngleLimit(xMotorParam[PITCH].EcoTar, 0.34, -0.65);
    xMotorParam[PITCH].ImuTar = AngleLimit(xMotorParam[PITCH].ImuTar, 18, -37);
    if(xMotorParam[PITCH].EcoTar<0.0005f && xMotorParam[PITCH].EcoTar>(-0.0005f))
    {
        // 小范围死区归零，减少电机抖动
        xMotorParam[PITCH].EcoTar = 0;
    }
}





