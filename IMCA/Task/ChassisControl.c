#include "ChassisControl.h"

#include "ControlTask.h"
#include "GimbalControl.h"
#include "Ramp.h"
#include "RcTask.h"
#include "Vision_interact.h"
#include "cmsis_os.h"
#include "judge_interact.h"
#include "user_lib.h"
float chassis_vx_channel = 0, chassis_vy_channel = 0, chassis_vw_channel = 0;
chassis_t Chassis;

/**********************************************************************************************************************
 * @Func	 float ChassisRotationPID(float measure, float target)
 * @Brief   底盘旋转PID计算
 * @Param	 float measure：当前测量值
 *          float target：目标值
 * @Retval	 float：PID输出值
 * @Note    使用Chassis_Rotate PID结构体进行角度控制
 *********************************************************************************************************************/
float ChassisRotationPID(float measure, float target)
{
    pid_calc(&Chassis_Rotate, measure, target);
    return Chassis_Rotate.pos_out;
}

uint16_t Stuck_YAW_cnt = 0;
int GYROSC_direction = -1;
/**********************************************************************************************************************
  * @Func	 void ChassisControlSetValue(void)
  * @Brief   底盘控制量给定
  * @Param	 无
  * @Retval	 无
  * @Note    根据控制模式(RC/PC/LOSE)和底盘模式(跟随/分离/小陀螺)设置vx/vy/vw通道值，
            并进行一阶低通滤波和反向系数处理
*********************************************************************************************************************/
void ChassisControlSetValue(void)
{

    GetGimbalFrontBackOrientation(&xMotorParam[YAW]);

    Getreceive_yaw_relative_angle(&xMotorParam[YAW]);
    if (xRobotMode.CtrlMode == LOSE)
    {
        chassis_vy_channel = 0;
        chassis_vx_channel = 0;
        chassis_vw_channel = 0;
    }
    else if (xRobotMode.CtrlMode == RC)
    {
        if (xRobotMode.ChassisMode == CHASSIS_SEPARA)
        {
            chassis_vy_channel = (float)RcDeadlineLimit(CH2, 10) / 660 * CHASSIS_MAXSPEED;
            chassis_vx_channel = -(float)RcDeadlineLimit(CH1, 10) / 660 * CHASSIS_MAXSPEED;
            chassis_vw_channel = 0;
        }
        else if (xRobotMode.ChassisMode == CHASSIS_FOLLOW)
        {
            if (Gimbal_Height_flag == High)
            {
                chassis_vy_channel = (float)RcDeadlineLimit(CH2, 10) / 660 * CHASSIS_MAXSPEED;
                chassis_vx_channel = -(float)RcDeadlineLimit(CH1, 10) / 660 * CHASSIS_MAXSPEED;

                chassis_vw_channel = -ChassisRotationPID(xMotorParam[YAW].MeasureDM->position, Gimbal_FOLLOWANGLE);
            }
            else
            {
                chassis_vy_channel = (float)RcDeadlineLimit(CH2, 10) / 660 * CHASSIS_MAXSPEED;
                chassis_vx_channel = -(float)RcDeadlineLimit(CH1, 10) / 660 * CHASSIS_MAXSPEED;
                chassis_vw_channel = -(float)RcDeadlineLimit(CH3, 10) / 660 * CHASSIS_MAXSPEED / 2;
            }
        }
        else if (xRobotMode.ChassisMode == CHASSIS_GYROSC)
        {
            chassis_vy_channel = (float)RcDeadlineLimit(CH2, 10) / 660 * CHASSIS_MAXSPEED;
            chassis_vx_channel = -(float)RcDeadlineLimit(CH1, 10) / 660 * CHASSIS_MAXSPEED;

            if (ABS(xMotorParam[YAW].MeasureDM->torque) > 9 && !mouse_right.deshake_key)
            {
                Stuck_YAW_cnt++;
                if (Stuck_YAW_cnt > 400)
                {
                    Stuck_YAW_cnt = 0;
                    xMotorParam[YAW].ImuTar = xMotorParam[YAW].MeasureImu->total_angle;
                    GYROSC_direction *= -1;
                }
            }
            else
            {
                Stuck_YAW_cnt = 0;
            }

            chassis_vw_channel = CHASSIS_MAXSPEED * GYROSC_direction;
        }
    }
    else if (xRobotMode.CtrlMode == PC)
    {

        // 根据PC控制模式下的按键输入设置底盘运动模式和
        if (Gimbal_Height_flag == High)
        {
            if (KEY_F)
                xRobotMode.ChassisMode = CHASSIS_FOLLOW;
            else if (KEY_G)
                xRobotMode.ChassisMode = CHASSIS_GYROSC;
            else if (KEY_B)
            {
                xRobotMode.ChassisMode = CHASSIS_SEPARA;
            }
        }
        else
        {
            xRobotMode.ChassisMode = CHASSIS_FOLLOW;
        }

        if (xRobotMode.ChassisMode == CHASSIS_FOLLOW || xRobotMode.ChassisMode == CHASSIS_SEPARA)
        {
            if (KEY_W)
            {
                chassis_vy_channel = -CHASSIS_MAXSPEED;
            }
            else if (KEY_S)
            {
                chassis_vy_channel = CHASSIS_MAXSPEED;
            }
            else
            {
                chassis_vy_channel = 0;
            }

            if (KEY_A)
            {
                chassis_vx_channel = -CHASSIS_MAXSPEED;
            }
            else if (KEY_D)
            {
                chassis_vx_channel = CHASSIS_MAXSPEED;
            }
            else
            {
                chassis_vx_channel = 0;
            }
        }
        else
        {
            if (KEY_W)
            {
                chassis_vy_channel = -CHASSIS_MAXSPEED / 2; // 不能直接全速前进，否则小陀螺移动就会出现只能移动，不能旋转的情况
            }
            else if (KEY_S)
            {
                chassis_vy_channel = CHASSIS_MAXSPEED / 2;
            }
            else
            {
                chassis_vy_channel = 0;
            }

            if (KEY_A)
            {
                chassis_vx_channel = -CHASSIS_MAXSPEED / 2;
            }
            else if (KEY_D)
            {
                chassis_vx_channel = CHASSIS_MAXSPEED / 2;
            }
            else
            {
                chassis_vx_channel = 0;
            }
        }
        if (xRobotMode.ChassisMode == CHASSIS_SEPARA)
        {
        }
        else if (xRobotMode.ChassisMode == CHASSIS_FOLLOW)
        {
            if (Gimbal_Height_flag == High)
            {
                chassis_vw_channel = -ChassisRotationPID(xMotorParam[YAW].MeasureDM->position, Gimbal_FOLLOWANGLE);
            }
            else
            {
                chassis_vw_channel = ChassisRotationPID(xMotorParam[YAW].MeasureImu->total_angle, xMotorParam[YAW].ImuTar);
            }

            // chassis_vw_channel = -ChassisRotationPID(xMotorParam[YAW].MeasureDM->position, -0.017f);
        }
        else if (xRobotMode.ChassisMode == CHASSIS_GYROSC)
        {

            if (ABS(xMotorParam[YAW].MeasureDM->torque) > 9 && !mouse_right.deshake_key)
            {
                Stuck_YAW_cnt++;
                if (Stuck_YAW_cnt > 400)
                {
                    Stuck_YAW_cnt = 0;
                    xMotorParam[YAW].ImuTar = xMotorParam[YAW].MeasureImu->total_angle;
                    GYROSC_direction *= -1;
                }
            }
            else
            {
                Stuck_YAW_cnt = 0; // 当检测到yaw电机扭矩持续过大时，认为电机可能被卡住了，切换小陀螺旋转方向，
            }

            chassis_vw_channel = (CHASSIS_MAXSPEED - sqrt((chassis_vx_channel * chassis_vx_channel + chassis_vy_channel * chassis_vy_channel))) * GYROSC_direction; // 小陀螺模式下，底盘旋转速度与底盘线速度成反比，底盘线速度越大，旋转速度越慢，解决了小陀螺模式下，底盘走不直问题，但还是有点漂移，问题是因为功率限制的
        }
    }
    if (xRobotMode.ChassisMode == CHASSIS_FOLLOW)
    {
        chassis_vx_channel = chassis_vx_channel * xRelativeAngle.gimbal_direction;
        chassis_vy_channel = chassis_vy_channel * xRelativeAngle.gimbal_direction;
    }
    FirstOrderFilterCali(&xChassisCmdSlowSetVx, chassis_vx_channel); // 一阶低通滤波
    FirstOrderFilterCali(&xChassisCmdSlowSetVy, chassis_vy_channel); // 一阶低通滤波
    // FirstOrderFilterCali(&xChassisCmdSlowSetVw, chassis_vw_channel);//一阶低通滤波
    // xChassisCmdSlowSetVx.out=chassis_vx_channel;
    // xChassisCmdSlowSetVy.out=chassis_vy_channel;
    xChassisCmdSlowSetVw.out = chassis_vw_channel;
    if (KEY_CTRL)
    {
        chassis_vx_channel *= 1.2;
        chassis_vy_channel *= 1.2;
    }
}

/**********************************************************************************************************************
 * @Func	 void ChassisControl_Calculating(void)
 * @Brief   底盘运动学解算
 * @Param	 无
 * @Retval	 无
 * @Note    根据底盘模式(分离/跟随/小陀螺)将vx/vy/vw解算为四个麦轮的目标速度EcoTar
 *********************************************************************************************************************/
void ChassisControl_Calculating(void)
{
    float sin_rotate, cos_rotate;
    switch (xRobotMode.ChassisMode)
    {
    case CHASSIS_SEPARA: // 分离模式   fabs绝对值
    {
        // sin_rotate = arm_sin_f32(xMotorParam[YAW].MeasureDM->position);
        // cos_rotate = arm_cos_f32(xMotorParam[YAW].MeasureDM->position);
        Chassis.car_vx = xChassisCmdSlowSetVx.out;
        Chassis.car_vy = xChassisCmdSlowSetVy.out;
        Chassis.car_vw = 0;
        xMotorParam[WHEEL_FR].EcoTar = -(Chassis.car_vx + Chassis.car_vy + Chassis.car_vw);
        xMotorParam[WHEEL_FL].EcoTar = -Chassis.car_vx + Chassis.car_vy - Chassis.car_vw;
        xMotorParam[WHEEL_BR].EcoTar = -(-Chassis.car_vx + Chassis.car_vy + Chassis.car_vw);
        xMotorParam[WHEEL_BL].EcoTar = Chassis.car_vx + Chassis.car_vy - Chassis.car_vw;
    }
    break;
    case CHASSIS_FOLLOW: // 跟随模式
    {
        if (Gimbal_Height_flag == High)
        {
            sin_rotate = arm_sin_f32(xRelativeAngle.receive_yaw_relative_rad); // 跟随模式下，底盘运动方向与云台前后方向一致时，sin_rotate为正，底盘运动方向与云台前后方向相反时，sin_rotate为负
            cos_rotate = arm_cos_f32(xRelativeAngle.receive_yaw_relative_rad);
            Chassis.car_vx = (sin_rotate * xChassisCmdSlowSetVy.out + cos_rotate * xChassisCmdSlowSetVx.out) * cos_rotate; // 跟随模式下，底盘运动方向与云台前后方向一致时，vx和vy的分量都较大，底盘运动方向与云台前后方向垂直时，vx和vy的分量都为0
            Chassis.car_vy = (cos_rotate * xChassisCmdSlowSetVy.out - sin_rotate * xChassisCmdSlowSetVx.out) * cos_rotate;
        }
        else
        {
            Chassis.car_vx = xChassisCmdSlowSetVx.out * arm_cos_f32(xRelativeAngle.receive_yaw_relative_rad); //
            Chassis.car_vy = xChassisCmdSlowSetVy.out * arm_cos_f32(xRelativeAngle.receive_yaw_relative_rad);
        }

        Chassis.car_vw = xChassisCmdSlowSetVw.out;
        xMotorParam[WHEEL_FR].EcoTar = -(Chassis.car_vx + Chassis.car_vy + Chassis.car_vw);
        xMotorParam[WHEEL_FL].EcoTar = -Chassis.car_vx + Chassis.car_vy - Chassis.car_vw;
        xMotorParam[WHEEL_BR].EcoTar = -(-Chassis.car_vx + Chassis.car_vy + Chassis.car_vw);
        xMotorParam[WHEEL_BL].EcoTar = Chassis.car_vx + Chassis.car_vy - Chassis.car_vw;
    }
    break;
    case CHASSIS_GYROSC: // 小陀螺模式
    {
        sin_rotate = arm_sin_f32((xMotorParam[YAW].MeasureDM->position - Gimbal_FrontAngle) / 180 * PI);
        cos_rotate = arm_cos_f32((xMotorParam[YAW].MeasureDM->position - Gimbal_FrontAngle) / 180 * PI);
        Chassis.car_vx = (sin_rotate * xChassisCmdSlowSetVy.out + cos_rotate * xChassisCmdSlowSetVx.out);
        Chassis.car_vy = (cos_rotate * xChassisCmdSlowSetVy.out - sin_rotate * xChassisCmdSlowSetVx.out);
        Chassis.car_vw = xChassisCmdSlowSetVw.out;
        xMotorParam[WHEEL_FR].EcoTar = -(Chassis.car_vx + Chassis.car_vy + Chassis.car_vw);
        xMotorParam[WHEEL_FL].EcoTar = -Chassis.car_vx + Chassis.car_vy - Chassis.car_vw;
        xMotorParam[WHEEL_BR].EcoTar = -(-Chassis.car_vx + Chassis.car_vy + Chassis.car_vw);
        xMotorParam[WHEEL_BL].EcoTar = Chassis.car_vx + Chassis.car_vy - Chassis.car_vw;
        // OR_Chassis.vw = xChassisCmdSlowSetVw.out;
    }
    break;
    }
}
