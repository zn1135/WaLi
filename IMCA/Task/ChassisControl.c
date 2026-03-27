#include "ChassisControl.h"

#include "cmsis_os.h"
#include "ControlTask.h"
#include "RcTask.h"
#include "user_lib.h"
#include "Ramp.h"
float chassis_vx_channel = 0, chassis_vy_channel = 0, chassis_vw_channel = 0;
chassis_t Chassis;

float ChassisRotationPID(float measure, float target) {
    pid_calc(&Chassis_Rotate, measure, target);
    return Chassis_Rotate.pos_out;
}



void ChassisControlSetValue(void) {
    // relative_angle_processing(&xMotorParam[YAW]);
    if (xRobotMode.CtrlMode == LOSE) {
        chassis_vy_channel=0;
        chassis_vx_channel=0;
        chassis_vw_channel=0;
    } else if (xRobotMode.CtrlMode == RC) {
        if (xRobotMode.ChassisMode == CHASSIS_SEPARA) {
            chassis_vy_channel = -(float) RcDeadlineLimit(remote_control.ch2, 10) / 660*CHASSIS_MAXSPEED;
            chassis_vx_channel = (float) RcDeadlineLimit(remote_control.ch1, 10) / 660*CHASSIS_MAXSPEED;
            chassis_vw_channel=0;
        } else if (xRobotMode.ChassisMode == CHASSIS_FOLLOW) {
            if (Gimbal_Height_flag==High) {
                chassis_vy_channel = -(float) RcDeadlineLimit(remote_control.ch2, 10) / 660*CHASSIS_MAXSPEED;
                chassis_vx_channel = (float) RcDeadlineLimit(remote_control.ch1, 10) / 660*CHASSIS_MAXSPEED;
                chassis_vw_channel = -ChassisRotationPID(xMotorParam[YAW].MeasureDM->position, 0.01069f);
            }else {
                chassis_vy_channel = -(float) RcDeadlineLimit(remote_control.ch2, 10) / 660*CHASSIS_MAXSPEED;
                chassis_vx_channel = (float) RcDeadlineLimit(remote_control.ch1, 10) / 660*CHASSIS_MAXSPEED;
                chassis_vw_channel  = -(float)RcDeadlineLimit(remote_control.ch3,10)/660*CHASSIS_MAXSPEED/2;

            }

        } else if (xRobotMode.ChassisMode == CHASSIS_GYROSC) {
            chassis_vy_channel = -(float) RcDeadlineLimit(remote_control.ch2, 10) / 660*CHASSIS_MAXSPEED;
            chassis_vx_channel = (float) RcDeadlineLimit(remote_control.ch1, 10) / 660*CHASSIS_MAXSPEED;
            chassis_vw_channel = 7000.0f;
        }
    } else if (xRobotMode.CtrlMode == PC) {

        // 根据PC控制模式下的按键输入设置底盘运动模式和
        if (Gimbal_Height_flag==High) {
            if (KEY_F)
                xRobotMode.ChassisMode = CHASSIS_FOLLOW;
            else if (KEY_G)
                xRobotMode.ChassisMode = CHASSIS_GYROSC;
            else if (KEY_B) {
                xRobotMode.ChassisMode = CHASSIS_SEPARA;
            }
        }else {
            xRobotMode.ChassisMode = CHASSIS_FOLLOW;
        }


        if (xRobotMode.ChassisMode == CHASSIS_SEPARA) {


        } else if (xRobotMode.ChassisMode == CHASSIS_FOLLOW) {
            if (Gimbal_Height_flag==High) {
                chassis_vw_channel = -ChassisRotationPID(xMotorParam[YAW].MeasureDM->position, -0.017f);
            }else {
                chassis_vw_channel = -RcDeadlineLimit(remote_control.mouse.x,10)*25.0f;//todo  10

            }

            // chassis_vw_channel = -ChassisRotationPID(xMotorParam[YAW].MeasureDM->position, -0.017f);
        } else if (xRobotMode.ChassisMode == CHASSIS_GYROSC) {

            chassis_vw_channel = 7000.0f;
        }

        //速度通道值
        if (KEY_W) {
            chassis_vy_channel = -CHASSIS_MAXSPEED;
        }
        else if (KEY_S) {
            chassis_vy_channel = CHASSIS_MAXSPEED;
        }
        else
            chassis_vy_channel = 0;
        if (KEY_A) {
            chassis_vx_channel=-CHASSIS_MAXSPEED;
        }else if (KEY_D) {
            chassis_vx_channel=CHASSIS_MAXSPEED;
        }else
            chassis_vx_channel=0;

        if (KEY_CTRL) {
            chassis_vy_channel*=1.2f;//2.0f
            chassis_vx_channel*=1.2f;//2.0
        }

    }
    FirstOrderFilterCali(&xChassisCmdSlowSetVx, chassis_vx_channel);//一阶低通滤波
    FirstOrderFilterCali(&xChassisCmdSlowSetVy, chassis_vy_channel);//一阶低通滤波
}


void ChassisControl_Calculating(void) {
    float sin_rotate, cos_rotate;
    switch (xRobotMode.ChassisMode) {
        case CHASSIS_SEPARA: //分离模式   fabs绝对值
        {
            // sin_rotate = arm_sin_f32(xMotorParam[YAW].MeasureDM->position);
            // cos_rotate = arm_cos_f32(xMotorParam[YAW].MeasureDM->position);
            Chassis.car_vx=xChassisCmdSlowSetVx.out;
            Chassis.car_vy=xChassisCmdSlowSetVy.out;
            Chassis.car_vw=0;
            xMotorParam[WHEEL_FR].EcoTar = -(Chassis.car_vx + Chassis.car_vy + Chassis.car_vw);
            xMotorParam[WHEEL_FL].EcoTar = -Chassis.car_vx + Chassis.car_vy - Chassis.car_vw;
            xMotorParam[WHEEL_BR].EcoTar = -(-Chassis.car_vx + Chassis.car_vy+Chassis.car_vw);
            xMotorParam[WHEEL_BL].EcoTar = Chassis.car_vx + Chassis.car_vy - Chassis.car_vw;
        }
        break;
        case CHASSIS_FOLLOW: //跟随模式
        {
            sin_rotate = arm_sin_f32(xMotorParam[YAW].MeasureDM->position);
            cos_rotate = arm_cos_f32(xMotorParam[YAW].MeasureDM->position);
            Chassis.car_vx=(sin_rotate*xChassisCmdSlowSetVy.out+cos_rotate*xChassisCmdSlowSetVx.out)*cos_rotate;
            Chassis.car_vy=(cos_rotate*xChassisCmdSlowSetVy.out-sin_rotate*xChassisCmdSlowSetVx.out)*cos_rotate;
            Chassis.car_vw=chassis_vw_channel;
            xMotorParam[WHEEL_FR].EcoTar = -(Chassis.car_vx + Chassis.car_vy + Chassis.car_vw);
            xMotorParam[WHEEL_FL].EcoTar = -Chassis.car_vx + Chassis.car_vy - Chassis.car_vw;
            xMotorParam[WHEEL_BR].EcoTar = -(-Chassis.car_vx + Chassis.car_vy+Chassis.car_vw);
            xMotorParam[WHEEL_BL].EcoTar = Chassis.car_vx + Chassis.car_vy - Chassis.car_vw;
        }
        break;
        case CHASSIS_GYROSC: //小陀螺模式
        {
            sin_rotate = arm_sin_f32(xMotorParam[YAW].MeasureDM->position);
            cos_rotate = arm_cos_f32(xMotorParam[YAW].MeasureDM->position);
            Chassis.car_vx=sin_rotate*xChassisCmdSlowSetVy.out+cos_rotate*xChassisCmdSlowSetVx.out;
            Chassis.car_vy=cos_rotate*xChassisCmdSlowSetVy.out-sin_rotate*xChassisCmdSlowSetVx.out;
            Chassis.car_vw=chassis_vw_channel;
            xMotorParam[WHEEL_FR].EcoTar = -(Chassis.car_vx + Chassis.car_vy + Chassis.car_vw);
            xMotorParam[WHEEL_FL].EcoTar = -Chassis.car_vx + Chassis.car_vy - Chassis.car_vw;
            xMotorParam[WHEEL_BR].EcoTar = -(-Chassis.car_vx + Chassis.car_vy + Chassis.car_vw);
            xMotorParam[WHEEL_BL].EcoTar = Chassis.car_vx + Chassis.car_vy - Chassis.car_vw;
            // OR_Chassis.vw = xChassisCmdSlowSetVw.out;
        }
        break;
    }
}
