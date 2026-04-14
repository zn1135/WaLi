//
// Created by zn1135 on 25-9-23.
//

#include "user_lib.h"

#include "ChassisControl.h"
#include "ControlTask.h"
#include "GimbalControl.h"
#include "RcTask.h"
#include "Motor.h"
#include "Ramp.h"
#include "Vision_interact.h"

uint8_t AUTO_Shoot_flag = 0;
relative_angle_t xRelativeAngle;
uint8_t yaw_flag;
uint8_t RB_Color=0;
uint8_t RoboHP_flag;
uint8_t yaw_position[4];
float Gimbal_FOLLOWANGLE = Gimbal_FrontAngle;
uint8_t Motor_TimeOut_flag = 0;
/**
 * @brief 角度限制函数
 *
 * 将目标角度限制在指定的最小值和最大值之间
 *
 * @param tar 目标角度值
 * @param max_angle 最大允许角度值
 * @param min_angle 最小允许角度值
 * @return 限制后的角度值
 */
float AngleLimit(float tar, float max_angle, float min_angle)
{
    if(tar >= max_angle)
    {
        tar = max_angle;
    }
    else if(tar <= min_angle)
    {
        tar = min_angle;
    }
    else;
    return tar;

}



void TargetAngleProc(motor_param_t *ptr)
{
    if(ptr->FeedbackMode == EcoFeedback)
    {
        switch(ptr->MotorType)
        {
            case DJI:
            {
                if (ptr->EcoTar <= ptr->MeasureEco->angle - 4096)
                    ptr->EcoTar += 8191;
                else if (ptr->EcoTar >= ptr->MeasureEco->angle + 4096)
                    ptr->EcoTar -= 8191;
            }break;
            case DM:
            {
                if (ptr->EcoTar <= ptr->MeasureDM->position - PI)
                    ptr->EcoTar += 2*PI;
                else if (ptr->EcoTar >= ptr->MeasureDM->position + PI)
                    ptr->EcoTar -= 2*PI;
            }break;
        }

    }
    else if(ptr->FeedbackMode == ImuFeedback)
    {
        if (ptr->ImuTar < ptr->MeasureImu->angle - 180)
            ptr->ImuTar += 360;
        else if (ptr->ImuTar > ptr->MeasureImu->angle + 180)
            ptr->ImuTar -= 360;
    }
    else;
}


void FloatToUint8_t(float fNum, uint8_t *strBuf, int nLen)
{
    if (nLen < 4)
        return;
    int i = 0;
    //  unsigned char nTmp;
    char *p = (char *)&fNum;
    for (i = 0; i < 4; i++) {
        strBuf[i] = *p;
        p++;
    }
}

float GetPositionRelativeAngle_front(motor_param_t * ptr)
{
	float relative_angle_dm = 0,relative_angle_out_dm = 0;;
	int8_t direction_flag = 0;
	//    relative_angle_dm = ptr->MeasureDM->position  ;
	relative_angle_dm = ptr->MeasureDM->position  - CHASSIS_Yaw_IMU_FRONT;
	//    relative_angle_dm = ptr->MeasureDJI->angle_rad  - CHASSIS_Yaw_IMU_FRONT;

	if(CHASSIS_Yaw_IMU_FRONT < CHASSIS_Yaw_IMU_BACK)
	{
		if((CHASSIS_Yaw_IMU_FRONT < ptr->MeasureDM->position)&&(ptr->MeasureDM->position < CHASSIS_Yaw_IMU_BACK))
		{
			direction_flag = -1;
		}
		else
		{
			direction_flag = 1;

		}
	}
	else
	{
		if((CHASSIS_Yaw_IMU_FRONT < ptr->MeasureDM->position)||(ptr->MeasureDM->position < CHASSIS_Yaw_IMU_BACK))
		{
			direction_flag = -1;
		}
		else
		{
			direction_flag = 1;
		}

	}
	//		if(ptr->MeasureDM->position>1.57f&&ptr->MeasureDM->position<4.71f)
	//			direction_flag = -1;
	//		else if((ptr->MeasureDM->position<1.57f&&ptr->MeasureDM->position>0)||(ptr->MeasureDM->position>4.71f && ptr->MeasureDM->position<6.28))
	//			direction_flag = 1;
	if(relative_angle_dm < 0)
		relative_angle_dm = -relative_angle_dm;
	//
	if(relative_angle_dm > 3.141592)
		relative_angle_out_dm = 6.28318 - relative_angle_dm;
	else
		relative_angle_out_dm = relative_angle_dm;
	//    xRelativeAngle.receive_yaw_relative_rad = relative_angle_out_dm * direction_flag;
	//		return (relative_angle_out_dm * direction_flag)*8192/2/PI;
	return relative_angle_out_dm * direction_flag;
}

float GetPositionRelativeAngle_back(motor_param_t * ptr)
{
	float relative_angle_dm = 0,relative_angle_out_dm = 0;;
	int16_t relative_angle = 0, relative_angle_out = 0;
	int8_t direction_flag = 0;

	relative_angle_dm = ptr->MeasureDM->position  - CHASSIS_Yaw_IMU_BACK; /*???????*/
	//    relative_angle_dm = ptr->MeasureDM->position   ; /*???????*/

	if(CHASSIS_Yaw_IMU_FRONT < CHASSIS_Yaw_IMU_BACK)
	{
		if((CHASSIS_Yaw_IMU_FRONT < ptr->MeasureDM->position)&&(ptr->MeasureDM->position < CHASSIS_Yaw_IMU_BACK))
		{
			direction_flag = -1;
		}
		else
		{
			direction_flag = 1;

		}
	}
	else
	{
		if((CHASSIS_Yaw_IMU_FRONT < ptr->MeasureDM->position)||(ptr->MeasureDM->position <CHASSIS_Yaw_IMU_BACK))
		{
			direction_flag = -1;
		}
		else
		{
			direction_flag = 1;
		}
	}

	if(relative_angle_dm < 0)
		relative_angle_dm = -relative_angle_dm;

	if(relative_angle_dm > 3.141592)
		relative_angle_out_dm = 6.28318 - relative_angle_dm;
	else
		relative_angle_out_dm = relative_angle_dm;

	//    xRelativeAngle.receive_yaw_relative_rad = relative_angle_out_dm * direction_flag;
	//    return (relative_angle_out_dm * direction_flag)*8192/2/PI;
	return relative_angle_out_dm * direction_flag;
}
/**
	* @brief  相对角度数据处理
  * @param  none
  * @retval none
	* @Other  none
  */
void relative_angle_processing(motor_param_t * ptr)
{
	/*以这个相对角度为基础，建立四个区*/
	float basic_angle;
	basic_angle = GetPositionRelativeAngle_front(ptr);
	xRelativeAngle.receive_yaw_relative_rad =   0   		 +  GetPositionRelativeAngle_front(ptr);
	xRelativeAngle.gimbal_direction = 1;

	if(xRobotMode.ChassisMode == CHASSIS_FOLLOW)	/*只有跟随模式才会触发*/
	{
		/*判断在哪个区*/
		if(xRelativeAngle.basic_region == 0)
		{
			xRelativeAngle.receive_yaw_relative_rad =   0   +  (float)GetPositionRelativeAngle_front(ptr);
			xRelativeAngle.gimbal_direction = 1;
		}
		else if(xRelativeAngle.basic_region == 3)
		{
			xRelativeAngle.receive_yaw_relative_rad =   0   +  (float)GetPositionRelativeAngle_front(ptr);
			xRelativeAngle.gimbal_direction = 1;
		}
		else if(xRelativeAngle.basic_region == 2)
		{
			xRelativeAngle.receive_yaw_relative_rad =   0   -  (float)GetPositionRelativeAngle_back(ptr);
			xRelativeAngle.gimbal_direction = 0;
		}
		else if(xRelativeAngle.basic_region == 1)
		{
			xRelativeAngle.receive_yaw_relative_rad =   0   -  (float)GetPositionRelativeAngle_back(ptr);
			xRelativeAngle.gimbal_direction = 0;
		}
	}
	/*若是分离模式和小陀螺模式*/
	else if(xRobotMode.ChassisMode == CHASSIS_SEPARA || xRobotMode.ChassisMode == CHASSIS_GYROSC)
	{
		/*判断在哪个区*/
		if(basic_angle<=1.57&&basic_angle>=0)
		{
			xRelativeAngle.basic_region = 0;
		}
		else if(basic_angle<0&&basic_angle>=-1.57)
		{
			xRelativeAngle.basic_region = 3;
		}
		else if(basic_angle<=-1.57&&basic_angle>=-3.14)
		{
			xRelativeAngle.basic_region = 2;
		}
		else if(basic_angle<=3.14&&basic_angle>1.57)
		{
			xRelativeAngle.basic_region = 1;
		}

		if(xRobotMode.ChassisMode == CHASSIS_GYROSC)
		{
			xRelativeAngle.gimbal_direction = 0;
			xRelativeAngle.receive_yaw_relative_rad =   0   +  (float)GetPositionRelativeAngle_front(ptr);
		}
		else if(xRobotMode.ChassisMode == CHASSIS_SEPARA)
		{
			if(xRelativeAngle.gimbal_direction == 0)
			{
				xRelativeAngle.receive_yaw_relative_rad =   0   +  (float)GetPositionRelativeAngle_front(ptr);
			}
			else
			{
				xRelativeAngle.receive_yaw_relative_rad =   0   -  (float)GetPositionRelativeAngle_back(ptr);
			}
		}
	}

}

void GetGimbalFrontBackOrientation(motor_param_t * ptr) {
	if (ptr->MeasureDM->position > 0.0f && ptr->MeasureDM->position < 180.0f) {
		if (highlow_flag==0) {
			Gimbal_FOLLOWANGLE =Gimbal_FrontAngle;
		}
		xRelativeAngle.gimbal_direction=1;
	} else {
		if (highlow_flag==0) {
			Gimbal_FOLLOWANGLE = Gimbal_backAngle;
		}
		xRelativeAngle.gimbal_direction=-1;
	}
}


void MotorWatchdogMonitor() {
	if (xMotorParam[WHEEL_FR].Timeout_Flag>20&&xMotorParam[WHEEL_FL].Timeout_Flag>20&&xMotorParam[WHEEL_BR].Timeout_Flag>20&&xMotorParam[WHEEL_BL].Timeout_Flag>20) {
		Motor_TimeOut_flag=0;
	}else {
		Motor_TimeOut_flag=1;
	}
	xMotorParam[WHEEL_FR].Timeout_Flag=0;
	xMotorParam[WHEEL_FL].Timeout_Flag=0;
	xMotorParam[WHEEL_BR].Timeout_Flag=0;
	xMotorParam[WHEEL_BR].Timeout_Flag=0;

	static uint8_t cnt_=0;
	if(cnt_++ > 10) {
		if (RC_TimeOut_cnt > 20) {
			RC_TimeOut_Flag=0;
		}else {
			RC_TimeOut_Flag=1;
		}
		RC_TimeOut_cnt=0;
		cnt_=0;
	}

}




void Getreceive_yaw_relative_angle(motor_param_t * ptr) {
	if (Gimbal_Height_flag==High) {
		xRelativeAngle.receive_yaw_relative_rad =(xMotorParam[YAW].MeasureDM->position-Gimbal_FrontAngle)/180*PI;
		while (xRelativeAngle.receive_yaw_relative_rad > PI) {
			xRelativeAngle.receive_yaw_relative_rad -= (2 * PI);
		}
		while (xRelativeAngle.receive_yaw_relative_rad <= (-PI)) {
			xRelativeAngle.receive_yaw_relative_rad += (2 * PI);
		}
	}else {
		xRelativeAngle.receive_yaw_relative_rad =(xMotorParam[YAW].ImuTar-xMotorParam[YAW].MeasureImu->total_angle)/180*PI;
		while (xRelativeAngle.receive_yaw_relative_rad > PI) {
			xRelativeAngle.receive_yaw_relative_rad -= (2 * PI);
		}
		while (xRelativeAngle.receive_yaw_relative_rad <= (-PI)) {
			xRelativeAngle.receive_yaw_relative_rad += (2 * PI);
		}
	}

}