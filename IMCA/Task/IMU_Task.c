//
// Created by zn1135 on 25-9-28.
//

#include "IMU_Task.h"
#include "imu_data_decode.h"


IMU_t IMU_Yaw;
IMU_t IMU_Pitch;
IMU_t IMU_Roll;

/**********************************************************************************************************************
  * @Func	 void Imu_param_update(receive_imusol_packet_t *Super_imu, IMU_t *IMU_Y, IMU_t *IMU_P, IMU_t *IMU_R)
  * @Brief   更新IMU三轴姿态数据
  * @Param	 receive_imusol_packet_t *Super_imu：超级电容IMU原始数据包
  *          IMU_t *IMU_Y：YAW轴数据结构体指针
  *          IMU_t *IMU_P：PITCH轴数据结构体指针
  *          IMU_t *IMU_R：ROLL轴数据结构体指针
  * @Retval	 无
  * @Note    更新欧拉角、角速度、线加速度，含过180度跨圈处理
*********************************************************************************************************************/
void Imu_param_update(receive_imusol_packet_t *Super_imu, IMU_t *IMU_Y, IMU_t *IMU_P, IMU_t *IMU_R)
{
    /*更新PITCH轴数据*/
    IMU_P->last_angle = IMU_P->angle;
    IMU_P->speed_gyro = Super_imu->gyr[0]; //角速度
    IMU_P->angle = Super_imu->eul[1]; /*获得IMU的x轴角度值，对应云台pitch轴*/
    IMU_P->angle_diff = IMU_P->angle - IMU_P->last_angle; /*直接作差*/
    if(IMU_P->angle_diff < -240) /*顺时针，过180度处理*/
    {
        IMU_P->round_cnt++;
    }
    else if(IMU_P->angle_diff > 240)  /*逆时针，过180度处理*/
    {
        IMU_P->round_cnt--;
    }
    IMU_P->total_angle = IMU_P->angle + IMU_P->round_cnt*360.0f; /*得到总角度*/
    IMU_P->circle_angle = 360.0f + IMU_P->angle;//将负角度取正

    /*更新YAW轴数据*/
    IMU_Y->last_angle = IMU_Y->angle;
    IMU_Y->speed_gyro = -Super_imu->gyr[2];
    IMU_Y->angle =Super_imu->eul[2];
    IMU_Y->angle_diff = IMU_Y->angle - IMU_Y->last_angle; /*直接作差*/
    if(IMU_Y->angle_diff < -240) /*顺时针，过180度处理*/
    {
        IMU_Y->round_cnt++;
    }
    else if(IMU_Y->angle_diff > 240)  /*逆时针，过180度处理*/
    {
        IMU_Y->round_cnt--;
    }

    IMU_Y->total_angle = IMU_Y->angle + IMU_Y->round_cnt*360.0f; /*得到总角度*/
    if(IMU_Y->angle <= 0)
    {
        IMU_Y->circle_angle = 360.0f + IMU_Y->angle;//将负角度取正
    }
    else
    {
        IMU_Y->circle_angle = IMU_Y->angle;
    }

    /*更新ROLL轴数据*/
    IMU_R->last_angle = IMU_R->angle;
    IMU_R->speed_gyro = Super_imu->gyr[1]; //角速度
    IMU_R->angle = Super_imu->eul[0];
    IMU_R->angle_diff = IMU_R->angle - IMU_R->last_angle; /*直接作差*/
    if(IMU_R->angle_diff < -240) /*顺时针，过180度处理*/
    {
        IMU_R->round_cnt++;
    }
    else if(IMU_R->angle_diff > 240)  /*逆时针，过180度处理*/
    {
        IMU_R->round_cnt--;
    }

    IMU_R->total_angle = IMU_R->angle + IMU_R->round_cnt*360.0f; /*得到总角度*/
    if(IMU_R->angle <= 0)
    {
        IMU_R->circle_angle = 360.0f + IMU_R->angle;//将负角度取正
    }
    else
    {
        IMU_R->circle_angle = IMU_R->angle;
    }
//    MotorImuUpdata(&Motor_Pitch, IMU_P, 1)
    IMU_Pitch.speed_acc =(Super_imu->acc[0] * 10.0f);           // 加速度,以g为单位


    IMU_Roll.speed_acc = (Super_imu->acc[1]* 10.0f);		// 转向加速度，同上

    IMU_Yaw.speed_acc = (Super_imu->acc[2]* 10.0f);

}


/**********************************************************************************************************************
  * @Func	 void MotorImuUpdata(motor_param_t * ptr, IMU_t * pxIMU)
  * @Brief   将IMU数据更新到电机参数中
  * @Param	 motor_param_t *ptr：电机参数结构体指针
  *          IMU_t *pxIMU：IMU数据结构体指针
  * @Retval	 无
  * @Note    更新电机的IMU角度、总角度、角速度和车载角度(减去偏移)
*********************************************************************************************************************/
void MotorImuUpdata(motor_param_t * ptr, IMU_t * pxIMU)
{
    /*是否使用eco补偿imu*/

    ptr->MeasureImu->angle = pxIMU->angle;
    ptr->MeasureImu->total_angle = pxIMU->total_angle;
    ptr->MeasureImu->palstance = pxIMU->speed_gyro;

    ptr->MeasureImu->car_angle = ptr->MeasureImu->angle - ptr->MeasureImu->offset_angle;
    ptr->MeasureImu->car_total_angle = ptr->MeasureImu->total_angle - ptr->MeasureImu->offset_angle;



}
