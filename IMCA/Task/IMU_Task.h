//
// Created by zn1135 on 25-9-28.
//

#ifndef BIG_WHEELLEG_IMU_TASK_H
#define BIG_WHEELLEG_IMU_TASK_H

#include "main.h"
#include "Motor.h"
#include "imu_data_decode.h"


typedef struct{
    float angle; //陀螺仪yaw轴的实时角度
    float speed_gyro; //陀螺仪反馈的速度
    float last_speed_gyro; //上次陀螺仪反馈的速度
    float speed_gyro_diff; //相邻两次的速度变化量
    float last_angle; //上一次角度
    float angle_diff; //前后两次直接作差得到的差值
    float angle_diff_process; //经过处理后的角度差值
    float total_angle; //转过的总角度
    float offset_relative_angle; //上电陀螺仪相对于底盘前面正中心的相对角度
    int32_t round_cnt; //圈数
    float circle_angle; //将[+180-180]转换为[0-360]
    float radian_angle;//弧度制
    float acc;
    float speed_acc;
}IMU_t;
extern IMU_t IMU_Yaw;//Y
extern IMU_t IMU_Pitch;//x
extern IMU_t IMU_Roll;//z

void Imu_param_update(receive_imusol_packet_t *Super_imu, IMU_t *IMU_Y, IMU_t *IMU_P, IMU_t *IMU_R);
void MotorImuUpdata(motor_param_t * ptr, IMU_t * pxIMU);
#endif //BIG_WHEELLEG_IMU_TASK_H
