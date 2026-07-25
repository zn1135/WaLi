//
// Created by zn1135 on 25-11-29.
//

#include "Vision_interact.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "IMU_Task.h"
#include "Judge_interact.h"
#include "RcTask.h"
#include "imu_data_decode.h"
#include "math.h"
#include "usbd_cdc_if.h"
#include "user_lib.h"
#include "Odometry.h"
bool MyColor;

uint8_t MiniPC_SendBuf[MINIPC_SENDLENGTH];
uint8_t MiniPC_RecBuf[MINIPC_REVCLENGTH];

MiniPC_ReceivePacket_Typedef MiniPC_ReceivePacket = {
    .header = 0,
};

// Send head
MiniPC_SendPacket_Typedef MiniPC_SendPacket = {
    .header = 0x5A,
    .end = 0xA5,
};

/**********************************************************************************************************************
 * @Func	 void MiniPC_SendVisionFrameInfo(MiniPC_SendPacket_Typedef *SendPacket)
 * @Brief   发送视觉帧信息给MiniPC
 * @Param	 MiniPC_SendPacket_Typedef *SendPacket：发送数据包结构体指针
 * @Retval	 无
 * @Note    将IMU姿态四元数、云台YAW/PITCH角度、模式位等信息打包通过USB CDC发送
 *********************************************************************************************************************/
void MiniPC_SendVisionFrameInfo(MiniPC_SendPacket_Typedef *SendPacket)
{
    float temp;
    if (temp > PI)
        temp -= 2 * PI;
    else if (temp < -PI)
        temp += 2 * PI;
    for (int i = 0; i < 4; i++)
    {
        MiniPC_SendPacket.q[i] = receive_imusol.quat[i];
    }
    if (mouse_right.deshake_key)
    {
        MiniPC_SendPacket.mode = 1;
    }
    else if (KEY_V && KEY_CTRL)
    {
        MiniPC_SendPacket.mode = 3;
    }
    else if (KEY_V)
    {
        MiniPC_SendPacket.mode = 2;
    }

    else
        MiniPC_SendPacket.mode = 0;
    MiniPC_SendPacket.yaw = IMU_Yaw.angle * PI / 180;
    MiniPC_SendPacket.yaw_vel = IMU_Yaw.speed_gyro  /180 * PI;
    //	MiniPC_SendPacket.yaw = xMotorParam[YAW].MeasureDM->position;
    //	MiniPC_SendPacket.pitch = xMotorParam[PITCH].MeasureDM->position;
    MiniPC_SendPacket.x=WheelOdom.pos_y_nav;
    MiniPC_SendPacket.y=WheelOdom.pos_x_nav;
    MiniPC_SendPacket.z=0;
    MiniPC_SendPacket.pitch = IMU_Pitch.angle * PI / 180; // xMotorParam[PITCH].MeasureImu->angle *PI/180;//IMU_Look[1];
    MiniPC_SendPacket.my_color = !RoboColor();            // send rival's color
    memcpy(MiniPC_SendBuf, (uint8_t *)SendPacket, MINIPC_SENDLENGTH);
    CDC_Transmit_FS(MiniPC_SendBuf, MINIPC_SENDLENGTH);
}

/**********************************************************************************************************************
 * @Func	 void vision_connect(uint8_t *RecvUsartData)
 * @Brief   解析MiniPC下发的视觉连接数据
 * @Param	 uint8_t *RecvUsartData：接收到的串口数据缓冲区
 * @Retval	 无
 * @Note    校验帧头(0x5A)和帧尾(0xA5)后将数据拷贝到MiniPC_ReceivePacket结构体
 *********************************************************************************************************************/
void vision_connect(uint8_t *RecvUsartData)
{
    // determine whether the first word section of the frame head is 0xA5
    if (RecvUsartData[0] == 0X5A)
    {
        if (RecvUsartData[MINIPC_REVCLENGTH - 1] == 0XA5)
        {
            memcpy(&MiniPC_ReceivePacket, RecvUsartData, MINIPC_REVCLENGTH); // Copy the data to the structure
        }
    }
}