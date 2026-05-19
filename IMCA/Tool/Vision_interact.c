//
// Created by zn1135 on 25-11-29.
//

#include "Vision_interact.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "imu_data_decode.h"
#include "IMU_Task.h"
#include "Judge_interact.h"
#include "math.h"
#include "RcTask.h"
#include "usbd_cdc_if.h"
#include "user_lib.h"

bool MyColor;

uint8_t MiniPC_SendBuf[MINIPC_SENDLENGTH];
uint8_t MiniPC_RecBuf[MINIPC_REVCLENGTH];

MiniPC_ReceivePacket_Typedef MiniPC_ReceivePacket = {
    .header = 0,
  };

//Send head
MiniPC_SendPacket_Typedef MiniPC_SendPacket = {
    .header = 0x5A,
        .end    = 0xA5,
};



void MiniPC_SendVisionFrameInfo(MiniPC_SendPacket_Typedef *SendPacket)
{
    float temp;
    if(temp > PI)
        temp -= 2*PI;
    else if(temp < -PI)
        temp += 2*PI;
    for(int i=0;i<4;i++)
    {
        MiniPC_SendPacket.q[i] = receive_imusol.quat[i];
    }
    if(mouse_right.deshake_key)
    {
        MiniPC_SendPacket.mode = 1;
    }
    else if (KEY_V&&KEY_CTRL)
    {
        MiniPC_SendPacket.mode = 3;
    }
    else if (KEY_V)
    {
        MiniPC_SendPacket.mode = 2;
    }

    else MiniPC_SendPacket.mode = 0;
    MiniPC_SendPacket.yaw = IMU_Yaw.angle *PI/180;
    MiniPC_SendPacket.yaw_vel = IMU_Yaw.speed_gyro;
    //	MiniPC_SendPacket.yaw = xMotorParam[YAW].MeasureDM->position;
    //	MiniPC_SendPacket.pitch = xMotorParam[PITCH].MeasureDM->position;
    MiniPC_SendPacket.pitch = IMU_Pitch.angle *PI/180;//xMotorParam[PITCH].MeasureImu->angle *PI/180;//IMU_Look[1];
    MiniPC_SendPacket.my_color = !RoboColor();	//send rival's color
    memcpy(MiniPC_SendBuf,(uint8_t *)SendPacket,MINIPC_SENDLENGTH);
    CDC_Transmit_FS(MiniPC_SendBuf, MINIPC_SENDLENGTH);
}

void vision_connect(uint8_t *RecvUsartData)
{
    // determine whether the first word section of the frame head is 0xA5
    if(RecvUsartData[0] == 0X5A)
    {
        if(RecvUsartData[MINIPC_REVCLENGTH - 1] == 0XA5)
        {
            memcpy(&MiniPC_ReceivePacket,RecvUsartData,MINIPC_REVCLENGTH);//Copy the data to the structure
        }

    }
}