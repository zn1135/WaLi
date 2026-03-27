//
// Created by zn1135 on 25-11-29.
//

#ifndef VISION_INTERACT_H
#define VISION_INTERACT_H

// MiniPC发送数据长度
#define MINIPC_SENDLENGTH					32U
// MiniPC接收数据长度
#define MINIPC_REVCLENGTH					27U
#include <stdbool.h>
#include <stdint.h>
#include "main.h"
typedef struct __attribute__((packed)) MiniPc_SendPacket_Typedef
{
    uint8_t header;
    uint8_t mode;
    float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    uint8_t my_color;
    uint8_t end;
} MiniPC_SendPacket_Typedef;

typedef struct __attribute__((packed)) MiniPC_ReceivePacket_Typedef
{
    uint8_t header;
    uint8_t fireflag;		//开火标志位
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint8_t end;
}MiniPC_ReceivePacket_Typedef;

extern bool MyColor;
extern MiniPC_SendPacket_Typedef MiniPC_SendPacket;
extern MiniPC_ReceivePacket_Typedef MiniPC_ReceivePacket;

extern uint8_t MiniPC_RecBuf[MINIPC_REVCLENGTH];
extern uint8_t MiniPC_SendBuf[MINIPC_SENDLENGTH];



void MiniPC_SendVisionFrameInfo(MiniPC_SendPacket_Typedef *SendPacket);
void vision_connect(uint8_t *RecvUsartData);
#endif //VISION_INTERACT_H
