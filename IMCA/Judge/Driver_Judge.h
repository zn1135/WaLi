#pragma once
#ifndef __DRIVER_JUDGE_H
#define __DRIVER_JUDGE_H
#include "stm32G4xx_hal.h"

#define JudgeBufferLength       150
#define JudgeFrameLength_1      46
#define JudgeFrameLength_2      11
#define JudgeFrameLength_3      24

#define JudgeFrameHeader        0xA5        //֡ͷ 

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
    
void Judge_InitConfig(void);

extern const unsigned char CRC8_INIT ;
extern uint16_t CRC_INIT ;  

#endif
