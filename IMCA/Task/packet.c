#include <string.h>
#include <stdio.h>
#include "packet.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#endif

#ifndef CH_OK
#define CH_OK   (0)
#endif

#ifndef CH_ERR
#define CH_ERR  (1)
#endif
uint32_t frame_count;
uint8_t Chaoneng_Imu; //IMU 200ms超时标志位
uint32_t frame_rate;  //获取帧频率

/**********************************************************************************************************************
  * @Func	 static void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len)
  * @Brief   CRC16校验值更新
  * @Param	 uint16_t *currect_crc：当前CRC值指针(输入/输出)
  *          const uint8_t *src：源数据
  *          uint32_t len：数据长度
  * @Retval	 无
  * @Note    用于IMU数据包的CRC16校验
*********************************************************************************************************************/
static void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len)
{
    uint32_t crc = *currect_crc;
    uint32_t j;
    for (j=0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *currect_crc = crc;
}


enum status
{
    kStatus_Idle,      //空闲状态
    kStatus_Cmd,       //命令字状态
    kStatus_LenLow,    //长度低位状态
    kStatus_LenHigh,   //长度高位状态
    kStatus_CRCLow,    //CRC低位状态
    kStatus_CRCHigh,   //CRC高位状态
    kStatus_Data,      //数据状态
};

/* 函数指针 */
static on_data_received_event event_handler;
static packet_t *RxPkt;

/**
 * @brief  初始化数据包解码模块
 * @note   生成初始化一个数据包接收器
 * @param  pkt 数据包指针
 * @param  func 数据接收成功回调函数
 * @code
 *      void OnDataReceived(packet_t *pkt)
 *      {
 *          pkt->buf 为数据 pkt->payload_len 为接收到的字节数 
 *      }
 *
 *      packet_t pkt;
 *      packet_decode_init(&pkt, OnDataReceived);
 * @endcode
 * @retval None
 */
void packet_decode_init(packet_t *pkt, on_data_received_event func)
{
    /* 保存回调函数指针，当数据包完整接收后会调用此函数 */
    event_handler = func;

    /* 清空数据包结构体内容，确保初始状态干净 */
    memset(pkt, 0, sizeof(packet_t));

    /* 保存数据包结构体指针，供后续解码过程使用 */
    RxPkt = pkt;
}

/**
 * @brief  解码IMU数据包
 * @note   在串口接收中断中调用此函数
 * @param  c 接收的数据字节
 * @retval CH_OK 解码成功
 */
uint32_t packet_decode(uint8_t c)
{
    static uint16_t CRCReceived = 0;            /* 从数据帧接收到的CRC值 */
    static uint16_t CRCCalculated = 0;          /* 从数据帧计算得到的CRC值 */
    static uint8_t status = kStatus_Idle;       /* 状态机 */
    static uint8_t crc_header[4] = {0x5A, 0xA5, 0x00, 0x00};
  
    switch(status)
    {
        case kStatus_Idle:
            if(c == 0x5A)
                status = kStatus_Cmd;
			break;
        case kStatus_Cmd:
            RxPkt->type = c;
			if(RxPkt->type == 0xA5)
				status = kStatus_LenLow;
            break;
        case kStatus_LenLow:
            RxPkt->payload_len = c;
            crc_header[2] = c;
            status = kStatus_LenHigh;
            break;
        case kStatus_LenHigh:
            RxPkt->payload_len |= (c<<8);
            crc_header[3] = c;
            status = kStatus_CRCLow;
            break;
        case kStatus_CRCLow:
            CRCReceived = c;
            status = kStatus_CRCHigh;
            break;
        case kStatus_CRCHigh:
            CRCReceived |= (c<<8);
            RxPkt->ofs = 0;
            CRCCalculated = 0;
            status = kStatus_Data;
            break;
        case kStatus_Data:
	
            RxPkt->buf[RxPkt->ofs++] = c;

            if(RxPkt->ofs >= MAX_PACKET_LEN)
            {
                status = kStatus_Idle;
                return CH_ERR;   
            }

            if(RxPkt->ofs >= RxPkt->payload_len && RxPkt->type == 0xA5)
            {
                /* 计算CRC */
                crc16_update(&CRCCalculated, crc_header, 4);
                crc16_update(&CRCCalculated, RxPkt->buf, RxPkt->ofs);
                
                /* CRC匹配 */
                if(CRCCalculated == CRCReceived)
                {
					frame_count++;
                    event_handler(RxPkt);
                }
                status = kStatus_Idle;
            }
            break;
        default:
            status = kStatus_Idle;
            break;
    }
    return CH_OK;
}