#include <string.h>
#include <stdio.h>


#include "imu_data_decode.h"








uint8_t IMURxBuffer[150]; // 用来接收串口5的数据
receive_imusol_packet_t receive_imusol;

/**********************************************************************************************************************
  * @Func	 void IMU_Data_Uptade(uint8_t *data, uint16_t len)
  * @Brief   超能IMU数据解析更新
  * @Param	 uint8_t *data：IMU原始数据
  *          uint16_t len：数据长度
  * @Retval	 无
  * @Note    校验帧头0x5A 0xA5和长度后，按数据ID拷贝到对应结构体
*********************************************************************************************************************/
void IMU_Data_Uptade(uint8_t *data, uint16_t len)
{
    if (data[0] != 0x5A || data[1] != 0xA5) {
        return;
    }
    if (len != (data[2] | data[3] << 8)) {
        return;
    }
    switch (data[6]) {
    case KItemIMUSOL:
            memcpy((void *) receive_imusol.acc,  data+ 18 , sizeof(float) * 16);
        break;

    }

}