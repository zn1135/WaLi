#include "CapInteract.h"
#include "Judge_interact.h"
#include "CanBus_Task.h"
#include "fdcan.h"
#include "RcTask.h"


cap_measure_t Cap_Measure = {.buffer_mode =0x02};
uint8_t shut_cap_switch;
/**
 * @brief :超电通信协议
 * @version :适配超电软件v2.1.8（0x03）
 * @details
 *  注意：1、由于裁判系统的缓冲值是每秒结算10次的，因此过高的发送频率是浪费can带宽的，
 * 		建议降低发送速率到10Hz，且当缓冲值更新时立即发送数据。
 *		2、如需使用命令控制超电，应该使用函数 PowerSend_Cmd
 *		3、缓冲模式： 0x01:保守（-2w） 0x02：均衡（缓存<60,降功率） 0x03 :激进（缓存>50，逐步提高功率）
 * 
*/

void PowerSend(void)
{
	uint8_t chassis_power_buffer=Chassis_power_buffer();
	uint8_t power_limit=0;
	if (KEY_SHIFT) {
		power_limit=10.0;
	}else {
		power_limit=send_chassis_power_limit();
	}



	SendCapPowerData(&hfdcan3, Cap_Measure.buffer_mode,power_limit,chassis_power_buffer,0x00,0x00);
}
/**********************************************************************************************************************
  * @Func	 void PowerSend_Cmd(uint8_t cmd, uint8_t cmd_val)
  * @Brief   发送控制命令给超电功率板
  * @Param	 uint8_t cmd：命令码，详见SUPERCAP_cmd_code
  *          uint8_t cmd_val：命令参数值
  * @Retval	 无
  * @Note    通过FDCAN3发送到功率板(0x107)
*********************************************************************************************************************/
void PowerSend_Cmd(uint8_t cmd,uint8_t cmd_val)
{
	uint8_t chassis_power_buffer = Chassis_power_buffer();
	uint8_t power_limit = send_chassis_power_limit();
	SendCapPowerData(&hfdcan3, Cap_Measure.buffer_mode, power_limit, chassis_power_buffer, cmd, cmd_val);
}

/**********************************************************************************************************************
* @brief    主控发送给功率板当前底盘参数
 * @param	   
*            buffer_mode为缓冲模式，buffer为剩余的缓冲能量，power_limit为当前底盘功率上限，cmd 控制命令，详细在 @arg SUPERCAP_cmd_code
 * @petval	 None
**********************************************************************************************************************/
void SendCapPowerData(FDCAN_HandleTypeDef *hfdcan, uint8_t buffer_mode, uint8_t power_limit, uint8_t buffer, uint8_t cmd, uint8_t cmd_val)
{
    FDCAN_TxHeaderTypeDef   tx_header={0};
    uint8_t               tx_data[8]={0};
    // uint8_t               chassis_data[2];
	
	//   Uint16ToChar(power_limit,chassis_data,2);
	  
	tx_header.Identifier = 0x107;
	tx_header.IdType = FDCAN_STANDARD_ID;
	tx_header.TxFrameType = FDCAN_DATA_FRAME;
	tx_header.DataLength = 0x08;
	tx_data[0] = buffer_mode;
	tx_data[1] = power_limit;
	tx_data[2] = buffer;
	tx_data[3] = cmd;
	tx_data[4] = cmd_val;

	uint8_t sum=0;
	for (size_t i = 0; i < 7; i++)
	{
		sum+=tx_data[i];
	}
	tx_data[7] =sum;

	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);	
}


/**********************************************************************************************************************
 * @brief    接收CAN通信超电传输过来的信息
 * @param	   
 *            uint8_t buff[]：CAN接收数据缓存区
 * [0]:	电容电压 
 * [1:2]:	电容功率		int16_t[高8位:低8位]		//瞬时
 * [3;4]:	底盘电机功率	int16_t[高8位:低8位]		//瞬时
 * [5]:	电池裁判功率	uint8_t					//瞬时
 * [6]:	状态码/响应码 			@arg SUPERCAP_state_code
 * [7]:	和校验 : 前7位数据的加和
 * 
 * @petval	  反馈报文0x108
**********************************************************************************************************************/
void GetCapMeasure(cap_measure_t* data, uint8_t buff[])
{
	uint8_t cap_Buff;
	int16_t cap_power;
	int16_t chassis_power;
	uint8_t battary_power;
	uint8_t state_core;

	uint8_t sum = 0;
	for (size_t i = 0; i < 7; i++)
	{
		sum += buff[i];
	}
	if (sum == buff[7])
		data->sum_check = 0x01;
	else
	{
		data->sum_check = 0x00;
		return;
	}

	cap_Buff = buff[0]; // 电压值 0 - 255//超出则为255
	cap_power = (buff[1] << 8) + buff[2];
	chassis_power = (buff[3] << 8) + buff[4];
	battary_power = buff[5];
	state_core = buff[6];


	data->cap_value = cap_Buff;
	data->cap_power = cap_power;
	data->chassis_power = chassis_power;
	data->battary_power = battary_power;
	data->state_core = state_core;
	

// if (data->state_core == 0xA1||data->state_core == 0xA3)// 关闭超电的mos管
// {
// 	PowerSend_Cmd(0xB2, 0);
// }


// if (data->state_core == 0xA1)// 启用高压容组模式
// {
// 	PowerSend_Cmd(0xD4, 0x02);
// }

}

// PowerSend_Cmd(0xC1 , 0x01); //设置can发送间隔为0x01毫秒