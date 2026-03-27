#ifndef _CAPINTERACTH
#define _CAPINTERACTH

#include "stm32g4xx_hal.h"

/** @defgroup SUPERCAP_state_code
  *
  * [ 0x01 : 0x1f ] : 软件版本号
  * [ 0x20 : 0x9f ] : 保留区
  *
  * 0x00:未启动(启动电压>20V)
  *
  * 0xA1:正常工作()
  * 0xA2:环路自检中
  * 0xA3:高压容组模式
  * 0xA9:超电接受到的数据包，和校验不通过
  * 0xAA:命令执行确认
  * 0xE1(已移除):电流环异常，停止工作，禁止重新上电 (蜂鸣器长鸣)
  * 0xE2:电流环正常，但连续触发安全限制，暂停工作，等待确认信号，检查是否有电路，或者负载过大(蜂鸣器，间隔1秒两声)
  * 0xEE:串口debug中，停止状态
*/

/** @defgroup SUPERCAP_cmd_code
 * 0x20 : 查询软件版本号
 * 0xB1 ：确认线路正常，恢复自动启动MOS管(仅在0xE2停机状态和手动停止状态下有效)
 * 0xB2 : 主动停止MOS管，暂停自动重启，需手动启动（0XB1）
 * 0xC1 : 设置Can发送间隔，默认10ms，参数范围(uint8_t) 0-255,0为响应式，单位为毫秒
 * [0xC2 : 0xCf ]（未开放）: 设置反馈的数据的均值平均周期，需含命令参数：0-255,0为瞬时值，单位为毫秒
 * 0xD1: 设置百分比缓冲的值 默认90%；参数范围(uint8_t) 0-100 ，不保存到flash
 * 0xD2（未开放）:同上，但保存到flash
 * 0xD3(已移除)：设置电容充电电压，值：(uint8_t)(电压值 * 10),内部限制为1.3V-24.3V，若大于21.5V，当满足一定条件时，会限制到21.5V
 * 0xD4:设置电容组充电电压,0x01:旧版24.3V电容组，实际限压23.5V；0x02：新版25.8V电容组，实际限压25.5V
 */

/*接收到的超电的参数结构体*/
typedef struct{
	int16_t cap_value;
	int16_t cap_power;
	int16_t chassis_power;
	uint8_t battary_power;
	uint8_t state_core; //@arg SUPERCAP_state_code
	uint8_t sum_check;
	uint8_t buffer_mode;// 0x01:保守（-2w） 0x02：均衡（缓存<60,降功率） 0x03 :激进（缓存>50，逐步提高功率）
}cap_measure_t;

void PowerSend(void);
void PowerSend_Cmd(uint8_t cmd,uint8_t cmd_val);
void SendCapPowerData(FDCAN_HandleTypeDef *hfdcan, uint8_t buffer_mode, uint8_t power_limit, uint8_t buffer, uint8_t cmd, uint8_t cmd_val);
void GetCapMeasure(cap_measure_t* data, uint8_t buff[]);

extern cap_measure_t Cap_Measure;
extern uint8_t shut_cap_switch;

#endif



