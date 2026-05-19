#include "judge_interact.h"
#include "usart.h"
#include "string.h"
#include "Driver_Judge.h"
#include "gpio.h"
#include "ChassisControl.h"
#include "CapInteract.h"


commucation_t Arm_Com;

/* ����ϵͳ���� */
set_control_judge Key_Set;

/* ������̨/���ƶ˵�ʵʱֵ */
uint16_t Speed_Limit;
uint16_t Heat_Limit;
uint16_t Heat;

//extern pid_t Chassis_ResidueJ;
//extern pid_t Chassis_Shift;
//extern pid_t Chassis_rotate;

/* send to gimbal */
uint16_t Speed_Limit;
uint16_t Heat_Limit;
uint16_t Heat;
uint8_t th2_val[4],th3_val[4],th4_val[4],th5_val[4],th6_val[4],di_val[4];


/* ����ϵͳЭ�� v1.6.1*/


/* ����ͨ������ */
ext_game_state_t                            Rec_game_state;                     // 0001
ext_game_result_t                          Rec_game_result;                    // 0002
ext_game_robot_HP_t                        Rec_game_robot_HP;                  // 0003
ext_ICRA_buff_debuff_zone_status_t         Rec_ICRA_buff_debuff_zone_status;   // 0005
ext_event_data_t                           Rec_event_data;                     // 0101
ext_supply_projectile_action_t             Rec_supply_projectile_action;       // 0103
ext_dart_remaining_time_t                  Rec_dart_remaining_time;            // 0105
ext_game_robot_status_t                    Rec_game_robot_status;              // 0201
ext_power_heat_data_t                      Rec_power_heat_data;                // 0202
ext_game_robot_pos_t                       Rec_game_robot_pos;                 // 0203
ext_buff_t                                 Rec_buff;                           // 0204
aerial_robot_energy_t                      Rec_robot_energy;                   // 0205
ext_robot_hurt_t                           Rec_robot_hurt;                     // 0206
ext_shoot_data_t                           Rec_shoot_data;                     // 0207
ext_bullet_remaining_t                     Rec_bullet_remaining;               // 0208
ext_rfid_status_t                          Rec_rfid_status;                    // 0209
ext_dart_client_cmd_t                      Rec_dart_client_cmd;                // 020A
ext_robot_interact_data_t                  Rec_robot_interact_data;            // 0301
ext_map_robot_data_t                       Rec_map_robot_data;                 // 0305
ext_custom_info_t                          Rec_custom_info;                    // 0308

/* ͼ��/����ͨ�� */
TuC_custom_control_t                       Rec_custom_robot_data;              // 0302
TuC_remote_control_t                       Rec_remote_control;                 // 0304
TuC_custom_client_data_t                   Rec_custom_client_data;             // 0306

Judge_RecData_t  Judge_RecData;
uint8_t judge_RxBuffer[Judge_RXBUF_SIZE]; // ����ϵͳ���ݽ��ջ�����

/*
 * @note ��ȡ����ϵͳ����֡
 * @param RcUsartData �������ݵ���ʼ��ַ
 * @return OK У��ͨ��
 */


/**********************************************************************************************************************
  * @Func	 uint8_t Judge_Read(uint8_t* RcUsartData)
  * @Brief   解析裁判系统串口数据帧
  * @Param	 uint8_t* RcUsartData：接收数据缓冲区首地址
  * @Retval	 uint8_t：OK表示校验通过
  * @Note    校验帧头(0xA5)→CRC8→CRC16后按cmd_id分发到对应结构体，支持连续多帧解析
*********************************************************************************************************************/
uint8_t Judge_Read(uint8_t* RcUsartData)
{
    uint8_t  judge_length = 0;

    Judge_RecData.crc8 = RcUsartData[4];
    Judge_RecData.datalength =( (RcUsartData[2]<<8) | RcUsartData[1] );
    Judge_RecData.cmd_id = ( (RcUsartData[6]<<8) | RcUsartData[5] );
    if(RcUsartData[0] == Judge_Frame_Header) // ֡ͷ 0xA5
    {
        if(Verify_CRC8_Check_Sum(RcUsartData, Judge_Len_Header) ==OK) // ֡ͷ CRC8 У��
        {

            judge_length = Judge_RecData.datalength + Judge_Len_Header +  Judge_Len_Cmdid + Judge_Len_Tail; // ��֡����

            Judge_RecData.crc16 =(RcUsartData[judge_length-1]<<8|RcUsartData[judge_length-2]);

            if(Verify_CRC16_Check_Sum(RcUsartData,judge_length)==OK) // CRC16 У��
            {
                switch(Judge_RecData.cmd_id)
                {
                case ID_game_state: memcpy(&Rec_game_state,RcUsartData+7,Judge_RecData.datalength); break;          // 0x0001
                case ID_game_result: memcpy(&Rec_game_result,RcUsartData+7,Judge_RecData.datalength); break;        // 0x0002
                case ID_game_robot_HP: memcpy(&Rec_game_robot_HP,RcUsartData+7,Judge_RecData.datalength); break;     // 0x0003
                case ID_dart_status: memcpy(&Rec_game_robot_HP,RcUsartData+7,Judge_RecData.datalength); break;       // 0x0004 ����״̬
                case ID_ICRA_buff_debuff_zone_status: memcpy(&Rec_ICRA_buff_debuff_zone_status,RcUsartData+7,Judge_RecData.datalength); break; // 0x0005
                case ID_event_data: memcpy(&Rec_event_data,RcUsartData+7,Judge_RecData.datalength); break;           // 0x0101
                case ID_supply_projectile_action: memcpy(&Rec_supply_projectile_action,RcUsartData+7,Judge_RecData.datalength); break; // 0x0102
                case ID_referee_warning: memcpy(&Rec_supply_projectile_action,RcUsartData+7,Judge_RecData.datalength); break;            // 0x0104
                case ID_dart_remaining_time: memcpy(&Rec_dart_remaining_time,RcUsartData+7,Judge_RecData.datalength); break;             // 0x0105
                case ID_game_robot_state: memcpy(&Rec_game_robot_status,RcUsartData+7,Judge_RecData.datalength); break;                   // 0x0201
                case ID_power_heat_data: memcpy(&Rec_power_heat_data,RcUsartData+7,Judge_RecData.datalength); break;                       // 0x0202
                case ID_game_robot_pos: memcpy(&Rec_game_robot_pos,RcUsartData+7,Judge_RecData.datalength); break;                         // 0x0203
                case ID_buff: memcpy(&Rec_buff,RcUsartData+7,Judge_RecData.datalength); break;                                             // 0x0204
                case ID_aerial_robot_energy: memcpy(&Rec_robot_energy,RcUsartData+7,Judge_RecData.datalength); break;                      // 0x0205
                case ID_robot_hurt: memcpy(&Rec_robot_hurt,RcUsartData+7,Judge_RecData.datalength); break;                                 // 0x0206
                case ID_shoot_data: memcpy(&Rec_shoot_data,RcUsartData+7,Judge_RecData.datalength); break;                                 // 0x0207
                case ID_bullet_remaining: memcpy(&Rec_bullet_remaining,RcUsartData+7,Judge_RecData.datalength); break;                     // 0x0208
                case ID_rfid_status: memcpy(&Rec_rfid_status,RcUsartData+7,Judge_RecData.datalength); break;                               // 0x0209
                case ID_dart_client_cmd: memcpy(&Rec_dart_client_cmd,RcUsartData+7,Judge_RecData.datalength); break;                       // 0x020A
                case ID_commu_data_cmd: break;                                                                                            // 0x0210
                case ID_student_interactive_header_data: break;                                                                           // 0x0301
                case ID_custom_robot_data: memcpy(&Rec_custom_robot_data,RcUsartData+7,Judge_RecData.datalength); break;                  // 0x0302
                case ID_remote_control: memcpy(&Rec_remote_control,RcUsartData+7,Judge_RecData.datalength); break;                        // 0x0304
                case ID_custom_client_data: memcpy(&Rec_custom_client_data,RcUsartData+7,Judge_RecData.datalength); break;                // 0x0306
                default: break;

                }

            }

        }
        Judge_RecData.next_frame_header =*(RcUsartData+Judge_Len_Header+Judge_Len_Cmdid+Judge_Len_Tail+Judge_RecData.datalength);
        if(*(RcUsartData+Judge_Len_Header+Judge_Len_Cmdid+Judge_Len_Tail+Judge_RecData.datalength)==0XA5)
        {
            Judge_Read(RcUsartData+Judge_Len_Header+Judge_Len_Cmdid+Judge_Len_Tail+Judge_RecData.datalength);
        }
    }

    return OK;
}

// ��ȡ��ɫ��0 �� / 1 ��
/**********************************************************************************************************************
  * @Func	 uint8_t RoboColor(void)
  * @Brief   获取机器人颜色
  * @Param	 无
  * @Retval	 uint8_t：0为红方，1为蓝方
*********************************************************************************************************************/
uint8_t RoboColor(void)
{
    static uint8_t color_t = 0;
    if(Rec_game_robot_status.robot_id<=7)
    {
        return color_t = 0;
    }
    else
    {
        return color_t = 1;
    }
}

/**********************************************************************************************************************
  * @Func	 uint16_t ID1_17_cooling_limit(void)
  * @Brief   获取17mm枪管热量上限
  * @Param	 无
  * @Retval	 uint16_t：热量上限值
*********************************************************************************************************************/
uint16_t ID1_17_cooling_limit(void) // 17mm ǹ���������ޣ�ǹ��1��
{
    return Rec_game_robot_status.shooter_barrel_heat_limit;
}

/**********************************************************************************************************************
  * @Func	 uint16_t FireCoolingRate(void)
  * @Brief   获取每秒冷却值
  * @Param	 无
  * @Retval	 uint16_t：冷却速率
*********************************************************************************************************************/
uint16_t FireCoolingRate(void) // ÿ��������ȴֵ
{
    return Rec_game_robot_status.shooter_barrel_cooling_value;
}

/**********************************************************************************************************************
  * @Func	 uint16_t FireHeat17(void)
  * @Brief   获取17mm枪管实时热量
  * @Param	 无
  * @Retval	 uint16_t：当前热量值
*********************************************************************************************************************/
uint16_t FireHeat17(void) // 17mm ǹ��������ʵʱ��
{
    return Rec_power_heat_data.shooter_id1_17mm_cooling_heat;
}

/**********************************************************************************************************************
  * @Func	 uint16_t RoboHP(void)
  * @Brief   获取机器人实时血量
  * @Param	 无
  * @Retval	 uint16_t：当前血量值
*********************************************************************************************************************/
uint16_t RoboHP(void) // ʵʱѪ��
{
    return Rec_game_robot_status.current_HP;
}

/**********************************************************************************************************************
  * @Func	 float FireSpeed(void)
  * @Brief   获取弹丸速度
  * @Param	 无
  * @Retval	 float：弹丸速度(m/s)
*********************************************************************************************************************/
float FireSpeed(void) // ������ٶ� (m/s)
{
    return Rec_shoot_data.bullet_speed;
}

/**********************************************************************************************************************
  * @Func	 uint8_t FireFeq(void)
  * @Brief   获取射速频率
  * @Param	 无
  * @Retval	 uint8_t：射频(Hz)
*********************************************************************************************************************/
uint8_t FireFeq(void) // ��Ƶ (Hz)
{
    return Rec_shoot_data.bullet_freq;
}

/**********************************************************************************************************************
  * @Func	 uint16_t RoboHP1(void)
  * @Brief   获取红1号机器人血量
  * @Param	 无
  * @Retval	 uint16_t：血量值
*********************************************************************************************************************/
uint16_t RoboHP1(void)
{
    return Rec_game_robot_HP.red_1_robot_HP;
}
/**********************************************************************************************************************
  * @Func	 uint16_t RoboHP2(void)
  * @Brief   获取红2号机器人血量
  * @Param	 无
  * @Retval	 uint16_t：血量值
*********************************************************************************************************************/
uint16_t RoboHP2(void)
{
    return Rec_game_robot_HP.red_2_robot_HP;
}
/**********************************************************************************************************************
  * @Func	 uint16_t RoboHP3(void)
  * @Brief   获取红3号机器人血量
  * @Param	 无
  * @Retval	 uint16_t：血量值
*********************************************************************************************************************/
uint16_t RoboHP3(void)
{
    return Rec_game_robot_HP.red_3_robot_HP;
}
/**********************************************************************************************************************
  * @Func	 uint16_t RoboHP4(void)
  * @Brief   获取红4号机器人血量
  * @Param	 无
  * @Retval	 uint16_t：血量值
*********************************************************************************************************************/
uint16_t RoboHP4(void)
{
    return Rec_game_robot_HP.red_4_robot_HP;
}
/**********************************************************************************************************************
  * @Func	 uint16_t RoboHP5(void)
  * @Brief   获取红5号机器人血量
  * @Param	 无
  * @Retval	 uint16_t：血量值
*********************************************************************************************************************/
uint16_t RoboHP5(void)
{
    return Rec_game_robot_HP.red_5_robot_HP;
}

/**********************************************************************************************************************
  * @Func	 uint32_t RFID_status(void)
  * @Brief   获取RFID状态
  * @Param	 无
  * @Retval	 uint32_t：RFID状态值
*********************************************************************************************************************/
uint32_t RFID_status(void)
{
    return Rec_rfid_status.rfid_status;
}

/**********************************************************************************************************************
  * @Func	 uint8_t Robot_level(void)
  * @Brief   获取机器人等级
  * @Param	 无
  * @Retval	 uint8_t：机器人等级
*********************************************************************************************************************/
uint8_t Robot_level(void)
{
    return Rec_game_robot_status.robot_level;
}

/**********************************************************************************************************************
  * @Func	 uint16_t Chassis_v(void)
  * @Brief   获取底盘电压
  * @Param	 无
  * @Retval	 uint16_t：底盘电压值(mV)
*********************************************************************************************************************/
uint16_t Chassis_v(void)
{
    return Rec_power_heat_data.chassis_volt;
}
/**********************************************************************************************************************
  * @Func	 uint16_t Chassis_i(void)
  * @Brief   获取底盘电流
  * @Param	 无
  * @Retval	 uint16_t：底盘电流值(mA)
*********************************************************************************************************************/
uint16_t Chassis_i(void)
{
    return Rec_power_heat_data.chassis_current;
}
/**********************************************************************************************************************
  * @Func	 float Chassis_power(void)
  * @Brief   获取底盘实时功率
  * @Param	 无
  * @Retval	 float：底盘功率值(W)
*********************************************************************************************************************/
float Chassis_power(void) // ����ʵʱ����
{
    return Rec_power_heat_data.chassis_power;
}

/**********************************************************************************************************************
  * @Func	 uint16_t send_chassis_power_limit(void)
  * @Brief   获取裁判系统底盘功率上限
  * @Param	 无
  * @Retval	 uint16_t：功率上限值(W)
*********************************************************************************************************************/
uint16_t send_chassis_power_limit(void)
{
    return Rec_game_robot_status.chassis_power_limit;
}

/**********************************************************************************************************************
  * @Func	 uint16_t Chassis_power_buffer(void)
  * @Brief   获取底盘功率缓冲值
  * @Param	 无
  * @Retval	 uint16_t：功率缓冲值
*********************************************************************************************************************/
uint16_t Chassis_power_buffer(void) // ��������
{
    return Rec_power_heat_data.chassis_power_buffer;
}

/**********************************************************************************************************************
  * @Func	 uint16_t shoot_42_heat(void)
  * @Brief   获取42mm枪管冷却热量
  * @Param	 无
  * @Retval	 uint16_t：42mm枪管热量值
*********************************************************************************************************************/
uint16_t shoot_42_heat(void)
{
    return Rec_power_heat_data.shooter_id1_42mm_cooling_heat;
}

/**********************************************************************************************************************
  * @Func	 float My_x(void)
  * @Brief   获取机器人X坐标
  * @Param	 无
  * @Retval	 float：X轴坐标
*********************************************************************************************************************/
float My_x(void)
{
    return Rec_game_robot_pos.x;
}
/**********************************************************************************************************************
  * @Func	 float My_y(void)
  * @Brief   获取机器人Y坐标
  * @Param	 无
  * @Retval	 float：Y轴坐标
*********************************************************************************************************************/
float My_y(void)
{
    return Rec_game_robot_pos.y;
}

/**********************************************************************************************************************
  * @Func	 uint8_t send_robot_hurt_id(void)
  * @Brief   获取机器人受击装甲板ID
  * @Param	 无
  * @Retval	 uint8_t：装甲板ID
*********************************************************************************************************************/
uint8_t send_robot_hurt_id() {
    return Rec_robot_hurt.armor_id;
}
/**********************************************************************************************************************
  * @Func	 uint8_t send_robot_hurt_type(void)
  * @Brief   获取机器人受击伤害类型
  * @Param	 无
  * @Retval	 uint8_t：伤害类型
*********************************************************************************************************************/
uint8_t send_robot_hurt_type() {
    return Rec_robot_hurt.hurt_type;
}


/**********************************************************************************************************************
  * @Func	 uint8_t RobotAimColor(void)
  * @Brief   获取自身机器人对应的瞄准颜色
  * @Param	 无
  * @Retval	 uint8_t：0瞄准红色，1瞄准蓝色
  * @Note    根据robot_id判断：1~5红方(瞄准蓝方=1)，101~105蓝方(瞄准红方=0)
*********************************************************************************************************************/
uint8_t RobotAimColor(void)
{
    uint8_t aim_color;
    if(Rec_game_robot_status.robot_id == 1) // ������Ϊ��ɫ
    {
        aim_color = 0; // ��׼��ɫ
    }
    else if(Rec_game_robot_status.robot_id == 101)  // ������Ϊ��ɫ
    {
        aim_color = 1; // ��׼��ɫ
    }
    else if(Rec_game_robot_status.robot_id == 0)  // δ��ȡ�� ID Ĭ�Ϻ�ɫ
    {
        aim_color = 0;
    }
    return aim_color;
}

/*
 * @note ���� UART4 �����ж� + DMA
 */
/**********************************************************************************************************************
  * @Func	 void HAL_UART4_Receive_IT_IDLE(void)
  * @Brief   启动UART4空闲中断+DMA接收
  * @Param	 无
  * @Retval	 无
  * @Note    用于裁判系统数据接收
*********************************************************************************************************************/
void HAL_UART4_Receive_IT_IDLE()
{
    __HAL_UART_ENABLE_IT(&huart4,UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart4,judge_RxBuffer,Judge_RXBUF_SIZE);
}

/**********************************************************************************************************************
  * @Func	 void SetControlParameter(void)
  * @Brief   根据裁判系统功率等级设置键盘速度参数
  * @Param	 无
  * @Retval	 无
  * @Note    根据chassis_power_limit映射key_speed/shift_speed/ctrl_speed三档速度
*********************************************************************************************************************/
void SetControlParameter(void)
{
/*
 功率等级与底盘功率对应表
 等级   步兵底盘功率     等级   英雄底盘功率
 1      60               1      45
 2      65               2      50
 3      70               3      55
 4      75               4      60
 5      80               5      65
 6      85               6      70
 7      90               7      75
 8      95               8      80
 9      100              9      90
 10     100              10     100
*/
    switch(send_chassis_power_limit())
    {
      case 45:{
                Key_Set.key_speed = 2.5f;
                Key_Set.shift_speed = 2.8f;
                Key_Set.ctrl_speed = 3.5f;}break;
      case 50:{
                Key_Set.key_speed = 2.5f;
                Key_Set.shift_speed = 2.8f;
                Key_Set.ctrl_speed = 3.5f;}break;
      case 55:{
                Key_Set.key_speed = 1.9f;
                Key_Set.shift_speed = 2.2f;
                Key_Set.ctrl_speed = 3.5f;}break;
      case 60:{
                Key_Set.key_speed = 3.8f;
                Key_Set.shift_speed = 4.2f;
                Key_Set.ctrl_speed = 3.5f;}break;
      case 65:{
                Key_Set.key_speed = 2.9f;
                Key_Set.shift_speed = 3.1f;
                Key_Set.ctrl_speed = 3.5f;}break;
      case 70:{
                Key_Set.key_speed = 2.9f;
                Key_Set.shift_speed = 3.1f;
                Key_Set.ctrl_speed = 3.5f;}break;
      case 75:{
                Key_Set.key_speed = 3.0f;
                Key_Set.shift_speed = 3.4f;
                Key_Set.ctrl_speed = 3.5f;}break;
      case 80:{
                Key_Set.key_speed = 2.6f;
                Key_Set.shift_speed = 3.1f;
                Key_Set.ctrl_speed = 3.5f;}break;
      case 100:{
                Key_Set.key_speed = 2.8f;
                Key_Set.shift_speed = 3.3f;
                Key_Set.ctrl_speed = 3.5f;}break;
      default:{
                Key_Set.key_speed = 2.0f;
                Key_Set.shift_speed = 2.5f;
                Key_Set.ctrl_speed = 3.5f;}break;
    }
}





/*
2024
1  2  3  4  5  6  7  8   9  10
60-65-70-75-80-85-90-95-100-100
45-50-55-60-65-70-75-80-90 -100
*/
//void SetControlParameter(void)
//{
//  float Spd, Rotate_Spd;
//    Control_Set.chassis_max_speed = (FL202(send_chassis_power_limit()) - Chassis_ResidueJ.pos_out)*SuperCapRatio(Cap_Measure.cap_value)*0.7;//
//    Spd  = FL202(send_chassis_power_limit()+20) - Chassis_ResidueJ.pos_out;//
//    Control_Set.chassis_shift_max_speed = (FL202(send_chassis_power_limit()) - Chassis_Shift.pos_out)*SuperCapRatio(Cap_Measure.cap_value);
//    Control_Set.chassis_ctrl_max_speed=FL202(120);
//
//    Control_Set.rotate_move_max_speed = Spd*CHASSIS_WHEEL_RADIUS*CHASSIS_WHEEL_RADIUS*RPM_To_Rads*SuperCapRatio(Cap_Measure.cap_value)*0.5;
//    Rotate_Spd = Spd*CHASSIS_WHEEL_RADIUS*CHASSIS_WHEEL_RADIUS*RPM_To_Rads*SuperCapRatio(Cap_Measure.cap_value);
//    Control_Set.chassis_rotate_max_speed = Rotate_Spd*0.5f*SuperCapRatio(Cap_Measure.cap_value);
//
//}

