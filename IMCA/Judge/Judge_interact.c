#include "judge_interact.h"
#include "usart.h"
#include "string.h"
#include "Driver_Judge.h"
#include "gpio.h"
#include "ChassisControl.h"
#include "CapInteract.h"


commucation_t Arm_Com;

/* 控制系统参数 */
set_control_judge Key_Set;

/* 发给云台/控制端的实时值 */
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


/* 裁判系统协议 v1.6.1*/


/* 常规通道数据 */
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

/* 图传/交互通道 */
TuC_custom_control_t                       Rec_custom_robot_data;              // 0302
TuC_remote_control_t                       Rec_remote_control;                 // 0304
TuC_custom_client_data_t                   Rec_custom_client_data;             // 0306

Judge_RecData_t  Judge_RecData;
uint8_t judge_RxBuffer[Judge_RXBUF_SIZE]; // 裁判系统数据接收缓冲区

/*
 * @note 读取裁判系统数据帧
 * @param RcUsartData 接收数据的起始地址
 * @return OK 校验通过
 */


uint8_t Judge_Read(uint8_t* RcUsartData)
{
    uint8_t  judge_length = 0;

    Judge_RecData.crc8 = RcUsartData[4];
    Judge_RecData.datalength =( (RcUsartData[2]<<8) | RcUsartData[1] );
    Judge_RecData.cmd_id = ( (RcUsartData[6]<<8) | RcUsartData[5] );
    if(RcUsartData[0] == Judge_Frame_Header) // 帧头 0xA5
    {
        if(Verify_CRC8_Check_Sum(RcUsartData, Judge_Len_Header) ==OK) // 帧头 CRC8 校验
        {

            judge_length = Judge_RecData.datalength + Judge_Len_Header +  Judge_Len_Cmdid + Judge_Len_Tail; // 整帧长度

            Judge_RecData.crc16 =(RcUsartData[judge_length-1]<<8|RcUsartData[judge_length-2]);

            if(Verify_CRC16_Check_Sum(RcUsartData,judge_length)==OK) // CRC16 校验
            {
                switch(Judge_RecData.cmd_id)
                {
                case ID_game_state: memcpy(&Rec_game_state,RcUsartData+7,Judge_RecData.datalength); break;          // 0x0001
                case ID_game_result: memcpy(&Rec_game_result,RcUsartData+7,Judge_RecData.datalength); break;        // 0x0002
                case ID_game_robot_HP: memcpy(&Rec_game_robot_HP,RcUsartData+7,Judge_RecData.datalength); break;     // 0x0003
                case ID_dart_status: memcpy(&Rec_game_robot_HP,RcUsartData+7,Judge_RecData.datalength); break;       // 0x0004 飞镖状态
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

// 读取颜色：0 红 / 1 蓝
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

uint16_t ID1_17_cooling_limit(void) // 17mm 枪口热量上限（枪口1）
{
    return Rec_game_robot_status.shooter_barrel_heat_limit;
}

uint16_t FireCoolingRate(void) // 每秒热量冷却值
{
    return Rec_game_robot_status.shooter_barrel_cooling_value;
}

uint16_t FireHeat17(void) // 17mm 枪口热量（实时）
{
    return Rec_power_heat_data.shooter_id1_17mm_cooling_heat;
}

uint16_t RoboHP(void) // 实时血量
{
    return Rec_game_robot_status.current_HP;
}

float FireSpeed(void) // 弹丸初速度 (m/s)
{
    return Rec_shoot_data.bullet_speed;
}

uint8_t FireFeq(void) // 射频 (Hz)
{
    return Rec_shoot_data.bullet_freq;
}

uint16_t RoboHP1(void)
{
    return Rec_game_robot_HP.red_1_robot_HP;
}
uint16_t RoboHP2(void)
{
    return Rec_game_robot_HP.red_2_robot_HP;
}
uint16_t RoboHP3(void)
{
    return Rec_game_robot_HP.red_3_robot_HP;
}
uint16_t RoboHP4(void)
{
    return Rec_game_robot_HP.red_4_robot_HP;
}
uint16_t RoboHP5(void)
{
    return Rec_game_robot_HP.red_5_robot_HP;
}

uint32_t RFID_status(void)
{
    return Rec_rfid_status.rfid_status;
}

uint8_t Robot_level(void)
{
    return Rec_game_robot_status.robot_level;
}

uint16_t Chassis_v(void)
{
    return Rec_power_heat_data.chassis_volt;
}

uint16_t Chassis_i(void)
{
    return Rec_power_heat_data.chassis_current;
}

float Chassis_power(void) // 底盘实时功率
{
    return Rec_power_heat_data.chassis_power;
}

uint16_t send_chassis_power_limit(void)
{
    return Rec_game_robot_status.chassis_power_limit;
}

uint16_t Chassis_power_buffer(void) // 缓冲能量
{
    return Rec_power_heat_data.chassis_power_buffer;
}

uint16_t shoot_42_heat(void)
{
    return Rec_power_heat_data.shooter_id1_42mm_cooling_heat;
}

float My_x(void)
{
    return Rec_game_robot_pos.x;
}

float My_y(void)
{
    return Rec_game_robot_pos.y;
}

uint8_t RobotAimColor(void)
{
    uint8_t aim_color;
    if(Rec_game_robot_status.robot_id == 1) // 机器人为红色
    {
        aim_color = 0; // 瞄准蓝色
    }
    else if(Rec_game_robot_status.robot_id == 101)  // 机器人为蓝色
    {
        aim_color = 1; // 瞄准红色
    }
    else if(Rec_game_robot_status.robot_id == 0)  // 未获取到 ID 默认红色
    {
        aim_color = 0;
    }
    return aim_color;
}

/*
 * @note 开启 UART4 空闲中断 + DMA
 */
void HAL_UART4_Receive_IT_IDLE()
{
    __HAL_UART_ENABLE_IT(&huart4,UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart4,judge_RxBuffer,Judge_RXBUF_SIZE);
}

void SetControlParameter(void)
{
/*
 功率上限与按键速度建议值
 等级   底盘功率         等级   底盘功率
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

