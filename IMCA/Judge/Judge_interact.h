#ifndef JUDGE_INTERACT_H
#define JUDGE_INTERACT_H
#include "main.h"

#define Judge_Frame_Header 0xA5
#define Judge_Len_Header   5
#define Judge_Len_Cmdid    2  // 命令码长度 0xXXXX
#define Judge_Len_Tail     2  // 帧尾长度
#define Judge_Len_Packed   50 // 数据包最大长度
#define Judge_RXBUF_SIZE   100
#define Tx_Data_MaxLength  128

/*
__packed 表示紧凑字节对齐，例如 int/float/double/char 的总大小是 4+4+8+1=17。
若不用 __packed，系统会按默认对齐方式（32 位系统 4 字节对齐），则占用 4+4+8+4=20（不足 4 字节补齐）。
*/

enum{
    FAULT = 0,
    OK    = 1
};

enum{
    Red_Blue_Color = 0,
    Yellow_Color   = 1,
    Green_Color    = 2,
    Orange_Color   = 3,
    Purple_Red_Color = 4,
    Pink_Color     = 5,
    Cyan_Color     = 6,
    black_Color    = 7,
    White_Colo     = 8
};

/* 比赛状态数据 0x0001 1HZ */
typedef  struct __attribute__((packed))
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_state_t;

/* 比赛结果数据 0x0002 比赛后发送 */
typedef  struct __attribute__((packed))
{
    uint8_t winner;
} ext_game_result_t;

/* 双方血量数据 0x0003 1HZ */
typedef  struct __attribute__((packed))
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/* 人工智能挑战赛增益/惩罚区域状态 0x0005 1HZ */
typedef  struct __attribute__((packed))
{
    uint8_t F1_zone_status : 1;
    uint8_t F1_zone_buff_debuff_status : 3;
    uint8_t F2_zone_status : 1;
    uint8_t F2_zone_buff_debuff_status : 3;
    uint8_t F3_zone_status : 1;
    uint8_t F3_zone_buff_debuff_status : 3;
    uint8_t F4_zone_status : 1;
    uint8_t F4_zone_buff_debuff_status : 3;
    uint8_t F5_zone_status : 1;
    uint8_t F5_zone_buff_debuff_status : 3;
    uint8_t F6_zone_status : 1;
    uint8_t F6_zone_buff_debuff_status : 3;
    uint16_t red1_bullet_left;
    uint16_t red2_bullet_left;
    uint16_t blue1_bullet_left;
    uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;

/* 场地事件数据 0x0101 事件改变后发送 */
typedef  struct __attribute__((packed))
{
    uint32_t event_type;
} ext_event_data_t;

/* 补给站补弹状态 0x0102 事件改变后发送 */
typedef  struct __attribute__((packed))
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/* 补给站预约补弹 0x0103（RM 库暂未使用） */
typedef  struct __attribute__((packed))
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

/* 裁判系统警告信息 0x0104 己方警告后发送 */
typedef  struct __attribute__((packed))
{
    uint8_t level;
    uint8_t foul_robot_id;
    uint8_t count;
} ext_referee_warning_t;

/* 飞镖发射口倒计时 0x0105 10HZ 发送范围：己方机器人 */
typedef  struct __attribute__((packed))
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} ext_dart_remaining_time_t;

/* 比赛机器人状态 0x0201 10Hz */
typedef  struct __attribute__((packed))
{
    uint8_t robot_id;                     // 机器人 ID
    uint8_t robot_level;                  // 机器人等级
    uint16_t current_HP;                  // 当前血量
    uint16_t maximum_HP;                  // 血量上限
    uint16_t shooter_barrel_cooling_value;// 射击热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;   // 射击热量上限
    uint16_t chassis_power_limit;         // 底盘功率上限
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} ext_game_robot_status_t;

/* 实时缓冲能量和热量数据 0x0202 50Hz */
typedef struct __attribute__((packed))
{
    uint16_t chassis_volt;                   // 底盘电压 (mV)
    uint16_t chassis_current;                // 底盘电流 (mA)
    float    chassis_power;                  // 底盘功率 (W)
    uint16_t chassis_power_buffer;           // 底盘功率缓冲能量 (J)
    uint16_t shooter_id1_17mm_cooling_heat;  // 枪口1 17mm 当前热量
    uint16_t shooter_id1_42mm_cooling_heat;  // 枪口1 42mm 当前热量
} ext_power_heat_data_t;

/* 机器人位置 0x0203 10HZ */
typedef  struct __attribute__((packed))
{
    float x;
    float y;
    float angle;
} ext_game_robot_pos_t;

/* 机器人增益 0x0204 状态改变后发送 */
typedef  struct __attribute__((packed))
{
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
} ext_buff_t;

/* 无人机能量状态 0x0205 10Hz */
typedef  struct __attribute__((packed))
{
    uint8_t attack_time;
    uint8_t time_remain;
} aerial_robot_energy_t;

/* 伤害状态 0x0206 伤害发生后发送 */
typedef  struct __attribute__((packed))
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/* 实时射击信息 0x0207 射击后发送 */
typedef struct __attribute__((packed))
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

/* 弹丸剩余发射数 0x0208 10Hz 所有机器人发送 */
typedef  struct __attribute__((packed))
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

/* 机器人 RFID 状态 0x0209 1Hz */
typedef  struct __attribute__((packed))
{
    uint32_t rfid_status;
} ext_rfid_status_t;

/* 飞镖机器人客户端指令 0x020A 10Hz */
typedef  struct __attribute__((packed))
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/* 交互数据接收信息 0x0301 上限 10Hz */
typedef  struct __attribute__((packed))
{
    uint16_t data_cmd_id;
    uint16_t send_ID;
    uint16_t receiver_ID;
} ext_robot_interact_data_t;

/* 自定义控制（机器人端）0x0302 */
typedef struct
{
    uint8_t data[30];
} TuC_custom_control_t;

/* 通用通信结构体 */
typedef struct{
    int16_t first;
    int16_t second;
    int16_t third;
    int16_t fourth;
    int16_t fifth;
    int16_t height; // 注意：使用 s16，超过 65535 时需处理
    int16_t time;   // 同步
    uint8_t reset;
    float th2;
    float th3;
    float th4;
    float th5;
    float th6;
    float di;
    char key1;
    char key2;
    char key3;
    char key4;
    char key5;
    char key6;
} commucation_t;

/* 选手端地图指令 0x0303 单向 */
typedef struct
{
    float target_position_x; // 单位 m
    float target_position_y;
    uint8_t cmd_keyboard;    // 键盘命令
    uint8_t target_robot_id; // 目标机器人 ID
    uint8_t cmd_source;      // 命令源 ID
} ext_map_command_t;

/* 远程控制（鼠标/键盘同步）0x0304 */
typedef struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} TuC_remote_control_t;

/* 选手端自定义客户端数据 0x0306 */
typedef struct
{
    uint16_t key_value;
    uint16_t x_position : 12;
    uint16_t mouse_left : 4;
    uint16_t y_position : 12;
    uint16_t mouse_right : 4;
    uint16_t reserved;
} TuC_custom_client_data_t;

/* 选手端小地图/路径规划信息 0x0308 3Hz 长度 34 */
typedef struct
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint16_t user_data[30];
} ext_custom_info_t;

/* 选手端机器人位置 0x0305 10Hz 长度 10 */
typedef struct
{
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t infantry_5_position_x;
    uint16_t infantry_5_position_y;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
} ext_map_robot_data_t;

typedef struct Judge_RecData_
{
    uint8_t  sof;
    uint16_t datalength;
    uint8_t  seq;
    uint8_t  crc8;
    uint16_t cmd_id;
    uint16_t frame_tail;
    uint16_t crc16; // CRC16 校验位
    uint8_t  next_frame_header; // 下一帧数据头
} Judge_RecData_t;

// 命令码 ID，对应接收数据类型
typedef enum
{
    ID_game_state                  = 0x0001, // 比赛状态
    ID_game_result                 = 0x0002, // 比赛结果
    ID_game_robot_HP               = 0x0003, // 双方血量
    ID_dart_status                 = 0x0004, // 飞镖状态
    ID_ICRA_buff_debuff_zone_status= 0x0005, // AI 挑战赛区域状态
    ID_event_data                  = 0x0101, // 场地事件
    ID_supply_projectile_action    = 0x0102, // 补给站补弹状态
    ID_referee_warning             = 0x0104, // 裁判警告
    ID_dart_remaining_time         = 0x0105, // 飞镖倒计时
    ID_game_robot_state            = 0x0201, // 机器人状态
    ID_power_heat_data             = 0x0202, // 能量与热量
    ID_game_robot_pos              = 0x0203, // 机器人位置
    ID_buff                        = 0x0204, // 机器人增益
    ID_aerial_robot_energy         = 0x0205, // 无人机能量
    ID_robot_hurt                  = 0x0206, // 伤害状态
    ID_shoot_data                  = 0x0207, // 射击信息
    ID_bullet_remaining            = 0x0208, // 弹丸剩余
    ID_rfid_status                 = 0x0209, // RFID 状态
    ID_dart_client_cmd             = 0x020A, // 飞镖客户端指令
    ID_commu_data_cmd              = 0x0210, // 自定义通信
    ID_student_interactive_header_data = 0x0301, // 交互数据头
    /* 以下为图传/交互通道 */
    ID_custom_robot_data           = 0x0302, // 自定义控制
    ID_remote_control              = 0x0304, // 远程控制
    ID_custom_client_data          = 0x0306, // 自定义客户端
} cmd_id;

// 命令码对应数据长度
typedef enum
{
    LEN_game_state                         = 11,  // 0x0001
    LEN_game_result                        = 1,   // 0x0002
    LEN_game_robot_HP                      = 32,  // 0x0003
    LEN_dart_status                        = 3,   // 0x0004
    LEN_ICRA_buff_debuff_zone_status       = 11,  // 0x0005
    LEN_event_data                         = 4,   // 0x0101
    LEN_supply_projectile_action           = 4,   // 0x0102
    LEN_referee_warning                    = 2,   // 0x0104
    LEN_dart_remaining_time                = 1,   // 0x0105
    LEN_game_robot_state                   = 27,  // 0x0201
    LEN_power_heat_data                    = 14,  // 0x0202
    LEN_game_robot_pos                     = 16,  // 0x0203
    LEN_buff                               = 1,   // 0x0204
    LEN_aerial_robot_energy                = 2,   // 0x0205
    LEN_robot_hurt                         = 1,   // 0x0206
    LEN_shoot_data                         = 7,   // 0x0207
    LEN_bullet_remaining                   = 6,   // 0x0208
    LEN_rfid_status                        = 4,   // 0x0209
    LEN_dart_client_cmd                    = 12,  // 0x020A
    LEN_commu_data_cmd                     = 2,   // 0x0210
    LEN_student_interactive_header_data    = 119, // 0x0301
    /* 图传/交互通道 */
    LEN_custom_robot_data                  = 30,  // 0x0302
    LEN_remote_control                     = 12,  // 0x0304
    LEN_custom_client_data                 = 8,   // 0x0306
} judge_data_length;

/* 数据读取系统的按键/速度参数设置 */
typedef struct{
    float key_speed;
    float shift_speed;
    float ctrl_speed;
} set_control_judge;

/* -------- 读裁判系统数据的接口 -------- */
float RoboChassisHeat(void);      // 底盘热量读取
uint16_t RoboHP(void);            // 当前血量
uint16_t FireHeat17(void);        // 17mm 枪口热量
float FireSpeed(void);            // 弹速 m/s
uint8_t FireFeq(void);            // 射频 Hz
uint8_t RoboColor(void);          // 颜色：0 红 / 1 蓝
uint16_t FireCoolingLimit(void);  // 枪口热量上限
uint16_t FireCoolingRate(void);   // 枪口热量冷却速率
uint16_t RoboHP1(void);
uint16_t RoboHP2(void);
uint16_t RoboHP3(void);
uint16_t RoboHP4(void);
uint16_t RoboHP5(void);
uint8_t Send_State(void);
uint32_t RFID_status(void);
uint8_t Robot_level(void);
uint16_t Chassis_v(void);
uint16_t Chassis_i(void);
float Chassis_power(void);
uint16_t Chassis_power_buffer(void);
uint16_t shoot_42_heat(void);
uint16_t shoot_17_limit(void);
uint16_t send_chassis_power_limit(void);
uint16_t ID1_17_cooling_limit(void);
uint8_t RobotAimColor(void);
void Rece_Custom(void);

extern uint16_t Speed_Limit;
extern uint16_t Heat_Limit;
extern uint16_t Heat;
extern uint8_t judge_RxBuffer[100];
extern set_control_judge Key_Set;
extern ext_game_robot_status_t  Rec_game_robot_status; // 机器人状态
extern ext_power_heat_data_t    Rec_power_heat_data;   // 0202 实时能量/热量
extern ext_bullet_remaining_t   Rec_bullet_remaining;  // 0208 弹量

void SetControlParameter(void); // 根据功率上限设置控制参数
void N_SetControlParameter(void);
uint8_t Judge_Read(uint8_t* RcUsartData);
void HAL_UART4_Receive_IT_IDLE(void); // Uart4 空闲中断 + DMA 使能

#endif
