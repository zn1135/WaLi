#ifndef UI_H
#define UI_H
#if 0
//    这些结构体都是我自己定义的
#endif
#include "main.h"
#include "UI_STRUCT.h"
#include "judge_interact.h"




#define  SendID  RobotSendID()      //机器人ID
#define  ReceID  RobotReceiveID() //客户端ID
#define  huartx  huart3 //裁判系统所使用的串口

/****************************************************************************************************/

#define Judge_Frame_Header 0xA5
#define Judge_Len_Header   5
#define Judge_Len_Cmdid 2 //命令码长度 0xXXXX
#define Judge_Len_Tail  2 //帧尾长度
#define Judge_Len_Packed 50//数据包长度
#define Judge_RXBUF_SIZE 512
#define Tx_Data_MaxLength 128

typedef struct {

	uint8_t  Aim_Flag;
	float    pitch_angle;
	uint8_t	 PIC_STA[2];
	uint32_t	pointer[2];
	uint8_t     CAPNOW[3];//电容

}ui_struct;

enum{
	Motion_Pic   	  = 0,
	LOARD_Pic 		= 1,
};

enum{
	X_ZHOU    = 0,
	Y_ZHOU    = 1
};




/* 外部调用结构体*/
extern  ui_struct ui_state;

uint8_t RobotSendID(void);
uint16_t RobotReceiveID(void);


/*主任务*/

void  Ui_Struct_Init(void);//UI结构体参数初始化
void  UI_ALL_Task(void);//UI建立任务

/*新版UI函数*/
void UI_TEST(uint8_t operate_);//可用
void UI_TEST_Character(char V1,uint32_t x ,uint32_t y,uint8_t name ,uint8_t operate_ );
void UI_TEST_Character_BULLET(uint8_t operate_);
void UI_NEW_INIT(void);
void UI_MODE_BUILD(void);

void UI_Hero_Task(void);
extern void UI_INFANTRY_Task(void)	;
void UI_ENGINEER_Task(void);

void ChassisGuidance(uint16_t circle_point_x,uint16_t circle_point_y,uint8_t Radius);



extern uint16_t last_bullet;
extern uint16_t now_bullet;
extern uint16_t add_bullet;
extern uint16_t all_bullet;

#endif
