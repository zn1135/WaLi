#ifndef __RC_TASK
#define __RC_TASK

#include "stm32g4xx_hal.h"
extern uint8_t RC_TimeOut_Flag;
extern uint8_t RC_TimeOut_cnt;
#define RC_CONTROL_ENABLE   /*ң��������ʹ�ܺ궨��*/

#define RC_BUFFER_SIZE 150
#define RC_Frame_Lentgh		18

/*��ֵ*/
#define RC_KEY_CODE             ((RC_TimeOut_Flag == 0U) ? (remote_control.keyBoard.key_code) : (VT_control.keyBoard.key_code))
#define KEY_W                   ((((uint16_t)1U << 0) & (RC_KEY_CODE)))
#define KEY_S                   ((((uint16_t)1U << 1) & (RC_KEY_CODE)))
#define KEY_A                   ((((uint16_t)1U << 2) & (RC_KEY_CODE)))
#define KEY_D                   ((((uint16_t)1U << 3) & (RC_KEY_CODE)))
#define KEY_SHIFT               ((((uint16_t)1U << 4) & (RC_KEY_CODE)))
#define KEY_CTRL                ((((uint16_t)1U << 5) & (RC_KEY_CODE)))
#define KEY_Q                   ((((uint16_t)1U << 6) & (RC_KEY_CODE)))
#define KEY_E                   ((((uint16_t)1U << 7) & (RC_KEY_CODE)))
#define KEY_R                   ((((uint16_t)1U << 8) & (RC_KEY_CODE)))
#define KEY_F                   ((((uint16_t)1U << 9) & (RC_KEY_CODE)))
#define KEY_G                   ((((uint16_t)1U << 10) & (RC_KEY_CODE)))
#define KEY_Z                   ((((uint16_t)1U << 11) & (RC_KEY_CODE)))
#define KEY_X                   ((((uint16_t)1U << 12) & (RC_KEY_CODE)))
#define KEY_C                   ((((uint16_t)1U << 13) & (RC_KEY_CODE)))
#define KEY_V                   ((((uint16_t)1U << 14) & (RC_KEY_CODE)))
#define KEY_B                   ((((uint16_t)1U << 15) & (RC_KEY_CODE)))

#define CH1                    ((RC_TimeOut_Flag == 0U) ? (remote_control.ch1) : (VT_control.ch1))
#define CH2                    ((RC_TimeOut_Flag == 0U) ? (remote_control.ch2) : (VT_control.ch2))
#define CH3                    ((RC_TimeOut_Flag == 0U) ? (remote_control.ch3) : (VT_control.ch4))
#define CH4                    ((RC_TimeOut_Flag == 0U) ? (remote_control.ch4) : (VT_control.ch3))
#define CH5                    ((RC_TimeOut_Flag == 0U) ? (remote_control.ch5) : (VT_control.ch5))

#define MOUSE_X                ((RC_TimeOut_Flag == 0U) ? (remote_control.mouse.x) : (VT_control.mouse.x))
#define MOUSE_Y                ((RC_TimeOut_Flag == 0U) ? (remote_control.mouse.y) : (VT_control.mouse.y))
#define MOUSE_Z                ((RC_TimeOut_Flag == 0U) ? (remote_control.mouse.z) : (VT_control.mouse.z))
#define MOUSE_PRESS_LEFT       ((RC_TimeOut_Flag == 0U) ? (remote_control.mouse.press_left) : (VT_control.mouse.press_left))
#define MOUSE_PRESS_RIGHT      ((RC_TimeOut_Flag == 0U) ? (remote_control.mouse.press_right) : (VT_control.mouse.press_right))

/*ң�������Ϸ������Ϸ�����������״̬*/

#define  RC_UPPER_LEFT_SW_UP    ((RC_TimeOut_Flag == 0U) ? (remote_control.switch_left == Switch_Up) : (VT_control._switch == VT_S))
#define  RC_UPPER_LEFT_SW_MID   ((RC_TimeOut_Flag == 0U) ? (remote_control.switch_left == Switch_Middle) : (VT_control._switch == VT_N))
#define  RC_UPPER_LEFT_SW_DOWN  ((RC_TimeOut_Flag == 0U) ? (remote_control.switch_left == Switch_Down) : (VT_control._switch == VT_C))
#define  RC_UPPER_RIGHT_SW_UP   ((RC_TimeOut_Flag == 0U) ? (remote_control.switch_right == Switch_Up) : (VT_control.Trigger_key == 1U))
#define  RC_UPPER_RIGHT_SW_MID  ((RC_TimeOut_Flag == 0U) ? (remote_control.switch_right == Switch_Middle) : (VT_control.Custom_rightKey == 1U))
#define  RC_UPPER_RIGHT_SW_DOWN ((RC_TimeOut_Flag == 0U) ? (remote_control.switch_right == Switch_Down) : (VT_control.Custom_leftKey == 1U))



typedef struct {
	int16_t ch1;	// each ch value from -364 -- +364
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int16_t ch5;

	uint8_t switch_left;	// 3 value
	uint8_t switch_right;

	struct {
		int16_t x;
		int16_t y;
		int16_t z;

		uint8_t press_left;
		uint8_t press_right;
	} mouse;

	struct {
		uint16_t key_code;
	} keyBoard;
} RC_Type;


typedef struct {
	int16_t ch1;	// each ch value from -364 -- +364
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int16_t ch5;

	uint8_t _switch;	// 3 value
	uint8_t PauseButton;
	uint8_t Custom_leftKey;
	uint8_t Custom_rightKey;
	uint8_t Trigger_key;

	struct {
		int16_t x;
		int16_t y;
		int16_t z;

		uint8_t press_left;
		uint8_t press_right;
		uint8_t press_middle;
	} mouse;

	struct {
		uint16_t key_code;
	} keyBoard;
} VT_Type;

enum{
	Switch_Up = 1,
	Switch_Middle = 3,
	Switch_Down = 2,
};

enum{
	Key_W,
	Key_S,
	//...
};

extern RC_Type mouse_control;
extern RC_Type remote_control;
extern uint8_t RC_Buffer[RC_BUFFER_SIZE];


extern VT_Type VT_control;
extern uint8_t VT_RxBuffer[100];


typedef struct
{
	uint8_t deshake_key;   // 当前按键稳定状态（去抖后的最终状态）
	uint8_t key_cnt_one;   // 连续检测到高电平(按下)的计数
	uint8_t key_cnt_zero;  // 连续检测到低电平(松开)的计数
	uint8_t key_val_one;   // 按下状态确认阈值
	uint8_t key_val_zero;  // 松开状态确认阈值
}Key_deshake_t;

// 按键去抖状态量

void Callback_VT_Handle(VT_Type* vt, uint8_t* buff);

/*遥控接收数据解析*/
void Callback_RC_Handle(RC_Type* rc, uint8_t* buff);

/*遥控通道死区处理*/
int16_t RcDeadlineLimit(int16_t input, uint8_t dealine);

// 鼠标左右键去抖
extern Key_deshake_t mouse_left;
extern Key_deshake_t mouse_right;

// 键盘按键去抖（变量名与按键名一一对应）
extern Key_deshake_t	W;
extern Key_deshake_t	S;
extern Key_deshake_t	A;
extern Key_deshake_t	D;
extern Key_deshake_t	SHIFT;
extern Key_deshake_t	CTRL;
extern Key_deshake_t	Q;
extern Key_deshake_t	E;
extern Key_deshake_t	R;
extern Key_deshake_t	F;
extern Key_deshake_t	G;
extern Key_deshake_t	Z;
extern Key_deshake_t	X;
extern Key_deshake_t	C;
extern Key_deshake_t	V;	
extern Key_deshake_t	B;

// 统一更新全部按键去抖状态
void deshake_all(void);
#endif

