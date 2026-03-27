#ifndef __RC_TASK
#define __RC_TASK

#include "stm32g4xx_hal.h"

#define RC_CONTROL_ENABLE   /*ң��������ʹ�ܺ궨��*/

#define RC_BUFFER_SIZE 150
#define RC_Frame_Lentgh		18

/*��ֵ*/
#define KEY_W 				((uint16_t)1 << 0)&remote_control.keyBoard.key_code
#define KEY_S 				((uint16_t)1 << 1)&remote_control.keyBoard.key_code 
#define KEY_A 				((uint16_t)1 << 2)&remote_control.keyBoard.key_code
#define KEY_D 				((uint16_t)1 << 3)&remote_control.keyBoard.key_code
#define KEY_SHIFT 			((uint16_t)1 << 4)&remote_control.keyBoard.key_code
#define KEY_CTRL 			((uint16_t)1 << 5)&remote_control.keyBoard.key_code
#define KEY_Q 				((uint16_t)1 << 6)&remote_control.keyBoard.key_code
#define KEY_E 				((uint16_t)1 << 7)&remote_control.keyBoard.key_code
#define KEY_R 				((uint16_t)1 << 8)&remote_control.keyBoard.key_code
#define KEY_F 				((uint16_t)1 << 9)&remote_control.keyBoard.key_code
#define KEY_G 				((uint16_t)1 << 10)&remote_control.keyBoard.key_code
#define KEY_Z 				((uint16_t)1 << 11)&remote_control.keyBoard.key_code
#define KEY_X 				((uint16_t)1 << 12)&remote_control.keyBoard.key_code
#define KEY_C 				((uint16_t)1 << 13)&remote_control.keyBoard.key_code
#define KEY_V 				((uint16_t)1 << 14)&remote_control.keyBoard.key_code
#define KEY_B 				((uint16_t)1 << 15)&remote_control.keyBoard.key_code

/*ң�������Ϸ������Ϸ�����������״̬*/
#define  RC_UPPER_LEFT_SW_UP     (remote_control.switch_left == Switch_Up)
#define  RC_UPPER_LEFT_SW_MID    (remote_control.switch_left == Switch_Middle)     
#define  RC_UPPER_LEFT_SW_DOWN   (remote_control.switch_left == Switch_Down)
#define  RC_UPPER_RIGHT_SW_UP    (remote_control.switch_right == Switch_Up)
#define  RC_UPPER_RIGHT_SW_MID   (remote_control.switch_right == Switch_Middle)
#define  RC_UPPER_RIGHT_SW_DOWN  (remote_control.switch_right == Switch_Down)


typedef struct {
	int16_t ch1;	//each ch value from -364 -- +364
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
  int16_t ch5;
	
	uint8_t switch_left;	//3 value
	uint8_t switch_right;
	
	struct {
		int16_t x;
		int16_t y;
		int16_t z;
	
		uint8_t press_left;
		uint8_t press_right;
	}mouse;
	
	struct {
		uint16_t key_code;
/**********************************************************************************
   * ����ͨ��:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
   *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
************************************************************************************/

	}keyBoard;
	

}RC_Type;


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



typedef struct
{
	uint8_t deshake_key;   // 当前按键稳定状态（去抖后的最终状态）
	uint8_t key_cnt_one;   // 连续检测到高电平(按下)的计数
	uint8_t key_cnt_zero;  // 连续检测到低电平(松开)的计数
	uint8_t key_val_one;   // 按下状态确认阈值
	uint8_t key_val_zero;  // 松开状态确认阈值
}Key_deshake_t;

// 按键去抖状态量


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

