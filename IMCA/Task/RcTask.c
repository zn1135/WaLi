/**********************************************************************************************************************
 * @file   RC_Task.c
 * @brief  DR16ң�ؽ������ݵĽ������ô���2��DMA����ң���������������ݣ�ʹ��ʱ�����ļ���ͷ�ļ����
 *         ��stm32f4xx_it.c�У���USART2_IRQHandler()���������HAL_UART_IDLE_IRQHandler(&huart2)
 *         �����жϿ��к���������USART2_IRQHandler()������ԭ����HAL_UART_IRQHandler(&huart2)ע�͵�
**********************************************************************************************************************/
#include "RcTask.h"

RC_Type remote_control;
RC_Type mouse_control;
uint8_t RC_Buffer[RC_BUFFER_SIZE];
//������İ���
//������İ���
Key_deshake_t mouse_left;//��������
Key_deshake_t mouse_right;//��������
Key_deshake_t	W;
Key_deshake_t	S;
Key_deshake_t	A;
Key_deshake_t	D;
Key_deshake_t	SHIFT;
Key_deshake_t	CTRL;
Key_deshake_t	Q;
Key_deshake_t	E;
Key_deshake_t	R;
Key_deshake_t	F;
Key_deshake_t	G;
Key_deshake_t	Z;
Key_deshake_t	X;
Key_deshake_t	C;
Key_deshake_t	V;	
Key_deshake_t	B;

/**********************************************************************************************************************
  * @Func	 void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
  * @Brief   DR16遥控器协议数据解析
  * @Param	 RC_Type* rc： 存储遥控器数据的结构体
  *          uint8_t* buff：用于解码的缓冲区
  * @Retval	 None  
 *********************************************************************************************************************/
void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
{
	// 解析遥控器通道数据 (ch1-ch4)
	rc->ch1 = (buff[0] | buff[1]<<8) & 0x07FF;
	rc->ch1 -= 1024;
	rc->ch2 = (buff[1]>>3 | buff[2]<<5 ) & 0x07FF;
	rc->ch2 -= 1024;
	rc->ch3 = (buff[2]>>6 | buff[3]<<2 | buff[4]<<10) & 0x07FF;
	rc->ch3 -= 1024;
	rc->ch4 = (buff[4]>>1 | buff[5]<<7) & 0x07FF;		
	rc->ch4 -= 1024;
	
	// 解析遥控器开关数据
	rc->switch_left = ( (buff[5] >> 4)& 0x000C ) >> 2;
	rc->switch_right =  (buff[5] >> 4)& 0x0003 ;
	
	// 解析鼠标坐标数据
	rc->mouse.x = buff[6] | (buff[7] << 8);	//x axis
	rc->mouse.y = buff[8] | (buff[9] << 8);
	rc->mouse.z = buff[10]| (buff[11] << 8);
	
	// 解析鼠标按键状态
	rc->mouse.press_left 	= buff[12];	// is pressed?
	rc->mouse.press_right = buff[13];
	
	// 解析键盘按键码和通道5数据
	rc->keyBoard.key_code = buff[14] | buff[15] << 8; //key borad code
	rc->ch5 = ((int16_t)buff[16] | ((int16_t)buff[17] << 8)) & 0x07FF;
	rc->ch5 -= 1024;
}

/**********************************************************************************************************************
  * @Func	 int16_t RcDeadlineLimit(int16_t input, uint8_t dealine)
  * @Brief   遥控器死区限制函数，过滤掉遥控器的小幅抖动
  * @Param	 int16_t input： 输入的遥控器通道值
  *          uint8_t dealine：死区阈值，当输入值的绝对值小于此值时被过滤
  * @Retval	 int16_t：处理后的输出值，超出死区范围返回原值，否则返回0
 *********************************************************************************************************************/
int16_t RcDeadlineLimit(int16_t input, uint8_t dealine)
{
    int16_t output;
    // 判断输入值是否超出死区范围
    if((input > dealine)||(input < -dealine))
    {
        output = input;
    }
    else
    {
        output = 0;
    }
    return output;
}

void key_deshake(int16_t key_input, Key_deshake_t *data)
{
	if(key_input)//��������
	{
		data->key_val_one = (data->key_val_one|(0x01 <<(data->key_cnt_one)));
		data->key_cnt_one++;	
		if(data->key_cnt_one>=8)
		{
			data->key_cnt_one=0;
		}
		if((data->key_val_one&0xFF)==0xFF)
		{
			data->deshake_key =1;
			data->key_val_one = 0;
		}
	}
	else
	{
		data->key_val_zero = (data->key_val_zero&(0x00 <<(data->key_cnt_zero)));
		data->key_cnt_zero++;	
		if(data->key_cnt_zero>=8)
		{
			data->key_cnt_zero=0;
		}
		if((data->key_val_zero&0xFF)==0x00)
		{
			data->deshake_key =0;
			data->key_val_zero = 0;
		}
	}
}

void deshake_all(void)
{
	key_deshake(remote_control.mouse.press_left, &mouse_left);
	key_deshake(remote_control.mouse.press_right, &mouse_right);
	key_deshake(((uint16_t)1 << 0)&remote_control.keyBoard.key_code, &W);
	key_deshake(((uint16_t)1 << 1)&remote_control.keyBoard.key_code, &S);
	key_deshake(((uint16_t)1 << 2)&remote_control.keyBoard.key_code, &A);
	key_deshake(((uint16_t)1 << 3)&remote_control.keyBoard.key_code, &D);
	key_deshake(((uint16_t)1 << 4)&remote_control.keyBoard.key_code, &SHIFT);
	key_deshake(((uint16_t)1 << 5)&remote_control.keyBoard.key_code, &CTRL);
	key_deshake(((uint16_t)1 << 6)&remote_control.keyBoard.key_code, &Q);
	key_deshake(((uint16_t)1 << 7)&remote_control.keyBoard.key_code, &E);
	key_deshake(((uint16_t)1 << 8)&remote_control.keyBoard.key_code, &R);
	key_deshake(((uint16_t)1 << 9)&remote_control.keyBoard.key_code, &F);
	key_deshake(((uint16_t)1 << 10)&remote_control.keyBoard.key_code, &G);
	key_deshake(((uint16_t)1 << 11)&remote_control.keyBoard.key_code, &Z);
	key_deshake(((uint16_t)1 << 12)&remote_control.keyBoard.key_code, &X);
	key_deshake(((uint16_t)1 << 13)&remote_control.keyBoard.key_code, &C);
	key_deshake(((uint16_t)1 << 14)&remote_control.keyBoard.key_code, &V);
	key_deshake(((uint16_t)1 << 15)&remote_control.keyBoard.key_code, &B);

}
