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

VT_Type VT_control;
uint8_t VT_RxBuffer[100] = {0};
// ������İ���
// ������İ���
Key_deshake_t mouse_left;  // ��������
Key_deshake_t mouse_right; // ��������
Key_deshake_t W;
Key_deshake_t S;
Key_deshake_t A;
Key_deshake_t D;
Key_deshake_t SHIFT;
Key_deshake_t CTRL;
Key_deshake_t Q;
Key_deshake_t E;
Key_deshake_t R;
Key_deshake_t F;
Key_deshake_t G;
Key_deshake_t Z;
Key_deshake_t X;
Key_deshake_t C;
Key_deshake_t V;
Key_deshake_t B;
uint8_t RC_TimeOut_Flag = 0;
uint8_t RC_TimeOut_cnt = 0;

/**********************************************************************************************************************
 * @Func	 static uint32_t VT_GetBits(const uint8_t *buff, uint16_t start_bit, uint8_t bit_len)
 * @Brief   从字节缓冲区按位提取数据
 * @Param	 const uint8_t *buff：字节缓冲区
 *          uint16_t start_bit：起始位偏移
 *          uint8_t bit_len：提取的位长度
 * @Retval	 uint32_t：提取的数值
 *********************************************************************************************************************/
static uint32_t VT_GetBits(const uint8_t *buff, uint16_t start_bit, uint8_t bit_len)
{
    uint32_t value = 0;

    for (uint8_t i = 0; i < bit_len; i++)
    {
        uint16_t bit_pos = start_bit + i;
        value |= ((uint32_t)((buff[bit_pos / 8] >> (bit_pos % 8)) & 0x01U)) << i;
    }

    return value;
}

/**********************************************************************************************************************
 * @Func	 void Callback_VT_Handle(VT_Type* vt, uint8_t* buff)
 * @Brief   VT图传遥控器协议数据解析
 * @Param	 VT_Type* vt：VT遥控器数据结构体指针
 *          uint8_t* buff：接收数据缓冲区
 * @Retval	 无
 * @Note    解析帧头0xA9 0x53，提取摇杆、鼠标和键盘按键数据
 *********************************************************************************************************************/
void Callback_VT_Handle(VT_Type *vt, uint8_t *buff)
{
    if (vt == 0 || buff == 0)
    {
        return;
    }

    // 帧头校验：0xA9 0x53
    if (buff[0] != 0xA9 || buff[1] != 0x53)
    {
        return;
    }

    // 解析遥控器通道数据 (ch1-ch4)
    vt->ch1 = (int16_t)((int32_t)VT_GetBits(buff, 16, 11) - 1024);
    vt->ch2 = (int16_t)((int32_t)VT_GetBits(buff, 27, 11) - 1024);
    vt->ch3 = (int16_t)((int32_t)VT_GetBits(buff, 38, 11) - 1024);
    vt->ch4 = (int16_t)((int32_t)VT_GetBits(buff, 49, 11) - 1024);

    vt->_switch = (uint8_t)VT_GetBits(buff, 60, 2);
    vt->PauseButton = (uint8_t)VT_GetBits(buff, 62, 1);
    vt->Custom_leftKey = (uint8_t)VT_GetBits(buff, 63, 1);
    vt->Custom_rightKey = (uint8_t)VT_GetBits(buff, 64, 1);
    vt->ch5 = (int16_t)((int32_t)VT_GetBits(buff, 65, 11) - 1024);
    vt->Trigger_key = (uint8_t)VT_GetBits(buff, 76, 1);

    vt->mouse.x = (int16_t)VT_GetBits(buff, 80, 16);
    vt->mouse.y = (int16_t)VT_GetBits(buff, 96, 16);
    vt->mouse.z = (int16_t)VT_GetBits(buff, 112, 16);

    vt->mouse.press_left = (uint8_t)(VT_GetBits(buff, 128, 2) & 0x01U);
    vt->mouse.press_right = (uint8_t)(VT_GetBits(buff, 130, 2) & 0x01U);
    vt->mouse.press_middle = (uint8_t)(VT_GetBits(buff, 132, 2) & 0x01U);

    // 解析键盘按键码
    vt->keyBoard.key_code = (uint16_t)VT_GetBits(buff, 136, 16);
}

/**********************************************************************************************************************
 * @Func	 void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
 * @Brief   DR16遥控器协议数据解析
 * @Param	 RC_Type* rc： 存储遥控器数据的结构体
 *          uint8_t* buff：用于解码的缓冲区
 * @Retval	 None
 *********************************************************************************************************************/
void Callback_RC_Handle(RC_Type *rc, uint8_t *buff)
{
    // 解析遥控器通道数据 (ch1-ch4)
    rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3 -= 1024;
    rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc->ch4 -= 1024;

    // 解析遥控器开关数据
    rc->switch_left = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->switch_right = (buff[5] >> 4) & 0x0003;

    // 解析鼠标坐标数据
    rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
    rc->mouse.y = buff[8] | (buff[9] << 8);
    rc->mouse.z = buff[10] | (buff[11] << 8);

    // 解析鼠标按键状态
    rc->mouse.press_left = buff[12]; // is pressed?
    rc->mouse.press_right = buff[13];

    // 解析键盘按键码和通道5数据
    rc->keyBoard.key_code = buff[14] | buff[15] << 8; // key borad code
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
    if ((input > dealine) || (input < -dealine))
    {
        output = input;
    }
    else
    {
        output = 0;
    }
    return output;
}

/**********************************************************************************************************************
 * @Func	 void key_deshake(int16_t key_input, Key_deshake_t *data)
 * @Brief   按键消抖处理
 * @Param	 int16_t key_input：按键原始输入值
 *          Key_deshake_t *data：按键消抖数据结构体指针
 * @Retval	 无
 * @Note    连续8次采样一致才更新消抖状态
 *********************************************************************************************************************/
void key_deshake(int16_t key_input, Key_deshake_t *data)
{
    if (key_input) // ��������
    {
        data->key_val_one = (data->key_val_one | (0x01 << (data->key_cnt_one)));
        data->key_cnt_one++;
        if (data->key_cnt_one >= 8)
        {
            data->key_cnt_one = 0;
        }
        if ((data->key_val_one & 0xFF) == 0xFF)
        {
            data->deshake_key = 1;
            data->key_val_one = 0;
        }
    }
    else
    {
        data->key_val_zero = (data->key_val_zero & (0x00 << (data->key_cnt_zero)));
        data->key_cnt_zero++;
        if (data->key_cnt_zero >= 8)
        {
            data->key_cnt_zero = 0;
        }
        if ((data->key_val_zero & 0xFF) == 0x00)
        {
            data->deshake_key = 0;
            data->key_val_zero = 0;
        }
    }
}

/**********************************************************************************************************************
 * @Func	 void deshake_all(void)
 * @Brief   全局按键消抖更新
 * @Param	 无
 * @Retval	 无
 * @Note    对所有鼠标按键和键盘按键(WSAD/SHIFT/CTRL/QERFGZXCVB)统一调用消抖函数
 *********************************************************************************************************************/
void deshake_all(void)
{
    key_deshake(MOUSE_PRESS_LEFT, &mouse_left);
    key_deshake(MOUSE_PRESS_RIGHT, &mouse_right);
    key_deshake(((uint16_t)1 << 0) & remote_control.keyBoard.key_code, &W);
    key_deshake(((uint16_t)1 << 1) & remote_control.keyBoard.key_code, &S);
    key_deshake(((uint16_t)1 << 2) & remote_control.keyBoard.key_code, &A);
    key_deshake(((uint16_t)1 << 3) & remote_control.keyBoard.key_code, &D);
    key_deshake(((uint16_t)1 << 4) & remote_control.keyBoard.key_code, &SHIFT);
    key_deshake(((uint16_t)1 << 5) & remote_control.keyBoard.key_code, &CTRL);
    key_deshake(((uint16_t)1 << 6) & remote_control.keyBoard.key_code, &Q);
    key_deshake(((uint16_t)1 << 7) & remote_control.keyBoard.key_code, &E);
    key_deshake(((uint16_t)1 << 8) & remote_control.keyBoard.key_code, &R);
    key_deshake(((uint16_t)1 << 9) & remote_control.keyBoard.key_code, &F);
    key_deshake(((uint16_t)1 << 10) & remote_control.keyBoard.key_code, &G);
    key_deshake(((uint16_t)1 << 11) & remote_control.keyBoard.key_code, &Z);
    key_deshake(((uint16_t)1 << 12) & remote_control.keyBoard.key_code, &X);
    key_deshake(((uint16_t)1 << 13) & remote_control.keyBoard.key_code, &C);
    key_deshake(((uint16_t)1 << 14) & remote_control.keyBoard.key_code, &V);
    key_deshake(((uint16_t)1 << 15) & remote_control.keyBoard.key_code, &B);
}
