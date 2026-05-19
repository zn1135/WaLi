//
// Created by zn1135 on 25-9-23.
//
#include "UartTask.h"
#include "main.h"
#include "imu_data_decode.h"
#include "packet.h"
#include "usart.h"
#include "RcTask.h"
#include "ControlTask.h"
#include "Judge_interact.h"


int8_t last_Hp;
int8_t last_Hp_becoming = 0;
/**********************************************************************************************************************
 * @brief  启动串口空闲中断,在初始化时调用
 * @param  UART_HandleTypeDef *huart   UART handle Structure
 *         uint8_t *pData              接收缓存区
 *         uint16_t Size               数据大小
 * @retval HAL status
**********************************************************************************************************************/
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA_NoIT(UART_HandleTypeDef *huart)
{

    /* 定义接收数据缓冲区指针和大小 */
    uint8_t *pData=0;
    uint16_t Size=0;
    HAL_StatusTypeDef statu;
//	    pData = RC_Buffer;
//	    Size = sizeof(pData);

    /* 判断是否为USART2串口 */
    if(huart == &huart2)
    {
        /* 配置USART2的接收缓冲区和大小 */
        /* 启动USART2串口空闲中断接收 */
        statu = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RC_Buffer, RC_BUFFER_SIZE);
        deshake_all();
        /* 禁用DMA半传输中断 */
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
    else if(huart == &huart5)
    {
        /* 配置USART5的接收缓冲区和大小 */
        /* 启动USART5串口空闲中断接收 */
        statu = HAL_UARTEx_ReceiveToIdle_DMA(&huart5, IMURxBuffer, sizeof(IMURxBuffer));
        /* 禁用DMA半传输中断 */
        __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
    }else if (huart == &huart3)
    {
        statu = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, judge_RxBuffer, sizeof(judge_RxBuffer));
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }else if (huart == &hlpuart1) {
        statu = HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, VT_RxBuffer, sizeof(VT_RxBuffer));
    }
    return statu;

}


/**********************************************************************************************************************
  * @Func	 void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
  * @Brief   UART错误回调
  * @Param	 UART_HandleTypeDef *huart：UART句柄
  * @Retval	 无
  * @Note    发生错误后重新使能各串口的空闲中断DMA接收
*********************************************************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART5)
    {
        __HAL_UNLOCK(huart);
        // packet_decode(IMURxBuffer[0]);
        IMU_Data_Uptade(IMURxBuffer, 76);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5,IMURxBuffer,sizeof(IMURxBuffer));		// 重新使能串口5接收中断
        __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
    }
    if(huart->Instance == USART2)
    {
//        MX_USART2_UART_Init();
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RC_Buffer, RC_BUFFER_SIZE);
        /* 禁用DMA半传输中断 */
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
    if (huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3,judge_RxBuffer,sizeof(judge_RxBuffer));		// 重新使能串口5接收中断
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
    if (huart->Instance==LPUART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, VT_RxBuffer, sizeof(VT_RxBuffer));
    }
}
/**********************************************************************************************************************
  * @Func	 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
  * @Brief   UART DMA接收完成回调
  * @Param	 UART_HandleTypeDef *huart：UART句柄
  * @Retval	 无
  * @Note    UART1 DMA接收完成后重新启动DMA接收
*********************************************************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == USART1)
    {
        HAL_UART_Receive_DMA(&huart1, aRxBuffer1, sizeof(aRxBuffer1));
    }

}
/**********************************************************************************************************************
  * @Func	 void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
  * @Brief   UART空闲中断回调
  * @Param	 UART_HandleTypeDef *huart：UART句柄
  *          uint16_t Size：接收数据长度
  * @Retval	 无
  * @Note    分发USART2(遥控器)/UART5(IMU)/USART3(裁判系统)/LPUART1(VT)的接收数据
*********************************************************************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if(huart->Instance == USART2)
    {
        RC_TimeOut_cnt++;
        Callback_RC_Handle(&remote_control, huart->pRxBuffPtr);
        deshake_all();
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RC_Buffer, RC_BUFFER_SIZE);
        /* 禁用DMA半传输中断 */
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
    if(huart->Instance == UART5)
    {
        // packet_decode(IMURxBuffer[0]);//解析数据
        IMU_Data_Uptade(IMURxBuffer, 76);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5,IMURxBuffer,sizeof(IMURxBuffer));		// 重新使能串口5接收中断

    }
    if(huart->Instance == USART3)
    {
        last_Hp=RoboHP();
        Judge_Read(judge_RxBuffer);
        last_Hp_becoming=last_Hp-RoboHP();
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3,judge_RxBuffer,sizeof(judge_RxBuffer));		// 重新使能串口3接收中断

        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
    if (huart->Instance==LPUART1)
    {
        Callback_VT_Handle(&VT_control,VT_RxBuffer);
        HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, VT_RxBuffer, sizeof(VT_RxBuffer));
    }
}