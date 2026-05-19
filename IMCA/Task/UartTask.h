//
// Created by zn1135 on 25-9-23.
//
#ifndef BIG_WHEELLEG_UARTTASK_H
#define BIG_WHEELLEG_UARTTASK_H
#include "main.h"

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA_NoIT(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
extern int8_t last_Hp_becoming ;
#endif //BIG_WHEELLEG_UARTTASK_H
