//
// Created by zn1135 on 25-9-17.
//

#ifndef ControlTask_H
#define ControlTask_H

#include "user_lib.h"
#include "main.h"
uint8_t GPIO_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void TotalInitTask(void);
void TotalControl(void);
void CtrlModeSelect(void);
void TotalControlSetValue(void);
void PidCalc(void);
void LiftLowerControl(void);
void ALL_reset(uint8_t reset);
extern robot_mode_t xRobotMode;
extern uint8_t Gimbal_Init_Flag;
extern uint8_t DM_again_flag;
extern uint8_t aRxBuffer1[150];
extern uint8_t Gimbal_Height_flag;

#endif //ControlTask_H
