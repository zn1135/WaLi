//
// Created by zn1135 on 25-9-24.
//



#ifndef BIG_WHEELLEG_CANBUS_TASK_H
#define BIG_WHEELLEG_CANBUS_TASK_H
#include "main.h"


void set_DM_J4310_MIT_info(FDCAN_HandleTypeDef *hcan,

                            uint16_t id, float _pos, float _vel,
                            float _KP, float _KD, float _torq);
void CANFilterInit(void);
void set_DM_J4310_motor_State(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t function);
void SetCanFronMotorValue(FDCAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void set_link_motor_State(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t function);
void set_link_motor_iq(FDCAN_HandleTypeDef *hcan, uint16_t id, int16_t iq);
void MotorCanSendCurrent(void);
void send_can2can(FDCAN_HandleTypeDef *hcan,uint16_t id,uint8_t iq1, uint8_t iq2, uint8_t iq3, uint8_t iq4, uint8_t iq5, uint8_t iq6, uint8_t iq7, uint8_t iq8);


#endif //BIG_WHEELLEG_CANBUS_TASK_H
