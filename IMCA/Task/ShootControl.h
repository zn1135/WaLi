//
// Created by zn1135 on 25-11-29.
//

#ifndef SHOOTCONTROL_H
#define SHOOTCONTROL_H
#include <stdint.h>


typedef struct
{
    float TEMP_15;
    float TEMP_18;
    float TEMP_30;
    float Level_1_fix_num;
    float Level_2_fix_num;
    float Level_3_fix_num;

}shoot_t;
extern uint16_t held_cnt;//Cumulative count
extern uint8_t single_shot;
extern uint8_t  Stuck_Bullet_falg;
void ShootControlSetValue(void);
void PluckControl();
#endif //SHOOTCONTROL_H
