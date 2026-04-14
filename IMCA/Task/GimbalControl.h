//
// Created by zn1135 on 25-11-29.
//

#ifndef GIMBALCONTROL_H
#define GIMBALCONTROL_H
#include <stdint.h>

void GimbalInit();
void GimbalControlSetValue(void);
extern int znlook[10];
extern float yaw_pos,yaw_acc,yaw_vel,yaw_output;
extern float pitch_pos,pitch_acc,pitch_vel,pitch_output;
extern float D_speed_yaw,gyrosc_speed;
extern float vision_pitch, vision_yaw;
extern uint8_t highlow_flag ;

#endif //GIMBALCONTROL_H
