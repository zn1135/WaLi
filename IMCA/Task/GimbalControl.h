//
// Created by zn1135 on 25-11-29.
//

#ifndef GIMBALCONTROL_H
#define GIMBALCONTROL_H
#include <stdint.h>

void GimbalInit();
void GimbalControlSetValue(void);
void YawEsoRecordAppliedTorque(float torque);
extern float yaw_eso_z1;
extern float yaw_eso_z2;
extern float yaw_eso_z3;
extern float yaw_eso_comp_filt;
extern volatile float yaw_eso_tau_applied;
extern int znlook[10];
extern float yaw_pos,yaw_acc,yaw_vel,yaw_output;
extern float pitch_pos,pitch_acc,pitch_vel,pitch_output;
extern float D_speed_yaw,gyrosc_speed;
extern float vision_pitch, vision_yaw;
extern uint8_t highlow_flag ;
extern uint16_t Stuck_Hight_cnt;
extern int32_t High_Low_Start_angle;
#endif //GIMBALCONTROL_H
