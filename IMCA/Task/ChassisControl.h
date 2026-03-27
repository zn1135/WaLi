//
// Created by zn1135 on 25-10-1.
//



/*
//
//                       _oo0oo_
//                      o8888888o
//                      88" . "88
//                      (| -_- |)
//                      0\  =  /0
//                    ___/`---'\___
//                  .' \\|     |// '.
//                 / \\|||  :  |||// \
//                / _||||| -:- |||||- \
//               |   | \\\  -  /// |   |
//               | \_|  ''\---/''  |_/ |
//               \  .-\__  '-'  ___/-. /
//             ___'. .'  /--.--\  `. .'___
//          ."" '<  `.___\_<|>_/___.' >' "".
//         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
//         \  \ `_.   \_ __\ /__ _/   .-` /  /
//     =====`-.____`.___ \_____/___.-`___.-'=====
//                       `=---='
//
//
//     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//               佛祖保佑         永无BUG
//
*/






#ifndef BIG_WHEELLEG_CHASSISCONTROL_H
#define BIG_WHEELLEG_CHASSISCONTROL_H
#include <stdint.h>
#include "kalman_filter.h"

typedef struct
{

    //车体坐标系中的速度
    float car_vx; //单位：m/s
    float car_vy;
    float car_vw; //rad/s

}chassis_t;





extern float chassis_vx_channel , chassis_vy_channel , chassis_vw_channel ;
void ChassisControlSetValue(void);
void ChassisControl_Calculating(void);
float ChassisRotationPID(float measure, float target);
// 腿部状态结构体定义

#endif //BIG_WHEELLEG_CHASSISCONTROL_H
