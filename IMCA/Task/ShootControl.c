//
// Created by zn1135 on 25-11-29.
//

#include "ShootControl.h"

#include <stdlib.h>

#include "ControlTask.h"
#include "Judge_interact.h"
#include "RcTask.h"
#include "Vision_interact.h"
#include "Motor.h"
uint8_t Leg_flag=0;
int32_t Set_Shoot_Speed;
float Target_Speed;
shoot_t  shoot_fix;
uint16_t held_cnt = 0;//Cumulative count
uint16_t Stuck_Bullet_cnt = 0;
uint8_t  Stuck_Bullet_falg = 0;

#define TotalHert (ID1_17_cooling_limit())     // 总热量
#define CurrentHeat (FireHeat17())        // 当前热量
#define CoolingRate (FireCoolingRate()) // 冷却速率

/* 射速控制参数 */
// #define RICT_Speed 6720    // 摩擦轮转速(rpm)
#define MaxShootSpeed 5000 // 最大射速(rpm)
#define MaxRateRate 0.5    // 保持最大射速的比率 (0.0 - 1.0)
#define ReservedHert 30    // 预留热量

/* 机械参数 */
#define SingleHeat 10               // 单发热量
#define ReductionRatio (36.0 / 1.0) // 拨弹盘电机减速比
#define FirePreCircle 8             // 每转多少发

/* 公式简化 */
// 公式：  转速 / 减速比 / * 每转发数 / 60 * 单发热量 = 热量增加速率 = 冷却速率    化简得下式
#define MinShootSpeed ((CoolingRate) / (SingleHeat) * 60 / (FirePreCircle) * (ReductionRatio)) // 最小射速(rpm)




void SelectJudgeSpeed()     //Data selection for referees system
{
    // 根据赛季规则设定射击速度，2024赛季步兵最大速度为25m/s
    if(KEY_V)
        Set_Shoot_Speed = -350;
    else
        Set_Shoot_Speed = -310;//358
    Target_Speed = (Set_Shoot_Speed * 19.2) + shoot_fix.TEMP_30 + shoot_fix.Level_3_fix_num;



    // 根据控制模式设置摩擦轮的目标速度
    if (xRobotMode.CtrlMode == RC)
    {
        xMotorParam[FRICTR].EcoTar = 0;
        xMotorParam[FRICTL].EcoTar = 0;

    }
    else if (xRobotMode.CtrlMode == PC)
    {
        // 在PC控制模式下，根据右上开关状态决定是否启动摩擦轮
        if (xRobotMode.GimbalMode ==NOSHOOT_MODE)
        {
        	xMotorParam[FRICTR].EcoTar = 0;
    		xMotorParam[FRICTL].EcoTar = 0;
        }
        else
        {
        	if (Gimbal_Height_flag!=Low) {
        		xMotorParam[FRICTR].EcoTar = -Set_Shoot_Speed * 19.2f;
        		xMotorParam[FRICTL].EcoTar = Set_Shoot_Speed * 19.2f;
        	}else
        	{
        		xMotorParam[FRICTR].EcoTar = 0;
        		xMotorParam[FRICTL].EcoTar = 0;
        	}
        }

    }
    else if (xRobotMode.CtrlMode == LOSE)
    {
        xMotorParam[FRICTR].EcoTar = 0;
        xMotorParam[FRICTL].EcoTar = 0;
    }

}
void ShootPluck_Fire() {
if (held_cnt >10 && held_cnt <=300) {
	if (CurrentHeat + SingleHeat > TotalHert - ReservedHert) {
		// 	// 如果当前热量加上单发热量超过总热量减去预留热量，说明无法发射

	}else {
		xMotorParam[PLUCK].CloseLoopMode = PosClsloop;
		//                    xMotorParam[PLUCK].EcoTar = xMotorParam[PLUCK].MeasureDJI->total_angle + (8192/8*36);
		xMotorParam[PLUCK].EcoTar = xMotorParam[PLUCK].MeasureEco->total_angle + (8192*36/8);

	}

}else {
	if (CurrentHeat + SingleHeat > TotalHert - ReservedHert) {
		// 如果当前热量加上单发热量超过总热量减去预留热量，说明无法发射
		xMotorParam[PLUCK].CloseLoopMode = SpdClsloop;
		xMotorParam[PLUCK].EcoTar = 0;//Robot_level()
	} else if (CurrentHeat < MaxRateRate * TotalHert) {
		// 如果当前热量低于最大射速的比率乘以总热量，说明可以以最大射速发射
		xMotorParam[PLUCK].CloseLoopMode = SpdClsloop;
		xMotorParam[PLUCK].EcoTar =MaxShootSpeed;//Robot_level()
	} else {
		// 否则，根据当前热量和冷却速率计算出一个适合的射速(一次函数，保证射速随着热量的增加线性下降，直到达到最小射速)
		float shootSpeed = MaxShootSpeed - (MaxShootSpeed - MinShootSpeed) * (CurrentHeat - MaxRateRate * TotalHert) / (TotalHert - MaxRateRate * TotalHert - ReservedHert);
		xMotorParam[PLUCK].CloseLoopMode = SpdClsloop;
		xMotorParam[PLUCK].EcoTar = shootSpeed;//Robot_level()
	}
	// xMotor_DJI[PLUCK].xMotorParam.LoopMode = SpdClsloop;
	// xMotor_DJI[PLUCK].xMotorParam.Tar = MaxShootSpeed;
}
}


void PluckControl()
{
	if (xRobotMode.CtrlMode == LOSE||xRobotMode.CtrlMode == RC) {
		xMotorParam[PLUCK].CloseLoopMode = SpdClsloop;
		xMotorParam[PLUCK].EcoTar = 0;
	}else if(xRobotMode.CtrlMode == PC && RC_UPPER_RIGHT_SW_DOWN)
	{
        xMotorParam[PLUCK].CloseLoopMode = SpdClsloop;
        xMotorParam[PLUCK].EcoTar = 3000;
	}
	else if(xRobotMode.CtrlMode == PC&& !RC_UPPER_RIGHT_SW_UP) {
		//        press_cnt++;//计数器
		if (Gimbal_Height_flag!=Low) {
			if(mouse_right.deshake_key&&MiniPC_ReceivePacket.fireflag == 2)
			{
				ShootPluck_Fire();
				// xMotorParam[PLUCK].CloseLoopMode = SpdClsloop;
				// xMotorParam[PLUCK].EcoTar =MaxShootSpeed;//Robot_level()
			}
			else if (KEY_V&&MiniPC_ReceivePacket.fireflag == 2) {
				xMotorParam[PLUCK].CloseLoopMode = PosClsloop;
				//                    xMotorParam[PLUCK].EcoTar = xMotorParam[PLUCK].MeasureDJI->total_angle + (8192/8*36);
				xMotorParam[PLUCK].EcoTar = xMotorParam[PLUCK].MeasureEco->total_angle + (8192*36/8);
			}
			else if(!mouse_left.deshake_key&&mouse_right.deshake_key&&MiniPC_ReceivePacket.fireflag != 2)
			{
				xMotorParam[PLUCK].CloseLoopMode = SpdClsloop;
				xMotorParam[PLUCK].EcoTar = 0;
			}
			else
			{
				if(mouse_left.deshake_key)
				{

					held_cnt++;//按下自动计数
					if(held_cnt > 300)//按下时间
					{
						//连发
						ShootPluck_Fire();
						// xMotorParam[PLUCK].CloseLoopMode = SpdClsloop;
						// //   xMotorParam[PLUCK].EcoTar = 660.0f*0+4400;//Robot_level()
						// xMotorParam[PLUCK].EcoTar = 3000;
					}
				}
				else
				{
					if(held_cnt >10&&held_cnt <=300)//单发
					{
						ShootPluck_Fire();
						held_cnt=0;
						// xMotorParam[PLUCK].CloseLoopMode = PosClsloop;
						// //                    xMotorParam[PLUCK].EcoTar = xMotorParam[PLUCK].MeasureDJI->total_angle + (8192/8*36);
						// xMotorParam[PLUCK].EcoTar = xMotorParam[PLUCK].MeasureEco->total_angle + (8192*36/8);

					}else{

						held_cnt=0;
					}
				}
			}
			if(xMotorParam[PLUCK].CloseLoopMode == SpdClsloop && held_cnt == 0 && mouse_right.deshake_key == 0)
			{
				xMotorParam[PLUCK].CloseLoopMode = SpdClsloop;
				xMotorParam[PLUCK].EcoTar = 0;
			}
		}else
		{
			if(xMotorParam[PLUCK].CloseLoopMode == SpdClsloop)
			{
				xMotorParam[PLUCK].EcoTar = 0;
			}
			else
			{
				xMotorParam[PLUCK].EcoTar = xMotorParam[PLUCK].MeasureEco->total_angle;
			}
		}
		if(abs(xMotorParam[PLUCK].CurrentOutput) > 8000)
		{
			// Stuck_Bullet_cnt ++;
			if(Stuck_Bullet_cnt >200)
			{
				Stuck_Bullet_falg = 1;
				xMotorParam[PLUCK].CloseLoopMode = PosClsloop;
				//				xMotorParam[PLUCK].EcoTar = xMotorParam[PLUCK].MeasureDJI->total_angle - 36864;8192*36*2.5/9
				xMotorParam[PLUCK].EcoTar = xMotorParam[PLUCK].MeasureEco->total_angle - 8192*36*2.5/9/2;
				Stuck_Bullet_cnt = 0;
			}
		}
		else
		{
			Stuck_Bullet_cnt = 0;
			Stuck_Bullet_falg = 0;
		}
	}else {
		xMotorParam[PLUCK].CloseLoopMode = SpdClsloop;
		xMotorParam[PLUCK].EcoTar = 0;
	}


//		shoot_heat();
}


void ShootControlSetValue(void) {
    SelectJudgeSpeed();
    if(mouse_right.deshake_key)
    {
        MiniPC_SendPacket.mode = 1;
    }
    else MiniPC_SendPacket.mode = 0;
}

