/**
  *********************************************************************
  * @file      ins_task.c/h
  * @brief     ����������mahony������ȡ������̬��ͬʱ��ȡ�����ھ�������ϵ�µ��˶����ٶ�
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	

#include "INS_task.h"
#include "math.h"
#include "ChassisControl.h"
#include "IMU_Task.h"
#include "Ramp.h"

// INS_t INS;
//
// struct MAHONY_FILTER_t mahony;
// Axis3f Gyro,Accel;
// float gravity[3] = {0, 0, 9.81f};


// uint32_t INS_DWT_Count = 0;
// float ins_dt = 0.004f;
// float ins_time;
// int stop_time;

// void INS_Init(void)
// {
// 	 mahony_init(&mahony,1.0f,0.0f,0.001f);
//    INS.AccelLPF = 0.0089f;
// }
//
// void INS_task(void)
// {
// 		mahony.dt = 0.003f;
//
// //	GetImuAngle_new(&receive_imusol); // ��ȡ����������
//
//     INS.Accel[0] = -IMU_X.speed_acc;
//     INS.Accel[1] = -IMU_Y.speed_acc;
//     INS.Accel[2] = IMU_Z.speed_acc;
// 		Accel.x=-IMU_X.speed_acc;
// 		Accel.y=-IMU_Y.speed_acc;
// 		Accel.z=IMU_Z.speed_acc;
//     INS.Gyro[0] = -IMU_X.speed_gyro;
//     INS.Gyro[1] = -IMU_Y.speed_gyro;
//     INS.Gyro[2] = IMU_Z.speed_gyro;
//   	Gyro.x=-IMU_X.speed_gyro;
// 		Gyro.y=-IMU_Y.speed_gyro;
// 		Gyro.z=IMU_Z.speed_gyro;
//
// 	mahony_input(&mahony,Gyro,Accel);// �������Ǻͼ��ٶȼƵ�����ֵ�����˲����ṹ��
// 	mahony_update(&mahony);// ʹ��Mahony�㷨�����˲���״̬
// 	mahony_output(&mahony);// ����ת��������ȡ��̬��
// 	RotationMatrix_update(&mahony);/*������ת����*///�������ϵ R ת������������ϵ b ������ת������
//
// 		INS.q[0]=mahony.q0;
// 		INS.q[1]=mahony.q1;
// 		INS.q[2]=mahony.q2;
// 		INS.q[3]=mahony.q3;
//
//       // �������ӵ�������ϵnת��������ϵb,�����ݼ��ٶȼ����ݼ����˶����ٶ�
// 		float gravity_b[3];
//     EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
//     for (uint8_t i = 0; i < 3; i++) // ͬ����һ����ͨ�˲�
//     {
//       INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * ins_dt / (INS.AccelLPF + ins_dt)
// 														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + ins_dt);
// //	  INS.MotionAccel_b[i] = (INS.Accel[i] ) * dt / (INS.AccelLPF + dt)
// //														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
// 	}
// 		BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // ת���ص���ϵn
//
// 		//��������
// 		if(fabsf(INS.MotionAccel_n[0])<0.06f)
// 		{
// 		  INS.MotionAccel_n[0]=0.0f;	//x��
// 		}
// 		if(fabsf(INS.MotionAccel_n[1])<0.06f)
// 		{
// 		  INS.MotionAccel_n[1]=0.0f;	//y��
// 		}
// 		if(fabsf(INS.MotionAccel_n[2])<0.06f)
// 		{
// 		  INS.MotionAccel_n[2]=0.0f;//z��
// 		}
//
// 		// if(ins_time>3000.0f)
// 		// {
// 		// 	INS.ins_flag=1;//��Ԫ���������������ٶ�Ҳ�������������Կ�ʼ��������
// 		// 	// ��ȡ��������
// 		// 	INS.Pitch=mahony.roll;
// 		// 	INS.Roll=mahony.pitch;
// 		// 	INS.Yaw=mahony.yaw;
//
// 		// //INS.YawTotalAngle=INS.YawTotalAngle+INS.Gyro[2]*0.001f;
//
// 		// 	if (INS.Yaw - INS.YawAngleLast > 3.1415926f)
// 		// 	{
// 		// 			INS.YawRoundCount--;
// 		// 	}
// 		// 	else if (INS.Yaw - INS.YawAngleLast < -3.1415926f)
// 		// 	{
// 		// 			INS.YawRoundCount++;
// 		// 	}
// 		// 	INS.YawTotalAngle = 6.283f* INS.YawRoundCount + INS.Yaw;
// 		// 	INS.YawAngleLast = INS.Yaw;
// 		// }
// 		// else
// 		// {
// 		//  ins_time++;
// 		// }
//
// }

float Rotation_BtoE[9];
float Acc_Body[3];
float Acc_Normol[3];
one_kalman_filter_init_t kalman_X_acc;
/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief          计算从机体坐标系到地球坐标系的旋转矩阵，并保存加速度计读数
 * @param[in]      roll: 横滚角（绕X轴旋转角度）
 * @param[in]      pitch: 俯仰角（绕Y轴旋转角度）
 * @param[in]      yaw: 偏航角（绕Z轴旋转角度）
 * @param[in]      X_Acc: X轴加速度计读数
 * @param[in]      Y_Acc: Y轴加速度计读数
 * @param[in]      Z_Acc: Z轴加速度计读数
 * @retval         无
 * @note           旋转矩阵按照ZYX欧拉角顺序计算，使用ARM DSP库的快速三角函数
 */
void Get_Rotation_BtoE(float roll,float pitch,float yaw,float P_Acc,float R_Acc,float Y_Acc)
{
	/* 根据欧拉角计算旋转矩阵元素 */
	Rotation_BtoE[0] = arm_cos_f32(yaw)*arm_cos_f32(pitch);
	Rotation_BtoE[1] = arm_cos_f32(yaw)*arm_sin_f32(pitch)*arm_sin_f32(roll)-arm_sin_f32(yaw)*arm_cos_f32(roll);
	Rotation_BtoE[2] = arm_cos_f32(yaw)*arm_sin_f32(pitch)*arm_cos_f32(roll)+arm_sin_f32(yaw)*arm_sin_f32(roll);
	Rotation_BtoE[3] = arm_sin_f32(yaw)*arm_cos_f32(pitch);
	Rotation_BtoE[4] = arm_sin_f32(yaw)*arm_sin_f32(pitch)*arm_sin_f32(roll)+arm_cos_f32(yaw)*arm_cos_f32(roll);
	Rotation_BtoE[5] = arm_sin_f32(yaw)*arm_sin_f32(pitch)*arm_cos_f32(roll)-arm_cos_f32(yaw)*arm_sin_f32(roll);
	Rotation_BtoE[6] = -arm_sin_f32(pitch);
	Rotation_BtoE[7] = arm_cos_f32(pitch)*arm_sin_f32(roll);
	Rotation_BtoE[8] = arm_cos_f32(pitch)*arm_cos_f32(roll);
	
	/* 保存机体坐标系下的加速度计读数 */
	Acc_Body[0] = P_Acc;//x
	Acc_Body[1] = R_Acc;//Y
	Acc_Body[2] = Y_Acc;//z
}

/**********************************************************************************************************************
  * @Func	 void My_BodyFrameToEarthFrame_Matrix(void)
  * @Brief   使用旋转矩阵将机体坐标系加速度转换到地球坐标系
  * @Param	 无
  * @Retval	 无
  * @Note    使用Get_Rotation_BtoE计算的旋转矩阵Rotation_BtoE进行坐标变换
*********************************************************************************************************************/
void My_BodyFrameToEarthFrame_Matrix(void)
{
	Acc_Normol[0] = Rotation_BtoE[0]*Acc_Body[0]+Rotation_BtoE[1]*Acc_Body[1]+Rotation_BtoE[2]*Acc_Body[2];
	Acc_Normol[1] = Rotation_BtoE[3]*Acc_Body[0]+Rotation_BtoE[4]*Acc_Body[1]+Rotation_BtoE[5]*Acc_Body[2];
	Acc_Normol[2] = Rotation_BtoE[6]*Acc_Body[0]+Rotation_BtoE[7]*Acc_Body[1]+Rotation_BtoE[8]*Acc_Body[2];
}

