/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */


#include "stdint.h"

//#define X 0
//#define Y 1
//#define Z 2



// typedef struct
// {
//     float q[4]; // ��Ԫ������ֵ
//
//     float Gyro[3];  // ���ٶ�
//     float Accel[3]; // ���ٶ�
//     float MotionAccel_b[3]; // ����������ٶ�
//     float MotionAccel_n[3]; // ����ϵ���ٶ�
//
//     float AccelLPF; // ���ٶȵ�ͨ�˲�ϵ��
//
//     // ���ٶ��ھ���ϵ��������ʾ
//     float xn[3];
//     float yn[3];
//     float zn[3];
//
//     float atanxz;
//     float atanyz;
//
//     // λ��
//     float Roll;
//     float Pitch;
//     float Yaw;
//     float YawTotalAngle;
// 		float YawAngleLast;
// 		float YawRoundCount;
//
// 		float v_n;//����ϵ����ˮƽ�˶�������ٶ�
// 		float x_n;//����ϵ����ˮƽ�˶������λ��
//
// 		uint8_t ins_flag;
// } INS_t;


/**
 * @brief ����������װ���Ĳ���,demo�п�����
 * 
 */
// typedef struct
// {
//     uint8_t flag;

//     float scale[3];

//     float Yaw;
//     float Pitch;
//     float Roll;
// } IMU_Param_t;
//
// extern INS_t INS;
extern float Rotation_BtoE[9];
extern float Acc_Body[3];
extern float Acc_Normol[3];

// extern void INS_Init(void);
// extern void INS_task(void);

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
void Get_Rotation_BtoE(float roll,float pitch,float yaw,float X_Acc,float Y_Acc,float Z_Acc);
void My_BodyFrameToEarthFrame_Matrix(void);
void My_BodyFrameToEarthFrame_f(void);




