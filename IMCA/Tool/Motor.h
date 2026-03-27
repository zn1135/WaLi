//
// Created by zn1135 on 25-9-24.
//

#ifndef BIG_WHEELLEG_MOTOR_H
#define BIG_WHEELLEG_MOTOR_H

#include <stdbool.h>
#include "main.h"
#include "pid.h"



#define MOTOR_NUM 10

#define DJI_MOTOR_LIBRART	 1		//use dji motor


#define DM_MOTOR_LIBRART 	 1		//use dm motor
#define LINK_MOTOR_LIBRART 	 1     //use link motor


enum{
    ECO = 0,
    IMU
};

enum{
    POS = 0,
    SPD

};

enum
{
    enable = 1,
    disable = 2,
    save_zero_point = 3,
    clear_error = 4,
};



typedef enum{
    DJI = 0,//DJI电机
    DM,//DM电机
    LINK
}motor_type_t;

typedef enum{
    Can1 = 0,
    Can2,
    Can3

}canbus_t;

// 定义电机反馈模式枚举类型
// EcoFeedback: ECO模式反馈
// ImuFeedback: IMU模式反馈
typedef enum{
    EcoFeedback = 0,    // ECO反馈模式
    ImuFeedback         // IMU反馈模式

}feedback_mode_t;

typedef enum{
    PosClsloop = 0,//位置环控制
    SpdClsloop//速度环控制

}closeloop_mode_t;

/**
 * @brief 电机IMU测量数据结构体
 *
 * 该结构体用于存储电机IMU传感器的测量数据，包括角度、角速度等信息
 */
__packed typedef struct moto_imu_measure_t{
    float  angle;               ///< 当前角度值
    float  total_angle;         ///< 累计总角度值
    float  palstance;           ///< 角速度（弧度/秒）
    float  offset_angle;        ///< 偏移角度
    float  car_angle;           ///< 车辆角度
    float  car_total_angle;     ///< 车辆累计总角度
    unsigned char    reverse_derection;	//0:forward 1:reverse


}moto_imu_measure_t;

typedef struct{
    int16_t 	 	speed_rpm;          // 电机转速，单位：rpm
    float  real_current;               // 电机实际电流
    int16_t 	given_current;         // 电机设定电流
    volatile uint8_t  	temperature;          // 电机温度
    volatile uint16_t 	angle;		   // 绝对角度，范围：[0,8191]
    volatile uint16_t 	last_angle;	   // 上次绝对角度，范围：[0,8191]
    int16_t 		offset_angle;      // 角度偏移量
    int32_t 		round_cnt;         // 电机转过的圈数
    int32_t 		total_angle;           // 电机总角度
    uint8_t 		buf_idx;           // 缓冲区索引
//	u16		angle_buf[FILTER_BUF_LEN];
    uint16_t 		fited_angle;       // 滤波后的角度
    uint32_t 		msg_cnt;           // 消息计数
    uint8_t 		lost_cnt;          // 丢失计数

    bool	lost_flag;		       // 电机丢失标志，1为丢失，0为在线
    bool	online_flag;           // 电机在线标志

}moto_eco_measure_t;

typedef struct
{
    uint8_t DM_J4310_InitFlag;//电机初始化标志位
    uint8_t mode;
    uint8_t ID;
    uint8_t ERROR;
    float total_position;
    /*电机反馈的整形原始数据*/
    uint16_t p_int;
    uint16_t v_int;
    uint16_t t_int;
    float Last_position; //位置
    float position; //位置
    float velocity; //速度
    float torque;   //扭矩
    float  round_cnt;
    float CAHSSIS_position;
    unsigned char    reverse_derection;	//0:forward 1:reverse
}DM_J4310_measure_t;

typedef struct{
    uint8_t Link_InitFlag;
    uint8_t command ;//命令
    int8_t	temperature;//电机温度1°C/LSB
    int16_t  	iq;//转矩电流值范围-2048~2048对应-33A~33A
    int16_t 	speed;//1dps/LSB
    uint16_t encoder;//14bit编码器的数值范围0~16383
    int32_t   Accel;//加速度1dps/s
    uint16_t  encoderRaw;//编码器原始位置14bit编码器的数值范围0~16383
    uint16_t  encoderOffset;//编码器零偏该点作为电机角度的0点
    int64_t   motorAngle;//电机角度正值为顺时针累计角度0.01°/LSB
    uint16_t  circleAngle;//电机单圈角度以编码器零点为起始点0.01°/LSB，0~35999
    uint16_t  voltage;//电压0.1V/LSB
    uint8_t   errorState;//错误标志
    int16_t   power;//输出功率-1000~1000
    int32_t	  angle;
    float   vision_encoder;

}mf9025_measure_t;

// 电机参数结构体定义
typedef struct motor_param_t{
    int16_t    ID;                          // 电机ID
    char * Name;                            // 电机名称
    motor_type_t  MotorType;             // 电机类型，参数定义见"motor_type_t"
     canbus_t CanBus;                     // CAN总线类型，参数定义见"canbus_t"，表示电机挂载的CAN总线
    uint8_t TotalAngleState;					//PID计算时是否使用总角度
    float TempCalc;									//双环pid的位置环输出
    float ImuTar;                           // IMU模式下的电机目标值
    float EcoTar;                           // ECO模式下的电机目标值
    float EcoTorque;
    float CurrentOutput;                    // PID计算后的输出值，可用于CAN发送缓冲区
    uint8_t Total_angle_Ctrl;               // 总角度控制模式标志
    // uint8_t DMPsoTar;
    // uint8_t DM
    // 反馈模式和闭环模式(当前被注释)
    feedback_mode_t FeedbackMode;        // 电机控制模式，参数定义见"feedback_mode_t"
    closeloop_mode_t CloseLoopMode;      // 电机闭环模式，参数定义见"closeloop_mode_t"
    // IMU反馈参数结构体指针(当前被注释)
     moto_imu_measure_t * MeasureImu;     // IMU反馈参数结构体

#if (DJI_MOTOR_LIBRART == 1)
    moto_eco_measure_t * MeasureEco;        // DJI电机ECO反馈参数结构体
    void (*DjiMotorCalc) (moto_eco_measure_t * MeasureEco, float Tar);  // DJI电机计算函数指针
#endif

#if (DM_MOTOR_LIBRART == 1)
    DM_J4310_measure_t * MeasureDM;         // DM电机ECO反馈参数结构体
    void (*DmMotorCalc) (DM_J4310_measure_t * MeasureDM, float Tar);    // DM电机计算函数指针
#endif

//     LINK电机相关定义
#if (LINK_MOTOR_LIBRART == 1)
        mf9025_measure_t   * MeasureLINK;   // LINK电机反馈参数结构体
#endif

    // PID参数结构体指针
     pid_t * PidParamPosEco;              // ECO模式位置PID参数结构体
     pid_t * PidParamSpdEco;              // ECO模式速度PID参数结构体
     pid_t * PidParamPosImu;              // IMU模式位置PID参数结构体
     pid_t * PidParamSpdImu;              // IMU模式速度PID参数结构体
    //

    //

    //
    // 电机控制函数指针(当前被注释)
    // void (*EcoSpeedCtrl) (s16 speed_rpm); // ECO速度控制函数指针
    // void (*EcoPosCtrl)   (s16 pos_out);   // ECO位置控制函数指针
    // void (*ImuSpeedCtrl) (s16 speed_rpm); // IMU速度控制函数指针
    // void (*ImuPosCtrl)   (s16 pos_out);   // IMU位置控制函数指针

    // 电机计算函数指针(旧版本定义，当前被注释)
    // void (*DjiMotorCalc) (motor_param_t pxMotorParam, fp32 Tar);
    // void (*DmMotorCalc) (motor_param_t pxMotorParam, fp32 Tar);
    // void (*LinkMotorCalc) (motor_param_t pxMotorParam, fp32 Tar);

}motor_param_t;

extern motor_param_t xMotorParam[MOTOR_NUM];
extern pid_t xMotorPid[2][2][MOTOR_NUM];
void MotorInit(void);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
void MotorCalc(motor_param_t * pxMotorParam);


#endif //BIG_WHEELLEG_MOTOR_H
