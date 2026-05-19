//
// Created by zn1135 on 25-9-24.
//

#include "CanBus_Task.h"

#include <stdlib.h>

#include "arm_math.h"
#include "CapInteract.h"
#include "ChassisControl.h"
#include "main.h"
#include "fdcan.h"
#include "Motor.h"
#include "ControlTask.h"

#include "FreeRTOS.h"
#include "GimbalControl.h"
#include "IMU_Task.h"
#include "task.h"
#include "RcTask.h"
#include "Vision_interact.h"
FDCAN_TxHeaderTypeDef tx_header;
uint8_t tx_data[8];
int16_t CanBuff[3][2][4];

/**********************************************************************************************************************
  * @Func	 void CANFilterInit(void)
  * @Brief   CAN总线滤波器初始化
  * @Param	 无
  * @Retval	 无
  * @Note    配置3路FDCAN的滤波器并启动，激活RX FIFO0新消息中断
*********************************************************************************************************************/
void CANFilterInit(void) {
    FDCAN_FilterTypeDef CAN_FilterConfigStructure;

    CAN_FilterConfigStructure.IdType = FDCAN_STANDARD_ID;
    CAN_FilterConfigStructure.FilterType = FDCAN_FILTER_MASK;
    CAN_FilterConfigStructure.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    CAN_FilterConfigStructure.FilterID1 = 0x000;
    CAN_FilterConfigStructure.FilterID2 = 0x000;

    CAN_FilterConfigStructure.FilterIndex = 0;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &CAN_FilterConfigStructure);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,FDCAN_TX_BUFFER0);

    CAN_FilterConfigStructure.FilterIndex = 9;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &CAN_FilterConfigStructure);
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,FDCAN_TX_BUFFER1);

    CAN_FilterConfigStructure.FilterIndex = 18;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &CAN_FilterConfigStructure);
    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,FDCAN_TX_BUFFER2);
}

/**********************************************************************************************************************
  * @Func	 void SetCanFronMotorValue(FDCAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
  * @Brief   CAN1发送前侧四个底盘电机电流值
  * @Param	 FDCAN_HandleTypeDef *hcan：FDCAN句柄
  *          int16_t iq1~iq4：四个电机的电流设定值
  * @Retval	 无
  * @Note    使用CAN ID 0x200，每个电流占2字节(高8位+低8位)
*********************************************************************************************************************/
void SetCanFronMotorValue(FDCAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4) {
    tx_header.Identifier = 0x200;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = 0x08;


    tx_data[0] = (iq1 >> 8);
    tx_data[1] = iq1;
    tx_data[2] = (iq2 >> 8);
    tx_data[3] = iq2;
    tx_data[4] = (iq3 >> 8);
    tx_data[5] = iq3;
    tx_data[6] = (iq4 >> 8);
    tx_data[7] = iq4;

    HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, tx_data);
}


/**********************************************************************************************************************
  * @Func	 void SetCanBackMotorValue(FDCAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
  * @Brief   CAN2发送后侧四个电机电流值
  * @Param	 FDCAN_HandleTypeDef *hcan：FDCAN句柄
  *          int16_t iq1~iq4：四个电机(高度/摩擦轮R/摩擦轮L/拨弹轮)的电流设定值
  * @Retval	 无
  * @Note    使用CAN ID 0x1FF
*********************************************************************************************************************/
void SetCanBackMotorValue(FDCAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4) {
    tx_header.Identifier = 0x1FF;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = 0x08;


    tx_data[0] = (iq1 >> 8);
    tx_data[1] = iq1;
    tx_data[2] = (iq2 >> 8);
    tx_data[3] = iq2;
    tx_data[4] = (iq3 >> 8);
    tx_data[5] = iq3;
    tx_data[6] = (iq4 >> 8);
    tx_data[7] = iq4;

    HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, tx_data);
}

/**********************************************************************************************************************
  * @Func	 void set_DM_J4310_motor_State(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t function)
  * @Brief   设置DM_J4310电机状态(使能/失能/零点保存/清除错误)
  * @Param	 FDCAN_HandleTypeDef *hcan：FDCAN句柄
  *          uint16_t id：电机CAN ID
  *          uint8_t function：功能码(enable/disable/save_zero_point/clear_error)
  * @Retval	 无
*********************************************************************************************************************/
void set_DM_J4310_motor_State(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t function) {
    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = 0x08;

    tx_data[0] = 0xFF;
    tx_data[1] = 0xFF;
    tx_data[2] = 0xFF;
    tx_data[3] = 0xFF;
    tx_data[4] = 0xFF;
    tx_data[5] = 0xFF;
    tx_data[6] = 0xFF;

    switch (function) {
        case enable: tx_data[7] = 0xFC;
            break;
        case disable: tx_data[7] = 0xFD;
            break;
        case save_zero_point: tx_data[7] = 0xFE;
            break;
        case clear_error: tx_data[7] = 0xFB;
            break;
    }

    HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, tx_data);
}

/**********************************************************************************************************************
  * @Func	 void set_DM_J4310_SPD_info(FDCAN_HandleTypeDef *hcan, uint16_t id, float _vel)
  * @Brief   设置DM_J4310电机速度模式目标值
  * @Param	 FDCAN_HandleTypeDef *hcan：FDCAN句柄
  *          uint16_t id：电机CAN ID
  *          float _vel：目标速度值
  * @Retval	 无
*********************************************************************************************************************/
void set_DM_J4310_SPD_info(FDCAN_HandleTypeDef *hcan, uint16_t id, float _vel) {
    uint8_t *vbuf;
    vbuf = (uint8_t *) &_vel;

    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = 0x08;

    tx_data[0] = *vbuf;
    tx_data[1] = *(vbuf + 1);
    tx_data[2] = *(vbuf + 2);
    tx_data[3] = *(vbuf + 3);

    HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, tx_data);
}

/**********************************************************************************************************************
  * @Func	 void set_DM_J4310_MIT_info(FDCAN_HandleTypeDef *hcan, uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
  * @Brief   设置DM_J4310电机MIT模式控制信息
  * @Param	 FDCAN_HandleTypeDef *hcan：FDCAN句柄
  *          uint16_t id：电机CAN ID
  *          float _pos：目标位置
  *          float _vel：目标速度
  *          float _KP：位置环比例系数
  *          float _KD：速度环微分系数
  *          float _torq：前馈力矩
  * @Retval	 无
  * @Note    将浮点参数经float_to_uint转换后按MIT协议打包发送
*********************************************************************************************************************/
void set_DM_J4310_MIT_info(FDCAN_HandleTypeDef *hcan,

                            uint16_t id, float _pos, float _vel,
                            float _KP, float _KD, float _torq)
{
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = 0x08;

    tx_data[0] = (pos_tmp >> 8);//取出高八位
    tx_data[1] =  pos_tmp;
    tx_data[2] = (vel_tmp >> 4);
    tx_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    tx_data[4] =  kp_tmp;
    tx_data[5] = (kd_tmp >> 4);
    tx_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    tx_data[7] =  tor_tmp;

    if(HAL_FDCAN_GetTxFifoFreeLevel(hcan)!=0)
        HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, tx_data);
}




/**********************************************************************************************************************
  * @Func	 void set_link_motor_State(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t function)
  * @Brief   设置LINK电机状态(使能/失能/零点保存)
  * @Param	 FDCAN_HandleTypeDef *hcan：FDCAN句柄
  *          uint16_t id：电机CAN ID
  *          uint8_t function：功能码(enable/disable/save_zero_point)
  * @Retval	 无
*********************************************************************************************************************/
void set_link_motor_State(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t function) {
    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = 0x08;
    tx_data[1] = 0x00;
    tx_data[2] = 0x00;
    tx_data[3] = 0x00;
    tx_data[4] = 0x00;
    tx_data[5] = 0x00;
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;
    switch (function) {
        case enable: tx_data[0] = 0x88;
            break;
        case disable: tx_data[0] = 0x81;
            break;
        case save_zero_point: tx_data[0] = 0x19;
            break;
    }
    HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, tx_data);
}

/**********************************************************************************************************************
  * @Func	 void set_link_motor_iq(FDCAN_HandleTypeDef *hcan, uint16_t id, int16_t iq)
  * @Brief   设置LINK电机电流值
  * @Param	 FDCAN_HandleTypeDef *hcan：FDCAN句柄
  *          uint16_t id：电机CAN ID
  *          int16_t iq：电流设定值
  * @Retval	 无
*********************************************************************************************************************/
void set_link_motor_iq(FDCAN_HandleTypeDef *hcan, uint16_t id, int16_t iq) {
    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = 0x08;
    tx_data[0] = 0xA1;
    tx_data[1] = 0x00;
    tx_data[2] = 0x00;
    tx_data[3] = 0x00;
    tx_data[4] = *(uint8_t*)(&iq);
    tx_data[5] = *((uint8_t*)(&iq)+1);
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;
    HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, tx_data);
}

/**********************************************************************************************************************
  * @Func	 void GetMotorMeasureNew(motor_param_t *motor_param, const uint8_t can_rx_data[])
  * @Brief   解析CAN接收的电机测量数据
  * @Param	 motor_param_t *motor_param：电机参数结构体指针
  *          const uint8_t can_rx_data[]：CAN接收数据缓冲区
  * @Retval	 无
  * @Note    根据电机类型(DJI/DM/LINK)分别解析角度、转速、电流和位置信息
*********************************************************************************************************************/
void GetMotorMeasureNew(motor_param_t *motor_param, const uint8_t can_rx_data[]) {
    motor_param->Timeout_Flag++;
    switch (motor_param->MotorType) {
        case DJI:
            motor_param->MeasureEco->last_angle = motor_param->MeasureEco->angle;
            motor_param->MeasureEco->angle = (uint16_t) (can_rx_data[0] << 8 | can_rx_data[1]);
            motor_param->MeasureEco->speed_rpm = (int16_t) (can_rx_data[2] << 8 | can_rx_data[3]);
            motor_param->MeasureEco->real_current = (int16_t) (can_rx_data[4] << 8 | can_rx_data[5]);
            if (motor_param->MeasureEco->angle - motor_param->MeasureEco->last_angle > 4096)
                motor_param->MeasureEco->round_cnt--;
            else if (motor_param->MeasureEco->angle - motor_param->MeasureEco->last_angle < -4096)
                motor_param->MeasureEco->round_cnt++;
            motor_param->MeasureEco->total_angle =
                    motor_param->MeasureEco->round_cnt * 8192 + motor_param->MeasureEco->angle - motor_param->MeasureEco
                    ->offset_angle;
            break;
        case DM:
            motor_param->MeasureDM->ID = can_rx_data[0] & 0x0F;
            motor_param->MeasureDM->ERROR = can_rx_data[0] >> 4;

            motor_param->MeasureDM->p_int = (can_rx_data[1] << 8) | can_rx_data[2];
            motor_param->MeasureDM->v_int = (can_rx_data[3] << 4) | (can_rx_data[4] >> 4);
            motor_param->MeasureDM->t_int = ((can_rx_data[4] & 0xF) << 8) | can_rx_data[5];
            motor_param->MeasureDM->Last_position = motor_param->MeasureDM->position;
            motor_param->MeasureDM->position = uint_to_float(motor_param->MeasureDM->p_int,P_MIN,P_MAX, 16);
            //(-12.5,12.5)
            motor_param->MeasureDM->last_v=motor_param->MeasureDM->velocity;
            motor_param->MeasureDM->velocity = uint_to_float(motor_param->MeasureDM->v_int,V_MIN,V_MAX, 12); //(-30,30)

            motor_param->MeasureDM->torque = uint_to_float(motor_param->MeasureDM->t_int,T_MIN,T_MAX, 12); //(-10,10)
            break;
        case LINK:
            motor_param->MeasureLINK->command = can_rx_data[0];
            motor_param->MeasureLINK->temperature = can_rx_data[1];
            motor_param->MeasureLINK->iq = (can_rx_data[3] << 8 | can_rx_data[2]);
            motor_param->MeasureLINK->speed = (can_rx_data[5] << 8 | can_rx_data[4]);
            motor_param->MeasureLINK->encoder = (can_rx_data[7] << 8 | can_rx_data[6]);

            break;
    }
}

/**********************************************************************************************************************
  * @Func	 void GetChassisMes(uint8_t buff[])
  * @Brief   解析底盘通用信息
  * @Param	 uint8_t buff[]：接收数据缓冲区
  * @Retval	 无
  * @Note    提取YAW标志、血量标志和颜色信息
*********************************************************************************************************************/
void GetChassisMes(uint8_t buff[])
{
    yaw_flag =buff[0];
    RoboHP_flag = buff[1];
    RB_Color = buff[2];
}





/**********************************************************************************************************************
  * @Func	 void MotorCanSendCurrent(void)
  * @Brief   通过CAN总线发送所有电机控制电流
  * @Param	 无
  * @Retval	 无
  * @Note    根据控制模式(LOSE/RC/PC)决定电机使能状态，发送底盘/云台/摩擦轮/拨弹轮电流
*********************************************************************************************************************/
void MotorCanSendCurrent(void) {
    if (xRobotMode.CtrlMode == LOSE) {
        set_DM_J4310_motor_State(&hfdcan3, xMotorParam[YAW].ID, disable);
        vTaskDelay (1);
        set_DM_J4310_motor_State(&hfdcan3, xMotorParam[YAW].ID, clear_error);
        xMotorParam[YAW].MeasureDM->DM_J4310_InitFlag = 0;
        SetCanFronMotorValue(&hfdcan1, 0,  0,  0, 0);
        set_DM_J4310_motor_State(&hfdcan3, xMotorParam[PITCH].ID, disable);
        // set_DM_J4310_motor_State(&hfdcan3, xMotorParam[YAW].ID, clear_error);
        xMotorParam[PITCH].MeasureDM->DM_J4310_InitFlag = 0;
        vTaskDelay (1);
        // if (xMotorParam[FRICTR].MeasureEco->speed_rpm<)
        SetCanBackMotorValue(&hfdcan2, 0,  0,  0, 0);
    } else if (xRobotMode.CtrlMode == RC || xRobotMode.CtrlMode == PC) {
        if (xMotorParam[PITCH].MeasureDM->ERROR==0) {
            set_DM_J4310_motor_State(&hfdcan3, xMotorParam[PITCH].ID, enable);
            xMotorParam[PITCH].MeasureDM->DM_J4310_InitFlag = 1;
        } else if (xMotorParam[PITCH].MeasureDM->ERROR==1) {
            // set_DM_J4310_MIT_info(&hfdcan3, xMotorParam[PITCH].ID, 0,0,0,0.0,xMotorParam[PITCH].CurrentOutput);
            set_DM_J4310_MIT_info(&hfdcan3, xMotorParam[PITCH].ID, 0,0,0,0.0,-1.1*ABS(arm_cos_f32(xMotorParam[PITCH].MeasureDM->position))+xMotorParam[PITCH].CurrentOutput);

        }
        if (xMotorParam[PITCH].MeasureDM->ERROR==1&&xMotorParam[YAW].MeasureDM->ERROR==1) {
            if (Motor_TimeOut_flag==0) {
                SetCanFronMotorValue(&hfdcan1, xMotorParam[WHEEL_FR].CurrentOutput,  xMotorParam[WHEEL_FL].CurrentOutput,  xMotorParam[WHEEL_BR].CurrentOutput,  xMotorParam[WHEEL_BL].CurrentOutput);
                // SetCanFronMotorValue(&hfdcan1, xMotorParam[WHEEL_FR].CurrentOutput,   0,   0,   0);
            }
                // SetCanFronMotorValue(&hfdcan1, 0, 0, 0 , xMotorParam[WHEEL_BL].CurrentOutput  );
            SetCanBackMotorValue(&hfdcan2, xMotorParam[Height].CurrentOutput,xMotorParam[FRICTR].CurrentOutput,xMotorParam[FRICTL].CurrentOutput,xMotorParam[PLUCK].CurrentOutput);

        }
        // SetCanFronMotorValue(&hfdcan1,0 ,  0,  0,  xMotorParam[WHEEL_BL].CurrentOutput);
        if (xMotorParam[YAW].MeasureDM->ERROR==0) {
            set_DM_J4310_motor_State(&hfdcan3, xMotorParam[YAW].ID, enable);
            xMotorParam[YAW].MeasureDM->DM_J4310_InitFlag = 1;
        } else if (xMotorParam[YAW].MeasureDM->ERROR==1) {
            if (AUTO_Shoot_flag==2) {
                if (xRobotMode.ChassisMode==CHASSIS_GYROSC) {
                    set_DM_J4310_MIT_info(&hfdcan3, xMotorParam[YAW].ID, 0.0f, 0.0f, 0.0f, 0.0f, yaw_output+((xMotorParam[WHEEL_FL].MeasureEco->speed_rpm+xMotorParam[WHEEL_FR].MeasureEco->speed_rpm+xMotorParam[WHEEL_BL].MeasureEco->speed_rpm+xMotorParam[WHEEL_BR].MeasureEco->speed_rpm)/1.414/4/670)*0.105f);
                }else {

                    set_DM_J4310_MIT_info(&hfdcan3, xMotorParam[YAW].ID, 0.0f, 0.0f, 0.0f, 0.0f, yaw_output);
                }
            }else {
                // set_DM_J4310_MIT_info(&hfdcan3, xMotorParam[YAW].ID, 0,0,0,0,((xMotorParam[WHEEL_FL].MeasureEco->speed_rpm+xMotorParam[WHEEL_FR].MeasureEco->speed_rpm+xMotorParam[WHEEL_BL].MeasureEco->speed_rpm+xMotorParam[WHEEL_BR].MeasureEco->speed_rpm)/1.414/4/670)*0.08f);
                set_DM_J4310_MIT_info(&hfdcan3, xMotorParam[YAW].ID, 0.0f, 0.0f, 0.0f, 0.0f, xMotorParam[YAW].CurrentOutput);
            }
        }



    } else;



}


/**********************************************************************************************************************
  * @Func	 void GetMotorOffset(motor_param_t *ptr, uint8_t can_rx_data[])
  * @Brief   获取电机编码器初始偏移值
  * @Param	 motor_param_t *ptr：电机参数结构体指针
  *          uint8_t can_rx_data[]：CAN接收数据缓冲区
  * @Retval	 无
  * @Note    读取当前编码器角度作为零点偏移，用于后续角度计算
*********************************************************************************************************************/
void GetMotorOffset(motor_param_t *ptr, uint8_t can_rx_data[])
{
    ptr->MeasureEco->angle = (uint16_t)(can_rx_data[0]<<8 | can_rx_data[1]) ;
    ptr->MeasureEco->offset_angle= ptr->MeasureEco->angle;
}

/******************FDCAN中断回调*******************/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    //    static uint16_t can_cnt;
    //	static uint16_t ledInit_flag = 0, ledInit_count = 0;

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);

    if (hfdcan->Instance == FDCAN1) {

        switch (rx_header.Identifier) {
            case WHEEL_FR_ID:

                GetMotorMeasureNew(&xMotorParam[WHEEL_FR], rx_data);
                break;
            case WHEEL_FL_ID:
                GetMotorMeasureNew(&xMotorParam[WHEEL_FL], rx_data);
                break;
            case WHEEL_BR_ID:
                GetMotorMeasureNew(&xMotorParam[WHEEL_BR], rx_data);
                break;
            case WHEEL_BL_ID:
                GetMotorMeasureNew(&xMotorParam[WHEEL_BL], rx_data);
                break;
        }
    } else if (hfdcan->Instance == FDCAN3) {
        switch (rx_header.Identifier) {
            case PITCH_ID:
                GetMotorMeasureNew(&xMotorParam[PITCH], rx_data);
                break;
            case YAW_ID:
                GetMotorMeasureNew(&xMotorParam[YAW], rx_data);

                // xMotorParam[YAW].MeasureDM->velocity+=0.007326f;
                xMotorParam[YAW].MeasureDM->velocity= IMU_Yaw.speed_gyro;
                xMotorParam[YAW].MeasureDM->position= xMotorParam[YAW].MeasureDM->position/PI*180;
                break;
            case CAP_ID:
                GetCapMeasure(&Cap_Measure,rx_data);
                break;

        }
    } else if (hfdcan->Instance == FDCAN2) {
        switch (rx_header.Identifier) {
            case Height_ID:
                xMotorParam[Height].MeasureEco->msg_cnt++<=50?GetMotorOffset(&xMotorParam[Height], rx_data):GetMotorMeasureNew(&xMotorParam[Height], rx_data);
                break;
            case FRICTR_ID:
                xMotorParam[FRICTR].MeasureEco->msg_cnt++<=50?GetMotorOffset(&xMotorParam[FRICTR], rx_data):GetMotorMeasureNew(&xMotorParam[FRICTR], rx_data);
                break;
            case FRICTL_ID:
                xMotorParam[FRICTL].MeasureEco->msg_cnt++<=50?GetMotorOffset(&xMotorParam[FRICTL], rx_data):GetMotorMeasureNew(&xMotorParam[FRICTL], rx_data);
                break;
            case PLUCK_ID:
                xMotorParam[PLUCK].MeasureEco->msg_cnt++<=50?GetMotorOffset(&xMotorParam[PLUCK], rx_data):GetMotorMeasureNew(&xMotorParam[PLUCK], rx_data);
                break;
        }
    }
}

/**********************************************************************************************************************
  * @Func	 void send_can2can(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t iq1, ..., uint8_t iq8)
  * @Brief   通用CAN数据发送函数
  * @Param	 FDCAN_HandleTypeDef *hcan：FDCAN句柄
  *          uint16_t id：CAN报文ID
  *          uint8_t iq1~iq8：8字节发送数据
  * @Retval	 无
*********************************************************************************************************************/
void send_can2can(FDCAN_HandleTypeDef *hcan,uint16_t id,uint8_t iq1, uint8_t iq2, uint8_t iq3, uint8_t iq4, uint8_t iq5, uint8_t iq6, uint8_t iq7, uint8_t iq8)
{

    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = 0x08;
    tx_data[0] = iq1;
    tx_data[1] = iq2;
    tx_data[2] = iq3;
    tx_data[3] = iq4;
    tx_data[4] = iq5;
    tx_data[5] = iq6;
    tx_data[6] = iq7;
    tx_data[7] = iq8;

    HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, tx_data);
}



