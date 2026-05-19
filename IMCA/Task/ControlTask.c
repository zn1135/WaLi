#include "ControlTask.h"

#include "adc_handle.h"
#include "main.h"
#include "usart.h"
#include "user_lib.h"
#include "imu_data_decode.h"
#include "UartTask.h"
#include "CanBus_Task.h"
#include "CapInteract.h"
#include "Motor.h"
#include "Vofa_send.h"
#include "RcTask.h"
#include "ChassisControl.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include "FreeRTOS.h"
#include "GimbalControl.h"
#include "IMU_Task.h"
#include "task.h"
#include "INS_task.h"
#include "Judge_interact.h"
#include "PowerLimit.h"
#include "Ramp.h"
#include "ShootControl.h"
#include "Vision_interact.h"
#include "VBatSense.h"

// VBatSense_Config_t cfg = {
//     .vref_mv = 3300,
//     .r_top_ohm = 10000,
//     .r_bottom_ohm = 1000
// };






robot_mode_t xRobotMode;
uint8_t Gimbal_Init_Flag = 0;
uint8_t DM_again_flag = 0;
uint8_t aRxBuffer1[150];
uint8_t Gimbal_Height_flag = High;
uint8_t cnt=0;
uint8_t fh=0;
#define GPIO_DEBOUNCE_MAX_ITEMS 16U // 最多支持的GPIO消抖通道数

#if (GPIO_DEBOUNCE_TICKS < 5U)
#define GPIO_DEBOUNCE_TICKS_EFFECTIVE 5U
#elif (GPIO_DEBOUNCE_TICKS > 10U)
#define GPIO_DEBOUNCE_TICKS_EFFECTIVE 10U
#else
#define GPIO_DEBOUNCE_TICKS_EFFECTIVE GPIO_DEBOUNCE_TICKS
#endif

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t stable_state;
    uint8_t last_sample;
    uint8_t sample_count;
    uint8_t used;
} gpio_debounce_state_t;

// 按“端口+引脚”保存独立的消抖状态，避免不同输入互相影响
static gpio_debounce_state_t s_gpio_debounce[GPIO_DEBOUNCE_MAX_ITEMS];

static gpio_debounce_state_t *GPIO_FindOrCreateDebounceState(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    uint8_t i;
    // 先查找已存在的状态槽
    for (i = 0U; i < GPIO_DEBOUNCE_MAX_ITEMS; i++) {
        if (s_gpio_debounce[i].used && s_gpio_debounce[i].port == GPIOx && s_gpio_debounce[i].pin == GPIO_Pin) {
            return &s_gpio_debounce[i];
        }
    }

    // 没找到则创建新槽，初始稳定态设为当前原始电平
    for (i = 0U; i < GPIO_DEBOUNCE_MAX_ITEMS; i++) {
        if (!s_gpio_debounce[i].used) {
            uint8_t raw = (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) ? 1U : 0U;
            s_gpio_debounce[i].used = 1U;
            s_gpio_debounce[i].port = GPIOx;
            s_gpio_debounce[i].pin = GPIO_Pin;
            s_gpio_debounce[i].stable_state = raw;
            s_gpio_debounce[i].last_sample = raw;
            s_gpio_debounce[i].sample_count = GPIO_DEBOUNCE_TICKS_EFFECTIVE;
            return &s_gpio_debounce[i];
        }
    }

    return NULL;
}



void TotalInitTask(void) {
    CANFilterInit();
    xRobotMode.CtrlMode = LOSE;

#if (IMU_UPDATA == 1)
    HAL_UARTEx_ReceiveToIdle_DMA_NoIT(&huart5); // 重新使能串口5接收中断

    // HAL_UART_Receive_DMA(&huart1, aRxBuffer1, sizeof(aRxBuffer1));
    // imu_data_decode_init();
    // xvEstimateKF_Init(&vaEstimateKF);//卡尔曼滤波初始化
#endif
    MouseRampInit();
    HAL_UARTEx_ReceiveToIdle_DMA_NoIT(&huart2); /*启动串口空闲中断，用于接收遥控器发来的数据*/
    HAL_UARTEx_ReceiveToIdle_DMA_NoIT(&huart3); /*启动串口空闲中断，用于接收JUDGE发来的数据*/
    HAL_UARTEx_ReceiveToIdle_DMA_NoIT(&hlpuart1); /*启动串口空闲中断，用于接收VT发来的数据*/
    MotorInit(); //电机参数初始化
    PidInit(); //pid初始化
    const static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static float chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    const static float chassis_w_order_filter[1] = {CHASSIS_ACCEL_W_NUM};
    FirstOrderFilterInit(&xChassisCmdSlowSetVx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    FirstOrderFilterInit(&xChassisCmdSlowSetVy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    FirstOrderFilterInit(&xChassisCmdSlowSetVw, CHASSIS_CONTROL_TIME, chassis_w_order_filter);
    // VBatSense_Init(&hadc3, &cfg);
    ADC_handle_Init();
}
uint32_t vbat_mv = 0;

void TotalControl(void) {
    // GPIO_Read(GPIOC, GPIO_PIN_9);
    // GPIO_Read(GPIOC, GPIO_PIN_7);
#if (IMU_UPDATA == 1)
    Imu_param_update(&receive_imusol, &IMU_Yaw, &IMU_Pitch, &IMU_Roll); // 获取陀螺仪数据
    MotorImuUpdata(&xMotorParam[YAW], &IMU_Yaw);
    // MotorImuUpdata(&xMotorParam[PITCH], &IMU_Pitch);

    xMotorParam[PITCH].MeasureImu->angle = -IMU_Pitch.angle/180*PI;
    xMotorParam[PITCH].MeasureImu->total_angle = -IMU_Pitch.total_angle/180*PI;
    xMotorParam[PITCH].MeasureImu->palstance =IMU_Pitch.speed_gyro/180*PI;
    // xMotorParam[PITCH].MeasureImu->palstance = -xMotorParam[PITCH].MeasureDM->velocity;
#endif
    CtrlModeSelect();
    FirstOrderFilterCali(&mouse_x, MOUSE_X);//一阶低通滤波
    FirstOrderFilterCali(&mouse_y, MOUSE_Y);//一阶低通滤波
    // xMotorParam[Leg_RB].MeasureLINK->encoder = (xMotorParam[Leg_RB].MeasureLINK->encoder<3.14)?3.14-xMotorParam[Leg_RB].MeasureLINK->encoder:9.42-xMotorParam[Leg_RB].MeasureLINK->encoder;
    deshake_all();
    TotalControlSetValue();
    // Vofa_Send_Data8( Cap_Measure.battary_power,//0
    //                 Cap_Measure.chassis_power, //1
    //                Cap_Measure.cap_power, //2
    //                Cap_Measure.cap_value, //3
    //                 xMotorParam[YAW].CurrentOutput, //4
    //                GPIO_Read(GPIOC, GPIO_PIN_9), //5
    //               send_chassis_power_limit() , //6
    //                Chassis_power_buffer()
    // );
    // Vofa_Send_Data8( xMotorParam[FRICTR].MeasureEco->speed_rpm,//0
    //           xMotorParam[FRICTL].MeasureEco->speed_rpm, //1
    //           FireSpeed(), //2
    //           xMotorParam[PITCH].MeasureImu->angle , //3
    //           xMotorParam[YAW].ImuTar, //4
    //           xMotorParam[YAW].MeasureImu->total_angle, //5
    //           xMotorParam[PITCH].EcoTar, //6
    //           Chassis_Rotate.pos_out
    // );
    // Vofa_Send_Data8(Gimbal_Height_flag,//0
    //             xMotorParam[Height].EcoTar, //1
    //             xRobotMode.ChassisMode , //2
    //            highlow_flag , //3
    //            remote_control.ch5, //4
    //             Gimbal_Init_Flag, //5
    //            mouse_x.out, //6
    //            remote_control.mouse.x
    // );xMotorParam[WHEEL_FR].Timeout_Flag
    //
    // Vofa_Send_Data16(
    //     xMotorParam[PLUCK].MeasureEco->speed_rpm, //0
    //     xMotorParam[PLUCK].EcoTar,//1
    //     FireSpeed(),//2
    //     xMotorParam[FRICTR].MeasureEco->speed_rpm,//3
    //     xMotorParam[FRICTL].MeasureEco->speed_rpm,//4
    //     VT_control.PauseButton,//5
    //     VT_control.Custom_leftKey,//6
    //     VT_control.Custom_rightKey,//7
    //     0,//8
    //     VT_control.Trigger_key,//9
    //     VT_control.mouse.x,//10
    //     VT_control.mouse.y,//11
    //     0,//12
    //     0,//13
    //     0,//14
    //     0//15
    // );
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc3);
//      Vofa_Send_Data8(xMotorParam[FRICTR].MeasureEco->speed_rpm,//0
//               xMotorParam[FRICTL].MeasureEco->speed_rpm, //1
//              xMotorParam[FRICTR].EcoTar , //2
//                FireSpeed(), //3
//               xMotorParam[Height].EcoTar, //4
//               // highlow_flag, //5
//                // ABS(xMotorParam[YAW].MeasureDM->position-Gimbal_FrontAngle), //6
//                // HAL_GetTick(),
//                Chassis_Power.pos_out ,
//               Cap_Measure.cap_value,
//                sqrt((chassis_vx_channel*chassis_vx_channel+chassis_vy_channel*chassis_vy_channel))/CHASSIS_MAXSPEED
// );
//     Vofa_Send_Data8(Cap_Measure.battary_power,//0
//           Cap_Measure.chassis_power, //1
//           Cap_Measure.cap_power, //2
//            Cap_Measure.cap_value, //3
//         Cap_Measure.state_core, //4
//          // highlow_flag, //5
//           // ABS(xMotorParam[YAW].MeasureDM->position-Gimbal_FrontAngle), //6
//           // HAL_GetTick(),
//           xMotorParam[YAW].MeasureDM->torque,
//           xMotorParam[YAW].CurrentOutput,
//           sqrt((chassis_vx_channel*chassis_vx_channel+chassis_vy_channel*chassis_vy_channel))/CHASSIS_MAXSPEED
// );
    Vofa_Send_Data8(send_robot_hurt_id() ,//0
                  send_robot_hurt_type(), //1
                   xMotorParam[PITCH].MeasureImu->palstance , //2
                    xMotorParam[YAW].MeasureImu->palstance , //3
                    xMotorParam[YAW].CurrentOutput, //4.
                  xMotorParam[YAW].MeasureDM->velocity-xMotorParam[YAW].MeasureDM->last_v, //5
                  xMotorParam[YAW].MeasureDM->velocity, //6
                  Stuck_YAW_cnt
    );
    // Vofa_Send_Data8(-MiniPC_ReceivePacket.pitch,//0
    //                 xMotorParam[PITCH].MeasureImu->angle, //1
    //                  xMotorParam[PITCH].TempCalc  , //2
    //                 xMotorParam[PITCH].MeasureImu->palstance, //3
    //                 xMotorParam[PITCH].CurrentOutput,//4
    //                 IMU_Pitch.speed_gyro/180*PI, //5
    //                 xMotorParam[YAW].ImuTar-xMotorParam[YAW].MeasureImu->total_angle, //6
    //                 Stuck_YAW_cnt //7
    // );
    // Vofa_Send_Data16(xMotorParam[WHEEL_FR].MeasureEco->speed_rpm,//0
    //                 xMotorParam[WHEEL_FL].MeasureEco->speed_rpm  , //1
    //                 xMotorParam[WHEEL_BR].MeasureEco->speed_rpm , //2
    //                 xMotorParam[WHEEL_BL].MeasureEco->speed_rpm, //3
    //                 xMotorParam[WHEEL_FR].EcoTar,//4
    //                 xMotorParam[WHEEL_FL].EcoTar, //5
    //                 xMotorParam[WHEEL_BR].EcoTar, //6
    //                 xMotorParam[WHEEL_BL].EcoTar, //7
    //                 xMotorParam[WHEEL_FR].CurrentOutput,
    //                 xMotorParam[WHEEL_FL].CurrentOutput,
    //                 xMotorParam[WHEEL_BR].CurrentOutput,
    //                 xMotorParam[WHEEL_BL].CurrentOutput,
    //                 xMotorParam[WHEEL_FR].PidParamSpdEco->pos_out,
    //                 xMotorParam[WHEEL_FL].PidParamSpdEco->pos_out,
    //                 xMotorParam[WHEEL_BR].PidParamSpdEco->pos_out,
    //                 xMotorParam[WHEEL_BL].PidParamSpdEco->pos_out
    //
    // );
    //


}


void CtrlModeSelect(void) {

    if (RoboHP()==0) {
        xRobotMode.ChassisMode = CHASSIS_FOLLOW;
        fh=1;
        // LiftLowerControl();
        // Gimbal_Init_Flag=0;
    }else if (fh==1) {
        osDelay(100);
        fh=0;
    }
        if (RC_UPPER_LEFT_SW_UP) {
            xRobotMode.CtrlMode = PC;
        } else if (RC_UPPER_LEFT_SW_MID) {
            xRobotMode.CtrlMode = RC;
        } else if (RC_UPPER_LEFT_SW_DOWN) {
            xRobotMode.CtrlMode = LOSE;
        } else;
        if (xRobotMode.CtrlMode == RC) {
            if (Gimbal_Height_flag==High)
            {
                if (RC_UPPER_RIGHT_SW_UP) {
                    xRobotMode.ChassisMode = CHASSIS_SEPARA;
                } else if (RC_UPPER_RIGHT_SW_MID) {
                    xRobotMode.ChassisMode = CHASSIS_FOLLOW;
                } else if (RC_UPPER_RIGHT_SW_DOWN) {
                    xRobotMode.ChassisMode = CHASSIS_GYROSC;
                } else;
            }else {
                xRobotMode.ChassisMode = CHASSIS_FOLLOW;
            }
        } else if (xRobotMode.CtrlMode == PC) {
            if (RC_UPPER_RIGHT_SW_UP) {
                xRobotMode.GimbalMode = NOSHOOT_MODE;
            } else if (RC_UPPER_RIGHT_SW_MID) {
                xRobotMode.GimbalMode = NORMAL_MODE;
            } else if (RC_UPPER_RIGHT_SW_DOWN) {
                xRobotMode.GimbalMode = UNBOUNCE_MODE;
                xRobotMode.ChassisMode = CHASSIS_SEPARA;
            } else;
        } else;
}


void PidCalc(void) {
    MotorCalc(&xMotorParam[WHEEL_FR]);
    MotorCalc(&xMotorParam[WHEEL_FL]);
    MotorCalc(&xMotorParam[WHEEL_BR]);
    MotorCalc(&xMotorParam[WHEEL_BL]);
    MotorCalc(&xMotorParam[YAW]);
    MotorCalc(&xMotorParam[PITCH]);
    MotorCalc(&xMotorParam[FRICTR]);
    MotorCalc(&xMotorParam[FRICTL]);
    MotorCalc(&xMotorParam[PLUCK]);
    MotorCalc(&xMotorParam[Height]);
}


void TotalControlSetValue(void) {
    if (!Gimbal_Init_Flag) {
        GimbalInit();
    } else if (Gimbal_Init_Flag) {
        if(ABS(CH5)>500||highlow_flag!=0) {
            LiftLowerControl();
        }
        GimbalControlSetValue();
        ShootControlSetValue();
        if (ABS(xMotorParam[FRICTR].MeasureEco->speed_rpm)>4000&&ABS(xMotorParam[FRICTL].MeasureEco->speed_rpm)>4000) {
            PluckControl();
        }

        ChassisControlSetValue();
        ChassisControl_Calculating();
        PidCalc();
        PowertotalLimit();
    }
}


void ALL_reset(uint8_t reset) {
    if (reset) //复位
    {
        HAL_Delay(100);
        __set_FAULTMASK(1);
        HAL_NVIC_SystemReset();
    }
}


uint8_t GPIO_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    // 读取原始电平（未消抖）
    uint8_t raw = (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    gpio_debounce_state_t *state = GPIO_FindOrCreateDebounceState(GPIOx, GPIO_Pin);

    if (state == NULL) {
        // 状态池满时回退为原始读值，保证函数可用
        return raw;
    }

    if (raw == state->last_sample) {
        // 连续采样一致：计数累加直到阈值
        if (state->sample_count < GPIO_DEBOUNCE_TICKS_EFFECTIVE) {
            state->sample_count++;
        }
    } else {
        // 电平变化：重置计数并开始确认新状态
        state->last_sample = raw;
        state->sample_count = 1U;
    }

    // 达到阈值后才更新稳定态（稳态电平模式）
    if (state->sample_count >= GPIO_DEBOUNCE_TICKS_EFFECTIVE) {
        state->stable_state = raw;
    }

    cnt = state->sample_count;
    return state->stable_state;
}

void LiftLowerControl() {
    static uint8_t high_enter_cnt = 0U;
    static uint8_t low_enter_cnt = 0U;
    static uint8_t high_exit_cnt = 0U;
    static uint8_t low_exit_cnt = 0U;

    // 先读取消抖后的稳定电平
    uint8_t high_hit = GPIO_Read(GPIOC, High_GPIO);
    uint8_t low_hit = GPIO_Read(GPIOC, Low_GPIO);

    // 三态滞回状态机：入态阈值短、出态阈值长，减少边界抖动导致的来回跳变
    switch (Gimbal_Height_flag) {
        case High:
            high_enter_cnt = 0U;
            low_enter_cnt = 0U;
            low_exit_cnt = 0U;
            if (high_hit) {
                high_exit_cnt = 0U;
            } else {
                if (high_exit_cnt < HEIGHT_STATE_EXIT_TICKS) {
                    high_exit_cnt++;
                }
                if (high_exit_cnt >= HEIGHT_STATE_EXIT_TICKS) {
                    Gimbal_Height_flag = Middle;
                    high_exit_cnt = 0U;
                }
            }
            break;

        case Low:
            high_enter_cnt = 0U;
            low_enter_cnt = 0U;
            high_exit_cnt = 0U;
            if (low_hit) {
                low_exit_cnt = 0U;
            } else {
                if (low_exit_cnt < HEIGHT_STATE_EXIT_TICKS) {
                    low_exit_cnt++;
                }
                if (low_exit_cnt >= HEIGHT_STATE_EXIT_TICKS) {
                    Gimbal_Height_flag = Middle;
                    low_exit_cnt = 0U;
                }
            }
            break;

        case Middle:
        default:
            high_exit_cnt = 0U;
            low_exit_cnt = 0U;
            // 两个限位同时有效时保持Middle，避免异常接线引发误判
            if (high_hit && !low_hit) {
                if (high_enter_cnt < HEIGHT_STATE_ENTER_TICKS) {
                    high_enter_cnt++;
                }
                low_enter_cnt = 0U;
                if (high_enter_cnt >= HEIGHT_STATE_ENTER_TICKS) {
                    Gimbal_Height_flag = High;
                    high_enter_cnt = 0U;
                }
            } else if (low_hit && !high_hit) {
                if (low_enter_cnt < HEIGHT_STATE_ENTER_TICKS) {
                    low_enter_cnt++;
                }
                high_enter_cnt = 0U;
                if (low_enter_cnt >= HEIGHT_STATE_ENTER_TICKS) {
                    Gimbal_Height_flag = Low;
                    low_enter_cnt = 0U;
                }
            } else {
                high_enter_cnt = 0U;
                low_enter_cnt = 0U;
            }
            break;
    }
}