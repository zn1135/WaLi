#include "ControlTask.h"

#include "CanBus_Task.h"
#include "CapInteract.h"
#include "ChassisControl.h"
#include "FreeRTOS.h"
#include "GimbalControl.h"
#include "IMU_Task.h"
#include "INS_task.h"
#include "Judge_interact.h"
#include "Motor.h"
#include "Odometry.h"
#include "PowerLimit.h"
#include "Ramp.h"
#include "RcTask.h"
#include "ShootControl.h"
#include "UartTask.h"
#include "Vision_interact.h"
#include "Vofa_send.h"
#include "adc_handle.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include "imu_data_decode.h"
#include "main.h"
#include "task.h"
#include "usart.h"
#include "user_lib.h"

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
uint8_t cnt = 0;
uint8_t fh = 0;

/* 摩擦轮前馈参数：先保持零系数观察基线，再根据实车数据逐步调节。 */
#define FRICTION_FF_C_ACC               1.5f
#define FRICTION_FF_C_DIFF              20.0f
#define FRICTION_FF_GAIN         0.0f         // 加速度前馈总增益
#define FRICTION_FF_DT_S         0.001f       // 控制周期，单位：s（1 kHz）
#define FRICTION_FF_ACCEL_ALPHA  0.05f        // 加速度一阶低通滤波系数
#define FRICTION_FF_OUTPUT_LIMIT 16384.0f     // M3508 最终电流输出限幅
#define RPM_TO_RAD_PER_SEC       0.104719755f // rpm 转 rad/s：2π/60

/* 前馈计算的历史状态，仅在本文件内部使用。 */
static float s_friction_last_speed_l;
static float s_friction_last_speed_r;
static float s_friction_accel_l;
static float s_friction_accel_r;
static uint8_t s_friction_ff_initialized;
#define GPIO_DEBOUNCE_MAX_ITEMS 16U // 最多支持的GPIO消抖通道数

#if (GPIO_DEBOUNCE_TICKS < 5U)
#define GPIO_DEBOUNCE_TICKS_EFFECTIVE 5U
#elif (GPIO_DEBOUNCE_TICKS > 10U)
#define GPIO_DEBOUNCE_TICKS_EFFECTIVE 10U
#else
#define GPIO_DEBOUNCE_TICKS_EFFECTIVE GPIO_DEBOUNCE_TICKS
#endif

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t stable_state;
    uint8_t last_sample;
    uint8_t sample_count;
    uint8_t used;
} gpio_debounce_state_t;

// 按“端口+引脚”保存独立的消抖状态，避免不同输入互相影响
static gpio_debounce_state_t s_gpio_debounce[GPIO_DEBOUNCE_MAX_ITEMS];

/**********************************************************************************************************************
 * @Func	 static gpio_debounce_state_t *GPIO_FindOrCreateDebounceState(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
 * @Brief   查找或创建GPIO消抖状态槽
 * @Param	 GPIO_TypeDef *GPIOx：GPIO端口
 *          uint16_t GPIO_Pin：GPIO引脚号
 * @Retval	 gpio_debounce_state_t*：对应的消抖状态结构体指针，失败返回NULL
 * @Note    每个GPIO引脚独立维护消抖状态，状态池满时返回NULL
 *********************************************************************************************************************/
static gpio_debounce_state_t *GPIO_FindOrCreateDebounceState(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    uint8_t i;
    // 先查找已存在的状态槽
    for (i = 0U; i < GPIO_DEBOUNCE_MAX_ITEMS; i++)
    {
        if (s_gpio_debounce[i].used && s_gpio_debounce[i].port == GPIOx && s_gpio_debounce[i].pin == GPIO_Pin)
        {
            return &s_gpio_debounce[i];
        }
    }

    // 没找到则创建新槽，初始稳定态设为当前原始电平
    for (i = 0U; i < GPIO_DEBOUNCE_MAX_ITEMS; i++)
    {
        if (!s_gpio_debounce[i].used)
        {
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

/**********************************************************************************************************************
 * @Func	 void TotalInitTask(void)
 * @Brief   系统总初始化任务
 * @Param	 无
 * @Retval	 无
 * @Note    依次初始化CAN滤波器、UART/IMU、电机参数、PID参数、ADC等所有外设模块
 *********************************************************************************************************************/
void TotalInitTask(void)
{
    CANFilterInit();            // CAN滤波器初始化
    xRobotMode.CtrlMode = LOSE; // 默认lose状态
    WheelOdom_Init();
#if (IMU_UPDATA == 1)
    HAL_UARTEx_ReceiveToIdle_DMA_NoIT(&huart5); // 重新使能串口5接收中断

    // HAL_UART_Receive_DMA(&huart1, aRxBuffer1, sizeof(aRxBuffer1));
    // imu_data_decode_init();
    // xvEstimateKF_Init(&vaEstimateKF);//卡尔曼滤波初始化
#endif
    MouseRampInit();
    HAL_UARTEx_ReceiveToIdle_DMA_NoIT(&huart2);   /*启动串口空闲中断，用于接收遥控器发来的数据*/
    HAL_UARTEx_ReceiveToIdle_DMA_NoIT(&huart3);   /*启动串口空闲中断，用于接收JUDGE发来的数据*/
    HAL_UARTEx_ReceiveToIdle_DMA_NoIT(&hlpuart1); /*启动串口空闲中断，用于接收VT发来的数据*/
    MotorInit();                                  // 电机参数初始化
    PidInit();                                    // pid初始化
    const static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static float chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    const static float chassis_w_order_filter[1] = {CHASSIS_ACCEL_W_NUM};
    FirstOrderFilterInit(&xChassisCmdSlowSetVx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    FirstOrderFilterInit(&xChassisCmdSlowSetVy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    FirstOrderFilterInit(&xChassisCmdSlowSetVw, CHASSIS_CONTROL_TIME, chassis_w_order_filter);
    // VBatSense_Init(&hadc3, &cfg);
    ADC_handle_Init();
    // Odometry_Init(); // 里程计(轮速+加速度融合)初始化
}
uint32_t vbat_mv = 0;

/**********************************************************************************************************************
 * @Func	 void TotalControl(void)
 * @Brief   系统总控制循环
 * @Param	 无
 * @Retval	 无
 * @Note    依次执行IMU数据更新、控制模式选择、输入滤波、按键消抖、控制量给定
 *********************************************************************************************************************/
void TotalControl(void)
{
    // GPIO_Read(GPIOC, GPIO_PIN_9);
    // GPIO_Read(GPIOC, GPIO_PIN_7);
#if (IMU_UPDATA == 1)
    Imu_param_update(&receive_imusol, &IMU_Yaw, &IMU_Pitch, &IMU_Roll); // 获取陀螺仪数据
    MotorImuUpdata(&xMotorParam[YAW], &IMU_Yaw);
    // MotorImuUpdata(&xMotorParam[PITCH], &IMU_Pitch);

    xMotorParam[PITCH].MeasureImu->angle = -IMU_Pitch.angle / 180 * PI;
    xMotorParam[PITCH].MeasureImu->total_angle = -IMU_Pitch.total_angle / 180 * PI;
    xMotorParam[PITCH].MeasureImu->palstance = IMU_Pitch.speed_gyro / 180 * PI;
    // todo：陀螺仪坐标系和电机坐标系不一致，导致正负号错误，后续需要重新定义坐标系并修改相关代码
    //  xMotorParam[PITCH].MeasureImu->palstance = -xMotorParam[PITCH].MeasureDM->velocity;
#endif
    WheelOdom_Update(
        (xMotorParam[YAW].MeasureDM->position - Gimbal_FrontAngle) / 180.0f * 3.14159265f
    );
    // Odometry_Update(); // 里程计更新：麦轮逆解算 + 加速度卡尔曼融合 + 世界系积分
    CtrlModeSelect();
    FirstOrderFilterCali(&mouse_x, MOUSE_X); // 一阶低通滤波
    FirstOrderFilterCali(&mouse_y, MOUSE_Y); // 一阶低通滤波
    // xMotorParam[Leg_RB].MeasureLINK->encoder = (xMotorParam[Leg_RB].MeasureLINK->encoder<3.14)?3.14-xMotorParam[Leg_RB].MeasureLINK->encoder:9.42-xMotorParam[Leg_RB].MeasureLINK->encoder;
    deshake_all(); // 按键消抖（没有用到）
    TotalControlSetValue();
    Vofa_Send_Data8(
         xMotorParam[YAW].Timeout_Flag ,  // 0 右摩擦轮实际转速 (rpm)
         High_Low_Start_angle - xMotorParam[Height].MeasureEco->total_angle,  // 1 左摩擦轮实际转速 (rpm)
        xMotorParam[Height].MeasureEco->speed_rpm,                                // 2 实测弹速 (m/s)
        xMotorParam[Height].MeasureEco->real_current,                 // 3 右摩擦轮目标转速 (rpm)
        GPIO_Read(GPIOC, High_GPIO),                 // 4 左摩擦轮目标转速 (rpm)
        GPIO_Read(GPIOC, Low_GPIO),                         // 5 右摩擦轮滤波加速度 (rad/s²)
        s_friction_accel_l,                         // 6 左摩擦轮滤波加速度 (rad/s²)
        -xMotorParam[FRICTL].MeasureEco->speed_rpm // 7 归一化转速差：左轮幅值-右轮幅值 (rpm)
    );
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
    HAL_ADC_Start(&hadc1); // 想做电池电压，但硬件没有做好
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
    // Vofa_Send_Data8(send_robot_hurt_id(),                                                      // 0
    //                 send_robot_hurt_type(),                                                    // 1
    //                 xMotorParam[PITCH].MeasureImu->palstance,                                  // 2
    //                 xMotorParam[YAW].MeasureImu->palstance,                                    // 3
    //                 xMotorParam[YAW].CurrentOutput,                                            // 4.
    //                 xMotorParam[YAW].MeasureDM->velocity - xMotorParam[YAW].MeasureDM->last_v, // 5
    //                 xMotorParam[YAW].MeasureDM->velocity,                                      // 6
    //                 Stuck_YAW_cnt);
    // ===== 里程计标定打印（标定完成后可改回上面的打印）=====
    // ch2/ch5 现在是【四元数去重力后】的机体系加速度。关键验证：
    //   1) 车不动，用手俯仰/转动云台 -> ch2、ch5 应保持≈0、不随 pitch 变化。
    //      若仍随云台姿态变化 -> 四元数约定不对，把 Odometry.c 里 ODOM_QUAT_CONJ 置 1 再试。
    //   2) 推车前进 -> ch2 先正(加速)后负(减速)；右移 -> ch5 同理。
    // ch0/ch3 轮速符号：前进 ch0>0、右移 ch3>0 为正确，反了翻 ODOM_VY_SIGN/ODOM_VX_SIGN。
    // ch1/ch4(融合) 正常时贴着 ch0/ch3 走、会被盖住，要看就把 ch0/ch3 隐藏。
    //   注意：必须【推动底盘】ch1/ch4 才有值；车静止时它们=0 是正常的。
    // ch7 位置：直线推 1m 看是否≈1m，标定 CHASSIS_WHEEL_RADIUS。
    // Vofa_Send_Data8(Odom.wheel_vy,    // 0 前后速度（轮速逆解算）
    //                 Odom.vy_body,     // 1 前后速度（卡尔曼融合，应贴着 ch0）
    //                 Odom.acc_body_y,  // 2 前后加速度（去重力, m/s^2）
    //                 Odom.wheel_vx,    // 3 左右速度（轮速逆解算）
    //                 Odom.vx_body,     // 4 左右速度（卡尔曼融合，应贴着 ch3）
    //                 Odom.acc_body_x,  // 5 左右加速度（去重力, m/s^2）
    //                 Odom.yaw_chassis, // 6 底盘航向 (rad)
    //                 Odom.pos_y);      // 7 前后位置 (m)
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

/**********************************************************************************************************************
 * @Func	 void CtrlModeSelect(void)
 * @Brief   控制模式选择
 * @Param	 无
 * @Retval	 无
 * @Note    根据遥控器/PC按键状态切换控制模式(PC/RC/LOSE)和底盘模式(跟随/分离/小陀螺)
 *********************************************************************************************************************/
void CtrlModeSelect(void)
{

    if (RoboHP() == 0)
    {
        xRobotMode.ChassisMode = CHASSIS_FOLLOW; // 复活默认为跟随模式
        fh = 1;                                  // 这个是在买活的时候，可能出现掉轮的情况，虽然已经有看门狗了，但还是出现掉轮，等100ms
        // LiftLowerControl();
        // Gimbal_Init_Flag=0;
    }
    else if (fh == 1)
    {
        osDelay(100);
        fh = 0;
    }
    if (RC_UPPER_LEFT_SW_UP)
    {
        xRobotMode.CtrlMode = PC;
    }
    else if (RC_UPPER_LEFT_SW_MID)
    {
        xRobotMode.CtrlMode = RC;
    }
    else if (RC_UPPER_LEFT_SW_DOWN)
    {
        xRobotMode.CtrlMode = LOSE;
    }
    else
        ;
    if (xRobotMode.CtrlMode == RC)
    {
        if (Gimbal_Height_flag == High)
        {
            if (RC_UPPER_RIGHT_SW_UP)
            {
                xRobotMode.ChassisMode = CHASSIS_SEPARA;
            }
            else if (RC_UPPER_RIGHT_SW_MID)
            {
                xRobotMode.ChassisMode = CHASSIS_FOLLOW;
            }
            else if (RC_UPPER_RIGHT_SW_DOWN)
            {
                xRobotMode.ChassisMode = CHASSIS_GYROSC;
            }
            else
                ;
        }
        else
        {
            xRobotMode.ChassisMode = CHASSIS_FOLLOW;
        }
    }
    else if (xRobotMode.CtrlMode == PC)
    {
        if (RC_UPPER_RIGHT_SW_UP)
        {
            xRobotMode.GimbalMode = NOSHOOT_MODE;
        }
        else if (RC_UPPER_RIGHT_SW_MID)
        {
            xRobotMode.GimbalMode = NORMAL_MODE;
        }
        else if (RC_UPPER_RIGHT_SW_DOWN)
        {
            xRobotMode.GimbalMode = UNBOUNCE_MODE;
            xRobotMode.ChassisMode = CHASSIS_SEPARA;
        }
        else
            ;
    }
    else
        ;
}

static float FrictionFeedforwardLimit(float output)
{
    // 前馈叠加在 PID 输出之后，因此必须再次限制最终发送电流。
    if (output > FRICTION_FF_OUTPUT_LIMIT)
    {
        return FRICTION_FF_OUTPUT_LIMIT;
    }
    if (output < -FRICTION_FF_OUTPUT_LIMIT)
    {
        return -FRICTION_FF_OUTPUT_LIMIT;
    }
    return output;
}

static void FrictionFeedforwardCalc(void)
{
    // 左轮为负转，右轮为正转；这里保留反馈转速的原始符号。
    const float speed_l = (float)xMotorParam[FRICTL].MeasureEco->speed_rpm;
    const float speed_r = (float)xMotorParam[FRICTR].MeasureEco->speed_rpm;

    // 禁射、失控或任意一侧目标为零时立即旁路前馈，并清空历史状态。
    if (xRobotMode.CtrlMode == LOSE ||
        xMotorParam[FRICTL].EcoTar == 0.0f ||
        xMotorParam[FRICTR].EcoTar == 0.0f)
    {
        s_friction_last_speed_l = 0.0f;
        s_friction_last_speed_r = 0.0f;
        s_friction_accel_l = 0.0f;
        s_friction_accel_r = 0.0f;
        s_friction_ff_initialized = 0U;
        return;
    }

    // 首次启动只记录当前转速，不计算差分，避免产生启动瞬间的加速度尖峰。
    if (!s_friction_ff_initialized)
    {
        s_friction_last_speed_l = speed_l;
        s_friction_last_speed_r = speed_r;
        s_friction_ff_initialized = 1U;
        return;
    }

    // a = (n_current - n_last) / dt * 2π/60，结果单位为 rad/s²。
    const float raw_accel_l = (speed_l - s_friction_last_speed_l) * RPM_TO_RAD_PER_SEC / FRICTION_FF_DT_S;
    const float raw_accel_r = (speed_r - s_friction_last_speed_r) * RPM_TO_RAD_PER_SEC / FRICTION_FF_DT_S;

    s_friction_last_speed_l = speed_l;
    s_friction_last_speed_r = speed_r;
    // 对差分得到的加速度进行低通滤波，降低编码器量化和反馈抖动的影响。
    s_friction_accel_l += FRICTION_FF_ACCEL_ALPHA * (raw_accel_l - s_friction_accel_l);
    s_friction_accel_r += FRICTION_FF_ACCEL_ALPHA * (raw_accel_r - s_friction_accel_r);

    // 将两轮方向归一化后计算“左轮速度幅值 - 右轮速度幅值”。
    // 左轮更快时 speed_diff > 0，同向叠加可减小左轮负向输出并增大右轮正向输出。
    const float speed_diff = -speed_l - speed_r;
    const float diff_ff = FRICTION_FF_C_DIFF * speed_diff;

    // 总输出 = 速度 PID - 加速度前馈 + 转速差前馈。
    const float output_l = xMotorParam[FRICTL].CurrentOutput -
                           FRICTION_FF_GAIN * FRICTION_FF_C_ACC * s_friction_accel_l + diff_ff;
    const float output_r = xMotorParam[FRICTR].CurrentOutput -
                           FRICTION_FF_GAIN * FRICTION_FF_C_ACC * s_friction_accel_r + diff_ff;

    xMotorParam[FRICTL].CurrentOutput = FrictionFeedforwardLimit(output_l);
    xMotorParam[FRICTR].CurrentOutput = FrictionFeedforwardLimit(output_r);
}

/**********************************************************************************************************************
 * @Func	 void PidCalc(void)
 * @Brief   全电机PID计算
 * @Param	 无
 * @Retval	 无
 * @Note    对全部10个电机(4底盘+2云台+2摩擦轮+1拨弹轮+1高度)依次调用MotorCalc
 *********************************************************************************************************************/
void PidCalc(void)
{
    MotorCalc(&xMotorParam[WHEEL_FR]);
    MotorCalc(&xMotorParam[WHEEL_FL]);
    MotorCalc(&xMotorParam[WHEEL_BR]);
    MotorCalc(&xMotorParam[WHEEL_BL]);
    MotorCalc(&xMotorParam[YAW]);
    MotorCalc(&xMotorParam[PITCH]);
    MotorCalc(&xMotorParam[FRICTR]);
    MotorCalc(&xMotorParam[FRICTL]);
    FrictionFeedforwardCalc();
    MotorCalc(&xMotorParam[PLUCK]);
    MotorCalc(&xMotorParam[Height]);
}

/**********************************************************************************************************************
 * @Func	 void TotalControlSetValue(void)
 * @Brief   统一设置各子系统控制量
 * @Param	 无
 * @Retval	 无
 * @Note    按顺序调用云台初始化→升降控制→云台控制→射击控制→拨弹控制→底盘控制→PID→功率限制
 *********************************************************************************************************************/
void TotalControlSetValue(void)
{
    if (!Gimbal_Init_Flag)
    {
        GimbalInit();
    }
    else if (Gimbal_Init_Flag)
    {
        if ((ABS(CH5) > 500&&xRobotMode.CtrlMode == RC) || highlow_flag != 0)
        {
            LiftLowerControl(); // 云台升降限位控制，只有当遥控器拨杆在中位以上或者高低标志位不为0时才执行升降检测，升降检测导致的误触发问题
        }
        GimbalControlSetValue();
        ShootControlSetValue();
        if (ABS(xMotorParam[FRICTR].MeasureEco->speed_rpm) > 4000 && ABS(xMotorParam[FRICTL].MeasureEco->speed_rpm) > 4000)
        {
            PluckControl(); // 拨弹控制，只有当摩擦轮速度超过4000rpm时才执行拨弹控制，避免在摩擦轮速度较低时拨弹导致的尿弹问题
        }

        ChassisControlSetValue();     // 底盘控制
        ChassisControl_Calculating(); // 底盘控制计算
        // ChassisTargetSpeedLimit(); // 底盘目标速度限制
        PidCalc();                    // PID计算
        PowertotalLimit();            // 功率限制
    }
}

/**********************************************************************************************************************
 * @Func	 void ALL_reset(uint8_t reset)
 * @Brief   系统复位
 * @Param	 uint8_t reset：非零时执行延时后NVIC系统复位
 * @Retval	 无
 * @Note    延时100ms后通过__set_FAULTMASK(1)和NVIC_SystemReset执行复位
 *********************************************************************************************************************/
void ALL_reset(uint8_t reset)
{
    if (reset) // 复位
    {
        HAL_Delay(100);
        __set_FAULTMASK(1);
        HAL_NVIC_SystemReset();
    }
}

/**********************************************************************************************************************
 * @Func	 uint8_t GPIO_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
 * @Brief   带消抖的GPIO读取
 * @Param	 GPIO_TypeDef *GPIOx：GPIO端口
 *          uint16_t GPIO_Pin：GPIO引脚号
 * @Retval	 uint8_t：消抖后的稳定电平(0或1)
 * @Note    连续采样达到阈值后才更新稳定态，有效抑制按键抖动（AI写的函数）
 *********************************************************************************************************************/
uint8_t GPIO_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    // 读取原始电平（未消抖）
    uint8_t raw = (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    gpio_debounce_state_t *state = GPIO_FindOrCreateDebounceState(GPIOx, GPIO_Pin);

    if (state == NULL)
    {
        // 状态池满时回退为原始读值，保证函数可用
        return raw;
    }

    if (raw == state->last_sample)
    {
        // 连续采样一致：计数累加直到阈值
        if (state->sample_count < GPIO_DEBOUNCE_TICKS_EFFECTIVE)
        {
            state->sample_count++;
        }
    }
    else
    {
        // 电平变化：重置计数并开始确认新状态
        state->last_sample = raw;
        state->sample_count = 1U;
    }

    // 达到阈值后才更新稳定态（稳态电平模式）
    if (state->sample_count >= GPIO_DEBOUNCE_TICKS_EFFECTIVE)
    {
        state->stable_state = raw;
    }

    cnt = state->sample_count;
    return state->stable_state;
}

/**********************************************************************************************************************
 * @Func	 void LiftLowerControl(void)
 * @Brief   云台升降限位状态机控制
 * @Param	 无
 * @Retval	 无
 * @Note    三态滞回状态机(High/Middle/Low)，带入态/出态阈值避免边界抖动（AI写的函数）
 *********************************************************************************************************************/
void LiftLowerControl()
{
    static uint8_t high_enter_cnt = 0U;
    static uint8_t low_enter_cnt = 0U;
    static uint8_t high_exit_cnt = 0U;
    static uint8_t low_exit_cnt = 0U;

    // 先读取消抖后的稳定电平
    uint8_t high_hit = GPIO_Read(GPIOC, High_GPIO);
    uint8_t low_hit = GPIO_Read(GPIOC, Low_GPIO);

    // 三态滞回状态机：入态阈值短、出态阈值长，减少边界抖动导致的来回跳变
    switch (Gimbal_Height_flag)
    {
    case High:
        high_enter_cnt = 0U;
        low_enter_cnt = 0U;
        low_exit_cnt = 0U;
        if (high_hit)
        {
            high_exit_cnt = 0U;
        }
        else
        {
            if (high_exit_cnt < HEIGHT_STATE_EXIT_TICKS)
            {
                high_exit_cnt++;
            }
            if (high_exit_cnt >= HEIGHT_STATE_EXIT_TICKS)
            {
                Gimbal_Height_flag = Middle;
                high_exit_cnt = 0U;
            }
        }
        break;

    case Low:
        high_enter_cnt = 0U;
        low_enter_cnt = 0U;
        high_exit_cnt = 0U;
        if (low_hit)
        {
            low_exit_cnt = 0U;
        }
        else
        {
            if (low_exit_cnt < HEIGHT_STATE_EXIT_TICKS)
            {
                low_exit_cnt++;
            }
            if (low_exit_cnt >= HEIGHT_STATE_EXIT_TICKS)
            {
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
        if (high_hit && !low_hit)
        {
            if (high_enter_cnt < HEIGHT_STATE_ENTER_TICKS)
            {
                high_enter_cnt++;
            }
            low_enter_cnt = 0U;
            if (high_enter_cnt >= HEIGHT_STATE_ENTER_TICKS)
            {
                Gimbal_Height_flag = High;
                high_enter_cnt = 0U;
            }
        }
        else if (low_hit && !high_hit)
        {
            if (low_enter_cnt < HEIGHT_STATE_ENTER_TICKS)
            {
                low_enter_cnt++;
            }
            high_enter_cnt = 0U;
            if (low_enter_cnt >= HEIGHT_STATE_ENTER_TICKS)
            {
                Gimbal_Height_flag = Low;
                low_enter_cnt = 0U;
            }
        }
        else
        {
            high_enter_cnt = 0U;
            low_enter_cnt = 0U;
        }
        break;
    }
}
