# 项目架构总览（中文）

本文档面向 STM32G474 半舵机云台项目，说明代码整体框架：系统如何在 FreeRTOS 下组织、初始化与运行；主要模块、线程、驱动、数据流与可扩展点，帮助快速理解与上手。


## 顶层结构
- CubeMX 生成的 HAL + CMSIS + FreeRTOS 位于 `Core/` 和 `Drivers/`。
- 业务逻辑集中在 `IMCA/` 下，分三层：
  - IMCA/Task：应用任务与高层控制（云台、底盘、射击、CAN、UART、遥控器、IMU）。
  - IMCA/Tool：可复用算法/工具（PID、卡尔曼、INS、Motor 抽象、VOFA 发送、视觉交互）。
  - IMCA/User：项目级工具函数与封装。
- 中间件与第三方源码在 `Middlewares/Third_Party`（FreeRTOS、ARM CMSIS DSP 等）。
- 构建系统为 CMake；顶层 `CMakeLists.txt`、`CMakePresets.json` 管理目标与预设。


## 启动与初始化路径
- 入口：`Core/Src/main.c`
  - HAL_Init -> SystemClock_Config -> 外设初始化（GPIO、DMA、FDCAN1/2/3、UART4/5、USART1/2/3、TIM4）。
  - osKernelInitialize -> `MX_FREERTOS_Init()` -> osKernelStart（启动调度器）。
- `Core/Src/app_freertos.c` 中的 `MX_FREERTOS_Init` 创建线程：
  - Judge_Task：初始化 USB 设备，并向上位机发送视觉信息。
  - Init_Task：执行 `TotalInitTask` 完成系统初始化后自结束。
  - Ctrl_Task：执行 `TotalControl`（主控制流水线）。
  - Interact_Task：发送 CAN 帧与电机电流，是外设交互与状态上报的通道。
  - Reset_Task：可选的系统复位逻辑（`ALL_reset`）。


## FreeRTOS 线程职责
- Judge_Task（裁判/上位交互）
  - 调用 `MX_USB_Device_Init()` 初始化 USB 设备栈（CDC/MSC 视 CubeMX 配置）。
  - 周期性调用 `MiniPC_SendVisionFrameInfo(&MiniPC_SendPacket)`（见 `Vision_interact`）发送视觉帧信息。
- Init_Task（初始化）
  - 调用 `TotalInitTask()`（`IMCA/Task/ControlTask.c`）：
    - 配置 CAN 过滤（`CANFilterInit`）。
    - 设置默认控制模式（LOSE）。
    - 使能 IMU（UART5）与遥控器（USART2）空闲中断 DMA。
    - 初始化电机参数（`MotorInit`）与 PID 参数（`PidInit`）。
  - 延时后终止自身（一次性任务）。
- Ctrl_Task（主控制）
  - 周期调用 `TotalControl()`：
    -（可选）更新 IMU 姿态（yaw/pitch/roll），并绑定到电机参数。
    - 基于遥控器拨杆选择控制模式（`CtrlModeSelect`）。
    - 进行去抖/去抖动处理（`deshake_all`），计算目标量（`TotalControlSetValue`）。
    - 通过 `Vofa_Send_Data16(...)` 发送诊断数据。
    - 对各执行机构进行 PID 计算（`PidCalc` -> `MotorCalc`）。
- Interact_Task（交互）
  - 通过 `MotorCanSendCurrent()` 下发电机电流。
  - 通过 `send_can2can(...)`（FDCAN2）汇总并广播云台状态/视觉/射击标志。
- Reset_Task（复位）
  - 调用 `ALL_reset(0)`；当入参设为 1 时，延时后进行 NVIC 系统复位。


## 控制流水线概览
- `ControlTask.c` 是核心调度者：
  - `CtrlModeSelect`：读取遥控器拨杆确定模式：
    - 左上拨杆选择 CtrlMode：PC、RC、LOSE。
    - RC 模式下，右上拨杆选择底盘模式（分离/跟随/陀螺）。
    - PC 模式下，右上拨杆选择云台模式（不射击/常规/去弹跳），并可能强制底盘分离。
  - `TotalControlSetValue`：依据 CtrlMode 与 GimbalMode 计算 PITCH/YAW 的期望角度/速度：
    - LOSE：失控/失联时，重置角度保存标志，待恢复后继承。
    - RC：将遥控器通道（ch3，ch4）映射为 yaw/pitch 设定值。
    - PC：
      - 鼠标右键或 ch5 触发进入 ImuFeedback，并在 `fireflag != 0` 时使用 MiniPC 的视觉目标（yaw/pitch/vel/acc）。
      - 否则，PITCH 切回 EcoFeedback，根据鼠标增量调整；YAW 使用总角度跟踪并按鼠标 X 调整。
    - 对 PITCH 施加机械/安全角度限制与零点死区处理。
  - `PidCalc`：对每个电机调用 `MotorCalc`，在编码器（Eco）或 IMU 反馈模式间切换并进行 PID 计算。
  - `GimbalInit`：等待 PITCH 与 YAW 回零且 DM 驱动上报初始化成功；然后设置 IMU 角偏移，启用总角度模式、方向标志，并置位 `Gimbal_Init_Flag`。


## 电机抽象（`IMCA/Tool/Motor.c`）
- `motor_param_t xMotorParam[MOTOR_NUM]` 描述执行器：
  - CAN ID、名称、所在总线、类型（DJI/DM）、反馈模式（Eco/IMU）、闭环类型（位置/速度）、测量链路（编码器/DM/IMU）、PID 参数指针等。
- 初始化映射：
  - FRICTR（右摩擦轮）：CAN1，DJI，速度环。
  - FRICTL（左摩擦轮）：CAN1，DJI，速度环。
  - PITCH：CAN3，DM，位置环，使用 IMU + DM 测量。
  - YAW：CAN2，DM，位置环，使用 IMU + DM 测量。
- `MotorCalc` 分派：
  - `PidCalcEco`：基于编码器/DM 的位置/速度（取决于 TotalAngleState 与 CloseLoopMode）进行 PID。
  - `PidCalcIMU`：基于 IMU 角或总角进行 PID；可按方向反相输出。
- 提供 MIT 模式 DM 驱动的浮点/整数比例映射辅助函数。


## 传感与数据来源
- IMU（`IMCA/Task/IMU_Task.*`）：提供角度、角速度、总角度；`Imu_param_update` 将 IMU 读数更新到 `IMU_Yaw/Pitch/Roll` 并通过 `MotorImuUpdata` 绑定到电机。
- 遥控器（`IMCA/Task/RcTask.*`）：读取通道、拨杆状态与鼠标增量；宏如 `RC_UPPER_LEFT_SW_UP` 表征拨杆位置。
- 视觉（`IMCA/Tool/Vision_interact.*`）：与 MiniPC 通讯；`MiniPC_ReceivePacket` 包含 yaw/pitch 目标、速度、加速度与 `fireflag`。
- 编码器/DM（`IMCA/Tool/Motor.h` + FDCAN）：DM_J4310 驱动测量（位置、速度、初始化标志），DJI 编码器测量等。


## 通讯
- CAN（FDCAN1/2/3）：
  - 过滤在 `CANFilterInit` 设置。
  - 下发接口：`MotorCanSendCurrent()`（电机电流）；`send_can2can(...)`（状态广播）。
- UART/USART：
  - UART5：IMU 数据，使用空闲中断 DMA（`HAL_UARTEx_ReceiveToIdle_DMA_NoIT(&huart5)`）。
  - USART2：遥控器接收，空闲中断 DMA。
  - 其他串口（UART4、USART1/3）可按需扩展。
- USB 设备：
  - 在 Judge_Task 中 `MX_USB_Device_Init()` 初始化。
  - 通过 `MiniPC_SendVisionFrameInfo` 将信息发送到 MiniPC。
- VOFA 可视化（`IMCA/Tool/Vofa_send.*`）：
  - `Vofa_Send_Data2/4/8/16` 将浮点通道打包并附加固定帧尾，便于上位机可视化。
  - 在 `TotalControl` 中用于诊断（位置、输出、IMU 角、模式等）。


## 模式与状态标志
- `xRobotMode` 保存 CtrlMode（PC/RC/LOSE）、GimbalMode（NOSHOOT/NORMAL/UNBOUNCE）、ChassisMode（SEPARA/FOLLOW/GYROSC）。
- `Gimbal_Init_Flag` 表示云台初始化完成（DM 驱动归零成功且角度对齐）。
- `AUTO_Shoot_flag`、`mouse_right.deshake_key`、`MiniPC_ReceivePacket.fireflag` 协同自动瞄准/射击行为。
- `TotalAngleState` 决定 IMU/编码器 PID 使用瞬时角还是累计总角。


## 关键数据流
- 初始化 -> 控制主循环：
  1. Init_Task 配置总线、DMA、过滤器与电机/PID 参数。
  2. Ctrl_Task 更新传感器（IMU、遥控、视觉），选择模式，计算设定值。
  3. 对各电机运行 PID，得到 `CurrentOutput`。
  4. Interact_Task 通过 CAN 下发电流并向上位机/其他设备报告状态。
- 视觉闭环：
  - MiniPC 下发 yaw/pitch 目标与动态信息，系统映射到 IMU 目标并进行 PID；状态通过 USB/VOFA 回传。


## 安全、限幅与复位
- 在 `GimbalControlSetValue` 中对 PITCH 进行角度限幅，避免机械越界。
- 近零区间设置死区，避免微漂。
- `ALL_reset(reset)` 在入参非零时执行延时后系统复位。


## 扩展与定制建议
- 在 `app_freertos.c` 新增线程并合理设置优先级与栈空间。
- 在 `MotorInit` 扩展电机清单，绑定测量/反馈与 PID 参数。
- 在 `ChassisControl.c`、`GimbalControl.c`、`ShootControl.c` 中实现更多策略。
- 接入新传感器：配置 DMA/中断与解码任务（参考 `IMU_Task.c`、`imu_data_decode.c`）。
- 增强诊断：通过 VOFA 或 USB CDC 打印；在调用处增减 `Vofa_Send_Data16` 通道。


## 构建与运行
- 使用 CMake 构建（如 CLion 或 CubeIDE）。项目包含 `cmake/gcc-arm-none-eabi.cmake` 以支持交叉编译与 CubeMX CMake 集成。
- 外设初始化由 CubeMX 生成；需确保 `Drivers/` 中包含 STM32Cube 固件包。
- 通过 ST-Link 等工具将固件烧录到 STM32G474。


## 文件导览（精选）
- Core/
  - Src/main.c：入口、时钟、HAL 初始化、RTOS 启动。
  - Src/app_freertos.c：线程创建与线程体。
- IMCA/Task/
  - ControlTask.c：初始化、主控制循环、PID 调用、VOFA 诊断。
  - GimbalControl.c：模式相关设定值计算、云台初始化判定、鼠标/视觉混控。
  - CanBus_Task.c/h：CAN 过滤、发送助手、电流打包。
  - RcTask.c/h：遥控器解析、拨杆/鼠标输入。
  - IMU_Task.c/h、imu_data_decode.c/h：IMU 处理与绑定。
  - ShootControl.c/h、ChassisControl.c/h：执行器策略。
  - UartTask.c/h：UART 处理助手。
- IMCA/Tool/
  - Motor.c/h：电机抽象、PID 选择、DM/DJI 处理。
  - pid.c/h：PID 实现。
  - kalman_filter.c/h、INS_task.c/h：滤波与惯导支持。
  - Vision_interact.c/h：MiniPC 包定义、收发。
  - Vofa_send.c/h：可视化通道发送。
  - Ramp.c/h：斜坡生成器。
- Drivers/：CMSIS、HAL 驱动。


## 常见排查
- IMU 无数据：检查 UART5 空闲 DMA 与 `IMU_UPDATA` 宏。
- 遥控无输入：确认 USART2 空闲 DMA 与 `RcTask` 解析是否正常。
- 电机不转：校验 FDCAN 过滤、ID 与 `MotorCanSendCurrent` 载荷；确认 DM_J4310 初始化标志。
- 云台一直未完成初始化：确认 PITCH 接近零位且 DM 驱动上报 `DM_J4310_InitFlag`。
- USB 视觉数据未输出：检查 Judge_Task 运行、USB 设备枚举、`MiniPC_SendPacket` 是否更新。


## 术语表
- EcoFeedback：来自编码器/DM 的反馈。
- ImuFeedback：来自 IMU 的反馈。
- PosClsloop/SpdClsloop：位置/速度闭环控制。
- TotalAngleState：控制中使用瞬时角或累计总角的状态标志。


---

英文版说明见本文档上方原始内容；如需补充模块关系图或在 README 中加入跳转链接到具体函数，请告诉我，我可以继续完善。
