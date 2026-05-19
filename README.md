# WallE Robot — RoboMaster 步兵机器人主控固件

## 1. 项目概述

本项目是基于 **STM32G474RET6** 微控制器的 RoboMaster 竞赛步兵机器人主控程序，搭载 **FreeRTOS** 实时操作系统，使用 **CubeMX + CLion** 工具链开发（ARM-GCC 编译，CMake 构建）。

机器人具备遥控/PC键鼠双模控制、全向麦轮底盘、双轴云台、摩擦轮发射、超级电容功率管理、MiniPC视觉自瞄等功能。

---

## 2. 硬件平台

| 部件 | 型号/说明 |
|------|-----------|
| 主控芯片 | STM32G474RET6 (Cortex-M4F, 170MHz) |
| CAN总线 | 3路 FDCAN：CAN1（前侧4底盘电机）、CAN2（后侧4电机）、CAN3（云台DM电机+超电） |
| 电机 | DJI M3508 ×6（4底盘+1高度+1摩擦轮左）、DJI M2006 ×2（1拨弹+1摩擦轮右）、DM J4310 ×2（YAW+PITCH云台电机） |
| IMU | 外接 IMU 模块，串口5接收数据 |
| 视觉 | MiniPC 通过 LPUART1 (VT串口) 通信，发送目标位姿、接收自瞄指令 |
| 超电 | 超级电容模块，CAN3/ID=0x108 通信，支持功率缓冲 |
| 遥控器 | DJI 遥控器（串口2）/ PC上位机VT（LPUART1），支持热切换 |
| 裁判系统 | 串口3 接收比赛裁判系统数据（热量、血量、伤害信息等） |
| USB | USB-CDC 虚拟串口，用于调试和固件升级 |

---

## 3. 代码目录结构

```
├── Core/                       # STM32CubeMX 自动生成的HAL外设驱动代码
│   ├── Inc/                    # 外设头文件 (adc.h, fdcan.h, tim.h, usart.h, FreeRTOSConfig.h ...)
│   └── Src/                    # 外设源文件及 main.c, app_freertos.c (FreeRTOS任务创建)
├── Drivers/                    # STM32 HAL库 / CMSIS
├── Middlewares/                # FreeRTOS / USB Device / DSP库
├── IMCA/                       # ★ 自定义应用层代码 (核心)
│   ├── User/                   # 通用定义与工具函数
│   │   ├── user_lib.h/c        # 全局宏/枚举/结构体/工具函数（角度映射、跨圈处理、前后向判断等）
│   ├── Task/                   # 各功能任务模块
│   │   ├── ControlTask.h/c     # 系统总控任务：总初始化、控制模式选择、控制量分派、PID调度
│   │   ├── ChassisControl.h/c  # 底盘控制：运动学解算（麦轮）、跟随/分离/小陀螺三种模式
│   │   ├── GimbalControl.h/c   # 云台控制：YAW/Pitch角度给定、自瞄、升降逻辑
│   │   ├── ShootControl.h/c    # 射击控制：摩擦轮转速、拨弹轮控制、热量约束、堵转保护
│   │   ├── CanBus_Task.h/c     # CAN总线任务：滤波器初始化、电机数据收发、CAN中断回调
│   │   ├── RcTask.h/c          # 遥控器/VT数据解析与按键去抖
│   │   ├── UartTask.h/c        # 串口空闲中断管理
│   │   ├── IMU_Task.h/c        # IMU数据更新任务
│   │   ├── imu_data_decode.h/c # IMU协议解析
│   │   └── packet.h/c          # 数据包封装/解包
│   ├── Tool/                   # 算法与工具库
│   │   ├── Motor.h/c           # 电机模型：参数定义、PID计算调度、uint/float互转
│   │   ├── pid.h/c             # PID控制器（位置式，含积分分离、限幅）
│   │   ├── PowerLimit.h/c      # 底盘功率限制：功率模型+滑动平均+PID限幅
│   │   ├── CapInteract.h/c     # 超级电容通信：状态读取、模式设置
│   │   ├── Vision_interact.h/c # MiniPC视觉通信：发送机器人位姿、接收自瞄指令
│   │   ├── kalman_filter.h/c   # 卡尔曼滤波器
│   │   ├── Ramp.h/c            # 斜坡/低通滤波器
│   │   ├── Vofa_send.h/c       # Vofa+ 调试上位机数据发送
│   │   ├── adc_handle.h/c      # ADC 电池电压采样处理
│   │   └── INS_task.h/c        # 惯导姿态解算
│   └── Judge/                  # 裁判系统交互与UI
│       ├── Judge_interact.h/c  # 裁判系统数据解析（血量、热量、伤害ID等）
│       ├── Driver_Judge.h/c    # 裁判系统底层驱动
│       ├── UI.h/c / UI_STRUCT.h # 裁判系统串口屏UI绘制
│       └── ui_picTask.c        # UI图片任务
├── USB_Device/                 # USB-CDC 虚拟串口
├── CMakeLists.txt              # CMake 构建配置
└── README.md                   # 本文件
```

---

## 4. FreeRTOS 任务架构

系统启动后创建 **5个FreeRTOS线程**（定义于 [app_freertos.c](Core/Src/app_freertos.c)）：

| 任务名 | 函数 | 功能 | 周期 |
|--------|------|------|------|
| **Judge_Task** | `JudgeTask()` | USB初始化 + MiniPC视觉帧发送 | ~1ms |
| **Init_Task** | `InitTask()` | 外设总初始化（CAN滤波、UART、电机参数、PID参数），完成后自删除 | 仅上电一次 |
| **Ctrl_Task** | `CtrlTask()` | 主控制循环：IMU更新→模式选择→滤波→消抖→控制量给定 | ~1ms |
| **Interact_Task** | `InteractTask()` | CAN总线电流发送 + 超电功率发送 | ~1ms |
| **Reset_Task** | `ResetTask()` | 看门狗监控 + UI刷新 + 组合键复位 | ~1ms |

**主控制循环执行流程**（`TotalControl()`）：
```
IMU数据更新 → 控制模式选择 → 鼠标一阶滤波 → 按键消抖 → TotalControlSetValue()
  └─ GimbalInit（如未初始化）
  └─ LiftLowerControl（升降限位）
  └─ GimbalControlSetValue（云台角度给定）
  └─ ShootControlSetValue（摩擦轮+拨弹）
  └─ PluckControl（拨弹逻辑）
  └─ ChassisControlSetValue（底盘速度给定）
  └─ ChassisControl_Calculating（麦轮运动学解算）
  └─ PidCalc（10个电机PID计算）
  └─ PowertotalLimit（功率限制）
```

---

## 5. 电机系统

### 5.1 电机列表（定义于 [user_lib.h](IMCA/User/user_lib.h)）

| 编号 | 名称 | 类型 | CAN总线 | CAN ID | 反馈 | 功能 |
|------|------|------|---------|--------|------|------|
| 0 | WHEEL_FR | DJI M3508 | CAN1 | 0x201 | 编码器 | 前右轮 |
| 1 | WHEEL_FL | DJI M3508 | CAN1 | 0x202 | 编码器 | 前左轮 |
| 2 | WHEEL_BR | DJI M3508 | CAN1 | 0x203 | 编码器 | 后右轮 |
| 3 | WHEEL_BL | DJI M3508 | CAN1 | 0x204 | 编码器 | 后左轮 |
| 4 | YAW | DM J4310 | CAN3 | 0x01 | DM编码器+IMU | 云台偏航 |
| 5 | PITCH | DM J4310 | CAN3 | 0x02 | DM编码器+IMU | 云台俯仰 |
| 6 | Height | DJI M3508 | CAN2 | 0x205 | 编码器 | 升降机构 |
| 7 | FRICTR | DJI M2006 | CAN2 | 0x206 | 编码器 | 摩擦轮右 |
| 8 | FRICTL | DJI M3508 | CAN2 | 0x207 | 编码器 | 摩擦轮左 |
| 9 | PLUCK | DJI M2006 | CAN2 | 0x208 | 编码器 | 拨弹轮 |

### 5.2 电机控制模式

- **反馈模式**：`EcoFeedback`（编码器反馈） / `ImuFeedback`（IMU姿态反馈）
- **闭环模式**：`PosClsloop`（位置环） / `SpdClsloop`（速度环）
- **PID结构**：位置-速度串级PID（[pid.h](IMCA/Tool/pid.h)）

### 5.3 DM电机 MIT 模式协议

DM J4310 使用 MIT 控制模式，通过 `set_DM_J4310_MIT_info()` 发送位置/速度/KP/KD/扭矩五个浮点参数，经 `float_to_uint()` 按位宽打包到8字节CAN帧中。

---

## 6. 控制模式

### 6.1 控制来源（[ControlTask.c](IMCA/Task/ControlTask.c) — `CtrlModeSelect()`）

| 模式 | 触发条件 | 说明 |
|------|----------|------|
| **RC** | 遥控器左上拨杆 上 | DJI遥控器控制 |
| **PC** | 遥控器左上拨杆 中 | PC键鼠 + VT上位机控制 |
| **LOSE** | 遥控器左上拨杆 下 或 遥控器信号丢失 | 失能所有电机，安全状态 |

### 6.2 底盘模式（[ChassisControl.c](IMCA/Task/ChassisControl.c)）

| 模式 | 说明 |
|------|------|
| **CHASSIS_FOLLOW** | 跟随模式：底盘方向跟随云台朝向（默认） |
| **CHASSIS_SEPARA** | 分离模式：底盘独立运动，不跟随云台 |
| **CHASSIS_GYROSC** | 小陀螺模式：底盘持续自转，带堵转反转检测 |

### 6.3 云台模式

| 模式 | 说明 |
|------|------|
| **NORMAL_MODE** | 正常模式：可瞄准可射击 |
| **UNBOUNCE_MODE** | 无弹模式：仅瞄准不射击 |
| **NOSHOOT_MODE** | 禁射模式：关闭摩擦轮 |

---

## 7. 功率管理系统

### 7.1 功率限制（[PowerLimit.c](IMCA/Tool/PowerLimit.c)）

- 使用四个电机的 **功率模型**（`model[4]`）：$P = k_1 \cdot I \cdot \omega + k_2 \cdot \omega^2 + k_3 \cdot I^2 + a/4$
- 实际电流经 20点滑动平均滤波后代入模型
- 用 PID 控制与裁判系统功率上限比较，输出系数（0~1）等比例缩放四个底盘电机电流

### 7.2 超级电容（[CapInteract.c](IMCA/Tool/CapInteract.c)）

- CAN3/ID=0x108 通信
- 电容电压 ≥ 16V 且状态正常时，额外增加功率缓冲
- 支持设置缓冲模式（保守/均衡/激进）

---

## 8. 射击系统

### 8.1 摩擦轮控制（[ShootControl.c](IMCA/Task/ShootControl.c)）

- 两摩擦轮对转，目标转速由裁判系统热量限制动态计算
- PC模式下仅高位开启摩擦轮，RC模式下关闭

### 8.2 拨弹轮控制

- **单发**：鼠标左键单击（hold_cnt: 10~300ms）
- **连发**：鼠标左键长按（hold_cnt > 300ms）
- **热量保护**：根据裁判系统热量数据动态调整射速，热量过高时停止发射
- **堵转保护**：电流 > 8000mA 持续超时自动反转清除卡弹

---

## 9. 云台升降限位状态机

升降机构通过两个 GPIO 限位开关检测High/Low位置（[ControlTask.c](IMCA/Task/ControlTask.c) — `LiftLowerControl()`）：

```
         ┌─────────┐
    ┌───→│  High   │←───┐
    │    └────┬────┘    │
    │  限位释放(num次)    │限位触发(num次)
    │    ┌────┴────┐    │
    └────│ Middle  │────┘
         └────┬────┘
        限位触发│限位释放
         ┌────┴────┐
         │   Low   │
         └─────────┘
```

- **进入阈值**（`HEIGHT_STATE_ENTER_TICKS = 10`）：Middle→High/Low 需连续触发10次
- **退出阈值**（`HEIGHT_STATE_EXIT_TICKS = 10`）：High/Low→Middle 需连续释放10次
- 两个限位同时有效时保持Middle，避免异常接线误判

---

## 10. 通信协议

### 10.1 MiniPC 视觉通信（[Vision_interact.h](IMCA/Tool/Vision_interact.h)）

**发送（机器人→MiniPC，32字节）：**
- 四元数姿态 q[4]、Yaw角度、Yaw角速度、Pitch角度、颜色信息

**接收（MiniPC→机器人，27字节）：**
- 瞄准目标 Yaw/Pitch 角度及角速度/角加速度、开火标志位

### 10.2 裁判系统（串口3, [Judge_interact.h](IMCA/Judge/Judge_interact.h)）

- 接收：血量、热量上限/当前/冷却、伤害来源ID/类型、比赛阶段等
- 发送：UI 绘制指令到裁判系统操作手界面

### 10.3 VT 上位机（LPUART1）

PC模式下的虚拟遥控器协议，结构与RC遥控器一致，键鼠数据通过串口发送到机器人。

---

## 11. 关键安全机制

| 机制 | 位置 | 说明 |
|------|------|------|
| 遥控器丢信号 | `MotorWatchdogMonitor()` | RC超时自动切为LOSE模式，停止所有电机 |
| 电机超时检测 | `GetMotorMeasureNew()` — `Timeout_Flag` | 每收到一帧CAN数据递增，看门狗定期清零并检测 |
| DM电机堵转检测 | `ChassisControlSetValue()` | YAW扭矩 > 9 持续 400ms 触发小陀螺反转 |
| 升降堵转保护 | `GimbalControlSetValue()` | Height电流 > 8000mA 持续 50ms 停止升降 |
| 拨弹堵转保护 | `PluckControl()` | 拨弹电流 > 8000mA 持续 200ms 反转清除 |
| 摩擦轮低转速禁射 | `TotalControlSetValue()` | 摩擦轮 < 4000rpm 时禁止拨弹（防止尿弹） |
| 组合键复位 | `ResetTask()` | Ctrl+R 触发 NVIC 系统复位 |
| 裁判系统复活检测 | `CtrlModeSelect()` | 检测 RoboHP 从0变1做复活后延时处理 |

---

## 12. 调试工具

- **Vofa+**：[Vofa_send.c](IMCA/Tool/Vofa_send.c) 通过串口发送调试波形数据到 Vofa+ 上位机显示
- **USB-CDC**：虚拟串口用于日志输出和参数调试
- **裁判系统UI**：[UI.c](IMCA/Judge/UI.c) 在比赛操作界面上绘制自定义UI元素

---

## 13. 构建与开发

### 环境要求
- **IDE**: CLion 或 VSCode + CMake
- **编译器**: arm-none-eabi-gcc (ARM GNU Toolchain)
- **构建系统**: CMake 3.28+
- **HAL配置**: STM32CubeMX (.ioc 文件: `G474_Example.ioc`)


---

## 14. 配置宏说明

全局配置宏定义在 [user_lib.h](IMCA/User/user_lib.h)：

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `IMU_UPDATA` | 1 | 是否启用 IMU 姿态更新 |
| `INFANTRY` | 1 | 兵种：步兵 |
| `CHASSIS_MAXSPEED` | 动态 | 底盘最大速度，功率35W时限速3500rpm，否则6000rpm |
| `GPIO_DEBOUNCE_TICKS` | 10 | 按键消抖采样周期数 |
| `HEIGHT_STATE_ENTER_TICKS` | 10 | 限位进入阈值 |
| `HEIGHT_STATE_EXIT_TICKS` | 10 | 限位退出阈值 |
