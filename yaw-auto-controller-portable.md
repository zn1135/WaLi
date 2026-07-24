# Yaw 自瞄控制器（可移植版）

一套用于 RM 云台 yaw 轴自瞄的**力矩域**控制方案：系统辨识 → 逆动力学前馈 + 力矩域 PD → ESO 扰动补偿，末端幅值/斜坡限幅。DM 电机走 MIT 模式**纯力矩**输出（pos/vel/KP/KD 传 0）。

换台机器人复现步骤：① 重新辨识 J/B/Fc → ② 贴三个函数 → ③ 按调参流程整定。全程台架、举升、不装弹。

---

## 0. 适用前提

- **执行器**：DM 系列直驱电机（如 J4310），支持 MIT 模式力矩控制。
- **反馈**：机体 IMU 提供 yaw 绝对角度 + 角速度（不用电机编码器内环）。
- **上位机**：视觉输出目标 `yaw / yaw_vel / yaw_acc`（位置/速度/加速度参考）。
- **控制周期**：本例约 6ms（InteractTask, `osDelay(1)` 有抖动），用 DWT 测真实 dt。

若你的执行器是减速箱 + 电流环电机，力矩域方案同样适用，但需把力矩命令换算成电流，且辨识参数含减速比。

---

## 1. 系统辨识（换机器人必做）

### 1.1 动力学模型

```
tau = J·θ̈ + B·θ̇ + Fc·sign(θ̇) + Tau0
```

| 符号 | 含义 | 本例值 | 单位 |
|------|------|--------|------|
| J | 转动惯量 | 0.020301 | kg·m² |
| B | 粘滞阻尼 | 0.057971 | N·m·s/rad |
| Fc | 库伦摩擦 | 0.218419 | N·m |
| Tau0 | 力矩偏置 | -0.002998 | N·m（前馈不用，交给 ESO） |

### 1.2 采集与拟合

1. **激励**：让 yaw 轴跑覆盖速度范围的轨迹（变速正反转、扫频/正弦），记录 `力矩命令 τ`、`角度 θ`、`角速度 θ̇`。
2. **采样周期**：务必记录**真实 dt**（本例 DWT 时钟，约 7.1ms，**不是名义 1ms**）。加速度由角速度对真实时间戳数值微分，别用固定 dt。
3. **VOFA 导出 CSV** → MATLAB 最小二乘拟合 `[θ̈, θ̇, sign(θ̇), 1] · [J, B, Fc, Tau0]ᵀ = τ`。
4. **验证**：用辨识参数正向仿真力矩，与实测力矩对比，残差应小且无系统性偏差。

> ⚠️ 采样周期踩坑：辨识 CSV 的真实 dt 必须用时间戳，名义周期会让加速度算错、J 拟歪。

---

## 2. 控制架构总览

```
视觉参考(θr, ωr, αr)  ┐
IMU 实测(θ, ω)         ┤
上一拍力矩 τ_last      ┘
        │
        ▼
┌─────────────────────────────────────────────┐
│ YawMitTorque()  (基础层，力矩域)              │
│                                               │
│  τ_ff  = J·αr + B·ωr + Fc·tanh(ωr/smooth)     │ 逆动力学前馈(用参考量)
│  τ_pd  = Kθ·(θr−θ) + Kω·(ωr−ω)                │ 力矩域 PD(用实测纠偏)
│  τ_eso = ESO 扰动补偿                          │ 可关，见第4节
│  τ = τ_ff + τ_pd + τ_eso                      │
└─────────────────────────────────────────────┘
        │
        ▼
  abs_limit(±T_MAX)  →  YawTorqueSlewLimit(斜坡)  →  DM MIT 纯力矩
```

**分层原则**：基础层（前馈+PD）自成一体、可独立运行；ESO 是加在其上的补偿层，可编译期开关。末端限幅统一在发送处，控制器内部不重复限。

---

## 3. 基础层：逆动力学前馈 + 力矩域 PD

### 3.1 为什么是"力矩域" PD

常见错误：把力矩域增益套进加速度域公式（`acc_r = acc_ff + K·err`，再乘 J）。这会让等效反馈被 J（≈0.02）缩小约 50 倍，PD 形同虚设。

正确做法：**PD 直接输出力矩**，与前馈力矩相加。Kθ/Kω 是力矩域量纲：
- Kθ：N·m/rad
- Kω：N·m/(rad/s)

### 3.2 库伦项用 tanh 平滑

硬 `sign(ω)` 在零速处有 ±Fc 的力矩台阶（本例 ±0.218，约量程 2%），换向/低速会"顿"。改 `Fc·tanh(ω/smooth)`：零速附近连续过渡，`smooth`（本例 0.15 rad/s）越小越接近 sign。

### 3.3 前馈用参考量、反馈用实测量

- 前馈（J·α+B·ω+Fc）用**规划参考** ωr/αr —— 靠模型预测。
- PD 用**实测** θ/ω 纠偏。
- 这是标准计算力矩控制（computed torque control）的姿势。

### 3.4 参数

| 宏 | 本例值 | 说明 |
|----|--------|------|
| `LQR_AUTO_K_THETA_DEFAULT` | 15.81 | 力矩域角度增益 Kθ |
| `LQR_AUTO_K_OMEGA_DEFAULT` | 1.568 | 力矩域角速度增益 Kω |
| `GIMBAL_YAW_COULOMB_SMOOTH` | 0.15 | 库伦零速平滑带 (rad/s) |

> 增益可由 LQR 设计（对 [θ,ω] 双积分模型配 Q/R 解 Riccati）得到初值，也可手动整定。本例为手动调定。

### 3.5 代码

```c
// 前馈：逆动力学，用参考量
float YawDynamicsFeedforward(float accRef, float velRef)
{
    float coulomb = GIMBAL_YAW_FC * tanhf(velRef / GIMBAL_YAW_COULOMB_SMOOTH);
    return GIMBAL_YAW_J*accRef + GIMBAL_YAW_B*velRef + coulomb;
}

// 基础层总力矩：前馈 + 力矩域 PD (+ ESO，见第4节)
float YawMitTorque(float posTar, float velTar, float posMea, float velMea, float accFf)
{
    float posErr = posTar - posMea;
    float velErr = velTar - velMea;
    float ff = YawDynamicsFeedforward(accFf, velTar);
    float pd = LQR_AUTO_K_THETA_DEFAULT*posErr + LQR_AUTO_K_OMEGA_DEFAULT*velErr;
    return ff + pd;   // ESO 补偿在此加，见第4节
}
```

---

## 4. ESO 扰动补偿层（可选，编译期开关）

### 4.1 作用

把所有**没建模/辨不准**的力矩——库伦残差、Tau0、装配不水平的重力分量、J/B 误差、外部扰动——统一当成一个"总扰动"，用三阶扩张状态观测器（ESO）在线估出来，反向补掉。相当于给辨识残差兜底。

### 4.2 观测器方程

模型 `J·θ̈ = τ − B·θ̇ + d`，令 `b0 = 1/J`，扩张态 `z3` 估 `d/J`：

```
e   = θ_meas − z1
ż1  = z2 + 3w0·e
ż2  = −(B/J)·z2 + b0·τ_applied + z3 + 3w0²·e
ż3  = w0³·e            // 观测器增益 beta = [3w0, 3w0², w0³]
```

前向欧拉离散（`z += dt·ż`）。z1≈角度、z2≈角速度、z3≈扰动/J。

补偿力矩：`τ_eso = −gain · z3 · J`（z3 乘 J 还原扰动力矩，取负抵消）。

### 4.3 关键工程处理（踩过的坑）

| 处理 | 为什么 |
|------|--------|
| **观测器输入用"施加后"的力矩** `τ_last` | 喂给 ESO 的必须是限幅+斜坡后**真正发出去**的力矩，不是原始算出值，否则观测器看到的和电机实际不一致。 |
| **补偿输出一阶低通** | ESO 补的是慢扰动（几 Hz 以下），高频全是 IMU 噪声被 w0³ 放大来的。**低通是抑抖主力**，优先于降带宽。 |
| **首帧只播种** | 首拍把 z1 对齐实测、z2/z3 清零，不补偿，避免初始大误差冲击。 |
| **退出自瞄复位** | 切出自瞄清零观测器 + 低通 + 斜坡历史，防重进时从陈旧状态起跳。 |
| **dt 一拍采一次** | ESO 和斜坡限幅共用同一个 dt（DWT 测），同拍只采一次存共享变量，否则第二次读 DWT 拿到≈0。 |
| **异常 dt 剔除** | dt 过小或过大（久未进自瞄后 DWT 回绕）回退默认值，防积分爆掉。 |

### 4.4 噪声放大的本质（务必理解）

`z3 ∝ w0³`。带宽 w0 从 8→12，噪声增益涨约 3.4 倍。**w0=12 本例实测就抖**。所以：

- **不要靠硬顶带宽**去追扰动 —— 噪声先炸。
- **降带宽/降增益是治标**：整体补偿变弱，慢扰动也一起牺牲。
- **低通是治本**：只砍高频噪声，慢扰动原样留住。抖了**优先降 `COMP_LPF_HZ`**。

### 4.5 参数

| 宏 | 本例值 | 说明 |
|----|--------|------|
| `GIMBAL_YAW_ESO_ENABLE` | 0 | 编译期开关。0=纯前馈+PD |
| `GIMBAL_YAW_ESO_BANDWIDTH` | 5.0 | 观测器带宽 w0 (rad/s)。抖了别往上顶 |
| `GIMBAL_YAW_ESO_COMP_GAIN` | 0.1 | 补偿强度 0~1，保守起步 |
| `GIMBAL_YAW_ESO_COMP_LIMIT` | 0.5 | 补偿力矩上限 (N·m)，防发散顶飞 |
| `GIMBAL_YAW_ESO_COMP_LPF_HZ` | 5.0 | 补偿低通截止 (Hz)，**抑抖主旋钮** |

### 4.6 代码

```c
// 观测器一拍推进
static void YawEsoUpdate(float thetaMea, float tauApplied, float dt)
{
    const float w0 = GIMBAL_YAW_ESO_BANDWIDTH;
    const float b0 = 1.0f / GIMBAL_YAW_J;
    const float damp = GIMBAL_YAW_B / GIMBAL_YAW_J;
    if (!yawEsoReady) { yawEsoZ1 = thetaMea; yawEsoZ2 = yawEsoZ3 = 0; yawEsoReady = 1; return; }
    float e = thetaMea - yawEsoZ1;
    yawEsoZ1 += dt * (yawEsoZ2 + 3*w0*e);
    yawEsoZ2 += dt * (-damp*yawEsoZ2 + b0*tauApplied + yawEsoZ3 + 3*w0*w0*e);
    yawEsoZ3 += dt * (w0*w0*w0*e);
}

// 补偿力矩：还原 → 低通 → 限幅
static float YawEsoCompensation(float dt)
{
    float raw = -GIMBAL_YAW_ESO_COMP_GAIN * yawEsoZ3 * GIMBAL_YAW_J;
    float rc = 1.0f / (2.0f*PI*GIMBAL_YAW_ESO_COMP_LPF_HZ);
    float alpha = dt / (dt + rc);
    yawEsoCompFilt += alpha * (raw - yawEsoCompFilt);   // 一阶低通
    float c = yawEsoCompFilt;
    if (c >  GIMBAL_YAW_ESO_COMP_LIMIT) c =  GIMBAL_YAW_ESO_COMP_LIMIT;
    if (c < -GIMBAL_YAW_ESO_COMP_LIMIT) c = -GIMBAL_YAW_ESO_COMP_LIMIT;
    return c;
}
```

在 `YawMitTorque` 里：`#if GIMBAL_YAW_ESO_ENABLE` 时 `return ff + pd + YawEsoCompensation(dt);`

---

## 5. 末端限幅（发送处统一施加）

顺序：**幅值限幅 → 斜坡限幅 → 发电机**。

```c
float tau = YawMitTorque(...);
abs_limit(&tau, T_MAX);              // 幅值：|τ| ≤ T_MAX
tau = YawTorqueSlewLimit(tau);       // 斜坡：每拍变化 ≤ RATE·dt
set_DM_MIT(..., 0,0,0,0, tau);       // MIT 纯力矩，pos/vel/KP/KD 传 0
```

- **斜坡限幅** `GIMBAL_YAW_TORQUE_SLEW_RATE`（本例 500 N·m/s）：约束每拍力矩变化，抑制自瞄锁定瞬间的力矩 snap。正常跟踪碰不到，只削满量程级突变。
- **退出自瞄**调 `YawTorqueSlewReset()`：清零斜坡历史 + ESO 状态。

```c
float YawTorqueSlewLimit(float cmd) {
    float maxD = GIMBAL_YAW_TORQUE_SLEW_RATE * yawAutoDtShared;  // 复用共享 dt
    float lo = yawTorqueCmdLast - maxD, hi = yawTorqueCmdLast + maxD;
    if (cmd > hi) cmd = hi; else if (cmd < lo) cmd = lo;
    yawTorqueCmdLast = cmd;   // 存"施加后"力矩，供 ESO 下一拍用
    return cmd;
}
```

---

## 6. 调参流程（务必按序，台架/举升/不装弹）

### 阶段 A — 基础层（ESO 关）
1. `GIMBAL_YAW_ESO_ENABLE = 0`，先只跑前馈+PD。
2. 静态给小角度目标，看是否稳、无自激。
3. 调 Kθ（角度跟踪硬度）、Kω（阻尼）。抖→降 Kθ 或加 Kω；软/跟不上→加 Kθ。
4. 确认库伦/前馈方向正确（低速无"顿"、换向平滑）。

### 阶段 B — 加 ESO
5. `ENABLE=1`，w0=5、gain=0.1、limit=0.5、LPF=5Hz（保守起步）。
6. 静止/慢跟踪听有无抖/啸叫。**抖→先降 `COMP_LPF_HZ`（5→3→2）**，不是降带宽。
7. 不抖后，`COMP_GAIN` 0.1→0.3→0.5，看稳态误差是否被补掉、有无过补震荡。
8. 还想补更强，`COMP_LIMIT` 0.5→1.5。
9. 想让扰动跟更快，`BANDWIDTH` 5→8，**一有抖立刻退回**。

> 口诀：**抖了降低通，别顶带宽。** 基础层没调好前不要开 ESO。

---

## 7. VOFA 观察通道（调参利器）

把关键量非 static 暴露，扔进 VOFA 数据帧对比波形：

| 通道 | 变量 | 判据 |
|------|------|------|
| 实测角 θ | `MeasureImu->angle` | 跟踪基准 |
| 目标角 θr | `MiniPC.yaw` | 跟踪基准 |
| 实测速 ω | `MeasureImu->palstance` | 跟踪基准 |
| 目标速/加速度 | `MiniPC.yaw_vel/yaw_acc` | 跟踪基准 |
| 施加力矩 | `yawTorqueCmdLast` | 有高频抖→往回查 ESO |
| ESO z1 | `yawEsoZ1` | 应**贴合实测角** |
| ESO z2 | `yawEsoZ2` | 应**贴合实测速** |
| ESO z3 | `yawEsoZ3` | 扰动估计原始值，**含毛刺** |
| ESO 补偿 | `yawEsoCompFilt` | 低通后，应**平滑** |

**核心对比**：z3(原始) vs CompFilt(低通后)——直接看出低通砍了多少噪声。
- CompFilt 还有毛刺 → 降 LPF_HZ。
- CompFilt 太滞后跟不上 z3 慢趋势 → 升 LPF_HZ。
- z3 发散/爆 → 降带宽。

---

## 8. 移植清单

换台机器人，照这个顺序：

- [ ] **重新辨识** J/B/Fc（第1节），改 `GIMBAL_YAW_J/B/FC`。Tau0 可不填（ESO 兜）。
- [ ] 贴 `YawDynamicsFeedforward` / `YawMitTorque` / `YawEsoUpdate` / `YawEsoCompensation` / `YawTorqueSlewLimit` / `YawAutoLoopDt` 六个函数。
- [ ] 确认 DWT 时钟可用（`CoreDebug->DEMCR` + `DWT->CTRL`），`SystemCoreClock` 正确。
- [ ] 自瞄分支调 `YawMitTorque` → `abs_limit` → `YawTorqueSlewLimit` → DM MIT 力矩。
- [ ] 非自瞄分支调 `YawTorqueSlewReset()`。
- [ ] 改硬限幅 `T_MIN/T_MAX` 为你电机的力矩量程。
- [ ] `ESO_ENABLE=0` 先调基础层（阶段 A），再开 ESO（阶段 B）。
- [ ] VOFA 接好观察通道（第7节）。

## 9. 已知局限

- 基础层是纯 PD**无积分**，稳态误差靠 ESO 补。若不开 ESO 又有恒定偏置（如重力分量），会有静差。
- ESO 反馈健康标志当前硬编码有效，IMU 失效不会触发保护——生产环境应接真实 IMU 健康位。
- 前向欧拉离散，dt 抖动大或带宽过高会影响观测器精度，故有 dt 剔除 + 带宽克制。
- 参数为本例（大惯量单轴云台、6ms 周期）整定值，换硬件必须重调，尤其 w0 和低通。

---

*文档基于实际固件整理，代码为简化示意，完整实现见 `IMCA/Task/GimbalControl.c`。*
