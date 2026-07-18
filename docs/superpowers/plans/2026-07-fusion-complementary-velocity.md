# 世界系速度融合:互补滤波(固定权重)

## 目标
轮速做低频锚点(不漂)+ IMU 世界系加速度做高频修正(不裸积分漂移),
输出平滑、跟手、抗小陀螺打滑/延迟的世界系速度 fused_vx/vy_nav。

## 核心算法(每周期 dt=0.001s 固定)
```
v_pred  = v_fused + a_nav * dt          // IMU 加速度预测(高频)
v_fused = v_pred + K * (v_wheel - v_pred)   // 轮速拉回(低频锚)
K = dt / (TAU + dt)
```
- 高频:IMU 加速度驱动 → 跟手、抗轮打滑
- 低频:被 v_wheel 拉住 → 不漂
- TAU = 信任时间常数,交叉频率 f_c = 1/(2π·TAU)
- 起步 TAU=0.2s (f_c≈0.8Hz),K≈0.00498,上车可调

## 边界处理
- **重力未收敛**(gravity_ready=0,前~2s):不积分 IMU,fused 直接跟随 v_wheel。
  避免开机时重力估计未稳、a_nav 带偏置污染积分。
- **yaw_valid=0**:无法转世界系,fused 保持上一拍(hold),不更新。
- 融合状态 fused_vx/vy_nav 在 WheelOdom_Init 清零。

## 纯函数(可主机测试)
OdometryKinematics.h 加:
```c
static inline float Odom_ComplementaryStep(float v_prev, float a_nav,
                                           float v_wheel, float dt, float k)
{
    float v_pred = v_prev + a_nav * dt;
    return v_pred + k * (v_wheel - v_pred);
}
```
测试:a=0 且 v_wheel 恒定 → 收敛到 v_wheel;a≠0 短冲击 → 跟随后被拉回。

## 改动清单
1. **OdometryKinematics.h**:加 Odom_ComplementaryStep 纯函数。
2. **Odometry.h**:WheelOdom_t 加 fused_vx_nav / fused_vy_nav 字段。
3. **Odometry.c**:顶部加 ODOM_FUSION_TAU 宏;Update 末尾按上述算法+边界更新 fused。
4. **ControlTask.c** VOFA:ch1→fused_vx_nav, ch3→fused_vy_nav
   (与 ch0/ch2 轮速世界系叠图直接对比;IMU 原始加速度已验证完可下线)。
   新通道表:0=轮速vx,1=融合vx,2=轮速vy,3=融合vy,4=连续航向,
   5=车体vx,6=车体vy,7=|v_body|。
5. **tests/**:加 Odom_ComplementaryStep 收敛性测试。

## 验证(上车)
- 直行推:fused(ch1/3)应平滑跟随轮速(ch0/2),噪声更低、无明显滞后。
- 急停/急启:fused 靠 IMU 高频应比纯轮速更快响应。
- 边转边走(小陀螺):fused 应比 ch0/2 摆动小(IMU 高频不受延迟/打滑污染,
  轮速仅做低频锚)。TAU 越大越信 IMU、越平滑;越小越跟轮速。

## 不做(本阶段)
- 自适应权重(按 yaw 角速度切换):固定权重跑通后再加,单独一步。

---

# 追加:航向同源化(修小陀螺速度)

## 问题(已定位)
小陀螺匀速时 IMU 加速度≈0,融合帮不上;根子在轮速链:v_body(轮电机CAN)与
chassis_yaw(云台IMU+YAW电机)异源有延迟 Δ,世界系速度被转了 ω·Δ。
实测确认(A):ch7=|v_body| 平 → 轮速本身没错,纯航向不同步。

## 修法:航向互补滤波(vw 高频传播 + chassis_yaw_cont 低频纠偏)
```
ωz      = vw_body_raw * ODOM_VW_TO_RADPS   // 轮速角速度, 与 v_body 同源零延迟
head_sync += ωz * dt                       // 高频积分
err      = wrap(chassis_yaw_cont - head_sync)
head_sync += KH * err                      // 低频纠偏(不漂,尺度锚在已验证的2π)
```
世界系速度重构改用 head_sync(与 v_body 同时刻同源)替代原始 chassis_yaw。

- vw_body 是四轮之和,小陀螺时大而干净(非小差),积分稳。
- ODOM_VW_TO_RADPS 起算 = WHEEL_ODOM_SCALE/(lx+ly);不准只影响暂态,
  低频纠偏保证稳态回到 chassis_yaw_cont。上车调。
- KH = dt/(TAU_H+dt),TAU_H≈0.5s 起步(慢纠偏,让高频 vw 主导暂态)。

## 追加改动
- Odometry.c:加 ODOM_VW_TO_RADPS、ODOM_HEAD_TAU 宏;head_sync 状态与更新;
  Odom_WheelBodyToNav 改用 head_sync。
- Odometry.h:WheelOdom_t 加 head_sync 字段。
- 可加 VOFA 临时通道对比 head_sync vs chassis_yaw_cont。

