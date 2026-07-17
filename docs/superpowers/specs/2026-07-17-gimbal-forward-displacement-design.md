# 小陀螺云台前后位移设计

## 目标

新增 `Odom.pos_gimbal_y`，表示机器人沿当前云台前向的累计位移，单位为米。小陀螺模式下底盘持续旋转时，该量不随底盘机体系旋转而来回变化。

## 保留的现有语义

- `Odom.pos_x`、`Odom.pos_y` 继续表示里程计世界系位置，不改名、不改计算方式。
- `Odom.wheel_vx`、`Odom.wheel_vy` 继续表示云台坐标系轮速。
- 世界系卡尔曼融合和位置积分继续使用世界系速度。

## 计算方式

卡尔曼融合得到世界系速度后，使用底盘里程计航向与电机相对角计算云台世界航向，再投影出当前云台前向速度。相对角使用 `xMotorParam[YAW].MeasureDM->position - Gimbal_FrontAngle`（单位为度，计算时转换为弧度）：

```c
relative_yaw = (xMotorParam[YAW].MeasureDM->position - Gimbal_FrontAngle) * PI / 180;
gimbal_yaw = Odom.yaw_chassis + relative_yaw;
v_gimbal_y = -Odom.vx_world * sin(gimbal_yaw)
            + Odom.vy_world * cos(gimbal_yaw);
Odom.pos_gimbal_y += v_gimbal_y * ODOM_DT;
```

`Odom.yaw_chassis` 已以里程计启动时的底盘朝向为零，加上云台相对角即可得到与 `pos_x/pos_y` 同一世界系的云台航向。位移积分使用融合速度而不是原始加速度或未经融合的轮速，保持现有滤波策略。

该定义是“沿每个时刻云台前向的速度积分”。云台在运动过程中改变朝向时，它表示前向路径分量的积分；小陀螺中云台通常保持朝向不变，此时等价于世界位移在云台前向轴上的投影。

## 输出接口

- 在 `Odometry_t` 中增加 `float pos_gimbal_y`。
- `Odometry_Reset()` 将该字段清零。
- `Odometry_Update()` 每周期更新该字段。
- `ControlTask.c` 的 VOFA 第 7 通道改为发送 `Odom.pos_gimbal_y`，并更新注释；原 `Odom.pos_y` 仍可通过结构体访问。

## 边界处理

- 初始化、复位后位移从零开始。
- 卡尔曼分配失败时沿用现有世界系轮速兜底速度，因此位移仍可继续更新。
- 不新增四元数积分速度；加速度零偏漂移不由本功能解决。

## 回归测试

新增纯函数测试覆盖世界系速度向云台前向投影及积分：

1. 云台航向为 0，世界前向速度全部计入 `pos_gimbal_y`。
2. 云台航向为 `+PI/2`，世界横向速度按正方向计入云台前向位移。
3. 云台航向为 `-PI/2`，验证相反方向符号。
4. 世界速度为 0，位移保持不变。
5. 多周期速度积分结果等于速度乘以周期数和 `ODOM_DT`。

## 验收标准

- 主机回归测试通过。
- Keil 固件构建无错误，新增代码无编译警告。
- 小陀螺保持云台朝向并向前移动时，VOFA 第 7 通道单调增加；反向移动时单调减少；仅原地旋转时保持接近零。
