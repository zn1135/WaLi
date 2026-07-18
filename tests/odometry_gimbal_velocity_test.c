#include <math.h>
#include <stdio.h>

#include "../IMCA/Tool/OdometryKinematics.h"

#define PI_F 3.14159265358979323846f
#define EPS 1.0e-4f

static int failures;

static void Near(const char *name, float actual, float expected)
{
    if (fabsf(actual - expected) > EPS)
    {
        fprintf(stderr, "FAIL %s actual=%.6f expected=%.6f\n", name, actual, expected);
        failures++;
    }
}

static void TestWheelForwardKinematics(void)
{
    float vx, vy, vw;
    // 纯 vy: FR=-vy FL=+vy BR=-vy BL=+vy (vy=100)
    Odom_WheelForwardKinematics(-100.0f, 100.0f, -100.0f, 100.0f, &vx, &vy, &vw);
    Near("pure_vy vx", vx, 0.0f);
    Near("pure_vy vy", vy, 100.0f);
    Near("pure_vy vw", vw, 0.0f);
    // 纯 vx: FR=-vx FL=-vx BR=+vx BL=+vx (vx=100)
    Odom_WheelForwardKinematics(-100.0f, -100.0f, 100.0f, 100.0f, &vx, &vy, &vw);
    Near("pure_vx vx", vx, 100.0f);
    Near("pure_vx vy", vy, 0.0f);
}

static void TestWheelBodyToNav(void)
{
    float nx, ny;
    // head=0：前向(vy=1) 落在 +nav_x
    Odom_WheelBodyToNav(0.0f, 1.0f, 0.0f, &nx, &ny);
    Near("head0 fwd x", nx, 1.0f);
    Near("head0 fwd y", ny, 0.0f);
    // head=90deg：前向(vy=1) 落在 +nav_y
    Odom_WheelBodyToNav(0.0f, 1.0f, 0.5f * PI_F, &nx, &ny);
    Near("head90 fwd x", nx, 0.0f);
    Near("head90 fwd y", ny, 1.0f);
}

static void TestWheelBodyToGimbal(void)
{
    float gx, gy;
    // rel=0：两系重合，恒等
    Odom_WheelBodyToGimbal(0.7f, 0.3f, 0.0f, &gx, &gy);
    Near("rel0 vx", gx, 0.7f);
    Near("rel0 vy", gy, 0.3f);
    // 旋向 R(+rel)。rel=90deg：前向(vy=1)->云台系VX=-1；右向(vx=1)->云台系VY=+1
    Odom_WheelBodyToGimbal(0.0f, 1.0f, 0.5f * PI_F, &gx, &gy);
    Near("rel90 fwd->vx", gx, -1.0f);
    Near("rel90 fwd->vy", gy, 0.0f);
    Odom_WheelBodyToGimbal(1.0f, 0.0f, 0.5f * PI_F, &gx, &gy);
    Near("rel90 right->vx", gx, 0.0f);
    Near("rel90 right->vy", gy, 1.0f);
    // 一般角校核：rel=30deg 手算 R(+rel)
    const float c = cosf(PI_F / 6.0f), s = sinf(PI_F / 6.0f);
    Odom_WheelBodyToGimbal(0.4f, 0.9f, PI_F / 6.0f, &gx, &gy);
    Near("rel30 vx", gx, 0.4f * c - 0.9f * s);
    Near("rel30 vy", gy, 0.4f * s + 0.9f * c);
}

// 关键：轮速前向 与 加速度前向 必须落在同一个 E 系(否则 90° 错位)。
static void TestWheelAccelSameFrame(void)
{
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float head;
    if (!Odom_QuaternionToYaw(identity, &head)) { failures++; return; }
    float vnx, vny;
    Odom_WheelBodyToNav(0.0f, 1.0f, head, &vnx, &vny); // 前向单位速度
    float a_nav[2];
    const float acc_s[3] = {0.0f, 1.0f, 0.0f};          // 前向单位比力
    Odom_AccelSensorToNavHorizontal(identity, acc_s, 0.0f, a_nav);
    Near("same frame x", vnx, a_nav[0]);
    Near("same frame y", vny, a_nav[1]);
}

static void TestQuaternionToYaw(void)
{
    // 单位四元数：前向传感器轴 {0,1,0} 在 nav 系不变，yaw = atan2(1,0) = pi/2
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float yaw;
    if (!Odom_QuaternionToYaw(identity, &yaw)) { failures++; return; }
    Near("identity yaw", yaw, 0.5f * PI_F);
}

static void TestAccelGravityRemoval(void)
{
    // 单位四元数静止：acc 传感器系 = {0,0,g}，去重力后水平应为 0
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float acc_s[3] = {0.0f, 0.0f, ODOM_KIN_G};
    float accel_nav[2];
    Odom_AccelSensorToNavHorizontal(identity, acc_s, 1.0f, accel_nav);
    Near("static ax_nav", accel_nav[0], 0.0f);
    Near("static ay_nav", accel_nav[1], 0.0f);
}

static void TestComplementaryFilter(void)
{
    const float dt = 0.001f, tau = 0.2f;
    const float k = dt / (tau + dt);
    // a=0、v_wheel 恒定：融合应收敛到 v_wheel
    float v = 0.0f;
    for (int i = 0; i < 5000; ++i) { v = Odom_ComplementaryStep(v, 0.0f, 2.0f, dt, k); }
    Near("converge to wheel", v, 2.0f);
    // 高频加速度冲击：单步应主要跟加速度(k 很小，轮速拉回弱)
    float v2 = Odom_ComplementaryStep(0.0f, 10.0f, 0.0f, dt, k);
    Near("accel dominates 1step", v2, 10.0f * dt * (1.0f - k));
    // 稳态一致性：v_wheel 恒定且 a=0 时，已在锚点则不动
    Near("anchor stays", Odom_ComplementaryStep(2.0f, 0.0f, 2.0f, dt, k), 2.0f);
}

static void TestKf2(void)
{
    const float dt = 0.001f;
    const float Qp = 1e-4f, Qv = 1e-3f, Rv = 1e-2f;
    OdomKf2_t kf;

    // 匀速 v=1 m/s，a=0，KF 收敛后 vel 应接近 1，pos 线性增长
    Odom_Kf2_Init(&kf, 1.0f, 1.0f, 0.1f);
    for (int i = 0; i < 5000; ++i)
    {
        Odom_Kf2_Predict(&kf, 0.0f, dt, Qp, Qv);
        Odom_Kf2_Update(&kf, 1.0f, Rv);
    }
    Near("kf steady vel", kf.v, 1.0f);
    // 5000步*0.001s*1m/s = 5m，允许一定漂移（初始不确定度）
    if (kf.p < 4.5f || kf.p > 5.5f)
    {
        fprintf(stderr, "FAIL kf steady pos=%.4f expected ~5.0\n", kf.p);
        failures++;
    }

    // 静止：vel 观测=0，加速度=0，KF 应快速收敛到 vel≈0
    Odom_Kf2_Init(&kf, 0.5f, 1.0f, 0.1f);
    for (int i = 0; i < 3000; ++i)
    {
        Odom_Kf2_Predict(&kf, 0.0f, dt, Qp, Qv);
        Odom_Kf2_Update(&kf, 0.0f, Rv);
    }
    Near("kf static vel", kf.v, 0.0f);

    // 协方差应始终为正（正定）
    if (kf.P00 <= 0.0f || kf.P11 <= 0.0f)
    {
        fprintf(stderr, "FAIL kf P not positive: P00=%.6f P11=%.6f\n",
                kf.P00, kf.P11);
        failures++;
    }
}

int main(void)
{
    TestWheelForwardKinematics();
    TestWheelBodyToNav();
    TestWheelBodyToGimbal();
    TestWheelAccelSameFrame();
    TestQuaternionToYaw();
    TestAccelGravityRemoval();
    TestComplementaryFilter();
    TestKf2();
    if (failures != 0)
    {
        fprintf(stderr, "%d assertion(s) failed\n", failures);
        return 1;
    }
    puts("PASS: wheel odometry kinematics");
    return 0;
}
