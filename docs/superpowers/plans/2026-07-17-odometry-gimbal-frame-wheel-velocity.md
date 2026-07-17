# Gimbal-Frame Odometry Wheel Velocity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `Odom.wheel_vx` and `Odom.wheel_vy` report wheel-derived velocity in the gimbal frame while preserving chassis-frame world-odometry inputs.

**Architecture:** Extract the existing mecanum inverse equations and the new chassis-to-gimbal rotation into a dependency-free inline kinematics header that can be compiled on the host. `Odometry.c` will keep chassis-frame velocity in local variables for world rotation and Kalman fusion, then publish a coherently rotated gimbal-frame velocity pair through `Odom.wheel_vx/wheel_vy`.

**Tech Stack:** C11, STM32G4 firmware, CMSIS-DSP trigonometry, MinGW GCC host regression test, Keil MDK firmware build.

---

## File map

- Create `IMCA/Tool/OdometryKinematics.h`: dependency-free mecanum inverse and chassis-to-gimbal velocity transforms shared by firmware and host tests.
- Create `tests/odometry_gimbal_velocity_test.c`: host regression tests for relative angles, translation plus rotation, and pure rotation.
- Modify `IMCA/Tool/Odometry.c:11,146-160,187-216`: retain chassis-frame local values for fusion and publish gimbal-frame wheel velocity.
- Modify `IMCA/Tool/Odometry.h:22-27`: document the public wheel velocity pair as gimbal-frame data.

### Task 1: Add failing host regression coverage

**Files:**
- Create: `tests/odometry_gimbal_velocity_test.c`
- Test: `tests/odometry_gimbal_velocity_test.c`

- [ ] **Step 1: Create the host regression test**

Create `tests/odometry_gimbal_velocity_test.c` with this complete content:

```c
#include <math.h>
#include <stdio.h>

#include "../IMCA/Tool/OdometryKinematics.h"

#define TEST_PI 3.14159265358979323846f
#define TEST_EPSILON 1.0e-5f

static int failures = 0;

static void AssertNear(const char *case_name,
                       const char *component,
                       float actual,
                       float expected)
{
    if (fabsf(actual - expected) > TEST_EPSILON)
    {
        fprintf(stderr,
                "FAIL %s %s: actual=%.7f expected=%.7f\n",
                case_name,
                component,
                actual,
                expected);
        failures++;
    }
}

static void ChassisVelocityToWheels(float vx_chassis,
                                    float vy_chassis,
                                    float rotation_linear,
                                    float *fr,
                                    float *fl,
                                    float *br,
                                    float *bl)
{
    *fr = -(vx_chassis + vy_chassis + rotation_linear);
    *fl = -vx_chassis + vy_chassis - rotation_linear;
    *br = vx_chassis - vy_chassis - rotation_linear;
    *bl = vx_chassis + vy_chassis - rotation_linear;
}

static void ExpectRecoveredGimbalVelocity(const char *case_name,
                                          float vx_gimbal,
                                          float vy_gimbal,
                                          float relative_yaw,
                                          float rotation_linear)
{
    const float sin_rel = sinf(relative_yaw);
    const float cos_rel = cosf(relative_yaw);
    const float vx_chassis = sin_rel * vy_gimbal + cos_rel * vx_gimbal;
    const float vy_chassis = cos_rel * vy_gimbal - sin_rel * vx_gimbal;
    float fr;
    float fl;
    float br;
    float bl;
    float recovered_vx_gimbal;
    float recovered_vy_gimbal;

    ChassisVelocityToWheels(vx_chassis,
                            vy_chassis,
                            rotation_linear,
                            &fr,
                            &fl,
                            &br,
                            &bl);

    const OdomMecanumVelocity_t decoded = Odom_DecodeCalibratedMecanum(fr, fl, br, bl);
    Odom_ChassisToGimbal(decoded.vx_chassis,
                         decoded.vy_chassis,
                         sin_rel,
                         cos_rel,
                         &recovered_vx_gimbal,
                         &recovered_vy_gimbal);

    AssertNear(case_name, "vx_gimbal", recovered_vx_gimbal, vx_gimbal);
    AssertNear(case_name, "vy_gimbal", recovered_vy_gimbal, vy_gimbal);
    AssertNear(case_name, "rotation", decoded.rotation_linear, rotation_linear);
}

int main(void)
{
    ExpectRecoveredGimbalVelocity("aligned", 0.35f, 1.20f, 0.0f, 0.0f);
    ExpectRecoveredGimbalVelocity("positive_quarter_turn", 0.0f, 1.0f, TEST_PI / 2.0f, 0.0f);
    ExpectRecoveredGimbalVelocity("negative_quarter_turn", 0.0f, 1.0f, -TEST_PI / 2.0f, 0.0f);
    ExpectRecoveredGimbalVelocity("translation_positive_spin", -0.4f, 1.3f, 0.73f, 2.2f);
    ExpectRecoveredGimbalVelocity("translation_negative_spin", 0.6f, -0.8f, -1.17f, -1.8f);
    ExpectRecoveredGimbalVelocity("pure_spin", 0.0f, 0.0f, 1.1f, 2.5f);

    if (failures != 0)
    {
        fprintf(stderr, "%d assertion(s) failed\n", failures);
        return 1;
    }

    puts("PASS: gimbal-frame odometry velocity cases");
    return 0;
}
```

- [ ] **Step 2: Create the ignored host-build directory**

Run:

```powershell
New-Item -ItemType Directory -Force -Path 'build\host-tests'
```

Expected: PowerShell reports the `build\host-tests` directory. The repository already ignores `build/`.

- [ ] **Step 3: Compile the test and verify that it fails for the missing production header**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -pedantic 'tests\odometry_gimbal_velocity_test.c' -o 'build\host-tests\odometry_gimbal_velocity_test.exe' -lm
```

Expected: compilation fails with a fatal error stating that `OdometryKinematics.h` does not exist. This is the required red phase.

### Task 2: Implement the pure kinematics functions

**Files:**
- Create: `IMCA/Tool/OdometryKinematics.h`
- Test: `tests/odometry_gimbal_velocity_test.c`

- [ ] **Step 1: Add the dependency-free production kinematics header**

Create `IMCA/Tool/OdometryKinematics.h` with this complete content:

```c
#ifndef ODOMETRY_KINEMATICS_H
#define ODOMETRY_KINEMATICS_H

// Wheel-feedback signs must match the current Chassis.car_vx/car_vy convention.
#define ODOM_VX_SIGN (+1.0f)
#define ODOM_VY_SIGN (+1.0f)

typedef struct
{
    float vx_chassis;
    float vy_chassis;
    float rotation_linear;
} OdomMecanumVelocity_t;

static inline OdomMecanumVelocity_t Odom_DecodeMecanum(float fr,
                                                        float fl,
                                                        float br,
                                                        float bl)
{
    OdomMecanumVelocity_t velocity;
    velocity.vx_chassis = (bl - fl + br - fr) * 0.25f;
    velocity.vy_chassis = (fl - fr + bl - br) * 0.25f;
    velocity.rotation_linear = -(fr + fl + br + bl) * 0.25f;
    return velocity;
}

static inline OdomMecanumVelocity_t Odom_DecodeCalibratedMecanum(float fr,
                                                                  float fl,
                                                                  float br,
                                                                  float bl)
{
    OdomMecanumVelocity_t velocity = Odom_DecodeMecanum(fr, fl, br, bl);
    velocity.vx_chassis *= ODOM_VX_SIGN;
    velocity.vy_chassis *= ODOM_VY_SIGN;
    return velocity;
}

static inline void Odom_ChassisToGimbal(float vx_chassis,
                                         float vy_chassis,
                                         float sin_relative_yaw,
                                         float cos_relative_yaw,
                                         float *vx_gimbal,
                                         float *vy_gimbal)
{
    *vx_gimbal = vx_chassis * cos_relative_yaw - vy_chassis * sin_relative_yaw;
    *vy_gimbal = vx_chassis * sin_relative_yaw + vy_chassis * cos_relative_yaw;
}

#endif // ODOMETRY_KINEMATICS_H
```

- [ ] **Step 2: Compile the host regression test**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -pedantic 'tests\odometry_gimbal_velocity_test.c' -o 'build\host-tests\odometry_gimbal_velocity_test.exe' -lm
```

Expected: exit code `0` with no warnings or errors.

- [ ] **Step 3: Run the host regression test**

Run:

```powershell
& 'build\host-tests\odometry_gimbal_velocity_test.exe'
```

Expected:

```text
PASS: gimbal-frame odometry velocity cases
```

- [ ] **Step 4: Commit the tested pure kinematics unit**

Stage only the new test and production header because the worktree contains unrelated user changes:

```powershell
git add -- 'tests\odometry_gimbal_velocity_test.c' 'IMCA\Tool\OdometryKinematics.h'
```

Commit:

```powershell
git commit -m "test: cover gimbal-frame odometry velocity"
```

Expected: one commit containing exactly the two new files.

### Task 3: Integrate gimbal-frame output without changing world odometry

**Files:**
- Modify: `IMCA/Tool/Odometry.c:11,146-160,187-216`
- Modify: `IMCA/Tool/Odometry.h:22-27`
- Test: `tests/odometry_gimbal_velocity_test.c`

- [ ] **Step 1: Include the pure kinematics header in `Odometry.c`**

Add this include beside the other local headers:

```c
#include "OdometryKinematics.h"
```

- [ ] **Step 2: Make inverse solve return chassis-frame local values**

Replace `Odom_InverseSolve(void)` with:

```c
static void Odom_InverseSolve(float *vx_chassis, float *vy_chassis)
{
    const float fr = xMotorParam[WHEEL_FR].MeasureEco->speed_rpm * WHEEL_RPM_TO_MPS;
    const float fl = xMotorParam[WHEEL_FL].MeasureEco->speed_rpm * WHEEL_RPM_TO_MPS;
    const float br = xMotorParam[WHEEL_BR].MeasureEco->speed_rpm * WHEEL_RPM_TO_MPS;
    const float bl = xMotorParam[WHEEL_BL].MeasureEco->speed_rpm * WHEEL_RPM_TO_MPS;
    const OdomMecanumVelocity_t velocity = Odom_DecodeCalibratedMecanum(fr, fl, br, bl);

    *vx_chassis = velocity.vx_chassis;
    *vy_chassis = velocity.vy_chassis;
    Odom.wheel_wz = velocity.rotation_linear / CHASSIS_ROTATE_RADIUS;
}
```

This preserves the existing wheel sign calibration and `wheel_wz` equation while preventing the chassis-frame translation from being prematurely exposed as gimbal-frame data.

- [ ] **Step 3: Publish gimbal-frame wheel velocity and retain chassis-frame fusion inputs**

Replace the inverse-solve call and wheel-to-world calculation at the start of `Odometry_Update()` sections 2 and 3 with:

```c
    /* ---------- 2. 逆解算：轮速 -> 底盘机体系速度 ---------- */
    float wheel_vx_chassis;
    float wheel_vy_chassis;
    Odom_InverseSolve(&wheel_vx_chassis, &wheel_vy_chassis);

    // 公开轮速使用云台坐标系；该矩阵是小陀螺指令变换的严格逆变换。
    const float sin_rel = arm_sin_f32(rel);
    const float cos_rel = arm_cos_f32(rel);
    Odom_ChassisToGimbal(wheel_vx_chassis,
                         wheel_vy_chassis,
                         sin_rel,
                         cos_rel,
                         &Odom.wheel_vx,
                         &Odom.wheel_vy);

    /* ---------- 3. 测量旋转到世界系 ---------- */
    // 世界系融合必须继续使用底盘机体系轮速和底盘航向。
    const float cc = arm_cos_f32(Odom.yaw_chassis);
    const float sc = arm_sin_f32(Odom.yaw_chassis);
    const float vx_w_meas = wheel_vx_chassis * cc - wheel_vy_chassis * sc;
    const float vy_w_meas = wheel_vx_chassis * sc + wheel_vy_chassis * cc;
```

Do not replace `wheel_vx_chassis/wheel_vy_chassis` in the world-frame equations with `Odom.wheel_vx/wheel_vy`; doing so would rotate the measurement twice.

- [ ] **Step 4: Update the public field documentation**

In `IMCA/Tool/Odometry.h`, make the wheel field comments explicitly state their common coordinate frame:

```c
    /* ---- 逆解算轮速（云台坐标系，单位 m/s；wheel_wz 为底盘 rad/s） ---- */
    float wheel_vx;     // 云台横向速度；右向为正（符号由 ODOM_VX_SIGN 标定）
    float wheel_vy;     // 云台纵向速度；沿云台前向为正（符号由 ODOM_VY_SIGN 标定）
    float wheel_wz;     // 底盘逆解偏航角速度 rad/s
```

- [ ] **Step 5: Re-run the host regression test**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -pedantic 'tests\odometry_gimbal_velocity_test.c' -o 'build\host-tests\odometry_gimbal_velocity_test.exe' -lm
```

Expected: exit code `0` with no warnings or errors.

Run:

```powershell
& 'build\host-tests\odometry_gimbal_velocity_test.exe'
```

Expected:

```text
PASS: gimbal-frame odometry velocity cases
```

- [ ] **Step 6: Build the firmware target**

Run:

```powershell
& 'D:\keil\UV4\UV4.exe' -b 'MDK-ARM\G474_Example.uvprojx' -t 'G474_Example' -j0
```

Expected: Keil finishes the `G474_Example` target build.

Check the build summary with a separate command, as required by `AGENTS.md`:

```powershell
Select-String -Path 'MDK-ARM\G474_Example\G474_Example.build_log.htm' -Pattern '0 Error\(s\)'
```

Expected: a summary line containing `0 Error(s)`. Compare the warning count to the pre-existing build log and require no new warning from `Odometry.c` or `OdometryKinematics.h`.

- [ ] **Step 7: Review the scoped source diff**

Run:

```powershell
git diff --check -- 'IMCA\Tool\Odometry.c' 'IMCA\Tool\Odometry.h'
```

Expected: no output.

Run:

```powershell
git diff -- 'IMCA\Tool\Odometry.c' 'IMCA\Tool\Odometry.h'
```

Expected: only the kinematics include, chassis-frame local data flow, gimbal-frame publication, and field-comment changes described above.

- [ ] **Step 8: Commit the firmware integration**

Stage only the two odometry files:

```powershell
git add -- 'IMCA\Tool\Odometry.c' 'IMCA\Tool\Odometry.h'
```

Commit:

```powershell
git commit -m "fix: report odometry wheel velocity in gimbal frame"
```

Expected: one commit containing only `Odometry.c` and `Odometry.h`.

### Task 4: Perform on-robot acceptance checks

**Files:**
- Verify: `IMCA/Task/ControlTask.c:250-266`

- [ ] **Step 1: Flash the verified firmware using the project's normal Keil workflow**

Use the already-built `G474_Example` target. Do not alter calibration constants before collecting the baseline traces.

- [ ] **Step 2: Verify pure small-gyro rotation in VOFA**

With translation commands released, enable small-gyro mode and observe channel 0 (`Odom.wheel_vy`) and channel 3 (`Odom.wheel_vx`).

Expected: both remain near zero apart from encoder noise and wheel slip while the chassis rotates.

- [ ] **Step 3: Verify cloud-platform-forward translation in VOFA**

Hold the forward command while small-gyro mode is rotating.

Expected: channel 0 remains primarily one-signed and approximately constant instead of following the chassis relative angle sinusoid. Reversing the command reverses channel 0.

- [ ] **Step 4: Verify lateral translation**

Hold each lateral command while small-gyro mode is rotating and observe channel 3.

Expected: channel 3 remains primarily one-signed and reverses with the command, confirming that `wheel_vx` and `wheel_vy` share the gimbal frame.

- [ ] **Step 5: Record any remaining scale or sign discrepancy separately**

If the transformed direction is stable but the sign or magnitude is wrong, record the four raw wheel RPMs and relative angle. Treat wheel sign/scale calibration as a separate change, consistent with the approved design's non-goals.
