# Gimbal-Forward Displacement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a cumulative displacement output along the current gimbal-forward axis for small-gyro motion without changing the existing world-frame position fields.

**Architecture:** Add a pure world-velocity-to-gimbal-forward projection helper beside the existing odometry kinematics helpers. `Odometry_Update()` will integrate that projected Kalman velocity into a new `Odom.pos_gimbal_y`, while `pos_x/pos_y` continue using the existing world-frame integration; VOFA channel 7 will expose the new field.

**Tech Stack:** C11, STM32G4 firmware, CMSIS-DSP trigonometry, MinGW GCC host regression test, Keil MDK firmware build.

---

## File map

- Modify `IMCA/Tool/OdometryKinematics.h`: add a pure world-to-gimbal-forward velocity projection helper.
- Modify `tests/odometry_gimbal_velocity_test.c`: add projection and multi-period integration assertions to the existing host test.
- Modify `IMCA/Tool/Odometry.h:43-47`: add and document `pos_gimbal_y` beside world position fields.
- Modify `IMCA/Tool/Odometry.c:124-130,262-270`: reset and integrate `pos_gimbal_y` from fused world velocity.
- Modify `IMCA/Task/ControlTask.c:259-266`: send `Odom.pos_gimbal_y` on VOFA channel 7.

### Task 1: Add the failing projection and integration regression test

**Files:**
- Modify: `tests/odometry_gimbal_velocity_test.c`
- Test: `tests/odometry_gimbal_velocity_test.c`

- [ ] **Step 1: Add a test that calls the wished-for projection helper**

Append these functions and calls before `main()` returns in `tests/odometry_gimbal_velocity_test.c`:

```c
static void ExpectGimbalForwardProjection(const char *case_name,
                                          float vx_world,
                                          float vy_world,
                                          float yaw,
                                          float expected_forward)
{
    const float actual_forward = Odom_WorldToGimbalForward(vx_world,
                                                            vy_world,
                                                            sinf(yaw),
                                                            cosf(yaw));
    AssertNear(case_name, "forward_velocity", actual_forward, expected_forward);
}

static void ExpectGimbalForwardIntegration(const char *case_name,
                                           float forward_velocity,
                                           int periods,
                                           float dt,
                                           float expected_displacement)
{
    float displacement = 0.0f;
    int i;
    for (i = 0; i < periods; i++)
    {
        displacement += forward_velocity * dt;
    }
    AssertNear(case_name, "forward_displacement", displacement, expected_displacement);
}
```

Add these calls in `main()` before the failure check:

```c
    ExpectGimbalForwardProjection("gimbal_yaw_zero", 0.0f, 1.5f, 0.0f, 1.5f);
    ExpectGimbalForwardProjection("gimbal_yaw_positive_quarter", 1.5f, 0.0f, TEST_PI / 2.0f, -1.5f);
    ExpectGimbalForwardProjection("gimbal_yaw_negative_quarter", 1.5f, 0.0f, -TEST_PI / 2.0f, 1.5f);
    ExpectGimbalForwardProjection("zero_world_velocity", 0.0f, 0.0f, 0.9f, 0.0f);
    ExpectGimbalForwardIntegration("forward_displacement_over_periods", 1.25f, 80, 0.001f, 0.1f);
```

- [ ] **Step 2: Compile and verify the new test fails because the helper is absent**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -pedantic 'tests\odometry_gimbal_velocity_test.c' -o 'build\host-tests\odometry_gimbal_velocity_test.exe' -lm
```

Expected: compilation fails with an undeclared `Odom_WorldToGimbalForward` error. This is the required red phase.

### Task 2: Implement the pure projection helper

**Files:**
- Modify: `IMCA/Tool/OdometryKinematics.h`
- Test: `tests/odometry_gimbal_velocity_test.c`

- [ ] **Step 1: Add the minimal world-to-gimbal-forward helper**

Add this function after `Odom_ChassisToGimbal()` in `IMCA/Tool/OdometryKinematics.h`:

```c
static inline float Odom_WorldToGimbalForward(float vx_world,
                                               float vy_world,
                                               float sin_gimbal_yaw,
                                               float cos_gimbal_yaw)
{
    return -vx_world * sin_gimbal_yaw + vy_world * cos_gimbal_yaw;
}
```

- [ ] **Step 2: Compile and run the host regression test**

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

- [ ] **Step 3: Commit the tested projection helper**

Stage only the helper and its host-test changes:

```powershell
git add -- 'IMCA\Tool\OdometryKinematics.h' 'tests\odometry_gimbal_velocity_test.c'
```

Commit:

```powershell
git commit -m "test: cover gimbal-forward displacement projection"
```

### Task 3: Integrate and expose cumulative gimbal-forward displacement

**Files:**
- Modify: `IMCA/Tool/Odometry.h:43-47`
- Modify: `IMCA/Tool/Odometry.c:124-130,262-270`
- Modify: `IMCA/Task/ControlTask.c:259-266`
- Test: `tests/odometry_gimbal_velocity_test.c`

- [ ] **Step 1: Add the public displacement field**

In `Odometry_t`, extend the position section to:

```c
    /* ---- 里程计输出：世界系位置与云台前向位移（单位 m）---- */
    float pos_x;
    float pos_y;
    float pos_gimbal_y; // 沿当前云台前向的累计位移
    float total_dist;   // 累计路程 m
```

- [ ] **Step 2: Reset the new field**

In `Odometry_Reset()`, after resetting `Odom.pos_y`, add:

```c
    Odom.pos_gimbal_y = 0.0f;
```

`Odometry_Init()` already clears the whole structure with `memset`, so no separate initialization is needed there.

- [ ] **Step 3: Integrate fused velocity in the gimbal-forward direction**

Immediately after the existing world position integration in `Odometry_Update()`:

```c
    const float cg = arm_cos_f32(Odom.yaw_gimbal);
    const float sg = arm_sin_f32(Odom.yaw_gimbal);
    const float vy_gimbal_fused = Odom_WorldToGimbalForward(Odom.vx_world,
                                                            Odom.vy_world,
                                                            sg,
                                                            cg);
    Odom.pos_gimbal_y += vy_gimbal_fused * ODOM_DT;
```

Keep `Odom.pos_x`, `Odom.pos_y`, and `Odom.total_dist` unchanged. The projection uses `Odom.yaw_gimbal`, whose zero is aligned to the odometry world frame at initialization.

- [ ] **Step 4: Change VOFA channel 7 to the new displacement**

In `ControlTask.c`, replace only the last telemetry argument:

```c
                    Odom.pos_gimbal_y); // 7 云台前后位移 (m)
```

- [ ] **Step 5: Compile and run the host regression test again**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -pedantic 'tests\odometry_gimbal_velocity_test.c' -o 'build\host-tests\odometry_gimbal_velocity_test.exe' -lm
```

Expected: exit code `0`.

Run:

```powershell
& 'build\host-tests\odometry_gimbal_velocity_test.exe'
```

Expected: `PASS: gimbal-frame odometry velocity cases`.

- [ ] **Step 6: Build the firmware target**

Run:

```powershell
& 'D:\keil\UV4\UV4.exe' -b 'MDK-ARM\G474_Example.uvprojx' -t 'G474_Example' -j0
```

Expected: Keil completes the `G474_Example` target build.

Check the result separately:

```powershell
Select-String -Path 'MDK-ARM\G474_Example\G474_Example.build_log.htm' -Pattern 'Error\(s\).*Warning\(s\)'
```

Expected: `0 Error(s)` and no warning originating from the new helper or `pos_gimbal_y` code.

- [ ] **Step 7: Review and commit the integration**

Run:

```powershell
git diff --check -- 'IMCA\Tool\Odometry.h' 'IMCA\Tool\Odometry.c' 'IMCA\Task\ControlTask.c'
```

Expected: no whitespace errors.

Stage only the three integration files:

```powershell
git add -- 'IMCA\Tool\Odometry.h' 'IMCA\Tool\Odometry.c' 'IMCA\Task\ControlTask.c'
```

Commit:

```powershell
git commit -m "feat: expose gimbal-forward displacement"
```

### Task 4: Hardware acceptance

**Files:**
- Verify: `IMCA/Task/ControlTask.c:259-266`

- [ ] **Step 1: Flash the built target through the normal Keil workflow**

- [ ] **Step 2: In small-gyro mode, move forward along the gimbal heading**

Expected: VOFA channel 7 (`Odom.pos_gimbal_y`) increases monotonically; reverse movement decreases it.

- [ ] **Step 3: Rotate in place without translation**

Expected: channel 7 stays near its prior value; chassis rotation alone does not accumulate forward displacement.
