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

    const OdomMecanumVelocity_t decoded = Odom_DecodeMecanum(fr, fl, br, bl);
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
