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

static inline float Odom_WorldToGimbalForward(float vx_world,
                                               float vy_world,
                                               float sin_gimbal_yaw,
                                               float cos_gimbal_yaw)
{
    return -vx_world * sin_gimbal_yaw + vy_world * cos_gimbal_yaw;
}

static inline float Odom_WorldToGimbalForwardFromChassis(float vx_world,
                                                          float vy_world,
                                                          float sin_chassis_yaw,
                                                          float cos_chassis_yaw,
                                                          float sin_relative_yaw,
                                                          float cos_relative_yaw)
{
    const float sin_gimbal_yaw = sin_chassis_yaw * cos_relative_yaw +
                                 cos_chassis_yaw * sin_relative_yaw;
    const float cos_gimbal_yaw = cos_chassis_yaw * cos_relative_yaw -
                                 sin_chassis_yaw * sin_relative_yaw;
    return Odom_WorldToGimbalForward(vx_world,
                                     vy_world,
                                     sin_gimbal_yaw,
                                     cos_gimbal_yaw);
}

static inline float Odom_GimbalToChassisYaw(float gimbal_yaw,
                                             float relative_yaw)
{
    return gimbal_yaw - relative_yaw;
}

#endif // ODOMETRY_KINEMATICS_H
