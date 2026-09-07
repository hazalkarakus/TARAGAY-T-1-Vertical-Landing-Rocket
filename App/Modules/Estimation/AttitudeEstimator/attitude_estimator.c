#include "Modules/Estimation/AttitudeEstimator/attitude_estimator.h"

#include "Common/app_config.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"
#include "Services/Timebase/timebase.h"

#include <math.h>
#include <stdint.h>

#define ATTITUDE_DEG_TO_RAD  0.01745329251994329577f
#define ATTITUDE_RAD_TO_DEG  57.295779513082320876f

static AttitudeEstimatorData_t attitude_data;
static uint32_t euler_decimation_counter = 0UL;
static uint32_t live_debug_decimation_counter = 0UL;

/* -------------------------------------------------------------------------- */
/* Live Expressions                                                           */
/* -------------------------------------------------------------------------- */

volatile uint8_t attitude_enabled = 0U;
volatile uint8_t attitude_initialized = 0U;
volatile uint8_t attitude_healthy = 0U;
volatile uint8_t attitude_accel_correction_active = 0U;

volatile float attitude_q_w = 1.0f;
volatile float attitude_q_x = 0.0f;
volatile float attitude_q_y = 0.0f;
volatile float attitude_q_z = 0.0f;

volatile float attitude_roll_deg = 0.0f;
volatile float attitude_pitch_deg = 0.0f;
volatile float attitude_yaw_deg = 0.0f;

volatile float attitude_world_specific_force_x_g = 0.0f;
volatile float attitude_world_specific_force_y_g = 0.0f;
volatile float attitude_world_specific_force_z_g = 0.0f;

volatile float attitude_world_linear_accel_x_mps2 = 0.0f;
volatile float attitude_world_linear_accel_y_mps2 = 0.0f;
volatile float attitude_world_linear_accel_z_mps2 = 0.0f;

volatile float attitude_accel_correction_weight = 0.0f;
volatile float attitude_integral_feedback_x_rad_s = 0.0f;
volatile float attitude_integral_feedback_y_rad_s = 0.0f;
volatile float attitude_integral_feedback_z_rad_s = 0.0f;

volatile uint32_t attitude_update_count = 0UL;
volatile uint32_t attitude_duplicate_skip_count = 0UL;
volatile uint32_t attitude_accel_correction_count = 0UL;
volatile uint32_t attitude_accel_reject_count = 0UL;
volatile uint32_t attitude_reset_count = 0UL;
volatile uint32_t attitude_numerical_error_count = 0UL;
volatile uint32_t attitude_gap_reset_count = 0UL;
volatile uint32_t attitude_last_timestamp_us = 0UL;
volatile uint32_t attitude_last_dt_us = 0UL;
volatile uint32_t attitude_max_dt_us = 0UL;

/* -------------------------------------------------------------------------- */

static float Attitude_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

/* -------------------------------------------------------------------------- */

static uint8_t Attitude_IsFinite(float value)
{
    if (value != value)
    {
        return 0U;
    }

    if ((value > 1000000.0f) || (value < -1000000.0f))
    {
        return 0U;
    }

    return 1U;
}

/* -------------------------------------------------------------------------- */

static void Attitude_UpdateLiveDebug(void)
{
    attitude_enabled = attitude_data.enabled;
    attitude_initialized = attitude_data.initialized;
    attitude_healthy = attitude_data.healthy;
    attitude_accel_correction_active =
        attitude_data.accel_correction_active;

    attitude_q_w = attitude_data.q_w;
    attitude_q_x = attitude_data.q_x;
    attitude_q_y = attitude_data.q_y;
    attitude_q_z = attitude_data.q_z;

    attitude_roll_deg = attitude_data.roll_deg;
    attitude_pitch_deg = attitude_data.pitch_deg;
    attitude_yaw_deg = attitude_data.yaw_deg;

    attitude_world_specific_force_x_g =
        attitude_data.world_specific_force_x_g;
    attitude_world_specific_force_y_g =
        attitude_data.world_specific_force_y_g;
    attitude_world_specific_force_z_g =
        attitude_data.world_specific_force_z_g;

    attitude_world_linear_accel_x_mps2 =
        attitude_data.world_linear_accel_x_mps2;
    attitude_world_linear_accel_y_mps2 =
        attitude_data.world_linear_accel_y_mps2;
    attitude_world_linear_accel_z_mps2 =
        attitude_data.world_linear_accel_z_mps2;

    attitude_accel_correction_weight =
        attitude_data.accel_correction_weight;

    attitude_integral_feedback_x_rad_s =
        attitude_data.integral_feedback_x_rad_s;
    attitude_integral_feedback_y_rad_s =
        attitude_data.integral_feedback_y_rad_s;
    attitude_integral_feedback_z_rad_s =
        attitude_data.integral_feedback_z_rad_s;

    attitude_update_count = attitude_data.update_count;
    attitude_duplicate_skip_count = attitude_data.duplicate_skip_count;
    attitude_accel_correction_count =
        attitude_data.accel_correction_count;
    attitude_accel_reject_count = attitude_data.accel_reject_count;
    attitude_reset_count = attitude_data.reset_count;
    attitude_numerical_error_count = attitude_data.numerical_error_count;
    attitude_gap_reset_count = attitude_data.gap_reset_count;
    attitude_last_timestamp_us = attitude_data.last_timestamp_us;
    attitude_last_dt_us = attitude_data.last_dt_us;
    attitude_max_dt_us = attitude_data.max_dt_us;
}

/* -------------------------------------------------------------------------- */

static uint8_t Attitude_NormalizeQuaternion(void)
{
    float norm_squared =
        (attitude_data.q_w * attitude_data.q_w) +
        (attitude_data.q_x * attitude_data.q_x) +
        (attitude_data.q_y * attitude_data.q_y) +
        (attitude_data.q_z * attitude_data.q_z);

    if ((!Attitude_IsFinite(norm_squared)) ||
        (norm_squared < 0.000001f) ||
        (norm_squared > 100.0f))
    {
        return 0U;
    }

    float inverse_norm = 1.0f / sqrtf(norm_squared);

    attitude_data.q_w *= inverse_norm;
    attitude_data.q_x *= inverse_norm;
    attitude_data.q_y *= inverse_norm;
    attitude_data.q_z *= inverse_norm;

    return 1U;
}

/* -------------------------------------------------------------------------- */

static void Attitude_UpdateEulerAngles(void)
{
    float qw = attitude_data.q_w;
    float qx = attitude_data.q_x;
    float qy = attitude_data.q_y;
    float qz = attitude_data.q_z;

    float sin_roll_cos_pitch = 2.0f * ((qw * qx) + (qy * qz));
    float cos_roll_cos_pitch =
        1.0f - (2.0f * ((qx * qx) + (qy * qy)));

    float sin_pitch = 2.0f * ((qw * qy) - (qz * qx));
    sin_pitch = Attitude_Clamp(sin_pitch, -1.0f, 1.0f);

    float sin_yaw_cos_pitch = 2.0f * ((qw * qz) + (qx * qy));
    float cos_yaw_cos_pitch =
        1.0f - (2.0f * ((qy * qy) + (qz * qz)));

    attitude_data.roll_deg =
        atan2f(sin_roll_cos_pitch, cos_roll_cos_pitch) *
        ATTITUDE_RAD_TO_DEG;

    attitude_data.pitch_deg = asinf(sin_pitch) * ATTITUDE_RAD_TO_DEG;

    attitude_data.yaw_deg =
        atan2f(sin_yaw_cos_pitch, cos_yaw_cos_pitch) *
        ATTITUDE_RAD_TO_DEG;

    attitude_data.euler_update_count++;
}

/* -------------------------------------------------------------------------- */

static void Attitude_UpdateWorldAcceleration(
    float accel_x_g,
    float accel_y_g,
    float accel_z_g
)
{
    float qw = attitude_data.q_w;
    float qx = attitude_data.q_x;
    float qy = attitude_data.q_y;
    float qz = attitude_data.q_z;

    float r00 = 1.0f - (2.0f * ((qy * qy) + (qz * qz)));
    float r01 = 2.0f * ((qx * qy) - (qw * qz));
    float r02 = 2.0f * ((qx * qz) + (qw * qy));

    float r10 = 2.0f * ((qx * qy) + (qw * qz));
    float r11 = 1.0f - (2.0f * ((qx * qx) + (qz * qz)));
    float r12 = 2.0f * ((qy * qz) - (qw * qx));

    float r20 = 2.0f * ((qx * qz) - (qw * qy));
    float r21 = 2.0f * ((qy * qz) + (qw * qx));
    float r22 = 1.0f - (2.0f * ((qx * qx) + (qy * qy)));

    attitude_data.world_specific_force_x_g =
        (r00 * accel_x_g) +
        (r01 * accel_y_g) +
        (r02 * accel_z_g);

    attitude_data.world_specific_force_y_g =
        (r10 * accel_x_g) +
        (r11 * accel_y_g) +
        (r12 * accel_z_g);

    attitude_data.world_specific_force_z_g =
        (r20 * accel_x_g) +
        (r21 * accel_y_g) +
        (r22 * accel_z_g);

    attitude_data.world_linear_accel_x_mps2 =
        attitude_data.world_specific_force_x_g *
        APP_ATTITUDE_GRAVITY_MPS2;

    attitude_data.world_linear_accel_y_mps2 =
        attitude_data.world_specific_force_y_g *
        APP_ATTITUDE_GRAVITY_MPS2;

    attitude_data.world_linear_accel_z_mps2 =
        (attitude_data.world_specific_force_z_g - 1.0f) *
        APP_ATTITUDE_GRAVITY_MPS2;
}

/* -------------------------------------------------------------------------- */

static float Attitude_ComputeAccelCorrectionWeight(float accel_norm_g)
{
    if ((accel_norm_g >= APP_ATTITUDE_ACCEL_FULL_WEIGHT_MIN_G) &&
        (accel_norm_g <= APP_ATTITUDE_ACCEL_FULL_WEIGHT_MAX_G))
    {
        return 1.0f;
    }

    if ((accel_norm_g <= APP_ATTITUDE_ACCEL_ZERO_WEIGHT_MIN_G) ||
        (accel_norm_g >= APP_ATTITUDE_ACCEL_ZERO_WEIGHT_MAX_G))
    {
        return 0.0f;
    }

    if (accel_norm_g < APP_ATTITUDE_ACCEL_FULL_WEIGHT_MIN_G)
    {
        return
            (accel_norm_g - APP_ATTITUDE_ACCEL_ZERO_WEIGHT_MIN_G) /
            (APP_ATTITUDE_ACCEL_FULL_WEIGHT_MIN_G -
             APP_ATTITUDE_ACCEL_ZERO_WEIGHT_MIN_G);
    }

    return
        (APP_ATTITUDE_ACCEL_ZERO_WEIGHT_MAX_G - accel_norm_g) /
        (APP_ATTITUDE_ACCEL_ZERO_WEIGHT_MAX_G -
         APP_ATTITUDE_ACCEL_FULL_WEIGHT_MAX_G);
}

/* -------------------------------------------------------------------------- */

static uint8_t Attitude_ResetFromAccelerometer(
    float accel_x_g,
    float accel_y_g,
    float accel_z_g,
    uint32_t timestamp_us,
    uint8_t gap_reset
)
{
    float norm_squared =
        (accel_x_g * accel_x_g) +
        (accel_y_g * accel_y_g) +
        (accel_z_g * accel_z_g);

    if ((!Attitude_IsFinite(norm_squared)) ||
        (norm_squared < 0.25f) ||
        (norm_squared > 4.0f))
    {
        attitude_data.healthy = 0U;
        attitude_data.numerical_error_count++;
        return 0U;
    }

    float inverse_norm = 1.0f / sqrtf(norm_squared);
    float ax = accel_x_g * inverse_norm;
    float ay = accel_y_g * inverse_norm;
    float az = accel_z_g * inverse_norm;

    /*
     * Build the minimum-angle quaternion rotating measured body-frame
     * specific-force direction onto world +Z. Initial yaw is therefore zero.
     */
    if (az > -0.9999f)
    {
        attitude_data.q_w = 1.0f + az;
        attitude_data.q_x = ay;
        attitude_data.q_y = -ax;
        attitude_data.q_z = 0.0f;
    }
    else
    {
        /* 180-degree case: choose body X as the rotation axis. */
        attitude_data.q_w = 0.0f;
        attitude_data.q_x = 1.0f;
        attitude_data.q_y = 0.0f;
        attitude_data.q_z = 0.0f;
    }

    if (Attitude_NormalizeQuaternion() == 0U)
    {
        attitude_data.q_w = 1.0f;
        attitude_data.q_x = 0.0f;
        attitude_data.q_y = 0.0f;
        attitude_data.q_z = 0.0f;
        attitude_data.healthy = 0U;
        attitude_data.numerical_error_count++;
        return 0U;
    }

    attitude_data.integral_feedback_x_rad_s = 0.0f;
    attitude_data.integral_feedback_y_rad_s = 0.0f;
    attitude_data.integral_feedback_z_rad_s = 0.0f;

    attitude_data.initialized = 1U;
    attitude_data.healthy = 1U;
    attitude_data.last_timestamp_us = timestamp_us;
    attitude_data.last_dt_us = 0UL;
    attitude_data.reset_count++;

    if (gap_reset != 0U)
    {
        attitude_data.gap_reset_count++;
    }

    Attitude_UpdateEulerAngles();
    Attitude_UpdateWorldAcceleration(accel_x_g, accel_y_g, accel_z_g);
    return 1U;
}

/* -------------------------------------------------------------------------- */

void AttitudeEstimator_Init(void)
{
    attitude_data.enabled =
        (APP_ATTITUDE_ESTIMATOR_ENABLED != 0U) ? 1U : 0U;

    attitude_data.initialized = 0U;
    attitude_data.healthy = 0U;
    attitude_data.accel_correction_active = 0U;

    attitude_data.q_w = 1.0f;
    attitude_data.q_x = 0.0f;
    attitude_data.q_y = 0.0f;
    attitude_data.q_z = 0.0f;

    attitude_data.roll_deg = 0.0f;
    attitude_data.pitch_deg = 0.0f;
    attitude_data.yaw_deg = 0.0f;

    attitude_data.world_specific_force_x_g = 0.0f;
    attitude_data.world_specific_force_y_g = 0.0f;
    attitude_data.world_specific_force_z_g = 0.0f;

    attitude_data.world_linear_accel_x_mps2 = 0.0f;
    attitude_data.world_linear_accel_y_mps2 = 0.0f;
    attitude_data.world_linear_accel_z_mps2 = 0.0f;

    attitude_data.accel_correction_weight = 0.0f;

    attitude_data.integral_feedback_x_rad_s = 0.0f;
    attitude_data.integral_feedback_y_rad_s = 0.0f;
    attitude_data.integral_feedback_z_rad_s = 0.0f;

    attitude_data.update_count = 0UL;
    attitude_data.euler_update_count = 0UL;
    attitude_data.duplicate_skip_count = 0UL;
    attitude_data.accel_correction_count = 0UL;
    attitude_data.accel_reject_count = 0UL;
    attitude_data.reset_count = 0UL;
    attitude_data.numerical_error_count = 0UL;
    attitude_data.gap_reset_count = 0UL;

    attitude_data.last_sensor_update_count = 0UL;
    attitude_data.last_timestamp_us = 0UL;
    attitude_data.last_dt_us = 0UL;
    attitude_data.max_dt_us = 0UL;

    euler_decimation_counter = 0UL;
    live_debug_decimation_counter = 0UL;
    Attitude_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */

void AttitudeEstimator_Update(void)
{
#if (APP_ATTITUDE_ESTIMATOR_ENABLED != 0U)
    const SensorData_t *sensor = SensorManager_GetDataPtr();
    uint32_t now_us = micros();

    if ((sensor->imu_valid == 0U) ||
        (sensor->accel_filtered_norm_g <= 0.0f))
    {
        if ((attitude_data.last_timestamp_us != 0UL) &&
            ((now_us - attitude_data.last_timestamp_us) >
             APP_ATTITUDE_STALE_TIMEOUT_US))
        {
            attitude_data.healthy = 0U;
        }

        Attitude_UpdateLiveDebug();
        return;
    }

    if (sensor->update_count == attitude_data.last_sensor_update_count)
    {
        attitude_data.duplicate_skip_count++;

        if ((attitude_data.last_timestamp_us != 0UL) &&
            ((now_us - attitude_data.last_timestamp_us) >
             APP_ATTITUDE_STALE_TIMEOUT_US))
        {
            attitude_data.healthy = 0U;
        }

        Attitude_UpdateLiveDebug();
        return;
    }

    attitude_data.last_sensor_update_count = sensor->update_count;

    if (attitude_data.initialized == 0U)
    {
        (void)Attitude_ResetFromAccelerometer(
            sensor->accel_x_filtered_g,
            sensor->accel_y_filtered_g,
            sensor->accel_z_filtered_g,
            sensor->timestamp_us,
            0U
        );

        Attitude_UpdateLiveDebug();
        return;
    }

    uint32_t dt_us = sensor->timestamp_us - attitude_data.last_timestamp_us;
    attitude_data.last_dt_us = dt_us;

    if (dt_us > attitude_data.max_dt_us)
    {
        attitude_data.max_dt_us = dt_us;
    }

    if ((dt_us == 0UL) ||
        (dt_us > APP_ATTITUDE_MAX_UPDATE_GAP_US))
    {
        (void)Attitude_ResetFromAccelerometer(
            sensor->accel_x_filtered_g,
            sensor->accel_y_filtered_g,
            sensor->accel_z_filtered_g,
            sensor->timestamp_us,
            1U
        );

        Attitude_UpdateLiveDebug();
        return;
    }

    float dt_s = (float)dt_us * 0.000001f;

    float gyro_x_rad_s =
        sensor->gyro_x_filtered_dps * ATTITUDE_DEG_TO_RAD;
    float gyro_y_rad_s =
        sensor->gyro_y_filtered_dps * ATTITUDE_DEG_TO_RAD;
    float gyro_z_rad_s =
        sensor->gyro_z_filtered_dps * ATTITUDE_DEG_TO_RAD;

    float accel_norm_g = sensor->accel_filtered_norm_g;
    float accel_weight =
        Attitude_ComputeAccelCorrectionWeight(accel_norm_g);

    attitude_data.accel_correction_weight = accel_weight;
    attitude_data.accel_correction_active =
        (accel_weight > 0.0f) ? 1U : 0U;

    if (accel_weight > 0.0f)
    {
        float inverse_accel_norm = 1.0f / accel_norm_g;
        float measured_up_x =
            sensor->accel_x_filtered_g * inverse_accel_norm;
        float measured_up_y =
            sensor->accel_y_filtered_g * inverse_accel_norm;
        float measured_up_z =
            sensor->accel_z_filtered_g * inverse_accel_norm;

        float qw = attitude_data.q_w;
        float qx = attitude_data.q_x;
        float qy = attitude_data.q_y;
        float qz = attitude_data.q_z;

        /* World +Z expressed in the body frame: R_body_to_world^T * e_z. */
        float predicted_up_x = 2.0f * ((qx * qz) - (qw * qy));
        float predicted_up_y = 2.0f * ((qy * qz) + (qw * qx));
        float predicted_up_z =
            1.0f - (2.0f * ((qx * qx) + (qy * qy)));

        /* measured x predicted gives the stabilizing body-frame error. */
        float error_x =
            (measured_up_y * predicted_up_z) -
            (measured_up_z * predicted_up_y);

        float error_y =
            (measured_up_z * predicted_up_x) -
            (measured_up_x * predicted_up_z);

        float error_z =
            (measured_up_x * predicted_up_y) -
            (measured_up_y * predicted_up_x);

        float integral_limit_rad_s =
            APP_ATTITUDE_INTEGRAL_LIMIT_DPS * ATTITUDE_DEG_TO_RAD;

        attitude_data.integral_feedback_x_rad_s +=
            APP_ATTITUDE_KI * accel_weight * error_x * dt_s;
        attitude_data.integral_feedback_y_rad_s +=
            APP_ATTITUDE_KI * accel_weight * error_y * dt_s;
        attitude_data.integral_feedback_z_rad_s +=
            APP_ATTITUDE_KI * accel_weight * error_z * dt_s;

        attitude_data.integral_feedback_x_rad_s =
            Attitude_Clamp(
                attitude_data.integral_feedback_x_rad_s,
                -integral_limit_rad_s,
                integral_limit_rad_s
            );

        attitude_data.integral_feedback_y_rad_s =
            Attitude_Clamp(
                attitude_data.integral_feedback_y_rad_s,
                -integral_limit_rad_s,
                integral_limit_rad_s
            );

        attitude_data.integral_feedback_z_rad_s =
            Attitude_Clamp(
                attitude_data.integral_feedback_z_rad_s,
                -integral_limit_rad_s,
                integral_limit_rad_s
            );

        gyro_x_rad_s +=
            (APP_ATTITUDE_KP * accel_weight * error_x) +
            attitude_data.integral_feedback_x_rad_s;

        gyro_y_rad_s +=
            (APP_ATTITUDE_KP * accel_weight * error_y) +
            attitude_data.integral_feedback_y_rad_s;

        gyro_z_rad_s +=
            (APP_ATTITUDE_KP * accel_weight * error_z) +
            attitude_data.integral_feedback_z_rad_s;

        attitude_data.accel_correction_count++;
    }
    else
    {
        attitude_data.accel_reject_count++;
    }

    float qw = attitude_data.q_w;
    float qx = attitude_data.q_x;
    float qy = attitude_data.q_y;
    float qz = attitude_data.q_z;

    float half_dt = 0.5f * dt_s;

    attitude_data.q_w +=
        (-qx * gyro_x_rad_s -
         qy * gyro_y_rad_s -
         qz * gyro_z_rad_s) * half_dt;

    attitude_data.q_x +=
        (qw * gyro_x_rad_s +
         qy * gyro_z_rad_s -
         qz * gyro_y_rad_s) * half_dt;

    attitude_data.q_y +=
        (qw * gyro_y_rad_s -
         qx * gyro_z_rad_s +
         qz * gyro_x_rad_s) * half_dt;

    attitude_data.q_z +=
        (qw * gyro_z_rad_s +
         qx * gyro_y_rad_s -
         qy * gyro_x_rad_s) * half_dt;

    if (Attitude_NormalizeQuaternion() == 0U)
    {
        attitude_data.numerical_error_count++;

        (void)Attitude_ResetFromAccelerometer(
            sensor->accel_x_filtered_g,
            sensor->accel_y_filtered_g,
            sensor->accel_z_filtered_g,
            sensor->timestamp_us,
            1U
        );

        Attitude_UpdateLiveDebug();
        return;
    }

    Attitude_UpdateWorldAcceleration(
        sensor->accel_x_filtered_g,
        sensor->accel_y_filtered_g,
        sensor->accel_z_filtered_g
    );

    euler_decimation_counter++;

    if (euler_decimation_counter >= APP_ATTITUDE_EULER_DECIMATION)
    {
        euler_decimation_counter = 0UL;
        Attitude_UpdateEulerAngles();
    }

    attitude_data.last_timestamp_us = sensor->timestamp_us;
    attitude_data.update_count++;
    attitude_data.healthy = 1U;

    /* P32 UART-only debug: keep the estimator at 1 kHz but mirror the legacy
     * volatile Live-Expressions block only at 200 Hz. Safety/failure paths
     * above still publish immediately. */
    live_debug_decimation_counter++;
    if (live_debug_decimation_counter >= 5UL)
    {
        live_debug_decimation_counter = 0UL;
        Attitude_UpdateLiveDebug();
    }
#else
    attitude_data.enabled = 0U;
    attitude_data.healthy = 0U;
    Attitude_UpdateLiveDebug();
#endif
}

/* -------------------------------------------------------------------------- */

AttitudeEstimatorData_t AttitudeEstimator_GetData(void)
{
    return attitude_data;
}

const AttitudeEstimatorData_t *AttitudeEstimator_GetDataPtr(void)
{
    return &attitude_data;
}

/* -------------------------------------------------------------------------- */

uint8_t AttitudeEstimator_IsHealthy(void)
{
    return attitude_data.healthy;
}
