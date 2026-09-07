#include "Modules/Estimation/VerticalEKF/vertical_ekf.h"

#include "Common/app_config.h"
#include "Modules/Sensors/Barometer/barometer.h"
#include "Modules/Sensors/Lidar/Lidar.h"
#include "Modules/Estimation/AttitudeEstimator/attitude_estimator.h"
#include "Services/Timebase/timebase.h"

#include <stdint.h>

#define VERTICAL_EKF_STATE_COUNT 3U
#define VERTICAL_EKF_Z_INDEX     0U
#define VERTICAL_EKF_V_INDEX     1U
#define VERTICAL_EKF_BIAS_INDEX  2U

static VerticalEKFData_t ekf_data;

static float ekf_state[VERTICAL_EKF_STATE_COUNT];
static float ekf_covariance[VERTICAL_EKF_STATE_COUNT][VERTICAL_EKF_STATE_COUNT];

static uint32_t last_baro_source_update_count = 0UL;
static uint32_t last_lidar_source_update_count = 0UL;

static float baro_reference_sum = 0.0f;
static float lidar_reference_sum = 0.0f;
static float lidar_reference_last_sample_m = 0.0f;

/* -------------------------------------------------------------------------- */
/* Live Expressions                                                           */
/* -------------------------------------------------------------------------- */

volatile uint8_t vertical_ekf_enabled = 0U;
volatile uint8_t vertical_ekf_initialized = 0U;
volatile uint8_t vertical_ekf_healthy = 0U;
volatile uint8_t vertical_ekf_baro_reference_ready = 0U;
volatile uint8_t vertical_ekf_lidar_reference_ready = 0U;
volatile uint8_t vertical_ekf_imu_aiding_active = 0U;

volatile float vertical_ekf_altitude_m = 0.0f;
volatile float vertical_ekf_velocity_mps = 0.0f;
volatile float vertical_ekf_acceleration_mps2 = 0.0f;
volatile float vertical_ekf_imu_accel_input_mps2 = 0.0f;
volatile float vertical_ekf_accel_bias_mps2 = 0.0f;

volatile float vertical_ekf_baro_reference_m = 0.0f;
volatile float vertical_ekf_lidar_reference_m = 0.0f;
volatile float vertical_ekf_baro_innovation_m = 0.0f;
volatile float vertical_ekf_lidar_innovation_m = 0.0f;

volatile float vertical_ekf_p00 = 0.0f;
volatile float vertical_ekf_p11 = 0.0f;
volatile float vertical_ekf_p22 = 0.0f;

volatile uint32_t vertical_ekf_predict_count = 0UL;
volatile uint32_t vertical_ekf_baro_update_count = 0UL;
volatile uint32_t vertical_ekf_lidar_update_count = 0UL;
volatile uint32_t vertical_ekf_baro_reject_count = 0UL;
volatile uint32_t vertical_ekf_lidar_reject_count = 0UL;
volatile uint32_t vertical_ekf_imu_aiding_accept_count = 0UL;
volatile uint32_t vertical_ekf_imu_aiding_reject_count = 0UL;
volatile uint32_t vertical_ekf_reset_count = 0UL;
volatile uint32_t vertical_ekf_numerical_error_count = 0UL;
volatile uint32_t vertical_ekf_predict_gap_reset_count = 0UL;
volatile uint32_t vertical_ekf_last_dt_us = 0UL;
volatile uint32_t vertical_ekf_max_dt_us = 0UL;

/* -------------------------------------------------------------------------- */

static float VerticalEKF_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* -------------------------------------------------------------------------- */

static uint8_t VerticalEKF_IsFiniteReasonable(float value)
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

static void VerticalEKF_UpdateLiveDebug(void)
{
    vertical_ekf_enabled = ekf_data.enabled;
    vertical_ekf_initialized = ekf_data.initialized;
    vertical_ekf_healthy = ekf_data.healthy;

    vertical_ekf_baro_reference_ready = ekf_data.baro_reference_ready;
    vertical_ekf_lidar_reference_ready = ekf_data.lidar_reference_ready;
    vertical_ekf_imu_aiding_active = ekf_data.imu_aiding_active;

    vertical_ekf_altitude_m = ekf_data.altitude_m;
    vertical_ekf_velocity_mps = ekf_data.vertical_velocity_mps;
    vertical_ekf_acceleration_mps2 = ekf_data.vertical_acceleration_mps2;
    vertical_ekf_imu_accel_input_mps2 = ekf_data.imu_accel_input_mps2;
    vertical_ekf_accel_bias_mps2 = ekf_data.accel_bias_mps2;

    vertical_ekf_baro_reference_m = ekf_data.baro_reference_m;
    vertical_ekf_lidar_reference_m = ekf_data.lidar_reference_m;

    vertical_ekf_baro_innovation_m = ekf_data.baro_innovation_m;
    vertical_ekf_lidar_innovation_m = ekf_data.lidar_innovation_m;

    vertical_ekf_p00 = ekf_data.covariance_altitude;
    vertical_ekf_p11 = ekf_data.covariance_velocity;
    vertical_ekf_p22 = ekf_data.covariance_accel_bias;

    vertical_ekf_predict_count = ekf_data.predict_count;
    vertical_ekf_baro_update_count = ekf_data.baro_update_count;
    vertical_ekf_lidar_update_count = ekf_data.lidar_update_count;

    vertical_ekf_baro_reject_count = ekf_data.baro_reject_count;
    vertical_ekf_lidar_reject_count = ekf_data.lidar_reject_count;
    vertical_ekf_imu_aiding_accept_count =
        ekf_data.imu_aiding_accept_count;
    vertical_ekf_imu_aiding_reject_count =
        ekf_data.imu_aiding_reject_count;

    vertical_ekf_reset_count = ekf_data.reset_count;
    vertical_ekf_numerical_error_count = ekf_data.numerical_error_count;
    vertical_ekf_predict_gap_reset_count = ekf_data.predict_gap_reset_count;

    vertical_ekf_last_dt_us = ekf_data.last_dt_us;
    vertical_ekf_max_dt_us = ekf_data.max_dt_us;
}

/* -------------------------------------------------------------------------- */

static void VerticalEKF_SetInitialCovariance(void)
{
    uint8_t row;
    uint8_t column;

    for (row = 0U; row < VERTICAL_EKF_STATE_COUNT; row++)
    {
        for (column = 0U; column < VERTICAL_EKF_STATE_COUNT; column++)
        {
            ekf_covariance[row][column] = 0.0f;
        }
    }

    ekf_covariance[VERTICAL_EKF_Z_INDEX][VERTICAL_EKF_Z_INDEX] =
        APP_VERTICAL_EKF_INITIAL_ALT_VAR_M2;

    ekf_covariance[VERTICAL_EKF_V_INDEX][VERTICAL_EKF_V_INDEX] =
        APP_VERTICAL_EKF_INITIAL_VEL_VAR_M2S2;

    ekf_covariance[VERTICAL_EKF_BIAS_INDEX][VERTICAL_EKF_BIAS_INDEX] =
        APP_VERTICAL_EKF_INITIAL_BIAS_VAR_M2S4;
}

/* -------------------------------------------------------------------------- */

static void VerticalEKF_InitializeState(float altitude_m, uint32_t now_us)
{
    ekf_state[VERTICAL_EKF_Z_INDEX] = altitude_m;
    ekf_state[VERTICAL_EKF_V_INDEX] = 0.0f;
    ekf_state[VERTICAL_EKF_BIAS_INDEX] = 0.0f;

    VerticalEKF_SetInitialCovariance();

    ekf_data.initialized = 1U;
    ekf_data.healthy = 1U;
    ekf_data.altitude_m = altitude_m;
    ekf_data.vertical_velocity_mps = 0.0f;
    ekf_data.vertical_acceleration_mps2 = 0.0f;
    ekf_data.imu_accel_input_mps2 = 0.0f;
    ekf_data.accel_bias_mps2 = 0.0f;
    ekf_data.imu_aiding_active = 0U;

    ekf_data.covariance_altitude =
        ekf_covariance[VERTICAL_EKF_Z_INDEX][VERTICAL_EKF_Z_INDEX];
    ekf_data.covariance_velocity =
        ekf_covariance[VERTICAL_EKF_V_INDEX][VERTICAL_EKF_V_INDEX];
    ekf_data.covariance_accel_bias =
        ekf_covariance[VERTICAL_EKF_BIAS_INDEX][VERTICAL_EKF_BIAS_INDEX];

    ekf_data.last_predict_timestamp_us = now_us;
    ekf_data.last_correction_timestamp_us = now_us;
    ekf_data.last_dt_us = 0UL;
    ekf_data.reset_count++;
}

/* -------------------------------------------------------------------------- */

static void VerticalEKF_ResetDynamics(uint32_t now_us)
{
    ekf_state[VERTICAL_EKF_V_INDEX] = 0.0f;

    /*
     * Preserve the learned acceleration bias across a scheduler gap.
     * Only dynamic velocity is reset.
     */
    VerticalEKF_SetInitialCovariance();

    ekf_data.last_predict_timestamp_us = now_us;
    ekf_data.last_dt_us = 0UL;
    ekf_data.reset_count++;
    ekf_data.predict_gap_reset_count++;
}

/* -------------------------------------------------------------------------- */

static void VerticalEKF_Predict(
    float dt_s,
    float imu_accel_input_mps2,
    uint8_t imu_aiding_active
)
{
    float transition[VERTICAL_EKF_STATE_COUNT][VERTICAL_EKF_STATE_COUNT];
    float temporary[VERTICAL_EKF_STATE_COUNT][VERTICAL_EKF_STATE_COUNT];
    float predicted_covariance[VERTICAL_EKF_STATE_COUNT][VERTICAL_EKF_STATE_COUNT];
    float process_noise[VERTICAL_EKF_STATE_COUNT][VERTICAL_EKF_STATE_COUNT];

    float dt2 = dt_s * dt_s;
    float dt3 = dt2 * dt_s;
    float dt4 = dt2 * dt2;

    float accel_variance =
        APP_VERTICAL_EKF_IMU_ACCEL_STD_MPS2 *
        APP_VERTICAL_EKF_IMU_ACCEL_STD_MPS2;

    float bias_rw_variance =
        APP_VERTICAL_EKF_ACCEL_BIAS_RW_STD_MPS3 *
        APP_VERTICAL_EKF_ACCEL_BIAS_RW_STD_MPS3;

    float corrected_acceleration_mps2 = 0.0f;

    uint8_t row;
    uint8_t column;
    uint8_t index;

    if (imu_aiding_active != 0U)
    {
        corrected_acceleration_mps2 =
            imu_accel_input_mps2 -
            ekf_state[VERTICAL_EKF_BIAS_INDEX];

        ekf_state[VERTICAL_EKF_Z_INDEX] +=
            (ekf_state[VERTICAL_EKF_V_INDEX] * dt_s) +
            (0.5f * corrected_acceleration_mps2 * dt2);

        ekf_state[VERTICAL_EKF_V_INDEX] +=
            corrected_acceleration_mps2 * dt_s;

        transition[0][0] = 1.0f;
        transition[0][1] = dt_s;
        transition[0][2] = -0.5f * dt2;

        transition[1][0] = 0.0f;
        transition[1][1] = 1.0f;
        transition[1][2] = -dt_s;

        transition[2][0] = 0.0f;
        transition[2][1] = 0.0f;
        transition[2][2] = 1.0f;

        ekf_data.imu_aiding_accept_count++;
    }
    else
    {
        /*
         * Safe fallback: constant-velocity prediction without using a stale or
         * invalid attitude-derived acceleration sample.
         */
        ekf_state[VERTICAL_EKF_Z_INDEX] +=
            ekf_state[VERTICAL_EKF_V_INDEX] * dt_s;

        transition[0][0] = 1.0f;
        transition[0][1] = dt_s;
        transition[0][2] = 0.0f;

        transition[1][0] = 0.0f;
        transition[1][1] = 1.0f;
        transition[1][2] = 0.0f;

        transition[2][0] = 0.0f;
        transition[2][1] = 0.0f;
        transition[2][2] = 1.0f;

        ekf_data.imu_aiding_reject_count++;
    }

    /*
     * Acceleration input noise integrated into altitude and velocity.
     */
    process_noise[0][0] = accel_variance * (0.25f * dt4);
    process_noise[0][1] = accel_variance * (0.5f * dt3);
    process_noise[0][2] = 0.0f;

    process_noise[1][0] = process_noise[0][1];
    process_noise[1][1] = accel_variance * dt2;
    process_noise[1][2] = 0.0f;

    process_noise[2][0] = 0.0f;
    process_noise[2][1] = 0.0f;
    process_noise[2][2] = bias_rw_variance * dt_s;

    for (row = 0U; row < VERTICAL_EKF_STATE_COUNT; row++)
    {
        for (column = 0U; column < VERTICAL_EKF_STATE_COUNT; column++)
        {
            temporary[row][column] = 0.0f;

            for (index = 0U; index < VERTICAL_EKF_STATE_COUNT; index++)
            {
                temporary[row][column] +=
                    transition[row][index] * ekf_covariance[index][column];
            }
        }
    }

    for (row = 0U; row < VERTICAL_EKF_STATE_COUNT; row++)
    {
        for (column = 0U; column < VERTICAL_EKF_STATE_COUNT; column++)
        {
            predicted_covariance[row][column] = process_noise[row][column];

            for (index = 0U; index < VERTICAL_EKF_STATE_COUNT; index++)
            {
                predicted_covariance[row][column] +=
                    temporary[row][index] * transition[column][index];
            }
        }
    }

    for (row = 0U; row < VERTICAL_EKF_STATE_COUNT; row++)
    {
        for (column = 0U; column < VERTICAL_EKF_STATE_COUNT; column++)
        {
            ekf_covariance[row][column] = predicted_covariance[row][column];
        }
    }

    ekf_data.imu_aiding_active = imu_aiding_active;
    ekf_data.imu_accel_input_mps2 = imu_accel_input_mps2;
    ekf_data.vertical_acceleration_mps2 = corrected_acceleration_mps2;
    ekf_data.predict_count++;
}

/* -------------------------------------------------------------------------- */

static uint8_t VerticalEKF_CorrectAltitude(
    float measurement_m,
    float measurement_variance,
    float gate_sigma,
    float absolute_gate_m,
    uint8_t is_lidar,
    uint32_t now_us
)
{
    float innovation;
    float innovation_covariance;
    float kalman_gain[VERTICAL_EKF_STATE_COUNT];
    float identity_minus_kh[VERTICAL_EKF_STATE_COUNT][VERTICAL_EKF_STATE_COUNT];
    float temporary[VERTICAL_EKF_STATE_COUNT][VERTICAL_EKF_STATE_COUNT];
    float corrected_covariance[VERTICAL_EKF_STATE_COUNT][VERTICAL_EKF_STATE_COUNT];
    float gate_sigma_squared;

    uint8_t row;
    uint8_t column;
    uint8_t index;

    innovation = measurement_m - ekf_state[VERTICAL_EKF_Z_INDEX];
    innovation_covariance =
        ekf_covariance[VERTICAL_EKF_Z_INDEX][VERTICAL_EKF_Z_INDEX] +
        measurement_variance;

    if (is_lidar != 0U)
    {
        ekf_data.lidar_innovation_m = innovation;
    }
    else
    {
        ekf_data.baro_innovation_m = innovation;
    }

    if (innovation_covariance <= 0.0000001f)
    {
        ekf_data.numerical_error_count++;
        return 0U;
    }

    gate_sigma_squared = gate_sigma * gate_sigma;

    if ((VerticalEKF_Abs(innovation) > absolute_gate_m) &&
        ((innovation * innovation) >
         (gate_sigma_squared * innovation_covariance)))
    {
        if (is_lidar != 0U)
        {
            ekf_data.lidar_reject_count++;
        }
        else
        {
            ekf_data.baro_reject_count++;
        }

        return 0U;
    }

    for (row = 0U; row < VERTICAL_EKF_STATE_COUNT; row++)
    {
        kalman_gain[row] =
            ekf_covariance[row][VERTICAL_EKF_Z_INDEX] /
            innovation_covariance;
    }

    for (row = 0U; row < VERTICAL_EKF_STATE_COUNT; row++)
    {
        ekf_state[row] += kalman_gain[row] * innovation;
    }

    for (row = 0U; row < VERTICAL_EKF_STATE_COUNT; row++)
    {
        for (column = 0U; column < VERTICAL_EKF_STATE_COUNT; column++)
        {
            identity_minus_kh[row][column] =
                (row == column) ? 1.0f : 0.0f;
        }

        identity_minus_kh[row][VERTICAL_EKF_Z_INDEX] -= kalman_gain[row];
    }

    for (row = 0U; row < VERTICAL_EKF_STATE_COUNT; row++)
    {
        for (column = 0U; column < VERTICAL_EKF_STATE_COUNT; column++)
        {
            temporary[row][column] = 0.0f;

            for (index = 0U; index < VERTICAL_EKF_STATE_COUNT; index++)
            {
                temporary[row][column] +=
                    identity_minus_kh[row][index] *
                    ekf_covariance[index][column];
            }
        }
    }

    for (row = 0U; row < VERTICAL_EKF_STATE_COUNT; row++)
    {
        for (column = 0U; column < VERTICAL_EKF_STATE_COUNT; column++)
        {
            corrected_covariance[row][column] =
                kalman_gain[row] * measurement_variance * kalman_gain[column];

            for (index = 0U; index < VERTICAL_EKF_STATE_COUNT; index++)
            {
                corrected_covariance[row][column] +=
                    temporary[row][index] *
                    identity_minus_kh[column][index];
            }
        }
    }

    for (row = 0U; row < VERTICAL_EKF_STATE_COUNT; row++)
    {
        for (column = 0U; column < VERTICAL_EKF_STATE_COUNT; column++)
        {
            ekf_covariance[row][column] = corrected_covariance[row][column];
        }
    }

    if (is_lidar != 0U)
    {
        ekf_data.lidar_update_count++;
    }
    else
    {
        ekf_data.baro_update_count++;
    }

    ekf_data.last_correction_timestamp_us = now_us;
    return 1U;
}

/* -------------------------------------------------------------------------- */

static void VerticalEKF_ProcessBarometer(
    const BarometerData_t *barometer,
    uint32_t now_us
)
{
    float measurement_m;

    if (barometer->update_count == last_baro_source_update_count)
    {
        return;
    }

    last_baro_source_update_count = barometer->update_count;

    if ((barometer->healthy == 0U) ||
        (barometer->pressure_valid == 0U) ||
        (barometer->calibrated == 0U) ||
        (barometer->filter_initialized == 0U))
    {
        return;
    }

    if (ekf_data.baro_reference_ready == 0U)
    {
        baro_reference_sum += barometer->filtered_altitude_m;
        ekf_data.baro_reference_sample_count++;

        if (ekf_data.baro_reference_sample_count >=
            APP_VERTICAL_EKF_BARO_REFERENCE_SAMPLES)
        {
            ekf_data.baro_reference_m =
                baro_reference_sum /
                (float)ekf_data.baro_reference_sample_count;

            ekf_data.baro_reference_ready = 1U;

            if (ekf_data.initialized == 0U)
            {
                VerticalEKF_InitializeState(0.0f, now_us);
            }
        }

        return;
    }

    measurement_m =
        barometer->filtered_altitude_m - ekf_data.baro_reference_m;

    if (ekf_data.initialized == 0U)
    {
        VerticalEKF_InitializeState(measurement_m, now_us);
        return;
    }

    (void)VerticalEKF_CorrectAltitude(
        measurement_m,
        APP_VERTICAL_EKF_BARO_STD_M * APP_VERTICAL_EKF_BARO_STD_M,
        APP_VERTICAL_EKF_BARO_GATE_SIGMA,
        APP_VERTICAL_EKF_BARO_ABS_GATE_M,
        0U,
        now_us
    );
}

/* -------------------------------------------------------------------------- */

static void VerticalEKF_ProcessLidar(
    const LidarData_t *lidar,
    uint32_t now_us
)
{
    float measurement_m;
    float distance_m;

    if (lidar->update_count == last_lidar_source_update_count)
    {
        return;
    }

    last_lidar_source_update_count = lidar->update_count;

    if ((lidar->connected == 0U) ||
        (lidar->distance_valid == 0U) ||
        (lidar->filter_initialized == 0U))
    {
        return;
    }

    distance_m = lidar->filtered_distance_m;

    if ((distance_m < APP_VERTICAL_EKF_LIDAR_MIN_M) ||
        (distance_m > APP_VERTICAL_EKF_LIDAR_MAX_M))
    {
        return;
    }

    if (ekf_data.lidar_reference_ready == 0U)
    {
        if ((ekf_data.lidar_reference_sample_count > 0UL) &&
            (VerticalEKF_Abs(
                distance_m - lidar_reference_last_sample_m
             ) > APP_VERTICAL_EKF_LIDAR_REFERENCE_JUMP_M))
        {
            lidar_reference_sum = distance_m;
            ekf_data.lidar_reference_sample_count = 1UL;
        }
        else
        {
            lidar_reference_sum += distance_m;
            ekf_data.lidar_reference_sample_count++;
        }

        lidar_reference_last_sample_m = distance_m;

        if (ekf_data.lidar_reference_sample_count >=
            APP_VERTICAL_EKF_LIDAR_REFERENCE_SAMPLES)
        {
            ekf_data.lidar_reference_m =
                lidar_reference_sum /
                (float)ekf_data.lidar_reference_sample_count;

            ekf_data.lidar_reference_ready = 1U;

            if (ekf_data.initialized == 0U)
            {
                VerticalEKF_InitializeState(0.0f, now_us);
            }
        }

        return;
    }

    measurement_m = distance_m - ekf_data.lidar_reference_m;

    if (ekf_data.initialized == 0U)
    {
        VerticalEKF_InitializeState(measurement_m, now_us);
        return;
    }

    (void)VerticalEKF_CorrectAltitude(
        measurement_m,
        APP_VERTICAL_EKF_LIDAR_STD_M * APP_VERTICAL_EKF_LIDAR_STD_M,
        APP_VERTICAL_EKF_LIDAR_GATE_SIGMA,
        APP_VERTICAL_EKF_LIDAR_ABS_GATE_M,
        1U,
        now_us
    );
}

/* -------------------------------------------------------------------------- */

static uint8_t VerticalEKF_CheckNumericalHealth(void)
{
    if ((VerticalEKF_IsFiniteReasonable(ekf_state[0]) == 0U) ||
        (VerticalEKF_IsFiniteReasonable(ekf_state[1]) == 0U) ||
        (VerticalEKF_IsFiniteReasonable(ekf_state[2]) == 0U))
    {
        return 0U;
    }

    if ((ekf_covariance[0][0] <= 0.0f) ||
        (ekf_covariance[1][1] <= 0.0f) ||
        (ekf_covariance[2][2] <= 0.0f))
    {
        return 0U;
    }

    if ((VerticalEKF_IsFiniteReasonable(ekf_covariance[0][0]) == 0U) ||
        (VerticalEKF_IsFiniteReasonable(ekf_covariance[1][1]) == 0U) ||
        (VerticalEKF_IsFiniteReasonable(ekf_covariance[2][2]) == 0U))
    {
        return 0U;
    }

    return 1U;
}

/* -------------------------------------------------------------------------- */

void VerticalEKF_Init(void)
{
    uint8_t row;
    uint8_t column;

    ekf_data.enabled = 0U;
    ekf_data.initialized = 0U;
    ekf_data.healthy = 0U;
    ekf_data.baro_reference_ready = 0U;
    ekf_data.lidar_reference_ready = 0U;
    ekf_data.imu_aiding_active = 0U;

    ekf_data.altitude_m = 0.0f;
    ekf_data.vertical_velocity_mps = 0.0f;
    ekf_data.vertical_acceleration_mps2 = 0.0f;
    ekf_data.imu_accel_input_mps2 = 0.0f;
    ekf_data.accel_bias_mps2 = 0.0f;

    ekf_data.baro_reference_m = 0.0f;
    ekf_data.lidar_reference_m = 0.0f;
    ekf_data.baro_innovation_m = 0.0f;
    ekf_data.lidar_innovation_m = 0.0f;

    ekf_data.covariance_altitude = 0.0f;
    ekf_data.covariance_velocity = 0.0f;
    ekf_data.covariance_accel_bias = 0.0f;

    ekf_data.predict_count = 0UL;
    ekf_data.baro_update_count = 0UL;
    ekf_data.lidar_update_count = 0UL;
    ekf_data.baro_reject_count = 0UL;
    ekf_data.lidar_reject_count = 0UL;
    ekf_data.imu_aiding_accept_count = 0UL;
    ekf_data.imu_aiding_reject_count = 0UL;
    ekf_data.reset_count = 0UL;
    ekf_data.numerical_error_count = 0UL;
    ekf_data.predict_gap_reset_count = 0UL;
    ekf_data.baro_reference_sample_count = 0UL;
    ekf_data.lidar_reference_sample_count = 0UL;
    ekf_data.last_predict_timestamp_us = 0UL;
    ekf_data.last_correction_timestamp_us = 0UL;
    ekf_data.last_dt_us = 0UL;
    ekf_data.max_dt_us = 0UL;

    for (row = 0U; row < VERTICAL_EKF_STATE_COUNT; row++)
    {
        ekf_state[row] = 0.0f;

        for (column = 0U; column < VERTICAL_EKF_STATE_COUNT; column++)
        {
            ekf_covariance[row][column] = 0.0f;
        }
    }

    last_baro_source_update_count = 0UL;
    last_lidar_source_update_count = 0UL;

    baro_reference_sum = 0.0f;
    lidar_reference_sum = 0.0f;
    lidar_reference_last_sample_m = 0.0f;

#if (APP_VERTICAL_EKF_ENABLED != 0U)
    ekf_data.enabled = 1U;
#endif

    VerticalEKF_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */

void VerticalEKF_Reset(void)
{
    uint8_t enabled = ekf_data.enabled;
    uint32_t previous_reset_count = ekf_data.reset_count;

    VerticalEKF_Init();

    ekf_data.enabled = enabled;
    ekf_data.reset_count = previous_reset_count + 1UL;

    VerticalEKF_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */

void VerticalEKF_Update(void)
{
#if (APP_VERTICAL_EKF_ENABLED != 0U)
    uint32_t now_us = micros();
    uint32_t dt_us;
    float dt_s;

    BarometerData_t barometer = Barometer_GetData();
    LidarData_t lidar = Lidar_GetData();
    AttitudeEstimatorData_t attitude = AttitudeEstimator_GetData();

    float imu_accel_input_mps2 = 0.0f;
    uint8_t imu_aiding_active = 0U;

#if (APP_VERTICAL_EKF_IMU_AIDING_ENABLED != 0U)
    if ((attitude.enabled != 0U) &&
        (attitude.initialized != 0U) &&
        (attitude.healthy != 0U) &&
        ((now_us - attitude.last_timestamp_us) <=
         APP_VERTICAL_EKF_IMU_STALE_TIMEOUT_US) &&
        (VerticalEKF_IsFiniteReasonable(
            attitude.world_linear_accel_z_mps2
         ) != 0U) &&
        (VerticalEKF_Abs(
            attitude.world_linear_accel_z_mps2
         ) <= APP_VERTICAL_EKF_IMU_ACCEL_ABS_MAX_MPS2))
    {
        imu_accel_input_mps2 =
            attitude.world_linear_accel_z_mps2;
        imu_aiding_active = 1U;
    }
#endif

    if (ekf_data.enabled == 0U)
    {
        return;
    }

    if (ekf_data.initialized != 0U)
    {
        dt_us = now_us - ekf_data.last_predict_timestamp_us;
        ekf_data.last_dt_us = dt_us;

        if (dt_us > ekf_data.max_dt_us)
        {
            ekf_data.max_dt_us = dt_us;
        }

        if ((ekf_data.last_predict_timestamp_us != 0UL) &&
            (dt_us > APP_VERTICAL_EKF_MAX_PREDICT_GAP_US))
        {
            VerticalEKF_ResetDynamics(now_us);
        }
        else if (dt_us > 0UL)
        {
            dt_s = (float)dt_us * 0.000001f;
            VerticalEKF_Predict(
                dt_s,
                imu_accel_input_mps2,
                imu_aiding_active
            );
            ekf_data.last_predict_timestamp_us = now_us;
        }
    }

    VerticalEKF_ProcessBarometer(&barometer, now_us);
    VerticalEKF_ProcessLidar(&lidar, now_us);

    if (ekf_data.initialized != 0U)
    {
        if (VerticalEKF_CheckNumericalHealth() == 0U)
        {
            ekf_data.numerical_error_count++;
            VerticalEKF_InitializeState(0.0f, now_us);
        }

        ekf_data.altitude_m = ekf_state[VERTICAL_EKF_Z_INDEX];
        ekf_data.vertical_velocity_mps = ekf_state[VERTICAL_EKF_V_INDEX];
        ekf_data.accel_bias_mps2 =
            ekf_state[VERTICAL_EKF_BIAS_INDEX];

        ekf_data.covariance_altitude = ekf_covariance[0][0];
        ekf_data.covariance_velocity = ekf_covariance[1][1];
        ekf_data.covariance_accel_bias = ekf_covariance[2][2];

        ekf_data.healthy =
            ((now_us - ekf_data.last_correction_timestamp_us) <=
             APP_VERTICAL_EKF_STALE_TIMEOUT_US) ? 1U : 0U;
    }

    VerticalEKF_UpdateLiveDebug();
#else
    return;
#endif
}

/* -------------------------------------------------------------------------- */

uint8_t VerticalEKF_IsInitialized(void)
{
    return ekf_data.initialized;
}

/* -------------------------------------------------------------------------- */

uint8_t VerticalEKF_IsHealthy(void)
{
    return ekf_data.healthy;
}

/* -------------------------------------------------------------------------- */

VerticalEKFData_t VerticalEKF_GetData(void)
{
    return ekf_data;
}
