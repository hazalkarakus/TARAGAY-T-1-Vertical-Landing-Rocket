#include "Modules/Sensors/SensorManager/sensor_manager.h"

#ifndef SENSOR_MANAGER_HEADER_V2
#error "Yanlis veya eski sensor_manager.h dosyasi kullaniliyor"
#endif

#include "Common/app_config.h"
#include "Common/Filters/butterworth_filter.h"


#include "Modules/Sensors/IMU/imu.h"
#include "Modules/Sensors/Barometer/barometer.h"

#include "Services/Timebase/timebase.h"

static SensorData_t sensor_data;

static Butterworth2LPF_t gyro_filter_x;
static Butterworth2LPF_t gyro_filter_y;
static Butterworth2LPF_t gyro_filter_z;

static Butterworth2LPF_t accel_filter_x;
static Butterworth2LPF_t accel_filter_y;
static Butterworth2LPF_t accel_filter_z;

static uint32_t imu_filter_last_sample_timestamp_us = 0UL;

/* Startup calibration state; updated only from the 1 kHz task context. */
static uint8_t imu_calibration_complete_state = 0U;
static uint32_t imu_calibration_sample_count_state = 0UL;
static uint32_t imu_calibration_last_sample_timestamp_us = 0UL;
static int64_t imu_calibration_gyro_sum_raw[3] = {0, 0, 0};
static int64_t imu_calibration_gyro_sum_square_raw[3] = {0, 0, 0};
static int64_t imu_calibration_accel_sum_raw[3] = {0, 0, 0};
static int64_t imu_calibration_accel_sum_square_raw[3] = {0, 0, 0};
static float imu_calibration_accel_norm_sum_g = 0.0f;
static float imu_calibration_accel_norm_sum_square_g = 0.0f;
static float imu_gyro_bias_raw[3] = {0.0f, 0.0f, 0.0f};
static float imu_accel_bias_raw[3] = {0.0f, 0.0f, 0.0f};
static uint32_t imu_last_consumed_generation = 0UL;
static uint16_t imu_calibration_consecutive_reject_count = 0U;

/* -------------------------------------------------------------------------- */
/* Live Expressions                                                           */
/* -------------------------------------------------------------------------- */

volatile uint32_t sensor_timestamp_us = 0UL;
volatile uint32_t sensor_update_count = 0UL;

volatile uint8_t sensor_imu_valid = 0U;
volatile uint32_t sensor_imu_valid_update_count = 0UL;
volatile uint32_t sensor_imu_rejected_update_count = 0UL;
volatile uint32_t sensor_imu_last_sample_timestamp_us = 0UL;
volatile uint32_t sensor_imu_sample_age_us = 0xFFFFFFFFUL;
volatile uint32_t sensor_imu_stale_invalidation_count = 0UL;


volatile uint8_t sensor_imu_calibration_complete = 0U;
volatile uint32_t sensor_imu_calibration_sample_count = 0UL;
volatile uint32_t sensor_imu_calibration_reset_count = 0UL;
volatile float sensor_imu_gyro_bias_x_raw = 0.0f;
volatile float sensor_imu_gyro_bias_y_raw = 0.0f;
volatile float sensor_imu_gyro_bias_z_raw = 0.0f;
volatile float sensor_imu_accel_bias_x_raw = 0.0f;
volatile float sensor_imu_accel_bias_y_raw = 0.0f;
volatile float sensor_imu_accel_bias_z_raw = 0.0f;
volatile float sensor_imu_accel_scale = 1.0f;
volatile float sensor_imu_calibration_gyro_stddev_x_dps = 0.0f;
volatile float sensor_imu_calibration_gyro_stddev_y_dps = 0.0f;
volatile float sensor_imu_calibration_gyro_stddev_z_dps = 0.0f;
volatile float sensor_imu_calibration_accel_stddev_g = 0.0f;
volatile float sensor_imu_calibration_accel_stddev_x_g = 0.0f;
volatile float sensor_imu_calibration_accel_stddev_y_g = 0.0f;
volatile float sensor_imu_calibration_accel_stddev_z_g = 0.0f;
volatile uint8_t sensor_imu_calibration_last_reject_reason = 0U;
volatile uint32_t sensor_imu_duplicate_generation_count = 0UL;
volatile uint32_t sensor_imu_last_generation = 0UL;

volatile uint8_t sensor_baro_valid = 0U;
volatile uint8_t sensor_baro_data_ready = 0U;
volatile uint8_t sensor_baro_pressure_valid = 0U;
volatile uint8_t sensor_baro_calibrated = 0U;

volatile int16_t sensor_gyro_x_raw = 0;
volatile int16_t sensor_gyro_y_raw = 0;
volatile int16_t sensor_gyro_z_raw = 0;

volatile int16_t sensor_accel_x_raw = 0;
volatile int16_t sensor_accel_y_raw = 0;
volatile int16_t sensor_accel_z_raw = 0;

volatile float sensor_gyro_x_dps = 0.0f;
volatile float sensor_gyro_y_dps = 0.0f;
volatile float sensor_gyro_z_dps = 0.0f;

volatile float sensor_accel_x_g = 0.0f;
volatile float sensor_accel_y_g = 0.0f;
volatile float sensor_accel_z_g = 0.0f;
volatile float sensor_accel_norm_g = 0.0f;

volatile uint8_t sensor_imu_filter_enabled = 0U;
volatile uint8_t sensor_imu_filter_config_ok = 0U;
volatile uint8_t sensor_imu_filter_initialized = 0U;

volatile float sensor_gyro_x_filtered_dps = 0.0f;
volatile float sensor_gyro_y_filtered_dps = 0.0f;
volatile float sensor_gyro_z_filtered_dps = 0.0f;

volatile float sensor_accel_x_filtered_g = 0.0f;
volatile float sensor_accel_y_filtered_g = 0.0f;
volatile float sensor_accel_z_filtered_g = 0.0f;
volatile float sensor_accel_filtered_norm_g = 0.0f;

volatile uint32_t sensor_imu_filter_update_count = 0UL;
volatile uint32_t sensor_imu_filter_duplicate_skip_count = 0UL;
volatile uint32_t sensor_imu_filter_reset_count = 0UL;
volatile uint32_t sensor_imu_filter_last_gap_us = 0UL;
volatile uint32_t sensor_imu_filter_max_gap_us = 0UL;

volatile float sensor_imu_gyro_filter_b0 = 0.0f;
volatile float sensor_imu_gyro_filter_b1 = 0.0f;
volatile float sensor_imu_gyro_filter_b2 = 0.0f;
volatile float sensor_imu_gyro_filter_a1 = 0.0f;
volatile float sensor_imu_gyro_filter_a2 = 0.0f;

volatile float sensor_imu_accel_filter_b0 = 0.0f;
volatile float sensor_imu_accel_filter_b1 = 0.0f;
volatile float sensor_imu_accel_filter_b2 = 0.0f;
volatile float sensor_imu_accel_filter_a1 = 0.0f;
volatile float sensor_imu_accel_filter_a2 = 0.0f;

volatile float sensor_baro_temperature_c = 0.0f;

volatile float sensor_baro_pressure_pa = 0.0f;
volatile float sensor_baro_filtered_pressure_pa = 0.0f;
volatile float sensor_baro_ground_pressure_pa = 0.0f;

volatile float sensor_baro_altitude_m = 0.0f;
volatile float sensor_baro_filtered_altitude_m = 0.0f;
volatile float sensor_baro_vertical_speed_mps = 0.0f;

volatile uint32_t sensor_baro_d1_raw = 0UL;
volatile uint32_t sensor_baro_d2_raw = 0UL;

volatile uint32_t sensor_baro_update_count = 0UL;
volatile uint32_t sensor_baro_valid_sample_count = 0UL;
volatile uint32_t sensor_baro_invalid_sample_count = 0UL;
volatile uint32_t sensor_baro_calibration_sample_count = 0UL;

/* -------------------------------------------------------------------------- */
/* Axis mapping                                                               */
/* -------------------------------------------------------------------------- */

#define BODY_GYRO_X_FROM_IMU(x, y, z)       (-(x))
#define BODY_GYRO_Y_FROM_IMU(x, y, z)       ( (y))
#define BODY_GYRO_Z_FROM_IMU(x, y, z)       ( (z))

#define BODY_ACCEL_X_FROM_IMU(x, y, z)      (-(x))
#define BODY_ACCEL_Y_FROM_IMU(x, y, z)      ( (y))
#define BODY_ACCEL_Z_FROM_IMU(x, y, z)      ( (z))

/* -------------------------------------------------------------------------- */

static float SensorManager_FastSqrtApprox(float value)
{
    if (value <= 0.0f)
    {
        return 0.0f;
    }

    float guess = value;

    for (uint8_t i = 0U; i < 6U; i++)
    {
        guess = 0.5f * (guess + (value / guess));
    }

    return guess;
}

static void SensorManager_ResetIMUCalibrationWindow(void)
{
    imu_calibration_sample_count_state = 0UL;
    imu_calibration_accel_norm_sum_g = 0.0f;
    imu_calibration_accel_norm_sum_square_g = 0.0f;

    for (uint8_t i = 0U; i < 3U; i++)
    {
        imu_calibration_gyro_sum_raw[i] = 0;
        imu_calibration_gyro_sum_square_raw[i] = 0;
        imu_calibration_accel_sum_raw[i] = 0;
        imu_calibration_accel_sum_square_raw[i] = 0;
    }

    sensor_imu_calibration_sample_count = 0UL;
}

static uint8_t SensorManager_UpdateIMUCalibration(
    int16_t gyro_x_raw,
    int16_t gyro_y_raw,
    int16_t gyro_z_raw,
    int16_t accel_x_raw,
    int16_t accel_y_raw,
    int16_t accel_z_raw,
    uint32_t sample_timestamp_us
)
{
#if (APP_IMU_STARTUP_CALIBRATION_ENABLED != 0U)
    const int16_t gyro_raw[3] = {gyro_x_raw, gyro_y_raw, gyro_z_raw};
    const int16_t accel_raw[3] = {accel_x_raw, accel_y_raw, accel_z_raw};
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float ax_g;
    float ay_g;
    float az_g;
    float gyro_norm_squared;
    float accel_norm_g;
    const float gyro_hard_limit_squared =
        APP_IMU_CALIBRATION_GYRO_HARD_LIMIT_DPS *
        APP_IMU_CALIBRATION_GYRO_HARD_LIMIT_DPS;

    if (imu_calibration_complete_state != 0U)
    {
        return 0U;
    }

    if ((sample_timestamp_us == 0UL) ||
        (sample_timestamp_us == imu_calibration_last_sample_timestamp_us))
    {
        return 0U;
    }

    imu_calibration_last_sample_timestamp_us = sample_timestamp_us;

    gx_dps = (float)gyro_x_raw * APP_IMU_GYRO_SCALE_DPS;
    gy_dps = (float)gyro_y_raw * APP_IMU_GYRO_SCALE_DPS;
    gz_dps = (float)gyro_z_raw * APP_IMU_GYRO_SCALE_DPS;

    ax_g = (float)accel_x_raw * APP_IMU_ACCEL_SCALE_G;
    ay_g = (float)accel_y_raw * APP_IMU_ACCEL_SCALE_G;
    az_g = (float)accel_z_raw * APP_IMU_ACCEL_SCALE_G;

    gyro_norm_squared =
        (gx_dps * gx_dps) +
        (gy_dps * gy_dps) +
        (gz_dps * gz_dps);

    accel_norm_g = SensorManager_FastSqrtApprox(
        (ax_g * ax_g) +
        (ay_g * ay_g) +
        (az_g * az_g)
    );

    /* Constant zero-rate offset is allowed; motion and impossible gravity are not. */
    if ((gyro_norm_squared > gyro_hard_limit_squared) ||
        (accel_norm_g < APP_IMU_CALIBRATION_ACCEL_MIN_G) ||
        (accel_norm_g > APP_IMU_CALIBRATION_ACCEL_MAX_G))
    {
        /*
         * A single rejected bus/sample transient must not erase more than one
         * second of good calibration data. Reset only after sustained motion.
         */
        imu_calibration_consecutive_reject_count++;
        sensor_imu_calibration_last_reject_reason = 1U;

        if (imu_calibration_consecutive_reject_count >=
            APP_IMU_CALIBRATION_MAX_CONSECUTIVE_REJECTS)
        {
            if (imu_calibration_sample_count_state != 0UL)
            {
                sensor_imu_calibration_reset_count++;
            }

            SensorManager_ResetIMUCalibrationWindow();
            imu_calibration_consecutive_reject_count = 0U;
        }

        return 0U;
    }

    imu_calibration_consecutive_reject_count = 0U;

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        imu_calibration_gyro_sum_raw[axis] += gyro_raw[axis];
        imu_calibration_gyro_sum_square_raw[axis] +=
            (int64_t)gyro_raw[axis] * (int64_t)gyro_raw[axis];

        imu_calibration_accel_sum_raw[axis] += accel_raw[axis];
        imu_calibration_accel_sum_square_raw[axis] +=
            (int64_t)accel_raw[axis] * (int64_t)accel_raw[axis];
    }

    imu_calibration_accel_norm_sum_g += accel_norm_g;
    imu_calibration_accel_norm_sum_square_g += accel_norm_g * accel_norm_g;
    imu_calibration_sample_count_state++;
    sensor_imu_calibration_sample_count = imu_calibration_sample_count_state;

    if (imu_calibration_sample_count_state >= APP_IMU_CALIBRATION_SAMPLE_COUNT)
    {
        const float sample_count = (float)imu_calibration_sample_count_state;
        const float expected_accel_g[3] =
        {
            APP_IMU_CALIBRATION_EXPECTED_ACCEL_X_G,
            APP_IMU_CALIBRATION_EXPECTED_ACCEL_Y_G,
            APP_IMU_CALIBRATION_EXPECTED_ACCEL_Z_G
        };
        float gyro_mean_raw[3];
        float accel_mean_raw[3];
        float gyro_stddev_dps[3];
        float accel_stddev_g[3];
        float average_accel_norm_g;
        float accel_norm_variance_g;
        float accel_norm_stddev_g;

        for (uint8_t axis = 0U; axis < 3U; axis++)
        {
            float gyro_variance_raw;
            float accel_variance_raw;

            gyro_mean_raw[axis] =
                (float)imu_calibration_gyro_sum_raw[axis] / sample_count;
            accel_mean_raw[axis] =
                (float)imu_calibration_accel_sum_raw[axis] / sample_count;

            gyro_variance_raw =
                ((float)imu_calibration_gyro_sum_square_raw[axis] / sample_count) -
                (gyro_mean_raw[axis] * gyro_mean_raw[axis]);
            accel_variance_raw =
                ((float)imu_calibration_accel_sum_square_raw[axis] / sample_count) -
                (accel_mean_raw[axis] * accel_mean_raw[axis]);

            if (gyro_variance_raw < 0.0f) gyro_variance_raw = 0.0f;
            if (accel_variance_raw < 0.0f) accel_variance_raw = 0.0f;

            gyro_stddev_dps[axis] = SensorManager_FastSqrtApprox(
                gyro_variance_raw) * APP_IMU_GYRO_SCALE_DPS;
            accel_stddev_g[axis] = SensorManager_FastSqrtApprox(
                accel_variance_raw) * APP_IMU_ACCEL_SCALE_G;
        }

        average_accel_norm_g = imu_calibration_accel_norm_sum_g / sample_count;
        accel_norm_variance_g =
            (imu_calibration_accel_norm_sum_square_g / sample_count) -
            (average_accel_norm_g * average_accel_norm_g);
        if (accel_norm_variance_g < 0.0f) accel_norm_variance_g = 0.0f;
        accel_norm_stddev_g = SensorManager_FastSqrtApprox(accel_norm_variance_g);

        sensor_imu_calibration_gyro_stddev_x_dps = gyro_stddev_dps[0];
        sensor_imu_calibration_gyro_stddev_y_dps = gyro_stddev_dps[1];
        sensor_imu_calibration_gyro_stddev_z_dps = gyro_stddev_dps[2];
        sensor_imu_calibration_accel_stddev_x_g = accel_stddev_g[0];
        sensor_imu_calibration_accel_stddev_y_g = accel_stddev_g[1];
        sensor_imu_calibration_accel_stddev_z_g = accel_stddev_g[2];
        sensor_imu_calibration_accel_stddev_g = accel_norm_stddev_g;

        if ((gyro_stddev_dps[0] > APP_IMU_CALIBRATION_GYRO_STDDEV_MAX_DPS) ||
            (gyro_stddev_dps[1] > APP_IMU_CALIBRATION_GYRO_STDDEV_MAX_DPS) ||
            (gyro_stddev_dps[2] > APP_IMU_CALIBRATION_GYRO_STDDEV_MAX_DPS) ||
            (accel_stddev_g[0] > APP_IMU_CALIBRATION_ACCEL_AXIS_STDDEV_MAX_G) ||
            (accel_stddev_g[1] > APP_IMU_CALIBRATION_ACCEL_AXIS_STDDEV_MAX_G) ||
            (accel_stddev_g[2] > APP_IMU_CALIBRATION_ACCEL_AXIS_STDDEV_MAX_G) ||
            (accel_norm_stddev_g > APP_IMU_CALIBRATION_ACCEL_STDDEV_MAX_G))
        {
            sensor_imu_calibration_reset_count++;
            sensor_imu_calibration_last_reject_reason = 2U;
            SensorManager_ResetIMUCalibrationWindow();
            return 0U;
        }

        for (uint8_t axis = 0U; axis < 3U; axis++)
        {
            const float expected_raw = expected_accel_g[axis] /
                                       APP_IMU_ACCEL_SCALE_G;
            const float accel_bias_g =
                (accel_mean_raw[axis] - expected_raw) *
                APP_IMU_ACCEL_SCALE_G;

            if ((accel_bias_g > APP_IMU_CALIBRATION_ACCEL_BIAS_MAX_G) ||
                (accel_bias_g < -APP_IMU_CALIBRATION_ACCEL_BIAS_MAX_G))
            {
                sensor_imu_calibration_reset_count++;
                sensor_imu_calibration_last_reject_reason = 3U;
                SensorManager_ResetIMUCalibrationWindow();
                return 0U;
            }

            imu_gyro_bias_raw[axis] = gyro_mean_raw[axis];
            imu_accel_bias_raw[axis] = accel_mean_raw[axis] - expected_raw;
        }

        imu_calibration_complete_state = 1U;
        sensor_imu_calibration_complete = 1U;
        sensor_imu_calibration_last_reject_reason = 0U;
        sensor_imu_gyro_bias_x_raw = imu_gyro_bias_raw[0];
        sensor_imu_gyro_bias_y_raw = imu_gyro_bias_raw[1];
        sensor_imu_gyro_bias_z_raw = imu_gyro_bias_raw[2];
        sensor_imu_accel_bias_x_raw = imu_accel_bias_raw[0];
        sensor_imu_accel_bias_y_raw = imu_accel_bias_raw[1];
        sensor_imu_accel_bias_z_raw = imu_accel_bias_raw[2];
        sensor_imu_accel_scale = 1.0f;

        sensor_imu_filter_initialized = 0U;
        return 1U;
    }
#else
    (void)gyro_x_raw;
    (void)gyro_y_raw;
    (void)gyro_z_raw;
    (void)accel_x_raw;
    (void)accel_y_raw;
    (void)accel_z_raw;
    (void)sample_timestamp_us;

    imu_calibration_complete_state = 1U;
    sensor_imu_calibration_complete = 1U;
#endif

    return 0U;
}

static uint8_t SensorManager_InitIMUFilters(void)
{
#if (APP_IMU_BUTTERWORTH_ENABLED != 0U)
    uint8_t ok = 1U;

    ok &= Butterworth2LPF_Init(
        &gyro_filter_x,
        APP_IMU_FILTER_SAMPLE_RATE_HZ,
        APP_IMU_GYRO_LPF_CUTOFF_HZ
    );

    ok &= Butterworth2LPF_Init(
        &gyro_filter_y,
        APP_IMU_FILTER_SAMPLE_RATE_HZ,
        APP_IMU_GYRO_LPF_CUTOFF_HZ
    );

    ok &= Butterworth2LPF_Init(
        &gyro_filter_z,
        APP_IMU_FILTER_SAMPLE_RATE_HZ,
        APP_IMU_GYRO_LPF_CUTOFF_HZ
    );

    ok &= Butterworth2LPF_Init(
        &accel_filter_x,
        APP_IMU_FILTER_SAMPLE_RATE_HZ,
        APP_IMU_ACCEL_LPF_CUTOFF_HZ
    );

    ok &= Butterworth2LPF_Init(
        &accel_filter_y,
        APP_IMU_FILTER_SAMPLE_RATE_HZ,
        APP_IMU_ACCEL_LPF_CUTOFF_HZ
    );

    ok &= Butterworth2LPF_Init(
        &accel_filter_z,
        APP_IMU_FILTER_SAMPLE_RATE_HZ,
        APP_IMU_ACCEL_LPF_CUTOFF_HZ
    );

    sensor_imu_filter_enabled = 1U;
    sensor_imu_filter_config_ok = ok;

    sensor_imu_gyro_filter_b0 = gyro_filter_x.b0;
    sensor_imu_gyro_filter_b1 = gyro_filter_x.b1;
    sensor_imu_gyro_filter_b2 = gyro_filter_x.b2;
    sensor_imu_gyro_filter_a1 = gyro_filter_x.a1;
    sensor_imu_gyro_filter_a2 = gyro_filter_x.a2;

    sensor_imu_accel_filter_b0 = accel_filter_x.b0;
    sensor_imu_accel_filter_b1 = accel_filter_x.b1;
    sensor_imu_accel_filter_b2 = accel_filter_x.b2;
    sensor_imu_accel_filter_a1 = accel_filter_x.a1;
    sensor_imu_accel_filter_a2 = accel_filter_x.a2;

    return ok;
#else
    sensor_imu_filter_enabled = 0U;
    sensor_imu_filter_config_ok = 1U;
    return 1U;
#endif
}

static void SensorManager_ResetIMUFilters(void)
{
    Butterworth2LPF_Reset(&gyro_filter_x, sensor_data.gyro_x_dps);
    Butterworth2LPF_Reset(&gyro_filter_y, sensor_data.gyro_y_dps);
    Butterworth2LPF_Reset(&gyro_filter_z, sensor_data.gyro_z_dps);

    Butterworth2LPF_Reset(&accel_filter_x, sensor_data.accel_x_g);
    Butterworth2LPF_Reset(&accel_filter_y, sensor_data.accel_y_g);
    Butterworth2LPF_Reset(&accel_filter_z, sensor_data.accel_z_g);

    sensor_data.gyro_x_filtered_dps = sensor_data.gyro_x_dps;
    sensor_data.gyro_y_filtered_dps = sensor_data.gyro_y_dps;
    sensor_data.gyro_z_filtered_dps = sensor_data.gyro_z_dps;

    sensor_data.accel_x_filtered_g = sensor_data.accel_x_g;
    sensor_data.accel_y_filtered_g = sensor_data.accel_y_g;
    sensor_data.accel_z_filtered_g = sensor_data.accel_z_g;
    sensor_data.accel_filtered_norm_g = sensor_data.accel_norm_g;

    sensor_imu_filter_initialized = 1U;
    sensor_imu_filter_reset_count++;
}

static void SensorManager_UpdateIMUFilters(uint32_t sample_timestamp_us)
{
#if (APP_IMU_BUTTERWORTH_ENABLED != 0U)
    uint32_t gap_us;
    float accel_squared;

    if (sensor_imu_filter_config_ok == 0U)
    {
        sensor_data.gyro_x_filtered_dps = sensor_data.gyro_x_dps;
        sensor_data.gyro_y_filtered_dps = sensor_data.gyro_y_dps;
        sensor_data.gyro_z_filtered_dps = sensor_data.gyro_z_dps;

        sensor_data.accel_x_filtered_g = sensor_data.accel_x_g;
        sensor_data.accel_y_filtered_g = sensor_data.accel_y_g;
        sensor_data.accel_z_filtered_g = sensor_data.accel_z_g;
        sensor_data.accel_filtered_norm_g = sensor_data.accel_norm_g;
        return;
    }

    if ((imu_filter_last_sample_timestamp_us != 0UL) &&
        (sample_timestamp_us == imu_filter_last_sample_timestamp_us))
    {
        sensor_imu_filter_duplicate_skip_count++;
        return;
    }

    gap_us = sample_timestamp_us - imu_filter_last_sample_timestamp_us;
    sensor_imu_filter_last_gap_us = gap_us;

    if (gap_us > sensor_imu_filter_max_gap_us)
    {
        sensor_imu_filter_max_gap_us = gap_us;
    }

    if ((sensor_imu_filter_initialized == 0U) ||
        (imu_filter_last_sample_timestamp_us == 0UL) ||
        (gap_us > APP_IMU_FILTER_RESET_GAP_US))
    {
        SensorManager_ResetIMUFilters();
    }
    else
    {
        sensor_data.gyro_x_filtered_dps =
            Butterworth2LPF_Process(&gyro_filter_x, sensor_data.gyro_x_dps);

        sensor_data.gyro_y_filtered_dps =
            Butterworth2LPF_Process(&gyro_filter_y, sensor_data.gyro_y_dps);

        sensor_data.gyro_z_filtered_dps =
            Butterworth2LPF_Process(&gyro_filter_z, sensor_data.gyro_z_dps);

        sensor_data.accel_x_filtered_g =
            Butterworth2LPF_Process(&accel_filter_x, sensor_data.accel_x_g);

        sensor_data.accel_y_filtered_g =
            Butterworth2LPF_Process(&accel_filter_y, sensor_data.accel_y_g);

        sensor_data.accel_z_filtered_g =
            Butterworth2LPF_Process(&accel_filter_z, sensor_data.accel_z_g);

        accel_squared =
            (sensor_data.accel_x_filtered_g * sensor_data.accel_x_filtered_g) +
            (sensor_data.accel_y_filtered_g * sensor_data.accel_y_filtered_g) +
            (sensor_data.accel_z_filtered_g * sensor_data.accel_z_filtered_g);

        sensor_data.accel_filtered_norm_g =
            SensorManager_FastSqrtApprox(accel_squared);
    }

    imu_filter_last_sample_timestamp_us = sample_timestamp_us;
    sensor_imu_filter_update_count++;
#else
    (void)sample_timestamp_us;

    sensor_data.gyro_x_filtered_dps = sensor_data.gyro_x_dps;
    sensor_data.gyro_y_filtered_dps = sensor_data.gyro_y_dps;
    sensor_data.gyro_z_filtered_dps = sensor_data.gyro_z_dps;

    sensor_data.accel_x_filtered_g = sensor_data.accel_x_g;
    sensor_data.accel_y_filtered_g = sensor_data.accel_y_g;
    sensor_data.accel_z_filtered_g = sensor_data.accel_z_g;
    sensor_data.accel_filtered_norm_g = sensor_data.accel_norm_g;
#endif
}

static void SensorManager_UpdateLiveDebug(void)
{
    sensor_timestamp_us = sensor_data.timestamp_us;
    sensor_update_count = sensor_data.update_count;

    sensor_imu_valid = sensor_data.imu_valid;

    sensor_baro_valid = sensor_data.baro_valid;
    sensor_baro_data_ready = sensor_data.baro_data_ready;
    sensor_baro_pressure_valid = sensor_data.baro_pressure_valid;
    sensor_baro_calibrated = sensor_data.baro_calibrated;

    sensor_gyro_x_raw = sensor_data.gyro_x_raw;
    sensor_gyro_y_raw = sensor_data.gyro_y_raw;
    sensor_gyro_z_raw = sensor_data.gyro_z_raw;

    sensor_accel_x_raw = sensor_data.accel_x_raw;
    sensor_accel_y_raw = sensor_data.accel_y_raw;
    sensor_accel_z_raw = sensor_data.accel_z_raw;

    sensor_gyro_x_dps = sensor_data.gyro_x_dps;
    sensor_gyro_y_dps = sensor_data.gyro_y_dps;
    sensor_gyro_z_dps = sensor_data.gyro_z_dps;

    sensor_accel_x_g = sensor_data.accel_x_g;
    sensor_accel_y_g = sensor_data.accel_y_g;
    sensor_accel_z_g = sensor_data.accel_z_g;
    sensor_accel_norm_g = sensor_data.accel_norm_g;

    sensor_gyro_x_filtered_dps = sensor_data.gyro_x_filtered_dps;
    sensor_gyro_y_filtered_dps = sensor_data.gyro_y_filtered_dps;
    sensor_gyro_z_filtered_dps = sensor_data.gyro_z_filtered_dps;

    sensor_accel_x_filtered_g = sensor_data.accel_x_filtered_g;
    sensor_accel_y_filtered_g = sensor_data.accel_y_filtered_g;
    sensor_accel_z_filtered_g = sensor_data.accel_z_filtered_g;
    sensor_accel_filtered_norm_g = sensor_data.accel_filtered_norm_g;

    sensor_baro_temperature_c = sensor_data.baro_temperature_c;

    sensor_baro_pressure_pa = sensor_data.baro_pressure_pa;
    sensor_baro_filtered_pressure_pa =
        sensor_data.baro_filtered_pressure_pa;

    sensor_baro_ground_pressure_pa =
        sensor_data.baro_ground_pressure_pa;

    sensor_baro_altitude_m = sensor_data.baro_altitude_m;

    sensor_baro_filtered_altitude_m =
        sensor_data.baro_filtered_altitude_m;

    sensor_baro_vertical_speed_mps =
        sensor_data.baro_vertical_speed_mps;

    sensor_baro_d1_raw = sensor_data.baro_d1_raw;
    sensor_baro_d2_raw = sensor_data.baro_d2_raw;

    sensor_baro_update_count = sensor_data.baro_update_count;

    sensor_baro_valid_sample_count =
        sensor_data.baro_valid_sample_count;

    sensor_baro_invalid_sample_count =
        sensor_data.baro_invalid_sample_count;

    sensor_baro_calibration_sample_count =
        sensor_data.baro_calibration_sample_count;
}

/* -------------------------------------------------------------------------- */

void SensorManager_Init(void)
{
    sensor_data.timestamp_us = 0UL;
    sensor_data.imu_sample_timestamp_us = 0UL;
    sensor_data.update_count = 0UL;

    sensor_data.imu_valid = 0U;

    sensor_imu_valid_update_count = 0UL;
    sensor_imu_rejected_update_count = 0UL;
    sensor_imu_last_sample_timestamp_us = 0UL;
    sensor_imu_sample_age_us = 0xFFFFFFFFUL;
    sensor_imu_stale_invalidation_count = 0UL;


#if (APP_IMU_STARTUP_CALIBRATION_ENABLED != 0U)
    imu_calibration_complete_state = 0U;
    sensor_imu_calibration_complete = 0U;
#else
    imu_calibration_complete_state = 1U;
    sensor_imu_calibration_complete = 1U;
#endif
    imu_calibration_last_sample_timestamp_us = 0UL;
    sensor_imu_calibration_reset_count = 0UL;
    sensor_imu_gyro_bias_x_raw = 0.0f;
    sensor_imu_gyro_bias_y_raw = 0.0f;
    sensor_imu_gyro_bias_z_raw = 0.0f;
    sensor_imu_accel_bias_x_raw = 0.0f;
    sensor_imu_accel_bias_y_raw = 0.0f;
    sensor_imu_accel_bias_z_raw = 0.0f;
    sensor_imu_accel_scale = 1.0f;
    sensor_imu_calibration_gyro_stddev_x_dps = 0.0f;
    sensor_imu_calibration_gyro_stddev_y_dps = 0.0f;
    sensor_imu_calibration_gyro_stddev_z_dps = 0.0f;
    sensor_imu_calibration_accel_stddev_g = 0.0f;
    sensor_imu_calibration_accel_stddev_x_g = 0.0f;
    sensor_imu_calibration_accel_stddev_y_g = 0.0f;
    sensor_imu_calibration_accel_stddev_z_g = 0.0f;
    sensor_imu_calibration_last_reject_reason = 0U;
    sensor_imu_duplicate_generation_count = 0UL;
    sensor_imu_last_generation = 0UL;
    imu_last_consumed_generation = 0UL;
    imu_calibration_consecutive_reject_count = 0U;
    imu_gyro_bias_raw[0] = 0.0f;
    imu_gyro_bias_raw[1] = 0.0f;
    imu_gyro_bias_raw[2] = 0.0f;
    imu_accel_bias_raw[0] = 0.0f;
    imu_accel_bias_raw[1] = 0.0f;
    imu_accel_bias_raw[2] = 0.0f;
    SensorManager_ResetIMUCalibrationWindow();

    sensor_data.baro_valid = 0U;
    sensor_data.baro_data_ready = 0U;
    sensor_data.baro_pressure_valid = 0U;
    sensor_data.baro_calibrated = 0U;

    sensor_data.gyro_x_raw = 0;
    sensor_data.gyro_y_raw = 0;
    sensor_data.gyro_z_raw = 0;

    sensor_data.accel_x_raw = 0;
    sensor_data.accel_y_raw = 0;
    sensor_data.accel_z_raw = 0;

    sensor_data.gyro_x_dps = 0.0f;
    sensor_data.gyro_y_dps = 0.0f;
    sensor_data.gyro_z_dps = 0.0f;

    sensor_data.accel_x_g = 0.0f;
    sensor_data.accel_y_g = 0.0f;
    sensor_data.accel_z_g = 0.0f;
    sensor_data.accel_norm_g = 0.0f;

    sensor_data.gyro_x_filtered_dps = 0.0f;
    sensor_data.gyro_y_filtered_dps = 0.0f;
    sensor_data.gyro_z_filtered_dps = 0.0f;

    sensor_data.accel_x_filtered_g = 0.0f;
    sensor_data.accel_y_filtered_g = 0.0f;
    sensor_data.accel_z_filtered_g = 0.0f;
    sensor_data.accel_filtered_norm_g = 0.0f;

    sensor_imu_filter_initialized = 0U;
    sensor_imu_filter_update_count = 0UL;
    sensor_imu_filter_duplicate_skip_count = 0UL;
    sensor_imu_filter_reset_count = 0UL;
    sensor_imu_filter_last_gap_us = 0UL;
    sensor_imu_filter_max_gap_us = 0UL;
    imu_filter_last_sample_timestamp_us = 0UL;

    (void)SensorManager_InitIMUFilters();

    sensor_data.baro_temperature_c = 0.0f;

    sensor_data.baro_pressure_pa = 0.0f;
    sensor_data.baro_filtered_pressure_pa = 0.0f;
    sensor_data.baro_ground_pressure_pa = 0.0f;

    sensor_data.baro_altitude_m = 0.0f;
    sensor_data.baro_filtered_altitude_m = 0.0f;
    sensor_data.baro_vertical_speed_mps = 0.0f;

    sensor_data.baro_d1_raw = 0UL;
    sensor_data.baro_d2_raw = 0UL;

    sensor_data.baro_update_count = 0UL;
    sensor_data.baro_valid_sample_count = 0UL;
    sensor_data.baro_invalid_sample_count = 0UL;
    sensor_data.baro_calibration_sample_count = 0UL;

    SensorManager_UpdateLiveDebug();
}

void SensorManager_UpdateIMU(void)
{
    IMU_RawData_t raw;
    uint32_t sample_timestamp_us = 0UL;
    uint32_t sample_generation = 0UL;
    uint8_t snapshot_valid;
    int16_t body_gyro_x_raw;
    int16_t body_gyro_y_raw;
    int16_t body_gyro_z_raw;
    int16_t body_accel_x_raw;
    int16_t body_accel_y_raw;
    int16_t body_accel_z_raw;
    float gyro_x_corrected_raw;
    float gyro_y_corrected_raw;
    float gyro_z_corrected_raw;
    float accel_x_corrected_raw;
    float accel_y_corrected_raw;
    float accel_z_corrected_raw;
    float accel_squared;

    snapshot_valid = IMU_ReadRawSnapshot(
        &raw,
        &sample_timestamp_us,
        &sample_generation
    );

    if ((IMU_IsConnected() == 0U) || (snapshot_valid == 0U))
    {
        sensor_data.imu_valid = 0U;
        sensor_imu_rejected_update_count++;
        SensorManager_UpdateLiveDebug();
        return;
    }

    /* Never propagate the same completed DMA packet twice. */
    if ((sample_generation == 0UL) ||
        (sample_generation == imu_last_consumed_generation))
    {
        sensor_data.imu_valid = 0U;
        sensor_imu_duplicate_generation_count++;
        SensorManager_UpdateLiveDebug();
        return;
    }

    imu_last_consumed_generation = sample_generation;
    sensor_imu_last_generation = sample_generation;
    sensor_imu_last_sample_timestamp_us = sample_timestamp_us;
    sensor_data.imu_sample_timestamp_us = sample_timestamp_us;
    sensor_data.timestamp_us = sample_timestamp_us;

    body_gyro_x_raw = BODY_GYRO_X_FROM_IMU(
        raw.gyro_x_raw, raw.gyro_y_raw, raw.gyro_z_raw);
    body_gyro_y_raw = BODY_GYRO_Y_FROM_IMU(
        raw.gyro_x_raw, raw.gyro_y_raw, raw.gyro_z_raw);
    body_gyro_z_raw = BODY_GYRO_Z_FROM_IMU(
        raw.gyro_x_raw, raw.gyro_y_raw, raw.gyro_z_raw);

    body_accel_x_raw = BODY_ACCEL_X_FROM_IMU(
        raw.accel_x_raw, raw.accel_y_raw, raw.accel_z_raw);
    body_accel_y_raw = BODY_ACCEL_Y_FROM_IMU(
        raw.accel_x_raw, raw.accel_y_raw, raw.accel_z_raw);
    body_accel_z_raw = BODY_ACCEL_Z_FROM_IMU(
        raw.accel_x_raw, raw.accel_y_raw, raw.accel_z_raw);

    /* Raw watch values always show the untouched, coherent DMA packet. */
    sensor_data.gyro_x_raw = body_gyro_x_raw;
    sensor_data.gyro_y_raw = body_gyro_y_raw;
    sensor_data.gyro_z_raw = body_gyro_z_raw;
    sensor_data.accel_x_raw = body_accel_x_raw;
    sensor_data.accel_y_raw = body_accel_y_raw;
    sensor_data.accel_z_raw = body_accel_z_raw;

    (void)SensorManager_UpdateIMUCalibration(
        body_gyro_x_raw,
        body_gyro_y_raw,
        body_gyro_z_raw,
        body_accel_x_raw,
        body_accel_y_raw,
        body_accel_z_raw,
        sample_timestamp_us
    );

    /* Estimators must not integrate uncalibrated startup bias. */
    if (imu_calibration_complete_state == 0U)
    {
        sensor_data.imu_valid = 0U;
        sensor_imu_rejected_update_count++;
        SensorManager_UpdateLiveDebug();
        return;
    }

    gyro_x_corrected_raw = (float)body_gyro_x_raw - imu_gyro_bias_raw[0];
    gyro_y_corrected_raw = (float)body_gyro_y_raw - imu_gyro_bias_raw[1];
    gyro_z_corrected_raw = (float)body_gyro_z_raw - imu_gyro_bias_raw[2];

    accel_x_corrected_raw = (float)body_accel_x_raw - imu_accel_bias_raw[0];
    accel_y_corrected_raw = (float)body_accel_y_raw - imu_accel_bias_raw[1];
    accel_z_corrected_raw = (float)body_accel_z_raw - imu_accel_bias_raw[2];

    sensor_data.gyro_x_dps = gyro_x_corrected_raw * APP_IMU_GYRO_SCALE_DPS;
    sensor_data.gyro_y_dps = gyro_y_corrected_raw * APP_IMU_GYRO_SCALE_DPS;
    sensor_data.gyro_z_dps = gyro_z_corrected_raw * APP_IMU_GYRO_SCALE_DPS;

    sensor_data.accel_x_g = accel_x_corrected_raw * APP_IMU_ACCEL_SCALE_G;
    sensor_data.accel_y_g = accel_y_corrected_raw * APP_IMU_ACCEL_SCALE_G;
    sensor_data.accel_z_g = accel_z_corrected_raw * APP_IMU_ACCEL_SCALE_G;

    accel_squared =
        (sensor_data.accel_x_g * sensor_data.accel_x_g) +
        (sensor_data.accel_y_g * sensor_data.accel_y_g) +
        (sensor_data.accel_z_g * sensor_data.accel_z_g);
    sensor_data.accel_norm_g = SensorManager_FastSqrtApprox(accel_squared);

    sensor_data.imu_valid = 1U;
    SensorManager_UpdateIMUFilters(sample_timestamp_us);

    sensor_data.update_count++;
    sensor_imu_valid_update_count++;
    SensorManager_UpdateLiveDebug();
}

void SensorManager_UpdateBarometer(void)
{
    Barometer_Update();

    BarometerData_t baro = Barometer_GetData();

    /*
     * Barometre ancak bu dört koşul birlikte sağlanırsa geçerli.
     */
    sensor_data.baro_valid =
        (baro.connected != 0U) &&
        (baro.data_ready != 0U) &&
        (baro.pressure_valid != 0U) &&
        (baro.calibrated != 0U);

    sensor_data.baro_data_ready = baro.data_ready;
    sensor_data.baro_pressure_valid = baro.pressure_valid;
    sensor_data.baro_calibrated = baro.calibrated;

    sensor_data.baro_temperature_c = baro.temperature_c;

    sensor_data.baro_pressure_pa = baro.pressure_pa;
    sensor_data.baro_filtered_pressure_pa =
        baro.filtered_pressure_pa;

    sensor_data.baro_ground_pressure_pa =
        baro.ground_pressure_pa;

    sensor_data.baro_altitude_m = baro.altitude_m;

    sensor_data.baro_filtered_altitude_m =
        baro.filtered_altitude_m;

    sensor_data.baro_vertical_speed_mps =
        baro.vertical_speed_mps;

    sensor_data.baro_d1_raw = baro.d1_raw;
    sensor_data.baro_d2_raw = baro.d2_raw;

    sensor_data.baro_update_count = baro.update_count;

    sensor_data.baro_valid_sample_count =
        baro.valid_sample_count;

    sensor_data.baro_invalid_sample_count =
        baro.invalid_sample_count;

    sensor_data.baro_calibration_sample_count =
        baro.calibration_sample_count;

    SensorManager_UpdateLiveDebug();
}

void SensorManager_ServiceFreshness(void)
{
    uint32_t now_us = micros();
    uint32_t sample_timestamp_us = sensor_data.imu_sample_timestamp_us;

    if (sample_timestamp_us == 0UL)
    {
        sensor_imu_sample_age_us = 0xFFFFFFFFUL;
    }
    else
    {
        sensor_imu_sample_age_us = (uint32_t)(now_us - sample_timestamp_us);
    }

    if ((sample_timestamp_us == 0UL) ||
        (sensor_imu_sample_age_us > APP_SENSOR_MANAGER_IMU_VALID_TIMEOUT_US))
    {
        if (sensor_data.imu_valid != 0U)
        {
            sensor_imu_stale_invalidation_count++;
        }

        sensor_data.imu_valid = 0U;
    }

    SensorManager_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */

SensorData_t SensorManager_GetData(void)
{
    return sensor_data;
}

const SensorData_t *SensorManager_GetDataPtr(void)
{
    return &sensor_data;
}

uint8_t SensorManager_IsIMUValid(void)
{
    return sensor_data.imu_valid;
}

uint8_t SensorManager_IsBarometerValid(void)
{
    return sensor_data.baro_valid;
}

uint32_t SensorManager_GetUpdateCount(void)
{
    return sensor_data.update_count;
}
