#include "Modules/Sensors/Barometer/barometer.h"
#include "Modules/Sensors/Barometer/ms5611_spi.h"

#include "Common/app_config.h"
#include "Common/Filters/butterworth_filter.h"

#include "Platform/board_handles.h"
#include "Services/Timebase/timebase.h"

#include "main.h"

/* -------------------------------------------------------------------------- */
/* Private data                                                               */
/* -------------------------------------------------------------------------- */

static BarometerData_t barometer_data;

static Butterworth2LPF_t pressure_filter;
static Butterworth2LPF_t vertical_speed_filter;

static float pressure_median_window[3];
static uint8_t pressure_median_count = 0U;
static uint8_t pressure_median_index = 0U;

static float calibration_pressure_sum = 0.0f;
static float previous_filtered_altitude_m = 0.0f;

static uint32_t last_ms5611_update_count = 0UL;
static uint32_t last_valid_sample_timestamp_us = 0UL;

/* P54: rate-aware raw pressure guard.  The accepted reference timestamp is
 * intentionally kept separate from the low-pass/filter timestamp so a rejected
 * transport spike cannot advance the physical-rate allowance. */
static uint8_t baro_raw_reference_valid = 0U;
static float baro_last_plausible_pressure_pa = 0.0f;
static uint32_t baro_last_plausible_timestamp_us = 0UL;
static uint8_t baro_raw_step_candidate_valid = 0U;
static float baro_raw_step_candidate_pa = 0.0f;

/* -------------------------------------------------------------------------- */
/* Live Expressions                                                           */
/* -------------------------------------------------------------------------- */

volatile uint8_t barometer_initialized = 0U;
volatile uint8_t barometer_connected = 0U;
volatile uint8_t barometer_data_ready = 0U;
volatile uint8_t barometer_pressure_valid = 0U;
volatile uint8_t barometer_calibrated = 0U;
volatile uint8_t barometer_healthy = 0U;

volatile uint8_t barometer_filter_enabled = 0U;
volatile uint8_t barometer_filter_config_ok = 0U;
volatile uint8_t barometer_filter_initialized = 0U;

volatile float barometer_temperature_c = 0.0f;

volatile float barometer_pressure_pa = 0.0f;
volatile float barometer_median_pressure_pa = 0.0f;
volatile float barometer_filtered_pressure_pa = 0.0f;
volatile float barometer_ground_pressure_pa = 0.0f;
volatile uint8_t barometer_ground_reference_tracking_allowed = 0U;
volatile uint8_t barometer_ground_reference_tracking_active = 0U;
volatile uint8_t barometer_ground_reference_frozen = 0U;
volatile float barometer_ground_reference_error_pa = 0.0f;
volatile float barometer_ground_reference_last_step_pa = 0.0f;
volatile uint32_t barometer_ground_reference_update_count = 0UL;
volatile uint32_t barometer_ground_reference_freeze_count = 0UL;
volatile uint32_t barometer_ground_reference_last_update_us = 0UL;

volatile float barometer_altitude_m = 0.0f;
volatile float barometer_filtered_altitude_m = 0.0f;
volatile float barometer_vertical_speed_mps = 0.0f;

volatile uint32_t barometer_d1_raw = 0UL;
volatile uint32_t barometer_d2_raw = 0UL;

volatile uint32_t barometer_update_count = 0UL;
volatile uint32_t barometer_source_update_count = 0UL;
volatile uint32_t barometer_valid_sample_count = 0UL;
volatile uint32_t barometer_invalid_sample_count = 0UL;
volatile uint32_t barometer_calibration_sample_count = 0UL;

volatile uint32_t barometer_duplicate_sample_skip_count = 0UL;

volatile uint32_t barometer_median_update_count = 0UL;
volatile uint32_t barometer_median_rejected_spike_count = 0UL;
volatile uint32_t barometer_raw_spike_reject_count = 0UL;
volatile uint32_t barometer_raw_step_confirm_count = 0UL;

volatile uint32_t barometer_filter_update_count = 0UL;
volatile uint32_t barometer_filter_reset_count = 0UL;

volatile uint32_t barometer_last_sample_timestamp_us = 0UL;
volatile uint32_t barometer_last_sample_interval_us = 0UL;
volatile uint32_t barometer_min_sample_interval_us = 0UL;
volatile uint32_t barometer_max_sample_interval_us = 0UL;

volatile uint32_t barometer_last_measure_duration_us = 0UL;
volatile uint32_t barometer_max_measure_duration_us = 0UL;

volatile float barometer_pressure_filter_b0 = 0.0f;
volatile float barometer_pressure_filter_b1 = 0.0f;
volatile float barometer_pressure_filter_b2 = 0.0f;
volatile float barometer_pressure_filter_a1 = 0.0f;
volatile float barometer_pressure_filter_a2 = 0.0f;

volatile float barometer_vertical_speed_filter_b0 = 0.0f;
volatile float barometer_vertical_speed_filter_b1 = 0.0f;
volatile float barometer_vertical_speed_filter_b2 = 0.0f;
volatile float barometer_vertical_speed_filter_a1 = 0.0f;
volatile float barometer_vertical_speed_filter_a2 = 0.0f;

/* -------------------------------------------------------------------------- */

static void Barometer_UpdateLiveDebug(void)
{
    barometer_initialized = barometer_data.initialized;
    barometer_connected = barometer_data.connected;
    barometer_data_ready = barometer_data.data_ready;
    barometer_pressure_valid = barometer_data.pressure_valid;
    barometer_calibrated = barometer_data.calibrated;
    barometer_healthy = barometer_data.healthy;

    barometer_filter_enabled = barometer_data.filter_enabled;
    barometer_filter_config_ok = barometer_data.filter_config_ok;
    barometer_filter_initialized = barometer_data.filter_initialized;

    barometer_temperature_c = barometer_data.temperature_c;

    barometer_pressure_pa = barometer_data.pressure_pa;
    barometer_median_pressure_pa = barometer_data.median_pressure_pa;
    barometer_filtered_pressure_pa = barometer_data.filtered_pressure_pa;
    barometer_ground_pressure_pa = barometer_data.ground_pressure_pa;
    barometer_ground_reference_tracking_allowed =
        barometer_data.ground_reference_tracking_allowed;
    barometer_ground_reference_tracking_active =
        barometer_data.ground_reference_tracking_active;
    barometer_ground_reference_frozen =
        barometer_data.ground_reference_frozen;
    barometer_ground_reference_error_pa =
        barometer_data.ground_reference_error_pa;
    barometer_ground_reference_last_step_pa =
        barometer_data.ground_reference_last_step_pa;
    barometer_ground_reference_update_count =
        barometer_data.ground_reference_update_count;
    barometer_ground_reference_freeze_count =
        barometer_data.ground_reference_freeze_count;
    barometer_ground_reference_last_update_us =
        barometer_data.ground_reference_last_update_us;

    barometer_altitude_m = barometer_data.altitude_m;
    barometer_filtered_altitude_m = barometer_data.filtered_altitude_m;
    barometer_vertical_speed_mps = barometer_data.vertical_speed_mps;

    barometer_d1_raw = barometer_data.d1_raw;
    barometer_d2_raw = barometer_data.d2_raw;

    barometer_update_count = barometer_data.update_count;
    barometer_source_update_count = barometer_data.source_update_count;
    barometer_valid_sample_count = barometer_data.valid_sample_count;
    barometer_invalid_sample_count = barometer_data.invalid_sample_count;
    barometer_calibration_sample_count =
        barometer_data.calibration_sample_count;

    barometer_duplicate_sample_skip_count =
        barometer_data.duplicate_sample_skip_count;

    barometer_median_update_count =
        barometer_data.median_update_count;
    barometer_median_rejected_spike_count =
        barometer_data.median_rejected_spike_count;
    barometer_raw_spike_reject_count =
        barometer_data.raw_spike_reject_count;
    barometer_raw_step_confirm_count =
        barometer_data.raw_step_confirm_count;

    barometer_filter_update_count = barometer_data.filter_update_count;
    barometer_filter_reset_count = barometer_data.filter_reset_count;

    barometer_last_sample_timestamp_us =
        barometer_data.last_sample_timestamp_us;
    barometer_last_sample_interval_us =
        barometer_data.last_sample_interval_us;
    barometer_min_sample_interval_us =
        barometer_data.min_sample_interval_us;
    barometer_max_sample_interval_us =
        barometer_data.max_sample_interval_us;

    barometer_last_measure_duration_us =
        barometer_data.last_measure_duration_us;
    barometer_max_measure_duration_us =
        barometer_data.max_measure_duration_us;
}

/* -------------------------------------------------------------------------- */

static void Barometer_ResetData(void)
{
    barometer_data.initialized = 0U;
    barometer_data.connected = 0U;
    barometer_data.data_ready = 0U;
    barometer_data.pressure_valid = 0U;
    barometer_data.calibrated = 0U;
    barometer_data.healthy = 0U;

    barometer_data.filter_enabled = 0U;
    barometer_data.filter_config_ok = 0U;
    barometer_data.filter_initialized = 0U;

    barometer_data.temperature_c = 0.0f;

    barometer_data.pressure_pa = 0.0f;
    barometer_data.median_pressure_pa = 0.0f;
    barometer_data.filtered_pressure_pa = 0.0f;
    barometer_data.ground_pressure_pa = 0.0f;
    barometer_data.ground_reference_tracking_allowed = 0U;
    barometer_data.ground_reference_tracking_active = 0U;
    barometer_data.ground_reference_frozen = 0U;
    barometer_data.ground_reference_reserved = 0U;
    barometer_data.ground_reference_error_pa = 0.0f;
    barometer_data.ground_reference_last_step_pa = 0.0f;
    barometer_data.ground_reference_update_count = 0UL;
    barometer_data.ground_reference_freeze_count = 0UL;
    barometer_data.ground_reference_last_update_us = 0UL;

    barometer_data.altitude_m = 0.0f;
    barometer_data.filtered_altitude_m = 0.0f;
    barometer_data.vertical_speed_mps = 0.0f;

    barometer_data.d1_raw = 0UL;
    barometer_data.d2_raw = 0UL;

    barometer_data.update_count = 0UL;
    barometer_data.source_update_count = 0UL;
    barometer_data.valid_sample_count = 0UL;
    barometer_data.invalid_sample_count = 0UL;
    barometer_data.calibration_sample_count = 0UL;

    barometer_data.duplicate_sample_skip_count = 0UL;

    barometer_data.median_update_count = 0UL;
    barometer_data.median_rejected_spike_count = 0UL;
    barometer_data.raw_spike_reject_count = 0UL;
    barometer_data.raw_step_confirm_count = 0UL;

    barometer_data.filter_update_count = 0UL;
    barometer_data.filter_reset_count = 0UL;

    barometer_data.last_sample_timestamp_us = 0UL;
    barometer_data.last_sample_interval_us = 0UL;
    barometer_data.min_sample_interval_us = 0UL;
    barometer_data.max_sample_interval_us = 0UL;

    barometer_data.last_measure_duration_us = 0UL;
    barometer_data.max_measure_duration_us = 0UL;

    pressure_median_window[0] = 0.0f;
    pressure_median_window[1] = 0.0f;
    pressure_median_window[2] = 0.0f;
    pressure_median_count = 0U;
    pressure_median_index = 0U;

    calibration_pressure_sum = 0.0f;
    previous_filtered_altitude_m = 0.0f;

    last_ms5611_update_count = 0UL;
    last_valid_sample_timestamp_us = 0UL;
    baro_raw_reference_valid = 0U;
    baro_last_plausible_pressure_pa = 0.0f;
    baro_last_plausible_timestamp_us = 0UL;
    baro_raw_step_candidate_valid = 0U;
    baro_raw_step_candidate_pa = 0.0f;

    Barometer_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */

static uint8_t Barometer_InitFilters(void)
{
#if (APP_BARO_BUTTERWORTH_ENABLED != 0U)
    uint8_t ok = 1U;

    ok &= Butterworth2LPF_Init(
        &pressure_filter,
        APP_BARO_FILTER_SAMPLE_RATE_HZ,
        APP_BARO_PRESSURE_LPF_CUTOFF_HZ
    );

    ok &= Butterworth2LPF_Init(
        &vertical_speed_filter,
        APP_BARO_FILTER_SAMPLE_RATE_HZ,
        APP_BARO_VERTICAL_SPEED_LPF_CUTOFF_HZ
    );

    barometer_data.filter_enabled = 1U;
    barometer_data.filter_config_ok = ok;

    barometer_pressure_filter_b0 = pressure_filter.b0;
    barometer_pressure_filter_b1 = pressure_filter.b1;
    barometer_pressure_filter_b2 = pressure_filter.b2;
    barometer_pressure_filter_a1 = pressure_filter.a1;
    barometer_pressure_filter_a2 = pressure_filter.a2;

    barometer_vertical_speed_filter_b0 = vertical_speed_filter.b0;
    barometer_vertical_speed_filter_b1 = vertical_speed_filter.b1;
    barometer_vertical_speed_filter_b2 = vertical_speed_filter.b2;
    barometer_vertical_speed_filter_a1 = vertical_speed_filter.a1;
    barometer_vertical_speed_filter_a2 = vertical_speed_filter.a2;

    return ok;
#else
    barometer_data.filter_enabled = 0U;
    barometer_data.filter_config_ok = 1U;
    return 1U;
#endif
}

/* -------------------------------------------------------------------------- */

static float Barometer_Median3(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

/* -------------------------------------------------------------------------- */

static void Barometer_ResetMedian(float pressure_pa)
{
    pressure_median_window[0] = pressure_pa;
    pressure_median_window[1] = pressure_pa;
    pressure_median_window[2] = pressure_pa;
    pressure_median_count = 3U;
    pressure_median_index = 0U;
    barometer_data.median_pressure_pa = pressure_pa;
}

/* -------------------------------------------------------------------------- */

static float Barometer_ProcessMedian(float pressure_pa)
{
    float median_pressure_pa;
    pressure_median_window[pressure_median_index] = pressure_pa;
    pressure_median_index = (uint8_t)((pressure_median_index + 1U) % 3U);
    if (pressure_median_count < 3U) { pressure_median_count++; }
    median_pressure_pa = (pressure_median_count < 3U) ? pressure_pa :
        Barometer_Median3(pressure_median_window[0], pressure_median_window[1], pressure_median_window[2]);
    if ((median_pressure_pa - pressure_pa > 0.5f) || (pressure_pa - median_pressure_pa > 0.5f))
    {
        barometer_data.median_rejected_spike_count++;
    }
    barometer_data.median_pressure_pa = median_pressure_pa;
    barometer_data.median_update_count++;
    return median_pressure_pa;
}

/* -------------------------------------------------------------------------- */

static uint8_t Barometer_IsPressureInRange(float pressure_pa)
{
    if (pressure_pa < APP_BARO_PRESSURE_MIN_PA)
    {
        return 0U;
    }

    if (pressure_pa > APP_BARO_PRESSURE_MAX_PA)
    {
        return 0U;
    }

    return 1U;
}

static float Barometer_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t Barometer_RawPressurePlausible(
    float pressure_pa,
    uint32_t now_us
)
{
#if (APP_BARO_RAW_STEP_GUARD_ENABLED != 0U)
    float delta_pa;
    float allowed_pa;
    uint32_t dt_us;

    if (baro_raw_reference_valid == 0U)
    {
        baro_raw_reference_valid = 1U;
        baro_last_plausible_pressure_pa = pressure_pa;
        baro_last_plausible_timestamp_us = now_us;
        baro_raw_step_candidate_valid = 0U;
        return 1U;
    }

    delta_pa = Barometer_AbsFloat(
        pressure_pa - baro_last_plausible_pressure_pa
    );

    /* A jump larger than the complete near-ground mission envelope is never
     * accepted, even if a bus fault repeats the same corrupt word twice. */
    if (delta_pa > APP_BARO_RAW_HARD_JUMP_PA)
    {
        baro_raw_step_candidate_pa = pressure_pa;
        baro_raw_step_candidate_valid = 1U;
        barometer_data.raw_spike_reject_count++;
        return 0U;
    }

    dt_us = (uint32_t)(now_us - baro_last_plausible_timestamp_us);
    allowed_pa = APP_BARO_RAW_BASE_JUMP_PA +
        (APP_BARO_RAW_MAX_RATE_PA_PER_S * ((float)dt_us * 0.000001f));

    if (allowed_pa > APP_BARO_RAW_HARD_JUMP_PA)
    {
        allowed_pa = APP_BARO_RAW_HARD_JUMP_PA;
    }

    if (delta_pa <= allowed_pa)
    {
        if ((baro_raw_step_candidate_valid != 0U) &&
            (Barometer_AbsFloat(pressure_pa - baro_raw_step_candidate_pa) <=
             APP_BARO_RAW_STEP_CONFIRM_TOLERANCE_PA))
        {
            /* A sustained, time-consistent new level was eventually reachable
             * under the physical rate limit. Keep the legacy diagnostic name. */
            barometer_data.raw_step_confirm_count++;
        }

        baro_last_plausible_pressure_pa = pressure_pa;
        baro_last_plausible_timestamp_us = now_us;
        baro_raw_step_candidate_valid = 0U;
        return 1U;
    }

    baro_raw_step_candidate_pa = pressure_pa;
    baro_raw_step_candidate_valid = 1U;
    barometer_data.raw_spike_reject_count++;
    return 0U;
#else
    baro_last_plausible_pressure_pa = pressure_pa;
    baro_last_plausible_timestamp_us = now_us;
    baro_raw_reference_valid = 1U;
    baro_raw_step_candidate_valid = 0U;
    return 1U;
#endif
}

/* -------------------------------------------------------------------------- */

static float Barometer_ComputeAltitudeLinear(
    float pressure_pa,
    float ground_pressure_pa
)
{
    /*
     * Existing near-ground approximation is preserved:
     * approximately 12 Pa per metre around sea-level conditions.
     */
    if (ground_pressure_pa <= 0.0f)
    {
        return 0.0f;
    }

    return (ground_pressure_pa - pressure_pa) / 12.0f;
}

/* P53: update the zero-pressure datum only when the app-level stationary gate
 * is open. This corrects slow warm-up / ambient pressure drift without
 * following real flight motion. The step clamp makes the reference robust to
 * a transient gate mistake; the one-way flight freeze is the primary safety
 * barrier. */
static void Barometer_UpdateGroundReferenceTracking(
    float filtered_pressure_pa,
    uint32_t now_us
)
{
#if (APP_BARO_GROUND_TRACKING_ENABLED != 0U)
    float error_pa;
    float step_pa;

    barometer_data.ground_reference_tracking_active = 0U;
    barometer_data.ground_reference_last_step_pa = 0.0f;

    if ((barometer_data.calibrated == 0U) ||
        (barometer_data.ground_reference_frozen != 0U) ||
        (barometer_data.ground_reference_tracking_allowed == 0U) ||
        (filtered_pressure_pa <= 0.0f))
    {
        return;
    }

    error_pa = filtered_pressure_pa - barometer_data.ground_pressure_pa;
    step_pa = APP_BARO_GROUND_TRACK_ALPHA * error_pa;

    if (step_pa > APP_BARO_GROUND_TRACK_MAX_STEP_PA)
    {
        step_pa = APP_BARO_GROUND_TRACK_MAX_STEP_PA;
    }
    else if (step_pa < -APP_BARO_GROUND_TRACK_MAX_STEP_PA)
    {
        step_pa = -APP_BARO_GROUND_TRACK_MAX_STEP_PA;
    }

    barometer_data.ground_pressure_pa += step_pa;
    barometer_data.ground_reference_tracking_active = 1U;
    barometer_data.ground_reference_error_pa = error_pa;
    barometer_data.ground_reference_last_step_pa = step_pa;
    barometer_data.ground_reference_update_count++;
    barometer_data.ground_reference_last_update_us = now_us;
#else
    (void)filtered_pressure_pa;
    (void)now_us;
    barometer_data.ground_reference_tracking_active = 0U;
#endif
}

/* -------------------------------------------------------------------------- */

static void Barometer_SaveSampleInterval(uint32_t now_us)
{
    uint32_t interval_us;

    if (last_valid_sample_timestamp_us == 0UL)
    {
        barometer_data.last_sample_interval_us = 0UL;
        return;
    }

    interval_us = now_us - last_valid_sample_timestamp_us;

    barometer_data.last_sample_interval_us = interval_us;

    if ((barometer_data.min_sample_interval_us == 0UL) ||
        (interval_us < barometer_data.min_sample_interval_us))
    {
        barometer_data.min_sample_interval_us = interval_us;
    }

    if (interval_us > barometer_data.max_sample_interval_us)
    {
        barometer_data.max_sample_interval_us = interval_us;
    }
}

/* -------------------------------------------------------------------------- */

static void Barometer_ResetFilters(float pressure_pa)
{
    float initial_altitude;

    Barometer_ResetMedian(pressure_pa);

    initial_altitude = Barometer_ComputeAltitudeLinear(
        pressure_pa,
        barometer_data.ground_pressure_pa
    );

#if (APP_BARO_BUTTERWORTH_ENABLED != 0U)
    if (barometer_data.filter_config_ok != 0U)
    {
        Butterworth2LPF_Reset(&pressure_filter, pressure_pa);
        Butterworth2LPF_Reset(&vertical_speed_filter, 0.0f);
        barometer_data.filter_initialized = 1U;
        barometer_data.filter_reset_count++;
    }
    else
    {
        barometer_data.filter_initialized = 0U;
    }
#else
    barometer_data.filter_initialized = 0U;
#endif

    barometer_data.filtered_pressure_pa = pressure_pa;
    barometer_data.filtered_altitude_m = initial_altitude;
    barometer_data.vertical_speed_mps = 0.0f;

    previous_filtered_altitude_m = initial_altitude;
}

/* -------------------------------------------------------------------------- */

static void Barometer_UpdateFilters(
    float median_pressure_pa,
    uint32_t now_us
)
{
    uint32_t gap_us;
    float dt_s;
    float raw_vertical_speed_mps;

    gap_us = now_us - last_valid_sample_timestamp_us;

#if (APP_BARO_BUTTERWORTH_ENABLED != 0U)
    if ((barometer_data.filter_config_ok == 0U) ||
        (barometer_data.filter_enabled == 0U))
    {
        barometer_data.filtered_pressure_pa = median_pressure_pa;
        barometer_data.filtered_altitude_m =
            Barometer_ComputeAltitudeLinear(
                median_pressure_pa,
                barometer_data.ground_pressure_pa
            );
        barometer_data.vertical_speed_mps = 0.0f;
        barometer_data.filter_initialized = 0U;
        return;
    }

    if ((barometer_data.filter_initialized == 0U) ||
        (last_valid_sample_timestamp_us == 0UL) ||
        (gap_us > APP_BARO_FILTER_RESET_GAP_US))
    {
        Barometer_ResetFilters(median_pressure_pa);
        barometer_data.filter_update_count++;
        return;
    }

    barometer_data.filtered_pressure_pa =
        Butterworth2LPF_Process(
            &pressure_filter,
            median_pressure_pa
        );

    barometer_data.filtered_altitude_m =
        Barometer_ComputeAltitudeLinear(
            barometer_data.filtered_pressure_pa,
            barometer_data.ground_pressure_pa
        );

    dt_s = (float)gap_us * 0.000001f;

    if (dt_s > 0.0f)
    {
        raw_vertical_speed_mps =
            (barometer_data.filtered_altitude_m -
             previous_filtered_altitude_m) / dt_s;

        barometer_data.vertical_speed_mps =
            Butterworth2LPF_Process(
                &vertical_speed_filter,
                raw_vertical_speed_mps
            );
    }

    previous_filtered_altitude_m =
        barometer_data.filtered_altitude_m;

    barometer_data.filter_update_count++;
#else
    (void)gap_us;
    (void)dt_s;
    (void)raw_vertical_speed_mps;

    barometer_data.filtered_pressure_pa = median_pressure_pa;
    barometer_data.filtered_altitude_m =
        Barometer_ComputeAltitudeLinear(
            median_pressure_pa,
            barometer_data.ground_pressure_pa
        );
    barometer_data.vertical_speed_mps = 0.0f;
#endif
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void Barometer_Init(void)
{
    Barometer_ResetData();
    (void)Barometer_InitFilters();

    if (MS5611_SPI_Init(&hspi2) == 0U)
    {
        barometer_data.initialized = 0U;
        barometer_data.connected = 0U;
        barometer_data.healthy = 0U;

        Barometer_UpdateLiveDebug();
        return;
    }

    barometer_data.initialized = 1U;
    barometer_data.connected = MS5611_SPI_IsConnected();

    barometer_data.data_ready = 0U;
    barometer_data.pressure_valid = 0U;
    barometer_data.calibrated = 0U;
    barometer_data.healthy = 0U;

    Barometer_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */

void Barometer_Update(void)
{
    uint32_t start_us = micros();
    uint32_t now_us;
    uint32_t duration_us;
    float median_pressure_pa;
    MS5611_SPI_Data_t ms;

    if ((barometer_data.initialized == 0U) ||
        (barometer_data.connected == 0U))
    {
        barometer_data.healthy = 0U;
        Barometer_UpdateLiveDebug();
        return;
    }

    /*
     * BMP585 is read directly at 200 Hz. Filtering and altitude calculations
     * run only when the low-level fresh-sample update_count changes.
     */
    MS5611_SPI_Update();
    ms = MS5611_SPI_GetData();

    barometer_data.connected = ms.connected;
    /* P112R12R8R4: low-level recovery may temporarily clear ms.data_ready.
     * Keep the already accepted barometer output available while it is still
     * inside the same 50 ms freshness budget; only the stale branch below
     * clears data_ready/valid. This prevents a 5-15 ms bounded recovery from
     * producing a false SYS_FAULT_BAROMETER_NO_DATA pulse. */
    if (ms.data_ready != 0U)
    {
        barometer_data.data_ready = 1U;
    }
    /* P112R12R8R3: temperature belongs to the same atomic P/T sample as
     * pressure. Do not publish a newly-read temperature until that pressure
     * sample passes the upper-layer plausibility guard below. */
    barometer_data.source_update_count = ms.update_count;

    if (ms.max_transaction_duration_us >
        barometer_data.max_measure_duration_us)
    {
        barometer_data.max_measure_duration_us =
            ms.max_transaction_duration_us;
    }

    if (ms.update_count == last_ms5611_update_count)
    {
        uint32_t age_us = 0xFFFFFFFFUL;

        if (ms.data_ready != 0U)
        {
            barometer_data.duplicate_sample_skip_count++;
        }

        if (last_valid_sample_timestamp_us != 0UL)
        {
            age_us = micros() - last_valid_sample_timestamp_us;
        }

        if (age_us > APP_BARO_STALE_TIMEOUT_US)
        {
            barometer_data.data_ready = 0U;
            barometer_data.pressure_valid = 0U;
            barometer_data.healthy = 0U;
        }
        else
        {
            barometer_data.healthy =
                (barometer_data.connected != 0U) &&
                (barometer_data.data_ready != 0U) &&
                (barometer_data.pressure_valid != 0U) &&
                (barometer_data.calibrated != 0U);
        }

        duration_us = micros() - start_us;
        barometer_data.last_measure_duration_us = duration_us;

        if (duration_us > barometer_data.max_measure_duration_us)
        {
            barometer_data.max_measure_duration_us = duration_us;
        }

        Barometer_UpdateLiveDebug();
        return;
    }

    last_ms5611_update_count = ms.update_count;

    if ((ms.adc_ok == 0U) ||
        (ms.data_ready == 0U) ||
        (Barometer_IsPressureInRange(ms.pressure_pa) == 0U))
    {
        barometer_data.pressure_valid = 0U;
        barometer_data.healthy = 0U;
        barometer_data.invalid_sample_count++;

        Barometer_UpdateLiveDebug();
        return;
    }

    now_us = micros();

    if (Barometer_RawPressurePlausible(ms.pressure_pa, now_us) == 0U)
    {
        /* Preserve the last accepted pressure and health. The low-level source
         * did progress, but this isolated sample is deliberately not exposed as
         * a new barometer measurement. */
        barometer_data.healthy =
            (barometer_data.connected != 0U) &&
            (barometer_data.data_ready != 0U) &&
            (barometer_data.pressure_valid != 0U) &&
            (barometer_data.calibrated != 0U);
        Barometer_UpdateLiveDebug();
        return;
    }

    barometer_data.pressure_pa = ms.pressure_pa;
    barometer_data.temperature_c = ms.temperature_c;
    barometer_data.d1_raw = ms.d1_raw;
    barometer_data.d2_raw = ms.d2_raw;

    Barometer_SaveSampleInterval(now_us);

    barometer_data.pressure_valid = 1U;
    barometer_data.valid_sample_count++;
    barometer_data.update_count++;

    if (barometer_data.calibrated == 0U)
    {
        calibration_pressure_sum += ms.pressure_pa;
        barometer_data.calibration_sample_count++;

        if (barometer_data.calibration_sample_count >=
            APP_BARO_CALIBRATION_SAMPLE_COUNT)
        {
            barometer_data.ground_pressure_pa =
                calibration_pressure_sum /
                (float)barometer_data.calibration_sample_count;
            barometer_data.ground_reference_error_pa = 0.0f;
            barometer_data.ground_reference_last_step_pa = 0.0f;

            barometer_data.altitude_m = 0.0f;
            Barometer_ResetFilters(ms.pressure_pa);

            barometer_data.calibrated = 1U;

            last_valid_sample_timestamp_us = now_us;
            barometer_data.last_sample_timestamp_us = now_us;
        }

        barometer_data.healthy =
            (barometer_data.connected != 0U) &&
            (barometer_data.data_ready != 0U) &&
            (barometer_data.pressure_valid != 0U) &&
            (barometer_data.calibrated != 0U);

        duration_us = micros() - start_us;
        barometer_data.last_measure_duration_us = duration_us;

        if (duration_us > barometer_data.max_measure_duration_us)
        {
            barometer_data.max_measure_duration_us = duration_us;
        }

        Barometer_UpdateLiveDebug();
        return;
    }

    barometer_data.altitude_m =
        Barometer_ComputeAltitudeLinear(
            ms.pressure_pa,
            barometer_data.ground_pressure_pa
        );

    median_pressure_pa = Barometer_ProcessMedian(ms.pressure_pa);
    Barometer_UpdateFilters(median_pressure_pa, now_us);

    Barometer_UpdateGroundReferenceTracking(
        barometer_data.filtered_pressure_pa,
        now_us
    );

    if (barometer_data.ground_reference_tracking_active != 0U)
    {
        /* Reference motion is not vehicle vertical motion. Recompute altitude
         * against the updated datum and suppress the artificial derivative. */
        barometer_data.altitude_m = Barometer_ComputeAltitudeLinear(
            ms.pressure_pa,
            barometer_data.ground_pressure_pa
        );
        barometer_data.filtered_altitude_m = Barometer_ComputeAltitudeLinear(
            barometer_data.filtered_pressure_pa,
            barometer_data.ground_pressure_pa
        );
        barometer_data.vertical_speed_mps = 0.0f;
        previous_filtered_altitude_m = barometer_data.filtered_altitude_m;
    }

    last_valid_sample_timestamp_us = now_us;
    barometer_data.last_sample_timestamp_us = now_us;

    barometer_data.healthy =
        (barometer_data.connected != 0U) &&
        (barometer_data.data_ready != 0U) &&
        (barometer_data.pressure_valid != 0U) &&
        (barometer_data.calibrated != 0U);

    duration_us = micros() - start_us;
    barometer_data.last_measure_duration_us = duration_us;

    if (duration_us > barometer_data.max_measure_duration_us)
    {
        barometer_data.max_measure_duration_us = duration_us;
    }

    Barometer_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */

void Barometer_SetGroundReferenceTrackingAllowed(uint8_t allowed)
{
    if (barometer_data.ground_reference_frozen != 0U)
    {
        barometer_data.ground_reference_tracking_allowed = 0U;
        barometer_data.ground_reference_tracking_active = 0U;
        return;
    }

    barometer_data.ground_reference_tracking_allowed =
        (allowed != 0U) ? 1U : 0U;

    if (allowed == 0U)
    {
        barometer_data.ground_reference_tracking_active = 0U;
        barometer_data.ground_reference_last_step_pa = 0.0f;
    }
}

void Barometer_FreezeGroundReference(void)
{
    if (barometer_data.ground_reference_frozen == 0U)
    {
        barometer_data.ground_reference_frozen = 1U;
        barometer_data.ground_reference_freeze_count++;
    }

    barometer_data.ground_reference_tracking_allowed = 0U;
    barometer_data.ground_reference_tracking_active = 0U;
    barometer_data.ground_reference_last_step_pa = 0.0f;
}

uint8_t Barometer_IsConnected(void)
{
    return barometer_data.connected;
}

uint8_t Barometer_IsDataReady(void)
{
    return barometer_data.data_ready;
}

uint8_t Barometer_IsPressureValid(void)
{
    return barometer_data.pressure_valid;
}

uint8_t Barometer_IsCalibrated(void)
{
    return barometer_data.calibrated;
}

uint8_t Barometer_IsHealthy(void)
{
    return barometer_data.healthy;
}

BarometerData_t Barometer_GetData(void)
{
    return barometer_data;
}

const BarometerData_t *Barometer_GetDataPtr(void)
{
    return &barometer_data;
}
