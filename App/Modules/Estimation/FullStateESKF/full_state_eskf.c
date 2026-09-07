#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"

#include "Common/app_config.h"
#include "Modules/Estimation/AttitudeEstimator/attitude_estimator.h"
#include "Modules/Sensors/Barometer/barometer.h"
#include "Modules/Sensors/Lidar/Lidar.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"
#include "Services/Timebase/timebase.h"

#include "stm32f4xx.h"

#include <math.h>
#include <float.h>
#include <stdint.h>
#include <string.h>

#define FULL_ESKF_STATE_COUNT             15U

#define FULL_ESKF_PX                      0U
#define FULL_ESKF_PY                      1U
#define FULL_ESKF_PZ                      2U
#define FULL_ESKF_VX                      3U
#define FULL_ESKF_VY                      4U
#define FULL_ESKF_VZ                      5U
#define FULL_ESKF_THETA_X                 6U
#define FULL_ESKF_THETA_Y                 7U
#define FULL_ESKF_THETA_Z                 8U
#define FULL_ESKF_BA_X                    9U
#define FULL_ESKF_BA_Y                   10U
#define FULL_ESKF_BA_Z                   11U
#define FULL_ESKF_BG_X                   12U
#define FULL_ESKF_BG_Y                   13U
#define FULL_ESKF_BG_Z                   14U

#define FULL_ESKF_DEG_TO_RAD              0.01745329251994329577f
#define FULL_ESKF_RAD_TO_DEG              57.295779513082320876f
#define FULL_ESKF_MIN_VARIANCE            1.0e-10f
#define FULL_ESKF_MAX_REASONABLE_VALUE     1000000.0f

/* XY-only containment for this build.
 *
 * There is no absolute horizontal position sensor in the current system, so
 * horizontal_position_valid remains 0 and raw inertial X/Y can drift over a
 * long bench test.  Keep the proven 15-state ESKF untouched for Z/attitude and
 * maintain a separate, bounded *diagnostic* X/Y projection for public/logged
 * output.  The diagnostic projection high-pass removes very slow horizontal
 * acceleration bias/gravity leakage, while preserving short-duration motion.
 * It is never marked as an absolute horizontal navigation solution and is not
 * allowed to change LIDAR, barometer, Z, quaternion, covariance or control
 * authority. */
#define FULL_ESKF_XY_RUNAWAY_POSITION_LIMIT_M    250.0f
#define FULL_ESKF_XY_RUNAWAY_VELOCITY_LIMIT_MPS  100.0f
#define FULL_ESKF_XY_DIAG_ACCEL_BIAS_TAU_S         3.0f
#define FULL_ESKF_XY_DIAG_STATIONARY_BIAS_TAU_S    0.50f
#define FULL_ESKF_XY_DIAG_VELOCITY_DAMP_TAU_S      8.0f
#define FULL_ESKF_XY_DIAG_POSITION_DAMP_TAU_S     60.0f
#define FULL_ESKF_XY_DIAG_POSITION_LIMIT_M         25.0f
#define FULL_ESKF_XY_DIAG_VELOCITY_LIMIT_MPS        5.0f

/* R8R9 covariance timing containment.
 *
 * Physical R8R8 data proved that the 200 Hz correction path is already
 * bounded (pure correction <=553 us), while the separate 25 Hz covariance
 * propagation reached ~980 us in the CubeIDE Debug build.  The Debug
 * configuration intentionally compiles most code without optimization, which
 * makes the tight 15x15 covariance loops dominated by loop/function-call
 * overhead.  Optimize ONLY the covariance propagation/sanitization kernels.
 * No -ffast-math is enabled, no estimator constants/gates/noise are changed,
 * and the rest of the firmware remains in the existing build configuration.
 */
#if defined(__GNUC__)
#define FULL_ESKF_COV_TIMING_OPT __attribute__((optimize("O2")))
#define FULL_ESKF_FORCE_INLINE   inline __attribute__((always_inline))
#else
#define FULL_ESKF_COV_TIMING_OPT
#define FULL_ESKF_FORCE_INLINE   inline
#endif

/* P51 non-blocking stationary-maintenance phases.
 * One scalar correction is executed per 200 Hz correction service call,
 * instead of bursting 6-9 corrections in one cooperative-scheduler slot. */
#define FULL_ESKF_STATIONARY_PHASE_IDLE      0U
#define FULL_ESKF_STATIONARY_PHASE_ZUPT_X    1U
#define FULL_ESKF_STATIONARY_PHASE_ZUPT_Y    2U
#define FULL_ESKF_STATIONARY_PHASE_ZUPT_Z    3U
#define FULL_ESKF_STATIONARY_PHASE_GYRO_X    4U
#define FULL_ESKF_STATIONARY_PHASE_GYRO_Y    5U
#define FULL_ESKF_STATIONARY_PHASE_GYRO_Z    6U
#define FULL_ESKF_STATIONARY_PHASE_ACCEL_X   7U
#define FULL_ESKF_STATIONARY_PHASE_ACCEL_Y   8U
#define FULL_ESKF_STATIONARY_PHASE_ACCEL_Z   9U

/* R8R8 deterministic gravity-aiding phases.  A 50 Hz gravity batch is
 * linearized once, then one attitude-only Joseph update is executed per
 * 200 Hz correction call.  This preserves the three-axis batch while
 * removing the single-slot 3-update burst seen in R8R7 timing. */
#define FULL_ESKF_GRAVITY_PHASE_IDLE          0U
#define FULL_ESKF_GRAVITY_PHASE_X             1U
#define FULL_ESKF_GRAVITY_PHASE_Y             2U
#define FULL_ESKF_GRAVITY_PHASE_Z             3U

#if (APP_FULL_ESKF_GRAVITY_AXES_PER_CORRECTION != 1U)
#error "R8R8 requires exactly one gravity axis per 200 Hz correction slot"
#endif

static FullStateESKFData_t eskf_data;

static float nominal_position[3];
static float nominal_velocity[3];
static float nominal_quaternion[4];
static float nominal_accel_bias[3];
static float nominal_gyro_bias[3];
static float covariance[FULL_ESKF_STATE_COUNT][FULL_ESKF_STATE_COUNT];
static float covariance_temp[FULL_ESKF_STATE_COUNT][FULL_ESKF_STATE_COUNT];
/* P46: last matrix that passed the complete integrity check. */
static float covariance_last_good[FULL_ESKF_STATE_COUNT][FULL_ESKF_STATE_COUNT];
static uint8_t covariance_last_good_valid = 0U;

#if (APP_FULL_ESKF_SPARSE_COVARIANCE_ENABLED == 0U)
static float transition[FULL_ESKF_STATE_COUNT][FULL_ESKF_STATE_COUNT];
#endif

static uint32_t last_baro_source_update_count = 0UL;
static uint32_t last_lidar_source_update_count = 0UL;

static float baro_reference_sum = 0.0f;
static uint8_t baro_reference_locked = 0U;

static float lidar_reference_sum = 0.0f;

static uint8_t origin_zero_applied = 0U;
static float lidar_reference_last_sample_m = 0.0f;

/* Un-aided horizontal diagnostic projection.  These states are deliberately
 * separate from nominal_position/nominal_velocity so drift containment cannot
 * feed back into the proven ESKF, vertical fusion or attitude solution. */
static float xy_diag_position_m[2];
static float xy_diag_velocity_mps[2];
static float xy_diag_accel_bias_mps2[2];
static uint8_t xy_diag_initialized = 0U;

static uint32_t covariance_sample_counter = 0UL;
static float covariance_dt_accum_s = 0.0f;
static float covariance_accel_integral[3];
static float covariance_gyro_integral[3];

static uint32_t live_debug_decimation_counter = 0UL;
static uint32_t covariance_health_check_decimation_counter = 0UL;
/* P112R12R8R6: bounded incremental covariance integrity scan.  The old
 * monolithic 15x15 scan could push both the 200 Hz correction and 25 Hz
 * covariance task beyond one IMU period. A bounded row slice is checked per
 * correction; catastrophic diagonal/non-finite faults are still guarded by
 * the fast check every 5 ms. */
static uint8_t covariance_integrity_slice_row = 0U;
static uint8_t covariance_integrity_slice_reason = FULL_ESKF_RESET_REASON_NONE;
static float covariance_integrity_slice_diag_min = 0.0f;
static float covariance_integrity_slice_diag_max = 0.0f;
static float covariance_integrity_slice_symmetry_max = 0.0f;
static uint8_t covariance_integrity_slice_diag_seen = 0U;
static uint32_t gravity_update_decimation_counter = 0UL;
static uint8_t gravity_update_phase = FULL_ESKF_GRAVITY_PHASE_IDLE;
static uint8_t gravity_batch_accepted_count = 0U;
static float gravity_batch_measured_up[3];
static float gravity_batch_predicted_up[3];
static float gravity_batch_measurement_variance = 0.0f;
static uint32_t stationary_update_decimation_counter = 0UL;
static uint8_t stationary_update_phase = FULL_ESKF_STATIONARY_PHASE_IDLE;
static uint32_t lidar_update_decimation_counter = 0UL;
static uint32_t stationary_confirmed_sample_count = 0UL;

static float stationary_gyro_lpf_dps[3];
static float vibration_previous_norm_g = 1.0f;
static float vibration_metric_ema_g = 0.0f;
static uint8_t vibration_metric_initialized = 0U;

static uint8_t gyro_bootstrap_done = 0U;
static uint32_t gyro_bootstrap_attempt_count = 0UL;
static uint32_t gyro_bootstrap_sample_count = 0UL;
static uint32_t gyro_bootstrap_quality_score = 0UL;
static float gyro_bootstrap_mean_dps[3];
static float gyro_bootstrap_m2_dps2[3];

static uint8_t lidar_recovery_active = 0U;
static uint8_t lidar_last_accepted_valid = 0U;
static uint32_t lidar_reacquire_count = 0UL;
static float lidar_last_accepted_distance_m = 0.0f;
static float lidar_reacquire_candidate_m = 0.0f;

/* P39: separate from the raw-distance jump recovery above. This state handles
 * a stable LIDAR stream that has drifted outside the normal ESKF innovation
 * gate because the barometer slowly pulled PZ away. */
static uint8_t lidar_innov_reacquire_active = 0U;
static uint32_t lidar_innov_reacquire_confirm_count = 0UL;
static uint32_t lidar_innov_reacquire_exit_count = 0UL;
volatile uint32_t full_eskf_lidar_innov_reacquire_count = 0UL;
volatile uint32_t full_eskf_lidar_innov_reacquire_success_count = 0UL;

static uint32_t vertical_divergence_candidate_count = 0UL;
static uint32_t vertical_reacquire_stable_count = 0UL;

/* P45: publication and covariance integrity guards. */
static uint8_t public_vertical_guard_armed = 0U;
static float public_vertical_guard_z_m = 0.0f;
static float public_vertical_guard_vz_mps = 0.0f;
static uint32_t public_vertical_guard_timestamp_us = 0UL;
static uint8_t covariance_sanitization_fault_pending = 0U;
static uint8_t covariance_sanitization_fault_reason =
    FULL_ESKF_RESET_REASON_NONE;

static void FullESKF_ClearCovarianceAccumulator(void);

/* -------------------------------------------------------------------------- */
/* Live Expressions                                                           */
/* -------------------------------------------------------------------------- */

volatile uint8_t full_eskf_enabled = 0U;
volatile uint8_t full_eskf_shadow_mode = 0U;
volatile uint8_t full_eskf_initialized = 0U;
volatile uint8_t full_eskf_healthy = 0U;
volatile uint8_t full_eskf_baro_reference_ready = 0U;
volatile uint8_t full_eskf_lidar_reference_ready = 0U;
volatile uint8_t full_eskf_gravity_correction_active = 0U;
volatile uint8_t full_eskf_stationary_detected = 0U;

volatile uint8_t full_eskf_horizontal_position_valid = 0U;
volatile uint8_t full_eskf_vertical_position_valid = 0U;
volatile uint8_t full_eskf_baro_fresh = 0U;
volatile uint8_t full_eskf_lidar_fresh = 0U;

volatile uint8_t full_eskf_origin_zeroed = 0U;
volatile uint8_t full_eskf_output_inhibited = 0U;
volatile uint8_t full_eskf_vertical_reacquire_active = 0U;
volatile uint8_t full_eskf_vertical_divergence_reason = 0U;
volatile uint32_t full_eskf_vertical_divergence_count = 0UL;
volatile uint32_t full_eskf_vertical_reacquire_count = 0UL;
volatile uint8_t full_eskf_reset_reason = FULL_ESKF_RESET_REASON_NONE;
volatile uint8_t full_eskf_covariance_integrity_ok = 0U;
volatile uint32_t full_eskf_public_output_reject_count = 0UL;
volatile uint32_t full_eskf_public_z_jump_reject_count = 0UL;
volatile uint32_t full_eskf_public_vz_jump_reject_count = 0UL;
volatile uint32_t full_eskf_covariance_integrity_check_count = 0UL;
volatile uint32_t full_eskf_covariance_fault_count = 0UL;
volatile uint32_t full_eskf_covariance_reinit_count = 0UL;
volatile float full_eskf_covariance_diag_min = 0.0f;
volatile float full_eskf_covariance_diag_max = 0.0f;
volatile float full_eskf_covariance_symmetry_error_max = 0.0f;
volatile uint8_t full_eskf_covariance_fault_stage = FULL_ESKF_COV_STAGE_NONE;
volatile uint8_t full_eskf_covariance_fault_state_index = FULL_ESKF_COV_STATE_INVALID;
volatile uint8_t full_eskf_covariance_fault_other_index = FULL_ESKF_COV_STATE_INVALID;
volatile uint8_t full_eskf_covariance_fault_used_last_good = 0U;
volatile float full_eskf_covariance_fault_raw_value = 0.0f;
volatile float full_eskf_covariance_fault_aux_value = 0.0f;
volatile uint32_t full_eskf_covariance_fault_timestamp_us = 0UL;
volatile uint8_t full_eskf_covariance_roundoff_last_stage = FULL_ESKF_COV_STAGE_NONE;
volatile uint8_t full_eskf_covariance_roundoff_last_state_index = FULL_ESKF_COV_STATE_INVALID;
volatile float full_eskf_covariance_roundoff_last_raw_value = 0.0f;
volatile uint32_t full_eskf_covariance_roundoff_last_timestamp_us = 0UL;
volatile uint32_t full_eskf_covariance_state_preserving_recovery_count = 0UL;
volatile uint32_t full_eskf_covariance_roundoff_clamp_count = 0UL;
volatile float full_eskf_vertical_sensor_consistency_m = 0.0f;
volatile float full_eskf_vertical_reacquire_target_m = 0.0f;
volatile uint32_t full_eskf_origin_zero_count = 0UL;
volatile uint32_t full_eskf_origin_zero_timestamp_us = 0UL;
volatile float full_eskf_origin_pre_position_x_m = 0.0f;
volatile float full_eskf_origin_pre_position_y_m = 0.0f;
volatile float full_eskf_origin_pre_position_z_m = 0.0f;

volatile uint32_t full_eskf_baro_sample_age_us = 0xFFFFFFFFUL;
volatile uint32_t full_eskf_lidar_sample_age_us = 0xFFFFFFFFUL;
volatile uint32_t full_eskf_baro_reference_invalidation_count = 0UL;

volatile uint8_t full_eskf_gyro_bootstrap_active = 0U;
volatile uint8_t full_eskf_gyro_bootstrap_done = 0U;
volatile uint32_t full_eskf_gyro_bootstrap_sample_count = 0UL;
volatile uint32_t full_eskf_gyro_bootstrap_attempt_count = 0UL;
volatile uint32_t full_eskf_gyro_bootstrap_quality_score = 0UL;
volatile float full_eskf_gyro_bootstrap_std_x_dps = 0.0f;
volatile float full_eskf_gyro_bootstrap_std_y_dps = 0.0f;
volatile float full_eskf_gyro_bootstrap_std_z_dps = 0.0f;
volatile float full_eskf_stationary_gyro_lpf_norm_dps = 0.0f;
volatile uint32_t full_eskf_stationary_confirmed_sample_count = 0UL;

volatile float full_eskf_position_x_m = 0.0f;
volatile float full_eskf_position_y_m = 0.0f;
volatile float full_eskf_position_z_m = 0.0f;
volatile float full_eskf_velocity_x_mps = 0.0f;
volatile float full_eskf_velocity_y_mps = 0.0f;
volatile float full_eskf_velocity_z_mps = 0.0f;

volatile float full_eskf_roll_deg = 0.0f;
volatile float full_eskf_pitch_deg = 0.0f;
volatile float full_eskf_yaw_deg = 0.0f;

volatile float full_eskf_accel_bias_x_mps2 = 0.0f;
volatile float full_eskf_accel_bias_y_mps2 = 0.0f;
volatile float full_eskf_accel_bias_z_mps2 = 0.0f;
volatile float full_eskf_gyro_bias_x_dps = 0.0f;
volatile float full_eskf_gyro_bias_y_dps = 0.0f;
volatile float full_eskf_gyro_bias_z_dps = 0.0f;

volatile float full_eskf_world_accel_x_mps2 = 0.0f;
volatile float full_eskf_world_accel_y_mps2 = 0.0f;
volatile float full_eskf_world_accel_z_mps2 = 0.0f;

volatile float full_eskf_baro_innovation_m = 0.0f;
volatile float full_eskf_lidar_innovation_m = 0.0f;
volatile uint8_t full_eskf_lidar_recovery_active = 0U;
volatile uint32_t full_eskf_lidar_reacquire_count = 0UL;
volatile uint32_t full_eskf_lidar_jump_reject_count = 0UL;

volatile uint32_t full_eskf_predict_count = 0UL;
volatile uint32_t full_eskf_public_output_count = 0UL;
volatile uint32_t full_eskf_last_public_output_timestamp_us = 0UL;
volatile uint32_t full_eskf_covariance_predict_count = 0UL;
volatile uint32_t full_eskf_gravity_update_count = 0UL;
volatile uint32_t full_eskf_gravity_joseph_update_count = 0UL;
volatile uint32_t full_eskf_gravity_joseph_fault_count = 0UL;
/* P49 Live Expressions: masked Joseph ZUPT diagnostics. */
volatile uint32_t full_eskf_zupt_joseph_update_count = 0UL;
volatile uint32_t full_eskf_zupt_joseph_fault_count = 0UL;
volatile uint32_t full_eskf_stationary_detector_count = 0UL;
volatile uint32_t full_eskf_stationary_update_count = 0UL;
volatile uint32_t full_eskf_baro_update_count = 0UL;
volatile uint32_t full_eskf_lidar_update_count = 0UL;
volatile uint32_t full_eskf_baro_reject_count = 0UL;
volatile uint32_t full_eskf_lidar_reject_count = 0UL;
volatile uint32_t full_eskf_reset_count = 0UL;
volatile uint32_t full_eskf_numerical_error_count = 0UL;
volatile uint32_t full_eskf_last_dt_us = 0UL;
volatile uint32_t full_eskf_max_dt_us = 0UL;
volatile uint32_t full_eskf_last_predict_exec_us = 0UL;
volatile uint32_t full_eskf_max_predict_exec_us = 0UL;
volatile uint32_t full_eskf_last_correction_exec_us = 0UL;
volatile uint32_t full_eskf_max_correction_exec_us = 0UL;

/* Cortex-M4 DWT cycle profiler. These values do not change estimator math. */
volatile uint8_t full_eskf_dwt_profiler_enabled = 0U;
volatile uint32_t full_eskf_cpu_clock_hz = 0UL;

volatile uint32_t full_eskf_last_predict_cycles = 0UL;
volatile uint32_t full_eskf_max_predict_cycles = 0UL;
volatile uint32_t full_eskf_total_predict_cycles = 0UL;
volatile uint32_t full_eskf_last_public_output_cycles = 0UL;
volatile uint32_t full_eskf_max_public_output_cycles = 0UL;
volatile uint32_t full_eskf_total_public_output_cycles = 0UL;
volatile uint32_t full_eskf_last_correction_cycles = 0UL;
volatile uint32_t full_eskf_max_correction_cycles = 0UL;
volatile uint32_t full_eskf_total_correction_cycles = 0UL;
volatile uint32_t full_eskf_last_covariance_cycles = 0UL;
volatile uint32_t full_eskf_max_covariance_cycles = 0UL;
volatile uint32_t full_eskf_total_covariance_cycles = 0UL;
volatile uint32_t full_eskf_last_gravity_cycles = 0UL;
volatile uint32_t full_eskf_max_gravity_cycles = 0UL;
volatile uint32_t full_eskf_total_gravity_cycles = 0UL;
volatile uint32_t full_eskf_last_stationary_cycles = 0UL;
volatile uint32_t full_eskf_max_stationary_cycles = 0UL;
volatile uint32_t full_eskf_total_stationary_cycles = 0UL;
volatile uint32_t full_eskf_last_baro_cycles = 0UL;
volatile uint32_t full_eskf_max_baro_cycles = 0UL;
volatile uint32_t full_eskf_total_baro_cycles = 0UL;
volatile uint32_t full_eskf_last_lidar_cycles = 0UL;
volatile uint32_t full_eskf_max_lidar_cycles = 0UL;
volatile uint32_t full_eskf_total_lidar_cycles = 0UL;


/* -------------------------------------------------------------------------- */

static void FullESKF_DWTProfilerInit(void)
{
#if (APP_FULL_ESKF_DWT_PROFILER_ENABLED != 0U)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0UL;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    full_eskf_dwt_profiler_enabled =
        ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0UL) ? 1U : 0U;
    full_eskf_cpu_clock_hz = SystemCoreClock;
#else
    full_eskf_dwt_profiler_enabled = 0U;
    full_eskf_cpu_clock_hz = SystemCoreClock;
#endif
}

/* -------------------------------------------------------------------------- */

static uint32_t FullESKF_DWTStart(void)
{
#if (APP_FULL_ESKF_DWT_PROFILER_ENABLED != 0U)
    if (full_eskf_dwt_profiler_enabled != 0U)
    {
        return DWT->CYCCNT;
    }
#endif

    return 0UL;
}

/* -------------------------------------------------------------------------- */

static uint32_t FullESKF_DWTElapsed(uint32_t start_cycles)
{
#if (APP_FULL_ESKF_DWT_PROFILER_ENABLED != 0U)
    if (full_eskf_dwt_profiler_enabled != 0U)
    {
        return DWT->CYCCNT - start_cycles;
    }
#else
    (void)start_cycles;
#endif

    return 0UL;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_ProfileStore(
    volatile uint32_t *last_cycles,
    volatile uint32_t *max_cycles,
    volatile uint32_t *total_cycles,
    uint32_t elapsed_cycles
)
{
    *last_cycles = elapsed_cycles;
    *total_cycles += elapsed_cycles;

    if (elapsed_cycles > *max_cycles)
    {
        *max_cycles = elapsed_cycles;
    }
}

/* -------------------------------------------------------------------------- */

static float FullESKF_Clamp(float value, float minimum, float maximum)
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

static FULL_ESKF_FORCE_INLINE float FullESKF_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/* -------------------------------------------------------------------------- */

static uint8_t FullESKF_IsFiniteReasonable(float value)
{
    if (value != value)
    {
        return 0U;
    }

    if ((value > FULL_ESKF_MAX_REASONABLE_VALUE) ||
        (value < -FULL_ESKF_MAX_REASONABLE_VALUE))
    {
        return 0U;
    }

    return 1U;
}

/* -------------------------------------------------------------------------- */

/* P52 observability-aware covariance guard.
 *
 * PX/PY are deliberately not declared valid in this build because there is no
 * absolute horizontal-position measurement (no GPS/vision position update).
 * Their covariance is therefore expected to grow during a long preflight/soak.
 * A large *finite* PX/PY covariance is uncertainty, not estimator corruption.
 *
 * Keep all safety checks that matter:
 *   - NaN/Inf is always rejected.
 *   - negative diagonal variance is always rejected.
 *   - symmetry is always checked.
 *   - the 1e6 magnitude sanity limit remains active for every observable /
 *     control-relevant covariance element.
 * Only the arbitrary upper-magnitude test is relaxed for elements touching
 * unobservable PX/PY. */
static FULL_ESKF_FORCE_INLINE uint8_t FullESKF_IsFiniteFloat(float value)
{
    if (value != value)
    {
        return 0U;
    }

    if ((value > FLT_MAX) || (value < -FLT_MAX))
    {
        return 0U;
    }

    return 1U;
}

static FULL_ESKF_FORCE_INLINE uint8_t FullESKF_CovarianceTouchesUnobservedHorizontalPosition(
    uint32_t row,
    uint32_t column
)
{
    return (uint8_t)(((row == FULL_ESKF_PX) ||
                      (row == FULL_ESKF_PY) ||
                      (column == FULL_ESKF_PX) ||
                      (column == FULL_ESKF_PY)) ? 1U : 0U);
}

static FULL_ESKF_FORCE_INLINE uint8_t FullESKF_IsFiniteReasonableCovariance(
    uint32_t row,
    uint32_t column,
    float value
)
{
    if (FullESKF_IsFiniteFloat(value) == 0U)
    {
        return 0U;
    }

    if (FullESKF_CovarianceTouchesUnobservedHorizontalPosition(row, column) != 0U)
    {
        return 1U;
    }

    if ((value > FULL_ESKF_MAX_REASONABLE_VALUE) ||
        (value < -FULL_ESKF_MAX_REASONABLE_VALUE))
    {
        return 0U;
    }

    return 1U;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_UpdateLiveDebug(void)
{
    full_eskf_enabled = eskf_data.enabled;
    full_eskf_shadow_mode = eskf_data.shadow_mode;
    full_eskf_initialized = eskf_data.initialized;
    full_eskf_healthy = eskf_data.healthy;
    full_eskf_baro_reference_ready = eskf_data.baro_reference_ready;
    full_eskf_lidar_reference_ready = eskf_data.lidar_reference_ready;
    full_eskf_gravity_correction_active =
        eskf_data.gravity_correction_active;
    full_eskf_stationary_detected = eskf_data.stationary_detected;

    full_eskf_horizontal_position_valid =
        eskf_data.horizontal_position_valid;
    full_eskf_vertical_position_valid =
        eskf_data.vertical_position_valid;
    full_eskf_baro_fresh = eskf_data.baro_fresh;
    full_eskf_lidar_fresh = eskf_data.lidar_fresh;

    full_eskf_origin_zeroed = eskf_data.origin_zeroed;
    full_eskf_output_inhibited = eskf_data.output_inhibited;
    full_eskf_vertical_reacquire_active = eskf_data.vertical_reacquire_active;
    full_eskf_vertical_divergence_reason = eskf_data.vertical_divergence_reason;
    full_eskf_vertical_divergence_count = eskf_data.vertical_divergence_count;
    full_eskf_vertical_reacquire_count = eskf_data.vertical_reacquire_count;
    full_eskf_reset_reason = eskf_data.reset_reason;
    full_eskf_covariance_integrity_ok = eskf_data.covariance_integrity_ok;
    full_eskf_public_output_reject_count = eskf_data.public_output_reject_count;
    full_eskf_public_z_jump_reject_count = eskf_data.public_z_jump_reject_count;
    full_eskf_public_vz_jump_reject_count = eskf_data.public_vz_jump_reject_count;
    full_eskf_covariance_integrity_check_count =
        eskf_data.covariance_integrity_check_count;
    full_eskf_covariance_fault_count = eskf_data.covariance_fault_count;
    full_eskf_covariance_reinit_count = eskf_data.covariance_reinit_count;
    full_eskf_covariance_diag_min = eskf_data.covariance_diag_min;
    full_eskf_covariance_diag_max = eskf_data.covariance_diag_max;
    full_eskf_covariance_symmetry_error_max =
        eskf_data.covariance_symmetry_error_max;
    full_eskf_covariance_fault_stage = eskf_data.covariance_fault_stage;
    full_eskf_covariance_fault_state_index =
        eskf_data.covariance_fault_state_index;
    full_eskf_covariance_fault_other_index =
        eskf_data.covariance_fault_other_index;
    full_eskf_covariance_fault_used_last_good =
        eskf_data.covariance_fault_used_last_good;
    full_eskf_covariance_fault_raw_value = eskf_data.covariance_fault_raw_value;
    full_eskf_covariance_fault_aux_value = eskf_data.covariance_fault_aux_value;
    full_eskf_covariance_fault_timestamp_us =
        eskf_data.covariance_fault_timestamp_us;
    full_eskf_covariance_roundoff_last_stage =
        eskf_data.covariance_roundoff_last_stage;
    full_eskf_covariance_roundoff_last_state_index =
        eskf_data.covariance_roundoff_last_state_index;
    full_eskf_covariance_roundoff_last_raw_value =
        eskf_data.covariance_roundoff_last_raw_value;
    full_eskf_covariance_roundoff_last_timestamp_us =
        eskf_data.covariance_roundoff_last_timestamp_us;
    full_eskf_covariance_state_preserving_recovery_count =
        eskf_data.covariance_state_preserving_recovery_count;
    full_eskf_covariance_roundoff_clamp_count =
        eskf_data.covariance_roundoff_clamp_count;
    full_eskf_vertical_sensor_consistency_m = eskf_data.vertical_sensor_consistency_m;
    full_eskf_vertical_reacquire_target_m = eskf_data.vertical_reacquire_target_m;
    full_eskf_origin_zero_count = eskf_data.origin_zero_count;
    full_eskf_origin_zero_timestamp_us =
        eskf_data.origin_zero_timestamp_us;
    full_eskf_origin_pre_position_x_m =
        eskf_data.origin_pre_position_x_m;
    full_eskf_origin_pre_position_y_m =
        eskf_data.origin_pre_position_y_m;
    full_eskf_origin_pre_position_z_m =
        eskf_data.origin_pre_position_z_m;

    full_eskf_baro_reference_invalidation_count =
        eskf_data.baro_reference_invalidation_count;

    full_eskf_position_x_m = eskf_data.position_x_m;
    full_eskf_position_y_m = eskf_data.position_y_m;
    full_eskf_position_z_m = eskf_data.position_z_m;
    full_eskf_velocity_x_mps = eskf_data.velocity_x_mps;
    full_eskf_velocity_y_mps = eskf_data.velocity_y_mps;
    full_eskf_velocity_z_mps = eskf_data.velocity_z_mps;

    full_eskf_roll_deg = eskf_data.roll_deg;
    full_eskf_pitch_deg = eskf_data.pitch_deg;
    full_eskf_yaw_deg = eskf_data.yaw_deg;

    full_eskf_accel_bias_x_mps2 = eskf_data.accel_bias_x_mps2;
    full_eskf_accel_bias_y_mps2 = eskf_data.accel_bias_y_mps2;
    full_eskf_accel_bias_z_mps2 = eskf_data.accel_bias_z_mps2;
    full_eskf_gyro_bias_x_dps = eskf_data.gyro_bias_x_dps;
    full_eskf_gyro_bias_y_dps = eskf_data.gyro_bias_y_dps;
    full_eskf_gyro_bias_z_dps = eskf_data.gyro_bias_z_dps;

    full_eskf_world_accel_x_mps2 = eskf_data.world_linear_accel_x_mps2;
    full_eskf_world_accel_y_mps2 = eskf_data.world_linear_accel_y_mps2;
    full_eskf_world_accel_z_mps2 = eskf_data.world_linear_accel_z_mps2;

    full_eskf_baro_innovation_m = eskf_data.baro_innovation_m;
    full_eskf_lidar_innovation_m = eskf_data.lidar_innovation_m;
    full_eskf_gyro_bootstrap_quality_score =
        gyro_bootstrap_quality_score;
    full_eskf_lidar_recovery_active = lidar_recovery_active;
    full_eskf_lidar_reacquire_count = lidar_reacquire_count;

    full_eskf_predict_count = eskf_data.predict_count;
    full_eskf_public_output_count = eskf_data.public_output_count;
    full_eskf_last_public_output_timestamp_us =
        eskf_data.last_public_output_timestamp_us;
    full_eskf_covariance_predict_count = eskf_data.covariance_predict_count;
    full_eskf_gravity_update_count = eskf_data.gravity_update_count;
    full_eskf_gravity_joseph_update_count =
        eskf_data.gravity_joseph_update_count;
    full_eskf_gravity_joseph_fault_count =
        eskf_data.gravity_joseph_fault_count;
    full_eskf_zupt_joseph_update_count =
        eskf_data.zupt_joseph_update_count;
    full_eskf_zupt_joseph_fault_count =
        eskf_data.zupt_joseph_fault_count;
    full_eskf_stationary_detector_count =
        eskf_data.stationary_detector_count;
    full_eskf_stationary_update_count = eskf_data.stationary_update_count;
    full_eskf_baro_update_count = eskf_data.baro_update_count;
    full_eskf_lidar_update_count = eskf_data.lidar_update_count;
    full_eskf_baro_reject_count = eskf_data.baro_reject_count;
    full_eskf_lidar_reject_count = eskf_data.lidar_reject_count;
    full_eskf_reset_count = eskf_data.reset_count;
    full_eskf_numerical_error_count = eskf_data.numerical_error_count;
    full_eskf_last_dt_us = eskf_data.last_dt_us;
    full_eskf_max_dt_us = eskf_data.max_dt_us;
    full_eskf_last_predict_exec_us = eskf_data.last_predict_exec_us;
    full_eskf_max_predict_exec_us = eskf_data.max_predict_exec_us;
    full_eskf_last_correction_exec_us = eskf_data.last_correction_exec_us;
    full_eskf_max_correction_exec_us = eskf_data.max_correction_exec_us;
}

/* -------------------------------------------------------------------------- */

static uint8_t FullESKF_NormalizeQuaternion(void)
{
    float norm_squared =
        (nominal_quaternion[0] * nominal_quaternion[0]) +
        (nominal_quaternion[1] * nominal_quaternion[1]) +
        (nominal_quaternion[2] * nominal_quaternion[2]) +
        (nominal_quaternion[3] * nominal_quaternion[3]);

    if ((FullESKF_IsFiniteReasonable(norm_squared) == 0U) ||
        (norm_squared < 0.000001f) ||
        (norm_squared > 100.0f))
    {
        return 0U;
    }

    float inverse_norm = 1.0f / sqrtf(norm_squared);

    nominal_quaternion[0] *= inverse_norm;
    nominal_quaternion[1] *= inverse_norm;
    nominal_quaternion[2] *= inverse_norm;
    nominal_quaternion[3] *= inverse_norm;

    return 1U;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_QuaternionToRotation(float rotation[3][3])
{
    float qw = nominal_quaternion[0];
    float qx = nominal_quaternion[1];
    float qy = nominal_quaternion[2];
    float qz = nominal_quaternion[3];

    rotation[0][0] = 1.0f - (2.0f * ((qy * qy) + (qz * qz)));
    rotation[0][1] = 2.0f * ((qx * qy) - (qw * qz));
    rotation[0][2] = 2.0f * ((qx * qz) + (qw * qy));

    rotation[1][0] = 2.0f * ((qx * qy) + (qw * qz));
    rotation[1][1] = 1.0f - (2.0f * ((qx * qx) + (qz * qz)));
    rotation[1][2] = 2.0f * ((qy * qz) - (qw * qx));

    rotation[2][0] = 2.0f * ((qx * qz) - (qw * qy));
    rotation[2][1] = 2.0f * ((qy * qz) + (qw * qx));
    rotation[2][2] = 1.0f - (2.0f * ((qx * qx) + (qy * qy)));
}

/* -------------------------------------------------------------------------- */

static void FullESKF_UpdateEulerAngles(void)
{
    float qw = nominal_quaternion[0];
    float qx = nominal_quaternion[1];
    float qy = nominal_quaternion[2];
    float qz = nominal_quaternion[3];

    float sin_roll_cos_pitch = 2.0f * ((qw * qx) + (qy * qz));
    float cos_roll_cos_pitch =
        1.0f - (2.0f * ((qx * qx) + (qy * qy)));

    float sin_pitch = 2.0f * ((qw * qy) - (qz * qx));
    sin_pitch = FullESKF_Clamp(sin_pitch, -1.0f, 1.0f);

    float sin_yaw_cos_pitch = 2.0f * ((qw * qz) + (qx * qy));
    float cos_yaw_cos_pitch =
        1.0f - (2.0f * ((qy * qy) + (qz * qz)));

    eskf_data.roll_deg =
        atan2f(sin_roll_cos_pitch, cos_roll_cos_pitch) *
        FULL_ESKF_RAD_TO_DEG;

    eskf_data.pitch_deg = asinf(sin_pitch) * FULL_ESKF_RAD_TO_DEG;

    eskf_data.yaw_deg =
        atan2f(sin_yaw_cos_pitch, cos_yaw_cos_pitch) *
        FULL_ESKF_RAD_TO_DEG;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_SetInitialCovariance(void)
{
    uint32_t row;
    uint32_t column;

    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        for (column = 0UL; column < FULL_ESKF_STATE_COUNT; column++)
        {
            covariance[row][column] = 0.0f;
        }
    }

    covariance[FULL_ESKF_PX][FULL_ESKF_PX] =
        APP_FULL_ESKF_INITIAL_POSITION_XY_STD_M *
        APP_FULL_ESKF_INITIAL_POSITION_XY_STD_M;
    covariance[FULL_ESKF_PY][FULL_ESKF_PY] =
        covariance[FULL_ESKF_PX][FULL_ESKF_PX];
    covariance[FULL_ESKF_PZ][FULL_ESKF_PZ] =
        APP_FULL_ESKF_INITIAL_POSITION_Z_STD_M *
        APP_FULL_ESKF_INITIAL_POSITION_Z_STD_M;

    covariance[FULL_ESKF_VX][FULL_ESKF_VX] =
        APP_FULL_ESKF_INITIAL_VELOCITY_STD_MPS *
        APP_FULL_ESKF_INITIAL_VELOCITY_STD_MPS;
    covariance[FULL_ESKF_VY][FULL_ESKF_VY] =
        covariance[FULL_ESKF_VX][FULL_ESKF_VX];
    covariance[FULL_ESKF_VZ][FULL_ESKF_VZ] =
        covariance[FULL_ESKF_VX][FULL_ESKF_VX];

    float attitude_variance =
        APP_FULL_ESKF_INITIAL_ATTITUDE_STD_DEG *
        FULL_ESKF_DEG_TO_RAD;
    attitude_variance *= attitude_variance;

    covariance[FULL_ESKF_THETA_X][FULL_ESKF_THETA_X] = attitude_variance;
    covariance[FULL_ESKF_THETA_Y][FULL_ESKF_THETA_Y] = attitude_variance;
    covariance[FULL_ESKF_THETA_Z][FULL_ESKF_THETA_Z] = attitude_variance;

    float accel_bias_variance =
        APP_FULL_ESKF_INITIAL_ACCEL_BIAS_STD_MPS2 *
        APP_FULL_ESKF_INITIAL_ACCEL_BIAS_STD_MPS2;

    covariance[FULL_ESKF_BA_X][FULL_ESKF_BA_X] = accel_bias_variance;
    covariance[FULL_ESKF_BA_Y][FULL_ESKF_BA_Y] = accel_bias_variance;
    covariance[FULL_ESKF_BA_Z][FULL_ESKF_BA_Z] = accel_bias_variance;

    float gyro_bias_std_rad_s =
        APP_FULL_ESKF_INITIAL_GYRO_BIAS_STD_DPS * FULL_ESKF_DEG_TO_RAD;
    float gyro_bias_variance =
        gyro_bias_std_rad_s * gyro_bias_std_rad_s;

    covariance[FULL_ESKF_BG_X][FULL_ESKF_BG_X] = gyro_bias_variance;
    covariance[FULL_ESKF_BG_Y][FULL_ESKF_BG_Y] = gyro_bias_variance;
    covariance[FULL_ESKF_BG_Z][FULL_ESKF_BG_Z] = gyro_bias_variance;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_SaveGoodCovariance(void)
{
    memcpy(covariance_last_good, covariance, sizeof(covariance_last_good));
    covariance_last_good_valid = 1U;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_LatchCovarianceFault(
    uint8_t reason,
    uint8_t stage,
    uint8_t state_index,
    uint8_t other_index,
    float raw_value,
    float aux_value
)
{
    uint8_t replace_detail = 0U;

    if ((reason == FULL_ESKF_RESET_REASON_NONE) ||
        (reason > FULL_ESKF_RESET_REASON_COV_ASYMMETRY))
    {
        return;
    }

    if (covariance_sanitization_fault_pending == 0U)
    {
        replace_detail = 1U;
    }
    else if (reason < covariance_sanitization_fault_reason)
    {
        replace_detail = 1U;
    }

    if (replace_detail != 0U)
    {
        covariance_sanitization_fault_reason = reason;
        eskf_data.covariance_fault_stage = stage;
        eskf_data.covariance_fault_state_index = state_index;
        eskf_data.covariance_fault_other_index = other_index;
        eskf_data.covariance_fault_raw_value = raw_value;
        eskf_data.covariance_fault_aux_value = aux_value;
        eskf_data.covariance_fault_timestamp_us = micros();
        eskf_data.covariance_fault_used_last_good = 0U;
    }

    covariance_sanitization_fault_pending = 1U;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_RecordCovarianceRoundoff(
    uint8_t stage,
    uint8_t state_index,
    float raw_value
)
{
    eskf_data.covariance_roundoff_clamp_count++;
    eskf_data.covariance_roundoff_last_stage = stage;
    eskf_data.covariance_roundoff_last_state_index = state_index;
    eskf_data.covariance_roundoff_last_raw_value = raw_value;
    eskf_data.covariance_roundoff_last_timestamp_us = micros();
}

/* -------------------------------------------------------------------------- */

static FULL_ESKF_COV_TIMING_OPT void FullESKF_SymmetrizeAndClampCovariance(uint8_t stage)
{
    uint32_t row;
    uint32_t column;

    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        for (column = row + 1UL;
             column < FULL_ESKF_STATE_COUNT;
             column++)
        {
            float a = covariance[row][column];
            float b = covariance[column][row];

            if ((FullESKF_IsFiniteReasonableCovariance(row, column, a) == 0U) ||
                (FullESKF_IsFiniteReasonableCovariance(column, row, b) == 0U))
            {
                FullESKF_LatchCovarianceFault(
                    FULL_ESKF_RESET_REASON_COV_NONFINITE, stage,
                    (uint8_t)row, (uint8_t)column, a, b);
                covariance[row][column] = 0.0f;
                covariance[column][row] = 0.0f;
                continue;
            }
            else
            {
                float difference = FullESKF_Abs(a - b);
                float magnitude = FullESKF_Abs(a);
                float tolerance;

                if (FullESKF_Abs(b) > magnitude)
                {
                    magnitude = FullESKF_Abs(b);
                }

                tolerance = APP_FULL_ESKF_COV_SYMMETRY_ABS_TOL +
                    (APP_FULL_ESKF_COV_SYMMETRY_REL_TOL * magnitude);

                if (difference > tolerance)
                {
                    FullESKF_LatchCovarianceFault(
                        FULL_ESKF_RESET_REASON_COV_ASYMMETRY, stage,
                        (uint8_t)row, (uint8_t)column, difference, tolerance);
                }
            }

            {
                float symmetric = 0.5f * (a + b);
                covariance[row][column] = symmetric;
                covariance[column][row] = symmetric;
            }
        }

        if (FullESKF_IsFiniteReasonableCovariance(row, row, covariance[row][row]) == 0U)
        {
            FullESKF_LatchCovarianceFault(
                FULL_ESKF_RESET_REASON_COV_NONFINITE, stage,
                (uint8_t)row, FULL_ESKF_COV_STATE_INVALID,
                covariance[row][row], 0.0f);
            covariance[row][row] = FULL_ESKF_MIN_VARIANCE;
        }
        else if (covariance[row][row] <= 0.0f)
        {
            float raw_diagonal = covariance[row][row];

            if (raw_diagonal >= -APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL)
            {
                FullESKF_RecordCovarianceRoundoff(
                    stage, (uint8_t)row, raw_diagonal);
            }
            else
            {
                FullESKF_LatchCovarianceFault(
                    FULL_ESKF_RESET_REASON_COV_DIAGONAL, stage,
                    (uint8_t)row, FULL_ESKF_COV_STATE_INVALID,
                    raw_diagonal, APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL);
            }

            covariance[row][row] = FULL_ESKF_MIN_VARIANCE;
        }
        else if (covariance[row][row] < FULL_ESKF_MIN_VARIANCE)
        {
            covariance[row][row] = FULL_ESKF_MIN_VARIANCE;
        }
    }
}

/* -------------------------------------------------------------------------- */

static void FullESKF_ClampCovarianceDiagonal(uint8_t stage)
{
    uint32_t index;

    for (index = 0UL; index < FULL_ESKF_STATE_COUNT; index++)
    {
        if (FullESKF_IsFiniteReasonableCovariance(index, index, covariance[index][index]) == 0U)
        {
            FullESKF_LatchCovarianceFault(
                FULL_ESKF_RESET_REASON_COV_NONFINITE, stage,
                (uint8_t)index, FULL_ESKF_COV_STATE_INVALID,
                covariance[index][index], 0.0f);
            covariance[index][index] = FULL_ESKF_MIN_VARIANCE;
        }
        else if (covariance[index][index] <= 0.0f)
        {
            float raw_diagonal = covariance[index][index];

            if (raw_diagonal >= -APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL)
            {
                FullESKF_RecordCovarianceRoundoff(
                    stage, (uint8_t)index, raw_diagonal);
            }
            else
            {
                FullESKF_LatchCovarianceFault(
                    FULL_ESKF_RESET_REASON_COV_DIAGONAL, stage,
                    (uint8_t)index, FULL_ESKF_COV_STATE_INVALID,
                    raw_diagonal, APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL);
            }

            covariance[index][index] = FULL_ESKF_MIN_VARIANCE;
        }
        else if (covariance[index][index] < FULL_ESKF_MIN_VARIANCE)
        {
            covariance[index][index] = FULL_ESKF_MIN_VARIANCE;
        }
    }
}

/* -------------------------------------------------------------------------- */

static void FullESKF_UpdatePublicState(void)
{
    if ((eskf_data.horizontal_position_valid == 0U) &&
        (origin_zero_applied != 0U) &&
        (xy_diag_initialized != 0U))
    {
        /* Public/logged X/Y are bounded diagnostics only.  The internal ESKF
         * nominal X/Y states remain untouched and horizontal_valid stays 0. */
        eskf_data.position_x_m = xy_diag_position_m[0];
        eskf_data.position_y_m = xy_diag_position_m[1];
        eskf_data.velocity_x_mps = xy_diag_velocity_mps[0];
        eskf_data.velocity_y_mps = xy_diag_velocity_mps[1];
    }
    else
    {
        eskf_data.position_x_m = nominal_position[0];
        eskf_data.position_y_m = nominal_position[1];
        eskf_data.velocity_x_mps = nominal_velocity[0];
        eskf_data.velocity_y_mps = nominal_velocity[1];
    }

    eskf_data.position_z_m = nominal_position[2];
    eskf_data.velocity_z_mps = nominal_velocity[2];

    eskf_data.q_w = nominal_quaternion[0];
    eskf_data.q_x = nominal_quaternion[1];
    eskf_data.q_y = nominal_quaternion[2];
    eskf_data.q_z = nominal_quaternion[3];

    eskf_data.accel_bias_x_mps2 = nominal_accel_bias[0];
    eskf_data.accel_bias_y_mps2 = nominal_accel_bias[1];
    eskf_data.accel_bias_z_mps2 = nominal_accel_bias[2];

    eskf_data.gyro_bias_x_rad_s = nominal_gyro_bias[0];
    eskf_data.gyro_bias_y_rad_s = nominal_gyro_bias[1];
    eskf_data.gyro_bias_z_rad_s = nominal_gyro_bias[2];

    eskf_data.gyro_bias_x_dps =
        nominal_gyro_bias[0] * FULL_ESKF_RAD_TO_DEG;
    eskf_data.gyro_bias_y_dps =
        nominal_gyro_bias[1] * FULL_ESKF_RAD_TO_DEG;
    eskf_data.gyro_bias_z_dps =
        nominal_gyro_bias[2] * FULL_ESKF_RAD_TO_DEG;

    eskf_data.covariance_position_x = covariance[FULL_ESKF_PX][FULL_ESKF_PX];
    eskf_data.covariance_position_y = covariance[FULL_ESKF_PY][FULL_ESKF_PY];
    eskf_data.covariance_position_z = covariance[FULL_ESKF_PZ][FULL_ESKF_PZ];

    eskf_data.covariance_velocity_x = covariance[FULL_ESKF_VX][FULL_ESKF_VX];
    eskf_data.covariance_velocity_y = covariance[FULL_ESKF_VY][FULL_ESKF_VY];
    eskf_data.covariance_velocity_z = covariance[FULL_ESKF_VZ][FULL_ESKF_VZ];

    eskf_data.covariance_roll_rad2 =
        covariance[FULL_ESKF_THETA_X][FULL_ESKF_THETA_X];
    eskf_data.covariance_pitch_rad2 =
        covariance[FULL_ESKF_THETA_Y][FULL_ESKF_THETA_Y];
    eskf_data.covariance_yaw_rad2 =
        covariance[FULL_ESKF_THETA_Z][FULL_ESKF_THETA_Z];

    eskf_data.covariance_accel_bias_x = covariance[FULL_ESKF_BA_X][FULL_ESKF_BA_X];
    eskf_data.covariance_accel_bias_y = covariance[FULL_ESKF_BA_Y][FULL_ESKF_BA_Y];
    eskf_data.covariance_accel_bias_z = covariance[FULL_ESKF_BA_Z][FULL_ESKF_BA_Z];

    eskf_data.covariance_gyro_bias_x = covariance[FULL_ESKF_BG_X][FULL_ESKF_BG_X];
    eskf_data.covariance_gyro_bias_y = covariance[FULL_ESKF_BG_Y][FULL_ESKF_BG_Y];
    eskf_data.covariance_gyro_bias_z = covariance[FULL_ESKF_BG_Z][FULL_ESKF_BG_Z];
}

/* -------------------------------------------------------------------------- */

static uint8_t FullESKF_CheckNumericalHealth(void)
{
    uint32_t i;

    for (i = 0UL; i < 3UL; i++)
    {
        if ((FullESKF_IsFiniteReasonable(nominal_position[i]) == 0U) ||
            (FullESKF_IsFiniteReasonable(nominal_velocity[i]) == 0U) ||
            (FullESKF_IsFiniteReasonable(nominal_accel_bias[i]) == 0U) ||
            (FullESKF_IsFiniteReasonable(nominal_gyro_bias[i]) == 0U))
        {
            return 0U;
        }
    }

    for (i = 0UL; i < 4UL; i++)
    {
        if (FullESKF_IsFiniteReasonable(nominal_quaternion[i]) == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

/* -------------------------------------------------------------------------- */

static uint8_t FullESKF_CheckCovarianceDiagonalFast(void)
{
    uint8_t reason = FULL_ESKF_RESET_REASON_NONE;
    uint32_t row;

    if (covariance_sanitization_fault_pending != 0U)
    {
        return covariance_sanitization_fault_reason;
    }

    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        float diagonal = covariance[row][row];

        if (FullESKF_IsFiniteReasonableCovariance(row, row, diagonal) == 0U)
        {
            FullESKF_LatchCovarianceFault(
                FULL_ESKF_RESET_REASON_COV_NONFINITE,
                FULL_ESKF_COV_STAGE_INTEGRITY, (uint8_t)row,
                FULL_ESKF_COV_STATE_INVALID, diagonal, 0.0f);
            return FULL_ESKF_RESET_REASON_COV_NONFINITE;
        }

        if ((diagonal <= -APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL) ||
            ((FullESKF_CovarianceTouchesUnobservedHorizontalPosition(
                row, row) == 0U) &&
             (diagonal > APP_FULL_ESKF_MAX_COVARIANCE)))
        {
            FullESKF_LatchCovarianceFault(
                FULL_ESKF_RESET_REASON_COV_DIAGONAL,
                FULL_ESKF_COV_STAGE_INTEGRITY, (uint8_t)row,
                FULL_ESKF_COV_STATE_INVALID, diagonal,
                APP_FULL_ESKF_MAX_COVARIANCE);
            reason = FULL_ESKF_RESET_REASON_COV_DIAGONAL;
            break;
        }
    }

    return reason;
}

static void FullESKF_ResetCovarianceIntegritySlice(void)
{
    covariance_integrity_slice_row = 0U;
    covariance_integrity_slice_reason = FULL_ESKF_RESET_REASON_NONE;
    covariance_integrity_slice_diag_min = APP_FULL_ESKF_MAX_COVARIANCE;
    covariance_integrity_slice_diag_max = 0.0f;
    covariance_integrity_slice_symmetry_max = 0.0f;
    covariance_integrity_slice_diag_seen = 0U;
}

static uint8_t FullESKF_ServiceCovarianceIntegritySlice(void)
{
    uint32_t rows_done = 0UL;

    if (covariance_integrity_slice_row >= FULL_ESKF_STATE_COUNT)
    {
        FullESKF_ResetCovarianceIntegritySlice();
    }

    while ((rows_done < APP_FULL_ESKF_INTEGRITY_ROWS_PER_CORRECTION) &&
           (covariance_integrity_slice_row < FULL_ESKF_STATE_COUNT))
    {
        uint32_t row = covariance_integrity_slice_row;
        uint32_t column;
        float diagonal = covariance[row][row];

        if (FullESKF_IsFiniteReasonableCovariance(row, row, diagonal) == 0U)
        {
            FullESKF_LatchCovarianceFault(
                FULL_ESKF_RESET_REASON_COV_NONFINITE,
                FULL_ESKF_COV_STAGE_INTEGRITY, (uint8_t)row,
                FULL_ESKF_COV_STATE_INVALID, diagonal, 0.0f);
            covariance_integrity_slice_reason =
                FULL_ESKF_RESET_REASON_COV_NONFINITE;
        }
        else
        {
            if (FullESKF_CovarianceTouchesUnobservedHorizontalPosition(
                    row, row) == 0U)
            {
                if (covariance_integrity_slice_diag_seen == 0U)
                {
                    covariance_integrity_slice_diag_min = diagonal;
                    covariance_integrity_slice_diag_max = diagonal;
                    covariance_integrity_slice_diag_seen = 1U;
                }
                else
                {
                    if (diagonal < covariance_integrity_slice_diag_min)
                    {
                        covariance_integrity_slice_diag_min = diagonal;
                    }
                    if (diagonal > covariance_integrity_slice_diag_max)
                    {
                        covariance_integrity_slice_diag_max = diagonal;
                    }
                }
            }

            if ((diagonal <= -APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL) ||
                ((FullESKF_CovarianceTouchesUnobservedHorizontalPosition(
                    row, row) == 0U) &&
                 (diagonal > APP_FULL_ESKF_MAX_COVARIANCE)))
            {
                FullESKF_LatchCovarianceFault(
                    FULL_ESKF_RESET_REASON_COV_DIAGONAL,
                    FULL_ESKF_COV_STAGE_INTEGRITY, (uint8_t)row,
                    FULL_ESKF_COV_STATE_INVALID, diagonal,
                    APP_FULL_ESKF_MAX_COVARIANCE);
                if ((covariance_integrity_slice_reason == FULL_ESKF_RESET_REASON_NONE) ||
                    (FULL_ESKF_RESET_REASON_COV_DIAGONAL < covariance_integrity_slice_reason))
                {
                    covariance_integrity_slice_reason =
                        FULL_ESKF_RESET_REASON_COV_DIAGONAL;
                }
            }
        }

        for (column = row + 1UL;
             column < FULL_ESKF_STATE_COUNT;
             column++)
        {
            float a = covariance[row][column];
            float b = covariance[column][row];

            if ((FullESKF_IsFiniteReasonableCovariance(row, column, a) == 0U) ||
                (FullESKF_IsFiniteReasonableCovariance(column, row, b) == 0U))
            {
                FullESKF_LatchCovarianceFault(
                    FULL_ESKF_RESET_REASON_COV_NONFINITE,
                    FULL_ESKF_COV_STAGE_INTEGRITY, (uint8_t)row,
                    (uint8_t)column, a, b);
                covariance_integrity_slice_reason =
                    FULL_ESKF_RESET_REASON_COV_NONFINITE;
                continue;
            }

            {
                float difference = FullESKF_Abs(a - b);
                float magnitude = FullESKF_Abs(a);
                float tolerance;

                if (FullESKF_Abs(b) > magnitude)
                {
                    magnitude = FullESKF_Abs(b);
                }
                if (difference > covariance_integrity_slice_symmetry_max)
                {
                    covariance_integrity_slice_symmetry_max = difference;
                }

                tolerance = APP_FULL_ESKF_COV_SYMMETRY_ABS_TOL +
                    (APP_FULL_ESKF_COV_SYMMETRY_REL_TOL * magnitude);

                if (difference > tolerance)
                {
                    FullESKF_LatchCovarianceFault(
                        FULL_ESKF_RESET_REASON_COV_ASYMMETRY,
                        FULL_ESKF_COV_STAGE_INTEGRITY, (uint8_t)row,
                        (uint8_t)column, difference, tolerance);
                    if ((covariance_integrity_slice_reason == FULL_ESKF_RESET_REASON_NONE) ||
                        (FULL_ESKF_RESET_REASON_COV_ASYMMETRY < covariance_integrity_slice_reason))
                    {
                        covariance_integrity_slice_reason =
                            FULL_ESKF_RESET_REASON_COV_ASYMMETRY;
                    }
                }
            }
        }

        covariance_integrity_slice_row++;
        rows_done++;
    }

    if (covariance_sanitization_fault_pending != 0U)
    {
        covariance_integrity_slice_reason =
            covariance_sanitization_fault_reason;
    }

    if (covariance_integrity_slice_reason != FULL_ESKF_RESET_REASON_NONE)
    {
        uint8_t reason = covariance_integrity_slice_reason;
        eskf_data.covariance_integrity_ok = 0U;
        eskf_data.covariance_fault_count++;
        FullESKF_ResetCovarianceIntegritySlice();
        return reason;
    }

    if (covariance_integrity_slice_row >= FULL_ESKF_STATE_COUNT)
    {
        if (covariance_integrity_slice_diag_seen == 0U)
        {
            covariance_integrity_slice_diag_min = 0.0f;
            covariance_integrity_slice_diag_max = 0.0f;
        }

        eskf_data.covariance_diag_min = covariance_integrity_slice_diag_min;
        eskf_data.covariance_diag_max = covariance_integrity_slice_diag_max;
        eskf_data.covariance_symmetry_error_max =
            covariance_integrity_slice_symmetry_max;
        eskf_data.covariance_integrity_check_count++;
        eskf_data.covariance_integrity_ok = 1U;
        covariance_sanitization_fault_pending = 0U;
        covariance_sanitization_fault_reason = FULL_ESKF_RESET_REASON_NONE;
        FullESKF_SaveGoodCovariance();
        FullESKF_ResetCovarianceIntegritySlice();
    }

    return FULL_ESKF_RESET_REASON_NONE;
}

static uint8_t FullESKF_CheckCovarianceIntegrity(void)
{
    uint8_t reason = FULL_ESKF_RESET_REASON_NONE;
    float diag_min = APP_FULL_ESKF_MAX_COVARIANCE;
    float diag_max = 0.0f;
    float symmetry_max = 0.0f;
    uint8_t diag_seen = 0U;
    uint32_t row;
    uint32_t column;

    if (covariance_sanitization_fault_pending != 0U)
    {
        reason = covariance_sanitization_fault_reason;
    }

    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        float diagonal = covariance[row][row];

        if (FullESKF_IsFiniteReasonableCovariance(row, row, diagonal) == 0U)
        {
            FullESKF_LatchCovarianceFault(
                FULL_ESKF_RESET_REASON_COV_NONFINITE,
                FULL_ESKF_COV_STAGE_INTEGRITY, (uint8_t)row,
                FULL_ESKF_COV_STATE_INVALID, diagonal, 0.0f);
            if ((reason == FULL_ESKF_RESET_REASON_NONE) ||
                (FULL_ESKF_RESET_REASON_COV_NONFINITE < reason))
            {
                reason = FULL_ESKF_RESET_REASON_COV_NONFINITE;
            }
        }
        else
        {
            if (FullESKF_CovarianceTouchesUnobservedHorizontalPosition(
                    row, row) == 0U)
            {
                if (diag_seen == 0U)
                {
                    diag_min = diagonal;
                    diag_max = diagonal;
                    diag_seen = 1U;
                }
                else
                {
                    if (diagonal < diag_min)
                    {
                        diag_min = diagonal;
                    }
                    if (diagonal > diag_max)
                    {
                        diag_max = diagonal;
                    }
                }
            }

            if ((diagonal <= -APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL) ||
                ((FullESKF_CovarianceTouchesUnobservedHorizontalPosition(
                    row, row) == 0U) &&
                 (diagonal > APP_FULL_ESKF_MAX_COVARIANCE)))
            {
                FullESKF_LatchCovarianceFault(
                    FULL_ESKF_RESET_REASON_COV_DIAGONAL,
                    FULL_ESKF_COV_STAGE_INTEGRITY, (uint8_t)row,
                    FULL_ESKF_COV_STATE_INVALID, diagonal,
                    APP_FULL_ESKF_MAX_COVARIANCE);
                if ((reason == FULL_ESKF_RESET_REASON_NONE) ||
                    (FULL_ESKF_RESET_REASON_COV_DIAGONAL < reason))
                {
                    reason = FULL_ESKF_RESET_REASON_COV_DIAGONAL;
                }
            }
        }

        for (column = row + 1UL;
             column < FULL_ESKF_STATE_COUNT;
             column++)
        {
            float a = covariance[row][column];
            float b = covariance[column][row];

            if ((FullESKF_IsFiniteReasonableCovariance(row, column, a) == 0U) ||
                (FullESKF_IsFiniteReasonableCovariance(column, row, b) == 0U))
            {
                FullESKF_LatchCovarianceFault(
                    FULL_ESKF_RESET_REASON_COV_NONFINITE,
                    FULL_ESKF_COV_STAGE_INTEGRITY, (uint8_t)row,
                    (uint8_t)column, a, b);
                if ((reason == FULL_ESKF_RESET_REASON_NONE) ||
                    (FULL_ESKF_RESET_REASON_COV_NONFINITE < reason))
                {
                    reason = FULL_ESKF_RESET_REASON_COV_NONFINITE;
                }
                continue;
            }

            {
                float difference = FullESKF_Abs(a - b);
                float magnitude = FullESKF_Abs(a);
                float tolerance;

                if (FullESKF_Abs(b) > magnitude)
                {
                    magnitude = FullESKF_Abs(b);
                }
                if (difference > symmetry_max)
                {
                    symmetry_max = difference;
                }

                tolerance = APP_FULL_ESKF_COV_SYMMETRY_ABS_TOL +
                    (APP_FULL_ESKF_COV_SYMMETRY_REL_TOL * magnitude);

                if (difference > tolerance)
                {
                    FullESKF_LatchCovarianceFault(
                        FULL_ESKF_RESET_REASON_COV_ASYMMETRY,
                        FULL_ESKF_COV_STAGE_INTEGRITY, (uint8_t)row,
                        (uint8_t)column, difference, tolerance);
                    if ((reason == FULL_ESKF_RESET_REASON_NONE) ||
                        (FULL_ESKF_RESET_REASON_COV_ASYMMETRY < reason))
                    {
                        reason = FULL_ESKF_RESET_REASON_COV_ASYMMETRY;
                    }
                }
            }
        }
    }

    if (diag_seen == 0U)
    {
        diag_min = 0.0f;
        diag_max = 0.0f;
    }

    eskf_data.covariance_diag_min = diag_min;
    eskf_data.covariance_diag_max = diag_max;
    eskf_data.covariance_symmetry_error_max = symmetry_max;
    eskf_data.covariance_integrity_check_count++;

    if (covariance_sanitization_fault_pending != 0U)
    {
        reason = covariance_sanitization_fault_reason;
    }

    if (reason == FULL_ESKF_RESET_REASON_NONE)
    {
        eskf_data.covariance_integrity_ok = 1U;
        covariance_sanitization_fault_pending = 0U;
        covariance_sanitization_fault_reason = FULL_ESKF_RESET_REASON_NONE;
        FullESKF_SaveGoodCovariance();
    }
    else
    {
        eskf_data.covariance_integrity_ok = 0U;
        eskf_data.covariance_fault_count++;
    }

    return reason;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_RecoverCovarianceOnly(uint8_t reason)
{
    if ((reason < FULL_ESKF_RESET_REASON_COV_NONFINITE) ||
        (reason > FULL_ESKF_RESET_REASON_COV_ASYMMETRY))
    {
        return;
    }

    if (covariance_last_good_valid != 0U)
    {
        memcpy(covariance, covariance_last_good, sizeof(covariance));
        eskf_data.covariance_fault_used_last_good = 1U;
    }
    else
    {
        FullESKF_SetInitialCovariance();
        FullESKF_SaveGoodCovariance();
        eskf_data.covariance_fault_used_last_good = 0U;
    }

    FullESKF_ClearCovarianceAccumulator();
    covariance_sanitization_fault_pending = 0U;
    covariance_sanitization_fault_reason = FULL_ESKF_RESET_REASON_NONE;

    /* A gravity batch linearized against the pre-rollback covariance must not
     * continue after a covariance recovery. Restart gravity aiding cleanly. */
    gravity_update_decimation_counter = 0UL;
    gravity_update_phase = FULL_ESKF_GRAVITY_PHASE_IDLE;
    gravity_batch_accepted_count = 0U;
    eskf_data.gravity_correction_active = 0U;

    eskf_data.numerical_error_count++;
    eskf_data.covariance_reinit_count++;
    eskf_data.covariance_state_preserving_recovery_count++;
    eskf_data.covariance_integrity_ok = 1U;
    /* Covariance-only rollback leaves the nominal state usable.  Do not
     * strand the correction loop in the unhealthy/reacquire path. */
    eskf_data.healthy = 1U;
    eskf_data.reset_reason = reason;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_SafeReinitialize(uint8_t reason)
{
    uint8_t enabled = eskf_data.enabled;
    uint8_t shadow_mode = eskf_data.shadow_mode;
    uint32_t reset_count = eskf_data.reset_count;
    uint32_t public_output_count = eskf_data.public_output_count;
    uint32_t last_public_output_timestamp_us =
        eskf_data.last_public_output_timestamp_us;
    uint32_t numerical_error_count = eskf_data.numerical_error_count;
    uint32_t public_output_reject_count = eskf_data.public_output_reject_count;
    uint32_t public_z_jump_reject_count = eskf_data.public_z_jump_reject_count;
    uint32_t public_vz_jump_reject_count = eskf_data.public_vz_jump_reject_count;
    uint32_t covariance_integrity_check_count =
        eskf_data.covariance_integrity_check_count;
    uint32_t covariance_fault_count = eskf_data.covariance_fault_count;
    uint32_t covariance_reinit_count = eskf_data.covariance_reinit_count;
    float covariance_diag_min = eskf_data.covariance_diag_min;
    float covariance_diag_max = eskf_data.covariance_diag_max;
    float covariance_symmetry_error_max =
        eskf_data.covariance_symmetry_error_max;
    uint8_t covariance_fault_stage = eskf_data.covariance_fault_stage;
    uint8_t covariance_fault_state_index = eskf_data.covariance_fault_state_index;
    uint8_t covariance_fault_other_index = eskf_data.covariance_fault_other_index;
    uint8_t covariance_fault_used_last_good = eskf_data.covariance_fault_used_last_good;
    float covariance_fault_raw_value = eskf_data.covariance_fault_raw_value;
    float covariance_fault_aux_value = eskf_data.covariance_fault_aux_value;
    uint32_t covariance_fault_timestamp_us = eskf_data.covariance_fault_timestamp_us;
    uint8_t covariance_roundoff_last_stage = eskf_data.covariance_roundoff_last_stage;
    uint8_t covariance_roundoff_last_state_index =
        eskf_data.covariance_roundoff_last_state_index;
    float covariance_roundoff_last_raw_value =
        eskf_data.covariance_roundoff_last_raw_value;
    uint32_t covariance_roundoff_last_timestamp_us =
        eskf_data.covariance_roundoff_last_timestamp_us;
    uint32_t covariance_state_preserving_recovery_count =
        eskf_data.covariance_state_preserving_recovery_count;
    uint32_t covariance_roundoff_clamp_count =
        eskf_data.covariance_roundoff_clamp_count;
    uint32_t gravity_joseph_update_count =
        eskf_data.gravity_joseph_update_count;
    uint32_t gravity_joseph_fault_count =
        eskf_data.gravity_joseph_fault_count;
    uint32_t zupt_joseph_update_count =
        eskf_data.zupt_joseph_update_count;
    uint32_t zupt_joseph_fault_count =
        eskf_data.zupt_joseph_fault_count;

    FullStateESKF_Init();

    eskf_data.enabled = enabled;
    eskf_data.shadow_mode = shadow_mode;
    /* reset_count is incremented when attitude-backed reacquisition completes.
     * Keep the public generation/timestamp monotonic across the internal reset:
     * the RCS consumer must not interpret the zeroed Init() state as a new sample. */
    eskf_data.reset_count = reset_count;
    eskf_data.public_output_count = public_output_count;
    eskf_data.last_public_output_timestamp_us =
        last_public_output_timestamp_us;
    eskf_data.numerical_error_count = numerical_error_count;
    eskf_data.public_output_reject_count = public_output_reject_count;
    eskf_data.public_z_jump_reject_count = public_z_jump_reject_count;
    eskf_data.public_vz_jump_reject_count = public_vz_jump_reject_count;
    eskf_data.covariance_integrity_check_count =
        covariance_integrity_check_count;
    eskf_data.covariance_fault_count = covariance_fault_count;
    eskf_data.covariance_reinit_count = covariance_reinit_count;
    eskf_data.covariance_diag_min = covariance_diag_min;
    eskf_data.covariance_diag_max = covariance_diag_max;
    eskf_data.covariance_symmetry_error_max =
        covariance_symmetry_error_max;
    eskf_data.covariance_fault_stage = covariance_fault_stage;
    eskf_data.covariance_fault_state_index = covariance_fault_state_index;
    eskf_data.covariance_fault_other_index = covariance_fault_other_index;
    eskf_data.covariance_fault_used_last_good = covariance_fault_used_last_good;
    eskf_data.covariance_fault_raw_value = covariance_fault_raw_value;
    eskf_data.covariance_fault_aux_value = covariance_fault_aux_value;
    eskf_data.covariance_fault_timestamp_us = covariance_fault_timestamp_us;
    eskf_data.covariance_roundoff_last_stage = covariance_roundoff_last_stage;
    eskf_data.covariance_roundoff_last_state_index =
        covariance_roundoff_last_state_index;
    eskf_data.covariance_roundoff_last_raw_value =
        covariance_roundoff_last_raw_value;
    eskf_data.covariance_roundoff_last_timestamp_us =
        covariance_roundoff_last_timestamp_us;
    eskf_data.covariance_state_preserving_recovery_count =
        covariance_state_preserving_recovery_count;
    eskf_data.covariance_roundoff_clamp_count = covariance_roundoff_clamp_count;
    eskf_data.gravity_joseph_update_count = gravity_joseph_update_count;
    eskf_data.gravity_joseph_fault_count = gravity_joseph_fault_count;
    eskf_data.zupt_joseph_update_count = zupt_joseph_update_count;
    eskf_data.zupt_joseph_fault_count = zupt_joseph_fault_count;
    eskf_data.reset_reason = reason;
    eskf_data.covariance_integrity_ok = 0U;

    FullESKF_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */

static uint8_t FullESKF_CheckPublicVerticalGuard(uint32_t now_us)
{
    uint32_t dt_us;
    float dt_s;
    float speed_mps;
    float allowed_z_jump_m;
    float allowed_vz_jump_mps;
    float z_jump_m;
    float vz_jump_mps;
    uint8_t z_bad;
    uint8_t vz_bad;

    if ((eskf_data.origin_zeroed == 0U) ||
        (eskf_data.output_inhibited != 0U) ||
        (eskf_data.vertical_reacquire_active != 0U))
    {
        public_vertical_guard_armed = 0U;
        return FULL_ESKF_RESET_REASON_NONE;
    }

    if (public_vertical_guard_armed == 0U)
    {
        return FULL_ESKF_RESET_REASON_NONE;
    }

    dt_us = (uint32_t)(now_us - public_vertical_guard_timestamp_us);
    if ((dt_us == 0UL) ||
        (dt_us > APP_FULL_ESKF_PUBLIC_GUARD_MAX_GAP_US))
    {
        public_vertical_guard_armed = 0U;
        return FULL_ESKF_RESET_REASON_NONE;
    }

    dt_s = (float)dt_us * 1.0e-6f;
    speed_mps = FullESKF_Abs(public_vertical_guard_vz_mps);
    if (FullESKF_Abs(nominal_velocity[2]) > speed_mps)
    {
        speed_mps = FullESKF_Abs(nominal_velocity[2]);
    }

    allowed_z_jump_m = APP_FULL_ESKF_PUBLIC_Z_JUMP_BASE_M +
        (APP_FULL_ESKF_PUBLIC_Z_SPEED_FACTOR * speed_mps * dt_s);
    allowed_vz_jump_mps = APP_FULL_ESKF_PUBLIC_VZ_JUMP_BASE_MPS +
        (APP_FULL_ESKF_PUBLIC_VZ_ACCEL_MARGIN_MPS2 * dt_s);

    z_jump_m = FullESKF_Abs(
        nominal_position[2] - public_vertical_guard_z_m);
    vz_jump_mps = FullESKF_Abs(
        nominal_velocity[2] - public_vertical_guard_vz_mps);

    z_bad = (z_jump_m > allowed_z_jump_m) ? 1U : 0U;
    vz_bad = (vz_jump_mps > allowed_vz_jump_mps) ? 1U : 0U;

    if ((z_bad != 0U) && (vz_bad != 0U))
    {
        return FULL_ESKF_RESET_REASON_PUBLIC_Z_VZ_JUMP;
    }
    if (z_bad != 0U)
    {
        return FULL_ESKF_RESET_REASON_PUBLIC_Z_JUMP;
    }
    if (vz_bad != 0U)
    {
        return FULL_ESKF_RESET_REASON_PUBLIC_VZ_JUMP;
    }

    return FULL_ESKF_RESET_REASON_NONE;
}

static void FullESKF_CommitPublicVerticalGuard(uint32_t now_us)
{
    if ((eskf_data.origin_zeroed == 0U) ||
        (eskf_data.output_inhibited != 0U) ||
        (eskf_data.vertical_reacquire_active != 0U))
    {
        public_vertical_guard_armed = 0U;
        return;
    }

    public_vertical_guard_z_m = nominal_position[2];
    public_vertical_guard_vz_mps = nominal_velocity[2];
    public_vertical_guard_timestamp_us = now_us;
    public_vertical_guard_armed = 1U;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_ClearCovarianceAccumulator(void)
{
    covariance_sample_counter = 0UL;
    covariance_dt_accum_s = 0.0f;

    covariance_accel_integral[0] = 0.0f;
    covariance_accel_integral[1] = 0.0f;
    covariance_accel_integral[2] = 0.0f;

    covariance_gyro_integral[0] = 0.0f;
    covariance_gyro_integral[1] = 0.0f;
    covariance_gyro_integral[2] = 0.0f;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_InitializeFromAttitude(
    const AttitudeEstimatorData_t *attitude,
    uint32_t timestamp_us
)
{
    memset(nominal_position, 0, sizeof(nominal_position));
    memset(nominal_velocity, 0, sizeof(nominal_velocity));
    memset(nominal_accel_bias, 0, sizeof(nominal_accel_bias));
    memset(nominal_gyro_bias, 0, sizeof(nominal_gyro_bias));

    nominal_quaternion[0] = attitude->q_w;
    nominal_quaternion[1] = attitude->q_x;
    nominal_quaternion[2] = attitude->q_y;
    nominal_quaternion[3] = attitude->q_z;

    if (FullESKF_NormalizeQuaternion() == 0U)
    {
        nominal_quaternion[0] = 1.0f;
        nominal_quaternion[1] = 0.0f;
        nominal_quaternion[2] = 0.0f;
        nominal_quaternion[3] = 0.0f;
    }

    FullESKF_SetInitialCovariance();
    FullESKF_SaveGoodCovariance();
    FullESKF_ClearCovarianceAccumulator();

    eskf_data.initialized = 1U;
    eskf_data.healthy = 1U;
    eskf_data.covariance_integrity_ok = 1U;
    eskf_data.last_predict_timestamp_us = timestamp_us;
    eskf_data.last_dt_us = 0UL;
    eskf_data.reset_count++;

    FullESKF_UpdateEulerAngles();
    FullESKF_UpdatePublicState();
}

/* -------------------------------------------------------------------------- */

static void FullESKF_InflateAfterGap(void)
{
    uint32_t i;

    for (i = 0UL; i < 3UL; i++)
    {
        covariance[FULL_ESKF_VX + i][FULL_ESKF_VX + i] += 1.0f;
        covariance[FULL_ESKF_THETA_X + i][FULL_ESKF_THETA_X + i] +=
            0.01f;
    }

    FullESKF_SymmetrizeAndClampCovariance(FULL_ESKF_COV_STAGE_GAP_INFLATE);
}

/* -------------------------------------------------------------------------- */

static FULL_ESKF_COV_TIMING_OPT void FullESKF_PropagateCovariance(
    float dt_s,
    const float specific_force_body_mps2[3],
    const float angular_rate_body_rad_s[3]
)
{
    uint32_t profile_start_cycles = FullESKF_DWTStart();
    float rotation[3][3];
    float skew_force[3][3];
    float skew_gyro[3][3];
    float velocity_attitude[3][3];
    uint32_t row;
    uint32_t column;
    uint32_t index;

    FullESKF_QuaternionToRotation(rotation);

    skew_force[0][0] = 0.0f;
    skew_force[0][1] = -specific_force_body_mps2[2];
    skew_force[0][2] = specific_force_body_mps2[1];
    skew_force[1][0] = specific_force_body_mps2[2];
    skew_force[1][1] = 0.0f;
    skew_force[1][2] = -specific_force_body_mps2[0];
    skew_force[2][0] = -specific_force_body_mps2[1];
    skew_force[2][1] = specific_force_body_mps2[0];
    skew_force[2][2] = 0.0f;

    skew_gyro[0][0] = 0.0f;
    skew_gyro[0][1] = -angular_rate_body_rad_s[2];
    skew_gyro[0][2] = angular_rate_body_rad_s[1];
    skew_gyro[1][0] = angular_rate_body_rad_s[2];
    skew_gyro[1][1] = 0.0f;
    skew_gyro[1][2] = -angular_rate_body_rad_s[0];
    skew_gyro[2][0] = -angular_rate_body_rad_s[1];
    skew_gyro[2][1] = angular_rate_body_rad_s[0];
    skew_gyro[2][2] = 0.0f;

    for (row = 0UL; row < 3UL; row++)
    {
        for (column = 0UL; column < 3UL; column++)
        {
            velocity_attitude[row][column] = 0.0f;

            for (index = 0UL; index < 3UL; index++)
            {
                velocity_attitude[row][column] -=
                    rotation[row][index] * skew_force[index][column];
            }
        }
    }

#if (APP_FULL_ESKF_SPARSE_COVARIANCE_ENABLED != 0U)
    /*
     * The discrete transition has a fixed sparse block structure:
     *
     * [ I  dtI  0   0   0 ]
     * [ 0   I   A   B   0 ]
     * [ 0   0   C   0  -dtI]
     * [ 0   0   0   I   0 ]
     * [ 0   0   0   0   I ]
     *
     * This path evaluates exactly the same non-zero terms, in the same
     * accumulation order as the legacy transition*P*transition^T loops.
     * Known-zero multiplications, branch checks and the 15x15 transition
     * initialization are removed.
     */
    float velocity_attitude_dt[3][3];
    float accel_bias_transition[3][3];
    float attitude_transition[3][3];

    for (row = 0UL; row < 3UL; row++)
    {
        for (column = 0UL; column < 3UL; column++)
        {
            velocity_attitude_dt[row][column] =
                velocity_attitude[row][column] * dt_s;

            accel_bias_transition[row][column] =
                -rotation[row][column] * dt_s;

            attitude_transition[row][column] =
                (row == column) ? 1.0f : 0.0f;

            attitude_transition[row][column] +=
                -skew_gyro[row][column] * dt_s;
        }
    }

    /* covariance_temp = transition * covariance */
    for (row = 0UL; row < 3UL; row++)
    {
        for (column = 0UL;
             column < FULL_ESKF_STATE_COUNT;
             column++)
        {
            float sum;

            covariance_temp[FULL_ESKF_PX + row][column] =
                covariance[FULL_ESKF_PX + row][column] +
                (dt_s * covariance[FULL_ESKF_VX + row][column]);

            sum = covariance[FULL_ESKF_VX + row][column];

            sum += velocity_attitude_dt[row][0] *
                   covariance[FULL_ESKF_THETA_X + 0U][column];
            sum += velocity_attitude_dt[row][1] *
                   covariance[FULL_ESKF_THETA_X + 1U][column];
            sum += velocity_attitude_dt[row][2] *
                   covariance[FULL_ESKF_THETA_X + 2U][column];

            sum += accel_bias_transition[row][0] *
                   covariance[FULL_ESKF_BA_X + 0U][column];
            sum += accel_bias_transition[row][1] *
                   covariance[FULL_ESKF_BA_X + 1U][column];
            sum += accel_bias_transition[row][2] *
                   covariance[FULL_ESKF_BA_X + 2U][column];

            covariance_temp[FULL_ESKF_VX + row][column] = sum;

            sum = 0.0f;

            sum += attitude_transition[row][0] *
                   covariance[FULL_ESKF_THETA_X + 0U][column];
            sum += attitude_transition[row][1] *
                   covariance[FULL_ESKF_THETA_X + 1U][column];
            sum += attitude_transition[row][2] *
                   covariance[FULL_ESKF_THETA_X + 2U][column];
            sum += (-dt_s) *
                   covariance[FULL_ESKF_BG_X + row][column];

            covariance_temp[FULL_ESKF_THETA_X + row][column] = sum;

            covariance_temp[FULL_ESKF_BA_X + row][column] =
                covariance[FULL_ESKF_BA_X + row][column];

            covariance_temp[FULL_ESKF_BG_X + row][column] =
                covariance[FULL_ESKF_BG_X + row][column];
        }
    }

    /* covariance = covariance_temp * transition^T */
    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        for (column = 0UL; column < 3UL; column++)
        {
            float sum;

            covariance[row][FULL_ESKF_PX + column] =
                covariance_temp[row][FULL_ESKF_PX + column] +
                (dt_s * covariance_temp[row][FULL_ESKF_VX + column]);

            sum = covariance_temp[row][FULL_ESKF_VX + column];

            sum += covariance_temp[row][FULL_ESKF_THETA_X + 0U] *
                   velocity_attitude_dt[column][0];
            sum += covariance_temp[row][FULL_ESKF_THETA_X + 1U] *
                   velocity_attitude_dt[column][1];
            sum += covariance_temp[row][FULL_ESKF_THETA_X + 2U] *
                   velocity_attitude_dt[column][2];

            sum += covariance_temp[row][FULL_ESKF_BA_X + 0U] *
                   accel_bias_transition[column][0];
            sum += covariance_temp[row][FULL_ESKF_BA_X + 1U] *
                   accel_bias_transition[column][1];
            sum += covariance_temp[row][FULL_ESKF_BA_X + 2U] *
                   accel_bias_transition[column][2];

            covariance[row][FULL_ESKF_VX + column] = sum;

            sum = 0.0f;

            sum += covariance_temp[row][FULL_ESKF_THETA_X + 0U] *
                   attitude_transition[column][0];
            sum += covariance_temp[row][FULL_ESKF_THETA_X + 1U] *
                   attitude_transition[column][1];
            sum += covariance_temp[row][FULL_ESKF_THETA_X + 2U] *
                   attitude_transition[column][2];
            sum += covariance_temp[row][FULL_ESKF_BG_X + column] *
                   (-dt_s);

            covariance[row][FULL_ESKF_THETA_X + column] = sum;

            covariance[row][FULL_ESKF_BA_X + column] =
                covariance_temp[row][FULL_ESKF_BA_X + column];

            covariance[row][FULL_ESKF_BG_X + column] =
                covariance_temp[row][FULL_ESKF_BG_X + column];
        }
    }
#else
    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        for (column = 0UL; column < FULL_ESKF_STATE_COUNT; column++)
        {
            transition[row][column] = (row == column) ? 1.0f : 0.0f;
        }
    }

    for (row = 0UL; row < 3UL; row++)
    {
        transition[FULL_ESKF_PX + row][FULL_ESKF_VX + row] = dt_s;

        for (column = 0UL; column < 3UL; column++)
        {
            transition[FULL_ESKF_VX + row][FULL_ESKF_THETA_X + column] =
                velocity_attitude[row][column] * dt_s;

            transition[FULL_ESKF_VX + row][FULL_ESKF_BA_X + column] =
                -rotation[row][column] * dt_s;

            transition[FULL_ESKF_THETA_X + row]
                      [FULL_ESKF_THETA_X + column] +=
                -skew_gyro[row][column] * dt_s;
        }

        transition[FULL_ESKF_THETA_X + row][FULL_ESKF_BG_X + row] = -dt_s;
    }

    /* Legacy reference path. */
    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        for (column = 0UL; column < FULL_ESKF_STATE_COUNT; column++)
        {
            float sum = 0.0f;

            for (index = 0UL; index < FULL_ESKF_STATE_COUNT; index++)
            {
                float coefficient = transition[row][index];

                if (coefficient != 0.0f)
                {
                    sum += coefficient * covariance[index][column];
                }
            }

            covariance_temp[row][column] = sum;
        }
    }

    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        for (column = 0UL; column < FULL_ESKF_STATE_COUNT; column++)
        {
            float sum = 0.0f;

            for (index = 0UL; index < FULL_ESKF_STATE_COUNT; index++)
            {
                float coefficient = transition[column][index];

                if (coefficient != 0.0f)
                {
                    sum += covariance_temp[row][index] * coefficient;
                }
            }

            covariance[row][column] = sum;
        }
    }
#endif

    float accel_noise_variance =
        APP_FULL_ESKF_ACCEL_NOISE_STD_MPS2 *
        APP_FULL_ESKF_ACCEL_NOISE_STD_MPS2;

    float gyro_noise_std_rad_s =
        APP_FULL_ESKF_GYRO_NOISE_STD_DPS * FULL_ESKF_DEG_TO_RAD;
    float gyro_noise_variance =
        gyro_noise_std_rad_s * gyro_noise_std_rad_s;

    float accel_bias_rw_variance =
        APP_FULL_ESKF_ACCEL_BIAS_RW_STD_MPS3 *
        APP_FULL_ESKF_ACCEL_BIAS_RW_STD_MPS3;

    float gyro_bias_rw_std_rad_s2 =
        APP_FULL_ESKF_GYRO_BIAS_RW_STD_DPS2 * FULL_ESKF_DEG_TO_RAD;
    float gyro_bias_rw_variance =
        gyro_bias_rw_std_rad_s2 * gyro_bias_rw_std_rad_s2;

    float dt2 = dt_s * dt_s;
    float dt3 = dt2 * dt_s;

    for (index = 0UL; index < 3UL; index++)
    {
        covariance[FULL_ESKF_PX + index][FULL_ESKF_PX + index] +=
            0.25f * accel_noise_variance * dt3;

        covariance[FULL_ESKF_VX + index][FULL_ESKF_VX + index] +=
            accel_noise_variance * dt_s;

        covariance[FULL_ESKF_THETA_X + index]
                  [FULL_ESKF_THETA_X + index] +=
            gyro_noise_variance * dt_s;

        covariance[FULL_ESKF_BA_X + index][FULL_ESKF_BA_X + index] +=
            accel_bias_rw_variance * dt_s;

        covariance[FULL_ESKF_BG_X + index][FULL_ESKF_BG_X + index] +=
            gyro_bias_rw_variance * dt_s;
    }

    FullESKF_SymmetrizeAndClampCovariance(FULL_ESKF_COV_STAGE_PROPAGATE);
    eskf_data.covariance_predict_count++;

    FullESKF_ProfileStore(
        &full_eskf_last_covariance_cycles,
        &full_eskf_max_covariance_cycles,
        &full_eskf_total_covariance_cycles,
        FullESKF_DWTElapsed(profile_start_cycles)
    );
}

/* -------------------------------------------------------------------------- */

static void FullESKF_InjectError(const float error_state[FULL_ESKF_STATE_COUNT])
{
    uint32_t i;

    for (i = 0UL; i < 3UL; i++)
    {
        nominal_position[i] += error_state[FULL_ESKF_PX + i];
        nominal_velocity[i] += error_state[FULL_ESKF_VX + i];
        nominal_accel_bias[i] += error_state[FULL_ESKF_BA_X + i];
        nominal_gyro_bias[i] += error_state[FULL_ESKF_BG_X + i];

        nominal_accel_bias[i] = FullESKF_Clamp(
            nominal_accel_bias[i],
            -APP_FULL_ESKF_ACCEL_BIAS_ABS_MAX_MPS2,
            APP_FULL_ESKF_ACCEL_BIAS_ABS_MAX_MPS2
        );

        nominal_gyro_bias[i] = FullESKF_Clamp(
            nominal_gyro_bias[i],
            -APP_FULL_ESKF_GYRO_BIAS_ABS_MAX_DPS * FULL_ESKF_DEG_TO_RAD,
            APP_FULL_ESKF_GYRO_BIAS_ABS_MAX_DPS * FULL_ESKF_DEG_TO_RAD
        );
    }

    {
        float accel_bias_norm_squared =
            (nominal_accel_bias[0] * nominal_accel_bias[0]) +
            (nominal_accel_bias[1] * nominal_accel_bias[1]) +
            (nominal_accel_bias[2] * nominal_accel_bias[2]);
        float accel_bias_limit = APP_FULL_ESKF_ACCEL_BIAS_ABS_MAX_MPS2;

        if (accel_bias_norm_squared >
            (accel_bias_limit * accel_bias_limit))
        {
            float scale = accel_bias_limit / sqrtf(accel_bias_norm_squared);

            nominal_accel_bias[0] *= scale;
            nominal_accel_bias[1] *= scale;
            nominal_accel_bias[2] *= scale;
        }
    }

    float delta_theta_x = error_state[FULL_ESKF_THETA_X];
    float delta_theta_y = error_state[FULL_ESKF_THETA_Y];
    float delta_theta_z = error_state[FULL_ESKF_THETA_Z];

    float delta_norm = sqrtf(
        (delta_theta_x * delta_theta_x) +
        (delta_theta_y * delta_theta_y) +
        (delta_theta_z * delta_theta_z)
    );

    if (delta_norm > APP_FULL_ESKF_MAX_ATTITUDE_INJECTION_RAD)
    {
        float scale = APP_FULL_ESKF_MAX_ATTITUDE_INJECTION_RAD / delta_norm;
        delta_theta_x *= scale;
        delta_theta_y *= scale;
        delta_theta_z *= scale;
    }

    float qw = nominal_quaternion[0];
    float qx = nominal_quaternion[1];
    float qy = nominal_quaternion[2];
    float qz = nominal_quaternion[3];

    float half_x = 0.5f * delta_theta_x;
    float half_y = 0.5f * delta_theta_y;
    float half_z = 0.5f * delta_theta_z;

    nominal_quaternion[0] =
        qw - (qx * half_x) - (qy * half_y) - (qz * half_z);
    nominal_quaternion[1] =
        qx + (qw * half_x) + (qy * half_z) - (qz * half_y);
    nominal_quaternion[2] =
        qy + (qw * half_y) - (qx * half_z) + (qz * half_x);
    nominal_quaternion[3] =
        qz + (qw * half_z) + (qx * half_y) - (qy * half_x);

    if (FullESKF_NormalizeQuaternion() == 0U)
    {
        eskf_data.numerical_error_count++;
        eskf_data.healthy = 0U;
    }
}

/* -------------------------------------------------------------------------- */

#if (APP_FULL_ESKF_SPARSE_SCALAR_UPDATES_ENABLED == 0U)
static uint8_t FullESKF_ScalarUpdate(
    const float measurement_jacobian[FULL_ESKF_STATE_COUNT],
    float innovation,
    float measurement_variance,
    float gate_sigma,
    float absolute_gate,
    uint8_t covariance_stage
)
{
    float covariance_times_h[FULL_ESKF_STATE_COUNT];
    float error_state[FULL_ESKF_STATE_COUNT];
    float innovation_variance = measurement_variance;
    float inverse_innovation_variance;
    uint32_t row;
    uint32_t column;

    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        float sum = 0.0f;

        for (column = 0UL; column < FULL_ESKF_STATE_COUNT; column++)
        {
            float coefficient = measurement_jacobian[column];

            if (coefficient != 0.0f)
            {
                sum += covariance[row][column] * coefficient;
            }
        }

        covariance_times_h[row] = sum;
        innovation_variance += measurement_jacobian[row] * sum;
    }

    if ((!FullESKF_IsFiniteReasonable(innovation_variance)) ||
        (innovation_variance <= FULL_ESKF_MIN_VARIANCE))
    {
        eskf_data.numerical_error_count++;
        return 0U;
    }

    if ((FullESKF_Abs(innovation) > absolute_gate) &&
        ((innovation * innovation) >
         ((gate_sigma * gate_sigma) * innovation_variance)))
    {
        return 0U;
    }

    inverse_innovation_variance = 1.0f / innovation_variance;

    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        error_state[row] =
            covariance_times_h[row] * inverse_innovation_variance * innovation;
    }

    /*
     * P_new = P - (P H^T)(P H^T)^T / S is symmetric. Updating only the
     * upper triangle nearly halves the scalar-correction matrix work.
     */
    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        for (column = row; column < FULL_ESKF_STATE_COUNT; column++)
        {
            float updated = covariance[row][column] -
                (covariance_times_h[row] * covariance_times_h[column] *
                 inverse_innovation_variance);

            covariance[row][column] = updated;
            covariance[column][row] = updated;
        }
    }

    FullESKF_ClampCovarianceDiagonal(covariance_stage);

    if (covariance_sanitization_fault_pending != 0U)
    {
        return 0U;
    }

    FullESKF_InjectError(error_state);
    return 1U;
}
#endif

/* -------------------------------------------------------------------------- */


#if (APP_FULL_ESKF_SPARSE_SCALAR_UPDATES_ENABLED != 0U)
/*
 * P49 masked Joseph ZUPT.
 *
 * H is a unit velocity observation.  The full ESKF cross-covariance benefit is
 * retained for position, velocity, attitude and accel-bias states.  Gyro bias
 * has a dedicated stationary correction immediately after ZUPT, so allowing
 * ZUPT's velocity residual to inject BG through cross-covariance is both
 * unnecessary and, in P48, numerically harmful (BGX/BGY negative diagonal).
 *
 * K[BGX..BGZ] is therefore forced to zero and the exact Joseph equation
 *
 *   P+ = P - K p_k' - p_k K' + K (Pkk + R) K'
 *
 * is evaluated in double precision for the 15x15 upper triangle.
 */
static uint8_t FullESKF_ZUPTMaskedJosephUnitState(
    uint32_t state_index,
    float innovation,
    float measurement_variance,
    float gate_sigma,
    float absolute_gate
)
{
    /* P50: algebraically exact fast form of the P49 masked Joseph update.
     *
     * P49 evaluated the full Joseph equation in double precision.  On the
     * Cortex-M4F, double precision is software-emulated and the 3-axis ZUPT
     * deterministically pushed the 1 kHz IMU task over one release.  The P49
     * bench log proved the relationship exactly:
     *
     *     4020 accepted ZUPT scalar updates / 3 = 1340 IMU deadline misses.
     *
     * For a unit-state measurement H=e_k and K_i=P_ik/S, Joseph simplifies to
     *
     *     P+_ij = P_ij - P_ik P_jk / S
     *
     * for every row/column whose gain is retained.  Because P49 intentionally
     * masks BGX/BGY/BGZ gains to zero, the exact masked-Joseph result is:
     *   - update all rows 0..BAZ against all columns, including BG cross terms;
     *   - leave the BGxBG 3x3 block unchanged.
     *
     * This implementation computes that exact result in hardware float, keeps
     * the same BG ownership rule, then runs the existing covariance clamp /
     * integrity guard.  No estimator semantics are changed; only the expensive
     * software-double form is removed from the 200 Hz correction path.
     */
    float prior_column[FULL_ESKF_STATE_COUNT];
    float scaled_column[FULL_ESKF_BG_X];
    float error_state[FULL_ESKF_STATE_COUNT] = {0.0f};
    float innovation_variance;
    float inverse_innovation_variance;
    uint32_t row;
    uint32_t column;

    if (state_index >= FULL_ESKF_STATE_COUNT)
    {
        return 0U;
    }

    innovation_variance =
        covariance[state_index][state_index] + measurement_variance;

    if ((!FullESKF_IsFiniteReasonable(innovation_variance)) ||
        (innovation_variance <= FULL_ESKF_MIN_VARIANCE))
    {
        eskf_data.numerical_error_count++;
        eskf_data.zupt_joseph_fault_count++;
        return 0U;
    }

    if ((FullESKF_Abs(innovation) > absolute_gate) &&
        ((innovation * innovation) >
         ((gate_sigma * gate_sigma) * innovation_variance)))
    {
        return 0U;
    }

    inverse_innovation_variance = 1.0f / innovation_variance;

    /* Snapshot the complete prior measurement column before modifying P. */
    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        prior_column[row] = covariance[row][state_index];
    }

    /* Retained gains are states PX..BAZ.  BGX..BGZ are deliberately zero. */
    for (row = 0UL; row < FULL_ESKF_BG_X; row++)
    {
        scaled_column[row] =
            prior_column[row] * inverse_innovation_variance;
        error_state[row] = scaled_column[row] * innovation;
    }

    /* Exact masked Joseph rank-one update.  Because only upper-triangle rows
     * 0..BAZ are visited, the BGxBG block is untouched by construction. */
    for (row = 0UL; row < FULL_ESKF_BG_X; row++)
    {
        float scaled = scaled_column[row];

        for (column = row; column < FULL_ESKF_STATE_COUNT; column++)
        {
            float updated = covariance[row][column] -
                (scaled * prior_column[column]);

            covariance[row][column] = updated;
            covariance[column][row] = updated;
        }
    }

    FullESKF_ClampCovarianceDiagonal(FULL_ESKF_COV_STAGE_ZUPT);

    if (covariance_sanitization_fault_pending != 0U)
    {
        eskf_data.zupt_joseph_fault_count++;
        return 0U;
    }

    FullESKF_InjectError(error_state);
    eskf_data.zupt_joseph_update_count++;
    return 1U;
}

/* -------------------------------------------------------------------------- */

static uint8_t FullESKF_ScalarUpdateUnitState(

    uint32_t state_index,
    float innovation,
    float measurement_variance,
    float gate_sigma,
    float absolute_gate,
    uint8_t covariance_stage
)
{
    float covariance_times_h[FULL_ESKF_STATE_COUNT];
    float error_state[FULL_ESKF_STATE_COUNT];
    float innovation_variance;
    float inverse_innovation_variance;
    uint32_t row;
    uint32_t column;

    if (state_index >= FULL_ESKF_STATE_COUNT)
    {
        return 0U;
    }

#if (APP_FULL_ESKF_ZUPT_MASKED_JOSEPH_ENABLED != 0U)
    if (covariance_stage == FULL_ESKF_COV_STAGE_ZUPT)
    {
        return FullESKF_ZUPTMaskedJosephUnitState(
            state_index,
            innovation,
            measurement_variance,
            gate_sigma,
            absolute_gate);
    }
#endif

    innovation_variance =
        covariance[state_index][state_index] + measurement_variance;

    if ((!FullESKF_IsFiniteReasonable(innovation_variance)) ||
        (innovation_variance <= FULL_ESKF_MIN_VARIANCE))
    {
        eskf_data.numerical_error_count++;
        return 0U;
    }

    if ((FullESKF_Abs(innovation) > absolute_gate) &&
        ((innovation * innovation) >
         ((gate_sigma * gate_sigma) * innovation_variance)))
    {
        return 0U;
    }

    inverse_innovation_variance = 1.0f / innovation_variance;

    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        covariance_times_h[row] = covariance[row][state_index];
        error_state[row] =
            covariance_times_h[row] * inverse_innovation_variance * innovation;
    }

    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        for (column = row; column < FULL_ESKF_STATE_COUNT; column++)
        {
            float updated = covariance[row][column] -
                (covariance_times_h[row] * covariance_times_h[column] *
                 inverse_innovation_variance);

            covariance[row][column] = updated;
            covariance[column][row] = updated;
        }
    }

    FullESKF_ClampCovarianceDiagonal(covariance_stage);

    if (covariance_sanitization_fault_pending != 0U)
    {
        return 0U;
    }

    FullESKF_InjectError(error_state);
    return 1U;
}

/* -------------------------------------------------------------------------- */

/* P48: gravity correction uses a gain restricted to the two attitude-error
 * states present in the gravity Jacobian.  P47 showed that the full 15-state
 * Kalman gain could drive BG_Y variance negative during the startup transient.
 * Gravity is an attitude-direction observation; gyro bias already has its own
 * stationary bias correction path, so gravity must not directly inject bias,
 * position, or velocity states.
 *
 * The covariance update is the exact Joseph form for a sparse K with only two
 * non-zero rows.  The untouched state-state block is preserved, the two
 * attitude cross rows are updated in O(N), and the 2x2 attitude block is
 * evaluated in double precision.  This keeps P positive-semidefinite without
 * the O(N^2) full-matrix gravity Joseph cost used in P47. */
static uint8_t FullESKF_GravityAttitudeOnlySparseJoseph(
    uint32_t first_state_index,
    float first_coefficient,
    uint32_t second_state_index,
    float second_coefficient,
    float innovation,
    float measurement_variance,
    float gate_sigma,
    float absolute_gate
)
{
    float covariance_times_h[FULL_ESKF_STATE_COUNT];
    float error_state[FULL_ESKF_STATE_COUNT] = {0.0f};
    float innovation_variance = measurement_variance;
    float inverse_innovation_variance;
    float gain_first;
    float gain_second;
    uint32_t index;

    if ((first_state_index < FULL_ESKF_THETA_X) ||
        (first_state_index > FULL_ESKF_THETA_Z) ||
        (second_state_index < FULL_ESKF_THETA_X) ||
        (second_state_index > FULL_ESKF_THETA_Z) ||
        (first_state_index == second_state_index))
    {
        return 0U;
    }

    /* Snapshot P*H' before any covariance element is changed. */
    for (index = 0UL; index < FULL_ESKF_STATE_COUNT; index++)
    {
        covariance_times_h[index] =
            (covariance[index][first_state_index] * first_coefficient) +
            (covariance[index][second_state_index] * second_coefficient);
    }

    innovation_variance +=
        (first_coefficient * covariance_times_h[first_state_index]) +
        (second_coefficient * covariance_times_h[second_state_index]);

    if ((!FullESKF_IsFiniteReasonable(innovation_variance)) ||
        (innovation_variance <= FULL_ESKF_MIN_VARIANCE))
    {
        eskf_data.numerical_error_count++;
        return 0U;
    }

    if ((FullESKF_Abs(innovation) > absolute_gate) &&
        ((innovation * innovation) >
         ((gate_sigma * gate_sigma) * innovation_variance)))
    {
        return 0U;
    }

    inverse_innovation_variance = 1.0f / innovation_variance;
    gain_first = covariance_times_h[first_state_index] *
                 inverse_innovation_variance;
    gain_second = covariance_times_h[second_state_index] *
                  inverse_innovation_variance;

    error_state[first_state_index] = gain_first * innovation;
    error_state[second_state_index] = gain_second * innovation;

    /* Sparse Joseph cross rows.  For every state j whose gain is zero:
     * P+_a,j = P_a,j - K_a (H P)_j and likewise for b. */
    for (index = 0UL; index < FULL_ESKF_STATE_COUNT; index++)
    {
        if ((index != first_state_index) &&
            (index != second_state_index))
        {
            float updated_first = covariance[first_state_index][index] -
                (gain_first * covariance_times_h[index]);
            float updated_second = covariance[second_state_index][index] -
                (gain_second * covariance_times_h[index]);

            covariance[first_state_index][index] = updated_first;
            covariance[index][first_state_index] = updated_first;
            covariance[second_state_index][index] = updated_second;
            covariance[index][second_state_index] = updated_second;
        }
    }

    /* Exact 2x2 Joseph attitude block, evaluated in double precision to avoid
     * the cancellation that was visible in the P46/P47 startup transient. */
    {
        double p11 = (double)covariance[first_state_index][first_state_index];
        double p12 = (double)covariance[first_state_index][second_state_index];
        double p22 = (double)covariance[second_state_index][second_state_index];
        double k1 = (double)gain_first;
        double k2 = (double)gain_second;
        double h1 = (double)first_coefficient;
        double h2 = (double)second_coefficient;
        double r = (double)measurement_variance;
        double a11 = 1.0 - (k1 * h1);
        double a12 = -(k1 * h2);
        double a21 = -(k2 * h1);
        double a22 = 1.0 - (k2 * h2);
        double m11 = (a11 * p11) + (a12 * p12);
        double m12 = (a11 * p12) + (a12 * p22);
        double m21 = (a21 * p11) + (a22 * p12);
        double m22 = (a21 * p12) + (a22 * p22);
        float updated_11 = (float)(
            (m11 * a11) + (m12 * a12) + (k1 * r * k1));
        float updated_12 = (float)(
            (m11 * a21) + (m12 * a22) + (k1 * r * k2));
        float updated_22 = (float)(
            (m21 * a21) + (m22 * a22) + (k2 * r * k2));

        covariance[first_state_index][first_state_index] = updated_11;
        covariance[first_state_index][second_state_index] = updated_12;
        covariance[second_state_index][first_state_index] = updated_12;
        covariance[second_state_index][second_state_index] = updated_22;
    }

    FullESKF_ClampCovarianceDiagonal(FULL_ESKF_COV_STAGE_GRAVITY);

    if (covariance_sanitization_fault_pending != 0U)
    {
        eskf_data.gravity_joseph_fault_count++;
        return 0U;
    }

    /* Only attitude-error entries are non-zero. */
    FullESKF_InjectError(error_state);
    eskf_data.gravity_joseph_update_count++;
    return 1U;
}

/* -------------------------------------------------------------------------- */

static uint8_t FullESKF_ScalarUpdateTwoStates(
    uint32_t first_state_index,
    float first_coefficient,
    uint32_t second_state_index,
    float second_coefficient,
    float innovation,
    float measurement_variance,
    float gate_sigma,
    float absolute_gate,
    uint8_t covariance_stage
)
{
    float covariance_times_h[FULL_ESKF_STATE_COUNT];
    float error_state[FULL_ESKF_STATE_COUNT];
    float innovation_variance = measurement_variance;
    float inverse_innovation_variance;
    uint32_t row;
    uint32_t column;

    if ((first_state_index >= FULL_ESKF_STATE_COUNT) ||
        (second_state_index >= FULL_ESKF_STATE_COUNT) ||
        (first_state_index == second_state_index))
    {
        return 0U;
    }

#if (APP_FULL_ESKF_GRAVITY_ATTITUDE_ONLY_SPARSE_JOSEPH != 0U)
    if (covariance_stage == FULL_ESKF_COV_STAGE_GRAVITY)
    {
        return FullESKF_GravityAttitudeOnlySparseJoseph(
            first_state_index,
            first_coefficient,
            second_state_index,
            second_coefficient,
            innovation,
            measurement_variance,
            gate_sigma,
            absolute_gate);
    }
#endif

    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        float sum = 0.0f;

        if (first_coefficient != 0.0f)
        {
            sum += covariance[row][first_state_index] * first_coefficient;
        }

        if (second_coefficient != 0.0f)
        {
            sum += covariance[row][second_state_index] * second_coefficient;
        }

        covariance_times_h[row] = sum;
    }

    if (first_coefficient != 0.0f)
    {
        innovation_variance +=
            first_coefficient * covariance_times_h[first_state_index];
    }

    if (second_coefficient != 0.0f)
    {
        innovation_variance +=
            second_coefficient * covariance_times_h[second_state_index];
    }

    if ((!FullESKF_IsFiniteReasonable(innovation_variance)) ||
        (innovation_variance <= FULL_ESKF_MIN_VARIANCE))
    {
        eskf_data.numerical_error_count++;
        return 0U;
    }

    if ((FullESKF_Abs(innovation) > absolute_gate) &&
        ((innovation * innovation) >
         ((gate_sigma * gate_sigma) * innovation_variance)))
    {
        return 0U;
    }

    inverse_innovation_variance = 1.0f / innovation_variance;

    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        error_state[row] =
            covariance_times_h[row] * inverse_innovation_variance * innovation;
    }

    for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)
    {
        for (column = row; column < FULL_ESKF_STATE_COUNT; column++)
        {
            float updated = covariance[row][column] -
                (covariance_times_h[row] * covariance_times_h[column] *
                 inverse_innovation_variance);

            covariance[row][column] = updated;
            covariance[column][row] = updated;
        }
    }

    FullESKF_ClampCovarianceDiagonal(covariance_stage);

    if (covariance_sanitization_fault_pending != 0U)
    {
        return 0U;
    }

    FullESKF_InjectError(error_state);
    return 1U;
}
#endif

/* -------------------------------------------------------------------------- */

static uint8_t FullESKF_ConstrainedSingleStateUpdate(
    uint32_t state_index,
    float innovation,
    float measurement_variance,
    float gate_sigma,
    float absolute_gate,
    float maximum_state_step,
    uint8_t covariance_stage
)
{
    float prior_variance;
    float innovation_variance;
    float kalman_gain;
    float correction;
    float effective_gain;
    float scale;
    float error_state[FULL_ESKF_STATE_COUNT] = {0.0f};
    uint32_t index;

    if (state_index >= FULL_ESKF_STATE_COUNT)
    {
        return 0U;
    }

    prior_variance = covariance[state_index][state_index];
    innovation_variance = prior_variance + measurement_variance;

    if ((!FullESKF_IsFiniteReasonable(innovation_variance)) ||
        (innovation_variance <= FULL_ESKF_MIN_VARIANCE))
    {
        eskf_data.numerical_error_count++;
        return 0U;
    }

    if ((FullESKF_Abs(innovation) > absolute_gate) &&
        ((innovation * innovation) >
         ((gate_sigma * gate_sigma) * innovation_variance)))
    {
        return 0U;
    }

    kalman_gain = prior_variance / innovation_variance;
    correction = kalman_gain * innovation;

    if (maximum_state_step > 0.0f)
    {
        correction = FullESKF_Clamp(
            correction,
            -maximum_state_step,
            maximum_state_step
        );
    }

    if (FullESKF_Abs(innovation) > 1.0e-9f)
    {
        effective_gain = correction / innovation;
    }
    else
    {
        effective_gain = 0.0f;
    }

    effective_gain = FullESKF_Clamp(effective_gain, 0.0f, 1.0f);
    scale = 1.0f - effective_gain;

    for (index = 0UL; index < FULL_ESKF_STATE_COUNT; index++)
    {
        if (index != state_index)
        {
            float updated_cross = covariance[state_index][index] * scale;
            covariance[state_index][index] = updated_cross;
            covariance[index][state_index] = updated_cross;
        }
    }

    covariance[state_index][state_index] =
        (scale * scale * prior_variance) +
        (effective_gain * effective_gain * measurement_variance);

    FullESKF_ClampCovarianceDiagonal(covariance_stage);

    if (covariance_sanitization_fault_pending != 0U)
    {
        return 0U;
    }

    error_state[state_index] = correction;
    FullESKF_InjectError(error_state);
    return 1U;
}

/* -------------------------------------------------------------------------- */

static uint8_t FullESKF_ConstrainedAltitudeUpdate(
    float innovation,
    float measurement_variance,
    float gate_sigma,
    float absolute_gate,
    float maximum_position_step,
    float maximum_velocity_step,
    uint8_t covariance_stage
)
{
    float innovation_variance =
        covariance[FULL_ESKF_PZ][FULL_ESKF_PZ] + measurement_variance;
    float position_gain;
    float velocity_gain;
    float position_correction;
    float velocity_correction;
    float prior_pz_row[FULL_ESKF_STATE_COUNT];
    float prior_vz_row[FULL_ESKF_STATE_COUNT];
    float error_state[FULL_ESKF_STATE_COUNT] = {0.0f};
    uint32_t index;

    if ((!FullESKF_IsFiniteReasonable(innovation_variance)) ||
        (innovation_variance <= FULL_ESKF_MIN_VARIANCE))
    {
        eskf_data.numerical_error_count++;
        return 0U;
    }

    /* LIDAR is safety-gated with either limit, not the permissive AND rule. */
    if ((FullESKF_Abs(innovation) > absolute_gate) ||
        ((innovation * innovation) >
         ((gate_sigma * gate_sigma) * innovation_variance)))
    {
        return 0U;
    }

    position_gain =
        covariance[FULL_ESKF_PZ][FULL_ESKF_PZ] /
        innovation_variance;
    velocity_gain =
        covariance[FULL_ESKF_VZ][FULL_ESKF_PZ] /
        innovation_variance;

    position_correction = position_gain * innovation;
    velocity_correction = velocity_gain * innovation;

    position_correction = FullESKF_Clamp(
        position_correction,
        -maximum_position_step,
        maximum_position_step
    );

    velocity_correction = FullESKF_Clamp(
        velocity_correction,
        -maximum_velocity_step,
        maximum_velocity_step
    );

    if (FullESKF_Abs(innovation) > 1.0e-9f)
    {
        position_gain = position_correction / innovation;
        velocity_gain = velocity_correction / innovation;
    }
    else
    {
        position_gain = 0.0f;
        velocity_gain = 0.0f;
    }

    for (index = 0UL; index < FULL_ESKF_STATE_COUNT; index++)
    {
        prior_pz_row[index] = covariance[FULL_ESKF_PZ][index];
        prior_vz_row[index] = covariance[FULL_ESKF_VZ][index];
    }

    /*
     * Joseph-form update with a gain restricted to PZ and VZ. All unrelated
     * X/Y, attitude and bias rows stay untouched, so one LIDAR sample cannot
     * kick the horizontal state. Only the two affected rows/columns are
     * evaluated, keeping the 200 Hz path light.
     */
    for (index = 0UL; index < FULL_ESKF_STATE_COUNT; index++)
    {
        if ((index == FULL_ESKF_PZ) || (index == FULL_ESKF_VZ))
        {
            continue;
        }

        covariance[FULL_ESKF_PZ][index] =
            (1.0f - position_gain) * prior_pz_row[index];
        covariance[index][FULL_ESKF_PZ] =
            covariance[FULL_ESKF_PZ][index];

        covariance[FULL_ESKF_VZ][index] =
            prior_vz_row[index] -
            (velocity_gain * prior_pz_row[index]);
        covariance[index][FULL_ESKF_VZ] =
            covariance[FULL_ESKF_VZ][index];
    }

    covariance[FULL_ESKF_PZ][FULL_ESKF_PZ] =
        prior_pz_row[FULL_ESKF_PZ] -
        (2.0f * position_gain * prior_pz_row[FULL_ESKF_PZ]) +
        (position_gain * position_gain * innovation_variance);

    covariance[FULL_ESKF_PZ][FULL_ESKF_VZ] =
        prior_pz_row[FULL_ESKF_VZ] -
        (position_gain * prior_pz_row[FULL_ESKF_VZ]) -
        (prior_pz_row[FULL_ESKF_PZ] * velocity_gain) +
        (position_gain * innovation_variance * velocity_gain);

    covariance[FULL_ESKF_VZ][FULL_ESKF_PZ] =
        covariance[FULL_ESKF_PZ][FULL_ESKF_VZ];

    covariance[FULL_ESKF_VZ][FULL_ESKF_VZ] =
        prior_vz_row[FULL_ESKF_VZ] -
        (2.0f * velocity_gain * prior_pz_row[FULL_ESKF_VZ]) +
        (velocity_gain * velocity_gain * innovation_variance);

    FullESKF_ClampCovarianceDiagonal(covariance_stage);

    if (covariance_sanitization_fault_pending != 0U)
    {
        return 0U;
    }

    error_state[FULL_ESKF_PZ] = position_correction;
    error_state[FULL_ESKF_VZ] = velocity_correction;
    FullESKF_InjectError(error_state);
    return 1U;
}

/* -------------------------------------------------------------------------- */

static float FullESKF_ComputeGravityWeight(float accel_norm_g)
{
    if ((accel_norm_g >= APP_FULL_ESKF_GRAVITY_FULL_WEIGHT_MIN_G) &&
        (accel_norm_g <= APP_FULL_ESKF_GRAVITY_FULL_WEIGHT_MAX_G))
    {
        return 1.0f;
    }

    if ((accel_norm_g <= APP_FULL_ESKF_GRAVITY_ZERO_WEIGHT_MIN_G) ||
        (accel_norm_g >= APP_FULL_ESKF_GRAVITY_ZERO_WEIGHT_MAX_G))
    {
        return 0.0f;
    }

    if (accel_norm_g < APP_FULL_ESKF_GRAVITY_FULL_WEIGHT_MIN_G)
    {
        return
            (accel_norm_g - APP_FULL_ESKF_GRAVITY_ZERO_WEIGHT_MIN_G) /
            (APP_FULL_ESKF_GRAVITY_FULL_WEIGHT_MIN_G -
             APP_FULL_ESKF_GRAVITY_ZERO_WEIGHT_MIN_G);
    }

    return
        (APP_FULL_ESKF_GRAVITY_ZERO_WEIGHT_MAX_G - accel_norm_g) /
        (APP_FULL_ESKF_GRAVITY_ZERO_WEIGHT_MAX_G -
         APP_FULL_ESKF_GRAVITY_FULL_WEIGHT_MAX_G);
}

/* -------------------------------------------------------------------------- */

static void FullESKF_ProcessGravity(const SensorData_t *sensor)
{
#if (APP_FULL_ESKF_GRAVITY_UPDATE_ENABLED != 0U)
    uint32_t axis;
    uint8_t accepted = 0U;

    /* Start of a new 50 Hz gravity batch.  Measurement direction and
     * linearization are snapshotted once so X/Y/Z remain one coherent vector
     * observation even though the three sparse Joseph updates are executed
     * over consecutive 200 Hz scheduler slots. */
    if (gravity_update_phase == FULL_ESKF_GRAVITY_PHASE_IDLE)
    {
        float accel_body[3];
        float accel_norm;
        float gravity_weight;
        float measurement_variance;
        float accel_norm_g;
        float vibration_r_multiplier = 1.0f;
        float qw;
        float qx;
        float qy;
        float qz;

        accel_body[0] =
            sensor->accel_x_filtered_g * APP_ATTITUDE_GRAVITY_MPS2 -
            nominal_accel_bias[0];
        accel_body[1] =
            sensor->accel_y_filtered_g * APP_ATTITUDE_GRAVITY_MPS2 -
            nominal_accel_bias[1];
        accel_body[2] =
            sensor->accel_z_filtered_g * APP_ATTITUDE_GRAVITY_MPS2 -
            nominal_accel_bias[2];

        accel_norm = sqrtf(
            (accel_body[0] * accel_body[0]) +
            (accel_body[1] * accel_body[1]) +
            (accel_body[2] * accel_body[2])
        );

        accel_norm_g = accel_norm / APP_ATTITUDE_GRAVITY_MPS2;
        gravity_weight = FullESKF_ComputeGravityWeight(accel_norm_g);

        /* Preserve the existing vibration-adaptive R calculation, but update
         * it once per gravity vector batch exactly as before. */
        if (vibration_metric_initialized == 0U)
        {
            vibration_previous_norm_g = accel_norm_g;
            vibration_metric_initialized = 1U;
        }
        else
        {
            float jerk_g = fabsf(accel_norm_g - vibration_previous_norm_g);
            vibration_previous_norm_g = accel_norm_g;
            vibration_metric_ema_g += APP_FULL_ESKF_VIBRATION_EMA_ALPHA *
                (jerk_g - vibration_metric_ema_g);
        }

        if (vibration_metric_ema_g > APP_FULL_ESKF_VIBRATION_JERK_START_G)
        {
            float span = APP_FULL_ESKF_VIBRATION_JERK_FULL_G -
                         APP_FULL_ESKF_VIBRATION_JERK_START_G;
            float ratio = (vibration_metric_ema_g -
                           APP_FULL_ESKF_VIBRATION_JERK_START_G) / span;
            if (ratio > 1.0f)
            {
                ratio = 1.0f;
            }
            vibration_r_multiplier = 1.0f + ratio *
                (APP_FULL_ESKF_VIBRATION_R_MAX_MULTIPLIER - 1.0f);
        }

        eskf_data.vibration_metric_g = vibration_metric_ema_g;
        eskf_data.vibration_r_multiplier = vibration_r_multiplier;
        eskf_data.gravity_correction_weight = gravity_weight;
        eskf_data.gravity_correction_active = 0U;
        gravity_batch_accepted_count = 0U;

        if ((gravity_weight <= 0.0f) || (accel_norm < 0.1f))
        {
            eskf_data.gravity_reject_count++;
            return;
        }

        gravity_batch_measured_up[0] = accel_body[0] / accel_norm;
        gravity_batch_measured_up[1] = accel_body[1] / accel_norm;
        gravity_batch_measured_up[2] = accel_body[2] / accel_norm;

        qw = nominal_quaternion[0];
        qx = nominal_quaternion[1];
        qy = nominal_quaternion[2];
        qz = nominal_quaternion[3];

        gravity_batch_predicted_up[0] =
            2.0f * ((qx * qz) - (qw * qy));
        gravity_batch_predicted_up[1] =
            2.0f * ((qy * qz) + (qw * qx));
        gravity_batch_predicted_up[2] =
            1.0f - (2.0f * ((qx * qx) + (qy * qy)));

        measurement_variance =
            APP_FULL_ESKF_GRAVITY_DIRECTION_STD *
            APP_FULL_ESKF_GRAVITY_DIRECTION_STD;
        measurement_variance /= (gravity_weight * gravity_weight);
        measurement_variance *= vibration_r_multiplier;
        gravity_batch_measurement_variance = measurement_variance;
        gravity_update_phase = FULL_ESKF_GRAVITY_PHASE_X;
    }

    if ((gravity_update_phase < FULL_ESKF_GRAVITY_PHASE_X) ||
        (gravity_update_phase > FULL_ESKF_GRAVITY_PHASE_Z))
    {
        gravity_update_phase = FULL_ESKF_GRAVITY_PHASE_IDLE;
        gravity_batch_accepted_count = 0U;
        eskf_data.gravity_correction_active = 0U;
        return;
    }

    axis = (uint32_t)(gravity_update_phase - FULL_ESKF_GRAVITY_PHASE_X);

#if (APP_FULL_ESKF_SPARSE_SCALAR_UPDATES_ENABLED != 0U)
    if (axis == 0UL)
    {
        accepted = FullESKF_ScalarUpdateTwoStates(
            FULL_ESKF_THETA_Y,
            -gravity_batch_predicted_up[2],
            FULL_ESKF_THETA_Z,
            gravity_batch_predicted_up[1],
            gravity_batch_measured_up[axis] -
                gravity_batch_predicted_up[axis],
            gravity_batch_measurement_variance,
            APP_FULL_ESKF_GRAVITY_GATE_SIGMA,
            APP_FULL_ESKF_GRAVITY_ABS_GATE,
            FULL_ESKF_COV_STAGE_GRAVITY
        );
    }
    else if (axis == 1UL)
    {
        accepted = FullESKF_ScalarUpdateTwoStates(
            FULL_ESKF_THETA_X,
            gravity_batch_predicted_up[2],
            FULL_ESKF_THETA_Z,
            -gravity_batch_predicted_up[0],
            gravity_batch_measured_up[axis] -
                gravity_batch_predicted_up[axis],
            gravity_batch_measurement_variance,
            APP_FULL_ESKF_GRAVITY_GATE_SIGMA,
            APP_FULL_ESKF_GRAVITY_ABS_GATE,
            FULL_ESKF_COV_STAGE_GRAVITY
        );
    }
    else
    {
        accepted = FullESKF_ScalarUpdateTwoStates(
            FULL_ESKF_THETA_X,
            -gravity_batch_predicted_up[1],
            FULL_ESKF_THETA_Y,
            gravity_batch_predicted_up[0],
            gravity_batch_measured_up[axis] -
                gravity_batch_predicted_up[axis],
            gravity_batch_measurement_variance,
            APP_FULL_ESKF_GRAVITY_GATE_SIGMA,
            APP_FULL_ESKF_GRAVITY_ABS_GATE,
            FULL_ESKF_COV_STAGE_GRAVITY
        );
    }
#else
    {
        float h[FULL_ESKF_STATE_COUNT] = {0.0f};

        if (axis == 0UL)
        {
            h[FULL_ESKF_THETA_Y] = -gravity_batch_predicted_up[2];
            h[FULL_ESKF_THETA_Z] = gravity_batch_predicted_up[1];
        }
        else if (axis == 1UL)
        {
            h[FULL_ESKF_THETA_X] = gravity_batch_predicted_up[2];
            h[FULL_ESKF_THETA_Z] = -gravity_batch_predicted_up[0];
        }
        else
        {
            h[FULL_ESKF_THETA_X] = -gravity_batch_predicted_up[1];
            h[FULL_ESKF_THETA_Y] = gravity_batch_predicted_up[0];
        }

        accepted = FullESKF_ScalarUpdate(
            h,
            gravity_batch_measured_up[axis] -
                gravity_batch_predicted_up[axis],
            gravity_batch_measurement_variance,
            APP_FULL_ESKF_GRAVITY_GATE_SIGMA,
            APP_FULL_ESKF_GRAVITY_ABS_GATE,
            FULL_ESKF_COV_STAGE_GRAVITY
        );
    }
#endif

    if (accepted != 0U)
    {
        if (gravity_batch_accepted_count < 3U)
        {
            gravity_batch_accepted_count++;
        }
        eskf_data.gravity_correction_active = 1U;
    }

    if (gravity_update_phase < FULL_ESKF_GRAVITY_PHASE_Z)
    {
        gravity_update_phase++;
        return;
    }

    /* Complete one logical three-axis gravity batch.  Update/reject counters
     * preserve their old per-batch meaning rather than counting each slice. */
    if (gravity_batch_accepted_count > 0U)
    {
        eskf_data.gravity_correction_active = 1U;
        eskf_data.gravity_update_count++;
    }
    else
    {
        eskf_data.gravity_correction_active = 0U;
        eskf_data.gravity_reject_count++;
    }

    gravity_update_phase = FULL_ESKF_GRAVITY_PHASE_IDLE;
    gravity_batch_accepted_count = 0U;
#else
    (void)sensor;
    gravity_update_phase = FULL_ESKF_GRAVITY_PHASE_IDLE;
    gravity_batch_accepted_count = 0U;
    eskf_data.gravity_correction_active = 0U;
    eskf_data.gravity_correction_weight = 0.0f;
#endif
}

/* -------------------------------------------------------------------------- */

static void FullESKF_ResetGyroBootstrapAccumulator(void)
{
    gyro_bootstrap_sample_count = 0UL;
    gyro_bootstrap_quality_score = 0UL;
    memset(gyro_bootstrap_mean_dps, 0, sizeof(gyro_bootstrap_mean_dps));
    memset(gyro_bootstrap_m2_dps2, 0, sizeof(gyro_bootstrap_m2_dps2));

    full_eskf_gyro_bootstrap_sample_count = 0UL;
    full_eskf_gyro_bootstrap_quality_score = 0UL;
    full_eskf_gyro_bootstrap_std_x_dps = 0.0f;
    full_eskf_gyro_bootstrap_std_y_dps = 0.0f;
    full_eskf_gyro_bootstrap_std_z_dps = 0.0f;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_ProcessGyroBiasBootstrap(const SensorData_t *sensor)
{
#if (APP_FULL_ESKF_GYRO_BOOTSTRAP_ENABLED != 0U)
    float gyro_sample_dps[3];
    float gyro_norm_dps;
    uint8_t sample_valid;
    uint32_t axis;

    if (gyro_bootstrap_done != 0U)
    {
        full_eskf_gyro_bootstrap_active = 0U;
        return;
    }

    full_eskf_gyro_bootstrap_active = 1U;

    if (gyro_bootstrap_attempt_count < 0xFFFFFFFFUL)
    {
        gyro_bootstrap_attempt_count++;
    }

    /*
     * A failed startup window does not permanently disable bootstrap. The
     * estimator clears the statistics and immediately starts a new window.
     */
    if (gyro_bootstrap_attempt_count >
        APP_FULL_ESKF_GYRO_BOOTSTRAP_MAX_SAMPLES)
    {
        gyro_bootstrap_attempt_count = 0UL;
        FullESKF_ResetGyroBootstrapAccumulator();
    }

    full_eskf_gyro_bootstrap_attempt_count =
        gyro_bootstrap_attempt_count;

    gyro_sample_dps[0] = sensor->gyro_x_filtered_dps;
    gyro_sample_dps[1] = sensor->gyro_y_filtered_dps;
    gyro_sample_dps[2] = sensor->gyro_z_filtered_dps;

    gyro_norm_dps = sqrtf(
        (gyro_sample_dps[0] * gyro_sample_dps[0]) +
        (gyro_sample_dps[1] * gyro_sample_dps[1]) +
        (gyro_sample_dps[2] * gyro_sample_dps[2])
    );

    sample_valid =
        ((sensor->accel_filtered_norm_g >=
          APP_FULL_ESKF_GYRO_BOOTSTRAP_ACCEL_MIN_G) &&
         (sensor->accel_filtered_norm_g <=
          APP_FULL_ESKF_GYRO_BOOTSTRAP_ACCEL_MAX_G) &&
         (gyro_norm_dps <= APP_FULL_ESKF_GYRO_BOOTSTRAP_MAX_NORM_DPS)) ?
        1U : 0U;

    if (sample_valid == 0U)
    {
        /*
         * Do not erase several seconds of good data because of one vibration
         * spike. Bad samples decrease a quality score; only a sustained bad
         * interval clears the Welford accumulator.
         */
        if (gyro_bootstrap_quality_score >
            APP_FULL_ESKF_GYRO_BOOTSTRAP_SCORE_DECREMENT)
        {
            gyro_bootstrap_quality_score -=
                APP_FULL_ESKF_GYRO_BOOTSTRAP_SCORE_DECREMENT;
        }
        else
        {
            FullESKF_ResetGyroBootstrapAccumulator();
        }

        full_eskf_gyro_bootstrap_quality_score =
            gyro_bootstrap_quality_score;
        return;
    }

    if (gyro_bootstrap_quality_score <
        APP_FULL_ESKF_GYRO_BOOTSTRAP_MIN_SAMPLES)
    {
        gyro_bootstrap_quality_score++;
    }

    gyro_bootstrap_sample_count++;

    for (axis = 0UL; axis < 3UL; axis++)
    {
        float delta =
            gyro_sample_dps[axis] - gyro_bootstrap_mean_dps[axis];

        gyro_bootstrap_mean_dps[axis] +=
            delta / (float)gyro_bootstrap_sample_count;

        gyro_bootstrap_m2_dps2[axis] +=
            delta *
            (gyro_sample_dps[axis] - gyro_bootstrap_mean_dps[axis]);
    }

    full_eskf_gyro_bootstrap_sample_count =
        gyro_bootstrap_sample_count;
    full_eskf_gyro_bootstrap_quality_score =
        gyro_bootstrap_quality_score;

    if ((gyro_bootstrap_sample_count <
         APP_FULL_ESKF_GYRO_BOOTSTRAP_MIN_SAMPLES) ||
        (gyro_bootstrap_quality_score <
         APP_FULL_ESKF_GYRO_BOOTSTRAP_MIN_SAMPLES))
    {
        return;
    }

    {
        float bootstrap_std_dps[3];
        uint8_t stable = 1U;

        for (axis = 0UL; axis < 3UL; axis++)
        {
            float variance = gyro_bootstrap_m2_dps2[axis] /
                (float)(gyro_bootstrap_sample_count - 1UL);

            if (variance < 0.0f)
            {
                variance = 0.0f;
            }

            bootstrap_std_dps[axis] = sqrtf(variance);

            if (bootstrap_std_dps[axis] >
                APP_FULL_ESKF_GYRO_BOOTSTRAP_MAX_STD_DPS)
            {
                stable = 0U;
            }
        }

        full_eskf_gyro_bootstrap_std_x_dps = bootstrap_std_dps[0];
        full_eskf_gyro_bootstrap_std_y_dps = bootstrap_std_dps[1];
        full_eskf_gyro_bootstrap_std_z_dps = bootstrap_std_dps[2];

        if (stable == 0U)
        {
            gyro_bootstrap_attempt_count = 0UL;
            FullESKF_ResetGyroBootstrapAccumulator();
            return;
        }

        for (axis = 0UL; axis < 3UL; axis++)
        {
            float bias_std_rad_s =
                bootstrap_std_dps[axis] * FULL_ESKF_DEG_TO_RAD;
            float minimum_std_rad_s =
                APP_FULL_ESKF_STATIONARY_GYRO_BIAS_STD_DPS *
                FULL_ESKF_DEG_TO_RAD;

            nominal_gyro_bias[axis] =
                gyro_bootstrap_mean_dps[axis] * FULL_ESKF_DEG_TO_RAD;

            if (bias_std_rad_s < minimum_std_rad_s)
            {
                bias_std_rad_s = minimum_std_rad_s;
            }

            covariance[FULL_ESKF_BG_X + axis]
                      [FULL_ESKF_BG_X + axis] =
                bias_std_rad_s * bias_std_rad_s;

            nominal_position[axis] = 0.0f;
            nominal_velocity[axis] = 0.0f;
            stationary_gyro_lpf_dps[axis] = 0.0f;
        }
    }

    {
        const AttitudeEstimatorData_t *attitude =
            AttitudeEstimator_GetDataPtr();

        if ((attitude->initialized != 0U) &&
            (attitude->healthy != 0U))
        {
            nominal_quaternion[0] = attitude->q_w;
            nominal_quaternion[1] = attitude->q_x;
            nominal_quaternion[2] = attitude->q_y;
            nominal_quaternion[3] = attitude->q_z;
            (void)FullESKF_NormalizeQuaternion();
        }
    }

    FullESKF_ClearCovarianceAccumulator();

    eskf_data.stationary_sample_count = 0UL;
    eskf_data.stationary_detected = 0U;

    gyro_bootstrap_done = 1U;
    full_eskf_gyro_bootstrap_done = 1U;
    full_eskf_gyro_bootstrap_active = 0U;
#else
    (void)sensor;
    gyro_bootstrap_done = 1U;
    full_eskf_gyro_bootstrap_done = 1U;
    full_eskf_gyro_bootstrap_active = 0U;
#endif
}

/* -------------------------------------------------------------------------- */

static void FullESKF_UpdateStationaryDetector(const SensorData_t *sensor)
{
    float gyro_residual_dps[3];
    float gyro_lpf_norm_dps;
    float gyro_threshold_dps;
    uint8_t vertical_speed_stationary_ok = 1U;
    uint32_t axis;

#if (APP_FULL_ESKF_GYRO_BOOTSTRAP_ENABLED != 0U)
    if (gyro_bootstrap_done == 0U)
    {
        eskf_data.stationary_sample_count = 0UL;
        eskf_data.stationary_detected = 0U;
        stationary_confirmed_sample_count = 0UL;
        full_eskf_stationary_confirmed_sample_count = 0UL;
        full_eskf_stationary_gyro_lpf_norm_dps = 0.0f;
        return;
    }
#endif

    eskf_data.stationary_detector_count++;

    gyro_residual_dps[0] =
        sensor->gyro_x_filtered_dps -
        (nominal_gyro_bias[0] * FULL_ESKF_RAD_TO_DEG);
    gyro_residual_dps[1] =
        sensor->gyro_y_filtered_dps -
        (nominal_gyro_bias[1] * FULL_ESKF_RAD_TO_DEG);
    gyro_residual_dps[2] =
        sensor->gyro_z_filtered_dps -
        (nominal_gyro_bias[2] * FULL_ESKF_RAD_TO_DEG);

    for (axis = 0UL; axis < 3UL; axis++)
    {
        stationary_gyro_lpf_dps[axis] +=
            APP_FULL_ESKF_STATIONARY_GYRO_LPF_ALPHA *
            (gyro_residual_dps[axis] - stationary_gyro_lpf_dps[axis]);
    }

    gyro_lpf_norm_dps = sqrtf(
        (stationary_gyro_lpf_dps[0] * stationary_gyro_lpf_dps[0]) +
        (stationary_gyro_lpf_dps[1] * stationary_gyro_lpf_dps[1]) +
        (stationary_gyro_lpf_dps[2] * stationary_gyro_lpf_dps[2])
    );

    full_eskf_stationary_gyro_lpf_norm_dps = gyro_lpf_norm_dps;

    gyro_threshold_dps =
        (eskf_data.stationary_detected == 0U) ?
        APP_FULL_ESKF_STATIONARY_GYRO_ACQUIRE_MAX_DPS :
        APP_FULL_ESKF_STATIONARY_GYRO_RELEASE_MAX_DPS;

    /* Before startup-origin lock, ZUPT is allowed to bootstrap velocity/bias.
     * Afterwards it must also agree with the estimated vertical velocity.
     * This blocks false "stationary" detection during a smooth powered
     * descent where accel norm can be ~1 g and angular rate can be small. */
    if ((origin_zero_applied != 0U) &&
        (FullESKF_Abs(nominal_velocity[2]) >
         APP_FULL_ESKF_STATIONARY_MAX_VERTICAL_SPEED_MPS))
    {
        vertical_speed_stationary_ok = 0U;
    }

    /* Velocity is a stronger motion cue than the accel/gyro score.  Release
     * immediately instead of waiting for score hysteresis; otherwise an
     * already-active ZUPT batch could pull vz back toward zero while the
     * detector is trying to release. */
    if (vertical_speed_stationary_ok == 0U)
    {
        eskf_data.stationary_sample_count = 0UL;
        eskf_data.stationary_detected = 0U;
        stationary_confirmed_sample_count = 0UL;
        full_eskf_stationary_confirmed_sample_count = 0UL;
        return;
    }

    if ((sensor->accel_filtered_norm_g >=
         APP_FULL_ESKF_STATIONARY_ACCEL_MIN_G) &&
        (sensor->accel_filtered_norm_g <=
         APP_FULL_ESKF_STATIONARY_ACCEL_MAX_G) &&
        (gyro_lpf_norm_dps <= gyro_threshold_dps) &&
        (vertical_speed_stationary_ok != 0U))
    {
        if (eskf_data.stationary_sample_count <
            APP_FULL_ESKF_STATIONARY_MIN_SAMPLES)
        {
            eskf_data.stationary_sample_count++;
        }
    }
    else
    {
        if (eskf_data.stationary_sample_count >
            APP_FULL_ESKF_STATIONARY_SCORE_DECREMENT)
        {
            eskf_data.stationary_sample_count -=
                APP_FULL_ESKF_STATIONARY_SCORE_DECREMENT;
        }
        else
        {
            eskf_data.stationary_sample_count = 0UL;
        }
    }

    if (eskf_data.stationary_detected == 0U)
    {
        if (eskf_data.stationary_sample_count >=
            APP_FULL_ESKF_STATIONARY_MIN_SAMPLES)
        {
            eskf_data.stationary_detected = 1U;
        }
    }
    else if (eskf_data.stationary_sample_count <=
             APP_FULL_ESKF_STATIONARY_RELEASE_SCORE)
    {
        eskf_data.stationary_detected = 0U;
    }

    if (eskf_data.stationary_detected != 0U)
    {
        if (stationary_confirmed_sample_count < 0xFFFFFFFFUL)
        {
            stationary_confirmed_sample_count++;
        }
    }
    else
    {
        stationary_confirmed_sample_count = 0UL;
    }

    full_eskf_stationary_confirmed_sample_count =
        stationary_confirmed_sample_count;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_ProcessStationaryUpdates(const SensorData_t *sensor)
{
#if (APP_FULL_ESKF_STATIONARY_UPDATE_ENABLED != 0U)
    uint32_t axis;

    if ((eskf_data.stationary_detected == 0U) ||
        ((origin_zero_applied != 0U) &&
         (FullESKF_Abs(nominal_velocity[2]) >
          APP_FULL_ESKF_STATIONARY_MAX_VERTICAL_SPEED_MPS)))
    {
        stationary_update_decimation_counter = 0UL;
        stationary_update_phase = FULL_ESKF_STATIONARY_PHASE_IDLE;
        return;
    }

    /* Keep the 10 Hz batch cadence, but make the batch non-blocking.
     * A batch is spread over consecutive 200 Hz correction calls so a single
     * cooperative-scheduler slot never executes all ZUPT/bias updates. */
    if (stationary_update_decimation_counter <
        APP_FULL_ESKF_STATIONARY_UPDATE_DECIMATION)
    {
        stationary_update_decimation_counter++;
    }

    if (stationary_update_phase == FULL_ESKF_STATIONARY_PHASE_IDLE)
    {
        if (stationary_update_decimation_counter <
            APP_FULL_ESKF_STATIONARY_UPDATE_DECIMATION)
        {
            return;
        }

        stationary_update_decimation_counter = 0UL;
        stationary_update_phase = FULL_ESKF_STATIONARY_PHASE_ZUPT_X;
    }

    if ((stationary_update_phase >= FULL_ESKF_STATIONARY_PHASE_ZUPT_X) &&
        (stationary_update_phase <= FULL_ESKF_STATIONARY_PHASE_ZUPT_Z))
    {
        axis = (uint32_t)(
            stationary_update_phase - FULL_ESKF_STATIONARY_PHASE_ZUPT_X);

#if (APP_FULL_ESKF_SPARSE_SCALAR_UPDATES_ENABLED != 0U)
        (void)FullESKF_ScalarUpdateUnitState(
            FULL_ESKF_VX + axis,
            -nominal_velocity[axis],
            APP_FULL_ESKF_ZUPT_STD_MPS * APP_FULL_ESKF_ZUPT_STD_MPS,
            APP_FULL_ESKF_STATIONARY_GATE_SIGMA,
            APP_FULL_ESKF_ZUPT_ABS_GATE_MPS,
            FULL_ESKF_COV_STAGE_ZUPT
        );
#else
        {
            float h[FULL_ESKF_STATE_COUNT] = {0.0f};
            h[FULL_ESKF_VX + axis] = 1.0f;

            (void)FullESKF_ScalarUpdate(
                h,
                -nominal_velocity[axis],
                APP_FULL_ESKF_ZUPT_STD_MPS * APP_FULL_ESKF_ZUPT_STD_MPS,
                APP_FULL_ESKF_STATIONARY_GATE_SIGMA,
                APP_FULL_ESKF_ZUPT_ABS_GATE_MPS,
                FULL_ESKF_COV_STAGE_ZUPT
            );
        }
#endif

        stationary_update_phase++;
        return;
    }

    if ((stationary_update_phase >= FULL_ESKF_STATIONARY_PHASE_GYRO_X) &&
        (stationary_update_phase <= FULL_ESKF_STATIONARY_PHASE_GYRO_Z))
    {
        float gyro_bias_measurement_std_rad_s =
            APP_FULL_ESKF_STATIONARY_GYRO_BIAS_STD_DPS *
            FULL_ESKF_DEG_TO_RAD;
        float gyro_bias_max_step_rad_s =
            APP_FULL_ESKF_STATIONARY_GYRO_BIAS_MAX_STEP_DPS *
            FULL_ESKF_DEG_TO_RAD;
        float gyro_measurement_rad_s;

        axis = (uint32_t)(
            stationary_update_phase - FULL_ESKF_STATIONARY_PHASE_GYRO_X);

        if (axis == 0UL)
        {
            gyro_measurement_rad_s =
                sensor->gyro_x_filtered_dps * FULL_ESKF_DEG_TO_RAD;
        }
        else if (axis == 1UL)
        {
            gyro_measurement_rad_s =
                sensor->gyro_y_filtered_dps * FULL_ESKF_DEG_TO_RAD;
        }
        else
        {
            gyro_measurement_rad_s =
                sensor->gyro_z_filtered_dps * FULL_ESKF_DEG_TO_RAD;
        }

        (void)FullESKF_ConstrainedSingleStateUpdate(
            FULL_ESKF_BG_X + axis,
            gyro_measurement_rad_s - nominal_gyro_bias[axis],
            gyro_bias_measurement_std_rad_s *
            gyro_bias_measurement_std_rad_s,
            APP_FULL_ESKF_STATIONARY_GATE_SIGMA,
            APP_FULL_ESKF_STATIONARY_GYRO_BIAS_ABS_GATE_DPS *
            FULL_ESKF_DEG_TO_RAD,
            gyro_bias_max_step_rad_s,
            FULL_ESKF_COV_STAGE_GYRO_BIAS
        );

        if (stationary_update_phase < FULL_ESKF_STATIONARY_PHASE_GYRO_Z)
        {
            stationary_update_phase++;
            return;
        }

        if ((stationary_confirmed_sample_count >=
             APP_FULL_ESKF_STATIONARY_ACCEL_BIAS_WARMUP_SAMPLES) &&
            (eskf_data.gravity_correction_active != 0U))
        {
            stationary_update_phase = FULL_ESKF_STATIONARY_PHASE_ACCEL_X;
        }
        else
        {
            stationary_update_phase = FULL_ESKF_STATIONARY_PHASE_IDLE;
            eskf_data.stationary_update_count++;
        }

        return;
    }

    if ((stationary_update_phase >= FULL_ESKF_STATIONARY_PHASE_ACCEL_X) &&
        (stationary_update_phase <= FULL_ESKF_STATIONARY_PHASE_ACCEL_Z))
    {
        float rotation[3][3];
        float gravity_body_axis;
        float accel_measurement_axis;

        /* If stationary qualification/gravity ownership changed mid-batch,
         * terminate the optional accel-bias tail without carrying stale work. */
        if ((stationary_confirmed_sample_count <
             APP_FULL_ESKF_STATIONARY_ACCEL_BIAS_WARMUP_SAMPLES) ||
            (eskf_data.gravity_correction_active == 0U))
        {
            stationary_update_phase = FULL_ESKF_STATIONARY_PHASE_IDLE;
            eskf_data.stationary_update_count++;
            return;
        }

        axis = (uint32_t)(
            stationary_update_phase - FULL_ESKF_STATIONARY_PHASE_ACCEL_X);

        FullESKF_QuaternionToRotation(rotation);
        gravity_body_axis =
            rotation[2][axis] * APP_ATTITUDE_GRAVITY_MPS2;

        if (axis == 0UL)
        {
            accel_measurement_axis =
                sensor->accel_x_filtered_g * APP_ATTITUDE_GRAVITY_MPS2;
        }
        else if (axis == 1UL)
        {
            accel_measurement_axis =
                sensor->accel_y_filtered_g * APP_ATTITUDE_GRAVITY_MPS2;
        }
        else
        {
            accel_measurement_axis =
                sensor->accel_z_filtered_g * APP_ATTITUDE_GRAVITY_MPS2;
        }

        (void)FullESKF_ConstrainedSingleStateUpdate(
            FULL_ESKF_BA_X + axis,
            accel_measurement_axis -
            (gravity_body_axis + nominal_accel_bias[axis]),
            APP_FULL_ESKF_STATIONARY_ACCEL_BIAS_STD_MPS2 *
            APP_FULL_ESKF_STATIONARY_ACCEL_BIAS_STD_MPS2,
            APP_FULL_ESKF_STATIONARY_GATE_SIGMA,
            APP_FULL_ESKF_STATIONARY_ACCEL_BIAS_ABS_GATE_MPS2,
            APP_FULL_ESKF_STATIONARY_ACCEL_BIAS_MAX_STEP_MPS2,
            FULL_ESKF_COV_STAGE_ACCEL_BIAS
        );

        if (stationary_update_phase < FULL_ESKF_STATIONARY_PHASE_ACCEL_Z)
        {
            stationary_update_phase++;
        }
        else
        {
            stationary_update_phase = FULL_ESKF_STATIONARY_PHASE_IDLE;
            eskf_data.stationary_update_count++;
        }

        return;
    }

    /* Defensive recovery from an impossible phase value. */
    stationary_update_phase = FULL_ESKF_STATIONARY_PHASE_IDLE;
#else
    (void)sensor;
#endif
}
/* -------------------------------------------------------------------------- */

static uint8_t FullESKF_IsBarometerFresh(
    const BarometerData_t *barometer,
    uint32_t now_us
)
{
    uint32_t age_us;

    if (barometer->last_sample_timestamp_us == 0UL)
    {
        full_eskf_baro_sample_age_us = 0xFFFFFFFFUL;
        return 0U;
    }

    age_us = (uint32_t)(now_us - barometer->last_sample_timestamp_us);
    full_eskf_baro_sample_age_us = age_us;

    if ((barometer->initialized == 0U) ||
        (barometer->connected == 0U) ||
        (barometer->data_ready == 0U) ||
        (barometer->healthy == 0U) ||
        (barometer->pressure_valid == 0U) ||
        (barometer->calibrated == 0U) ||
        (barometer->filter_initialized == 0U) ||
        (age_us > APP_FULL_ESKF_BARO_STALE_TIMEOUT_US))
    {
        return 0U;
    }

    return 1U;
}

/* -------------------------------------------------------------------------- */

static uint8_t FullESKF_IsLidarFresh(
    const LidarData_t *lidar,
    uint32_t now_us
)
{
    uint32_t age_us;

    if (lidar->last_sample_timestamp_us == 0UL)
    {
        full_eskf_lidar_sample_age_us = 0xFFFFFFFFUL;
        return 0U;
    }

    age_us = (uint32_t)(now_us - lidar->last_sample_timestamp_us);
    full_eskf_lidar_sample_age_us = age_us;

    if ((lidar->initialized == 0U) ||
        (lidar->connected == 0U) ||
        (lidar->data_ready == 0U) ||
        (lidar->distance_valid == 0U) ||
        (lidar->filter_initialized == 0U) ||
        (age_us > APP_FULL_ESKF_LIDAR_STALE_TIMEOUT_US))
    {
        return 0U;
    }

    return 1U;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_UpdateXYDiagnosticProjection(
    float world_accel_x_mps2,
    float world_accel_y_mps2,
    float dt_s)
{
    float accel_xy[2];
    float bias_tau_s;
    float bias_gain;
    float velocity_damp;
    float position_damp;
    float half_dt_squared;
    uint32_t axis;

    if ((origin_zero_applied == 0U) ||
        (dt_s <= 0.0f) ||
        (dt_s > 0.05f))
    {
        memset(xy_diag_position_m, 0, sizeof(xy_diag_position_m));
        memset(xy_diag_velocity_mps, 0, sizeof(xy_diag_velocity_mps));
        memset(xy_diag_accel_bias_mps2, 0, sizeof(xy_diag_accel_bias_mps2));
        xy_diag_initialized = 0U;
        return;
    }

    /* A future real horizontal aiding source owns X/Y directly. */
    if (eskf_data.horizontal_position_valid != 0U)
    {
        xy_diag_position_m[0] = nominal_position[0];
        xy_diag_position_m[1] = nominal_position[1];
        xy_diag_velocity_mps[0] = nominal_velocity[0];
        xy_diag_velocity_mps[1] = nominal_velocity[1];
        xy_diag_accel_bias_mps2[0] = 0.0f;
        xy_diag_accel_bias_mps2[1] = 0.0f;
        xy_diag_initialized = 1U;
        return;
    }

    if (xy_diag_initialized == 0U)
    {
        memset(xy_diag_position_m, 0, sizeof(xy_diag_position_m));
        memset(xy_diag_velocity_mps, 0, sizeof(xy_diag_velocity_mps));
        memset(xy_diag_accel_bias_mps2, 0, sizeof(xy_diag_accel_bias_mps2));
        xy_diag_initialized = 1U;
    }

    accel_xy[0] = world_accel_x_mps2;
    accel_xy[1] = world_accel_y_mps2;

    /* When stationary is confidently confirmed, learn the residual horizontal
     * acceleration bias faster and force only the diagnostic velocity to zero.
     * The real ESKF nominal states/covariance are not modified. */
    if ((eskf_data.stationary_detected != 0U) &&
        (stationary_confirmed_sample_count >=
         APP_FULL_ESKF_STATIONARY_MIN_SAMPLES))
    {
        bias_tau_s = FULL_ESKF_XY_DIAG_STATIONARY_BIAS_TAU_S;
    }
    else
    {
        bias_tau_s = FULL_ESKF_XY_DIAG_ACCEL_BIAS_TAU_S;
    }

    bias_gain = dt_s / (bias_tau_s + dt_s);
    velocity_damp =
        1.0f - FullESKF_Clamp(
            dt_s / FULL_ESKF_XY_DIAG_VELOCITY_DAMP_TAU_S, 0.0f, 1.0f);
    position_damp =
        1.0f - FullESKF_Clamp(
            dt_s / FULL_ESKF_XY_DIAG_POSITION_DAMP_TAU_S, 0.0f, 1.0f);
    half_dt_squared = 0.5f * dt_s * dt_s;

    for (axis = 0UL; axis < 2UL; axis++)
    {
        float corrected_accel;

        xy_diag_accel_bias_mps2[axis] +=
            bias_gain *
            (accel_xy[axis] - xy_diag_accel_bias_mps2[axis]);

        corrected_accel =
            accel_xy[axis] - xy_diag_accel_bias_mps2[axis];

        xy_diag_position_m[axis] +=
            (xy_diag_velocity_mps[axis] * dt_s) +
            (corrected_accel * half_dt_squared);
        xy_diag_velocity_mps[axis] += corrected_accel * dt_s;

        if ((eskf_data.stationary_detected != 0U) &&
            (stationary_confirmed_sample_count >=
             APP_FULL_ESKF_STATIONARY_MIN_SAMPLES))
        {
            xy_diag_velocity_mps[axis] = 0.0f;
        }
        else
        {
            xy_diag_velocity_mps[axis] *= velocity_damp;
        }

        xy_diag_position_m[axis] *= position_damp;
        xy_diag_position_m[axis] = FullESKF_Clamp(
            xy_diag_position_m[axis],
            -FULL_ESKF_XY_DIAG_POSITION_LIMIT_M,
            FULL_ESKF_XY_DIAG_POSITION_LIMIT_M);
        xy_diag_velocity_mps[axis] = FullESKF_Clamp(
            xy_diag_velocity_mps[axis],
            -FULL_ESKF_XY_DIAG_VELOCITY_LIMIT_MPS,
            FULL_ESKF_XY_DIAG_VELOCITY_LIMIT_MPS);
    }
}

/* -------------------------------------------------------------------------- */

static void FullESKF_ServiceXYRunawayGuard(void)
{
    uint8_t runaway = 0U;

    /* Do not touch startup and do not interfere with any future aided-X/Y
     * implementation. This guard is active only for the current explicitly
     * unaided/diagnostic horizontal state. */
    if ((origin_zero_applied == 0U) ||
        (eskf_data.horizontal_position_valid != 0U))
    {
        return;
    }

    if ((FullESKF_Abs(nominal_position[0]) >=
         FULL_ESKF_XY_RUNAWAY_POSITION_LIMIT_M) ||
        (FullESKF_Abs(nominal_position[1]) >=
         FULL_ESKF_XY_RUNAWAY_POSITION_LIMIT_M) ||
        (FullESKF_Abs(nominal_velocity[0]) >=
         FULL_ESKF_XY_RUNAWAY_VELOCITY_LIMIT_MPS) ||
        (FullESKF_Abs(nominal_velocity[1]) >=
         FULL_ESKF_XY_RUNAWAY_VELOCITY_LIMIT_MPS))
    {
        runaway = 1U;
    }

    if (runaway != 0U)
    {
        /* X/Y are not absolute-navigation states in this build. Contain only
         * the unobservable horizontal runaway before the SD int16-cm rail. */
        nominal_position[0] = 0.0f;
        nominal_position[1] = 0.0f;
        nominal_velocity[0] = 0.0f;
        nominal_velocity[1] = 0.0f;
    }
}

/* -------------------------------------------------------------------------- */

static void FullESKF_UpdateNavigationValidity(void)
{
    /*
     * There is currently no GPS / optical / UWB / external horizontal
     * position measurement in the state estimator. The inertial X/Y states
     * continue to propagate for diagnostics, but must not be consumed as an
     * absolute horizontal position by GNC.
     */
    eskf_data.horizontal_position_valid = 0U;

    if ((eskf_data.output_inhibited != 0U) ||
        (eskf_data.vertical_reacquire_active != 0U))
    {
        eskf_data.vertical_position_valid = 0U;
        return;
    }

    eskf_data.vertical_position_valid =
        (((eskf_data.lidar_reference_ready != 0U) &&
          (eskf_data.lidar_fresh != 0U)) ||
         ((eskf_data.baro_reference_ready != 0U) &&
          (eskf_data.baro_fresh != 0U))) ? 1U : 0U;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_ApplyStartupOriginIfReady(uint32_t now_us)
{
    if (origin_zero_applied != 0U)
    {
        return;
    }

    /*
     * Do not align the coordinate origin until:
     * - attitude gyro bootstrap is locked,
     * - the vehicle has remained stationary for a full confirmation window,
     * - a fresh LIDAR vertical reference exists.
     *
     * LIDAR is used as the mandatory startup vertical reference because the
     * barometer may be intentionally absent during bench validation.
     */
    if ((gyro_bootstrap_done == 0U) ||
        (eskf_data.stationary_detected == 0U) ||
        (stationary_confirmed_sample_count <
         APP_FULL_ESKF_ORIGIN_ZERO_STATIONARY_SAMPLES) ||
        (eskf_data.lidar_reference_ready == 0U) ||
        (eskf_data.lidar_fresh == 0U))
    {
        return;
    }

    eskf_data.origin_pre_position_x_m = nominal_position[0];
    eskf_data.origin_pre_position_y_m = nominal_position[1];
    eskf_data.origin_pre_position_z_m = nominal_position[2];

    /*
     * This is a coordinate-frame translation, not a measurement correction.
     * Covariance is intentionally preserved.
     */
    nominal_position[0] = 0.0f;
    nominal_position[1] = 0.0f;
    nominal_position[2] = 0.0f;

    /*
     * The stationary detector already constrains velocity. Explicitly zeroing
     * the residual at the origin event makes the published startup state
     * deterministic and prevents pre-bootstrap velocity residue.
     */
    nominal_velocity[0] = 0.0f;
    nominal_velocity[1] = 0.0f;
    nominal_velocity[2] = 0.0f;

    origin_zero_applied = 1U;
    eskf_data.origin_zeroed = 1U;
    eskf_data.origin_zero_count++;
    eskf_data.origin_zero_timestamp_us = now_us;
}

/* -------------------------------------------------------------------------- */

static void FullESKF_ResetVerticalSubstate(
    float position_z_m,
    float velocity_z_mps
)
{
    uint32_t i;

    nominal_position[2] = position_z_m;
    nominal_velocity[2] = velocity_z_mps;

    for (i = 0UL; i < FULL_ESKF_STATE_COUNT; i++)
    {
        covariance[FULL_ESKF_PZ][i] = 0.0f;
        covariance[i][FULL_ESKF_PZ] = 0.0f;
        covariance[FULL_ESKF_VZ][i] = 0.0f;
        covariance[i][FULL_ESKF_VZ] = 0.0f;
    }

    covariance[FULL_ESKF_PZ][FULL_ESKF_PZ] =
        APP_FULL_ESKF_INITIAL_POSITION_Z_STD_M *
        APP_FULL_ESKF_INITIAL_POSITION_Z_STD_M;
    covariance[FULL_ESKF_VZ][FULL_ESKF_VZ] =
        APP_FULL_ESKF_INITIAL_VELOCITY_STD_MPS *
        APP_FULL_ESKF_INITIAL_VELOCITY_STD_MPS;

    FullESKF_ClearCovarianceAccumulator();
    lidar_recovery_active = 0U;
    lidar_reacquire_count = 0UL;
    lidar_last_accepted_valid = 0U;
    lidar_innov_reacquire_active = 0U;
    lidar_innov_reacquire_confirm_count = 0UL;
    lidar_innov_reacquire_exit_count = 0UL;
}

static void FullESKF_ServiceVerticalDivergence(
    const BarometerData_t *barometer,
    const LidarData_t *lidar,
    uint32_t now_us
)
{
#if (APP_FULL_ESKF_VERTICAL_DIVERGENCE_ENABLED != 0U)
    uint8_t baro_ok;
    uint8_t lidar_ok;
    float baro_rel = 0.0f;
    float lidar_rel = 0.0f;
    float baro_err = 0.0f;
    float lidar_err = 0.0f;
    float consistency = 0.0f;
    uint8_t divergence_candidate = 0U;
    uint8_t divergence_reason = 0U;

    if ((eskf_data.initialized == 0U) ||
        (eskf_data.origin_zeroed == 0U))
    {
        return;
    }

    baro_ok = ((eskf_data.baro_reference_ready != 0U) &&
               (FullESKF_IsBarometerFresh(barometer, now_us) != 0U)) ? 1U : 0U;
    lidar_ok = ((eskf_data.lidar_reference_ready != 0U) &&
                (FullESKF_IsLidarFresh(lidar, now_us) != 0U)) ? 1U : 0U;

    if (baro_ok != 0U)
    {
        baro_rel = barometer->filtered_altitude_m - eskf_data.baro_reference_m;
        baro_err = baro_rel - nominal_position[2];
    }
    if (lidar_ok != 0U)
    {
        lidar_rel = lidar->filtered_distance_m - eskf_data.lidar_reference_m;
        lidar_err = lidar_rel - nominal_position[2];
    }

    if ((baro_ok != 0U) && (lidar_ok != 0U))
    {
        consistency = FullESKF_Abs(baro_rel - lidar_rel);
        eskf_data.vertical_sensor_consistency_m = consistency;

        if ((consistency <= APP_FULL_ESKF_VERTICAL_DUAL_CONSISTENCY_M) &&
            (FullESKF_Abs(baro_err) >= APP_FULL_ESKF_VERTICAL_DIVERGENCE_ABS_M) &&
            (FullESKF_Abs(lidar_err) >= APP_FULL_ESKF_VERTICAL_DIVERGENCE_ABS_M) &&
            ((baro_err * lidar_err) > 0.0f))
        {
            divergence_candidate = 1U;
            divergence_reason = 1U;
        }
    }
    else
    {
        eskf_data.vertical_sensor_consistency_m = 0.0f;
    }

    if ((divergence_candidate == 0U) && (baro_ok != 0U) &&
        (lidar_ok == 0U) &&
        (FullESKF_Abs(nominal_position[2]) >= APP_FULL_ESKF_VERTICAL_HARD_POSITION_M) &&
        (FullESKF_Abs(baro_err) >= APP_FULL_ESKF_VERTICAL_HARD_BARO_ERROR_M))
    {
        divergence_candidate = 1U;
        divergence_reason = 2U;
    }

    if ((eskf_data.vertical_reacquire_active == 0U) &&
        (divergence_candidate != 0U))
    {
        vertical_divergence_candidate_count++;
        eskf_data.vertical_divergence_candidate_count =
            vertical_divergence_candidate_count;

        if (vertical_divergence_candidate_count >=
            APP_FULL_ESKF_VERTICAL_DIVERGENCE_CONFIRM_SAMPLES)
        {
            float target_m;
            float target_vz_mps = 0.0f;

            eskf_data.output_inhibited = 1U;
            eskf_data.vertical_reacquire_active = 1U;
            eskf_data.vertical_divergence_reason = divergence_reason;
            eskf_data.vertical_divergence_count++;
            vertical_reacquire_stable_count = 0UL;

            if ((baro_ok != 0U) && (lidar_ok != 0U) &&
                (consistency <= APP_FULL_ESKF_VERTICAL_DUAL_CONSISTENCY_M))
            {
                target_m = 0.5f * (baro_rel + lidar_rel);
            }
            else
            {
                target_m = baro_rel;
            }

            if ((baro_ok != 0U) &&
                (FullESKF_IsFiniteReasonable(barometer->vertical_speed_mps) != 0U) &&
                (FullESKF_Abs(barometer->vertical_speed_mps) <=
                 APP_FULL_ESKF_VERTICAL_REACQUIRE_VZ_MAX_MPS))
            {
                target_vz_mps = barometer->vertical_speed_mps;
            }

            eskf_data.vertical_reacquire_target_m = target_m;
            FullESKF_ResetVerticalSubstate(target_m, target_vz_mps);
            eskf_data.vertical_reacquire_count++;
            vertical_divergence_candidate_count = 0UL;
        }
    }
    else if (eskf_data.vertical_reacquire_active == 0U)
    {
        vertical_divergence_candidate_count = 0UL;
        eskf_data.vertical_divergence_candidate_count = 0UL;
    }

    if (eskf_data.vertical_reacquire_active != 0U)
    {
        if ((baro_ok != 0U) && (lidar_ok != 0U) &&
            (FullESKF_Abs(baro_rel - lidar_rel) <=
             APP_FULL_ESKF_VERTICAL_DUAL_CONSISTENCY_M) &&
            (FullESKF_Abs(baro_rel - nominal_position[2]) <=
             APP_FULL_ESKF_VERTICAL_REACQUIRE_INNOV_M) &&
            (FullESKF_Abs(lidar_rel - nominal_position[2]) <=
             APP_FULL_ESKF_VERTICAL_REACQUIRE_INNOV_M))
        {
            vertical_reacquire_stable_count++;
        }
        else
        {
            vertical_reacquire_stable_count = 0UL;
        }

        eskf_data.vertical_reacquire_stable_count =
            vertical_reacquire_stable_count;

        if (vertical_reacquire_stable_count >=
            APP_FULL_ESKF_VERTICAL_REACQUIRE_STABLE_SAMPLES)
        {
            eskf_data.vertical_reacquire_active = 0U;
            eskf_data.output_inhibited = 0U;
            eskf_data.vertical_divergence_reason = 0U;
            vertical_reacquire_stable_count = 0UL;
            eskf_data.vertical_reacquire_stable_count = 0UL;
        }
    }
#else
    (void)barometer;
    (void)lidar;
    (void)now_us;
#endif
}

static void FullESKF_ProcessBarometer(
    const BarometerData_t *barometer,
    uint32_t now_us
)
{
    uint8_t fresh;

    fresh = FullESKF_IsBarometerFresh(barometer, now_us);
    eskf_data.baro_fresh = fresh;

    /*
     * Current usability and historical reference memory are deliberately
     * separate:
     *
     * baro_reference_locked:
     *     reference was successfully acquired in this ESKF session.
     *
     * baro_reference_ready:
     *     the locked reference is CURRENTLY usable because live barometer data
     *     is connected, valid and fresh.
     *
     * Therefore unplugging a barometer immediately drives reference_ready=0,
     * but a short reconnect does not redefine the altitude origin.
     */
    if (fresh == 0U)
    {
        if (eskf_data.baro_reference_ready != 0U)
        {
            eskf_data.baro_reference_invalidation_count++;
        }

        eskf_data.baro_reference_ready = 0U;
        last_baro_source_update_count = barometer->update_count;
        return;
    }

    if ((baro_reference_locked != 0U) &&
        (eskf_data.baro_reference_ready == 0U))
    {
        eskf_data.baro_reference_ready = 1U;
    }

    if (barometer->update_count == last_baro_source_update_count)
    {
        return;
    }

    last_baro_source_update_count = barometer->update_count;

    /* P53: while the barometer is deliberately following the stationary
     * preflight pressure datum, keep the ESKF's independent barometric origin
     * synchronized to that moving datum. Otherwise the sensor zero could be
     * corrected while the ESKF continues subtracting a stale pre-warmup
     * reference. Once PE9 separation freezes the barometer datum, this value
     * naturally freezes too. */
    if (barometer->ground_reference_tracking_active != 0U)
    {
        eskf_data.baro_reference_m = barometer->filtered_altitude_m;
        baro_reference_locked = 1U;
        eskf_data.baro_reference_ready = 1U;
    }

    if (baro_reference_locked == 0U)
    {
        baro_reference_sum += barometer->filtered_altitude_m;
        eskf_data.baro_reference_sample_count++;

        if (eskf_data.baro_reference_sample_count >=
            APP_FULL_ESKF_BARO_REFERENCE_SAMPLES)
        {
            eskf_data.baro_reference_m =
                baro_reference_sum /
                (float)eskf_data.baro_reference_sample_count;

            baro_reference_locked = 1U;
            eskf_data.baro_reference_ready = 1U;
        }

        return;
    }

    {
        float measurement_m =
            barometer->filtered_altitude_m - eskf_data.baro_reference_m;
        float innovation = measurement_m - nominal_position[2];
        uint8_t accepted;

        eskf_data.baro_innovation_m = innovation;

#if (APP_FULL_ESKF_SPARSE_SCALAR_UPDATES_ENABLED != 0U)
        accepted = FullESKF_ScalarUpdateUnitState(
            FULL_ESKF_PZ,
            innovation,
            APP_FULL_ESKF_BARO_STD_M * APP_FULL_ESKF_BARO_STD_M,
            APP_FULL_ESKF_BARO_GATE_SIGMA,
            APP_FULL_ESKF_BARO_ABS_GATE_M,
            FULL_ESKF_COV_STAGE_BARO
        );
#else
        {
            float h[FULL_ESKF_STATE_COUNT] = {0.0f};
            h[FULL_ESKF_PZ] = 1.0f;

            accepted = FullESKF_ScalarUpdate(
                h,
                innovation,
                APP_FULL_ESKF_BARO_STD_M * APP_FULL_ESKF_BARO_STD_M,
                APP_FULL_ESKF_BARO_GATE_SIGMA,
                APP_FULL_ESKF_BARO_ABS_GATE_M,
                FULL_ESKF_COV_STAGE_BARO
            );
        }
#endif

        if (accepted != 0U)
        {
            eskf_data.baro_update_count++;
            eskf_data.last_correction_timestamp_us = now_us;
        }
        else
        {
            eskf_data.baro_reject_count++;
        }
    }
}

/* -------------------------------------------------------------------------- */

static void FullESKF_ProcessLidar(
    const LidarData_t *lidar,
    uint32_t now_us
)
{
    float distance_m;
    float measurement_m;
    float innovation;
    uint8_t recovering_sample = 0U;

    eskf_data.lidar_fresh = FullESKF_IsLidarFresh(lidar, now_us);

    if (eskf_data.lidar_fresh == 0U)
    {
        last_lidar_source_update_count = lidar->update_count;
        return;
    }

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

    if ((distance_m < APP_FULL_ESKF_LIDAR_MIN_M) ||
        (distance_m > APP_FULL_ESKF_LIDAR_MAX_M))
    {
        eskf_data.lidar_reject_count++;
        return;
    }

#if (APP_FULL_ESKF_LIDAR_TILT_COMPENSATION_ENABLED != 0U)
    {
        float rotation[3][3];
        FullESKF_QuaternionToRotation(rotation);
        distance_m *= FullESKF_Abs(rotation[2][2]);
    }
#endif

    if (eskf_data.lidar_reference_ready == 0U)
    {
        if ((eskf_data.lidar_reference_sample_count > 0UL) &&
            (FullESKF_Abs(
                distance_m - lidar_reference_last_sample_m
             ) > APP_FULL_ESKF_LIDAR_REFERENCE_JUMP_M))
        {
            lidar_reference_sum = distance_m;
            eskf_data.lidar_reference_sample_count = 1UL;
        }
        else
        {
            lidar_reference_sum += distance_m;
            eskf_data.lidar_reference_sample_count++;
        }

        lidar_reference_last_sample_m = distance_m;

        if (eskf_data.lidar_reference_sample_count >=
            APP_FULL_ESKF_LIDAR_REFERENCE_SAMPLES)
        {
            eskf_data.lidar_reference_m =
                lidar_reference_sum /
                (float)eskf_data.lidar_reference_sample_count;
            eskf_data.lidar_reference_ready = 1U;

            lidar_last_accepted_distance_m =
                eskf_data.lidar_reference_m;
            lidar_last_accepted_valid = 1U;
            lidar_recovery_active = 0U;
            lidar_reacquire_count = 0UL;
        }

        return;
    }

    lidar_update_decimation_counter++;

    if (lidar_update_decimation_counter <
        APP_FULL_ESKF_LIDAR_UPDATE_DECIMATION)
    {
        return;
    }

    lidar_update_decimation_counter = 0UL;

    if (lidar_last_accepted_valid == 0U)
    {
        lidar_last_accepted_distance_m = distance_m;
        lidar_last_accepted_valid = 1U;
    }

    if (lidar_recovery_active == 0U)
    {
        if (FullESKF_Abs(
                distance_m - lidar_last_accepted_distance_m
            ) > APP_FULL_ESKF_LIDAR_MAX_SAMPLE_JUMP_M)
        {
            lidar_recovery_active = 1U;
            lidar_reacquire_candidate_m = distance_m;
            lidar_reacquire_count = 1UL;
            full_eskf_lidar_jump_reject_count++;
            eskf_data.lidar_reject_count++;
            return;
        }
    }
    else
    {
        recovering_sample = 1U;

        if (FullESKF_Abs(
                distance_m - lidar_reacquire_candidate_m
            ) <= APP_FULL_ESKF_LIDAR_REACQUIRE_CONSISTENCY_M)
        {
            lidar_reacquire_count++;
            lidar_reacquire_candidate_m +=
                (distance_m - lidar_reacquire_candidate_m) /
                (float)lidar_reacquire_count;
        }
        else
        {
            lidar_reacquire_candidate_m = distance_m;
            lidar_reacquire_count = 1UL;
        }

        if (lidar_reacquire_count <
            APP_FULL_ESKF_LIDAR_REACQUIRE_SAMPLES)
        {
            eskf_data.lidar_reject_count++;
            return;
        }

        distance_m = lidar_reacquire_candidate_m;
        measurement_m = distance_m - eskf_data.lidar_reference_m;
        innovation = measurement_m - nominal_position[2];

        if (FullESKF_Abs(innovation) >
            APP_FULL_ESKF_LIDAR_REACQUIRE_ABS_GATE_M)
        {
            eskf_data.lidar_innovation_m = innovation;
            eskf_data.lidar_reject_count++;
            return;
        }
    }

    measurement_m = distance_m - eskf_data.lidar_reference_m;
    innovation = measurement_m - nominal_position[2];
    eskf_data.lidar_innovation_m = innovation;

#if (APP_FULL_ESKF_LIDAR_INNOV_REACQUIRE_ENABLED != 0U)
    if (lidar_innov_reacquire_active == 0U)
    {
        float abs_innov = FullESKF_Abs(innovation);

        if ((abs_innov > APP_FULL_ESKF_LIDAR_ABS_GATE_M) &&
            (abs_innov <= APP_FULL_ESKF_LIDAR_INNOV_REACQUIRE_MAX_M))
        {
            lidar_innov_reacquire_confirm_count++;
            if (lidar_innov_reacquire_confirm_count <
                APP_FULL_ESKF_LIDAR_INNOV_REACQUIRE_CONFIRM_SAMPLES)
            {
                eskf_data.lidar_reject_count++;
                return;
            }

            lidar_innov_reacquire_active = 1U;
            lidar_innov_reacquire_confirm_count = 0UL;
            lidar_innov_reacquire_exit_count = 0UL;
            full_eskf_lidar_innov_reacquire_count++;
        }
        else
        {
            lidar_innov_reacquire_confirm_count = 0UL;
        }
    }

    if (lidar_innov_reacquire_active != 0U)
    {
        uint8_t accepted = FullESKF_ConstrainedAltitudeUpdate(
            innovation,
            APP_FULL_ESKF_LIDAR_STD_M * APP_FULL_ESKF_LIDAR_STD_M,
            APP_FULL_ESKF_LIDAR_GATE_SIGMA,
            APP_FULL_ESKF_LIDAR_INNOV_REACQUIRE_MAX_M,
            APP_FULL_ESKF_LIDAR_MAX_POSITION_STEP_M,
            APP_FULL_ESKF_LIDAR_MAX_VELOCITY_STEP_MPS,
            FULL_ESKF_COV_STAGE_LIDAR);

        if (accepted != 0U)
        {
            eskf_data.lidar_update_count++;
            eskf_data.last_correction_timestamp_us = now_us;
            lidar_last_accepted_distance_m = distance_m;
            lidar_last_accepted_valid = 1U;

            if (FullESKF_Abs(innovation) <=
                APP_FULL_ESKF_LIDAR_INNOV_REACQUIRE_EXIT_M)
            {
                lidar_innov_reacquire_exit_count++;
            }
            else
            {
                lidar_innov_reacquire_exit_count = 0UL;
            }

            if (lidar_innov_reacquire_exit_count >=
                APP_FULL_ESKF_LIDAR_INNOV_REACQUIRE_EXIT_SAMPLES)
            {
                lidar_innov_reacquire_active = 0U;
                lidar_innov_reacquire_exit_count = 0UL;
                full_eskf_lidar_innov_reacquire_success_count++;
            }

            if (recovering_sample != 0U)
            {
                lidar_recovery_active = 0U;
                lidar_reacquire_count = 0UL;
            }
        }
        else
        {
            eskf_data.lidar_reject_count++;
        }
        return;
    }
#endif

    if (FullESKF_ConstrainedAltitudeUpdate(
            innovation,
            APP_FULL_ESKF_LIDAR_STD_M * APP_FULL_ESKF_LIDAR_STD_M,
            APP_FULL_ESKF_LIDAR_GATE_SIGMA,
            APP_FULL_ESKF_LIDAR_ABS_GATE_M,
            APP_FULL_ESKF_LIDAR_MAX_POSITION_STEP_M,
            APP_FULL_ESKF_LIDAR_MAX_VELOCITY_STEP_MPS,
            FULL_ESKF_COV_STAGE_LIDAR) != 0U)
    {
        eskf_data.lidar_update_count++;
        eskf_data.last_correction_timestamp_us = now_us;
        lidar_last_accepted_distance_m = distance_m;
        lidar_last_accepted_valid = 1U;

        if (recovering_sample != 0U)
        {
            lidar_recovery_active = 0U;
            lidar_reacquire_count = 0UL;
        }
    }
    else
    {
        eskf_data.lidar_reject_count++;
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void FullStateESKF_Init(void)
{
    FullESKF_DWTProfilerInit();

    memset(&eskf_data, 0, sizeof(eskf_data));
    memset(nominal_position, 0, sizeof(nominal_position));
    memset(nominal_velocity, 0, sizeof(nominal_velocity));
    memset(nominal_accel_bias, 0, sizeof(nominal_accel_bias));
    memset(nominal_gyro_bias, 0, sizeof(nominal_gyro_bias));
    memset(covariance, 0, sizeof(covariance));
    memset(covariance_temp, 0, sizeof(covariance_temp));
    memset(covariance_last_good, 0, sizeof(covariance_last_good));
    covariance_last_good_valid = 0U;

#if (APP_FULL_ESKF_SPARSE_COVARIANCE_ENABLED == 0U)
    memset(transition, 0, sizeof(transition));
#endif

    nominal_quaternion[0] = 1.0f;
    nominal_quaternion[1] = 0.0f;
    nominal_quaternion[2] = 0.0f;
    nominal_quaternion[3] = 0.0f;

    last_baro_source_update_count = 0UL;
    last_lidar_source_update_count = 0UL;

    baro_reference_sum = 0.0f;
    baro_reference_locked = 0U;

    lidar_reference_sum = 0.0f;

    origin_zero_applied = 0U;
    memset(xy_diag_position_m, 0, sizeof(xy_diag_position_m));
    memset(xy_diag_velocity_mps, 0, sizeof(xy_diag_velocity_mps));
    memset(xy_diag_accel_bias_mps2, 0, sizeof(xy_diag_accel_bias_mps2));
    xy_diag_initialized = 0U;
    vibration_previous_norm_g = 1.0f;
    vibration_metric_ema_g = 0.0f;
    vibration_metric_initialized = 0U;
    eskf_data.vibration_r_multiplier = 1.0f;
    lidar_reference_last_sample_m = 0.0f;
    lidar_recovery_active = 0U;
    lidar_last_accepted_valid = 0U;
    lidar_reacquire_count = 0UL;
    lidar_last_accepted_distance_m = 0.0f;
    lidar_reacquire_candidate_m = 0.0f;
    lidar_innov_reacquire_active = 0U;
    lidar_innov_reacquire_confirm_count = 0UL;
    lidar_innov_reacquire_exit_count = 0UL;
    full_eskf_lidar_innov_reacquire_count = 0UL;
    full_eskf_lidar_innov_reacquire_success_count = 0UL;
    vertical_divergence_candidate_count = 0UL;
    vertical_reacquire_stable_count = 0UL;
    public_vertical_guard_armed = 0U;
    public_vertical_guard_z_m = 0.0f;
    public_vertical_guard_vz_mps = 0.0f;
    public_vertical_guard_timestamp_us = 0UL;
    covariance_sanitization_fault_pending = 0U;
    covariance_sanitization_fault_reason = FULL_ESKF_RESET_REASON_NONE;
    eskf_data.covariance_fault_stage = FULL_ESKF_COV_STAGE_NONE;
    eskf_data.covariance_fault_state_index = FULL_ESKF_COV_STATE_INVALID;
    eskf_data.covariance_fault_other_index = FULL_ESKF_COV_STATE_INVALID;
    eskf_data.covariance_fault_used_last_good = 0U;
    eskf_data.covariance_roundoff_last_stage = FULL_ESKF_COV_STAGE_NONE;
    eskf_data.covariance_roundoff_last_state_index = FULL_ESKF_COV_STATE_INVALID;
    eskf_data.output_inhibited = 0U;
    eskf_data.vertical_reacquire_active = 0U;
    eskf_data.vertical_divergence_reason = 0U;
    eskf_data.vertical_sensor_consistency_m = 0.0f;
    eskf_data.vertical_reacquire_target_m = 0.0f;
    live_debug_decimation_counter = 0UL;
    covariance_health_check_decimation_counter = 0UL;
    FullESKF_ResetCovarianceIntegritySlice();
    gravity_update_decimation_counter = 0UL;
    gravity_update_phase = FULL_ESKF_GRAVITY_PHASE_IDLE;
    gravity_batch_accepted_count = 0U;
    gravity_batch_measurement_variance = 0.0f;
    memset(gravity_batch_measured_up, 0, sizeof(gravity_batch_measured_up));
    memset(gravity_batch_predicted_up, 0, sizeof(gravity_batch_predicted_up));
    stationary_update_decimation_counter = 0UL;
    stationary_update_phase = FULL_ESKF_STATIONARY_PHASE_IDLE;
    lidar_update_decimation_counter = 0UL;
    stationary_confirmed_sample_count = 0UL;

    memset(stationary_gyro_lpf_dps, 0, sizeof(stationary_gyro_lpf_dps));

    gyro_bootstrap_done = 0U;
    gyro_bootstrap_attempt_count = 0UL;
    FullESKF_ResetGyroBootstrapAccumulator();

    full_eskf_gyro_bootstrap_active =
        (APP_FULL_ESKF_GYRO_BOOTSTRAP_ENABLED != 0U) ? 1U : 0U;
    full_eskf_gyro_bootstrap_done =
        (APP_FULL_ESKF_GYRO_BOOTSTRAP_ENABLED != 0U) ? 0U : 1U;
    full_eskf_gyro_bootstrap_attempt_count = 0UL;
    full_eskf_gyro_bootstrap_quality_score = 0UL;
    full_eskf_stationary_gyro_lpf_norm_dps = 0.0f;
    full_eskf_stationary_confirmed_sample_count = 0UL;
    full_eskf_lidar_recovery_active = 0U;
    full_eskf_lidar_reacquire_count = 0UL;
    full_eskf_lidar_jump_reject_count = 0UL;

    full_eskf_horizontal_position_valid = 0U;
    full_eskf_vertical_position_valid = 0U;
    full_eskf_baro_fresh = 0U;
    full_eskf_lidar_fresh = 0U;

    full_eskf_origin_zeroed = 0U;
    full_eskf_origin_zero_count = 0UL;
    full_eskf_origin_zero_timestamp_us = 0UL;
    full_eskf_origin_pre_position_x_m = 0.0f;
    full_eskf_origin_pre_position_y_m = 0.0f;
    full_eskf_origin_pre_position_z_m = 0.0f;

    full_eskf_baro_sample_age_us = 0xFFFFFFFFUL;
    full_eskf_lidar_sample_age_us = 0xFFFFFFFFUL;
    full_eskf_baro_reference_invalidation_count = 0UL;
    full_eskf_last_public_output_timestamp_us = 0UL;

    full_eskf_last_predict_cycles = 0UL;
    full_eskf_max_predict_cycles = 0UL;
    full_eskf_total_predict_cycles = 0UL;
    full_eskf_last_public_output_cycles = 0UL;
    full_eskf_max_public_output_cycles = 0UL;
    full_eskf_total_public_output_cycles = 0UL;
    full_eskf_last_correction_cycles = 0UL;
    full_eskf_max_correction_cycles = 0UL;
    full_eskf_total_correction_cycles = 0UL;
    full_eskf_last_covariance_cycles = 0UL;
    full_eskf_max_covariance_cycles = 0UL;
    full_eskf_total_covariance_cycles = 0UL;
    full_eskf_last_gravity_cycles = 0UL;
    full_eskf_max_gravity_cycles = 0UL;
    full_eskf_total_gravity_cycles = 0UL;
    full_eskf_last_stationary_cycles = 0UL;
    full_eskf_max_stationary_cycles = 0UL;
    full_eskf_total_stationary_cycles = 0UL;
    full_eskf_last_baro_cycles = 0UL;
    full_eskf_max_baro_cycles = 0UL;
    full_eskf_total_baro_cycles = 0UL;
    full_eskf_last_lidar_cycles = 0UL;
    full_eskf_max_lidar_cycles = 0UL;
    full_eskf_total_lidar_cycles = 0UL;

    FullESKF_ClearCovarianceAccumulator();

#if (APP_FULL_ESKF_ENABLED != 0U)
    eskf_data.enabled = 1U;
#endif

#if (APP_FULL_ESKF_SHADOW_MODE != 0U)
    eskf_data.shadow_mode = 1U;
#endif

    FullESKF_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */

void FullStateESKF_Reset(void)
{
    FullESKF_SafeReinitialize(FULL_ESKF_RESET_REASON_MANUAL);
}

/* -------------------------------------------------------------------------- */

void FullStateESKF_Predict(void)
{
#if (APP_FULL_ESKF_ENABLED != 0U)
    uint32_t start_us = micros();
    uint32_t predict_start_cycles = FullESKF_DWTStart();
    const SensorData_t *sensor = SensorManager_GetDataPtr();
    const AttitudeEstimatorData_t *attitude = AttitudeEstimator_GetDataPtr();
    uint32_t now_us = micros();

    if ((sensor->imu_valid == 0U) ||
        (sensor->accel_filtered_norm_g <= 0.0f))
    {
        if ((eskf_data.last_predict_timestamp_us != 0UL) &&
            ((now_us - eskf_data.last_predict_timestamp_us) >
             APP_FULL_ESKF_STALE_TIMEOUT_US))
        {
            eskf_data.healthy = 0U;
        }

        goto predict_exit;
    }

    if (sensor->update_count == eskf_data.last_sensor_update_count)
    {
        eskf_data.duplicate_skip_count++;
        goto predict_exit;
    }

    eskf_data.last_sensor_update_count = sensor->update_count;

    if (eskf_data.initialized == 0U)
    {
        if ((attitude->initialized != 0U) &&
            (attitude->healthy != 0U))
        {
            FullESKF_InitializeFromAttitude(attitude, sensor->timestamp_us);
        }

        goto predict_exit;
    }

    uint32_t dt_us = sensor->timestamp_us - eskf_data.last_predict_timestamp_us;
    eskf_data.last_dt_us = dt_us;

    if (dt_us > eskf_data.max_dt_us)
    {
        eskf_data.max_dt_us = dt_us;
    }

    if ((dt_us == 0UL) ||
        (dt_us > APP_FULL_ESKF_MAX_PREDICT_GAP_US))
    {
        eskf_data.gap_skip_count++;
        eskf_data.last_predict_timestamp_us = sensor->timestamp_us;
        FullESKF_ClearCovarianceAccumulator();
        FullESKF_InflateAfterGap();

        if ((attitude->initialized != 0U) &&
            (attitude->healthy != 0U))
        {
            nominal_quaternion[0] = attitude->q_w;
            nominal_quaternion[1] = attitude->q_x;
            nominal_quaternion[2] = attitude->q_y;
            nominal_quaternion[3] = attitude->q_z;
            (void)FullESKF_NormalizeQuaternion();
        }

        goto predict_health;
    }

    float dt_s = (float)dt_us * 0.000001f;

    float angular_rate_body[3];
    angular_rate_body[0] =
        sensor->gyro_x_filtered_dps * FULL_ESKF_DEG_TO_RAD -
        nominal_gyro_bias[0];
    angular_rate_body[1] =
        sensor->gyro_y_filtered_dps * FULL_ESKF_DEG_TO_RAD -
        nominal_gyro_bias[1];
    angular_rate_body[2] =
        sensor->gyro_z_filtered_dps * FULL_ESKF_DEG_TO_RAD -
        nominal_gyro_bias[2];

    float specific_force_body[3];
    specific_force_body[0] =
        sensor->accel_x_filtered_g * APP_ATTITUDE_GRAVITY_MPS2 -
        nominal_accel_bias[0];
    specific_force_body[1] =
        sensor->accel_y_filtered_g * APP_ATTITUDE_GRAVITY_MPS2 -
        nominal_accel_bias[1];
    specific_force_body[2] =
        sensor->accel_z_filtered_g * APP_ATTITUDE_GRAVITY_MPS2 -
        nominal_accel_bias[2];

    float qw = nominal_quaternion[0];
    float qx = nominal_quaternion[1];
    float qy = nominal_quaternion[2];
    float qz = nominal_quaternion[3];
    float half_dt = 0.5f * dt_s;

    nominal_quaternion[0] +=
        (-qx * angular_rate_body[0] -
         qy * angular_rate_body[1] -
         qz * angular_rate_body[2]) * half_dt;

    nominal_quaternion[1] +=
        (qw * angular_rate_body[0] +
         qy * angular_rate_body[2] -
         qz * angular_rate_body[1]) * half_dt;

    nominal_quaternion[2] +=
        (qw * angular_rate_body[1] -
         qx * angular_rate_body[2] +
         qz * angular_rate_body[0]) * half_dt;

    nominal_quaternion[3] +=
        (qw * angular_rate_body[2] +
         qx * angular_rate_body[1] -
         qy * angular_rate_body[0]) * half_dt;

    if (FullESKF_NormalizeQuaternion() == 0U)
    {
        eskf_data.numerical_error_count++;
        eskf_data.healthy = 0U;
        goto predict_exit;
    }

    float rotation[3][3];
    FullESKF_QuaternionToRotation(rotation);

    float world_accel[3];
    world_accel[0] =
        (rotation[0][0] * specific_force_body[0]) +
        (rotation[0][1] * specific_force_body[1]) +
        (rotation[0][2] * specific_force_body[2]);

    world_accel[1] =
        (rotation[1][0] * specific_force_body[0]) +
        (rotation[1][1] * specific_force_body[1]) +
        (rotation[1][2] * specific_force_body[2]);

    world_accel[2] =
        (rotation[2][0] * specific_force_body[0]) +
        (rotation[2][1] * specific_force_body[1]) +
        (rotation[2][2] * specific_force_body[2]) -
        APP_ATTITUDE_GRAVITY_MPS2;

    /* Update only the unaided public/logging X/Y diagnostic projection.
     * Z and the nominal 15-state ESKF propagation below remain byte-for-byte
     * equivalent in behaviour. */
    FullESKF_UpdateXYDiagnosticProjection(
        world_accel[0], world_accel[1], dt_s);

    float half_dt_squared = 0.5f * dt_s * dt_s;
    uint32_t axis;

    for (axis = 0UL; axis < 3UL; axis++)
    {
        nominal_position[axis] +=
            (nominal_velocity[axis] * dt_s) +
            (world_accel[axis] * half_dt_squared);

        nominal_velocity[axis] += world_accel[axis] * dt_s;

        covariance_accel_integral[axis] +=
            specific_force_body[axis] * dt_s;

        covariance_gyro_integral[axis] +=
            angular_rate_body[axis] * dt_s;
    }

    covariance_dt_accum_s += dt_s;
    covariance_sample_counter++;

    eskf_data.world_linear_accel_x_mps2 = world_accel[0];
    eskf_data.world_linear_accel_y_mps2 = world_accel[1];
    eskf_data.world_linear_accel_z_mps2 = world_accel[2];

    eskf_data.last_predict_timestamp_us = sensor->timestamp_us;
    eskf_data.predict_count++;

predict_health:
    /* Keep the 1 kHz path light. Routine health checks and public commits are
     * owned by the dedicated 200 Hz correction task. */
    if ((eskf_data.healthy == 0U) && (FullESKF_CheckNumericalHealth() != 0U))
    {
        eskf_data.healthy = 1U;
    }

predict_exit:
    eskf_data.last_predict_exec_us = micros() - start_us;

    if (eskf_data.last_predict_exec_us > eskf_data.max_predict_exec_us)
    {
        eskf_data.max_predict_exec_us = eskf_data.last_predict_exec_us;
    }

    FullESKF_ProfileStore(
        &full_eskf_last_predict_cycles,
        &full_eskf_max_predict_cycles,
        &full_eskf_total_predict_cycles,
        FullESKF_DWTElapsed(predict_start_cycles)
    );
#else
    eskf_data.enabled = 0U;
    eskf_data.healthy = 0U;
    FullESKF_UpdateLiveDebug();
#endif
}

/* -------------------------------------------------------------------------- */


uint8_t FullStateESKF_ServiceCovariance(void)
{
#if (APP_FULL_ESKF_ENABLED != 0U)
    if ((eskf_data.initialized == 0U) ||
        (eskf_data.healthy == 0U) ||
        (covariance_sample_counter < APP_FULL_ESKF_COVARIANCE_DECIMATION) ||
        (covariance_dt_accum_s <= 0.0f))
    {
        return 0U;
    }

    {
        float average_specific_force[3];
        float average_angular_rate[3];
        uint32_t axis;

        for (axis = 0UL; axis < 3UL; axis++)
        {
            average_specific_force[axis] =
                covariance_accel_integral[axis] / covariance_dt_accum_s;
            average_angular_rate[axis] =
                covariance_gyro_integral[axis] / covariance_dt_accum_s;
        }

        FullESKF_PropagateCovariance(
            covariance_dt_accum_s,
            average_specific_force,
            average_angular_rate
        );
        FullESKF_ClearCovarianceAccumulator();

        {
            uint8_t covariance_reason =
                FullESKF_CheckCovarianceDiagonalFast();

            if (covariance_reason != FULL_ESKF_RESET_REASON_NONE)
            {
                FullESKF_RecoverCovarianceOnly(covariance_reason);
                FullESKF_ResetCovarianceIntegritySlice();
                return 0U;
            }

            /* Start a new bounded full-integrity sweep from the freshly
             * propagated covariance. The sweep itself runs row-by-row in the
             * 200 Hz correction path, never as one >1 ms monolithic task. */
            FullESKF_ResetCovarianceIntegritySlice();
        }
    }

    return 1U;
#else
    return 0U;
#endif
}

void FullStateESKF_CorrectMeasurements(void)
{
#if (APP_FULL_ESKF_ENABLED != 0U)
    uint32_t start_us = micros();
    uint32_t correction_start_cycles = FullESKF_DWTStart();
    uint32_t now_us = start_us;

    if (eskf_data.initialized == 0U)
    {
        goto correction_exit;
    }

    /* P45: if prediction/injection already marked the nominal state unhealthy,
     * do not leave the filter permanently latched unhealthy. Numerical failures
     * have already incremented numerical_error_count at the detection site; here
     * we only start the existing attitude-backed non-blocking reacquisition. */
    if (eskf_data.healthy == 0U)
    {
        uint8_t covariance_reason;
        covariance_health_check_decimation_counter = 0UL;
        covariance_reason = FullESKF_CheckCovarianceIntegrity();

        if (FullESKF_CheckNumericalHealth() == 0U)
        {
            FullESKF_SafeReinitialize(FULL_ESKF_RESET_REASON_STATE_NUMERICAL);
        }
        else if (covariance_reason != FULL_ESKF_RESET_REASON_NONE)
        {
            FullESKF_RecoverCovarianceOnly(covariance_reason);
        }

        goto correction_exit;
    }

    const SensorData_t *sensor = SensorManager_GetDataPtr();
    const BarometerData_t *barometer = Barometer_GetDataPtr();
    const LidarData_t *lidar = Lidar_GetDataPtr();

    /* P34: the 25 Hz 15x15 covariance propagation is serviced separately in
     * a scheduler-slack window. This keeps the native 200 Hz public correction
     * path bounded and prevents covariance work from stealing the next 1 kHz
     * IMU release. */

    if ((sensor->imu_valid != 0U) &&
        ((now_us - sensor->timestamp_us) <= APP_FULL_ESKF_IMU_STALE_TIMEOUT_US))
    {
        uint32_t stationary_cycles;
        uint32_t section_start_cycles = FullESKF_DWTStart();

        FullESKF_ProcessGyroBiasBootstrap(sensor);
        FullESKF_UpdateStationaryDetector(sensor);

        stationary_cycles = FullESKF_DWTElapsed(section_start_cycles);

        /* R8R8: keep a 50 Hz gravity-batch launch cadence, but execute only
         * one X/Y/Z sparse Joseph slice per 200 Hz correction.  The cadence
         * counter keeps advancing while a 3-slot batch is active, so the next
         * batch still starts 20 ms after the previous one. */
        gravity_update_decimation_counter++;

        if (gyro_bootstrap_done != 0U)
        {
            uint8_t run_gravity_slice = 0U;

            if (gravity_update_phase != FULL_ESKF_GRAVITY_PHASE_IDLE)
            {
                run_gravity_slice = 1U;
            }
            else if (gravity_update_decimation_counter >=
                     APP_FULL_ESKF_GRAVITY_UPDATE_DECIMATION)
            {
                gravity_update_decimation_counter = 0UL;
                run_gravity_slice = 1U;
            }

            if (run_gravity_slice != 0U)
            {
                section_start_cycles = FullESKF_DWTStart();
                FullESKF_ProcessGravity(sensor);

                FullESKF_ProfileStore(
                    &full_eskf_last_gravity_cycles,
                    &full_eskf_max_gravity_cycles,
                    &full_eskf_total_gravity_cycles,
                    FullESKF_DWTElapsed(section_start_cycles)
                );
            }
            else
            {
                full_eskf_last_gravity_cycles = 0UL;
            }
        }
        else
        {
            /* Before gyro bootstrap is complete, gravity aiding remains fully
             * paused and any half-started diagnostic batch is discarded. */
            gravity_update_phase = FULL_ESKF_GRAVITY_PHASE_IDLE;
            gravity_batch_accepted_count = 0U;
            eskf_data.gravity_correction_active = 0U;
            eskf_data.gravity_correction_weight = 0.0f;
            full_eskf_last_gravity_cycles = 0UL;
        }

        section_start_cycles = FullESKF_DWTStart();
        FullESKF_ProcessStationaryUpdates(sensor);
        stationary_cycles += FullESKF_DWTElapsed(section_start_cycles);

        FullESKF_ProfileStore(
            &full_eskf_last_stationary_cycles,
            &full_eskf_max_stationary_cycles,
            &full_eskf_total_stationary_cycles,
            stationary_cycles
        );
    }
    else
    {
        eskf_data.gravity_correction_active = 0U;
        eskf_data.stationary_detected = 0U;
        eskf_data.stationary_sample_count = 0UL;
        stationary_confirmed_sample_count = 0UL;
        full_eskf_stationary_confirmed_sample_count = 0UL;
        gravity_update_decimation_counter = 0UL;
        gravity_update_phase = FULL_ESKF_GRAVITY_PHASE_IDLE;
        gravity_batch_accepted_count = 0U;
    }

    /*
     * Refresh current sensor usability before the one-shot origin decision.
     * The actual measurement-processing functions perform the same checks
     * again before consuming a new sample.
     */
    eskf_data.baro_fresh =
        FullESKF_IsBarometerFresh(barometer, now_us);
    eskf_data.lidar_fresh =
        FullESKF_IsLidarFresh(lidar, now_us);

    if (eskf_data.baro_fresh == 0U)
    {
        if (eskf_data.baro_reference_ready != 0U)
        {
            eskf_data.baro_reference_invalidation_count++;
        }
        eskf_data.baro_reference_ready = 0U;
    }
    else if (baro_reference_locked != 0U)
    {
        eskf_data.baro_reference_ready = 1U;
    }

    FullESKF_UpdateNavigationValidity();
    FullESKF_ApplyStartupOriginIfReady(now_us);

    /* P37: contain a runaway vertical state before the normal innovation
     * gates can lock both absolute sensors out forever. */
    FullESKF_ServiceVerticalDivergence(barometer, lidar, now_us);
    FullESKF_UpdateNavigationValidity();

    {
        uint32_t section_start_cycles = FullESKF_DWTStart();

        FullESKF_ProcessBarometer(barometer, now_us);

        FullESKF_ProfileStore(
            &full_eskf_last_baro_cycles,
            &full_eskf_max_baro_cycles,
            &full_eskf_total_baro_cycles,
            FullESKF_DWTElapsed(section_start_cycles)
        );
    }

    {
        uint32_t section_start_cycles = FullESKF_DWTStart();

        FullESKF_ProcessLidar(lidar, now_us);

        FullESKF_ProfileStore(
            &full_eskf_last_lidar_cycles,
            &full_eskf_max_lidar_cycles,
            &full_eskf_total_lidar_cycles,
            FullESKF_DWTElapsed(section_start_cycles)
        );
    }

    FullESKF_UpdateNavigationValidity();
    FullESKF_ServiceXYRunawayGuard();

    /* P45: no corrupted state/covariance may cross the 200 Hz public boundary. */
    if (FullESKF_CheckNumericalHealth() == 0U)
    {
        eskf_data.healthy = 0U;
        eskf_data.numerical_error_count++;
        FullESKF_SafeReinitialize(
            FULL_ESKF_RESET_REASON_STATE_NUMERICAL);
        goto correction_exit;
    }

    /* P112R12R8R6 timing containment: every 200 Hz correction gets a cheap
     * diagonal/non-finite guard, while the complete symmetry/integrity matrix
     * is scanned incrementally. This preserves fault detection without a
     * single cooperative call consuming an entire 1 ms IMU period. */
    {
        uint8_t covariance_reason = FullESKF_CheckCovarianceDiagonalFast();

        if (covariance_reason == FULL_ESKF_RESET_REASON_NONE)
        {
            covariance_reason = FullESKF_ServiceCovarianceIntegritySlice();
        }

        if (covariance_reason != FULL_ESKF_RESET_REASON_NONE)
        {
            eskf_data.healthy = 0U;
            FullESKF_RecoverCovarianceOnly(covariance_reason);
            goto correction_exit;
        }
    }

    if (eskf_data.healthy != 0U)
    {
        uint32_t public_start_cycles = FullESKF_DWTStart();
        uint32_t public_now_us = micros();
        uint8_t publish_guard_reason =
            FullESKF_CheckPublicVerticalGuard(public_now_us);

        if (publish_guard_reason != FULL_ESKF_RESET_REASON_NONE)
        {
            eskf_data.public_output_reject_count++;
            if ((publish_guard_reason == FULL_ESKF_RESET_REASON_PUBLIC_Z_JUMP) ||
                (publish_guard_reason == FULL_ESKF_RESET_REASON_PUBLIC_Z_VZ_JUMP))
            {
                eskf_data.public_z_jump_reject_count++;
            }
            if ((publish_guard_reason == FULL_ESKF_RESET_REASON_PUBLIC_VZ_JUMP) ||
                (publish_guard_reason == FULL_ESKF_RESET_REASON_PUBLIC_Z_VZ_JUMP))
            {
                eskf_data.public_vz_jump_reject_count++;
            }
            eskf_data.numerical_error_count++;
            FullESKF_SafeReinitialize(publish_guard_reason);
            goto correction_exit;
        }

        FullESKF_UpdateEulerAngles();
        FullESKF_UpdatePublicState();
        eskf_data.last_public_output_timestamp_us = public_now_us;
        eskf_data.public_output_count++;
        FullESKF_CommitPublicVerticalGuard(public_now_us);
        full_eskf_public_output_count = eskf_data.public_output_count;
        full_eskf_last_public_output_timestamp_us = eskf_data.last_public_output_timestamp_us;
        live_debug_decimation_counter++;
        if (live_debug_decimation_counter >= APP_FULL_ESKF_LIVE_DEBUG_DECIMATION)
        {
            live_debug_decimation_counter = 0UL;
            FullESKF_UpdateLiveDebug();
        }
        FullESKF_ProfileStore(&full_eskf_last_public_output_cycles,
                              &full_eskf_max_public_output_cycles,
                              &full_eskf_total_public_output_cycles,
                              FullESKF_DWTElapsed(public_start_cycles));
    }

correction_exit:
    eskf_data.last_correction_exec_us = micros() - start_us;

    if (eskf_data.last_correction_exec_us >
        eskf_data.max_correction_exec_us)
    {
        eskf_data.max_correction_exec_us =
            eskf_data.last_correction_exec_us;
    }

    FullESKF_ProfileStore(
        &full_eskf_last_correction_cycles,
        &full_eskf_max_correction_cycles,
        &full_eskf_total_correction_cycles,
        FullESKF_DWTElapsed(correction_start_cycles)
    );

    /* Public/live state is committed above by this 200 Hz correction slot. */
#else
    eskf_data.enabled = 0U;
    eskf_data.healthy = 0U;
    FullESKF_UpdateLiveDebug();
#endif
}

/* -------------------------------------------------------------------------- */

uint8_t FullStateESKF_IsInitialized(void)
{
    return eskf_data.initialized;
}

/* -------------------------------------------------------------------------- */

uint8_t FullStateESKF_IsLidarInnovationReacquireActive(void)
{
    return lidar_innov_reacquire_active;
}

uint32_t FullStateESKF_GetLidarInnovationReacquireCount(void)
{
    return full_eskf_lidar_innov_reacquire_count;
}

uint32_t FullStateESKF_GetLidarInnovationReacquireSuccessCount(void)
{
    return full_eskf_lidar_innov_reacquire_success_count;
}

uint8_t FullStateESKF_IsHealthy(void)
{
    return eskf_data.healthy;
}

/* -------------------------------------------------------------------------- */

FullStateESKFData_t FullStateESKF_GetData(void)
{
    return eskf_data;
}

const FullStateESKFData_t *FullStateESKF_GetDataPtr(void)
{
    return &eskf_data;
}
