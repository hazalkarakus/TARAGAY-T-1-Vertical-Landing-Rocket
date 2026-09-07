#ifndef APP_MODULES_ESTIMATION_FULL_STATE_ESKF_H
#define APP_MODULES_ESTIMATION_FULL_STATE_ESKF_H

#include <stdint.h>

/* P45 reset/reacquire reason codes exposed in UART diagnostics. */
#define FULL_ESKF_RESET_REASON_NONE                    0U
#define FULL_ESKF_RESET_REASON_MANUAL                  1U
#define FULL_ESKF_RESET_REASON_STATE_NUMERICAL         2U
#define FULL_ESKF_RESET_REASON_COV_NONFINITE           3U
#define FULL_ESKF_RESET_REASON_COV_DIAGONAL            4U
#define FULL_ESKF_RESET_REASON_COV_ASYMMETRY           5U
#define FULL_ESKF_RESET_REASON_PUBLIC_Z_JUMP            6U
#define FULL_ESKF_RESET_REASON_PUBLIC_VZ_JUMP           7U
#define FULL_ESKF_RESET_REASON_PUBLIC_Z_VZ_JUMP         8U

/* P46 covariance root-cause operation stages exposed in UART diagnostics. */
#define FULL_ESKF_COV_STAGE_NONE                        0U
#define FULL_ESKF_COV_STAGE_PROPAGATE                   1U
#define FULL_ESKF_COV_STAGE_GRAVITY                     2U
#define FULL_ESKF_COV_STAGE_ZUPT                        3U
#define FULL_ESKF_COV_STAGE_GYRO_BIAS                   4U
#define FULL_ESKF_COV_STAGE_ACCEL_BIAS                  5U
#define FULL_ESKF_COV_STAGE_BARO                        6U
#define FULL_ESKF_COV_STAGE_LIDAR                       7U
#define FULL_ESKF_COV_STAGE_INTEGRITY                   8U
#define FULL_ESKF_COV_STAGE_GAP_INFLATE                 9U
#define FULL_ESKF_COV_STATE_INVALID                     255U

typedef struct
{
    uint8_t enabled;
    uint8_t shadow_mode;
    uint8_t initialized;
    uint8_t healthy;

    uint8_t baro_reference_ready;
    uint8_t lidar_reference_ready;
    uint8_t gravity_correction_active;
    uint8_t stationary_detected;

    /*
     * Navigation validity.
     * No horizontal absolute aiding exists yet, therefore X/Y position is
     * intentionally reported INVALID even though the inertial state is still
     * propagated for diagnostics.
     */
    uint8_t horizontal_position_valid;
    uint8_t vertical_position_valid;
    uint8_t baro_fresh;
    uint8_t lidar_fresh;

    /* One-shot startup coordinate-origin alignment status. */
    uint8_t origin_zeroed;
    uint8_t output_inhibited;
    uint8_t vertical_reacquire_active;
    uint8_t vertical_divergence_reason;

    /* P45 estimator integrity/reacquire diagnostics. */
    uint8_t reset_reason;
    uint8_t covariance_integrity_ok;

    /* World frame: ENU-style, +Z upward. */
    float position_x_m;
    float position_y_m;
    float position_z_m;

    float velocity_x_mps;
    float velocity_y_mps;
    float velocity_z_mps;

    /* Quaternion rotates body-frame vectors into the world frame. */
    float q_w;
    float q_x;
    float q_y;
    float q_z;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    float accel_bias_x_mps2;
    float accel_bias_y_mps2;
    float accel_bias_z_mps2;

    float gyro_bias_x_rad_s;
    float gyro_bias_y_rad_s;
    float gyro_bias_z_rad_s;

    float gyro_bias_x_dps;
    float gyro_bias_y_dps;
    float gyro_bias_z_dps;

    float world_linear_accel_x_mps2;
    float world_linear_accel_y_mps2;
    float world_linear_accel_z_mps2;

    float baro_reference_m;
    float lidar_reference_m;
    float baro_innovation_m;
    float lidar_innovation_m;
    float vertical_sensor_consistency_m;
    float vertical_reacquire_target_m;

    float origin_pre_position_x_m;
    float origin_pre_position_y_m;
    float origin_pre_position_z_m;

    float gravity_correction_weight;
    float vibration_metric_g;
    float vibration_r_multiplier;

    float covariance_position_x;
    float covariance_position_y;
    float covariance_position_z;

    float covariance_velocity_x;
    float covariance_velocity_y;
    float covariance_velocity_z;

    float covariance_roll_rad2;
    float covariance_pitch_rad2;
    float covariance_yaw_rad2;

    float covariance_accel_bias_x;
    float covariance_accel_bias_y;
    float covariance_accel_bias_z;

    float covariance_gyro_bias_x;
    float covariance_gyro_bias_y;
    float covariance_gyro_bias_z;

    /* P45/P46 full-matrix covariance diagnostics. */
    float covariance_diag_min;
    float covariance_diag_max;
    float covariance_symmetry_error_max;

    /* P46: first raw fault captured before any clamp/rollback. */
    uint8_t covariance_fault_stage;
    uint8_t covariance_fault_state_index;
    uint8_t covariance_fault_other_index;
    uint8_t covariance_fault_used_last_good;
    float covariance_fault_raw_value;
    float covariance_fault_aux_value;
    uint32_t covariance_fault_timestamp_us;

    /* P46: most recent tiny negative diagonal treated as float roundoff. */
    uint8_t covariance_roundoff_last_stage;
    uint8_t covariance_roundoff_last_state_index;
    float covariance_roundoff_last_raw_value;
    uint32_t covariance_roundoff_last_timestamp_us;

    uint32_t predict_count;
    uint32_t public_output_count;
    uint32_t covariance_predict_count;
    uint32_t gravity_update_count;
    /* P48: gravity uses an attitude-only sparse Joseph update; these counters
     * keep the P47 telemetry names for wire compatibility. */
    uint32_t gravity_joseph_update_count;
    uint32_t gravity_joseph_fault_count;
    /* P49: ZUPT uses a masked full-state Joseph update with gyro-bias gain = 0. */
    uint32_t zupt_joseph_update_count;
    uint32_t zupt_joseph_fault_count;
    uint32_t stationary_detector_count;
    uint32_t stationary_update_count;
    uint32_t baro_update_count;
    uint32_t lidar_update_count;

    uint32_t baro_reject_count;
    uint32_t lidar_reject_count;
    uint32_t gravity_reject_count;

    uint32_t duplicate_skip_count;
    uint32_t gap_skip_count;
    uint32_t reset_count;
    uint32_t numerical_error_count;

    /* P45 cumulative estimator integrity counters. */
    uint32_t public_output_reject_count;
    uint32_t public_z_jump_reject_count;
    uint32_t public_vz_jump_reject_count;
    uint32_t covariance_integrity_check_count;
    uint32_t covariance_fault_count;
    uint32_t covariance_reinit_count;
    uint32_t covariance_state_preserving_recovery_count;
    uint32_t covariance_roundoff_clamp_count;

    uint32_t baro_reference_sample_count;
    uint32_t lidar_reference_sample_count;
    uint32_t stationary_sample_count;

    uint32_t baro_reference_invalidation_count;
    uint32_t vertical_divergence_count;
    uint32_t vertical_reacquire_count;
    uint32_t vertical_divergence_candidate_count;
    uint32_t vertical_reacquire_stable_count;
    uint32_t origin_zero_count;
    uint32_t origin_zero_timestamp_us;

    uint32_t last_sensor_update_count;
    uint32_t last_predict_timestamp_us;
    uint32_t last_public_output_timestamp_us;
    uint32_t last_correction_timestamp_us;
    uint32_t last_dt_us;
    uint32_t max_dt_us;

    uint32_t last_predict_exec_us;
    uint32_t max_predict_exec_us;
    uint32_t last_correction_exec_us;
    uint32_t max_correction_exec_us;

} FullStateESKFData_t;

/* DWT profiler values exported for the SD logger and Live Expressions. */
extern volatile uint8_t full_eskf_dwt_profiler_enabled;
extern volatile uint32_t full_eskf_cpu_clock_hz;

extern volatile uint32_t full_eskf_last_predict_cycles;
extern volatile uint32_t full_eskf_max_predict_cycles;
extern volatile uint32_t full_eskf_total_predict_cycles;
extern volatile uint32_t full_eskf_last_public_output_cycles;
extern volatile uint32_t full_eskf_max_public_output_cycles;
extern volatile uint32_t full_eskf_total_public_output_cycles;
extern volatile uint32_t full_eskf_last_correction_cycles;
extern volatile uint32_t full_eskf_max_correction_cycles;
extern volatile uint32_t full_eskf_total_correction_cycles;
extern volatile uint32_t full_eskf_last_covariance_cycles;
extern volatile uint32_t full_eskf_max_covariance_cycles;
extern volatile uint32_t full_eskf_total_covariance_cycles;
extern volatile uint32_t full_eskf_last_gravity_cycles;
extern volatile uint32_t full_eskf_max_gravity_cycles;
extern volatile uint32_t full_eskf_total_gravity_cycles;
extern volatile uint32_t full_eskf_last_stationary_cycles;
extern volatile uint32_t full_eskf_max_stationary_cycles;
extern volatile uint32_t full_eskf_total_stationary_cycles;
extern volatile uint32_t full_eskf_last_baro_cycles;
extern volatile uint32_t full_eskf_max_baro_cycles;
extern volatile uint32_t full_eskf_total_baro_cycles;
extern volatile uint32_t full_eskf_last_lidar_cycles;
extern volatile uint32_t full_eskf_max_lidar_cycles;
extern volatile uint32_t full_eskf_total_lidar_cycles;

void FullStateESKF_Init(void);
void FullStateESKF_Predict(void);
void FullStateESKF_CorrectMeasurements(void);
uint8_t FullStateESKF_ServiceCovariance(void);
void FullStateESKF_Reset(void);

uint8_t FullStateESKF_IsInitialized(void);
uint8_t FullStateESKF_IsHealthy(void);
uint8_t FullStateESKF_IsLidarInnovationReacquireActive(void);
uint32_t FullStateESKF_GetLidarInnovationReacquireCount(void);
uint32_t FullStateESKF_GetLidarInnovationReacquireSuccessCount(void);
FullStateESKFData_t FullStateESKF_GetData(void);
const FullStateESKFData_t *FullStateESKF_GetDataPtr(void);

#endif
