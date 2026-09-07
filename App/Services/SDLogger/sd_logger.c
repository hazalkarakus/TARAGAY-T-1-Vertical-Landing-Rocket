#include "Services/SDLogger/sd_logger.h"

#include "Common/app_config.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"
#include "Modules/Sensors/IMU/imu.h"
#include "Modules/Sensors/Barometer/barometer.h"
#include "Modules/Sensors/Lidar/Lidar.h"
#include "Modules/Control/NeedleValve/needle_valve_controller.h"
#include "Modules/Control/TaragayFlightLogic/taragay_flight_logic.h"
#include "Services/SolenoidOutput/solenoid_output.h"
#include "Services/NeedleValveIntegrationTest/needle_valve_integration_test.h"
#include "Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.h"
#include "Services/NeedleValveHardware/needle_valve_hw.h"
#include "Modules/RemoteControl/remote_control.h"
#include "Modules/NRF24/nrf24.h"
#include "Services/ServoOutput/servo_output.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"
#include "Services/SystemMonitor/system_monitor.h"
#include "Services/Timebase/timebase.h"
#include "Services/PreflightTrigger/preflight_trigger.h"
#include "Services/SensorQualification/sensor_qualification.h"

#include "main.h"
#include "fatfs.h"
#include "sd_diskio.h"
#include "diskio.h"
#include "ff.h"
#include "tim.h"
#include "sdio.h"
#include "bsp_driver_sd.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Configuration checks                                                       */
/* -------------------------------------------------------------------------- */

#define SDLOGGER_MAGIC                    0x54475931UL /* TGY1 */
#define SDLOGGER_FORMAT_VERSION           14U
#define SDLOGGER_FRAME_SIZE               384U
#define SDLOGGER_RAM_FLIGHT_FRAME_SIZE     192U
#define SDLOGGER_RAM_FLIGHT_CAPACITY       256U
#define SDLOGGER_RAM_FLIGHT_FORMAT_VERSION 16U
#define SDLOGGER_RAM_FLIGHT_MARKER         0xF10BU
#define SDLOGGER_BUFFER_COUNT             APP_SDLOGGER_DMA_BUFFER_COUNT
#define SDLOGGER_INVALID_BUFFER           0xFFU

#define SDLOGGER_BUFFER_EMPTY             0U
#define SDLOGGER_BUFFER_FILLING           1U
#define SDLOGGER_BUFFER_READY             2U
#define SDLOGGER_BUFFER_WRITING           3U

#define SDLOGGER_HEALTH_IMU_OK            (1U << 0)
#define SDLOGGER_HEALTH_BARO_OK           (1U << 1)
#define SDLOGGER_HEALTH_LIDAR_OK          (1U << 2)
#define SDLOGGER_HEALTH_SYSTEM_OK         (1U << 3)
#define SDLOGGER_HEALTH_SD_OK             (1U << 4)
#define SDLOGGER_HEALTH_NEEDLE_OK         (1U << 5)
#define SDLOGGER_HEALTH_NRF_OK            (1U << 6)
#define SDLOGGER_HEALTH_SERVO_OK          (1U << 7)

#define SDLOGGER_SECTOR_SIZE              512U

#define SDLOGGER_ASYNC_IDLE               0U
#define SDLOGGER_ASYNC_DATA_DMA           1U
#define SDLOGGER_ASYNC_GUARD_DMA          2U

/*
 * Completed APP_SDLOGGER_BUFFER_SIZE-byte buffers between metadata
 * synchronizations.
 *
 * 0U  : only SDLogger_RequestSync() / SDLogger_Stop()
 * 4U  : about 1.16 seconds at 200 Hz with a 14848-byte buffer
 * 8U  : about 2.32 seconds at 200 Hz with a 14848-byte buffer
 * 16U : about 4.64 seconds at 200 Hz with a 14848-byte buffer
 */
#ifndef APP_SDLOGGER_SYNC_INTERVAL_BUFFERS
#define APP_SDLOGGER_SYNC_INTERVAL_BUFFERS 0U
#endif

#ifndef APP_SDLOGGER_PREALLOCATE_ENABLED
#define APP_SDLOGGER_PREALLOCATE_ENABLED 0U
#endif

#ifndef APP_SDLOGGER_PREALLOCATE_BYTES
#define APP_SDLOGGER_PREALLOCATE_BYTES (128UL * 1024UL * 1024UL)
#endif

#ifndef APP_SDLOGGER_PREALLOCATE_MIN_BYTES
#define APP_SDLOGGER_PREALLOCATE_MIN_BYTES (96UL * 1024UL * 1024UL)
#endif

#ifndef APP_SDLOGGER_GUARD_SECTOR_ENABLED
#define APP_SDLOGGER_GUARD_SECTOR_ENABLED 1U
#endif

#ifndef APP_SDLOGGER_RAW_DMA_ENABLED
#define APP_SDLOGGER_RAW_DMA_ENABLED 0U
#endif

#ifndef APP_SDLOGGER_DMA_TIMEOUT_MS
#define APP_SDLOGGER_DMA_TIMEOUT_MS 250UL
#endif

#ifndef APP_SDLOGGER_CARD_POLL_PERIOD_MS
#define APP_SDLOGGER_CARD_POLL_PERIOD_MS 1UL
#endif

#ifndef APP_SDLOGGER_STOP_TIMEOUT_MS
#define APP_SDLOGGER_STOP_TIMEOUT_MS 5000UL
#endif

#if ((APP_SDLOGGER_RING_FRAME_COUNT < 4U) || \
     ((APP_SDLOGGER_RING_FRAME_COUNT & (APP_SDLOGGER_RING_FRAME_COUNT - 1U)) != 0U))
#error "APP_SDLOGGER_RING_FRAME_COUNT must be a power of two and >= 4"
#endif

#if (APP_SDLOGGER_RING_DRAIN_MAX_FRAMES == 0U)
#error "APP_SDLOGGER_RING_DRAIN_MAX_FRAMES must be > 0"
#endif

#if ((APP_SDLOGGER_DMA_BUFFER_COUNT < 2U) || (APP_SDLOGGER_DMA_BUFFER_COUNT > 4U))
#error "APP_SDLOGGER_DMA_BUFFER_COUNT must be 2..4"
#endif

#if ((APP_SDLOGGER_RING_BACKPRESSURE_CLEAR_FRAMES >=       APP_SDLOGGER_RING_BACKPRESSURE_START_FRAMES) ||      (APP_SDLOGGER_RING_BACKPRESSURE_START_FRAMES >=       APP_SDLOGGER_RING_BACKPRESSURE_CRITICAL_FRAMES) ||      (APP_SDLOGGER_RING_BACKPRESSURE_CRITICAL_FRAMES >=       (APP_SDLOGGER_RING_FRAME_COUNT - 1U)))
#error "Invalid P47 SD ring backpressure thresholds"
#endif

#if (APP_SDLOGGER_SAMPLE_PERIOD_US != 33333UL)
#error "TIM5 is configured for 30 Hz; APP_SDLOGGER_SAMPLE_PERIOD_US must be 33333"
#endif

#if ((APP_SDLOGGER_R10_RAM_DECIMATION < 1U) || (APP_SDLOGGER_R10_RAM_DECIMATION > 16U))
#error "APP_SDLOGGER_R10_RAM_DECIMATION must be 1..16"
#endif

#define SDLOGGER_RING_INDEX_MASK \
    (APP_SDLOGGER_RING_FRAME_COUNT - 1U)

#if ((APP_SDLOGGER_BUFFER_SIZE % SDLOGGER_FRAME_SIZE) != 0U)
#error "APP_SDLOGGER_BUFFER_SIZE must be a multiple of SDLOGGER_FRAME_SIZE"
#endif

#if ((APP_SDLOGGER_WRITE_CHUNK_SIZE == 0U) || \
     (APP_SDLOGGER_WRITE_CHUNK_SIZE > APP_SDLOGGER_BUFFER_SIZE))
#error "Invalid APP_SDLOGGER_WRITE_CHUNK_SIZE"
#endif

#if ((APP_SDLOGGER_BUFFER_SIZE % SDLOGGER_SECTOR_SIZE) != 0U)
#error "APP_SDLOGGER_BUFFER_SIZE must be 512-byte aligned"
#endif

#if ((APP_SDLOGGER_RAW_DMA_ENABLED != 0U) && \
     (APP_SDLOGGER_PREALLOCATE_ENABLED == 0U))
#error "Raw SDIO DMA requires file preallocation"
#endif

#if (APP_SDLOGGER_DMA_TIMEOUT_MS == 0UL)
#error "APP_SDLOGGER_DMA_TIMEOUT_MS must be > 0"
#endif

#if (APP_SDLOGGER_STOP_TIMEOUT_MS == 0UL)
#error "APP_SDLOGGER_STOP_TIMEOUT_MS must be > 0"
#endif

#if (APP_SDLOGGER_PREALLOCATE_ENABLED != 0U)
#if ((APP_SDLOGGER_PREALLOCATE_BYTES % SDLOGGER_SECTOR_SIZE) != 0UL)
#error "APP_SDLOGGER_PREALLOCATE_BYTES must be 512-byte aligned"
#endif
#if ((APP_SDLOGGER_PREALLOCATE_MIN_BYTES % SDLOGGER_SECTOR_SIZE) != 0UL)
#error "APP_SDLOGGER_PREALLOCATE_MIN_BYTES must be 512-byte aligned"
#endif
#if (APP_SDLOGGER_PREALLOCATE_MIN_BYTES > APP_SDLOGGER_PREALLOCATE_BYTES)
#error "APP_SDLOGGER_PREALLOCATE_MIN_BYTES exceeds requested size"
#endif
#if (APP_SDLOGGER_PREALLOCATE_MIN_BYTES < APP_SDLOGGER_BUFFER_SIZE)
#error "APP_SDLOGGER_PREALLOCATE_MIN_BYTES is too small"
#endif
#endif

/* -------------------------------------------------------------------------- */
/* 384-byte binary frame (V14)                                                */
/* -------------------------------------------------------------------------- */

#define SDLOGGER_FAST_IMU_SAMPLES_PER_FRAME 5U

typedef struct
{
    uint32_t timestamp_us;
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
} SDLoggerFastIMUSample_t;

typedef char SDLoggerFastIMUSampleSizeCheck[
    (sizeof(SDLoggerFastIMUSample_t) == 16U) ? 1 : -1
];

typedef struct
{
    /* Motor/needle diagnostics captured read-only; no actuator control path. */
    uint16_t position_turns_x10000;
    uint16_t max_turns_x10000;
    uint16_t max_open_adc;
    uint16_t adc_per_turn;
    uint16_t max_travel_adc;
    uint16_t homing_start_adc;
    uint16_t hw_adc_raw12;
    uint16_t hw_adc_mv;

    uint32_t stall_ms;
    uint32_t homing_elapsed_ms;
    uint32_t control_tick_count;
    uint32_t adc_invalid_count;

    uint16_t p83_adc1_raw;
    uint16_t p83_adc2_raw;
    uint16_t p83_pair_diff;
    uint16_t p83_pair_candidate;
    uint16_t p83_median7;
    uint16_t p83_filtered_adc;
    uint8_t p83_feedback_valid;
    uint8_t p83_confidence_pct;
    uint8_t p83_mode;
    uint8_t p83_acq_progress_pct;

    uint32_t p83_pair_reject_count;
    uint32_t p83_rate_reject_count;
    uint32_t p83_quarantine_count;
    uint32_t p83_reacquire_count;
    uint32_t p83_adc_timeout_count;

    uint16_t p110_start_adc;
    uint16_t p110_target_adc;
    uint16_t p110_current_adc;
    int16_t p110_error_adc;
    uint16_t p110_speed_adc_s;
    uint16_t p110_stop_distance_adc;
    uint16_t p110_brake_entry_adc;
    uint16_t p110_final_adc;
    int16_t p110_final_error_adc;
    uint16_t p110_total_powered_ms;
    uint8_t p110_state;
    uint8_t p110_result;
    uint8_t p110_direction;
    uint8_t p110_active_pwm;
    uint8_t p110_abort_reason;
    uint8_t p111_state;
    uint8_t p111_result;
    uint8_t reserved;
} SDLoggerMotorExt_t;

typedef char SDLoggerMotorExtSizeCheck[
    (sizeof(SDLoggerMotorExt_t) == 96U) ? 1 : -1
];

typedef struct
{
    uint32_t magic;
    uint32_t sequence;
    uint32_t timestamp_us;
    uint32_t timestamp_ms;

    /* Raw barometer values. */
    int32_t pressure_pa;
    int32_t altitude_cm;

    uint32_t sensor_update_count;
    uint32_t baro_update_count;
    uint32_t lidar_update_count;

    uint16_t format_version;
    uint16_t frame_size;

    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;

    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    int16_t temperature_cx100;
    uint16_t lidar_distance_cm;
    uint16_t cpu_load_x100;

    uint8_t health_flags;
    uint8_t fault_code;

    /*
     * IMU Butterworth outputs are stored as raw-equivalent counts.
     */
    int16_t accel_x_filtered_raw_eq;
    int16_t accel_y_filtered_raw_eq;
    int16_t accel_z_filtered_raw_eq;

    int16_t gyro_x_filtered_raw_eq;
    int16_t gyro_y_filtered_raw_eq;
    int16_t gyro_z_filtered_raw_eq;

    uint16_t accel_filtered_norm_mg;

    /* IMU bits 0..2, barometer/LIDAR bits in sensor_filter_flags. */
    uint8_t filter_flags;
    uint8_t sensor_filter_flags;

    uint16_t filter_reset_count;

    /* V4: median pressure minus raw pressure, in Pa. */
    int16_t baro_median_delta_pa;

    int32_t filtered_pressure_pa;
    int32_t filtered_altitude_mm;

    int16_t vertical_speed_cms;

    uint16_t lidar_median_mm;
    uint16_t lidar_filtered_mm;

    /* Align the V5 extension to a 32-bit boundary. */
    uint16_t ekf_alignment_reserved;

    /* V10 full-state ESKF shadow output. */
    int32_t full_eskf_position_z_mm;
    int32_t full_eskf_velocity_z_mms;

    int16_t full_eskf_gyro_bias_x_mdps;
    int16_t full_eskf_gyro_bias_y_mdps;
    int16_t full_eskf_gyro_bias_z_mdps;

    uint8_t full_eskf_flags;
    uint8_t full_eskf_reset_count;

    int16_t full_eskf_position_x_cm;
    int16_t full_eskf_position_y_cm;
    int16_t full_eskf_velocity_x_cms;
    int16_t full_eskf_velocity_y_cms;

    int16_t full_eskf_accel_bias_x_mms2;
    int16_t full_eskf_accel_bias_y_mms2;
    int16_t full_eskf_accel_bias_z_mms2;

    /* V10 FullStateESKF quaternion, Euler, world-accel and counters. */
    int16_t full_eskf_q_w_q15;
    int16_t full_eskf_q_x_q15;
    int16_t full_eskf_q_y_q15;
    int16_t full_eskf_q_z_q15;

    int16_t full_eskf_roll_cdeg;
    int16_t full_eskf_pitch_cdeg;
    int16_t full_eskf_yaw_cdeg;

    int16_t full_eskf_world_accel_x_cms2;
    int16_t full_eskf_world_accel_y_cms2;
    int16_t full_eskf_world_accel_z_cms2;

    uint8_t full_eskf_attitude_flags;
    uint8_t full_eskf_gravity_weight_x255;
    uint16_t full_eskf_predict_count_low;

    uint16_t full_eskf_public_output_count_low;
    uint16_t full_eskf_covariance_predict_count_low;
    uint16_t full_eskf_gravity_update_count_low;
    uint16_t full_eskf_stationary_detector_count_low;
    uint16_t full_eskf_stationary_update_count_low;

    /* V10 DWT profiler maxima and wrapping cumulative cycle counters. */
    uint32_t full_eskf_max_predict_cycles;
    uint32_t full_eskf_total_predict_cycles;
    uint32_t full_eskf_max_public_output_cycles;
    uint32_t full_eskf_total_public_output_cycles;
    uint32_t full_eskf_max_correction_cycles;
    uint32_t full_eskf_total_correction_cycles;
    uint32_t full_eskf_max_covariance_cycles;
    uint32_t full_eskf_total_covariance_cycles;
    uint32_t full_eskf_max_gravity_cycles;
    uint32_t full_eskf_total_gravity_cycles;
    uint32_t full_eskf_max_stationary_cycles;
    uint32_t full_eskf_total_stationary_cycles;
    uint32_t full_eskf_max_baro_cycles;
    uint32_t full_eskf_total_baro_cycles;
    uint32_t full_eskf_max_lidar_cycles;
    uint32_t full_eskf_total_lidar_cycles;
    uint32_t full_eskf_cpu_clock_hz;

    /* V11 IMU watchdog/recovery and RCS inhibit diagnostics. */
    uint8_t imu_sample_valid;
    uint8_t imu_recovery_state;
    uint8_t rcs_imu_inhibit;
    uint8_t imu_diag_flags;

    uint32_t imu_stale_count;
    uint32_t imu_pattern_error_count;
    uint32_t imu_dma_timeout_count;
    uint32_t imu_recovery_count;
    uint32_t imu_register_error_count;

    uint16_t imu_diag_reserved;

    /* V12 needle-valve telemetry (16 bytes). */
    uint16_t needle_requested_cmd_x10000;
    uint16_t needle_limited_cmd_x10000;
    uint16_t needle_raw_adc;
    uint16_t needle_zero_adc;
    uint16_t needle_target_adc;
    int16_t needle_error_adc;
    uint8_t needle_rpwm;
    uint8_t needle_lpwm;
    uint8_t needle_state_flags; /* b0 EN,b1 ZERO,b2 LOCK,b3 HOME,b4 HOME_DONE,b5 ACTIVE,b6 OPEN,b7 CLOSE */
    uint8_t needle_fault;

    /*
     * V13 NRF + tahliye-servo telemetry (16 bytes).
     *
     * Field order is intentional: nrf_valid_packet_count starts at offset
     * 272, which is naturally 32-bit aligned. This keeps the complete frame
     * exactly 288 bytes without __packed and preserves every V12 offset.
     */
    uint8_t nrf_link_active;
    uint8_t nrf_command;

    uint32_t nrf_valid_packet_count;

    uint8_t nrf_last_sequence;
    uint8_t servo_target_open;

    uint16_t servo_pulse_us;
    uint16_t nrf_last_packet_age_ms;
    uint16_t nrf_irq_count_low;
    uint16_t nrf_rx_count_low;

    /*
     * V14 extension starts at byte 286.
     * Keep a 16-bit marker so the 1 kHz history begins 32-bit aligned.
     */
    uint16_t v14_extension_marker;

    /*
     * Five raw IMU samples provide a compact 1 kHz raw stream inside each
     * 200 Hz flight-state frame. Samples are chronological oldest -> newest.
     */
    SDLoggerFastIMUSample_t fast_imu[SDLOGGER_FAST_IMU_SAMPLES_PER_FRAME];

    uint8_t fast_imu_sample_count;
    uint8_t fast_imu_valid_mask;

    uint16_t imu_sample_age_us;
    uint16_t baro_sample_age_us;
    uint16_t lidar_sample_age_us;

    uint8_t sensor_qual_state;
    uint8_t sensor_qual_good_windows;
    uint16_t sensor_qual_flags_low;
    uint16_t sensor_qual_flags_high;

    uint16_t crc16;

} SDLoggerFrame_t;

typedef char SDLoggerFrameSizeCheck[
    (sizeof(SDLoggerFrame_t) == SDLOGGER_FRAME_SIZE) ? 1 : -1
];

typedef char SDLoggerCRCOffsetCheck[
    (offsetof(SDLoggerFrame_t, crc16) == 382U) ? 1 : -1
];

/*
 * R8R35R3R10 RAM-first flight record. V16 expands the compact record to
 * 192 bytes only for read-only motor diagnostics. The CCM allocation stays
 * exactly 49,152 bytes: 256 x 192 bytes. At 7.5 Hz this still preserves
 * about 34.1 seconds of flight, far above the intended mission duration.
 *
 * No SDIO/FatFS operation is performed while this record is captured.
 */
typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_us;
    uint32_t timestamp_ms;
    int32_t pressure_pa;
    int32_t altitude_mm;
    int32_t eskf_z_mm;
    int32_t eskf_vz_mms;

    int16_t eskf_x_cm;
    int16_t eskf_y_cm;
    int16_t eskf_vx_cms;
    int16_t eskf_vy_cms;
    int16_t roll_cdeg;
    int16_t pitch_cdeg;
    int16_t yaw_cdeg;
    int16_t pitch_rate_cdeg_s;
    int16_t yaw_rate_cdeg_s;
    int16_t q_w_q15;
    int16_t q_x_q15;
    int16_t q_y_q15;
    int16_t q_z_q15;
    uint16_t lidar_mm;
    uint16_t cpu_load_x100;
    uint16_t needle_requested_x10000;
    uint16_t needle_limited_x10000;
    uint16_t needle_raw_adc;
    uint16_t needle_target_adc;
    int16_t needle_error_adc;
    uint16_t valve_cmd_x10000;
    uint16_t imu_age_us;
    uint16_t baro_age_us;
    uint16_t lidar_age_us;

    uint8_t health_flags;
    uint8_t fault_code;
    uint8_t mission_state;
    uint8_t flight_input_valid;
    uint8_t rcs_requested_mask;
    uint8_t rcs_applied_mask;
    uint8_t needle_rpwm;
    uint8_t needle_lpwm;
    uint8_t needle_state_flags;
    uint8_t needle_fault;
    uint8_t full_eskf_flags;
    uint8_t full_eskf_attitude_flags;
    uint8_t flight_rcs_fault;
    uint8_t mount_cal_valid;

    uint16_t marker;
    uint16_t needle_zero_adc;
    uint16_t crc16;

    SDLoggerMotorExt_t motor_ext;
} SDLoggerRamFlightFrame_t;

typedef char SDLoggerRamFlightFrameSizeCheck[
    (sizeof(SDLoggerRamFlightFrame_t) == SDLOGGER_RAM_FLIGHT_FRAME_SIZE) ? 1 : -1
];

typedef union
{
    SDLoggerFrame_t normal[APP_SDLOGGER_RING_FRAME_COUNT];
    SDLoggerRamFlightFrame_t flight[SDLOGGER_RAM_FLIGHT_CAPACITY];
} SDLoggerCaptureStorage_t;

typedef char SDLoggerCaptureStorageSizeCheck[
    (sizeof(SDLoggerCaptureStorage_t) ==
     (APP_SDLOGGER_RING_FRAME_COUNT * SDLOGGER_FRAME_SIZE)) ? 1 : -1
];

/*
 * Main-context source publication buffer. The 1 kHz IMU task writes the
 * inactive copy and atomically publishes its index. TIM5 reads only the
 * currently published copy, so it never sees a partially updated sensor
 * structure.
 */
typedef struct
{
    int32_t pressure_pa;
    int32_t altitude_cm;

    uint32_t sensor_update_count;
    uint32_t baro_update_count;
    uint32_t lidar_update_count;

    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;

    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    int16_t temperature_cx100;
    uint16_t lidar_distance_cm;
    uint16_t cpu_load_x100;

    uint8_t health_flags;
    uint8_t fault_code;

    int16_t accel_x_filtered_raw_eq;
    int16_t accel_y_filtered_raw_eq;
    int16_t accel_z_filtered_raw_eq;

    int16_t gyro_x_filtered_raw_eq;
    int16_t gyro_y_filtered_raw_eq;
    int16_t gyro_z_filtered_raw_eq;

    uint16_t accel_filtered_norm_mg;

    uint8_t filter_flags;
    uint8_t sensor_filter_flags;

    uint16_t filter_reset_count;
    int16_t baro_median_delta_pa;

    int32_t filtered_pressure_pa;
    int32_t filtered_altitude_mm;

    int16_t vertical_speed_cms;

    uint16_t lidar_median_mm;
    uint16_t lidar_filtered_mm;

    int32_t full_eskf_position_z_mm;
    int32_t full_eskf_velocity_z_mms;

    int16_t full_eskf_gyro_bias_x_mdps;
    int16_t full_eskf_gyro_bias_y_mdps;
    int16_t full_eskf_gyro_bias_z_mdps;

    uint8_t full_eskf_flags;
    uint8_t full_eskf_reset_count;

    int16_t full_eskf_position_x_cm;
    int16_t full_eskf_position_y_cm;
    int16_t full_eskf_velocity_x_cms;
    int16_t full_eskf_velocity_y_cms;

    int16_t full_eskf_accel_bias_x_mms2;
    int16_t full_eskf_accel_bias_y_mms2;
    int16_t full_eskf_accel_bias_z_mms2;

    int16_t full_eskf_q_w_q15;
    int16_t full_eskf_q_x_q15;
    int16_t full_eskf_q_y_q15;
    int16_t full_eskf_q_z_q15;

    int16_t full_eskf_roll_cdeg;
    int16_t full_eskf_pitch_cdeg;
    int16_t full_eskf_yaw_cdeg;

    int16_t full_eskf_world_accel_x_cms2;
    int16_t full_eskf_world_accel_y_cms2;
    int16_t full_eskf_world_accel_z_cms2;

    uint8_t full_eskf_attitude_flags;
    uint8_t full_eskf_gravity_weight_x255;
    uint16_t full_eskf_predict_count_low;

    uint16_t full_eskf_public_output_count_low;
    uint16_t full_eskf_covariance_predict_count_low;
    uint16_t full_eskf_gravity_update_count_low;
    uint16_t full_eskf_stationary_detector_count_low;
    uint16_t full_eskf_stationary_update_count_low;

    uint32_t full_eskf_max_predict_cycles;
    uint32_t full_eskf_total_predict_cycles;
    uint32_t full_eskf_max_public_output_cycles;
    uint32_t full_eskf_total_public_output_cycles;
    uint32_t full_eskf_max_correction_cycles;
    uint32_t full_eskf_total_correction_cycles;
    uint32_t full_eskf_max_covariance_cycles;
    uint32_t full_eskf_total_covariance_cycles;
    uint32_t full_eskf_max_gravity_cycles;
    uint32_t full_eskf_total_gravity_cycles;
    uint32_t full_eskf_max_stationary_cycles;
    uint32_t full_eskf_total_stationary_cycles;
    uint32_t full_eskf_max_baro_cycles;
    uint32_t full_eskf_total_baro_cycles;
    uint32_t full_eskf_max_lidar_cycles;
    uint32_t full_eskf_total_lidar_cycles;
    uint32_t full_eskf_cpu_clock_hz;

    uint8_t imu_sample_valid;
    uint8_t imu_recovery_state;
    uint8_t rcs_imu_inhibit;
    uint8_t imu_diag_flags;

    uint32_t imu_stale_count;
    uint32_t imu_pattern_error_count;
    uint32_t imu_dma_timeout_count;
    uint32_t imu_recovery_count;
    uint32_t imu_register_error_count;

    uint16_t needle_requested_cmd_x10000;
    uint16_t needle_limited_cmd_x10000;
    uint16_t needle_raw_adc;
    uint16_t needle_zero_adc;
    uint16_t needle_target_adc;
    int16_t needle_error_adc;
    uint8_t needle_rpwm;
    uint8_t needle_lpwm;
    uint8_t needle_state_flags;
    uint8_t needle_fault;
    SDLoggerMotorExt_t motor_ext;

    uint8_t nrf_link_active;
    uint8_t nrf_command;
    uint8_t nrf_last_sequence;
    uint8_t servo_target_open;

    uint16_t servo_pulse_us;
    uint16_t nrf_last_packet_age_ms;

    uint32_t nrf_valid_packet_count;

    uint16_t nrf_irq_count_low;
    uint16_t nrf_rx_count_low;

    SDLoggerFastIMUSample_t fast_imu[SDLOGGER_FAST_IMU_SAMPLES_PER_FRAME];
    uint8_t fast_imu_sample_count;
    uint8_t fast_imu_valid_mask;

    uint16_t imu_sample_age_us;
    uint16_t baro_sample_age_us;
    uint16_t lidar_sample_age_us;

    uint8_t sensor_qual_state;
    uint8_t sensor_qual_good_windows;
    uint16_t sensor_qual_flags_low;
    uint16_t sensor_qual_flags_high;

    /* R3R10 control/status scalars copied in main context for compact RAM log. */
    uint8_t mission_state;
    uint8_t flight_input_valid;
    uint8_t flight_rcs_requested_mask;
    uint8_t flight_rcs_applied_mask;
    uint8_t flight_rcs_fault;
    uint8_t mount_cal_valid;
    uint16_t flight_valve_cmd_x10000;
    int16_t flight_pitch_rate_cdeg_s;
    int16_t flight_yaw_rate_cdeg_s;

} SDLoggerSourceSnapshot_t;

/* Scheduler CPU monitor live value. */
extern volatile uint32_t cpu_load_percent_x100;
extern volatile uint8_t sm_system_ok;
extern volatile uint8_t sm_fault_code;
extern volatile uint8_t v30_stop_latched;

/* SensorManager Butterworth diagnostics. */
extern volatile uint8_t sensor_imu_filter_enabled;
extern volatile uint8_t sensor_imu_filter_config_ok;
extern volatile uint8_t sensor_imu_filter_initialized;
extern volatile uint32_t sensor_imu_filter_reset_count;

/* SDIO DMA completion/error flags from bsp_driver_sd.c. */
extern volatile uint8_t sd_dma_tx_complete;
extern volatile uint8_t sd_dma_transfer_error;
extern volatile uint8_t sd_dma_transfer_active;

/* -------------------------------------------------------------------------- */
/* Live Expressions                                                           */
/* -------------------------------------------------------------------------- */

volatile uint8_t sd_logger_initialized = 0U;
volatile uint8_t sd_logger_ready = 0U;
volatile uint8_t sd_logger_mount_ok = 0U;
volatile uint8_t sd_logger_test_done = 0U;
volatile uint8_t sd_logger_test_passed = 0U;
volatile uint8_t sd_logger_file_open = 0U;
volatile uint8_t sd_logger_logging_active = 0U;

volatile uint32_t sd_logger_init_count = 0UL;
volatile uint32_t sd_logger_update_count = 0UL;
volatile uint32_t sd_logger_error_count = 0UL;
volatile uint32_t sd_logger_write_error_count = 0UL;

volatile uint8_t sd_logger_last_result = 0U;
volatile uint8_t sd_logger_disk_init_status = STA_NOINIT;
volatile uint8_t sd_logger_mount_retry_count = 0U;
volatile uint32_t sd_logger_bytes_written = 0UL;
volatile uint32_t sd_logger_total_bytes_written = 0UL;
volatile uint32_t sd_logger_last_write_bytes = 0UL;

volatile uint32_t sd_logger_frame_count = 0UL;
volatile uint32_t sd_logger_dropped_frame_count = 0UL;
volatile uint32_t sd_logger_missed_period_count = 0UL;
volatile uint32_t sd_logger_flush_count = 0UL;
volatile uint8_t sd_logger_sync_pending = 0U;
volatile uint32_t sd_logger_buffers_since_sync = 0UL;
volatile uint32_t sd_logger_sync_request_count = 0UL;
volatile uint32_t sd_logger_sync_count = 0UL;
volatile uint32_t sd_logger_sync_deferred_count = 0UL;
volatile uint32_t sd_logger_sync_error_count = 0UL;
volatile uint32_t sd_logger_last_sync_duration_us = 0UL;
volatile uint32_t sd_logger_max_sync_duration_us = 0UL;
volatile uint32_t sd_logger_buffer_overrun_count = 0UL;

volatile uint8_t sd_logger_active_buffer = 0U;
volatile uint8_t sd_logger_writing_buffer = SDLOGGER_INVALID_BUFFER;
volatile uint8_t sd_logger_buffer0_state = SDLOGGER_BUFFER_EMPTY;
volatile uint8_t sd_logger_buffer1_state = SDLOGGER_BUFFER_EMPTY;
volatile uint32_t sd_logger_buffer0_fill = 0UL;
volatile uint32_t sd_logger_buffer1_fill = 0UL;
volatile uint32_t sd_logger_write_offset = 0UL;

volatile uint32_t sd_logger_next_sample_us = 0UL;
volatile uint32_t sd_logger_last_capture_us = 0UL;
volatile uint32_t sd_logger_last_write_duration_us = 0UL;
volatile uint32_t sd_logger_max_write_duration_us = 0UL;

volatile uint32_t sd_logger_last_sequence = 0UL;
volatile uint16_t sd_logger_last_crc16 = 0U;
volatile uint32_t sd_logger_frame_struct_size = sizeof(SDLoggerFrame_t);

/* V14 high-rate sensor evidence diagnostics. */
volatile uint32_t sd_logger_fast_imu_push_count = 0UL;
volatile uint32_t sd_logger_fast_imu_duplicate_skip_count = 0UL;
volatile uint32_t sd_logger_fast_imu_history_count = 0UL;
volatile uint8_t sd_logger_fast_imu_last_valid_mask = 0U;
volatile uint16_t sd_logger_last_imu_age_us = 65535U;
volatile uint16_t sd_logger_last_baro_age_us = 65535U;
volatile uint16_t sd_logger_last_lidar_age_us = 65535U;

/* V13 latest NRF / tahliye-servo values published to SD. */
volatile uint8_t sd_logger_nrf_link_active = 0U;
volatile uint8_t sd_logger_nrf_command = 0U;
volatile uint8_t sd_logger_nrf_last_sequence = 0U;
volatile uint8_t sd_logger_servo_target_open = 0U;
volatile uint16_t sd_logger_servo_pulse_us = 0U;
volatile uint16_t sd_logger_nrf_last_packet_age_ms = 65535U;
volatile uint32_t sd_logger_nrf_valid_packet_count = 0UL;
volatile uint16_t sd_logger_nrf_irq_count_low = 0U;
volatile uint16_t sd_logger_nrf_rx_count_low = 0U;

/* V12 latest needle snapshot used by the binary logger. */
volatile uint16_t sd_logger_needle_requested_cmd_x10000 = 0U;
volatile uint16_t sd_logger_needle_limited_cmd_x10000 = 0U;
volatile uint16_t sd_logger_needle_raw_adc = 0U;
volatile uint16_t sd_logger_needle_zero_adc = 0U;
volatile uint16_t sd_logger_needle_target_adc = 0U;
volatile int16_t sd_logger_needle_error_adc = 0;
volatile uint8_t sd_logger_needle_rpwm = 0U;
volatile uint8_t sd_logger_needle_lpwm = 0U;
volatile uint8_t sd_logger_needle_state_flags = 0U;
volatile uint8_t sd_logger_needle_fault = 0U;

/* Startup preallocation / power-loss recovery diagnostics. */
volatile uint8_t sd_logger_preallocate_enabled =
    APP_SDLOGGER_PREALLOCATE_ENABLED;
volatile uint8_t sd_logger_preallocate_ok = 0U;
volatile uint32_t sd_logger_preallocated_bytes = 0UL;
volatile uint32_t sd_logger_preallocation_attempt_count = 0UL;
volatile uint32_t sd_logger_preallocation_fallback_count = 0UL;
volatile uint32_t sd_logger_preallocation_duration_ms = 0UL;
volatile uint32_t sd_logger_finalize_duration_ms = 0UL;
volatile uint32_t sd_logger_runtime_sync_suppressed_count = 0UL;
volatile uint32_t sd_logger_guard_write_count = 0UL;
volatile uint32_t sd_logger_last_guard_write_duration_us = 0UL;
volatile uint32_t sd_logger_max_guard_write_duration_us = 0UL;

/* Non-blocking raw SDIO DMA diagnostics. */
volatile uint8_t sd_logger_raw_dma_enabled =
    APP_SDLOGGER_RAW_DMA_ENABLED;
volatile uint8_t sd_logger_async_state = SDLOGGER_ASYNC_IDLE;
volatile uint32_t sd_logger_file_start_sector = 0UL;
volatile uint32_t sd_logger_preallocated_sector_count = 0UL;
volatile uint32_t sd_logger_next_data_sector_offset = 0UL;
volatile uint32_t sd_logger_async_data_start_count = 0UL;
volatile uint32_t sd_logger_async_data_complete_count = 0UL;
volatile uint32_t sd_logger_async_card_busy_poll_count = 0UL;
volatile uint32_t sd_logger_async_timeout_count = 0UL;

/* R8R35R3R8 motor-aware SD write hold diagnostics. */
volatile uint8_t sd_logger_motor_write_hold_active = 0U;
volatile uint32_t sd_logger_motor_write_hold_entry_count = 0UL;
volatile uint32_t sd_logger_motor_write_hold_defer_count = 0UL;
volatile uint32_t sd_logger_motor_write_hold_until_ms = 0UL;
/* R8R35R3R9 actuator-quiet diagnostics. */
volatile uint32_t sd_logger_r9_rcs_hold_event_count = 0UL;
volatile uint32_t sd_logger_r9_hold_remaining_ms = 0UL;

/* R8R35R3R10 RAM-first flight logger diagnostics. */
volatile uint8_t sd_logger_r10_ram_mode = 0U;
volatile uint32_t sd_logger_r10_ram_frame_count = 0UL;
volatile uint32_t sd_logger_r10_ram_capacity = SDLOGGER_RAM_FLIGHT_CAPACITY;
volatile uint32_t sd_logger_r10_ram_overflow_count = 0UL;
static volatile uint8_t sd_r10_ram_decimation_phase = 0U;
volatile uint8_t sd_logger_r10_capture_frozen = 0U;
volatile uint8_t sd_logger_r10_flush_active = 0U;
volatile uint32_t sd_logger_r10_flush_index = 0UL;
volatile uint8_t sd_logger_r10_flush_complete = 0U;
volatile uint8_t sd_logger_r10_tail_dma_at_entry = 0U;
volatile uint32_t sd_logger_r10_physical_sd_fault_count = 0UL;
volatile uint8_t sd_logger_r10_postflight_reason = 0U; /* 1 touchdown, 2 E-stop */
volatile uint32_t sd_logger_r10_entry_ms = 0UL;
volatile uint32_t sd_logger_r10_flush_start_ms = 0UL;

/* V8.13 transient SDIO retry diagnostics. */
volatile uint32_t sd_logger_dma_retry_attempt_count = 0UL;
volatile uint32_t sd_logger_dma_retry_success_count = 0UL;
volatile uint32_t sd_logger_dma_retry_exhausted_count = 0UL;
volatile uint32_t sd_logger_dma_retry_card_busy_count = 0UL;
volatile uint32_t sd_logger_dma_retry_abort_count = 0UL;
volatile uint8_t sd_logger_dma_retry_pending = 0U;
volatile uint8_t sd_logger_dma_retry_current = 0U;
volatile uint8_t sd_logger_runtime_recovery_active = 0U;
volatile uint32_t sd_logger_runtime_recovery_count = 0UL;
volatile uint32_t sd_logger_runtime_recovery_success_count = 0UL;
volatile uint32_t sd_logger_runtime_recovery_failure_count = 0UL;
volatile uint32_t sd_logger_runtime_recovery_flight_abort_count = 0UL;
volatile uint32_t sd_logger_runtime_recovery_last_us = 0UL;
volatile uint32_t sd_logger_runtime_recovery_max_us = 0UL;
volatile uint32_t sd_logger_runtime_last_hal_error = 0UL;

/* TIM5 capture diagnostics */
volatile uint8_t sd_logger_capture_timer_started = 0U;
volatile uint32_t sd_logger_capture_timer_start_error_count = 0UL;
volatile uint32_t sd_logger_timer_irq_count = 0UL;
volatile uint32_t sd_logger_timer_last_interval_us = 0UL;
volatile uint32_t sd_logger_timer_min_interval_us = 0UL;
volatile uint32_t sd_logger_timer_max_interval_us = 0UL;
volatile uint32_t sd_logger_timer_last_isr_duration_us = 0UL;
volatile uint32_t sd_logger_timer_max_isr_duration_us = 0UL;

/* Published source diagnostics */
volatile uint32_t sd_logger_source_publish_count = 0UL;
volatile uint8_t sd_logger_source_active_index = 0U;

/* Capture ring diagnostics */
volatile uint32_t sd_logger_ring_capacity = APP_SDLOGGER_RING_FRAME_COUNT - 1U;
volatile uint32_t sd_logger_ring_count = 0UL;
volatile uint32_t sd_logger_ring_high_watermark = 0UL;
volatile uint32_t sd_logger_ring_push_count = 0UL;
volatile uint32_t sd_logger_ring_pop_count = 0UL;
volatile uint32_t sd_logger_ring_overrun_count = 0UL;
volatile uint32_t sd_logger_drain_last_us = 0UL;
volatile uint32_t sd_logger_drain_max_us = 0UL;
volatile uint32_t sd_logger_drain_budget_yield_count = 0UL;
volatile uint8_t sd_logger_backpressure_level = 0U;
volatile uint32_t sd_logger_backpressure_entry_count = 0UL;
volatile uint32_t sd_logger_backpressure_critical_entry_count = 0UL;
volatile uint8_t sd_logger_guard_pending = 0U;
volatile uint32_t sd_logger_guard_deferred_count = 0UL;
/* P49: DMA writer buffers are ordered by monotonic ready ticket, never index. */
volatile uint32_t sd_logger_fifo_order_fault_count = 0UL;
volatile uint32_t sd_logger_fifo_last_started_order = 0UL;
volatile uint32_t sd_logger_fifo_next_ready_order = 1UL;
volatile uint32_t sd_logger_ring_bytes =
    APP_SDLOGGER_RING_FRAME_COUNT * sizeof(SDLoggerFrame_t);

volatile uint32_t sd_logger_ring_address = 0UL;
volatile uint32_t sd_logger_ring_end_address = 0UL;
volatile uint8_t sd_logger_ring_ccm_expected = 1U;

/* -------------------------------------------------------------------------- */
/* Private data                                                               */
/* -------------------------------------------------------------------------- */

static FIL sd_log_file;

static uint8_t sd_guard_sector[SDLOGGER_SECTOR_SIZE]
    __attribute__((aligned(4)));

/* R3R10: four 1536-byte DMA-accessible writer buffers remain in normal SRAM.
 * Short quiet-side transfers reduce PE9-boundary exposure and free SRAM. */
static uint8_t sd_buffers[SDLOGGER_BUFFER_COUNT][APP_SDLOGGER_BUFFER_SIZE]
    __attribute__((aligned(4)));

static uint8_t sd_buffer_state[SDLOGGER_BUFFER_COUNT];
static uint32_t sd_buffer_fill[SDLOGGER_BUFFER_COUNT];
/* P49 chronological ticket assigned when a writer buffer becomes READY. */
static uint32_t sd_buffer_ready_order[SDLOGGER_BUFFER_COUNT];
static uint32_t sd_next_ready_order = 1UL;
static uint32_t sd_last_started_ready_order = 0UL;

static uint8_t sd_active_buffer = 0U;
static uint8_t sd_writing_buffer = SDLOGGER_INVALID_BUFFER;
static uint32_t sd_write_offset = 0UL;
static uint32_t sd_sequence = 0UL;
static uint32_t sd_next_sample_us = 0UL;

static uint8_t sd_sync_pending = 0U;
static uint8_t sd_sync_defer_reported = 0U;
static uint32_t sd_buffers_since_sync = 0UL;

static uint32_t sd_preallocated_bytes = 0UL;

/* Raw SDIO DMA state. */
static uint8_t sd_async_state = SDLOGGER_ASYNC_IDLE;
static uint32_t sd_file_start_sector = 0UL;
static uint32_t sd_preallocated_sector_count = 0UL;
static uint32_t sd_next_data_sector_offset = 0UL;
static uint32_t sd_async_transfer_start_us = 0UL;
static uint32_t sd_async_transfer_start_ms = 0UL;
static uint32_t sd_async_transfer_sector_count = 0UL;
static uint32_t sd_async_data_bytes = 0UL;
static uint32_t sd_async_next_card_poll_ms = 0UL;

#define SDLOGGER_DMA_RETRY_LIMIT          2U
#define SDLOGGER_DMA_START_RETRY_LIMIT_FLIGHT 8U
#define SDLOGGER_DMA_RETRY_DELAY_MS       5UL

static uint8_t sd_dma_retry_pending_internal = 0U;
static uint8_t sd_dma_retry_count_current = 0U;
static uint32_t sd_dma_retry_due_ms = 0UL;

/* R8R35R3R8: non-blocking SDIO write blackout around needle motor activity. */
static uint32_t sd_motor_write_hold_until_ms = 0UL;
static uint8_t sd_motor_write_hold_prev = 0U;
static uint8_t sd_flight_active_prev = 0U;
static uint32_t sd_r9_rcs_pulse_complete_prev = 0UL;
static uint32_t sd_r10_postflight_quiet_since_ms = 0UL;
/* R3R10R4R1 postflight replay gate. This state is deliberately private so the
 * TGY73 schema stays byte-for-byte compatible with R4. */
static uint8_t sd_r10_postflight_cmd_path_prepared = 0U;
static uint32_t sd_r10_replay_fault_baseline = 0UL;

/*
 * V8.15C CPU-only single-producer/single-consumer capture ring.
 *
 * This ring is placed in CCMRAM to create large main-SRAM/stack margin.
 * SDIO DMA never reads this ring directly. Main context drains frames into
 * sd_buffers[], which intentionally remain in normal DMA-accessible SRAM.
 * P48 uses four 9 KiB writer buffers so an SD-card busy spike can be absorbed
 * by writer SRAM before the fixed 49 KiB CCM ring is threatened.
 *
 * The identical 49,152-byte allocation is overlaid in R3R10:
 *   ground: 128 x 384-byte normal V14 frames
 *   flight: 256 x 192-byte compact RAM frames. Background capture is normally
 *           7.5 Hz; motor-active intervals are captured at full 30 Hz.
 */
static SDLoggerCaptureStorage_t sd_capture_storage
    __attribute__((section(".ccmram"), aligned(4)));
#define sd_capture_ring (sd_capture_storage.normal)

static volatile uint32_t sd_ring_head = 0UL;
static volatile uint32_t sd_ring_tail = 0UL;

/* Main publishes, TIM5 consumes. */
static SDLoggerSourceSnapshot_t sd_source_snapshots[2];
static volatile uint8_t sd_source_active_index = 0U;

/* Rolling 1 kHz raw IMU history; P32 writes it via SDLogger_PushFastIMU(). */
static SDLoggerFastIMUSample_t sd_fast_imu_history[SDLOGGER_FAST_IMU_SAMPLES_PER_FRAME];
static uint8_t sd_fast_imu_history_head = 0U;
static uint8_t sd_fast_imu_history_count = 0U;
static uint32_t sd_fast_imu_last_timestamp_us = 0UL;

static uint32_t sd_timer_previous_capture_us = 0UL;

static uint8_t SDLogger_FindReadyBuffer(void);
static void SDLogger_MarkBufferReady(uint8_t buffer_index);
static void SDLogger_SelectNewActiveBuffer(uint8_t buffer_index);
static uint8_t SDLogger_PreallocateFile(void);
static uint8_t SDLogger_WriteGuardSector(void);
static void SDLogger_ServiceAsyncDMA(void);
static uint8_t SDLogger_StartReadyBufferDMA(void);
static uint8_t SDLogger_RestartCurrentDMA(void);
static uint8_t SDLogger_MotorWriteHoldActive(void);
static void SDLogger_UpdateMotorWriteHold(void);
static void SDLogger_ExtendWriteHoldUntil(uint32_t candidate_until_ms);
static uint8_t SDLogger_ScheduleDMARetry(void);
static uint8_t SDLogger_ScheduleDMAStartRetry(void);
static uint8_t SDLogger_AttemptRuntimeHostRecovery(void);
static uint8_t SDLogger_StartGuardDMA(void);
static uint8_t SDLogger_DrainAllBlocking(uint32_t timeout_ms);
static void SDLogger_R10EnterRamFlightMode(void);
static void SDLogger_R10CaptureCompactFromISR(
    const SDLoggerSourceSnapshot_t *source, uint32_t capture_us);
static void SDLogger_R10ServicePostflight(void);
static void SDLogger_R10PumpReplayFrames(void);
static void SDLogger_R10BuildReplayFrame(
    const SDLoggerRamFlightFrame_t *compact, SDLoggerFrame_t *frame);

/* -------------------------------------------------------------------------- */

static void SDLogger_UpdateLiveDebug(void)
{
    sd_logger_active_buffer = sd_active_buffer;
    sd_logger_writing_buffer = sd_writing_buffer;

    sd_logger_buffer0_state = sd_buffer_state[0];
    sd_logger_buffer1_state = sd_buffer_state[1];

    sd_logger_buffer0_fill = sd_buffer_fill[0];
    sd_logger_buffer1_fill = sd_buffer_fill[1];

    sd_logger_write_offset = sd_write_offset;
    sd_logger_next_sample_us = sd_next_sample_us;

    sd_logger_sync_pending = sd_sync_pending;
    sd_logger_buffers_since_sync = sd_buffers_since_sync;

    sd_logger_source_active_index = sd_source_active_index;

    sd_logger_async_state = sd_async_state;
    sd_logger_file_start_sector = sd_file_start_sector;
    sd_logger_preallocated_sector_count =
        sd_preallocated_sector_count;
    sd_logger_next_data_sector_offset =
        sd_next_data_sector_offset;

    if (sd_logger_r10_ram_mode != 0U)
    {
        sd_logger_ring_count =
            sd_logger_r10_ram_frame_count - sd_logger_r10_flush_index;
    }
    else
    {
        uint32_t head = sd_ring_head;
        uint32_t tail = sd_ring_tail;
        sd_logger_ring_count =
            (head - tail) & SDLOGGER_RING_INDEX_MASK;
    }
    sd_logger_fifo_last_started_order = sd_last_started_ready_order;
    sd_logger_fifo_next_ready_order = sd_next_ready_order;
}

/* -------------------------------------------------------------------------- */

static int32_t SDLogger_FloatToInt32(float value, float scale)
{
    float scaled = value * scale;

    if (scaled >= 2147483520.0f)
    {
        return INT32_MAX;
    }

    if (scaled <= -2147483520.0f)
    {
        return INT32_MIN;
    }

    if (scaled >= 0.0f)
    {
        return (int32_t)(scaled + 0.5f);
    }

    return (int32_t)(scaled - 0.5f);
}

/* -------------------------------------------------------------------------- */

static int16_t SDLogger_FloatToInt16(float value, float scale)
{
    float scaled = value * scale;

    if (scaled >= 32767.0f)
    {
        return INT16_MAX;
    }

    if (scaled <= -32768.0f)
    {
        return INT16_MIN;
    }

    if (scaled >= 0.0f)
    {
        return (int16_t)(scaled + 0.5f);
    }

    return (int16_t)(scaled - 0.5f);
}

/* -------------------------------------------------------------------------- */

static uint16_t SDLogger_FloatToUInt16(float value, float scale)
{
    float scaled = value * scale;

    if (scaled <= 0.0f)
    {
        return 0U;
    }

    if (scaled >= 65535.0f)
    {
        return 65535U;
    }

    return (uint16_t)(scaled + 0.5f);
}

/* -------------------------------------------------------------------------- */

static const uint16_t crc16_ccitt_table[256] =
{
    0x0000U, 0x1021U, 0x2042U, 0x3063U, 0x4084U, 0x50A5U, 0x60C6U, 0x70E7U,
    0x8108U, 0x9129U, 0xA14AU, 0xB16BU, 0xC18CU, 0xD1ADU, 0xE1CEU, 0xF1EFU,
    0x1231U, 0x0210U, 0x3273U, 0x2252U, 0x52B5U, 0x4294U, 0x72F7U, 0x62D6U,
    0x9339U, 0x8318U, 0xB37BU, 0xA35AU, 0xD3BDU, 0xC39CU, 0xF3FFU, 0xE3DEU,
    0x2462U, 0x3443U, 0x0420U, 0x1401U, 0x64E6U, 0x74C7U, 0x44A4U, 0x5485U,
    0xA56AU, 0xB54BU, 0x8528U, 0x9509U, 0xE5EEU, 0xF5CFU, 0xC5ACU, 0xD58DU,
    0x3653U, 0x2672U, 0x1611U, 0x0630U, 0x76D7U, 0x66F6U, 0x5695U, 0x46B4U,
    0xB75BU, 0xA77AU, 0x9719U, 0x8738U, 0xF7DFU, 0xE7FEU, 0xD79DU, 0xC7BCU,
    0x48C4U, 0x58E5U, 0x6886U, 0x78A7U, 0x0840U, 0x1861U, 0x2802U, 0x3823U,
    0xC9CCU, 0xD9EDU, 0xE98EU, 0xF9AFU, 0x8948U, 0x9969U, 0xA90AU, 0xB92BU,
    0x5AF5U, 0x4AD4U, 0x7AB7U, 0x6A96U, 0x1A71U, 0x0A50U, 0x3A33U, 0x2A12U,
    0xDBFDU, 0xCBDCU, 0xFBBFU, 0xEB9EU, 0x9B79U, 0x8B58U, 0xBB3BU, 0xAB1AU,
    0x6CA6U, 0x7C87U, 0x4CE4U, 0x5CC5U, 0x2C22U, 0x3C03U, 0x0C60U, 0x1C41U,
    0xEDAEU, 0xFD8FU, 0xCDECU, 0xDDCDU, 0xAD2AU, 0xBD0BU, 0x8D68U, 0x9D49U,
    0x7E97U, 0x6EB6U, 0x5ED5U, 0x4EF4U, 0x3E13U, 0x2E32U, 0x1E51U, 0x0E70U,
    0xFF9FU, 0xEFBEU, 0xDFDDU, 0xCFFCU, 0xBF1BU, 0xAF3AU, 0x9F59U, 0x8F78U,
    0x9188U, 0x81A9U, 0xB1CAU, 0xA1EBU, 0xD10CU, 0xC12DU, 0xF14EU, 0xE16FU,
    0x1080U, 0x00A1U, 0x30C2U, 0x20E3U, 0x5004U, 0x4025U, 0x7046U, 0x6067U,
    0x83B9U, 0x9398U, 0xA3FBU, 0xB3DAU, 0xC33DU, 0xD31CU, 0xE37FU, 0xF35EU,
    0x02B1U, 0x1290U, 0x22F3U, 0x32D2U, 0x4235U, 0x5214U, 0x6277U, 0x7256U,
    0xB5EAU, 0xA5CBU, 0x95A8U, 0x8589U, 0xF56EU, 0xE54FU, 0xD52CU, 0xC50DU,
    0x34E2U, 0x24C3U, 0x14A0U, 0x0481U, 0x7466U, 0x6447U, 0x5424U, 0x4405U,
    0xA7DBU, 0xB7FAU, 0x8799U, 0x97B8U, 0xE75FU, 0xF77EU, 0xC71DU, 0xD73CU,
    0x26D3U, 0x36F2U, 0x0691U, 0x16B0U, 0x6657U, 0x7676U, 0x4615U, 0x5634U,
    0xD94CU, 0xC96DU, 0xF90EU, 0xE92FU, 0x99C8U, 0x89E9U, 0xB98AU, 0xA9ABU,
    0x5844U, 0x4865U, 0x7806U, 0x6827U, 0x18C0U, 0x08E1U, 0x3882U, 0x28A3U,
    0xCB7DU, 0xDB5CU, 0xEB3FU, 0xFB1EU, 0x8BF9U, 0x9BD8U, 0xABBBU, 0xBB9AU,
    0x4A75U, 0x5A54U, 0x6A37U, 0x7A16U, 0x0AF1U, 0x1AD0U, 0x2AB3U, 0x3A92U,
    0xFD2EU, 0xED0FU, 0xDD6CU, 0xCD4DU, 0xBDAAU, 0xAD8BU, 0x9DE8U, 0x8DC9U,
    0x7C26U, 0x6C07U, 0x5C64U, 0x4C45U, 0x3CA2U, 0x2C83U, 0x1CE0U, 0x0CC1U,
    0xEF1FU, 0xFF3EU, 0xCF5DU, 0xDF7CU, 0xAF9BU, 0xBFBAU, 0x8FD9U, 0x9FF8U,
    0x6E17U, 0x7E36U, 0x4E55U, 0x5E74U, 0x2E93U, 0x3EB2U, 0x0ED1U, 0x1EF0U,
};

static uint16_t SDLogger_CRC16_CCITT(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint32_t i = 0UL; i < length; i++)
    {
        uint8_t index = (uint8_t)(((uint16_t)(crc >> 8)) ^ data[i]);
        crc = (uint16_t)((uint16_t)(crc << 8) ^ crc16_ccitt_table[index]);
    }

    return crc;
}

/* -------------------------------------------------------------------------- */

static uint32_t SDLogger_RingCount(void)
{
    uint32_t head = sd_ring_head;
    uint32_t tail = sd_ring_tail;

    return (head - tail) & SDLOGGER_RING_INDEX_MASK;
}

/* -------------------------------------------------------------------------- */

static uint8_t SDLogger_RingPushFromISR(const SDLoggerFrame_t *frame)
{
    uint32_t head = sd_ring_head;
    uint32_t next = (head + 1UL) & SDLOGGER_RING_INDEX_MASK;

    if (next == sd_ring_tail)
    {
        sd_logger_ring_overrun_count++;
        sd_logger_dropped_frame_count++;
        return 0U;
    }

    sd_capture_ring[head] = *frame;
    __DMB();
    sd_ring_head = next;

    sd_logger_ring_push_count++;

    uint32_t count = SDLogger_RingCount();
    sd_logger_ring_count = count;

    if (count > sd_logger_ring_high_watermark)
    {
        sd_logger_ring_high_watermark = count;
    }

    return 1U;
}

/* -------------------------------------------------------------------------- */

static uint8_t SDLogger_RingPop(SDLoggerFrame_t *frame)
{
    uint32_t tail = sd_ring_tail;

    if (tail == sd_ring_head)
    {
        return 0U;
    }

    *frame = sd_capture_ring[tail];
    __DMB();
    sd_ring_tail = (tail + 1UL) & SDLOGGER_RING_INDEX_MASK;

    sd_logger_ring_pop_count++;
    sd_logger_ring_count = SDLogger_RingCount();

    return 1U;
}

/* -------------------------------------------------------------------------- */

static uint16_t SDLogger_ClampAgeUs(uint32_t age_us)
{
    return (age_us > 65535UL) ? 65535U : (uint16_t)age_us;
}

static void SDLogger_PushFastIMUSample(const SensorData_t *sensor)
{
    SDLoggerFastIMUSample_t *sample;
    uint32_t sample_timestamp_us;

    if (sensor == NULL)
    {
        return;
    }

    sample_timestamp_us = IMU_GetLastSampleTimestampUs();

    if (sample_timestamp_us == 0UL)
    {
        return;
    }

    /*
     * PublishSources normally runs once per 1 kHz IMU task. Guard against a
     * repeated generation so the embedded fast stream never invents samples.
     */
    if (sample_timestamp_us == sd_fast_imu_last_timestamp_us)
    {
        sd_logger_fast_imu_duplicate_skip_count++;
        return;
    }

    sd_fast_imu_last_timestamp_us = sample_timestamp_us;

    sample = &sd_fast_imu_history[sd_fast_imu_history_head];

    sample->timestamp_us = sample_timestamp_us;
    sample->accel_x_raw = sensor->accel_x_raw;
    sample->accel_y_raw = sensor->accel_y_raw;
    sample->accel_z_raw = sensor->accel_z_raw;
    sample->gyro_x_raw = sensor->gyro_x_raw;
    sample->gyro_y_raw = sensor->gyro_y_raw;
    sample->gyro_z_raw = sensor->gyro_z_raw;

    sd_fast_imu_history_head =
        (uint8_t)((sd_fast_imu_history_head + 1U) %
                  SDLOGGER_FAST_IMU_SAMPLES_PER_FRAME);

    if (sd_fast_imu_history_count < SDLOGGER_FAST_IMU_SAMPLES_PER_FRAME)
    {
        sd_fast_imu_history_count++;
    }

    sd_logger_fast_imu_push_count++;
    sd_logger_fast_imu_history_count = sd_fast_imu_history_count;
}

static void SDLogger_CopyFastIMUHistory(SDLoggerSourceSnapshot_t *snapshot)
{
    uint8_t count;
    uint8_t first;
    uint8_t i;

    if (snapshot == NULL)
    {
        return;
    }

    count = sd_fast_imu_history_count;
    snapshot->fast_imu_sample_count = count;
    snapshot->fast_imu_valid_mask = 0U;

    if (count == 0U)
    {
        return;
    }

    first =
        (uint8_t)((sd_fast_imu_history_head +
                   SDLOGGER_FAST_IMU_SAMPLES_PER_FRAME -
                   count) %
                  SDLOGGER_FAST_IMU_SAMPLES_PER_FRAME);

    for (i = 0U; i < count; i++)
    {
        uint8_t src_index =
            (uint8_t)((first + i) % SDLOGGER_FAST_IMU_SAMPLES_PER_FRAME);

        snapshot->fast_imu[i] = sd_fast_imu_history[src_index];

        if (snapshot->fast_imu[i].timestamp_us != 0UL)
        {
            snapshot->fast_imu_valid_mask |= (uint8_t)(1U << i);
        }
    }

    sd_logger_fast_imu_last_valid_mask =
        snapshot->fast_imu_valid_mask;
}

/* -------------------------------------------------------------------------- */

void SDLogger_PushFastIMU(void)
{
#if (APP_SDLOGGER_ENABLED != 0U)
    /* A failed/not-yet-started SD path must cost essentially nothing in the
     * 1 kHz control loop. */
    if ((sd_logger_r10_ram_mode == 0U) &&
        ((sd_logger_ready == 0U) ||
         (sd_logger_logging_active == 0U) ||
         (sd_logger_file_open == 0U)))
    {
        return;
    }

    SDLogger_PushFastIMUSample(SensorManager_GetDataPtr());
#endif
}

/* -------------------------------------------------------------------------- */

void SDLogger_PublishSources(void)
{
#if (APP_SDLOGGER_ENABLED != 0U)
    /* P32: do not spend hundreds of float/int conversions when the card did
     * not mount or logging has stopped. */
    if ((sd_logger_ready == 0U) ||
        (sd_logger_logging_active == 0U) ||
        (sd_logger_file_open == 0U))
    {
        return;
    }
    const SensorData_t *sensor = SensorManager_GetDataPtr();
    const BarometerData_t *barometer = Barometer_GetDataPtr();
    const LidarData_t *lidar = Lidar_GetDataPtr();
    const FullStateESKFData_t *full_eskf = FullStateESKF_GetDataPtr();
    TaragayFlightLogicStatus_t flight_logic = TaragayFlightLogic_GetStatus();

    uint8_t next_index = (uint8_t)(sd_source_active_index ^ 1U);
    SDLoggerSourceSnapshot_t *snapshot = &sd_source_snapshots[next_index];
    uint32_t now_us = micros();

    /* P32: the 1 kHz raw history is maintained separately by
     * SDLogger_PushFastIMU(); this 200 Hz path only builds the coherent frame
     * source snapshot consumed by TIM5. */

    memset(snapshot, 0, sizeof(*snapshot));
    SDLogger_CopyFastIMUHistory(snapshot);

    /* Raw barometer values. */
    snapshot->pressure_pa =
        SDLogger_FloatToInt32(sensor->baro_pressure_pa, 1.0f);

    snapshot->altitude_cm =
        SDLogger_FloatToInt32(sensor->baro_altitude_m, 100.0f);

    /* Filtered barometer values. */
    snapshot->filtered_pressure_pa =
        SDLogger_FloatToInt32(
            sensor->baro_filtered_pressure_pa,
            1.0f
        );

    snapshot->filtered_altitude_mm =
        SDLogger_FloatToInt32(
            sensor->baro_filtered_altitude_m,
            1000.0f
        );

    snapshot->vertical_speed_cms =
        SDLogger_FloatToInt16(
            sensor->baro_vertical_speed_mps,
            100.0f
        );

    snapshot->sensor_update_count = sensor->update_count;
    snapshot->baro_update_count = sensor->baro_update_count;
    snapshot->lidar_update_count = lidar->update_count;

    snapshot->accel_x_raw = sensor->accel_x_raw;
    snapshot->accel_y_raw = sensor->accel_y_raw;
    snapshot->accel_z_raw = sensor->accel_z_raw;

    snapshot->gyro_x_raw = sensor->gyro_x_raw;
    snapshot->gyro_y_raw = sensor->gyro_y_raw;
    snapshot->gyro_z_raw = sensor->gyro_z_raw;

    /*
     * Store filtered engineering values as raw-equivalent int16 counts.
     * This preserves the current IMU resolution without floats in the file.
     */
    snapshot->accel_x_filtered_raw_eq =
        SDLogger_FloatToInt16(
            sensor->accel_x_filtered_g,
            (1.0f / APP_IMU_ACCEL_SCALE_G)
        );

    snapshot->accel_y_filtered_raw_eq =
        SDLogger_FloatToInt16(
            sensor->accel_y_filtered_g,
            (1.0f / APP_IMU_ACCEL_SCALE_G)
        );

    snapshot->accel_z_filtered_raw_eq =
        SDLogger_FloatToInt16(
            sensor->accel_z_filtered_g,
            (1.0f / APP_IMU_ACCEL_SCALE_G)
        );

    snapshot->gyro_x_filtered_raw_eq =
        SDLogger_FloatToInt16(
            sensor->gyro_x_filtered_dps,
            (1.0f / APP_IMU_GYRO_SCALE_DPS)
        );

    snapshot->gyro_y_filtered_raw_eq =
        SDLogger_FloatToInt16(
            sensor->gyro_y_filtered_dps,
            (1.0f / APP_IMU_GYRO_SCALE_DPS)
        );

    snapshot->gyro_z_filtered_raw_eq =
        SDLogger_FloatToInt16(
            sensor->gyro_z_filtered_dps,
            (1.0f / APP_IMU_GYRO_SCALE_DPS)
        );

    snapshot->accel_filtered_norm_mg =
        SDLogger_FloatToUInt16(
            sensor->accel_filtered_norm_g,
            1000.0f
        );

    snapshot->filter_flags = 0U;

    if (sensor_imu_filter_enabled != 0U)
    {
        snapshot->filter_flags |= (1U << 0);
    }

    if (sensor_imu_filter_config_ok != 0U)
    {
        snapshot->filter_flags |= (1U << 1);
    }

    if (sensor_imu_filter_initialized != 0U)
    {
        snapshot->filter_flags |= (1U << 2);
    }

    snapshot->sensor_filter_flags = 0U;

    if (barometer->filter_enabled != 0U)
    {
        snapshot->sensor_filter_flags |= (1U << 0);
    }

    if (barometer->filter_config_ok != 0U)
    {
        snapshot->sensor_filter_flags |= (1U << 1);
    }

    if (barometer->filter_initialized != 0U)
    {
        snapshot->sensor_filter_flags |= (1U << 2);
    }

    if (lidar->median_initialized != 0U)
    {
        snapshot->sensor_filter_flags |= (1U << 3);
    }

    if (lidar->filter_enabled != 0U)
    {
        snapshot->sensor_filter_flags |= (1U << 4);
    }

    if (lidar->filter_config_ok != 0U)
    {
        snapshot->sensor_filter_flags |= (1U << 5);
    }

    if (lidar->filter_initialized != 0U)
    {
        snapshot->sensor_filter_flags |= (1U << 6);
    }

    if (sensor_imu_filter_reset_count > 65535UL)
    {
        snapshot->filter_reset_count = 65535U;
    }
    else
    {
        snapshot->filter_reset_count =
            (uint16_t)sensor_imu_filter_reset_count;
    }

    snapshot->baro_median_delta_pa =
        SDLogger_FloatToInt16(
            barometer->median_pressure_pa - barometer->pressure_pa,
            1.0f
        );

    snapshot->temperature_cx100 =
        SDLogger_FloatToInt16(sensor->baro_temperature_c, 100.0f);

    snapshot->lidar_distance_cm = lidar->distance_cm;

    snapshot->lidar_median_mm =
        SDLogger_FloatToUInt16(
            lidar->median_distance_m,
            1000.0f
        );

    snapshot->lidar_filtered_mm =
        SDLogger_FloatToUInt16(
            lidar->filtered_distance_m,
            1000.0f
        );

    /*
     * V10 preserves all V9 state/counter fields and appends DWT cycle data.
     * The logger packs 24 V14 frames per 9216-byte non-blocking raw-SD DMA write.
     */
    snapshot->full_eskf_position_z_mm =
        SDLogger_FloatToInt32(full_eskf->position_z_m, 1000.0f);

    snapshot->full_eskf_velocity_z_mms =
        SDLogger_FloatToInt32(full_eskf->velocity_z_mps, 1000.0f);

    snapshot->full_eskf_gyro_bias_x_mdps =
        SDLogger_FloatToInt16(full_eskf->gyro_bias_x_dps, 1000.0f);
    snapshot->full_eskf_gyro_bias_y_mdps =
        SDLogger_FloatToInt16(full_eskf->gyro_bias_y_dps, 1000.0f);
    snapshot->full_eskf_gyro_bias_z_mdps =
        SDLogger_FloatToInt16(full_eskf->gyro_bias_z_dps, 1000.0f);

    snapshot->full_eskf_flags = 0U;

    if (full_eskf->enabled != 0U)
    {
        snapshot->full_eskf_flags |= (1U << 0);
    }

    if (full_eskf->initialized != 0U)
    {
        snapshot->full_eskf_flags |= (1U << 1);
    }

    if (full_eskf->healthy != 0U)
    {
        snapshot->full_eskf_flags |= (1U << 2);
    }

    if (full_eskf->baro_reference_ready != 0U)
    {
        snapshot->full_eskf_flags |= (1U << 3);
    }

    if (full_eskf->lidar_reference_ready != 0U)
    {
        snapshot->full_eskf_flags |= (1U << 4);
    }

    if (full_eskf->gravity_correction_active != 0U)
    {
        snapshot->full_eskf_flags |= (1U << 5);
    }

    if (full_eskf->stationary_detected != 0U)
    {
        snapshot->full_eskf_flags |= (1U << 6);
    }

    if (full_eskf->shadow_mode != 0U)
    {
        snapshot->full_eskf_flags |= (1U << 7);
    }

    snapshot->full_eskf_reset_count =
        (full_eskf->reset_count > 255UL) ?
        255U : (uint8_t)full_eskf->reset_count;

    snapshot->full_eskf_position_x_cm =
        SDLogger_FloatToInt16(full_eskf->position_x_m, 100.0f);
    snapshot->full_eskf_position_y_cm =
        SDLogger_FloatToInt16(full_eskf->position_y_m, 100.0f);

    snapshot->full_eskf_velocity_x_cms =
        SDLogger_FloatToInt16(full_eskf->velocity_x_mps, 100.0f);
    snapshot->full_eskf_velocity_y_cms =
        SDLogger_FloatToInt16(full_eskf->velocity_y_mps, 100.0f);

    snapshot->full_eskf_accel_bias_x_mms2 =
        SDLogger_FloatToInt16(full_eskf->accel_bias_x_mps2, 1000.0f);
    snapshot->full_eskf_accel_bias_y_mms2 =
        SDLogger_FloatToInt16(full_eskf->accel_bias_y_mps2, 1000.0f);
    snapshot->full_eskf_accel_bias_z_mms2 =
        SDLogger_FloatToInt16(full_eskf->accel_bias_z_mps2, 1000.0f);

    snapshot->full_eskf_q_w_q15 =
        SDLogger_FloatToInt16(full_eskf->q_w, 32767.0f);
    snapshot->full_eskf_q_x_q15 =
        SDLogger_FloatToInt16(full_eskf->q_x, 32767.0f);
    snapshot->full_eskf_q_y_q15 =
        SDLogger_FloatToInt16(full_eskf->q_y, 32767.0f);
    snapshot->full_eskf_q_z_q15 =
        SDLogger_FloatToInt16(full_eskf->q_z, 32767.0f);

    snapshot->full_eskf_roll_cdeg =
        SDLogger_FloatToInt16(full_eskf->roll_deg, 100.0f);
    snapshot->full_eskf_pitch_cdeg =
        SDLogger_FloatToInt16(full_eskf->pitch_deg, 100.0f);
    snapshot->full_eskf_yaw_cdeg =
        SDLogger_FloatToInt16(full_eskf->yaw_deg, 100.0f);

    snapshot->full_eskf_world_accel_x_cms2 =
        SDLogger_FloatToInt16(
            full_eskf->world_linear_accel_x_mps2,
            100.0f
        );

    snapshot->full_eskf_world_accel_y_cms2 =
        SDLogger_FloatToInt16(
            full_eskf->world_linear_accel_y_mps2,
            100.0f
        );

    snapshot->full_eskf_world_accel_z_cms2 =
        SDLogger_FloatToInt16(
            full_eskf->world_linear_accel_z_mps2,
            100.0f
        );

    snapshot->full_eskf_attitude_flags = 0U;

    if (full_eskf->enabled != 0U)
    {
        snapshot->full_eskf_attitude_flags |= (1U << 0);
    }

    if (full_eskf->initialized != 0U)
    {
        snapshot->full_eskf_attitude_flags |= (1U << 1);
    }

    if (full_eskf->healthy != 0U)
    {
        snapshot->full_eskf_attitude_flags |= (1U << 2);
    }

    if (full_eskf->gravity_correction_active != 0U)
    {
        snapshot->full_eskf_attitude_flags |= (1U << 3);
    }

    if (full_eskf->stationary_detected != 0U)
    {
        snapshot->full_eskf_attitude_flags |= (1U << 4);
    }

    /* V8.15B: use previously reserved flag bits; V13 frame size is unchanged. */
    if (full_eskf->horizontal_position_valid != 0U)
    {
        snapshot->full_eskf_attitude_flags |= (1U << 5);
    }

    if (full_eskf->vertical_position_valid != 0U)
    {
        snapshot->full_eskf_attitude_flags |= (1U << 6);
    }

    if (full_eskf->origin_zeroed != 0U)
    {
        snapshot->full_eskf_attitude_flags |= (1U << 7);
    }

    snapshot->full_eskf_gravity_weight_x255 =
        (uint8_t)SDLogger_FloatToUInt16(
            full_eskf->gravity_correction_weight,
            255.0f
        );

    snapshot->full_eskf_predict_count_low =
        (uint16_t)(full_eskf->predict_count & 0xFFFFUL);

    snapshot->full_eskf_public_output_count_low =
        (uint16_t)(full_eskf->public_output_count & 0xFFFFUL);

    snapshot->full_eskf_covariance_predict_count_low =
        (uint16_t)(full_eskf->covariance_predict_count & 0xFFFFUL);

    snapshot->full_eskf_gravity_update_count_low =
        (uint16_t)(full_eskf->gravity_update_count & 0xFFFFUL);

    snapshot->full_eskf_stationary_detector_count_low =
        (uint16_t)(full_eskf->stationary_detector_count & 0xFFFFUL);

    snapshot->full_eskf_stationary_update_count_low =
        (uint16_t)(full_eskf->stationary_update_count & 0xFFFFUL);

    snapshot->full_eskf_max_predict_cycles = full_eskf_max_predict_cycles;
    snapshot->full_eskf_total_predict_cycles = full_eskf_total_predict_cycles;
    snapshot->full_eskf_max_public_output_cycles =
        full_eskf_max_public_output_cycles;
    snapshot->full_eskf_total_public_output_cycles =
        full_eskf_total_public_output_cycles;
    snapshot->full_eskf_max_correction_cycles =
        full_eskf_max_correction_cycles;
    snapshot->full_eskf_total_correction_cycles =
        full_eskf_total_correction_cycles;
    snapshot->full_eskf_max_covariance_cycles =
        full_eskf_max_covariance_cycles;
    snapshot->full_eskf_total_covariance_cycles =
        full_eskf_total_covariance_cycles;
    snapshot->full_eskf_max_gravity_cycles = full_eskf_max_gravity_cycles;
    snapshot->full_eskf_total_gravity_cycles =
        full_eskf_total_gravity_cycles;
    snapshot->full_eskf_max_stationary_cycles =
        full_eskf_max_stationary_cycles;
    snapshot->full_eskf_total_stationary_cycles =
        full_eskf_total_stationary_cycles;
    snapshot->full_eskf_max_baro_cycles = full_eskf_max_baro_cycles;
    snapshot->full_eskf_total_baro_cycles = full_eskf_total_baro_cycles;
    snapshot->full_eskf_max_lidar_cycles = full_eskf_max_lidar_cycles;
    snapshot->full_eskf_total_lidar_cycles = full_eskf_total_lidar_cycles;
    snapshot->full_eskf_cpu_clock_hz = full_eskf_cpu_clock_hz;

    /* V11 IMU watchdog/recovery diagnostics. */
    snapshot->imu_sample_valid = IMU_IsLastSampleValid();
    snapshot->imu_recovery_state = IMU_GetRecoveryState();
    snapshot->rcs_imu_inhibit = IMU_ShouldInhibitRCS();

    snapshot->imu_diag_flags = 0U;
    if (snapshot->imu_sample_valid != 0U)
    {
        snapshot->imu_diag_flags |= (1U << 0);
    }
    if (snapshot->imu_recovery_state != IMU_RECOVERY_STATE_NORMAL)
    {
        snapshot->imu_diag_flags |= (1U << 1);
    }
    if (snapshot->rcs_imu_inhibit != 0U)
    {
        snapshot->imu_diag_flags |= (1U << 2);
    }

    snapshot->imu_stale_count = IMU_GetStaleCount();
    snapshot->imu_pattern_error_count = IMU_GetPatternErrorCount();
    snapshot->imu_dma_timeout_count = IMU_GetDmaTimeoutCount();
    snapshot->imu_recovery_count = IMU_GetRecoveryCount();
    snapshot->imu_register_error_count = IMU_GetRegisterErrorCount();

    /*
     * V12 needle telemetry.
     * Do not call NeedleValveController_GetStatus() here because it briefly
     * masks interrupts. The 200 Hz TIM7 controller must remain untouched by
     * logging; these scalar volatile reads are sufficient for a 1 kHz logger
     * source snapshot.
     */
    snapshot->needle_requested_cmd_x10000 =
        SDLogger_FloatToUInt16(needle_valve_requested_cmd, 10000.0f);
    snapshot->needle_limited_cmd_x10000 =
        SDLogger_FloatToUInt16(needle_valve_limited_cmd, 10000.0f);

    snapshot->needle_raw_adc = needle_valve_raw_adc;
    snapshot->needle_zero_adc = needle_valve_zero_adc;
    snapshot->needle_target_adc = needle_valve_target_adc;
    snapshot->needle_error_adc = needle_valve_error_adc;
    snapshot->needle_rpwm = needle_valve_rpwm;
    snapshot->needle_lpwm = needle_valve_lpwm;

    snapshot->needle_state_flags = 0U;
    if (needle_valve_enabled != 0U)
    {
        snapshot->needle_state_flags |= (1U << 0);
    }
    if (needle_valve_zero_valid != 0U)
    {
        snapshot->needle_state_flags |= (1U << 1);
    }
    if (needle_valve_lock != 0U)
    {
        snapshot->needle_state_flags |= (1U << 2);
    }
    if (needle_valve_homing_active != 0U)
    {
        snapshot->needle_state_flags |= (1U << 3);
    }
    if (needle_valve_homing_complete != 0U)
    {
        snapshot->needle_state_flags |= (1U << 4);
    }
    if ((needle_valve_rpwm != 0U) || (needle_valve_lpwm != 0U))
    {
        snapshot->needle_state_flags |= (1U << 5);
    }
    if (needle_valve_lpwm != 0U)
    {
        snapshot->needle_state_flags |= (1U << 6); /* OPEN */
    }
    if (needle_valve_rpwm != 0U)
    {
        snapshot->needle_state_flags |= (1U << 7); /* CLOSE */
    }
    snapshot->needle_fault = needle_valve_fault;

    /* V16 read-only motor diagnostic snapshot. No controller API is called
     * here; only already-published volatile telemetry is copied. */
    snapshot->motor_ext.position_turns_x10000 =
        SDLogger_FloatToUInt16(needle_valve_position_turns, 10000.0f);
    snapshot->motor_ext.max_turns_x10000 =
        SDLogger_FloatToUInt16(needle_valve_max_turns, 10000.0f);
    snapshot->motor_ext.max_open_adc = needle_valve_max_open_adc;
    snapshot->motor_ext.adc_per_turn = needle_valve_adc_per_turn;
    snapshot->motor_ext.max_travel_adc = needle_valve_max_travel_adc;
    snapshot->motor_ext.homing_start_adc = needle_valve_homing_start_adc;
    snapshot->motor_ext.hw_adc_raw12 = needle_valve_hw_adc_raw12;
    snapshot->motor_ext.hw_adc_mv = needle_valve_hw_adc_mv;
    snapshot->motor_ext.stall_ms = needle_valve_stall_ms;
    snapshot->motor_ext.homing_elapsed_ms = needle_valve_homing_elapsed_ms;
    snapshot->motor_ext.control_tick_count = needle_valve_control_tick_count;
    snapshot->motor_ext.adc_invalid_count = needle_valve_adc_invalid_count;

    snapshot->motor_ext.p83_adc1_raw = p83_adc1_raw;
    snapshot->motor_ext.p83_adc2_raw = p83_adc2_raw;
    snapshot->motor_ext.p83_pair_diff = p83_pair_diff;
    snapshot->motor_ext.p83_pair_candidate = p83_pair_candidate;
    snapshot->motor_ext.p83_median7 = p83_median7;
    snapshot->motor_ext.p83_filtered_adc = p83_filtered_adc;
    snapshot->motor_ext.p83_feedback_valid = p83_feedback_valid;
    snapshot->motor_ext.p83_confidence_pct = p83_confidence_pct;
    snapshot->motor_ext.p83_mode = p83_mode;
    snapshot->motor_ext.p83_acq_progress_pct = p83_acq_progress_pct;
    snapshot->motor_ext.p83_pair_reject_count = p83_pair_reject_count;
    snapshot->motor_ext.p83_rate_reject_count = p83_rate_reject_count;
    snapshot->motor_ext.p83_quarantine_count = p83_quarantine_count;
    snapshot->motor_ext.p83_reacquire_count = p83_reacquire_count;
    snapshot->motor_ext.p83_adc_timeout_count = p83_adc_timeout_count;

    snapshot->motor_ext.p110_start_adc = p110_start_adc;
    snapshot->motor_ext.p110_target_adc = p110_target_adc;
    snapshot->motor_ext.p110_current_adc = p110_current_adc;
    snapshot->motor_ext.p110_error_adc = p110_error_adc;
    snapshot->motor_ext.p110_speed_adc_s = p110_speed_adc_s;
    snapshot->motor_ext.p110_stop_distance_adc = p110_stop_distance_adc;
    snapshot->motor_ext.p110_brake_entry_adc = p110_brake_entry_adc;
    snapshot->motor_ext.p110_final_adc = p110_final_adc;
    snapshot->motor_ext.p110_final_error_adc = p110_final_error_adc;
    snapshot->motor_ext.p110_total_powered_ms = p110_total_powered_ms;
    snapshot->motor_ext.p110_state = p110_state;
    snapshot->motor_ext.p110_result = p110_result;
    snapshot->motor_ext.p110_direction = p110_direction;
    snapshot->motor_ext.p110_active_pwm = p110_active_pwm;
    snapshot->motor_ext.p110_abort_reason = p110_abort_reason;
    snapshot->motor_ext.p111_state = p111_state;
    snapshot->motor_ext.p111_result = p111_result;
    snapshot->motor_ext.reserved = 0U;

    sd_logger_needle_requested_cmd_x10000 =
        snapshot->needle_requested_cmd_x10000;
    sd_logger_needle_limited_cmd_x10000 =
        snapshot->needle_limited_cmd_x10000;
    sd_logger_needle_raw_adc = snapshot->needle_raw_adc;
    sd_logger_needle_zero_adc = snapshot->needle_zero_adc;
    sd_logger_needle_target_adc = snapshot->needle_target_adc;
    sd_logger_needle_error_adc = snapshot->needle_error_adc;
    sd_logger_needle_rpwm = snapshot->needle_rpwm;
    sd_logger_needle_lpwm = snapshot->needle_lpwm;
    sd_logger_needle_state_flags = snapshot->needle_state_flags;
    sd_logger_needle_fault = snapshot->needle_fault;

    {
        NRF24_Data_t nrf_live = NRF24_GetData();
        uint32_t packet_age = remote_rx_last_packet_age_ms;

        snapshot->nrf_link_active = RemoteControl_IsLinkActive();
        snapshot->nrf_command = RemoteControl_GetCommand();
        snapshot->nrf_last_sequence = remote_rx_last_sequence;
        snapshot->servo_target_open = vent_servo_target_open;
        snapshot->servo_pulse_us = vent_servo_pulse_us;

        if (packet_age > 65535UL)
        {
            packet_age = 65535UL;
        }
        snapshot->nrf_last_packet_age_ms = (uint16_t)packet_age;

        snapshot->nrf_valid_packet_count = remote_rx_valid_packet_count;
        snapshot->nrf_irq_count_low =
            (uint16_t)(remote_rx_irq_count & 0xFFFFUL);
        snapshot->nrf_rx_count_low =
            (uint16_t)(nrf_live.rx_count & 0xFFFFUL);

        sd_logger_nrf_link_active = snapshot->nrf_link_active;
        sd_logger_nrf_command = snapshot->nrf_command;
        sd_logger_nrf_last_sequence = snapshot->nrf_last_sequence;
        sd_logger_servo_target_open = snapshot->servo_target_open;
        sd_logger_servo_pulse_us = snapshot->servo_pulse_us;
        sd_logger_nrf_last_packet_age_ms = snapshot->nrf_last_packet_age_ms;
        sd_logger_nrf_valid_packet_count = snapshot->nrf_valid_packet_count;
        sd_logger_nrf_irq_count_low = snapshot->nrf_irq_count_low;
        sd_logger_nrf_rx_count_low = snapshot->nrf_rx_count_low;
    }

    if (cpu_load_percent_x100 > 65535UL)
    {
        snapshot->cpu_load_x100 = 65535U;
    }
    else
    {
        snapshot->cpu_load_x100 = (uint16_t)cpu_load_percent_x100;
    }

    if ((sensor->imu_valid != 0U) &&
        (snapshot->imu_sample_valid != 0U) &&
        (snapshot->rcs_imu_inhibit == 0U))
    {
        snapshot->health_flags |= SDLOGGER_HEALTH_IMU_OK;
    }

    if (sensor->baro_valid != 0U)
    {
        snapshot->health_flags |= SDLOGGER_HEALTH_BARO_OK;
    }

    if ((lidar->connected != 0U) && (lidar->distance_valid != 0U))
    {
        snapshot->health_flags |= SDLOGGER_HEALTH_LIDAR_OK;
    }

    if (sm_system_ok != 0U)
    {
        snapshot->health_flags |= SDLOGGER_HEALTH_SYSTEM_OK;
    }

    if (sd_logger_ready != 0U)
    {
        snapshot->health_flags |= SDLOGGER_HEALTH_SD_OK;
    }

    if (((snapshot->needle_state_flags & (1U << 1)) != 0U) &&
        (snapshot->needle_fault == NEEDLE_VALVE_FAULT_NONE))
    {
        snapshot->health_flags |= SDLOGGER_HEALTH_NEEDLE_OK;
    }

    {
        NRF24_Data_t nrf_health = NRF24_GetData();

        if ((nrf_health.initialized != 0U) &&
            (nrf_health.connected != 0U) &&
            (nrf_health.mode == NRF24_MODE_RX))
        {
            snapshot->health_flags |= SDLOGGER_HEALTH_NRF_OK;
        }
    }

    if ((vent_servo_init_ok != 0U) &&
        (snapshot->servo_pulse_us >= APP_VENT_SERVO_MIN_PULSE_US) &&
        (snapshot->servo_pulse_us <= APP_VENT_SERVO_MAX_PULSE_US))
    {
        snapshot->health_flags |= SDLOGGER_HEALTH_SERVO_OK;
    }

    snapshot->fault_code = sm_fault_code;

    /* Freshness and qualification metadata captured with the source snapshot. */
    {
        uint32_t imu_timestamp_us = IMU_GetLastSampleTimestampUs();

        if (imu_timestamp_us != 0UL)
        {
            snapshot->imu_sample_age_us =
                SDLogger_ClampAgeUs(now_us - imu_timestamp_us);
        }
        else
        {
            snapshot->imu_sample_age_us = 65535U;
        }
    }

    if (barometer->last_sample_timestamp_us != 0UL)
    {
        snapshot->baro_sample_age_us =
            SDLogger_ClampAgeUs(now_us - barometer->last_sample_timestamp_us);
    }
    else
    {
        snapshot->baro_sample_age_us = 65535U;
    }

    if (lidar->last_sample_timestamp_us != 0UL)
    {
        snapshot->lidar_sample_age_us =
            SDLogger_ClampAgeUs(now_us - lidar->last_sample_timestamp_us);
    }
    else
    {
        snapshot->lidar_sample_age_us = 65535U;
    }

    snapshot->sensor_qual_state = sensor_qual_state;
    snapshot->sensor_qual_good_windows = sensor_qual_good_windows;
    snapshot->sensor_qual_flags_low = (uint16_t)(sensor_qual_flags & 0xFFFFUL);
    snapshot->sensor_qual_flags_high =
        (uint16_t)((sensor_qual_flags >> 16) & 0xFFFFUL);

    snapshot->mission_state = flight_logic.mission_state;
    snapshot->flight_input_valid = flight_logic.input_valid;
    snapshot->flight_rcs_requested_mask = flight_logic.rcs_requested_mask;
    snapshot->flight_rcs_applied_mask = flight_logic.rcs_applied_mask;
    snapshot->flight_rcs_fault = flight_logic.rcs_fault;
    snapshot->mount_cal_valid = flight_logic.mount_cal_valid;
    snapshot->flight_valve_cmd_x10000 =
        SDLogger_FloatToUInt16(flight_logic.valve_cmd, 10000.0f);
    snapshot->flight_pitch_rate_cdeg_s =
        SDLogger_FloatToInt16(flight_logic.input_pitch_rate_dps, 100.0f);
    snapshot->flight_yaw_rate_cdeg_s =
        SDLogger_FloatToInt16(flight_logic.input_yaw_rate_dps, 100.0f);

    sd_logger_last_imu_age_us = snapshot->imu_sample_age_us;
    sd_logger_last_baro_age_us = snapshot->baro_sample_age_us;
    sd_logger_last_lidar_age_us = snapshot->lidar_sample_age_us;

    /* Publish only after the inactive copy is complete. */
    __DMB();
    sd_source_active_index = next_index;
    sd_logger_source_active_index = next_index;
    sd_logger_source_publish_count++;
#else
    return;
#endif
}

/* -------------------------------------------------------------------------- */

void SDLogger_TimerCaptureISR(void)
{
#if (APP_SDLOGGER_ENABLED != 0U)
    uint32_t isr_start_us = micros();
    uint32_t capture_us = isr_start_us;

    sd_logger_timer_irq_count++;

    if (sd_timer_previous_capture_us != 0UL)
    {
        uint32_t interval_us = capture_us - sd_timer_previous_capture_us;
        sd_logger_timer_last_interval_us = interval_us;

        if ((sd_logger_timer_min_interval_us == 0UL) ||
            (interval_us < sd_logger_timer_min_interval_us))
        {
            sd_logger_timer_min_interval_us = interval_us;
        }

        if (interval_us > sd_logger_timer_max_interval_us)
        {
            sd_logger_timer_max_interval_us = interval_us;
        }

        if (interval_us >= (APP_SDLOGGER_SAMPLE_PERIOD_US * 2UL))
        {
            sd_logger_missed_period_count +=
                (interval_us / APP_SDLOGGER_SAMPLE_PERIOD_US) - 1UL;
        }
    }

    sd_timer_previous_capture_us = capture_us;

    if (sd_logger_r10_ram_mode != 0U)
    {
        /* Keep the proven TIM5 rate and RAM-first flight architecture.
         *
         * Background flight telemetry remains decimated by the existing
         * APP_SDLOGGER_R10_RAM_DECIMATION value.  While the needle motor is
         * actually moving/homing or P110 owns an active move, capture EVERY
         * 30 Hz TIM5 sample instead.  This adds no SDIO traffic during motor
         * activity: records still stay in CCM RAM and are replayed only after
         * touchdown/latched STOP + the existing actuator-quiet gate.
         *
         * Result: full V16 motor diagnostics (ADC, target, PWM, direction,
         * P83 feedback, P110 state/speed/result, position-turn estimate, etc.)
         * are not reduced to 7.5 Hz during the actual motor motion. */
        uint8_t source_index = sd_source_active_index;
        uint8_t keep_sample;
        uint8_t motor_detail_active;
        __DMB();
        SDLoggerSourceSnapshot_t source = sd_source_snapshots[source_index];

        motor_detail_active =
            ((source.needle_rpwm != 0U) ||
             (source.needle_lpwm != 0U) ||
             ((source.needle_state_flags &
               ((1U << 3) | (1U << 5) | (1U << 6) | (1U << 7))) != 0U) ||
             ((source.motor_ext.p110_state >= 1U) &&
              (source.motor_ext.p110_state <= 4U))) ? 1U : 0U;

        if (motor_detail_active != 0U)
        {
            keep_sample = 1U;
            /* Make the first post-motion background sample immediate too. */
            sd_r10_ram_decimation_phase = 0U;
        }
        else
        {
            keep_sample =
                (sd_r10_ram_decimation_phase == 0U) ? 1U : 0U;

            sd_r10_ram_decimation_phase++;
            if (sd_r10_ram_decimation_phase >= APP_SDLOGGER_R10_RAM_DECIMATION)
            {
                sd_r10_ram_decimation_phase = 0U;
            }
        }

        if (keep_sample != 0U)
        {
            SDLogger_R10CaptureCompactFromISR(&source, capture_us);
        }

        uint32_t duration_us = micros() - isr_start_us;
        sd_logger_timer_last_isr_duration_us = duration_us;
        if (duration_us > sd_logger_timer_max_isr_duration_us)
        {
            sd_logger_timer_max_isr_duration_us = duration_us;
        }
        return;
    }

    if ((sd_logger_logging_active != 0U) &&
        (sd_logger_file_open != 0U) &&
        (sd_logger_ready != 0U))
    {
        SDLoggerFrame_t frame;
        uint8_t source_index = sd_source_active_index;
        __DMB();
        SDLoggerSourceSnapshot_t source =
            sd_source_snapshots[source_index];

        frame.magic = SDLOGGER_MAGIC;
        frame.sequence = sd_sequence;
        frame.timestamp_us = capture_us;
        frame.timestamp_ms = HAL_GetTick();

        frame.pressure_pa = source.pressure_pa;
        frame.altitude_cm = source.altitude_cm;

        frame.sensor_update_count = source.sensor_update_count;
        frame.baro_update_count = source.baro_update_count;
        frame.lidar_update_count = source.lidar_update_count;

        frame.format_version = SDLOGGER_FORMAT_VERSION;
        frame.frame_size = SDLOGGER_FRAME_SIZE;

        frame.accel_x_raw = source.accel_x_raw;
        frame.accel_y_raw = source.accel_y_raw;
        frame.accel_z_raw = source.accel_z_raw;

        frame.gyro_x_raw = source.gyro_x_raw;
        frame.gyro_y_raw = source.gyro_y_raw;
        frame.gyro_z_raw = source.gyro_z_raw;

        frame.temperature_cx100 = source.temperature_cx100;
        frame.lidar_distance_cm = source.lidar_distance_cm;
        frame.cpu_load_x100 = source.cpu_load_x100;
        frame.health_flags = source.health_flags;
        frame.fault_code = source.fault_code;

        frame.accel_x_filtered_raw_eq =
            source.accel_x_filtered_raw_eq;
        frame.accel_y_filtered_raw_eq =
            source.accel_y_filtered_raw_eq;
        frame.accel_z_filtered_raw_eq =
            source.accel_z_filtered_raw_eq;

        frame.gyro_x_filtered_raw_eq =
            source.gyro_x_filtered_raw_eq;
        frame.gyro_y_filtered_raw_eq =
            source.gyro_y_filtered_raw_eq;
        frame.gyro_z_filtered_raw_eq =
            source.gyro_z_filtered_raw_eq;

        frame.accel_filtered_norm_mg =
            source.accel_filtered_norm_mg;

        frame.filter_flags = source.filter_flags;
        frame.sensor_filter_flags = source.sensor_filter_flags;
        frame.filter_reset_count = source.filter_reset_count;

        frame.baro_median_delta_pa = source.baro_median_delta_pa;

        frame.filtered_pressure_pa = source.filtered_pressure_pa;
        frame.filtered_altitude_mm = source.filtered_altitude_mm;
        frame.vertical_speed_cms = source.vertical_speed_cms;

        frame.lidar_median_mm = source.lidar_median_mm;
        frame.lidar_filtered_mm = source.lidar_filtered_mm;

        frame.full_eskf_position_z_mm =
            source.full_eskf_position_z_mm;
        frame.full_eskf_velocity_z_mms =
            source.full_eskf_velocity_z_mms;

        frame.full_eskf_gyro_bias_x_mdps =
            source.full_eskf_gyro_bias_x_mdps;
        frame.full_eskf_gyro_bias_y_mdps =
            source.full_eskf_gyro_bias_y_mdps;
        frame.full_eskf_gyro_bias_z_mdps =
            source.full_eskf_gyro_bias_z_mdps;

        frame.ekf_alignment_reserved =
            (source.motor_ext.stall_ms > 65535UL) ? 65535U :
            (uint16_t)source.motor_ext.stall_ms;
        frame.full_eskf_flags = source.full_eskf_flags;
        frame.full_eskf_reset_count = source.full_eskf_reset_count;

        frame.full_eskf_position_x_cm =
            source.full_eskf_position_x_cm;
        frame.full_eskf_position_y_cm =
            source.full_eskf_position_y_cm;
        frame.full_eskf_velocity_x_cms =
            source.full_eskf_velocity_x_cms;
        frame.full_eskf_velocity_y_cms =
            source.full_eskf_velocity_y_cms;

        frame.full_eskf_accel_bias_x_mms2 =
            source.full_eskf_accel_bias_x_mms2;
        frame.full_eskf_accel_bias_y_mms2 =
            source.full_eskf_accel_bias_y_mms2;
        frame.full_eskf_accel_bias_z_mms2 =
            source.full_eskf_accel_bias_z_mms2;

        frame.full_eskf_q_w_q15 = source.full_eskf_q_w_q15;
        frame.full_eskf_q_x_q15 = source.full_eskf_q_x_q15;
        frame.full_eskf_q_y_q15 = source.full_eskf_q_y_q15;
        frame.full_eskf_q_z_q15 = source.full_eskf_q_z_q15;

        frame.full_eskf_roll_cdeg = source.full_eskf_roll_cdeg;
        frame.full_eskf_pitch_cdeg = source.full_eskf_pitch_cdeg;
        frame.full_eskf_yaw_cdeg = source.full_eskf_yaw_cdeg;

        frame.full_eskf_world_accel_x_cms2 = source.full_eskf_world_accel_x_cms2;
        frame.full_eskf_world_accel_y_cms2 = source.full_eskf_world_accel_y_cms2;
        frame.full_eskf_world_accel_z_cms2 = source.full_eskf_world_accel_z_cms2;

        frame.full_eskf_attitude_flags = source.full_eskf_attitude_flags;
        frame.full_eskf_gravity_weight_x255 =
            source.full_eskf_gravity_weight_x255;
        frame.full_eskf_predict_count_low =
            source.full_eskf_predict_count_low;
        frame.full_eskf_public_output_count_low =
            source.full_eskf_public_output_count_low;
        frame.full_eskf_covariance_predict_count_low =
            source.full_eskf_covariance_predict_count_low;
        frame.full_eskf_gravity_update_count_low =
            source.full_eskf_gravity_update_count_low;
        frame.full_eskf_stationary_detector_count_low =
            source.full_eskf_stationary_detector_count_low;
        frame.full_eskf_stationary_update_count_low =
            source.full_eskf_stationary_update_count_low;

        frame.full_eskf_max_predict_cycles =
            source.full_eskf_max_predict_cycles;
        frame.full_eskf_total_predict_cycles =
            source.full_eskf_total_predict_cycles;
        frame.full_eskf_max_public_output_cycles =
            source.full_eskf_max_public_output_cycles;
        frame.full_eskf_total_public_output_cycles =
            source.full_eskf_total_public_output_cycles;
        frame.full_eskf_max_correction_cycles =
            source.full_eskf_max_correction_cycles;
        frame.full_eskf_total_correction_cycles =
            source.full_eskf_total_correction_cycles;
        frame.full_eskf_max_covariance_cycles =
            source.full_eskf_max_covariance_cycles;
        frame.full_eskf_total_covariance_cycles =
            source.full_eskf_total_covariance_cycles;
        frame.full_eskf_max_gravity_cycles =
            source.full_eskf_max_gravity_cycles;
        frame.full_eskf_total_gravity_cycles =
            source.full_eskf_total_gravity_cycles;
        frame.full_eskf_max_stationary_cycles =
            source.full_eskf_max_stationary_cycles;
        frame.full_eskf_total_stationary_cycles =
            source.full_eskf_total_stationary_cycles;
        frame.full_eskf_max_baro_cycles =
            source.full_eskf_max_baro_cycles;
        frame.full_eskf_total_baro_cycles =
            source.full_eskf_total_baro_cycles;
        frame.full_eskf_max_lidar_cycles =
            source.full_eskf_max_lidar_cycles;
        frame.full_eskf_total_lidar_cycles =
            source.full_eskf_total_lidar_cycles;
        frame.full_eskf_cpu_clock_hz = source.full_eskf_cpu_clock_hz;

        frame.imu_sample_valid = source.imu_sample_valid;
        frame.imu_recovery_state = source.imu_recovery_state;
        frame.rcs_imu_inhibit = source.rcs_imu_inhibit;
        frame.imu_diag_flags = source.imu_diag_flags;

        frame.imu_stale_count = source.imu_stale_count;
        frame.imu_pattern_error_count = source.imu_pattern_error_count;
        frame.imu_dma_timeout_count = source.imu_dma_timeout_count;
        frame.imu_recovery_count = source.imu_recovery_count;
        frame.imu_register_error_count = source.imu_register_error_count;
        frame.imu_diag_reserved =
            (source.motor_ext.homing_elapsed_ms > 65535UL) ? 65535U :
            (uint16_t)source.motor_ext.homing_elapsed_ms;

        frame.needle_requested_cmd_x10000 =
            source.needle_requested_cmd_x10000;
        frame.needle_limited_cmd_x10000 =
            source.needle_limited_cmd_x10000;
        frame.needle_raw_adc = source.needle_raw_adc;
        frame.needle_zero_adc = source.needle_zero_adc;
        frame.needle_target_adc = source.needle_target_adc;
        frame.needle_error_adc = source.needle_error_adc;
        frame.needle_rpwm = source.needle_rpwm;
        frame.needle_lpwm = source.needle_lpwm;
        frame.needle_state_flags = source.needle_state_flags;
        frame.needle_fault = source.needle_fault;

        frame.nrf_link_active = source.nrf_link_active;
        frame.nrf_command = source.nrf_command;
        frame.nrf_last_sequence = source.nrf_last_sequence;
        frame.servo_target_open = source.servo_target_open;
        frame.servo_pulse_us = source.servo_pulse_us;
        frame.nrf_last_packet_age_ms = source.nrf_last_packet_age_ms;
        frame.nrf_valid_packet_count = source.nrf_valid_packet_count;
        frame.nrf_irq_count_low = source.nrf_irq_count_low;
        frame.nrf_rx_count_low = source.nrf_rx_count_low;

        frame.v14_extension_marker = 0x1419U;

        for (uint32_t i = 0UL;
             i < SDLOGGER_FAST_IMU_SAMPLES_PER_FRAME;
             i++)
        {
            frame.fast_imu[i] = source.fast_imu[i];
        }

        frame.fast_imu_sample_count = source.fast_imu_sample_count;
        frame.fast_imu_valid_mask = source.fast_imu_valid_mask;

        frame.imu_sample_age_us = source.imu_sample_age_us;
        frame.baro_sample_age_us = source.baro_sample_age_us;
        frame.lidar_sample_age_us = source.lidar_sample_age_us;

        frame.sensor_qual_state = source.sensor_qual_state;
        frame.sensor_qual_good_windows = source.sensor_qual_good_windows;
        frame.sensor_qual_flags_low = source.sensor_qual_flags_low;
        frame.sensor_qual_flags_high = source.sensor_qual_flags_high;

        /* CRC is calculated later in main context while draining the ring. */
        frame.crc16 = 0U;

        if (SDLogger_RingPushFromISR(&frame) != 0U)
        {
            sd_logger_frame_count++;
            sd_logger_last_sequence = frame.sequence;
            sd_logger_last_capture_us = capture_us;
            sd_sequence++;
        }
    }

    uint32_t duration_us = micros() - isr_start_us;
    sd_logger_timer_last_isr_duration_us = duration_us;

    if (duration_us > sd_logger_timer_max_isr_duration_us)
    {
        sd_logger_timer_max_isr_duration_us = duration_us;
    }
#endif
}

/* -------------------------------------------------------------------------- */

static void SDLogger_MarkBufferReady(uint8_t buffer_index)
{
    if (buffer_index >= SDLOGGER_BUFFER_COUNT)
    {
        sd_logger_fifo_order_fault_count++;
        return;
    }

    sd_buffer_ready_order[buffer_index] = sd_next_ready_order;
    sd_next_ready_order++;
    if (sd_next_ready_order == 0UL)
    {
        /* 32-bit wrap is not reachable in a flight, but keep zero reserved as
         * "no ticket" so diagnostics remain unambiguous. */
        sd_next_ready_order = 1UL;
    }

    sd_buffer_state[buffer_index] = SDLOGGER_BUFFER_READY;
}

/* -------------------------------------------------------------------------- */

static void SDLogger_SelectNewActiveBuffer(uint8_t buffer_index)
{
    sd_buffer_fill[buffer_index] = 0UL;
    sd_buffer_state[buffer_index] = SDLOGGER_BUFFER_FILLING;
    sd_active_buffer = buffer_index;
}

/* -------------------------------------------------------------------------- */

static uint8_t SDLogger_EnsureActiveBuffer(void)
{
    if ((sd_active_buffer < SDLOGGER_BUFFER_COUNT) &&
        (sd_buffer_state[sd_active_buffer] == SDLOGGER_BUFFER_FILLING) &&
        ((sd_buffer_fill[sd_active_buffer] + SDLOGGER_FRAME_SIZE) <=
         APP_SDLOGGER_BUFFER_SIZE))
    {
        return 1U;
    }

    for (uint8_t i = 0U; i < SDLOGGER_BUFFER_COUNT; i++)
    {
        if (sd_buffer_state[i] == SDLOGGER_BUFFER_EMPTY)
        {
            SDLogger_SelectNewActiveBuffer(i);
            return 1U;
        }
    }

    return 0U;
}

/* -------------------------------------------------------------------------- */

static uint8_t SDLogger_UpdateBackpressureLevel(uint32_t ring_count)
{
    uint8_t previous = sd_logger_backpressure_level;
    uint8_t next = previous;

    if (ring_count >= APP_SDLOGGER_RING_BACKPRESSURE_CRITICAL_FRAMES)
    {
        next = 2U;
    }
    else if (ring_count >= APP_SDLOGGER_RING_BACKPRESSURE_START_FRAMES)
    {
        next = 1U;
    }
    else if (ring_count <= APP_SDLOGGER_RING_BACKPRESSURE_CLEAR_FRAMES)
    {
        next = 0U;
    }

    if ((previous == 0U) && (next >= 1U))
    {
        sd_logger_backpressure_entry_count++;
    }
    if ((previous < 2U) && (next == 2U))
    {
        sd_logger_backpressure_critical_entry_count++;
    }

    sd_logger_backpressure_level = next;
    return next;
}

/* -------------------------------------------------------------------------- */

static void SDLogger_DrainRingToBuffers(void)
{
    uint32_t drain_start_us = micros();
    uint8_t pressure_level = SDLogger_UpdateBackpressureLevel(SDLogger_RingCount());
    uint32_t max_frames = APP_SDLOGGER_RING_DRAIN_MAX_FRAMES;
    uint32_t budget_us = APP_SDLOGGER_RING_DRAIN_BUDGET_US;

    if (pressure_level >= 2U)
    {
        max_frames = APP_SDLOGGER_RING_DRAIN_MAX_FRAMES_CRITICAL;
        budget_us = APP_SDLOGGER_RING_DRAIN_BUDGET_US_CRITICAL;
    }
    else if (pressure_level == 1U)
    {
        max_frames = APP_SDLOGGER_RING_DRAIN_MAX_FRAMES_HIGH;
        budget_us = APP_SDLOGGER_RING_DRAIN_BUDGET_US_HIGH;
    }

    for (uint32_t i = 0UL; i < max_frames; i++)
    {
        if ((i != 0UL) &&
            ((uint32_t)(micros() - drain_start_us) >= budget_us))
        {
            sd_logger_drain_budget_yield_count++;
            break;
        }
        if (SDLogger_RingCount() == 0UL)
        {
            break;
        }
        if (SDLogger_EnsureActiveBuffer() == 0U)
        {
            break;
        }

        SDLoggerFrame_t frame;
        if (SDLogger_RingPop(&frame) == 0U)
        {
            break;
        }

        frame.crc16 = SDLogger_CRC16_CCITT(
            (const uint8_t *)&frame,
            (uint32_t)offsetof(SDLoggerFrame_t, crc16));
        memcpy(
            &sd_buffers[sd_active_buffer][sd_buffer_fill[sd_active_buffer]],
            &frame,
            SDLOGGER_FRAME_SIZE);
        sd_buffer_fill[sd_active_buffer] += SDLOGGER_FRAME_SIZE;
        sd_logger_last_crc16 = frame.crc16;

        if (sd_buffer_fill[sd_active_buffer] >= APP_SDLOGGER_BUFFER_SIZE)
        {
            SDLogger_MarkBufferReady(sd_active_buffer);
            sd_active_buffer = SDLOGGER_INVALID_BUFFER;
        }
    }

    (void)SDLogger_UpdateBackpressureLevel(SDLogger_RingCount());
    sd_logger_drain_last_us = (uint32_t)(micros() - drain_start_us);
    if (sd_logger_drain_last_us > sd_logger_drain_max_us)
    {
        sd_logger_drain_max_us = sd_logger_drain_last_us;
    }
}

/* -------------------------------------------------------------------------- */

static uint8_t SDLogger_FindReadyBuffer(void)
{
    uint8_t best_buffer = SDLOGGER_INVALID_BUFFER;
    uint32_t best_order = 0UL;

    for (uint8_t i = 0U; i < SDLOGGER_BUFFER_COUNT; i++)
    {
        if (sd_buffer_state[i] == SDLOGGER_BUFFER_READY)
        {
            uint32_t order = sd_buffer_ready_order[i];

            if (order == 0UL)
            {
                /* READY without a chronological ticket must never happen. */
                sd_logger_fifo_order_fault_count++;
                continue;
            }

            if ((best_buffer == SDLOGGER_INVALID_BUFFER) ||
                (order < best_order))
            {
                best_buffer = i;
                best_order = order;
            }
        }
    }

    return best_buffer;
}

/* -------------------------------------------------------------------------- */

static void SDLogger_HandleWriteFailure(FRESULT result)
{
    sd_logger_last_result = (uint8_t)result;
    sd_logger_error_count++;
    sd_logger_write_error_count++;

    /* R3R10: after PE9 the recorder authority is RAM, not the physical SD.
     * A straddling preflight DMA failure is diagnostic only and must not stop
     * compact RAM capture or remove the main-loop SD service that supervises it. */
    if (sd_logger_r10_ram_mode != 0U)
    {
        sd_logger_r10_physical_sd_fault_count++;
    }
    else
    {
        sd_logger_ready = 0U;
        sd_logger_logging_active = 0U;
    }

    if (sd_dma_transfer_active != 0U)
    {
        if (PreflightTrigger_IsFlightActive() != 0U)
        {
            /* Never let an SD fault inject a polling HAL abort into flight
             * control timing.  Quiesce hardware and leave logging stopped. */
            (void)BSP_SD_RuntimeQuiesceNonBlocking();
        }
        else
        {
            (void)HAL_SD_Abort(&hsd);
        }
    }

    sd_async_state = SDLOGGER_ASYNC_IDLE;
    sd_writing_buffer = SDLOGGER_INVALID_BUFFER;

    SDLogger_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */

static void SDLogger_HandleSyncFailure(FRESULT result)
{
    sd_logger_last_result = (uint8_t)result;
    sd_logger_error_count++;
    sd_logger_sync_error_count++;
    sd_logger_ready = 0U;
    sd_logger_logging_active = 0U;

    SDLogger_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */

static uint8_t SDLogger_WriteGuardSector(void)
{
#if ((APP_SDLOGGER_PREALLOCATE_ENABLED != 0U) && \
     (APP_SDLOGGER_GUARD_SECTOR_ENABLED != 0U))
    FRESULT result;
    UINT bytes_written = 0U;
    uint32_t start_us;
    FSIZE_t data_end = (FSIZE_t)sd_logger_total_bytes_written;

    if ((data_end + SDLOGGER_SECTOR_SIZE) > sd_preallocated_bytes)
    {
        return 0U;
    }

    result = f_lseek(&sd_log_file, data_end);
    sd_logger_last_result = (uint8_t)result;

    if (result != FR_OK)
    {
        return 0U;
    }

    start_us = micros();
    result = f_write(
        &sd_log_file,
        sd_guard_sector,
        SDLOGGER_SECTOR_SIZE,
        &bytes_written
    );
    sd_logger_last_guard_write_duration_us = micros() - start_us;

    if (sd_logger_last_guard_write_duration_us >
        sd_logger_max_guard_write_duration_us)
    {
        sd_logger_max_guard_write_duration_us =
            sd_logger_last_guard_write_duration_us;
    }

    sd_logger_last_result = (uint8_t)result;

    if ((result != FR_OK) || (bytes_written != SDLOGGER_SECTOR_SIZE))
    {
        return 0U;
    }

    result = f_lseek(&sd_log_file, data_end);
    sd_logger_last_result = (uint8_t)result;

    if (result != FR_OK)
    {
        return 0U;
    }

    sd_logger_guard_write_count++;
#endif

    return 1U;
}

/* -------------------------------------------------------------------------- */

static uint8_t SDLogger_PreallocateFile(void)
{
#if (APP_SDLOGGER_PREALLOCATE_ENABLED != 0U)
    FRESULT result;
    uint32_t requested_bytes = APP_SDLOGGER_PREALLOCATE_BYTES;
    uint32_t start_ms = HAL_GetTick();
    uint32_t sync_start_us;

    for (;;)
    {
        sd_logger_preallocation_attempt_count++;

        result = f_expand(
            &sd_log_file,
            (FSIZE_t)requested_bytes,
            1U
        );

        sd_logger_last_result = (uint8_t)result;

        if (result == FR_OK)
        {
            break;
        }

        if ((result != FR_DENIED) ||
            (requested_bytes <= APP_SDLOGGER_PREALLOCATE_MIN_BYTES))
        {
            return 0U;
        }

        requested_bytes >>= 1;
        requested_bytes &= ~(uint32_t)(SDLOGGER_SECTOR_SIZE - 1UL);

        if (requested_bytes < APP_SDLOGGER_PREALLOCATE_MIN_BYTES)
        {
            requested_bytes = APP_SDLOGGER_PREALLOCATE_MIN_BYTES;
        }

        sd_logger_preallocation_fallback_count++;
    }

    /* Commit FAT allocation and the non-zero directory size before capture. */
    sync_start_us = micros();
    result = f_sync(&sd_log_file);
    sd_logger_last_sync_duration_us = micros() - sync_start_us;

    if (sd_logger_last_sync_duration_us > sd_logger_max_sync_duration_us)
    {
        sd_logger_max_sync_duration_us = sd_logger_last_sync_duration_us;
    }

    sd_logger_last_result = (uint8_t)result;

    if (result != FR_OK)
    {
        sd_logger_sync_error_count++;
        return 0U;
    }

    result = f_lseek(&sd_log_file, 0UL);
    sd_logger_last_result = (uint8_t)result;

    if (result != FR_OK)
    {
        return 0U;
    }

#if (APP_SDLOGGER_RAW_DMA_ENABLED != 0U)
    if ((sd_log_file.obj.sclust < 2UL) ||
        (SDFatFS.csize == 0U))
    {
        sd_logger_last_result = (uint8_t)FR_INT_ERR;
        return 0U;
    }

    sd_file_start_sector =
        SDFatFS.database +
        ((sd_log_file.obj.sclust - 2UL) * (uint32_t)SDFatFS.csize);

    sd_preallocated_sector_count =
        requested_bytes / SDLOGGER_SECTOR_SIZE;
    sd_next_data_sector_offset = 0UL;
#endif

    sd_preallocated_bytes = requested_bytes;
    sd_logger_preallocated_bytes = requested_bytes;

    /* Invalidate the first unwritten sector before capture begins. */
    if (SDLogger_WriteGuardSector() == 0U)
    {
        return 0U;
    }

    sd_logger_preallocate_ok = 1U;
    sd_logger_preallocation_duration_ms = HAL_GetTick() - start_ms;
    sd_logger_sync_count++;

    return 1U;
#else
    sd_preallocated_bytes = 0UL;
    return 1U;
#endif
}

/* -------------------------------------------------------------------------- */

static void SDLogger_RequestSyncInternal(void)
{
#if (APP_SDLOGGER_PREALLOCATE_ENABLED != 0U)
    if (sd_logger_logging_active != 0U)
    {
        sd_logger_runtime_sync_suppressed_count++;
        SDLogger_UpdateLiveDebug();
        return;
    }
#endif

    if ((sd_logger_file_open != 0U) &&
        (sd_logger_ready != 0U) &&
        (sd_sync_pending == 0U))
    {
        sd_sync_pending = 1U;
        sd_sync_defer_reported = 0U;
        sd_logger_sync_request_count++;
    }

    SDLogger_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */

static void SDLogger_PerformSyncIfIdle(void)
{
    FRESULT result;
    uint32_t start_us;
    uint32_t duration_us;

    if ((sd_sync_pending == 0U) ||
        (sd_logger_file_open == 0U) ||
        (sd_logger_ready == 0U))
    {
        return;
    }

    /*
     * A full buffer waiting for SD always has priority.
     * The currently active filling buffer remains in RAM while f_sync runs.
     */
    if ((sd_writing_buffer != SDLOGGER_INVALID_BUFFER) ||
        (sd_async_state != SDLOGGER_ASYNC_IDLE) ||
        (SDLogger_FindReadyBuffer() != SDLOGGER_INVALID_BUFFER) ||
        (SDLogger_RingCount() != 0UL))
    {
        if (sd_sync_defer_reported == 0U)
        {
            sd_logger_sync_deferred_count++;
            sd_sync_defer_reported = 1U;
        }

        return;
    }

    start_us = micros();
    result = f_sync(&sd_log_file);
    duration_us = micros() - start_us;

    sd_logger_last_sync_duration_us = duration_us;

    if (duration_us > sd_logger_max_sync_duration_us)
    {
        sd_logger_max_sync_duration_us = duration_us;
    }

    sd_logger_last_result = (uint8_t)result;

    if (result != FR_OK)
    {
        SDLogger_HandleSyncFailure(result);
        return;
    }

    sd_sync_pending = 0U;
    sd_sync_defer_reported = 0U;
    sd_buffers_since_sync = 0UL;
    sd_logger_sync_count++;

    SDLogger_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */

static void SDLogger_R10EnterRamFlightMode(void)
{
    uint32_t primask;

    if (sd_logger_r10_ram_mode != 0U)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    /* The same CCM allocation is now reinterpreted as 256 x 192-byte compact
     * V16 records. Pending normal capture-ring frames are intentionally discarded;
     * preflight data already committed to SD remains intact. */
    sd_ring_head = 0UL;
    sd_ring_tail = 0UL;
    sd_logger_ring_count = 0UL;
    sd_logger_r10_ram_frame_count = 0UL;
    sd_r10_ram_decimation_phase = 0U;
    sd_logger_ring_capacity = SDLOGGER_RAM_FLIGHT_CAPACITY;
    sd_logger_r10_capture_frozen = 0U;
    sd_logger_r10_flush_active = 0U;
    sd_logger_r10_flush_index = 0UL;
    sd_logger_r10_flush_complete = 0U;
    sd_logger_r10_postflight_reason = 0U;
    sd_r10_postflight_quiet_since_ms = 0UL;
    sd_r10_postflight_cmd_path_prepared = 0U;
    sd_r10_replay_fault_baseline = 0UL;
    sd_logger_r10_entry_ms = HAL_GetTick();
    sd_logger_r10_tail_dma_at_entry =
        ((sd_async_state != SDLOGGER_ASYNC_IDLE) ||
         (sd_writing_buffer != SDLOGGER_INVALID_BUFFER) ||
         (sd_dma_transfer_active != 0U)) ? 1U : 0U;

    sd_sync_pending = 0U;
    sd_logger_guard_pending = 0U;
    sd_logger_r10_ram_mode = 1U;

    __DMB();
    if (primask == 0UL)
    {
        __enable_irq();
    }
}

static void SDLogger_R10CaptureCompactFromISR(
    const SDLoggerSourceSnapshot_t *source,
    uint32_t capture_us)
{
    SDLoggerRamFlightFrame_t frame;
    uint32_t index;

    if ((source == NULL) || (sd_logger_r10_capture_frozen != 0U))
    {
        return;
    }

    index = sd_logger_r10_ram_frame_count;
    if (index >= SDLOGGER_RAM_FLIGHT_CAPACITY)
    {
        sd_logger_r10_ram_overflow_count++;
        sd_logger_r10_capture_frozen = 1U;
        return;
    }

    memset(&frame, 0, sizeof(frame));
    frame.sequence = sd_sequence;
    frame.timestamp_us = capture_us;
    frame.timestamp_ms = HAL_GetTick();
    frame.pressure_pa = source->pressure_pa;
    frame.altitude_mm = source->filtered_altitude_mm;
    frame.eskf_z_mm = source->full_eskf_position_z_mm;
    frame.eskf_vz_mms = source->full_eskf_velocity_z_mms;
    frame.eskf_x_cm = source->full_eskf_position_x_cm;
    frame.eskf_y_cm = source->full_eskf_position_y_cm;
    frame.eskf_vx_cms = source->full_eskf_velocity_x_cms;
    frame.eskf_vy_cms = source->full_eskf_velocity_y_cms;
    frame.roll_cdeg = source->full_eskf_roll_cdeg;
    frame.pitch_cdeg = source->full_eskf_pitch_cdeg;
    frame.yaw_cdeg = source->full_eskf_yaw_cdeg;
    frame.pitch_rate_cdeg_s = source->flight_pitch_rate_cdeg_s;
    frame.yaw_rate_cdeg_s = source->flight_yaw_rate_cdeg_s;
    frame.q_w_q15 = source->full_eskf_q_w_q15;
    frame.q_x_q15 = source->full_eskf_q_x_q15;
    frame.q_y_q15 = source->full_eskf_q_y_q15;
    frame.q_z_q15 = source->full_eskf_q_z_q15;
    frame.lidar_mm = source->lidar_filtered_mm;
    frame.cpu_load_x100 = source->cpu_load_x100;
    frame.needle_requested_x10000 = source->needle_requested_cmd_x10000;
    frame.needle_limited_x10000 = source->needle_limited_cmd_x10000;
    frame.needle_raw_adc = source->needle_raw_adc;
    frame.needle_target_adc = source->needle_target_adc;
    frame.needle_error_adc = source->needle_error_adc;
    frame.valve_cmd_x10000 = source->flight_valve_cmd_x10000;
    frame.imu_age_us = source->imu_sample_age_us;
    frame.baro_age_us = source->baro_sample_age_us;
    frame.lidar_age_us = source->lidar_sample_age_us;
    frame.health_flags = source->health_flags;
    frame.fault_code = source->fault_code;
    frame.mission_state = source->mission_state;
    frame.flight_input_valid = source->flight_input_valid;
    frame.rcs_requested_mask = source->flight_rcs_requested_mask;
    frame.rcs_applied_mask = source->flight_rcs_applied_mask;
    frame.needle_rpwm = source->needle_rpwm;
    frame.needle_lpwm = source->needle_lpwm;
    frame.needle_state_flags = source->needle_state_flags;
    frame.needle_fault = source->needle_fault;
    frame.full_eskf_flags = source->full_eskf_flags;
    frame.full_eskf_attitude_flags = source->full_eskf_attitude_flags;
    frame.flight_rcs_fault = source->flight_rcs_fault;
    frame.mount_cal_valid = source->mount_cal_valid;
    frame.motor_ext = source->motor_ext;
    frame.marker = SDLOGGER_RAM_FLIGHT_MARKER;
    frame.needle_zero_adc = source->needle_zero_adc;
    frame.crc16 = 0U;

    sd_capture_storage.flight[index] = frame;
    __DMB();
    sd_logger_r10_ram_frame_count = index + 1UL;

    sd_logger_frame_count++;
    sd_logger_ring_push_count++;
    sd_logger_ring_count = sd_logger_r10_ram_frame_count;
    if (sd_logger_ring_count > sd_logger_ring_high_watermark)
    {
        sd_logger_ring_high_watermark = sd_logger_ring_count;
    }
    sd_logger_last_sequence = frame.sequence;
    sd_logger_last_capture_us = capture_us;
    sd_sequence++;

    /* Keep one confirmed touchdown record, then freeze the RAM image so main
     * can replay it to SD without racing the capture ISR. */
    if (source->mission_state == (uint8_t)TARAGAY_MISSION_TOUCHDOWN)
    {
        sd_logger_r10_capture_frozen = 1U;
        sd_logger_r10_postflight_reason = 1U;
    }
}

static void SDLogger_R10BuildReplayFrame(
    const SDLoggerRamFlightFrame_t *compact,
    SDLoggerFrame_t *frame)
{
    memset(frame, 0, sizeof(*frame));
    frame->magic = SDLOGGER_MAGIC;
    frame->sequence = compact->sequence;
    frame->timestamp_us = compact->timestamp_us;
    frame->timestamp_ms = compact->timestamp_ms;
    frame->pressure_pa = compact->pressure_pa;
    frame->altitude_cm = compact->altitude_mm / 10;
    frame->format_version = SDLOGGER_RAM_FLIGHT_FORMAT_VERSION;
    frame->frame_size = SDLOGGER_FRAME_SIZE;
    frame->lidar_distance_cm = compact->lidar_mm / 10U;
    frame->lidar_filtered_mm = compact->lidar_mm;
    frame->cpu_load_x100 = compact->cpu_load_x100;
    frame->health_flags = compact->health_flags;
    frame->fault_code = compact->fault_code;
    frame->full_eskf_position_z_mm = compact->eskf_z_mm;
    frame->full_eskf_velocity_z_mms = compact->eskf_vz_mms;
    frame->full_eskf_position_x_cm = compact->eskf_x_cm;
    frame->full_eskf_position_y_cm = compact->eskf_y_cm;
    frame->full_eskf_velocity_x_cms = compact->eskf_vx_cms;
    frame->full_eskf_velocity_y_cms = compact->eskf_vy_cms;
    frame->full_eskf_q_w_q15 = compact->q_w_q15;
    frame->full_eskf_q_x_q15 = compact->q_x_q15;
    frame->full_eskf_q_y_q15 = compact->q_y_q15;
    frame->full_eskf_q_z_q15 = compact->q_z_q15;
    frame->full_eskf_roll_cdeg = compact->roll_cdeg;
    frame->full_eskf_pitch_cdeg = compact->pitch_cdeg;
    frame->full_eskf_yaw_cdeg = compact->yaw_cdeg;
    frame->full_eskf_flags = compact->full_eskf_flags;
    frame->full_eskf_attitude_flags = compact->full_eskf_attitude_flags;
    frame->needle_requested_cmd_x10000 = compact->needle_requested_x10000;
    frame->needle_limited_cmd_x10000 = compact->needle_limited_x10000;
    frame->needle_raw_adc = compact->needle_raw_adc;
    frame->needle_zero_adc = compact->needle_zero_adc;
    frame->needle_target_adc = compact->needle_target_adc;
    frame->needle_error_adc = compact->needle_error_adc;
    frame->needle_rpwm = compact->needle_rpwm;
    frame->needle_lpwm = compact->needle_lpwm;
    frame->needle_state_flags = compact->needle_state_flags;
    frame->needle_fault = compact->needle_fault;
    frame->imu_sample_age_us = compact->imu_age_us;
    frame->baro_sample_age_us = compact->baro_age_us;
    frame->lidar_sample_age_us = compact->lidar_age_us;

    /* V16 motor extension overlay. Bytes 148..243 are unused by the compact
     * flight replay path; normal V14 frames keep their original ESKF/IMU
     * meanings. The decoder branches on format_version before interpreting. */
    memcpy(((uint8_t *)frame) + 148U,
           &compact->motor_ext,
           sizeof(compact->motor_ext));

    /* V16/R3R10 metadata reuse. Dedicated decoder interprets these fields. */
    frame->imu_diag_reserved = compact->valve_cmd_x10000;
    frame->sensor_qual_state = compact->mission_state;
    frame->sensor_qual_good_windows = compact->rcs_requested_mask;
    frame->sensor_qual_flags_low =
        (uint16_t)compact->rcs_applied_mask |
        ((uint16_t)compact->flight_input_valid << 8);
    frame->sensor_qual_flags_high =
        (uint16_t)compact->flight_rcs_fault |
        ((uint16_t)compact->mount_cal_valid << 8);
    frame->filter_reset_count = (uint16_t)compact->pitch_rate_cdeg_s;
    frame->baro_median_delta_pa = compact->yaw_rate_cdeg_s;
    frame->v14_extension_marker = SDLOGGER_RAM_FLIGHT_MARKER;
    frame->crc16 = 0U;
}

static void SDLogger_R10PumpReplayFrames(void)
{
    uint32_t pumped = 0UL;

    while ((sd_logger_r10_flush_index < sd_logger_r10_ram_frame_count) &&
           (pumped < 6UL))
    {
        SDLoggerFrame_t frame;
        const SDLoggerRamFlightFrame_t *compact =
            &sd_capture_storage.flight[sd_logger_r10_flush_index];

        if (SDLogger_EnsureActiveBuffer() == 0U)
        {
            break;
        }

        SDLogger_R10BuildReplayFrame(compact, &frame);
        frame.crc16 = SDLogger_CRC16_CCITT(
            (const uint8_t *)&frame,
            (uint32_t)offsetof(SDLoggerFrame_t, crc16));

        memcpy(
            &sd_buffers[sd_active_buffer][sd_buffer_fill[sd_active_buffer]],
            &frame,
            SDLOGGER_FRAME_SIZE);
        sd_buffer_fill[sd_active_buffer] += SDLOGGER_FRAME_SIZE;
        sd_logger_last_crc16 = frame.crc16;
        sd_logger_r10_flush_index++;
        sd_logger_ring_pop_count++;
        pumped++;

        if (sd_buffer_fill[sd_active_buffer] >= APP_SDLOGGER_BUFFER_SIZE)
        {
            SDLogger_MarkBufferReady(sd_active_buffer);
            sd_active_buffer = SDLOGGER_INVALID_BUFFER;
        }
    }

    sd_logger_ring_count =
        sd_logger_r10_ram_frame_count - sd_logger_r10_flush_index;

    if (sd_logger_r10_flush_index >= sd_logger_r10_ram_frame_count)
    {
        if ((sd_active_buffer < SDLOGGER_BUFFER_COUNT) &&
            (sd_buffer_state[sd_active_buffer] == SDLOGGER_BUFFER_FILLING) &&
            (sd_buffer_fill[sd_active_buffer] > 0UL))
        {
            SDLogger_MarkBufferReady(sd_active_buffer);
            sd_active_buffer = SDLOGGER_INVALID_BUFFER;
        }
    }
}

static void SDLogger_R10ServicePostflight(void)
{
    uint32_t now_ms = HAL_GetTick();
    TaragayFlightLogicStatus_t flight_logic = TaragayFlightLogic_GetStatus();
    SolenoidOutputStatus_t solenoid = SolenoidOutput_GetStatus();
    uint8_t safe_reason = 0U;
    uint8_t actuators_quiet;

    if (sd_logger_r10_ram_mode == 0U)
    {
        return;
    }

    if (sd_logger_r10_flush_complete != 0U)
    {
        return;
    }

    if (flight_logic.mission_state == (uint8_t)TARAGAY_MISSION_TOUCHDOWN)
    {
        safe_reason = 1U;
    }
    else if (v30_stop_latched != 0U)
    {
        safe_reason = 2U;
    }

    if (safe_reason == 0U)
    {
        sd_r10_postflight_quiet_since_ms = 0UL;
        return;
    }

    /* R3R10R4R1: STOP alone is NOT permission to start the SD quiet timer.
     * The only legal post-STOP motor action is the needle safe-close, so wait
     * until that one-shot motion has positively completed. A failed close keeps
     * the complete RAM image intact and permanently blocks replay for this
     * session rather than allowing SD traffic while the actuator state is
     * uncertain. Touchdown keeps the older actuator-quiet gate. */
    if (safe_reason == 2U)
    {
        if ((NeedleValveAutonomousControl_HasEStopSafeCloseFailed() != 0U) ||
            (NeedleValveAutonomousControl_IsEStopSafeCloseComplete() == 0U))
        {
            sd_r10_postflight_quiet_since_ms = 0UL;
            return;
        }
    }

    actuators_quiet =
        ((needle_valve_rpwm == 0U) &&
         (needle_valve_lpwm == 0U) &&
         (solenoid.requested_mask == SOLENOID_VALVE_NONE) &&
         (solenoid.applied_mask == SOLENOID_VALVE_NONE)) ? 1U : 0U;

    if (actuators_quiet == 0U)
    {
        /* Any real motor/RCS activity restarts the FULL quiet interval. */
        sd_r10_postflight_quiet_since_ms = 0UL;
        return;
    }

    if (sd_r10_postflight_quiet_since_ms == 0UL)
    {
        /* This timestamp is intentionally captured only after CLOSE COMPLETE
         * + physical outputs OFF. It must never inherit STOP reception time or
         * an older motor/RCS cooldown timestamp. */
        sd_r10_postflight_quiet_since_ms = now_ms;
        return;
    }

    if ((uint32_t)(now_ms - sd_r10_postflight_quiet_since_ms) <
        APP_SDLOGGER_R10_POSTFLIGHT_QUIET_MS)
    {
        return;
    }

    if (sd_logger_r10_capture_frozen == 0U)
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        sd_logger_r10_capture_frozen = 1U;
        sd_logger_r10_postflight_reason = safe_reason;
        __DMB();
        if (primask == 0UL)
        {
            __enable_irq();
        }
    }

    /* A PE9-boundary/tail-DMA fault is historical evidence, not a reason to
     * throw away the flight RAM image. After 3 s of verified electrical quiet,
     * repair only the host/DMA command path. This helper contains no HAL_Delay,
     * no card re-enumeration and no new SD command; it clears stale SDIO/DMA/HAL
     * state before the first postflight replay command is launched. */
    if (sd_r10_postflight_cmd_path_prepared == 0U)
    {
        if ((sd_async_state != SDLOGGER_ASYNC_IDLE) ||
            (sd_dma_transfer_active != 0U))
        {
            return;
        }

        if (BSP_SD_RuntimeSoftRecover() != MSD_OK)
        {
            sd_logger_r10_physical_sd_fault_count++;
            return;
        }

        /* Drop any stale retry bookkeeping from a pre-PE9/tail transaction.
         * The RAM replay starts as a clean postflight transaction stream. */
        sd_dma_retry_pending_internal = 0U;
        sd_logger_dma_retry_pending = 0U;
        sd_dma_retry_count_current = 0U;
        sd_logger_dma_retry_current = 0U;
        sd_r10_replay_fault_baseline = sd_logger_r10_physical_sd_fault_count;
        sd_r10_postflight_cmd_path_prepared = 1U;
    }

    /* If the freshly prepared postflight path itself faults, stop issuing new
     * replay commands. The compact RAM image is still untouched in CCM and the
     * failure stays visible for bench diagnosis. Pre-existing tail faults do
     * not block the first replay attempt because the baseline is captured only
     * after the 3 s quiet-side recovery above. */
    if ((sd_r10_postflight_cmd_path_prepared != 0U) &&
        (sd_logger_r10_physical_sd_fault_count > sd_r10_replay_fault_baseline))
    {
        sd_logger_r10_flush_active = 0U;
        return;
    }

    if (sd_logger_r10_flush_active == 0U)
    {
        if (sd_r10_postflight_cmd_path_prepared == 0U)
        {
            return;
        }

        if ((sd_async_state != SDLOGGER_ASYNC_IDLE) ||
            (sd_dma_transfer_active != 0U))
        {
            return;
        }

        /* Discard any not-yet-launched preflight writer data. The physical
         * file contains all transfers confirmed before PE9; flight replay is
         * appended chronologically after that committed prefix. */
        memset(sd_buffer_fill, 0, sizeof(sd_buffer_fill));
        memset(sd_buffer_state, 0, sizeof(sd_buffer_state));
        memset(sd_buffer_ready_order, 0, sizeof(sd_buffer_ready_order));
        sd_writing_buffer = SDLOGGER_INVALID_BUFFER;
        sd_active_buffer = 0U;
        sd_next_ready_order = 1UL;
        sd_last_started_ready_order = 0UL;
        SDLogger_SelectNewActiveBuffer(0U);
        for (uint8_t i = 1U; i < SDLOGGER_BUFFER_COUNT; i++)
        {
            sd_buffer_state[i] = SDLOGGER_BUFFER_EMPTY;
        }
        sd_logger_guard_pending = 0U;
        sd_logger_r10_flush_index = 0UL;
        sd_logger_r10_flush_active = 1U;
        sd_logger_r10_flush_start_ms = now_ms;
        sd_logger_r10_postflight_reason = safe_reason;
        sd_r10_replay_fault_baseline = sd_logger_r10_physical_sd_fault_count;
    }

    SDLogger_R10PumpReplayFrames();

    if (sd_async_state == SDLOGGER_ASYNC_IDLE)
    {
        (void)SDLogger_StartReadyBufferDMA();
    }

    if ((sd_logger_r10_flush_index >= sd_logger_r10_ram_frame_count) &&
        (SDLogger_FindReadyBuffer() == SDLOGGER_INVALID_BUFFER) &&
        (sd_writing_buffer == SDLOGGER_INVALID_BUFFER) &&
        (sd_async_state == SDLOGGER_ASYNC_IDLE))
    {
        sd_logger_r10_flush_active = 0U;
        sd_logger_r10_flush_complete = 1U;
        /* Keep RAM-flight mode latched for this PE9 session. Capture is frozen
         * and no further SD commands are launched after the replay completes. */
        sd_logger_ring_count = 0UL;
    }
}

static void SDLogger_ExtendWriteHoldUntil(uint32_t candidate_until_ms)
{
    /* Wrap-safe monotonic deadline extension: a shorter noisy-event cooldown
     * must never shorten a longer hold already in force. This fixes R3R8,
     * where motor activity in the PE9 transition overwrote 2500 ms with
     * only 500 ms. */
    if ((int32_t)(candidate_until_ms - sd_motor_write_hold_until_ms) > 0)
    {
        sd_motor_write_hold_until_ms = candidate_until_ms;
    }
}

static void SDLogger_UpdateMotorWriteHold(void)
{
#if (APP_SDLOGGER_MOTOR_WRITE_HOLD_ENABLED != 0U)
    uint32_t now_ms = HAL_GetTick();
    uint8_t flight_active = PreflightTrigger_IsFlightActive();
    uint8_t motor_active = 0U;
    uint8_t rcs_active = 0U;
    uint32_t rcs_pulse_complete_count;
    SolenoidOutputStatus_t solenoid_status = SolenoidOutput_GetStatus();

    /* P110 state: 1 SEARCH, 2 DRIVE, 3 BRAKE, 4 CORRECTION_DWELL.
     * PWM globals provide an independent low-level check without masking IRQs. */
    if (((p110_state >= 1U) && (p110_state <= 4U)) ||
        (needle_valve_rpwm != 0U) || (needle_valve_lpwm != 0U))
    {
        motor_active = 1U;
    }

    if ((solenoid_status.requested_mask != SOLENOID_VALVE_NONE) ||
        (solenoid_status.applied_mask != SOLENOID_VALVE_NONE))
    {
        rcs_active = 1U;
    }

    /* PE9/flight entry hold. Use MAX-deadline semantics so subsequent shorter
     * motor/RCS cooldowns can extend, but never shorten, this window. */
    if ((flight_active != 0U) && (sd_flight_active_prev == 0U))
    {
        SDLogger_ExtendWriteHoldUntil(
            now_ms + APP_SDLOGGER_FLIGHT_ENTRY_HOLD_MS);
    }

    if (motor_active != 0U)
    {
        SDLogger_ExtendWriteHoldUntil(
            now_ms + APP_SDLOGGER_MOTOR_WRITE_COOLDOWN_MS);
    }

    /* R3R8 hardware evidence showed the SD command failure 178 ms after an
     * RCS pulse cluster. Suppress only NEW SD command launches around the
     * physical/requested RCS activity; RAM capture and control continue. */
    if (rcs_active != 0U)
    {
        SDLogger_ExtendWriteHoldUntil(
            now_ms + APP_SDLOGGER_RCS_WRITE_COOLDOWN_MS);
    }

    rcs_pulse_complete_count = solenoid_status.pulse_complete_count;
    if (rcs_pulse_complete_count != sd_r9_rcs_pulse_complete_prev)
    {
        uint32_t delta = rcs_pulse_complete_count - sd_r9_rcs_pulse_complete_prev;
        /* Counter reset/wrap is not a flight event; count one observation only. */
        if (rcs_pulse_complete_count < sd_r9_rcs_pulse_complete_prev)
        {
            delta = 1UL;
        }
        sd_logger_r9_rcs_hold_event_count += delta;
        sd_r9_rcs_pulse_complete_prev = rcs_pulse_complete_count;
        SDLogger_ExtendWriteHoldUntil(
            now_ms + APP_SDLOGGER_RCS_WRITE_COOLDOWN_MS);
    }

    sd_flight_active_prev = flight_active;

    if ((int32_t)(sd_motor_write_hold_until_ms - now_ms) > 0)
    {
        sd_logger_motor_write_hold_active = 1U;
        sd_logger_r9_hold_remaining_ms =
            sd_motor_write_hold_until_ms - now_ms;
    }
    else
    {
        sd_logger_motor_write_hold_active = 0U;
        sd_logger_r9_hold_remaining_ms = 0UL;
    }

    sd_logger_motor_write_hold_until_ms = sd_motor_write_hold_until_ms;

    if ((sd_logger_motor_write_hold_active != 0U) &&
        (sd_motor_write_hold_prev == 0U))
    {
        sd_logger_motor_write_hold_entry_count++;
    }
    sd_motor_write_hold_prev = sd_logger_motor_write_hold_active;
#else
    sd_logger_motor_write_hold_active = 0U;
    sd_logger_r9_hold_remaining_ms = 0UL;
#endif
}

static uint8_t SDLogger_MotorWriteHoldActive(void)
{
#if (APP_SDLOGGER_MOTOR_WRITE_HOLD_ENABLED != 0U)
    if (sd_logger_motor_write_hold_active != 0U)
    {
        sd_logger_motor_write_hold_defer_count++;
        p112_sd_suppressed_count++;
        return 1U;
    }
#endif
    return 0U;
}

/* -------------------------------------------------------------------------- */

static uint8_t SDLogger_StartGuardDMA(void)
{
#if ((APP_SDLOGGER_RAW_DMA_ENABLED != 0U) && \
     (APP_SDLOGGER_GUARD_SECTOR_ENABLED != 0U))
    uint32_t guard_sector;

    if ((sd_logger_r10_flush_active == 0U) &&
        (SDLogger_MotorWriteHoldActive() != 0U))
    {
        return 0U;
    }

    if (sd_next_data_sector_offset >= sd_preallocated_sector_count)
    {
        SDLogger_HandleWriteFailure(FR_DENIED);
        return 0U;
    }

    guard_sector =
        sd_file_start_sector + sd_next_data_sector_offset;

    sd_async_state = SDLOGGER_ASYNC_GUARD_DMA;
    sd_async_transfer_start_us = micros();
    sd_async_transfer_start_ms = HAL_GetTick();
    sd_async_next_card_poll_ms = HAL_GetTick();

    if (BSP_SD_WriteBlocks_DMA(
            (uint32_t *)(void *)sd_guard_sector,
            guard_sector,
            1UL) != MSD_OK)
    {
        /* R8R35R3R6: a proven needle-power transient can make a new SDIO DMA
         * launch fail even though the previous transfer completed cleanly.
         * During flight, retry ONLY this not-yet-started transfer; never run a
         * card re-init or a blocking DMA abort from the control window. */
        if (SDLogger_ScheduleDMAStartRetry() == 0U)
        {
            SDLogger_HandleWriteFailure(FR_DISK_ERR);
        }
        return 0U;
    }
#else
    sd_async_state = SDLOGGER_ASYNC_IDLE;
#endif

    return 1U;
}

/* -------------------------------------------------------------------------- */

static uint8_t SDLogger_StartReadyBufferDMA(void)
{
#if (APP_SDLOGGER_RAW_DMA_ENABLED != 0U)
    uint8_t ready_buffer;
    uint32_t actual_size;
    uint32_t transfer_size;
    uint32_t sector_count;
    uint32_t first_sector;
    uint32_t ready_order;

    if ((sd_async_state != SDLOGGER_ASYNC_IDLE) ||
        (sd_writing_buffer != SDLOGGER_INVALID_BUFFER))
    {
        return 0U;
    }

    if (SDLogger_MotorWriteHoldActive() != 0U)
    {
        return 0U;
    }

    ready_buffer = SDLogger_FindReadyBuffer();

    if (ready_buffer == SDLOGGER_INVALID_BUFFER)
    {
        return 0U;
    }

    ready_order = sd_buffer_ready_order[ready_buffer];
    if (ready_order == 0UL)
    {
        sd_logger_fifo_order_fault_count++;
        return 0U;
    }

    if ((sd_last_started_ready_order != 0UL) &&
        (ready_order <= sd_last_started_ready_order))
    {
        /* This is diagnostic/fail-visible. The min-ticket selector above
         * should make it impossible unless metadata was corrupted. */
        sd_logger_fifo_order_fault_count++;
    }

    actual_size = sd_buffer_fill[ready_buffer];

    if (actual_size == 0UL)
    {
        sd_buffer_state[ready_buffer] = SDLOGGER_BUFFER_EMPTY;
        sd_buffer_ready_order[ready_buffer] = 0UL;
        return 0U;
    }

    transfer_size =
        (actual_size + (SDLOGGER_SECTOR_SIZE - 1UL)) &
        ~(SDLOGGER_SECTOR_SIZE - 1UL);

    if (transfer_size > APP_SDLOGGER_BUFFER_SIZE)
    {
        SDLogger_HandleWriteFailure(FR_INT_ERR);
        return 0U;
    }

    if (transfer_size > actual_size)
    {
        memset(
            &sd_buffers[ready_buffer][actual_size],
            0,
            transfer_size - actual_size
        );
    }

    sector_count = transfer_size / SDLOGGER_SECTOR_SIZE;

    /* Keep one sector available for the moving invalid guard. */
    if ((sd_next_data_sector_offset + sector_count + 1UL) >
        sd_preallocated_sector_count)
    {
        SDLogger_HandleWriteFailure(FR_DENIED);
        return 0U;
    }

    first_sector =
        sd_file_start_sector + sd_next_data_sector_offset;

    sd_writing_buffer = ready_buffer;
    sd_buffer_state[ready_buffer] = SDLOGGER_BUFFER_WRITING;
    sd_write_offset = 0UL;

    sd_async_transfer_sector_count = sector_count;
    sd_async_data_bytes = actual_size;
    sd_async_transfer_start_us = micros();
    sd_async_transfer_start_ms = HAL_GetTick();
    sd_async_next_card_poll_ms = HAL_GetTick();
    sd_async_state = SDLOGGER_ASYNC_DATA_DMA;

    if (BSP_SD_WriteBlocks_DMA(
            (uint32_t *)(void *)sd_buffers[ready_buffer],
            first_sector,
            sector_count) != MSD_OK)
    {
        /* R8R35R3R6: preserve the full ready buffer and retry the same sector
         * address after a short backoff. The sector offset advances only after
         * confirmed completion, so this cannot skip flight-log sectors. */
        if (SDLogger_ScheduleDMAStartRetry() == 0U)
        {
            SDLogger_HandleWriteFailure(FR_DISK_ERR);
        }
        return 0U;
    }

    sd_dma_retry_count_current = 0U;
    sd_logger_dma_retry_current = 0U;
    sd_last_started_ready_order = ready_order;
    sd_logger_async_data_start_count++;
    return 1U;
#else
    return 0U;
#endif
}

/* -------------------------------------------------------------------------- */


static uint8_t SDLogger_AttemptRuntimeHostRecovery(void)
{
    uint32_t start_us;
    uint32_t duration_us;

    sd_logger_runtime_recovery_count++;
    sd_logger_runtime_last_hal_error = hsd.ErrorCode;

    /* During an active flight, a logger fault must never block control for a
     * full SD card re-enumeration.  Stop logging cleanly and surface the fault. */
    if (PreflightTrigger_IsFlightActive() != 0U)
    {
        sd_logger_runtime_recovery_flight_abort_count++;
        return 0U;
    }

    sd_logger_runtime_recovery_active = 1U;
    start_us = micros();

    if (BSP_SD_RuntimeReinit() != MSD_OK)
    {
        duration_us = micros() - start_us;
        sd_logger_runtime_recovery_last_us = duration_us;
        if (duration_us > sd_logger_runtime_recovery_max_us)
        {
            sd_logger_runtime_recovery_max_us = duration_us;
        }
        sd_logger_runtime_recovery_failure_count++;
        sd_logger_runtime_recovery_active = 0U;
        return 0U;
    }

    duration_us = micros() - start_us;
    sd_logger_runtime_recovery_last_us = duration_us;
    if (duration_us > sd_logger_runtime_recovery_max_us)
    {
        sd_logger_runtime_recovery_max_us = duration_us;
    }

    sd_logger_runtime_recovery_success_count++;
    sd_logger_runtime_recovery_active = 0U;
    sd_dma_retry_count_current = 0U;
    sd_logger_dma_retry_current = 0U;
    sd_dma_retry_pending_internal = 1U;
    sd_logger_dma_retry_pending = 1U;
    sd_dma_retry_due_ms = HAL_GetTick() + SDLOGGER_DMA_RETRY_DELAY_MS;
    return 1U;
}

static uint8_t SDLogger_ScheduleDMAStartRetry(void)
{
    /*
     * R8R35R3R6 - flight-safe START-only retry.
     *
     * Hardware A/B logs isolated the remaining SD failure to the powered
     * needle motor path. The observed failure is an immediate DMA launch
     * error: the previous transfer is already complete, BSP reports no active
     * transfer, and the new HAL_SD_WriteBlocks_DMA() call fails once.
     *
     * In flight we therefore do NOT re-enumerate the card and do NOT call
     * HAL_DMA_Abort(). We retain the current writer buffer/sector address and
     * retry the exact same launch after 5 ms. The retry count is bounded so a
     * genuinely dead SD path still fails visible instead of consuming control
     * time indefinitely.
     */
    if (PreflightTrigger_IsFlightActive() != 0U)
    {
        sd_logger_runtime_last_hal_error = hsd.ErrorCode;

        if (sd_dma_retry_count_current >= SDLOGGER_DMA_START_RETRY_LIMIT_FLIGHT)
        {
            sd_logger_dma_retry_exhausted_count++;
            sd_logger_runtime_recovery_count++;
            sd_logger_runtime_recovery_flight_abort_count++;
            return 0U;
        }

        sd_dma_retry_count_current++;
        sd_logger_dma_retry_current = sd_dma_retry_count_current;
        sd_logger_dma_retry_attempt_count++;

        /* BSP marked the failed launch as transfer_error even though no DMA is
         * active. Keep the async transaction alive; the next BSP launch resets
         * these flags in BSP_SD_DMA_BeginTransfer(). */
        sd_dma_retry_pending_internal = 1U;
        sd_logger_dma_retry_pending = 1U;
        sd_dma_retry_due_ms = HAL_GetTick() + SDLOGGER_DMA_RETRY_DELAY_MS;
        return 1U;
    }

    /* Pre-flight retains the older recovery policy, including host repair. */
    return SDLogger_ScheduleDMARetry();
}

/* -------------------------------------------------------------------------- */

static uint8_t SDLogger_ScheduleDMARetry(void)
{
    /* Flight safety first: never spend control time repairing/re-enumerating
     * the SD host while PE9 has armed the vehicle.  The caller will mark the
     * logger failed and SystemMonitor will expose SYS_FAULT_SD_LOGGING, while
     * flight control keeps its timing. */
    if (PreflightTrigger_IsFlightActive() != 0U)
    {
        sd_logger_runtime_last_hal_error = hsd.ErrorCode;
        sd_logger_runtime_recovery_count++;
        sd_logger_runtime_recovery_flight_abort_count++;
        return 0U;
    }

    /*
     * DATA and GUARD transfers are both safely rewriteable at the same sector
     * address. sd_next_data_sector_offset advances only after a confirmed
     * DATA completion, so a full-buffer retry cannot skip sectors.
     */
    if (sd_dma_retry_count_current >= SDLOGGER_DMA_RETRY_LIMIT)
    {
        sd_logger_dma_retry_exhausted_count++;
        return SDLogger_AttemptRuntimeHostRecovery();
    }

    if (sd_dma_transfer_active != 0U)
    {
        sd_logger_dma_retry_abort_count++;
    }

    sd_logger_runtime_last_hal_error = hsd.ErrorCode;
    (void)BSP_SD_RuntimeSoftRecover();

    sd_dma_retry_count_current++;
    sd_logger_dma_retry_current = sd_dma_retry_count_current;
    sd_logger_dma_retry_attempt_count++;

    sd_dma_retry_pending_internal = 1U;
    sd_logger_dma_retry_pending = 1U;
    sd_dma_retry_due_ms = HAL_GetTick() + SDLOGGER_DMA_RETRY_DELAY_MS;

    return 1U;
}

/* -------------------------------------------------------------------------- */

static uint8_t SDLogger_RestartCurrentDMA(void)
{
    uint32_t first_sector;

    if (sd_dma_retry_pending_internal == 0U)
    {
        return 0U;
    }

    if (SDLogger_MotorWriteHoldActive() != 0U)
    {
        return 0U;
    }

    if ((int32_t)(HAL_GetTick() - sd_dma_retry_due_ms) < 0)
    {
        return 0U;
    }

    sd_async_transfer_start_us = micros();
    sd_async_transfer_start_ms = HAL_GetTick();
    sd_async_next_card_poll_ms = HAL_GetTick();

    if (sd_async_state == SDLOGGER_ASYNC_DATA_DMA)
    {
        if (sd_writing_buffer >= SDLOGGER_BUFFER_COUNT)
        {
            return 0U;
        }

        first_sector =
            sd_file_start_sector + sd_next_data_sector_offset;

        if (BSP_SD_WriteBlocks_DMA(
                (uint32_t *)(void *)sd_buffers[sd_writing_buffer],
                first_sector,
                sd_async_transfer_sector_count) != MSD_OK)
        {
            if (SDLogger_ScheduleDMAStartRetry() == 0U)
            {
                SDLogger_HandleWriteFailure(FR_DISK_ERR);
            }
            return 0U;
        }
    }
    else if (sd_async_state == SDLOGGER_ASYNC_GUARD_DMA)
    {
        first_sector =
            sd_file_start_sector + sd_next_data_sector_offset;

        if (BSP_SD_WriteBlocks_DMA(
                (uint32_t *)(void *)sd_guard_sector,
                first_sector,
                1UL) != MSD_OK)
        {
            if (SDLogger_ScheduleDMAStartRetry() == 0U)
            {
                SDLogger_HandleWriteFailure(FR_DISK_ERR);
            }
            return 0U;
        }
    }
    else
    {
        sd_dma_retry_pending_internal = 0U;
        sd_logger_dma_retry_pending = 0U;
        return 0U;
    }

    sd_dma_retry_pending_internal = 0U;
    sd_logger_dma_retry_pending = 0U;

    return 1U;
}

/* -------------------------------------------------------------------------- */

static void SDLogger_ServiceAsyncDMA(void)
{
#if (APP_SDLOGGER_RAW_DMA_ENABLED != 0U)
    uint32_t duration_us;

    if (sd_async_state == SDLOGGER_ASYNC_IDLE)
    {
        return;
    }

    if (sd_dma_retry_pending_internal != 0U)
    {
        (void)SDLogger_RestartCurrentDMA();
        return;
    }

    if ((uint32_t)(HAL_GetTick() - sd_async_transfer_start_ms) >=
        APP_SDLOGGER_DMA_TIMEOUT_MS)
    {
        sd_logger_async_timeout_count++;

        if (SDLogger_ScheduleDMARetry() == 0U)
        {
            SDLogger_HandleWriteFailure(FR_TIMEOUT);
        }
        return;
    }

    if (sd_dma_transfer_error != 0U)
    {
        if (SDLogger_ScheduleDMARetry() == 0U)
        {
            SDLogger_HandleWriteFailure(FR_DISK_ERR);
        }
        return;
    }

    if (sd_dma_tx_complete == 0U)
    {
        return;
    }

    if ((int32_t)(HAL_GetTick() - sd_async_next_card_poll_ms) < 0)
    {
        return;
    }

    sd_async_next_card_poll_ms =
        HAL_GetTick() + APP_SDLOGGER_CARD_POLL_PERIOD_MS;

    if (BSP_SD_GetCardState() != MSD_OK)
    {
        sd_logger_async_card_busy_poll_count++;
        return;
    }

    duration_us = micros() - sd_async_transfer_start_us;

    if (sd_async_state == SDLOGGER_ASYNC_DATA_DMA)
    {
        uint8_t completed_buffer = sd_writing_buffer;

        sd_logger_last_write_duration_us = duration_us;

        if (duration_us > sd_logger_max_write_duration_us)
        {
            sd_logger_max_write_duration_us = duration_us;
        }

        sd_logger_last_write_bytes = sd_async_data_bytes;
        sd_logger_total_bytes_written += sd_async_data_bytes;
        sd_logger_bytes_written = sd_logger_total_bytes_written;

        sd_next_data_sector_offset +=
            sd_async_transfer_sector_count;

        sd_logger_async_data_complete_count++;

        if (sd_dma_retry_count_current != 0U)
        {
            sd_logger_dma_retry_success_count++;
        }

        sd_dma_retry_count_current = 0U;
        sd_logger_dma_retry_current = 0U;

        sd_logger_flush_count++;
        sd_buffers_since_sync++;

        sd_buffer_fill[completed_buffer] = 0UL;
        sd_buffer_state[completed_buffer] = SDLOGGER_BUFFER_EMPTY;
        sd_buffer_ready_order[completed_buffer] = 0UL;

        sd_writing_buffer = SDLOGGER_INVALID_BUFFER;
        sd_write_offset = 0UL;

        if (sd_active_buffer == SDLOGGER_INVALID_BUFFER)
        {
            SDLogger_SelectNewActiveBuffer(completed_buffer);
        }

        /* P47: DATA has priority. Keep one moving guard pending at the newest
         * data end and write it only after ready DATA/backlog has drained. */
        sd_async_state = SDLOGGER_ASYNC_IDLE;
        if (sd_logger_guard_pending == 0U)
        {
            sd_logger_guard_pending = 1U;
        }
        else
        {
            sd_logger_guard_deferred_count++;
        }
    }
    else if (sd_async_state == SDLOGGER_ASYNC_GUARD_DMA)
    {
        sd_logger_last_guard_write_duration_us = duration_us;

        if (duration_us > sd_logger_max_guard_write_duration_us)
        {
            sd_logger_max_guard_write_duration_us = duration_us;
        }

        sd_logger_guard_write_count++;
        sd_logger_guard_pending = 0U;

        if (sd_dma_retry_count_current != 0U)
        {
            sd_logger_dma_retry_success_count++;
        }

        sd_dma_retry_count_current = 0U;
        sd_logger_dma_retry_current = 0U;

        sd_async_state = SDLOGGER_ASYNC_IDLE;
    }

    SDLogger_UpdateLiveDebug();
#endif
}

/* -------------------------------------------------------------------------- */

static uint8_t SDLogger_HasPendingWriteWork(void)
{
    if (SDLogger_RingCount() != 0UL)
    {
        return 1U;
    }

    if (SDLogger_FindReadyBuffer() != SDLOGGER_INVALID_BUFFER)
    {
        return 1U;
    }

    if (sd_writing_buffer != SDLOGGER_INVALID_BUFFER)
    {
        return 1U;
    }

    if (sd_async_state != SDLOGGER_ASYNC_IDLE)
    {
        return 1U;
    }
    if (sd_logger_guard_pending != 0U)
    {
        return 1U;
    }

    return 0U;
}

/* -------------------------------------------------------------------------- */

static uint8_t SDLogger_DrainAllBlocking(uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();

    while (SDLogger_HasPendingWriteWork() != 0U)
    {
        SDLogger_ServiceAsyncDMA();
        SDLogger_DrainRingToBuffers();

        if (sd_async_state == SDLOGGER_ASYNC_IDLE)
        {
            if (SDLogger_StartReadyBufferDMA() == 0U)
            {
                if ((sd_async_state == SDLOGGER_ASYNC_IDLE) &&
                    (sd_logger_guard_pending != 0U))
                {
                    (void)SDLogger_StartGuardDMA();
                }
            }
        }

        if (sd_logger_ready == 0U)
        {
            return 0U;
        }

        if ((uint32_t)(HAL_GetTick() - start_ms) >= timeout_ms)
        {
            sd_logger_async_timeout_count++;
            SDLogger_HandleWriteFailure(FR_TIMEOUT);
            return 0U;
        }

        __NOP();
    }

    return 1U;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void SDLogger_Init(void)
{
    sd_logger_ring_address =
        (uint32_t)(uintptr_t)&sd_capture_ring[0];
    sd_logger_ring_end_address =
        (uint32_t)(uintptr_t)&sd_capture_ring[APP_SDLOGGER_RING_FRAME_COUNT];

    FRESULT result;

    sd_logger_init_count++;

    sd_logger_initialized = 0U;
    sd_logger_ready = 0U;
    sd_logger_mount_ok = 0U;
    sd_logger_test_done = 0U;
    sd_logger_test_passed = 0U;
    sd_logger_file_open = 0U;
    sd_logger_logging_active = 0U;

    sd_logger_error_count = 0UL;
    sd_logger_write_error_count = 0UL;
    sd_logger_last_result = 0U;
    sd_logger_disk_init_status = STA_NOINIT;
    sd_logger_mount_retry_count = 0U;
    sd_logger_bytes_written = 0UL;
    sd_logger_total_bytes_written = 0UL;
    sd_logger_last_write_bytes = 0UL;

    sd_logger_frame_count = 0UL;
    sd_logger_dropped_frame_count = 0UL;
    sd_logger_missed_period_count = 0UL;
    sd_logger_flush_count = 0UL;
    sd_logger_buffer_overrun_count = 0UL;

    sd_logger_sync_pending = 0U;
    sd_logger_buffers_since_sync = 0UL;
    sd_logger_sync_request_count = 0UL;
    sd_logger_sync_count = 0UL;
    sd_logger_sync_deferred_count = 0UL;
    sd_logger_sync_error_count = 0UL;
    sd_logger_last_sync_duration_us = 0UL;
    sd_logger_max_sync_duration_us = 0UL;

    sd_logger_preallocate_enabled = APP_SDLOGGER_PREALLOCATE_ENABLED;
    sd_logger_preallocate_ok = 0U;
    sd_logger_preallocated_bytes = 0UL;
    sd_logger_preallocation_attempt_count = 0UL;
    sd_logger_preallocation_fallback_count = 0UL;
    sd_logger_preallocation_duration_ms = 0UL;
    sd_logger_finalize_duration_ms = 0UL;
    sd_logger_runtime_sync_suppressed_count = 0UL;
    sd_logger_guard_write_count = 0UL;
    sd_logger_last_guard_write_duration_us = 0UL;
    sd_logger_max_guard_write_duration_us = 0UL;

    sd_logger_raw_dma_enabled = APP_SDLOGGER_RAW_DMA_ENABLED;
    sd_logger_async_state = SDLOGGER_ASYNC_IDLE;
    sd_logger_file_start_sector = 0UL;
    sd_logger_preallocated_sector_count = 0UL;
    sd_logger_next_data_sector_offset = 0UL;
    sd_logger_async_data_start_count = 0UL;
    sd_logger_async_data_complete_count = 0UL;

    sd_logger_dma_retry_attempt_count = 0UL;
    sd_logger_dma_retry_success_count = 0UL;
    sd_logger_dma_retry_exhausted_count = 0UL;
    sd_logger_dma_retry_card_busy_count = 0UL;
    sd_logger_dma_retry_abort_count = 0UL;
    sd_logger_dma_retry_pending = 0U;
    sd_logger_dma_retry_current = 0U;

    sd_logger_runtime_recovery_active = 0U;
    sd_logger_runtime_recovery_count = 0UL;
    sd_logger_runtime_recovery_success_count = 0UL;
    sd_logger_runtime_recovery_failure_count = 0UL;
    sd_logger_runtime_recovery_flight_abort_count = 0UL;
    sd_logger_runtime_recovery_last_us = 0UL;
    sd_logger_runtime_recovery_max_us = 0UL;
    sd_logger_runtime_last_hal_error = 0UL;

    sd_dma_retry_pending_internal = 0U;
    sd_dma_retry_count_current = 0U;
    sd_dma_retry_due_ms = 0UL;
    sd_logger_async_card_busy_poll_count = 0UL;
    sd_logger_async_timeout_count = 0UL;

    sd_logger_motor_write_hold_active = 0U;
    sd_logger_motor_write_hold_entry_count = 0UL;
    sd_logger_motor_write_hold_defer_count = 0UL;
    sd_logger_motor_write_hold_until_ms = 0UL;
    sd_motor_write_hold_until_ms = 0UL;
    sd_motor_write_hold_prev = 0U;
    sd_flight_active_prev = 0U;
    sd_r9_rcs_pulse_complete_prev = 0UL;
    sd_logger_r9_rcs_hold_event_count = 0UL;
    sd_logger_r9_hold_remaining_ms = 0UL;
    sd_logger_r10_ram_mode = 0U;
    sd_logger_r10_ram_frame_count = 0UL;
    sd_r10_ram_decimation_phase = 0U;
    sd_logger_r10_ram_capacity = SDLOGGER_RAM_FLIGHT_CAPACITY;
    sd_logger_r10_ram_overflow_count = 0UL;
    sd_logger_r10_capture_frozen = 0U;
    sd_logger_r10_flush_active = 0U;
    sd_logger_r10_flush_index = 0UL;
    sd_logger_r10_flush_complete = 0U;
    sd_logger_r10_tail_dma_at_entry = 0U;
    sd_logger_r10_physical_sd_fault_count = 0UL;
    sd_logger_r10_postflight_reason = 0U;
    sd_logger_r10_entry_ms = 0UL;
    sd_logger_r10_flush_start_ms = 0UL;
    sd_r10_postflight_quiet_since_ms = 0UL;
    sd_r10_postflight_cmd_path_prepared = 0U;
    sd_r10_replay_fault_baseline = 0UL;

    sd_logger_last_write_duration_us = 0UL;
    sd_logger_max_write_duration_us = 0UL;
    sd_logger_last_capture_us = 0UL;
    sd_logger_last_sequence = 0UL;
    sd_logger_last_crc16 = 0U;

    sd_logger_fast_imu_push_count = 0UL;
    sd_logger_fast_imu_duplicate_skip_count = 0UL;
    sd_logger_fast_imu_history_count = 0UL;
    sd_logger_fast_imu_last_valid_mask = 0U;
    sd_logger_last_imu_age_us = 65535U;
    sd_logger_last_baro_age_us = 65535U;
    sd_logger_last_lidar_age_us = 65535U;

    sd_logger_capture_timer_started = 0U;
    sd_logger_capture_timer_start_error_count = 0UL;
    sd_logger_timer_irq_count = 0UL;
    sd_logger_timer_last_interval_us = 0UL;
    sd_logger_timer_min_interval_us = 0UL;
    sd_logger_timer_max_interval_us = 0UL;
    sd_logger_timer_last_isr_duration_us = 0UL;
    sd_logger_timer_max_isr_duration_us = 0UL;

    sd_logger_source_publish_count = 0UL;
    sd_logger_source_active_index = 0U;

    sd_logger_ring_count = 0UL;
    sd_logger_ring_capacity = APP_SDLOGGER_RING_FRAME_COUNT - 1U;
    sd_logger_ring_high_watermark = 0UL;
    sd_logger_ring_push_count = 0UL;
    sd_logger_ring_pop_count = 0UL;
    sd_logger_ring_overrun_count = 0UL;
    sd_logger_drain_last_us = 0UL;
    sd_logger_drain_max_us = 0UL;
    sd_logger_drain_budget_yield_count = 0UL;
    sd_logger_backpressure_level = 0U;
    sd_logger_backpressure_entry_count = 0UL;
    sd_logger_backpressure_critical_entry_count = 0UL;
    sd_logger_guard_pending = 0U;
    sd_logger_guard_deferred_count = 0UL;

    memset(sd_guard_sector, 0, sizeof(sd_guard_sector));
    memset(sd_buffers, 0, sizeof(sd_buffers));
    memset(sd_buffer_fill, 0, sizeof(sd_buffer_fill));
    memset(sd_buffer_state, 0, sizeof(sd_buffer_state));
    memset(sd_buffer_ready_order, 0, sizeof(sd_buffer_ready_order));
    memset(sd_capture_ring, 0, sizeof(sd_capture_ring));
    memset(sd_source_snapshots, 0, sizeof(sd_source_snapshots));
    memset(sd_fast_imu_history, 0, sizeof(sd_fast_imu_history));

    sd_ring_head = 0UL;
    sd_ring_tail = 0UL;
    sd_source_active_index = 0U;

    sd_fast_imu_history_head = 0U;
    sd_fast_imu_history_count = 0U;
    sd_fast_imu_last_timestamp_us = 0UL;

    sd_timer_previous_capture_us = 0UL;

    sd_active_buffer = 0U;
    sd_writing_buffer = SDLOGGER_INVALID_BUFFER;
    sd_next_ready_order = 1UL;
    sd_last_started_ready_order = 0UL;
    sd_logger_fifo_order_fault_count = 0UL;
    sd_logger_fifo_last_started_order = 0UL;
    sd_logger_fifo_next_ready_order = 1UL;
    sd_write_offset = 0UL;
    sd_sequence = 0UL;

    sd_sync_pending = 0U;
    sd_sync_defer_reported = 0U;
    sd_buffers_since_sync = 0UL;
    sd_preallocated_bytes = 0UL;

    sd_async_state = SDLOGGER_ASYNC_IDLE;
    sd_file_start_sector = 0UL;
    sd_preallocated_sector_count = 0UL;
    sd_next_data_sector_offset = 0UL;
    sd_async_transfer_start_us = 0UL;
    sd_async_transfer_start_ms = 0UL;
    sd_async_transfer_sector_count = 0UL;
    sd_async_data_bytes = 0UL;
    sd_async_next_card_poll_ms = 0UL;

    SDLogger_SelectNewActiveBuffer(0U);
    for (uint8_t buffer_index = 1U;
         buffer_index < SDLOGGER_BUFFER_COUNT;
         buffer_index++)
    {
        sd_buffer_state[buffer_index] = SDLOGGER_BUFFER_EMPTY;
    }

    /* V46 SD bring-up: initialize disk layer explicitly before FatFS mount.
     * This restores the proven startup sequence used by the earlier working SD build. */
    (void)f_mount(0, (TCHAR const *)SDPath, 0U);
    sd_logger_disk_init_status = (uint8_t)disk_initialize(0U);

    if ((sd_logger_disk_init_status & STA_NOINIT) != 0U)
    {
        HAL_Delay(20U);
        sd_logger_mount_retry_count++;
        sd_logger_disk_init_status = (uint8_t)disk_initialize(0U);
    }

    if ((sd_logger_disk_init_status & STA_NOINIT) != 0U)
    {
        sd_logger_last_result = (uint8_t)FR_NOT_READY;
        sd_logger_error_count++;
        sd_logger_initialized = 1U;
        SDLogger_UpdateLiveDebug();
        return;
    }

    result = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
    sd_logger_last_result = (uint8_t)result;

    if (result != FR_OK)
    {
        /* One clean disk re-init + remount retry. Never auto-format in V46. */
        (void)f_mount(0, (TCHAR const *)SDPath, 0U);
        HAL_Delay(20U);
        sd_logger_mount_retry_count++;
        sd_logger_disk_init_status = (uint8_t)disk_initialize(0U);

        if ((sd_logger_disk_init_status & STA_NOINIT) == 0U)
        {
            result = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
            sd_logger_last_result = (uint8_t)result;
        }
    }

    if (result != FR_OK)
    {
        sd_logger_error_count++;
        sd_logger_initialized = 1U;
        SDLogger_UpdateLiveDebug();
        return;
    }

    sd_logger_mount_ok = 1U;

    result = f_open(
        &sd_log_file,
        APP_SDLOGGER_FILE_NAME,
        FA_CREATE_ALWAYS | FA_WRITE
    );

    sd_logger_last_result = (uint8_t)result;

    if (result != FR_OK)
    {
        sd_logger_error_count++;
        sd_logger_initialized = 1U;
        SDLogger_UpdateLiveDebug();
        return;
    }

    sd_logger_file_open = 1U;

    if (SDLogger_PreallocateFile() == 0U)
    {
        sd_logger_error_count++;
        (void)f_close(&sd_log_file);
        sd_logger_file_open = 0U;
        sd_logger_initialized = 1U;
        SDLogger_UpdateLiveDebug();
        return;
    }

    sd_logger_ready = 1U;
    sd_logger_logging_active = 1U;

    /* Existing test flags now mean mount + binary file open passed. */
    sd_logger_test_done = 1U;
    sd_logger_test_passed = 1U;

    sd_next_sample_us = 0UL;
    SDLogger_PublishSources();

    __HAL_TIM_SET_COUNTER(&htim5, 0UL);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);

    if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
    {
        sd_logger_capture_timer_start_error_count++;
        sd_logger_error_count++;
        sd_logger_logging_active = 0U;
        sd_logger_ready = 0U;
        (void)f_close(&sd_log_file);
        sd_logger_file_open = 0U;
        sd_logger_initialized = 1U;
        SDLogger_UpdateLiveDebug();
        return;
    }

    sd_logger_capture_timer_started = 1U;
    sd_logger_initialized = 1U;

    SDLogger_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */

void SDLogger_Update(void)
{
#if (APP_SDLOGGER_ENABLED != 0U)
    sd_logger_update_count++;

    if ((sd_logger_logging_active == 0U) ||
        (sd_logger_ready == 0U))
    {
        return;
    }

    /* R3R10 flight entry is an authority switch, not a timed blackout. From
     * the first main-loop observation of PE9 onward, no NEW SDIO command is
     * launched until touchdown or latched E-stop plus a quiet window. */
    if ((sd_logger_r10_ram_mode == 0U) &&
        (PreflightTrigger_IsFlightActive() != 0U))
    {
        SDLogger_R10EnterRamFlightMode();
    }

    if (sd_logger_r10_ram_mode != 0U)
    {
        /* A DMA command launched before PE9 may still be completing. Service
         * only that existing transaction; never drain/launch/sync in flight. */
        if (sd_async_state != SDLOGGER_ASYNC_IDLE)
        {
            SDLogger_ServiceAsyncDMA();
        }

        SDLogger_R10ServicePostflight();
        SDLogger_UpdateLiveDebug();
        return;
    }

    /* Ground/preflight and completed-postflight behavior remains the proven
     * non-blocking raw-SD DMA path. */
    SDLogger_ServiceAsyncDMA();
    SDLogger_DrainRingToBuffers();

    if (sd_async_state == SDLOGGER_ASYNC_IDLE)
    {
        if (SDLogger_StartReadyBufferDMA() == 0U)
        {
            if ((sd_async_state == SDLOGGER_ASYNC_IDLE) &&
                (sd_logger_guard_pending != 0U))
            {
                if (SDLogger_RingCount() <= APP_SDLOGGER_GUARD_RESUME_MAX_RING_FRAMES)
                {
                    (void)SDLogger_StartGuardDMA();
                }
                else
                {
                    sd_logger_guard_deferred_count++;
                }
            }
        }
    }

    SDLogger_PerformSyncIfIdle();
    SDLogger_UpdateLiveDebug();
#else
    (void)sd_logger_update_count;
#endif
}

/* -------------------------------------------------------------------------- */

void SDLogger_Stop(void)
{
    FRESULT result = FR_OK;

    if (sd_logger_capture_timer_started != 0U)
    {
        (void)HAL_TIM_Base_Stop_IT(&htim5);
        sd_logger_capture_timer_started = 0U;
    }

    sd_logger_logging_active = 0U;

    /* Drain all complete captured frames and full writer buffers. */
    (void)SDLogger_DrainAllBlocking(APP_SDLOGGER_STOP_TIMEOUT_MS);

    /* Convert the final partially filled buffer into a DMA-ready buffer. */
    if ((sd_active_buffer < SDLOGGER_BUFFER_COUNT) &&
        (sd_buffer_state[sd_active_buffer] == SDLOGGER_BUFFER_FILLING) &&
        (sd_buffer_fill[sd_active_buffer] > 0UL))
    {
        SDLogger_MarkBufferReady(sd_active_buffer);
        sd_active_buffer = SDLOGGER_INVALID_BUFFER;
    }

    (void)SDLogger_DrainAllBlocking(APP_SDLOGGER_STOP_TIMEOUT_MS);

    if (sd_logger_file_open != 0U)
    {
        uint32_t finalize_start_ms = HAL_GetTick();

#if (APP_SDLOGGER_PREALLOCATE_ENABLED != 0U)
        /* Remove the unused preallocated tail after all raw DMA is complete. */
        result = f_lseek(
            &sd_log_file,
            (FSIZE_t)sd_logger_total_bytes_written
        );
        sd_logger_last_result = (uint8_t)result;

        if (result == FR_OK)
        {
            result = f_truncate(&sd_log_file);
            sd_logger_last_result = (uint8_t)result;
        }

        if (result != FR_OK)
        {
            sd_logger_error_count++;
        }
#endif

        if ((result == FR_OK) && (sd_logger_ready != 0U))
        {
            sd_sync_pending = 1U;
            sd_sync_defer_reported = 0U;
            SDLogger_PerformSyncIfIdle();
        }

        result = f_close(&sd_log_file);
        sd_logger_last_result = (uint8_t)result;

        if (result != FR_OK)
        {
            sd_logger_error_count++;
        }

        sd_logger_finalize_duration_ms = HAL_GetTick() - finalize_start_ms;
        sd_logger_file_open = 0U;
    }

    sd_logger_ready = 0U;
    sd_async_state = SDLOGGER_ASYNC_IDLE;
    SDLogger_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */

void SDLogger_RequestSync(void)
{
    SDLogger_RequestSyncInternal();
}

/* -------------------------------------------------------------------------- */

uint8_t SDLogger_IsReady(void)
{
    return sd_logger_ready;
}

uint8_t SDLogger_IsLogging(void)
{
    return sd_logger_logging_active;
}

uint8_t SDLogger_TestPassed(void)
{
    return sd_logger_test_passed;
}
/* -------------------------------------------------------------------------- */
/* HAL timer callback                                                         */
/* -------------------------------------------------------------------------- */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim != NULL) && (htim->Instance == TIM5))
    {
        SDLogger_TimerCaptureISR();
    }
}
