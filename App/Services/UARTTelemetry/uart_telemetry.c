#include "Services/UARTTelemetry/uart_telemetry.h"
#include "app.h"

#include "Common/app_config.h"
#include "Common/app_version.h"
#include "Common/runtime_delay_guard.h"
#include "Core/Scheduler/scheduler.h"
#include "Modules/Control/AttitudeControl/attitude_control.h"
#include "Modules/Control/GeneratedFlightControl/generated_flight_control.h"
#include "Modules/Control/TaragayFlightLogic/taragay_flight_logic.h"
#include "Modules/Control/NeedleValve/needle_valve_controller.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"
#include "Modules/NRF24/nrf24.h"
#include "Modules/RemoteControl/remote_control.h"
#include "Modules/Sensors/Lidar/Lidar.h"
#include "Modules/Sensors/Barometer/barometer.h"
#include "Modules/Sensors/IMU/imu.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"
#include "Services/PreflightTrigger/preflight_trigger.h"
#include "Services/NeedleValveIntegrationTest/needle_valve_integration_test.h"
#include "Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.h"
#include "Services/NRFTelemetry/nrf_telemetry.h"
#include "Services/SDLogger/sd_logger.h"
#include "Services/SolenoidOutput/solenoid_output.h"
#include "Services/SystemMonitor/system_monitor.h"
#include "Services/Timebase/timebase.h"
#include "usart.h"

#include "main.h"

#include <stdint.h>

/*
 * V66/P55 repeated-pattern fast IMU configuration repair stream
 * ----------------------------
 * A single static buffer is handed to DMA1 Stream6. It is never modified
 * while DMA owns it. No delay, polling transmit, printf or dynamic allocation
 * is used in this path.
 */
#define UART_TELEMETRY_BUFFER_SIZE       5120U
#define UART_TELEMETRY_PERIOD_MS \
    APP_V55_UART_TELEMETRY_PERIOD_MS
#define UART_TELEMETRY_INVALID_AGE_MS 0xFFFFFFFFUL

extern volatile uint32_t cpu_load_percent_x100;
extern volatile uint32_t cpu_external_load_percent_x100;
extern volatile uint32_t scheduler_slow_task_defer_count;
extern volatile uint32_t v87_sd_update_last_us;
extern volatile uint32_t v87_sd_update_max_us;
extern volatile uint32_t p34_bg_fresh_last_us;
extern volatile uint32_t p34_bg_fresh_max_us;
extern volatile uint32_t p34_bg_remote_last_us;
extern volatile uint32_t p34_bg_remote_max_us;
extern volatile uint32_t p34_bg_control_last_us;
extern volatile uint32_t p34_bg_control_max_us;
extern volatile uint32_t p34_bg_uart_last_us;
extern volatile uint32_t p34_bg_uart_max_us;
extern volatile uint32_t p34_covariance_last_us;
extern volatile uint32_t p34_covariance_max_us;
extern volatile uint32_t p34_covariance_service_count;
extern volatile uint32_t p34_sd_defer_count;
extern volatile uint32_t p34_control_defer_count;
extern volatile uint32_t p34_uart_defer_count;
extern volatile uint8_t v30_stop_latched;
extern volatile uint8_t nrf24_connected;
extern volatile uint32_t nrf24_rx_count;
extern volatile uint32_t nrf24_error_count;
extern volatile uint8_t nrf24_status_reg;
extern volatile uint8_t nrf24_config_reg;
extern volatile uint8_t nrf24_rf_ch_reg;
extern volatile uint8_t nrf24_rf_setup_reg;
extern volatile uint8_t nrf24_fifo_status_reg;

/* P32 SD boot/host diagnostics are UART-first; no Live Expressions required. */
extern volatile uint8_t sd_bsp_hal_init_status;
extern volatile uint8_t sd_bsp_wide_bus_status;
extern volatile uint32_t sd_bsp_hal_error_code;
extern volatile uint32_t sd_v47_init_attempt_count;
extern volatile uint32_t sd_v47_host_reset_count;
extern volatile uint32_t sd_v47_last_attempt_error;
extern volatile uint8_t sd_v47_recovered;
/* R8R35R3 SD-service throughput diagnostics (read-only). */
extern volatile uint32_t sd_logger_ring_push_count;
extern volatile uint32_t sd_logger_ring_pop_count;
extern volatile uint32_t sd_logger_async_data_start_count;
extern volatile uint32_t sd_logger_async_data_complete_count;
extern volatile uint32_t sd_logger_frame_count;
extern volatile uint32_t sd_logger_drain_last_us;
extern volatile uint32_t sd_logger_drain_max_us;
extern volatile uint32_t sd_logger_drain_budget_yield_count;
extern volatile uint32_t sd_logger_max_write_duration_us;
/* R8R35R3R4 host-busy launch admission diagnostics. */
extern volatile uint32_t sd_logger_dma_retry_card_busy_count;
extern volatile uint32_t sd_bsp_hal_state;
extern volatile uint32_t sd_dma_start_error_count;
extern volatile uint32_t sd_r7_write_start_fail_count;
extern volatile uint32_t sd_r7_first_fail_time_ms;
extern volatile uint32_t sd_r7_first_fail_hal_status;
extern volatile uint32_t sd_r7_first_fail_hsd_state;
extern volatile uint32_t sd_r7_first_fail_hsd_error;
extern volatile uint32_t sd_r7_first_fail_hsd_context;
extern volatile uint32_t sd_r7_first_fail_dma_state;
extern volatile uint32_t sd_r7_first_fail_dma_error;
extern volatile uint32_t sd_r7_first_fail_sdio_sta;
extern volatile uint32_t sd_r7_first_fail_sdio_dctrl;
extern volatile uint32_t sd_r7_first_fail_sdio_dcount;
extern volatile uint32_t sd_r7_last_fail_time_ms;
extern volatile uint32_t sd_r7_last_fail_hal_status;
extern volatile uint32_t sd_r7_last_fail_hsd_state;
extern volatile uint32_t sd_r7_last_fail_hsd_error;
extern volatile uint32_t sd_r7_last_fail_hsd_context;
extern volatile uint32_t sd_r7_last_fail_dma_state;
extern volatile uint32_t sd_r7_last_fail_dma_error;
extern volatile uint32_t sd_r7_last_fail_sdio_sta;
extern volatile uint32_t sd_r7_last_fail_sdio_dctrl;
extern volatile uint32_t sd_r7_last_fail_sdio_dcount;
extern volatile uint32_t v87_sd_update_count;
extern volatile uint32_t p112_sd_suppressed_count;
extern volatile uint32_t sd_p37_soft_recovery_count;
extern volatile uint32_t sd_p37_runtime_reinit_count;
extern volatile uint32_t sd_p37_runtime_reinit_success_count;
extern volatile uint32_t sd_p37_runtime_reinit_failure_count;
extern volatile uint32_t sd_p37_last_runtime_error;
extern volatile uint8_t sd_logger_motor_write_hold_active;
extern volatile uint32_t sd_logger_r9_hold_remaining_ms;
extern volatile uint32_t sd_logger_r9_rcs_hold_event_count;
extern volatile uint32_t sd_bsp_sdio_clkcr;

volatile uint8_t uart_telemetry_initialized = 0U;
volatile uint8_t uart_telemetry_streaming = 0U;
volatile uint32_t uart_telemetry_send_count = 0UL;
volatile uint32_t uart_telemetry_busy_skip_count = 0UL;
volatile uint32_t uart_telemetry_format_error_count = 0UL;
volatile uint32_t uart_telemetry_frame_sequence = 0UL;
volatile uint32_t uart_telemetry_last_service_ms = 0UL;
volatile uint16_t uart_telemetry_last_message_length = 0U;
volatile uint16_t uart_telemetry_last_crc16 = 0U;

typedef struct
{
    uint32_t sequence;
    uint32_t time_ms;

    PreflightTriggerStatus_t preflight;
    SensorData_t sensor;
    BarometerData_t baro;
    LidarData_t lidar;
    FullStateESKFData_t eskf;
    IMU_PatternDiagnostic_t imu_pattern_diag;
    IMU_StaleDiagnostic_t imu_stale_diag;
    AttitudeControlStatus_t rcs;
    SolenoidOutputStatus_t solenoid;
    GeneratedFlightControlStatus_t main_control;
    TaragayFlightLogicStatus_t flight_logic;
    NeedleValveStatus_t needle;
    SystemStatus_t system;

    uint32_t imu_age_ms;
    uint32_t imu_recovery_state;
    uint32_t imu_recovery_step;
    uint32_t imu_recovery_count;
    uint32_t imu_recovery_attempts;
    uint32_t imu_recovery_failures;
    uint32_t imu_recovery_last_us;
    uint32_t imu_recovery_max_us;
    uint32_t imu_stale_count;
    uint32_t imu_pattern_errors;
    uint32_t imu_pattern_retries;
    uint32_t imu_pattern_retry_success;
    uint32_t imu_pattern_recovery_escalations;
    uint32_t imu_fast_config_checks;
    uint32_t imu_fast_config_repair_attempts;
    uint32_t imu_fast_config_repair_success;
    uint32_t imu_fast_config_repair_failures;
    uint32_t imu_fast_config_repair_last_us;
    uint32_t imu_fast_config_repair_max_us;
    uint32_t imu_stale_fast_config_checks;
    uint32_t imu_stale_fast_config_repair_attempts;
    uint32_t imu_stale_fast_config_repair_success;
    uint32_t imu_stale_fast_config_repair_failures;
    uint32_t imu_stale_retry_success;
    uint32_t imu_stale_recovery_escalations;
    uint32_t imu_stale_fast_config_repair_last_us;
    uint32_t imu_stale_fast_config_repair_max_us;
    uint32_t imu_stale_warmup_events;
    uint32_t imu_stale_warmup_polls;
    uint32_t imu_stale_warmup_success;
    uint32_t imu_stale_warmup_timeouts;
    uint32_t imu_stale_warmup_first_ready_us;
    uint32_t imu_stale_warmup_max_ready_us;
    uint32_t imu_stale_warmup_last_status;
    uint32_t imu_stale_warmup_active;
    uint32_t imu_invalid_samples;
    uint32_t imu_redundant_rejects;
    uint32_t lidar_age_ms;
    uint32_t lidar_recovery_active;
    uint32_t lidar_recovery_step;
    uint32_t lidar_recovery_attempts;
    uint32_t lidar_recovery_success;
    uint32_t lidar_recovery_failures;
    uint32_t lidar_recovery_last_us;
    uint32_t lidar_recovery_max_us;
    uint32_t lidar_recovery_step_max_us;
    uint32_t eskf_public_age_ms;
    uint32_t scheduler_realign_count;
    uint32_t cpu_load_x100;
    uint32_t cpu_external_load_x100;
    uint32_t sd_update_last_us;
    uint32_t sd_update_max_us;
    uint32_t sd_drain_last_us;
    uint32_t sd_drain_max_us;
    uint32_t sd_drain_yields;
    uint32_t covariance_last_us;
    uint32_t covariance_max_us;
    uint32_t covariance_service_count;
    uint32_t gravity_joseph_update_count;
    uint32_t gravity_joseph_fault_count;
    uint32_t scheduler_slow_defers;
    uint32_t bg_fresh_last_us;
    uint32_t bg_remote_last_us;
    uint32_t bg_control_last_us;
    uint32_t bg_uart_last_us;
    uint32_t bg_uart_max_us;
    uint32_t sd_defer_count;
    uint32_t control_defer_count;
    uint32_t uart_defer_count;
    uint32_t cpu_task_load_x100[5];
    uint32_t task_last_exec_us[5];
    uint32_t task_max_exec_us[5];
    uint32_t task_deadline_miss[5];

    uint32_t nrf_rx_count;
    uint32_t nrf_packet_age_ms;
    uint32_t nrf_error_count;
    uint32_t nrf_invalid_packet_count;
    uint32_t nrf_irq_count;

    uint32_t sd_error_count;
    uint32_t sd_write_error_count;
    uint32_t sd_dropped_frame_count;
    uint32_t sd_ring_overrun_count;
    uint32_t sd_async_timeout_count;
    uint32_t sd_ring_count;
    uint32_t sd_ring_high_watermark;
    uint32_t sd_backpressure_level;
    uint32_t sd_backpressure_entries;
    uint32_t sd_backpressure_critical_entries;
    uint32_t sd_guard_pending;
    uint32_t sd_guard_deferred;
    uint32_t sd_card_busy_polls;
    uint32_t sd_write_max_us;
    uint32_t sd_guard_max_us;
    uint32_t sd_fifo_order_faults;
    uint32_t sd_fifo_last_started_order;
    uint32_t sd_fifo_next_ready_order;
    uint32_t sd_frame_count;
    uint32_t sd_source_publish_count;
    uint32_t sd_host_init_attempts;
    uint32_t sd_host_reset_count;
    uint32_t sd_hal_error_code;
    uint32_t sd_last_attempt_error;
    uint32_t sd_soft_recoveries;
    uint32_t sd_runtime_reinits;
    uint32_t sd_runtime_reinit_success;
    uint32_t sd_runtime_reinit_failures;
    uint32_t sd_runtime_last_error;
    uint32_t sd_runtime_recovery_count;
    uint32_t sd_runtime_recovery_success;
    uint32_t sd_runtime_recovery_failures;
    uint32_t sd_runtime_recovery_flight_aborts;
    uint32_t sd_runtime_recovery_last_us;
    uint32_t sd_runtime_recovery_max_us;

    uint32_t uart_dma_error_count;
    uint32_t uart_busy_skip_count;

    uint8_t nrf_connected;
    uint8_t nrf_link;
    uint8_t nrf_command;
    uint8_t nrf_flags;
    uint8_t nrf_status_reg;
    uint8_t nrf_config_reg;
    uint8_t nrf_channel_reg;
    uint8_t nrf_rf_setup_reg;
    uint8_t nrf_fifo_status_reg;
    uint8_t stop_latched;

    uint8_t sd_initialized;
    uint8_t sd_mount_ok;
    uint8_t sd_file_open;
    uint8_t sd_ready;
    uint8_t sd_logging;
    uint8_t sd_last_result;
    uint8_t sd_disk_status;
    uint8_t sd_mount_retry_count;
    uint8_t sd_hal_init_status;
    uint8_t sd_wide_status;
    uint8_t sd_recovered;

    uint8_t system_ok;
    uint8_t system_fault;
    uint32_t fast_fault;
    uint32_t baro_raw_spike_rejects;
    uint32_t baro_raw_step_confirms;
    uint32_t eskf_lidar_soft_reacq_active;
    uint32_t eskf_lidar_soft_reacq_count;
    uint32_t eskf_lidar_soft_reacq_success;
} UARTTelemetrySnapshot_t;

static char uart_telemetry_buffer[UART_TELEMETRY_BUFFER_SIZE];
static UARTTelemetrySnapshot_t uart_snapshot;
static uint8_t uart_telemetry_stage = 0U;
static uint32_t uart_telemetry_previous_service_ms = 0UL;

static int32_t UARTTelemetry_ScaleFloat(float value, float scale)
{
    float scaled;

    if (value != value)
    {
        return 0;
    }

    scaled = value * scale;

    if (scaled > 2147483000.0f)
    {
        return 2147483000L;
    }

    if (scaled < -2147483000.0f)
    {
        return -2147483000L;
    }

    return (scaled >= 0.0f)
        ? (int32_t)(scaled + 0.5f)
        : (int32_t)(scaled - 0.5f);
}

static uint32_t UARTTelemetry_SampleAgeMs(uint32_t sample_timestamp_us)
{
    uint32_t now_us;

    if (sample_timestamp_us == 0UL)
    {
        return UART_TELEMETRY_INVALID_AGE_MS;
    }

    /* Use the same TIM2/micros timebase as the sensor timestamp. */
    now_us = micros();
    return (uint32_t)(now_us - sample_timestamp_us) / 1000UL;
}

static void UARTTelemetry_TakeSnapshot(void)
{
    const SensorData_t *sensor = SensorManager_GetDataPtr();
    const LidarData_t *lidar = Lidar_GetDataPtr();
    const BarometerData_t *baro = Barometer_GetDataPtr();
    const FullStateESKFData_t *eskf = FullStateESKF_GetDataPtr();
    uint32_t now_ms = HAL_GetTick();

    uart_snapshot.sequence = uart_telemetry_frame_sequence;
    uart_snapshot.time_ms = now_ms;
    uart_snapshot.preflight = PreflightTrigger_GetStatus();

    if (sensor != 0)
    {
        uart_snapshot.sensor = *sensor;
    }

    if (baro != 0)
    {
        uart_snapshot.baro = *baro;
    }

    if (lidar != 0)
    {
        uart_snapshot.lidar = *lidar;
    }

    (void)IMU_GetPatternDiagnostic(&uart_snapshot.imu_pattern_diag);
    (void)IMU_GetStaleDiagnostic(&uart_snapshot.imu_stale_diag);

    if (eskf != 0)
    {
        uart_snapshot.eskf = *eskf;
    }

    uart_snapshot.rcs = AttitudeControl_GetStatus();
    uart_snapshot.solenoid = SolenoidOutput_GetStatus();
    uart_snapshot.main_control = GeneratedFlightControl_GetStatus();
    uart_snapshot.flight_logic = TaragayFlightLogic_GetStatus();
#if (APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE != 0U)
    uart_snapshot.needle = NeedleValveAutonomousControl_GetTelemetryStatus();
#elif (APP_NEEDLE_RAW_CHARACTERIZATION_MODE != 0U)
    uart_snapshot.needle = NeedleValveIntegrationTest_GetTelemetryStatus();
#else
    uart_snapshot.needle = NeedleValveController_GetStatus();
#endif
    uart_snapshot.system = SystemMonitor_GetStatus();

    uart_snapshot.imu_age_ms = UARTTelemetry_SampleAgeMs(
        uart_snapshot.sensor.imu_sample_timestamp_us);
    uart_snapshot.imu_recovery_state = IMU_GetRecoveryState();
    uart_snapshot.imu_recovery_step = IMU_GetRecoveryStep();
    uart_snapshot.imu_recovery_count = IMU_GetRecoveryCount();
    uart_snapshot.imu_recovery_attempts = IMU_GetRecoveryAttemptCount();
    uart_snapshot.imu_recovery_failures = IMU_GetRecoveryFailureCount();
    uart_snapshot.imu_recovery_last_us = IMU_GetRecoveryLastDurationUs();
    uart_snapshot.imu_recovery_max_us = IMU_GetRecoveryMaxDurationUs();
    uart_snapshot.imu_stale_count = IMU_GetStaleCount();
    uart_snapshot.imu_pattern_errors = IMU_GetPatternErrorCount();
    uart_snapshot.imu_pattern_retries = IMU_GetPatternRetryCount();
    uart_snapshot.imu_pattern_retry_success = IMU_GetPatternRetrySuccessCount();
    uart_snapshot.imu_pattern_recovery_escalations =
        IMU_GetPatternRecoveryEscalationCount();
    uart_snapshot.imu_fast_config_checks = IMU_GetFastConfigCheckCount();
    uart_snapshot.imu_fast_config_repair_attempts =
        IMU_GetFastConfigRepairAttemptCount();
    uart_snapshot.imu_fast_config_repair_success =
        IMU_GetFastConfigRepairSuccessCount();
    uart_snapshot.imu_fast_config_repair_failures =
        IMU_GetFastConfigRepairFailureCount();
    uart_snapshot.imu_fast_config_repair_last_us =
        IMU_GetFastConfigRepairLastDurationUs();
    uart_snapshot.imu_fast_config_repair_max_us =
        IMU_GetFastConfigRepairMaxDurationUs();
    uart_snapshot.imu_stale_fast_config_checks =
        IMU_GetStaleFastConfigCheckCount();
    uart_snapshot.imu_stale_fast_config_repair_attempts =
        IMU_GetStaleFastConfigRepairAttemptCount();
    uart_snapshot.imu_stale_fast_config_repair_success =
        IMU_GetStaleFastConfigRepairSuccessCount();
    uart_snapshot.imu_stale_fast_config_repair_failures =
        IMU_GetStaleFastConfigRepairFailureCount();
    uart_snapshot.imu_stale_retry_success =
        IMU_GetStaleRetrySuccessCount();
    uart_snapshot.imu_stale_recovery_escalations =
        IMU_GetStaleRecoveryEscalationCount();
    uart_snapshot.imu_stale_fast_config_repair_last_us =
        IMU_GetStaleFastConfigRepairLastDurationUs();
    uart_snapshot.imu_stale_fast_config_repair_max_us =
        IMU_GetStaleFastConfigRepairMaxDurationUs();
    uart_snapshot.imu_stale_warmup_events = IMU_GetStaleWarmupEventCount();
    uart_snapshot.imu_stale_warmup_polls = IMU_GetStaleWarmupPollCount();
    uart_snapshot.imu_stale_warmup_success = IMU_GetStaleWarmupSuccessCount();
    uart_snapshot.imu_stale_warmup_timeouts = IMU_GetStaleWarmupTimeoutCount();
    uart_snapshot.imu_stale_warmup_first_ready_us = IMU_GetStaleWarmupFirstReadyUs();
    uart_snapshot.imu_stale_warmup_max_ready_us = IMU_GetStaleWarmupMaxReadyUs();
    uart_snapshot.imu_stale_warmup_last_status = IMU_GetStaleWarmupLastStatus();
    uart_snapshot.imu_stale_warmup_active = IMU_GetStaleWarmupActive();
    uart_snapshot.imu_invalid_samples = IMU_GetInvalidSampleCount();
    uart_snapshot.imu_redundant_rejects = IMU_GetRedundantRejectCount();
    uart_snapshot.lidar_age_ms = UARTTelemetry_SampleAgeMs(
        uart_snapshot.lidar.last_sample_timestamp_us);
    uart_snapshot.lidar_recovery_active = uart_snapshot.lidar.recovery_active;
    uart_snapshot.lidar_recovery_step = uart_snapshot.lidar.recovery_step;
    uart_snapshot.lidar_recovery_attempts = uart_snapshot.lidar.recovery_attempt_count;
    uart_snapshot.lidar_recovery_success = uart_snapshot.lidar.recovery_success_count;
    uart_snapshot.lidar_recovery_failures = uart_snapshot.lidar.recovery_failure_count;
    uart_snapshot.lidar_recovery_last_us = uart_snapshot.lidar.recovery_last_duration_us;
    uart_snapshot.lidar_recovery_max_us = uart_snapshot.lidar.recovery_max_duration_us;
    uart_snapshot.lidar_recovery_step_max_us = uart_snapshot.lidar.recovery_step_max_us;
    uart_snapshot.eskf_public_age_ms = UARTTelemetry_SampleAgeMs(
        uart_snapshot.eskf.last_public_output_timestamp_us);
    uart_snapshot.scheduler_realign_count =
        scheduler_release_realign_count;
    uart_snapshot.cpu_load_x100 = cpu_load_percent_x100;
    uart_snapshot.cpu_external_load_x100 = cpu_external_load_percent_x100;
    uart_snapshot.sd_update_last_us = v87_sd_update_last_us;
    uart_snapshot.sd_update_max_us = v87_sd_update_max_us;
    uart_snapshot.sd_drain_last_us = sd_logger_drain_last_us;
    uart_snapshot.sd_drain_max_us = sd_logger_drain_max_us;
    uart_snapshot.sd_drain_yields = sd_logger_drain_budget_yield_count;
    uart_snapshot.covariance_last_us = p34_covariance_last_us;
    uart_snapshot.covariance_max_us = p34_covariance_max_us;
    uart_snapshot.covariance_service_count = p34_covariance_service_count;
    uart_snapshot.gravity_joseph_update_count = uart_snapshot.eskf.gravity_joseph_update_count;
    uart_snapshot.gravity_joseph_fault_count = uart_snapshot.eskf.gravity_joseph_fault_count;
    uart_snapshot.scheduler_slow_defers = scheduler_slow_task_defer_count;
    uart_snapshot.bg_fresh_last_us = p34_bg_fresh_last_us;
    uart_snapshot.bg_remote_last_us = p34_bg_remote_last_us;
    uart_snapshot.bg_control_last_us = p34_bg_control_last_us;
    uart_snapshot.bg_uart_last_us = p34_bg_uart_last_us;
    uart_snapshot.bg_uart_max_us = p34_bg_uart_max_us;
    uart_snapshot.sd_defer_count = p34_sd_defer_count;
    uart_snapshot.control_defer_count = p34_control_defer_count;
    uart_snapshot.uart_defer_count = p34_uart_defer_count;
    uart_snapshot.cpu_task_load_x100[0] = cpu_task0_load_percent_x100;
    uart_snapshot.cpu_task_load_x100[1] = cpu_task1_load_percent_x100;
    uart_snapshot.cpu_task_load_x100[2] = cpu_task2_load_percent_x100;
    uart_snapshot.cpu_task_load_x100[3] = cpu_task3_load_percent_x100;
    uart_snapshot.cpu_task_load_x100[4] = cpu_task4_load_percent_x100;

    {
        uint32_t task_index;
        for (task_index = 0UL; task_index < 5UL; task_index++)
        {
            Task_t *task = Scheduler_GetTask(task_index);
            if (task != 0)
            {
                uart_snapshot.task_last_exec_us[task_index] = task->last_exec_us;
                uart_snapshot.task_max_exec_us[task_index] = task->max_exec_us;
                uart_snapshot.task_deadline_miss[task_index] = task->deadline_miss_count;
            }
        }
    }

    uart_snapshot.nrf_connected = nrf24_connected;
    uart_snapshot.nrf_link = RemoteControl_IsLinkActive();
    uart_snapshot.nrf_command = RemoteControl_GetCommand();
    uart_snapshot.nrf_flags = RemoteControl_GetCommandFlags();
    uart_snapshot.nrf_packet_age_ms = remote_rx_last_packet_age_ms;
    uart_snapshot.nrf_rx_count = nrf24_rx_count;
    uart_snapshot.nrf_error_count = nrf24_error_count;
    uart_snapshot.nrf_invalid_packet_count = remote_rx_invalid_packet_count;
    uart_snapshot.nrf_irq_count = remote_rx_irq_count;
    uart_snapshot.nrf_status_reg = nrf24_status_reg;
    uart_snapshot.nrf_config_reg = nrf24_config_reg;
    uart_snapshot.nrf_channel_reg = nrf24_rf_ch_reg;
    uart_snapshot.nrf_rf_setup_reg = nrf24_rf_setup_reg;
    uart_snapshot.nrf_fifo_status_reg = nrf24_fifo_status_reg;
    uart_snapshot.stop_latched = v30_stop_latched;

    uart_snapshot.sd_initialized = sd_logger_initialized;
    uart_snapshot.sd_mount_ok = sd_logger_mount_ok;
    uart_snapshot.sd_file_open = sd_logger_file_open;
    uart_snapshot.sd_ready = SDLogger_IsReady();
    uart_snapshot.sd_logging = SDLogger_IsLogging();
    uart_snapshot.sd_last_result = sd_logger_last_result;
    uart_snapshot.sd_disk_status = sd_logger_disk_init_status;
    uart_snapshot.sd_mount_retry_count = sd_logger_mount_retry_count;
    uart_snapshot.sd_error_count = sd_logger_error_count;
    uart_snapshot.sd_write_error_count = sd_logger_write_error_count;
    uart_snapshot.sd_dropped_frame_count = sd_logger_dropped_frame_count;
    uart_snapshot.sd_ring_overrun_count = sd_logger_ring_overrun_count;
    uart_snapshot.sd_async_timeout_count = sd_logger_async_timeout_count;
    uart_snapshot.sd_ring_count = sd_logger_ring_count;
    uart_snapshot.sd_ring_high_watermark = sd_logger_ring_high_watermark;
    uart_snapshot.sd_backpressure_level = sd_logger_backpressure_level;
    uart_snapshot.sd_backpressure_entries = sd_logger_backpressure_entry_count;
    uart_snapshot.sd_backpressure_critical_entries = sd_logger_backpressure_critical_entry_count;
    uart_snapshot.sd_guard_pending = sd_logger_guard_pending;
    uart_snapshot.sd_guard_deferred = sd_logger_guard_deferred_count;
    uart_snapshot.sd_card_busy_polls = sd_logger_async_card_busy_poll_count;
    uart_snapshot.sd_write_max_us = sd_logger_max_write_duration_us;
    uart_snapshot.sd_guard_max_us = sd_logger_max_guard_write_duration_us;
    uart_snapshot.sd_fifo_order_faults = sd_logger_fifo_order_fault_count;
    uart_snapshot.sd_fifo_last_started_order = sd_logger_fifo_last_started_order;
    uart_snapshot.sd_fifo_next_ready_order = sd_logger_fifo_next_ready_order;
    uart_snapshot.sd_frame_count = sd_logger_frame_count;
    uart_snapshot.sd_source_publish_count = sd_logger_source_publish_count;
    uart_snapshot.sd_hal_init_status = sd_bsp_hal_init_status;
    uart_snapshot.sd_wide_status = sd_bsp_wide_bus_status;
    uart_snapshot.sd_host_init_attempts = sd_v47_init_attempt_count;
    uart_snapshot.sd_host_reset_count = sd_v47_host_reset_count;
    uart_snapshot.sd_hal_error_code = sd_bsp_hal_error_code;
    uart_snapshot.sd_last_attempt_error = sd_v47_last_attempt_error;
    uart_snapshot.sd_soft_recoveries = sd_p37_soft_recovery_count;
    uart_snapshot.sd_runtime_reinits = sd_p37_runtime_reinit_count;
    uart_snapshot.sd_runtime_reinit_success = sd_p37_runtime_reinit_success_count;
    uart_snapshot.sd_runtime_reinit_failures = sd_p37_runtime_reinit_failure_count;
    uart_snapshot.sd_runtime_last_error = sd_p37_last_runtime_error;
    uart_snapshot.sd_runtime_recovery_count = sd_logger_runtime_recovery_count;
    uart_snapshot.sd_runtime_recovery_success = sd_logger_runtime_recovery_success_count;
    uart_snapshot.sd_runtime_recovery_failures = sd_logger_runtime_recovery_failure_count;
    uart_snapshot.sd_runtime_recovery_flight_aborts = sd_logger_runtime_recovery_flight_abort_count;
    uart_snapshot.sd_runtime_recovery_last_us = sd_logger_runtime_recovery_last_us;
    uart_snapshot.sd_runtime_recovery_max_us = sd_logger_runtime_recovery_max_us;
    uart_snapshot.sd_recovered = sd_v47_recovered;

    uart_snapshot.system_ok = uart_snapshot.system.system_ok;
    uart_snapshot.system_fault = (uint8_t)uart_snapshot.system.fault_code;
    uart_snapshot.fast_fault = (uint32_t)SystemMonitor_GetFastFaultCode();
    uart_snapshot.baro_raw_spike_rejects =
        (baro != 0) ? baro->raw_spike_reject_count : 0UL;
    uart_snapshot.baro_raw_step_confirms =
        (baro != 0) ? baro->raw_step_confirm_count : 0UL;
    uart_snapshot.eskf_lidar_soft_reacq_active =
        (uint32_t)FullStateESKF_IsLidarInnovationReacquireActive();
    uart_snapshot.eskf_lidar_soft_reacq_count =
        FullStateESKF_GetLidarInnovationReacquireCount();
    uart_snapshot.eskf_lidar_soft_reacq_success =
        FullStateESKF_GetLidarInnovationReacquireSuccessCount();
    uart_snapshot.uart_dma_error_count = usart2_tx_dma_error_count;
    uart_snapshot.uart_busy_skip_count = uart_telemetry_busy_skip_count;
}

static uint8_t UARTTelemetry_SendText(const char *text, uint16_t length)
{
    if (USART2_TxDMA_Start((const uint8_t *)text, length) == 0U)
    {
        uart_telemetry_busy_skip_count++;
        return 0U;
    }

    uart_telemetry_send_count++;
    uart_telemetry_last_message_length = length;
    return 1U;
}

void UARTTelemetry_Init(void)
{
    uint8_t *bytes = (uint8_t *)&uart_snapshot;
    uint32_t index;
    uint32_t now_ms = HAL_GetTick();

    for (index = 0UL; index < (uint32_t)sizeof(uart_snapshot); index++)
    {
        bytes[index] = 0U;
    }

    uart_telemetry_initialized = (usart2_initialized != 0U) ? 1U : 0U;
    uart_telemetry_streaming = 0U;
    uart_telemetry_send_count = 0UL;
    uart_telemetry_busy_skip_count = 0UL;
    uart_telemetry_format_error_count = 0UL;
    uart_telemetry_frame_sequence = 0UL;
    uart_telemetry_last_service_ms = now_ms;
    uart_telemetry_last_message_length = 0U;
    uart_telemetry_last_crc16 = 0U;
    uart_telemetry_stage = 0U;
    uart_telemetry_previous_service_ms = now_ms - UART_TELEMETRY_PERIOD_MS;
}

__attribute__((optimize("O2")))
static uint8_t UARTTelemetry_AppendChar(uint16_t *position, char value)
{
    if ((*position + 1U) >= UART_TELEMETRY_BUFFER_SIZE)
    {
        return 0U;
    }

    uart_telemetry_buffer[*position] = value;
    (*position)++;
    uart_telemetry_buffer[*position] = '\0';
    return 1U;
}

__attribute__((optimize("O2")))
static uint8_t UARTTelemetry_AppendText(
    uint16_t *position,
    const char *text
)
{
    while (*text != '\0')
    {
        if (UARTTelemetry_AppendChar(position, *text) == 0U)
        {
            return 0U;
        }
        text++;
    }

    return 1U;
}

__attribute__((optimize("O2")))
static uint8_t UARTTelemetry_AppendUnsigned(
    uint16_t *position,
    uint32_t value
)
{
    char reverse_digits[10];
    uint8_t digit_count = 0U;

    if (value == 0UL)
    {
        return UARTTelemetry_AppendChar(position, '0');
    }

    while ((value != 0UL) && (digit_count < sizeof(reverse_digits)))
    {
        reverse_digits[digit_count] = (char)('0' + (value % 10UL));
        digit_count++;
        value /= 10UL;
    }

    while (digit_count > 0U)
    {
        digit_count--;
        if (UARTTelemetry_AppendChar(
                position,
                reverse_digits[digit_count]
            ) == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

__attribute__((optimize("O2")))
static uint8_t UARTTelemetry_AppendSigned(
    uint16_t *position,
    int32_t value
)
{
    uint32_t magnitude;

    if (value < 0)
    {
        if (UARTTelemetry_AppendChar(position, '-') == 0U)
        {
            return 0U;
        }
        magnitude = (uint32_t)(-(int64_t)value);
    }
    else
    {
        magnitude = (uint32_t)value;
    }

    return UARTTelemetry_AppendUnsigned(position, magnitude);
}

__attribute__((optimize("O2")))
static uint8_t UARTTelemetry_AppendHex16(
    uint16_t *position,
    uint16_t value
)
{
    static const char hex_digits[] = "0123456789ABCDEF";
    int8_t shift;

    for (shift = 12; shift >= 0; shift -= 4)
    {
        if (UARTTelemetry_AppendChar(
                position,
                hex_digits[(value >> (uint8_t)shift) & 0x0FU]
            ) == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

static const uint16_t uart_crc16_ccitt_table[256] =
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

__attribute__((optimize("O2")))
static uint16_t UARTTelemetry_Crc16Ccitt(
    const uint8_t *data,
    uint16_t length
)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;

    for (index = 0U; index < length; index++)
    {
        uint8_t table_index =
            (uint8_t)(((uint16_t)(crc >> 8)) ^ data[index]);
        crc = (uint16_t)(
            (uint16_t)(crc << 8) ^ uart_crc16_ccitt_table[table_index]
        );
    }

    return crc;
}

static uint8_t UARTTelemetry_BuildDataLine(uint16_t *message_length)
{
    uint16_t position = 0U;
    uint16_t crc;
    uint8_t auto_damping_mask = 0U;

#define FIELD_U32(value_) \
    do \
    { \
        if ((UARTTelemetry_AppendChar(&position, ',') == 0U) || \
            (UARTTelemetry_AppendUnsigned( \
                &position, (uint32_t)(value_)) == 0U)) \
        { \
            return 0U; \
        } \
    } while (0)

#define FIELD_I32(value_) \
    do \
    { \
        if ((UARTTelemetry_AppendChar(&position, ',') == 0U) || \
            (UARTTelemetry_AppendSigned( \
                &position, (int32_t)(value_)) == 0U)) \
        { \
            return 0U; \
        } \
    } while (0)

    uart_telemetry_buffer[0] = '\0';

    if (UARTTelemetry_AppendText(&position, "$TGY68") == 0U)
    {
        return 0U;
    }

    if (uart_snapshot.rcs.roll_auto_damping != 0U)
    {
        auto_damping_mask |= 0x01U;
    }
    if (uart_snapshot.rcs.pitch_auto_damping != 0U)
    {
        auto_damping_mask |= 0x02U;
    }

    FIELD_U32(uart_snapshot.sequence);
    FIELD_U32(uart_snapshot.time_ms);
    FIELD_U32(uart_snapshot.preflight.state);
    FIELD_U32(uart_snapshot.preflight.flight_active);
    FIELD_U32(uart_snapshot.preflight.raw_open);
    FIELD_U32(uart_snapshot.preflight.debounced_open);
    FIELD_U32(uart_snapshot.preflight.preflight_ready);
    FIELD_U32(uart_snapshot.preflight.fault_latched);
    FIELD_U32(uart_snapshot.preflight.flight_time_ms);
    FIELD_U32(uart_snapshot.preflight.connector_seen);
    FIELD_U32(App_IsActuatorAuthorized());

    FIELD_U32(uart_snapshot.sensor.imu_valid);
    FIELD_U32(uart_snapshot.imu_age_ms);
    FIELD_U32(uart_snapshot.imu_recovery_state);
    FIELD_U32(uart_snapshot.imu_recovery_step);
    FIELD_U32(uart_snapshot.imu_recovery_count);
    FIELD_U32(uart_snapshot.imu_recovery_attempts);
    FIELD_U32(uart_snapshot.imu_recovery_failures);
    FIELD_U32(uart_snapshot.imu_recovery_last_us);
    FIELD_U32(uart_snapshot.imu_recovery_max_us);
    FIELD_U32(uart_snapshot.imu_stale_count);
    FIELD_U32(uart_snapshot.imu_pattern_errors);
    FIELD_U32(uart_snapshot.imu_pattern_retries);
    FIELD_U32(uart_snapshot.imu_pattern_retry_success);
    FIELD_U32(uart_snapshot.imu_pattern_recovery_escalations);
    FIELD_U32(uart_snapshot.imu_fast_config_checks);
    FIELD_U32(uart_snapshot.imu_fast_config_repair_attempts);
    FIELD_U32(uart_snapshot.imu_fast_config_repair_success);
    FIELD_U32(uart_snapshot.imu_fast_config_repair_failures);
    FIELD_U32(uart_snapshot.imu_fast_config_repair_last_us);
    FIELD_U32(uart_snapshot.imu_fast_config_repair_max_us);
    FIELD_U32(uart_snapshot.imu_stale_fast_config_checks);
    FIELD_U32(uart_snapshot.imu_stale_fast_config_repair_attempts);
    FIELD_U32(uart_snapshot.imu_stale_fast_config_repair_success);
    FIELD_U32(uart_snapshot.imu_stale_fast_config_repair_failures);
    FIELD_U32(uart_snapshot.imu_stale_retry_success);
    FIELD_U32(uart_snapshot.imu_stale_recovery_escalations);
    FIELD_U32(uart_snapshot.imu_stale_fast_config_repair_last_us);
    FIELD_U32(uart_snapshot.imu_stale_fast_config_repair_max_us);
    FIELD_U32(uart_snapshot.imu_stale_warmup_events);
    FIELD_U32(uart_snapshot.imu_stale_warmup_polls);
    FIELD_U32(uart_snapshot.imu_stale_warmup_success);
    FIELD_U32(uart_snapshot.imu_stale_warmup_timeouts);
    FIELD_U32(uart_snapshot.imu_stale_warmup_first_ready_us);
    FIELD_U32(uart_snapshot.imu_stale_warmup_max_ready_us);
    FIELD_U32(uart_snapshot.imu_stale_warmup_last_status);
    FIELD_U32(uart_snapshot.imu_stale_warmup_active);
    FIELD_U32(uart_snapshot.imu_stale_diag.valid);
    FIELD_U32(uart_snapshot.imu_stale_diag.event_count);
    FIELD_U32(uart_snapshot.imu_stale_diag.timestamp_us / 1000UL);
    FIELD_U32(uart_snapshot.imu_stale_diag.stale_age_us);
    FIELD_U32(uart_snapshot.imu_stale_diag.register_snapshot_valid);
    FIELD_U32(uart_snapshot.imu_stale_diag.whoami);
    FIELD_U32(uart_snapshot.imu_stale_diag.ctrl1_xl);
    FIELD_U32(uart_snapshot.imu_stale_diag.ctrl2_g);
    FIELD_U32(uart_snapshot.imu_stale_diag.ctrl3_c);
    FIELD_U32(uart_snapshot.imu_stale_diag.ctrl4_c);
    FIELD_U32(uart_snapshot.imu_invalid_samples);
    FIELD_U32(uart_snapshot.imu_redundant_rejects);
    FIELD_U32(uart_snapshot.imu_pattern_diag.valid);
    FIELD_U32(uart_snapshot.imu_pattern_diag.event_count);
    FIELD_U32(uart_snapshot.imu_pattern_diag.timestamp_us / 1000UL);
    FIELD_U32(uart_snapshot.imu_pattern_diag.repeated_magnitude_raw);
    FIELD_U32(uart_snapshot.imu_pattern_diag.sign_mask);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[0].gyro_x_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[0].gyro_y_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[0].gyro_z_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[0].accel_x_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[0].accel_y_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[0].accel_z_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[1].gyro_x_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[1].gyro_y_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[1].gyro_z_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[1].accel_x_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[1].accel_y_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[1].accel_z_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[2].gyro_x_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[2].gyro_y_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[2].gyro_z_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[2].accel_x_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[2].accel_y_raw);
    FIELD_I32(uart_snapshot.imu_pattern_diag.burst[2].accel_z_raw);
    FIELD_U32(uart_snapshot.imu_pattern_diag.register_snapshot_valid);
    FIELD_U32(uart_snapshot.imu_pattern_diag.whoami);
    FIELD_U32(uart_snapshot.imu_pattern_diag.ctrl1_xl);
    FIELD_U32(uart_snapshot.imu_pattern_diag.ctrl2_g);
    FIELD_U32(uart_snapshot.imu_pattern_diag.ctrl3_c);
    FIELD_U32(uart_snapshot.imu_pattern_diag.ctrl4_c);
    FIELD_U32(uart_snapshot.sensor.baro_valid);
    FIELD_U32(uart_snapshot.lidar.distance_valid);
    FIELD_U32(uart_snapshot.eskf.initialized);
    FIELD_U32(uart_snapshot.eskf.healthy);
    FIELD_U32(uart_snapshot.eskf.output_inhibited);
    FIELD_U32(uart_snapshot.eskf.vertical_reacquire_active);
    FIELD_U32(uart_snapshot.eskf.vertical_divergence_reason);
    FIELD_U32(uart_snapshot.eskf.vertical_divergence_count);
    FIELD_U32(uart_snapshot.eskf.vertical_reacquire_count);
    FIELD_U32(uart_snapshot.eskf.vertical_divergence_candidate_count);
    FIELD_U32(uart_snapshot.eskf.vertical_reacquire_stable_count);
    FIELD_I32(UARTTelemetry_ScaleFloat(uart_snapshot.eskf.vertical_sensor_consistency_m, 1000.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(uart_snapshot.eskf.vertical_reacquire_target_m, 1000.0f));
    FIELD_U32(uart_snapshot.eskf_public_age_ms);
    FIELD_U32(uart_snapshot.eskf.public_output_count);
    FIELD_U32(uart_snapshot.eskf.reset_reason);
    FIELD_U32(uart_snapshot.eskf.reset_count);
    FIELD_U32(uart_snapshot.eskf.numerical_error_count);
    FIELD_U32(uart_snapshot.eskf.public_output_reject_count);
    FIELD_U32(uart_snapshot.eskf.public_z_jump_reject_count);
    FIELD_U32(uart_snapshot.eskf.public_vz_jump_reject_count);
    FIELD_U32(uart_snapshot.eskf.covariance_integrity_ok);
    FIELD_U32(uart_snapshot.eskf.covariance_integrity_check_count);
    FIELD_U32(uart_snapshot.eskf.covariance_fault_count);
    FIELD_U32(uart_snapshot.eskf.covariance_reinit_count);
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.eskf.covariance_diag_min, 1000000.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.eskf.covariance_diag_max, 1000.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.eskf.covariance_symmetry_error_max, 1000000.0f));
    FIELD_U32(uart_snapshot.eskf.covariance_fault_stage);
    FIELD_U32(uart_snapshot.eskf.covariance_fault_state_index);
    FIELD_U32(uart_snapshot.eskf.covariance_fault_other_index);
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.eskf.covariance_fault_raw_value, 1000000000.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.eskf.covariance_fault_aux_value, 1000000000.0f));
    FIELD_U32(uart_snapshot.eskf.covariance_fault_timestamp_us / 1000UL);
    FIELD_U32(uart_snapshot.eskf.covariance_fault_used_last_good);
    FIELD_U32(uart_snapshot.eskf.covariance_state_preserving_recovery_count);
    FIELD_U32(uart_snapshot.eskf.covariance_roundoff_clamp_count);
    FIELD_U32(uart_snapshot.eskf.covariance_roundoff_last_stage);
    FIELD_U32(uart_snapshot.eskf.covariance_roundoff_last_state_index);
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.eskf.covariance_roundoff_last_raw_value, 1000000000.0f));
    FIELD_U32(uart_snapshot.eskf.covariance_roundoff_last_timestamp_us / 1000UL);
    FIELD_U32(uart_snapshot.eskf.baro_fresh);
    FIELD_U32(uart_snapshot.eskf.lidar_fresh);
    FIELD_U32(uart_snapshot.main_control.vertical_sensor_source_mask);
    FIELD_U32(uart_snapshot.main_control.vertical_sensor_degraded);

    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.eskf.position_z_m, 1000.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.eskf.velocity_z_mps, 1000.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.sensor.baro_filtered_altitude_m, 1000.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.lidar.filtered_distance_m, 1000.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.sensor.baro_filtered_pressure_pa, 1.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.baro.ground_pressure_pa, 1.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.baro.temperature_c, 100.0f));
    FIELD_U32(uart_snapshot.baro.ground_reference_tracking_allowed);
    FIELD_U32(uart_snapshot.baro.ground_reference_tracking_active);
    FIELD_U32(uart_snapshot.baro.ground_reference_frozen);
    FIELD_U32(uart_snapshot.baro.ground_reference_update_count);
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.baro.ground_reference_error_pa, 1000.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.baro.ground_reference_last_step_pa, 1000.0f));
    FIELD_U32(uart_snapshot.baro.ground_reference_last_update_us / 1000UL);
    FIELD_U32(uart_snapshot.baro.ground_reference_freeze_count);
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.eskf.baro_innovation_m, 1000.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.eskf.lidar_innovation_m, 1000.0f));

    FIELD_U32(uart_snapshot.eskf.baro_update_count);
    FIELD_U32(uart_snapshot.eskf.lidar_update_count);
    FIELD_U32(uart_snapshot.eskf.baro_reject_count);
    FIELD_U32(uart_snapshot.eskf.lidar_reject_count);
    FIELD_U32(uart_snapshot.lidar.state);
    FIELD_U32(uart_snapshot.lidar_age_ms);
    FIELD_U32(uart_snapshot.lidar.read_count);
    FIELD_U32(uart_snapshot.lidar.error_count);
    FIELD_U32(uart_snapshot.lidar.timeout_count);
    FIELD_U32(uart_snapshot.lidar.wait_busy_timeout_count);
    FIELD_U32(uart_snapshot.lidar.bus_recovery_count);
    FIELD_U32(uart_snapshot.lidar_recovery_active);
    FIELD_U32(uart_snapshot.lidar_recovery_step);
    FIELD_U32(uart_snapshot.lidar_recovery_attempts);
    FIELD_U32(uart_snapshot.lidar_recovery_success);
    FIELD_U32(uart_snapshot.lidar_recovery_failures);
    FIELD_U32(uart_snapshot.lidar_recovery_last_us);
    FIELD_U32(uart_snapshot.lidar_recovery_max_us);
    FIELD_U32(uart_snapshot.lidar_recovery_step_max_us);

    FIELD_I32(UARTTelemetry_ScaleFloat(uart_snapshot.rcs.roll_deg, 100.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(uart_snapshot.rcs.pitch_deg, 100.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.rcs.roll_rate_dps, 1000.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.rcs.pitch_rate_dps, 1000.0f));
    FIELD_U32(uart_snapshot.rcs.state);
    FIELD_U32(uart_snapshot.rcs.fault);
    FIELD_U32(uart_snapshot.rcs.valve_demand_mask);
    FIELD_U32(uart_snapshot.rcs.valve_applied_mask);
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.rcs.roll_pd_time_signed_ms, 1000.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.rcs.pitch_pd_time_signed_ms, 1000.0f));
    FIELD_U32(uart_snapshot.rcs.roll_pulse_remaining_ms);
    FIELD_U32(uart_snapshot.rcs.pitch_pulse_remaining_ms);
    FIELD_U32(uart_snapshot.rcs.roll_cooldown_remaining_ms);
    FIELD_U32(uart_snapshot.rcs.pitch_cooldown_remaining_ms);
    FIELD_U32(auto_damping_mask);
    FIELD_U32(uart_snapshot.rcs.dry_run);
    FIELD_U32(uart_snapshot.rcs.event_count);
    FIELD_U32(uart_snapshot.solenoid.flight_authorized);
    FIELD_U32(uart_snapshot.solenoid.safety_inhibited);
    FIELD_U32(uart_snapshot.solenoid.safety_inhibit_count);

    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.main_control.vertical_200hz_valve_cmd, 10000.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.main_control.vertical_target_force_n, 100.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.main_control.vertical_estimated_thrust_n, 100.0f));
    FIELD_U32(uart_snapshot.main_control.suicide_burn_active);
    FIELD_U32(uart_snapshot.main_control.main_output_valid);
    FIELD_U32(APP_GENERATED_FC_PHYSICAL_OUTPUT_ENABLED);

    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.needle.requested_cmd, 10000.0f));
    FIELD_I32(UARTTelemetry_ScaleFloat(
        uart_snapshot.needle.limited_cmd, 10000.0f));
    FIELD_U32(uart_snapshot.needle.raw_adc);
    FIELD_U32(uart_snapshot.needle.target_adc);
    FIELD_U32(uart_snapshot.needle.enabled);
    FIELD_U32(uart_snapshot.needle.position_locked);
    FIELD_U32(uart_snapshot.needle.fault);
    FIELD_U32(uart_snapshot.needle.zero_adc);
    FIELD_I32(uart_snapshot.needle.error_adc);
    FIELD_U32(uart_snapshot.needle.rpwm);
    FIELD_U32(uart_snapshot.needle.lpwm);
    FIELD_U32(uart_snapshot.needle.zero_valid);
    FIELD_U32(uart_snapshot.needle.homing_active);
    FIELD_U32(uart_snapshot.needle.homing_complete);
    FIELD_U32(uart_snapshot.needle.stall_ms);

    FIELD_U32(uart_snapshot.nrf_connected);
    FIELD_U32(uart_snapshot.nrf_link);
    FIELD_U32(uart_snapshot.nrf_command);
    FIELD_U32(uart_snapshot.nrf_flags);
    FIELD_U32(uart_snapshot.nrf_packet_age_ms);
    FIELD_U32(uart_snapshot.nrf_rx_count);
    FIELD_U32(uart_snapshot.nrf_error_count);
    FIELD_U32(uart_snapshot.nrf_invalid_packet_count);
    FIELD_U32(uart_snapshot.nrf_irq_count);
    FIELD_U32(uart_snapshot.nrf_status_reg);
    FIELD_U32(uart_snapshot.nrf_config_reg);
    FIELD_U32(uart_snapshot.nrf_channel_reg);
    FIELD_U32(uart_snapshot.nrf_rf_setup_reg);
    FIELD_U32(uart_snapshot.nrf_fifo_status_reg);
    FIELD_U32(uart_snapshot.stop_latched);

    FIELD_U32(uart_snapshot.sd_initialized);
    FIELD_U32(uart_snapshot.sd_mount_ok);
    FIELD_U32(uart_snapshot.sd_file_open);
    FIELD_U32(uart_snapshot.sd_ready);
    FIELD_U32(uart_snapshot.sd_logging);
    FIELD_U32(uart_snapshot.sd_last_result);
    FIELD_U32(uart_snapshot.sd_disk_status);
    FIELD_U32(uart_snapshot.sd_mount_retry_count);
    FIELD_U32(uart_snapshot.sd_error_count);
    FIELD_U32(uart_snapshot.sd_write_error_count);
    FIELD_U32(uart_snapshot.sd_dropped_frame_count);
    FIELD_U32(uart_snapshot.sd_ring_overrun_count);
    FIELD_U32(uart_snapshot.sd_async_timeout_count);
    FIELD_U32(uart_snapshot.sd_ring_count);
    FIELD_U32(uart_snapshot.sd_ring_high_watermark);
    FIELD_U32(uart_snapshot.sd_backpressure_level);
    FIELD_U32(uart_snapshot.sd_backpressure_entries);
    FIELD_U32(uart_snapshot.sd_backpressure_critical_entries);
    FIELD_U32(uart_snapshot.sd_guard_pending);
    FIELD_U32(uart_snapshot.sd_guard_deferred);
    FIELD_U32(uart_snapshot.sd_card_busy_polls);
    FIELD_U32(uart_snapshot.sd_write_max_us);
    FIELD_U32(uart_snapshot.sd_guard_max_us);
    FIELD_U32(uart_snapshot.sd_fifo_order_faults);
    FIELD_U32(uart_snapshot.sd_fifo_last_started_order);
    FIELD_U32(uart_snapshot.sd_fifo_next_ready_order);
    FIELD_U32(uart_snapshot.sd_frame_count);
    FIELD_U32(uart_snapshot.sd_source_publish_count);
    FIELD_U32(uart_snapshot.sd_hal_init_status);
    FIELD_U32(uart_snapshot.sd_wide_status);
    FIELD_U32(uart_snapshot.sd_host_init_attempts);
    FIELD_U32(uart_snapshot.sd_host_reset_count);
    FIELD_U32(uart_snapshot.sd_hal_error_code);
    FIELD_U32(uart_snapshot.sd_last_attempt_error);
    FIELD_U32(uart_snapshot.sd_recovered);
    FIELD_U32(uart_snapshot.sd_soft_recoveries);
    FIELD_U32(uart_snapshot.sd_runtime_reinits);
    FIELD_U32(uart_snapshot.sd_runtime_reinit_success);
    FIELD_U32(uart_snapshot.sd_runtime_reinit_failures);
    FIELD_U32(uart_snapshot.sd_runtime_last_error);
    FIELD_U32(uart_snapshot.sd_runtime_recovery_count);
    FIELD_U32(uart_snapshot.sd_runtime_recovery_success);
    FIELD_U32(uart_snapshot.sd_runtime_recovery_failures);
    FIELD_U32(uart_snapshot.sd_runtime_recovery_flight_aborts);
    FIELD_U32(uart_snapshot.sd_runtime_recovery_last_us);
    FIELD_U32(uart_snapshot.sd_runtime_recovery_max_us);

    FIELD_U32(uart_snapshot.cpu_load_x100);
    FIELD_U32(uart_snapshot.cpu_external_load_x100);
    FIELD_U32(uart_snapshot.sd_update_last_us);
    FIELD_U32(uart_snapshot.sd_update_max_us);
    FIELD_U32(uart_snapshot.sd_drain_last_us);
    FIELD_U32(uart_snapshot.sd_drain_max_us);
    FIELD_U32(uart_snapshot.sd_drain_yields);
    FIELD_U32(uart_snapshot.covariance_last_us);
    FIELD_U32(uart_snapshot.covariance_max_us);
    FIELD_U32(uart_snapshot.covariance_service_count);
    FIELD_U32(uart_snapshot.gravity_joseph_update_count);
    FIELD_U32(uart_snapshot.gravity_joseph_fault_count);
    FIELD_U32(uart_snapshot.eskf.zupt_joseph_update_count);
    FIELD_U32(uart_snapshot.eskf.zupt_joseph_fault_count);
    FIELD_U32(uart_snapshot.scheduler_slow_defers);
    FIELD_U32(uart_snapshot.bg_fresh_last_us);
    FIELD_U32(uart_snapshot.bg_remote_last_us);
    FIELD_U32(uart_snapshot.bg_control_last_us);
    FIELD_U32(uart_snapshot.bg_uart_last_us);
    FIELD_U32(uart_snapshot.bg_uart_max_us);
    FIELD_U32(uart_snapshot.sd_defer_count);
    FIELD_U32(uart_snapshot.control_defer_count);
    FIELD_U32(uart_snapshot.uart_defer_count);
    FIELD_U32(uart_snapshot.cpu_task_load_x100[0]);
    FIELD_U32(uart_snapshot.cpu_task_load_x100[1]);
    FIELD_U32(uart_snapshot.cpu_task_load_x100[2]);
    FIELD_U32(uart_snapshot.cpu_task_load_x100[3]);
    FIELD_U32(uart_snapshot.cpu_task_load_x100[4]);
    FIELD_U32(uart_snapshot.task_last_exec_us[0]);
    FIELD_U32(uart_snapshot.task_last_exec_us[1]);
    FIELD_U32(uart_snapshot.task_last_exec_us[2]);
    FIELD_U32(uart_snapshot.task_last_exec_us[3]);
    FIELD_U32(uart_snapshot.task_last_exec_us[4]);
    FIELD_U32(uart_snapshot.task_max_exec_us[0]);
    FIELD_U32(uart_snapshot.task_max_exec_us[1]);
    FIELD_U32(uart_snapshot.task_max_exec_us[2]);
    FIELD_U32(uart_snapshot.task_max_exec_us[3]);
    FIELD_U32(uart_snapshot.task_max_exec_us[4]);
    FIELD_U32(uart_snapshot.task_deadline_miss[0]);
    FIELD_U32(uart_snapshot.task_deadline_miss[1]);
    FIELD_U32(uart_snapshot.task_deadline_miss[2]);
    FIELD_U32(uart_snapshot.task_deadline_miss[3]);
    FIELD_U32(uart_snapshot.task_deadline_miss[4]);

    FIELD_U32(uart_snapshot.fast_fault);
    FIELD_U32(uart_snapshot.baro_raw_spike_rejects);
    FIELD_U32(uart_snapshot.baro_raw_step_confirms);
    FIELD_U32(uart_snapshot.eskf_lidar_soft_reacq_active);
    FIELD_U32(uart_snapshot.eskf_lidar_soft_reacq_count);
    FIELD_U32(uart_snapshot.eskf_lidar_soft_reacq_success);

    FIELD_U32(uart_snapshot.system_ok);
    FIELD_U32(uart_snapshot.system_fault);
    FIELD_U32(uart_snapshot.system.imu_fresh);
    FIELD_U32(uart_snapshot.system.lidar_fresh);
    FIELD_U32(uart_snapshot.system.eskf_public_fresh);
    FIELD_U32(uart_snapshot.scheduler_realign_count);
    FIELD_U32(uart_snapshot.uart_dma_error_count);
    FIELD_U32(uart_snapshot.uart_busy_skip_count);

    /* P68 bench-only NRF TDD diagnostics. These are read-only counters from
     * NRFTelemetry; no control, sensor, SD, ESKF or RF state-machine behavior
     * is modified by exposing them on the diagnostic UART. */
    FIELD_U32(nrf_tlm_schedule_count);
    FIELD_U32(nrf_tlm_tx_start_count);
    FIELD_U32(nrf_tlm_tx_success_count);
    FIELD_U32(nrf_tlm_tx_fail_count);
    FIELD_U32(nrf_tlm_pending_replace_count);
    FIELD_U32(nrf_tlm_last_tx_duration_us);
    FIELD_U32(nrf_tlm_max_tx_duration_us);

    /* P83 robust-feedback telemetry retained as the P87 comparison path. */
    FIELD_U32(p83_diag_flags);
    FIELD_U32(p83_samples_total);
    FIELD_U32(p83_adc1_raw);
    FIELD_U32(p83_adc2_raw);
    FIELD_U32(p83_pair_diff);
    FIELD_U32(p83_pair_candidate);
    FIELD_U32(p83_median7);
    FIELD_U32(p83_filtered_adc);
    FIELD_U32(p83_feedback_valid);
    FIELD_U32(p83_confidence_pct);
    FIELD_U32(p83_pair_reject_count);
    FIELD_U32(p83_rate_reject_count);
    FIELD_U32(p83_quarantine_count);
    FIELD_U32(p83_reacquire_count);
    FIELD_U32(p83_mode);
    FIELD_U32(p83_acq_progress_pct);
    FIELD_U32(p83_adc_timeout_count);
    FIELD_U32(p83_win_raw_pp);
    FIELD_U32(p83_win_filtered_pp);
    FIELD_U32(p83_vref_win_pp_raw12);

    /* P87 cumulative ADC sweep diagnostics. */
    FIELD_U32(p87_sweep_samples);
    FIELD_U32(p87_adc1_min);
    FIELD_U32(p87_adc1_max);
    FIELD_U32(p87_adc2_min);
    FIELD_U32(p87_adc2_max);
    FIELD_U32(p87_candidate_min);
    FIELD_U32(p87_candidate_max);
    FIELD_U32(p87_adc1_max_step);
    FIELD_U32(p87_adc1_step_from);
    FIELD_U32(p87_adc1_step_to);
    FIELD_U32(p87_adc2_max_step);
    FIELD_U32(p87_adc2_step_from);
    FIELD_U32(p87_adc2_step_to);
    FIELD_U32(p87_candidate_max_step);
    FIELD_U32(p87_candidate_step_from);
    FIELD_U32(p87_candidate_step_to);
    FIELD_U32(p87_filtered_max_step);
    FIELD_U32(p87_filtered_step_from);
    FIELD_U32(p87_filtered_step_to);
    FIELD_U32(p87_raw_gap_event_count);
    FIELD_U32(p87_filtered_gap_event_count);
    FIELD_U32(p87_pair_diff_max);

    /* P99 exact actuator-sequence diagnostics. */
    FIELD_U32(p88_abort_reason);
    FIELD_U32(p97_phase);
    FIELD_U32(p97_open_chunk_count);
    FIELD_U32(p97_open_drive_ms);
    FIELD_U32(p97_return_chunk_count);
    FIELD_U32(p97_return_drive_ms);

    /* P105 three-cycle repeatability summary. */
    FIELD_U32(p105_cycle_index);
    FIELD_U32(p105_completed_cycles);
    FIELD_U32(p105_pass_mask);
    FIELD_U32(p105_last_cycle_abort);
    FIELD_U32(p105_last_start_adc);
    FIELD_U32(p105_last_target_adc);
    FIELD_U32(p105_last_open_end_adc);
    FIELD_I32(p105_last_open_error_adc);
    FIELD_U32(p105_last_home_adc);
    FIELD_U32(p105_last_open_chunks);
    FIELD_U32(p105_last_open_drive_ms);
    FIELD_U32(p105_last_return_chunks);
    FIELD_U32(p105_last_return_drive_ms);


    /* P106 loaded-breakaway characterization. */
    FIELD_U32(p106_state);
    FIELD_U32(p106_stage);
    FIELD_U32(p106_test_pwm);
    FIELD_U32(p106_start_adc);
    FIELD_U32(p106_stage_start_adc);
    FIELD_U32(p106_end_adc);
    FIELD_I32(p106_delta_adc);
    FIELD_U32(p106_breakaway_found);
    FIELD_U32(p106_breakaway_pwm);
    FIELD_U32(p106_result);


    /* P107 loaded boost+sustain 50-ADC test. */
    FIELD_U32(p107_state);
    FIELD_U32(p107_result);
    FIELD_U32(p107_start_adc);
    FIELD_U32(p107_target_adc);
    FIELD_U32(p107_end_adc);
    FIELD_I32(p107_error_adc);
    FIELD_U32(p107_boost_pwm);
    FIELD_U32(p107_sustain_pwm);
    FIELD_U32(p107_boost_ms);
    FIELD_U32(p107_sustain_ms);
    FIELD_U32(p107_max_open_drop_adc);
    FIELD_U32(p107_brake_cause);


    /* P109 pre-brake/coast characterization. */
    FIELD_U32(p109_state);
    FIELD_U32(p109_result);
    FIELD_U32(p109_start_adc);
    FIELD_U32(p109_target_adc);
    FIELD_U32(p109_prebrake_adc);
    FIELD_U32(p109_brake_entry_adc);
    FIELD_U32(p109_adc_100ms);
    FIELD_U32(p109_adc_250ms);
    FIELD_U32(p109_adc_500ms);
    FIELD_I32(p109_coast_100_adc);
    FIELD_I32(p109_coast_250_adc);
    FIELD_I32(p109_coast_500_adc);
    FIELD_I32(p109_final_error_adc);
    FIELD_U32(p109_boost_ms);
    FIELD_U32(p109_sustain_ms);
    FIELD_U32(p109_brake_cause);

    /* P110 adaptive needle position controller. */
    FIELD_U32(p110_state);
    FIELD_U32(p110_result);
    FIELD_U32(p110_direction);
    FIELD_U32(p110_start_adc);
    FIELD_U32(p110_target_adc);
    FIELD_U32(p110_current_adc);
    FIELD_I32(p110_error_adc);
    FIELD_U32(p110_active_pwm);
    FIELD_U32(p110_learned_breakaway_pwm);
    FIELD_U32(p110_sustain_pwm);
    FIELD_U32(p110_speed_adc_s);
    FIELD_U32(p110_stop_distance_adc);
    FIELD_U32(p110_brake_entry_adc);
    FIELD_U32(p110_coast_max_adc);
    FIELD_U32(p110_final_adc);
    FIELD_I32(p110_final_error_adc);
    FIELD_U32(p110_search_ms);
    FIELD_U32(p110_drive_ms);
    FIELD_U32(p110_brake_ms);
    FIELD_U32(p110_total_powered_ms);
    FIELD_U32(p110_correction_count);
    FIELD_U32(p110_breakaway_found);
    FIELD_U32(p110_abort_reason);

    /* P111 five-cycle adaptive-learning supervisor. */
    FIELD_U32(p111_state);
    FIELD_U32(p111_result);
    FIELD_U32(p111_cycle);
    FIELD_U32(p111_completed_cycles);
    FIELD_U32(p111_pass_mask);
    FIELD_U32(p111_phase);
    FIELD_U32(p111_moves_completed);
    FIELD_U32(p111_baseline_adc);
    FIELD_U32(p111_open_target_adc);
    FIELD_U32(p111_learned_open_pwm);
    FIELD_U32(p111_learned_close_pwm);
    FIELD_U32(p111_learned_open_coast);
    FIELD_U32(p111_learned_close_coast);
    FIELD_I32(p111_last_open_error_adc);
    FIELD_I32(p111_last_close_error_adc);
    FIELD_U32(p111_last_open_powered_ms);
    FIELD_U32(p111_last_close_powered_ms);
    FIELD_U32(p111_last_open_breakaway_pwm);
    FIELD_U32(p111_last_close_breakaway_pwm);
    FIELD_U32(p111_last_open_stop_adc);
    FIELD_U32(p111_last_close_stop_adc);
    FIELD_U32(p111_last_open_coast_adc);
    FIELD_U32(p111_last_close_coast_adc);
    FIELD_U32(p111_abort_reason);
    /* P112 deterministic timing / suppression telemetry. */
    FIELD_U32(p112_isr_control_ticks);
    FIELD_U32(p112_isr_adc_samples);
    FIELD_U32(p112_hard_off_count);
    FIELD_U32(p112_uart_suppressed_count);
    FIELD_U32(p112_sd_suppressed_count);
    FIELD_U32(p112_isr_last_us);
    FIELD_U32(p112_isr_max_us);
    FIELD_U32(p112_timing_critical);
    FIELD_U32(p112_decel_active);
    FIELD_U32(p112_decel_pwm);
    FIELD_U32(p112_initial_coast_adc);

    /* P112R11R1 commissioning-only read-only diagnostics. Appended so all
     * historical field indices remain unchanged. */
    FIELD_U32(NeedleValveAutonomousControl_GetCommissioningState());
    FIELD_U32(NeedleValveAutonomousControl_GetCommissioningAbortReason());
    FIELD_U32(NeedleValveAutonomousControl_GetCommissioningAbortCount());
    FIELD_U32(NeedleValveAutonomousControl_GetCommissioningButtonDebounced());
    FIELD_U32(NeedleValveAutonomousControl_GetCommissioningOpenTargetAdc());

    crc = UARTTelemetry_Crc16Ccitt(
        (const uint8_t *)uart_telemetry_buffer,
        position
    );
    uart_telemetry_last_crc16 = crc;

    if ((UARTTelemetry_AppendChar(&position, '*') == 0U) ||
        (UARTTelemetry_AppendHex16(&position, crc) == 0U) ||
        (UARTTelemetry_AppendChar(&position, '\r') == 0U) ||
        (UARTTelemetry_AppendChar(&position, '\n') == 0U))
    {
        return 0U;
    }

#undef FIELD_U32
#undef FIELD_I32

    *message_length = position;
    return 1U;
}

static uint32_t UARTTelemetry_ProfileCyclesToUs(uint32_t cycles)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000UL;

    if (cycles_per_us == 0UL)
    {
        return 0UL;
    }

    return (cycles + cycles_per_us - 1UL) / cycles_per_us;
}

__attribute__((optimize("O2")))
static uint8_t UARTTelemetry_BuildTimingLine(uint16_t *message_length)
{
    uint16_t position = 0U;
    uint16_t crc;
    NeedleValveAutonomousStatus_t needle_auto;
    NeedleValveStatus_t needle_telem;

#define T70_U32(value_) \
    do \
    { \
        if ((UARTTelemetry_AppendChar(&position, ',') == 0U) || \
            (UARTTelemetry_AppendUnsigned(&position, (uint32_t)(value_)) == 0U)) \
        { \
            return 0U; \
        } \
    } while (0)

#define T70_I32(value_) \
    do \
    { \
        if ((UARTTelemetry_AppendChar(&position, ',') == 0U) || \
            (UARTTelemetry_AppendSigned(&position, (int32_t)(value_)) == 0U)) \
        { \
            return 0U; \
        } \
    } while (0)

    needle_auto = NeedleValveAutonomousControl_GetStatus();
    needle_telem = NeedleValveAutonomousControl_GetTelemetryStatus();

    uart_telemetry_buffer[0] = '\0';
    if (UARTTelemetry_AppendText(&position, "$TGY73") == 0U)
    {
        return 0U;
    }

    T70_U32(uart_snapshot.sequence);
    T70_U32(uart_snapshot.time_ms);
    T70_U32(uart_snapshot.preflight.preflight_ready);
    T70_U32(uart_snapshot.preflight.flight_active);
    T70_U32(uart_snapshot.preflight.debounced_open);
    T70_U32(App_IsActuatorAuthorized());

    T70_U32(uart_snapshot.sensor.imu_valid);
    T70_U32(uart_snapshot.imu_age_ms);
    T70_U32(uart_snapshot.sensor.baro_valid);
    T70_U32(uart_snapshot.eskf.baro_fresh);
    T70_U32((uint32_t)uart_snapshot.baro.pressure_pa);
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.baro.temperature_c, 100.0f));

    T70_U32(uart_snapshot.lidar.distance_valid);
    T70_U32(uart_snapshot.system.lidar_fresh);
    T70_U32(uart_snapshot.lidar_age_ms);
    T70_U32(uart_snapshot.lidar.state);
    T70_U32(uart_snapshot.lidar.error_count);
    T70_U32(uart_snapshot.lidar.timeout_count);
    T70_U32(uart_snapshot.lidar.bus_recovery_count);
    T70_U32(uart_snapshot.lidar_recovery_active);
    T70_U32(uart_snapshot.lidar_recovery_step);
    T70_U32(uart_snapshot.lidar_recovery_attempts);
    T70_U32(uart_snapshot.lidar_recovery_success);
    T70_U32(uart_snapshot.lidar_recovery_failures);
    T70_U32(uart_snapshot.lidar_recovery_last_us);
    T70_U32(uart_snapshot.lidar_recovery_max_us);

    T70_U32(uart_snapshot.eskf.healthy);
    T70_U32(uart_snapshot.eskf_public_age_ms);
    T70_U32(uart_snapshot.eskf.covariance_integrity_ok);
    T70_U32(uart_snapshot.eskf.covariance_integrity_check_count);

    /* R8R8: isolate the estimator itself from the rest of task-4.  The
     * correction/predict fields are native microseconds; section profilers
     * use DWT cycles converted with the live HCLK value. */
    T70_U32(uart_snapshot.eskf.last_correction_exec_us);
    T70_U32(uart_snapshot.eskf.max_correction_exec_us);
    T70_U32(uart_snapshot.eskf.last_predict_exec_us);
    T70_U32(uart_snapshot.eskf.max_predict_exec_us);
    T70_U32(UARTTelemetry_ProfileCyclesToUs(full_eskf_last_gravity_cycles));
    T70_U32(UARTTelemetry_ProfileCyclesToUs(full_eskf_max_gravity_cycles));
    T70_U32(UARTTelemetry_ProfileCyclesToUs(full_eskf_last_stationary_cycles));
    T70_U32(UARTTelemetry_ProfileCyclesToUs(full_eskf_max_stationary_cycles));
    T70_U32(UARTTelemetry_ProfileCyclesToUs(full_eskf_last_baro_cycles));
    T70_U32(UARTTelemetry_ProfileCyclesToUs(full_eskf_max_baro_cycles));
    T70_U32(UARTTelemetry_ProfileCyclesToUs(full_eskf_last_lidar_cycles));
    T70_U32(UARTTelemetry_ProfileCyclesToUs(full_eskf_max_lidar_cycles));
    T70_U32(UARTTelemetry_ProfileCyclesToUs(full_eskf_last_public_output_cycles));
    T70_U32(UARTTelemetry_ProfileCyclesToUs(full_eskf_max_public_output_cycles));
    T70_U32(uart_snapshot.eskf.public_output_count);
    T70_U32(uart_snapshot.eskf.gravity_update_count);
    T70_U32(uart_snapshot.eskf.gravity_reject_count);
    T70_U32(uart_snapshot.eskf.gravity_joseph_update_count);
    T70_U32(uart_snapshot.eskf.gravity_joseph_fault_count);

    T70_U32(uart_snapshot.cpu_load_x100);
    T70_U32(uart_snapshot.cpu_external_load_x100);
    T70_U32(uart_snapshot.cpu_task_load_x100[0]);
    T70_U32(uart_snapshot.cpu_task_load_x100[1]);
    T70_U32(uart_snapshot.cpu_task_load_x100[2]);
    T70_U32(uart_snapshot.cpu_task_load_x100[4]);
    T70_U32(uart_snapshot.covariance_last_us);
    T70_U32(uart_snapshot.covariance_max_us);

    T70_U32(uart_snapshot.task_last_exec_us[0]);
    T70_U32(uart_snapshot.task_last_exec_us[1]);
    T70_U32(uart_snapshot.task_last_exec_us[2]);
    T70_U32(uart_snapshot.task_last_exec_us[4]);
    T70_U32(uart_snapshot.task_max_exec_us[0]);
    T70_U32(uart_snapshot.task_max_exec_us[1]);
    T70_U32(uart_snapshot.task_max_exec_us[2]);
    T70_U32(uart_snapshot.task_max_exec_us[4]);
    T70_U32(uart_snapshot.task_deadline_miss[0]);
    T70_U32(uart_snapshot.task_deadline_miss[1]);
    T70_U32(uart_snapshot.task_deadline_miss[2]);
    T70_U32(uart_snapshot.task_deadline_miss[4]);

    T70_U32(uart_snapshot.system_ok);
    T70_U32(uart_snapshot.system_fault);
    T70_U32(uart_snapshot.system.imu_fresh);
    T70_U32(uart_snapshot.system.lidar_fresh);
    T70_U32(uart_snapshot.system.eskf_public_fresh);
    T70_U32(uart_snapshot.scheduler_realign_count);

    /* R8R10: explain why a healthy <1 ms estimator can still miss a 5 ms
     * release.  Task indices are fixed: 0=IMU, 1=BARO, 4=ESKF. */
    T70_U32(scheduler_task_last_lateness_us[0]);
    T70_U32(scheduler_task_max_lateness_us[0]);
    T70_U32(scheduler_task_last_lateness_us[1]);
    T70_U32(scheduler_task_max_lateness_us[1]);
    T70_U32(scheduler_task_last_lateness_us[4]);
    T70_U32(scheduler_task_max_lateness_us[4]);
    T70_U32(scheduler_task_defer_count[1]);
    T70_U32(scheduler_task_defer_streak[1]);
    T70_U32(scheduler_task_max_defer_streak[1]);
    T70_U32(scheduler_task_last_defer_slack_us[1]);
    T70_U32(scheduler_task_defer_count[4]);
    T70_U32(scheduler_task_defer_streak[4]);
    T70_U32(scheduler_task_max_defer_streak[4]);
    T70_U32(scheduler_task_last_defer_slack_us[4]);

    T70_U32(p34_bg_fresh_last_us);
    T70_U32(p34_bg_fresh_max_us);
    T70_U32(p34_bg_remote_last_us);
    T70_U32(p34_bg_remote_max_us);
    T70_U32(p34_bg_control_last_us);
    T70_U32(p34_bg_control_max_us);
    T70_U32(v87_sd_update_last_us);
    T70_U32(v87_sd_update_max_us);

    T70_U32(uart_snapshot.bg_uart_last_us);
    T70_U32(uart_snapshot.bg_uart_max_us);
    T70_U32(uart_snapshot.uart_defer_count);
    T70_U32(uart_snapshot.sd_ready);

    /* R8R13: P73 fast/redundant nRF coexistence qualification. Read-only diagnostics only;
     * the proven P64/P69 explicit-TDD state machine and remote parser are
     * unchanged. Task index 3 is the 200 Hz NRF snapshot task. */
    T70_U32(uart_snapshot.nrf_connected);
    T70_U32(uart_snapshot.nrf_link);
    T70_U32(uart_snapshot.nrf_flags);
    T70_U32(uart_snapshot.nrf_packet_age_ms);
    T70_U32(uart_snapshot.nrf_rx_count);
    T70_U32(remote_rx_valid_packet_count);
    T70_U32(uart_snapshot.nrf_error_count);
    T70_U32(uart_snapshot.nrf_invalid_packet_count);
    T70_U32(uart_snapshot.nrf_irq_count);
    T70_U32(uart_snapshot.nrf_status_reg);
    T70_U32(uart_snapshot.nrf_config_reg);
    T70_U32(uart_snapshot.nrf_channel_reg);
    T70_U32(uart_snapshot.nrf_rf_setup_reg);
    T70_U32(uart_snapshot.nrf_fifo_status_reg);
    T70_U32(uart_snapshot.cpu_task_load_x100[3]);
    T70_U32(uart_snapshot.task_last_exec_us[3]);
    T70_U32(uart_snapshot.task_max_exec_us[3]);
    T70_U32(uart_snapshot.task_deadline_miss[3]);
    T70_U32(nrf_tlm_schedule_count);
    T70_U32(nrf_tlm_tx_start_count);
    T70_U32(nrf_tlm_tx_success_count);
    T70_U32(nrf_tlm_tx_fail_count);
    T70_U32(nrf_tlm_pending_replace_count);
    T70_U32(nrf_tlm_last_tx_duration_us);
    T70_U32(nrf_tlm_max_tx_duration_us);

    /* R8R16 E-STOP one-shot needle emergency-close diagnostics. */
    T70_U32(App_IsStopLatched());
    T70_U32(NeedleValveAutonomousControl_IsEStopSafeCloseActive());
    T70_U32(NeedleValveAutonomousControl_IsEStopSafeCloseComplete());
    T70_U32(NeedleValveAutonomousControl_HasEStopSafeCloseFailed());
    T70_U32(NeedleValveAutonomousControl_GetEStopSafeCloseFailReason());
    T70_U32(NeedleValveAutonomousControl_GetEStopSafeCloseStartCount());
    T70_U32(NeedleValveAutonomousControl_GetEStopSafeCloseElapsedMs());

    T70_U32(p110_active_pwm);
    T70_U32(p111_moves_completed);
    T70_U32(p111_abort_reason);
    T70_U32(uart_snapshot.needle.fault);
    T70_U32(uart_snapshot.rcs.valve_applied_mask);
    T70_U32(p112_hard_off_count);
    T70_U32(p112_isr_max_us);

    /* R8R17: append-only needle position diagnostics.  Read-only snapshot;
     * no control, scheduling, ESKF, nRF or actuator decision path changes. */
    T70_U32(needle_telem.raw_adc);
    T70_U32(needle_telem.zero_adc);
    T70_U32(needle_telem.target_adc);
    T70_I32(needle_telem.error_adc);
    T70_U32(needle_telem.zero_valid);
    T70_U32(needle_telem.position_locked);
    T70_U32((uint32_t)needle_auto.state);
    T70_U32(needle_auto.move_in_progress);
    T70_U32(p110_direction);
    T70_U32(needle_telem.lpwm);
    T70_U32(needle_telem.rpwm);
    T70_I32(UARTTelemetry_ScaleFloat(needle_auto.requested_command, 10000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(needle_telem.limited_cmd, 10000.0f));
    T70_U32(NeedleValveAutonomousControl_WasEStopSafeCloseOverrideUsed());

    /* R8R19: latest MATLAB-source-authority flight logic, compute-only. */
    T70_U32(uart_snapshot.flight_logic.synthetic_input_active);
    T70_U32(uart_snapshot.flight_logic.mission_state);
    T70_U32(uart_snapshot.flight_logic.step_count);
    T70_U32(uart_snapshot.flight_logic.logic_elapsed_ms);
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.input_z_cg_m, 1000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.input_vz_mps, 1000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.input_x_m, 1000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.input_y_m, 1000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.input_pitch_deg, 100.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.input_yaw_deg, 100.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.z_reference_m, 1000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.hover_best_s, 1000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.target_force_n, 100.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.valve_cmd, 10000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.target_pitch_rad * 57.2957795f, 100.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.target_yaw_rad * 57.2957795f, 100.0f));
    T70_U32(uart_snapshot.flight_logic.thrust_shortage);
    T70_U32(uart_snapshot.flight_logic.rcs_fault);
    T70_U32(uart_snapshot.flight_logic.rcs_requested_mask);
    T70_U32(uart_snapshot.flight_logic.rcs_applied_mask);
    T70_U32(uart_snapshot.flight_logic.rcs_pitch_mode);
    T70_U32(uart_snapshot.flight_logic.rcs_yaw_mode);
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.predicted_pitch_deg, 100.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.predicted_yaw_deg, 100.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.time_to_ground_s, 1000.0f));

    /* R8R22: append-only real-ESKF source qualification diagnostics. */
    T70_U32(uart_snapshot.flight_logic.real_input_active);
    T70_U32(uart_snapshot.flight_logic.input_valid);
    T70_U32(uart_snapshot.flight_logic.input_reject_reason);
    T70_U32(uart_snapshot.flight_logic.horizontal_position_valid);
    T70_U32(uart_snapshot.flight_logic.vertical_position_valid);
    T70_U32(uart_snapshot.flight_logic.eskf_origin_zeroed);
    T70_U32(uart_snapshot.flight_logic.eskf_output_inhibited);
    T70_U32(uart_snapshot.flight_logic.input_eskf_age_ms);
    T70_U32(uart_snapshot.flight_logic.input_imu_age_ms);
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.input_vx_mps, 1000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.input_vy_mps, 1000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.input_pitch_rate_dps, 100.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.input_yaw_rate_dps, 100.0f));
    T70_U32(uart_snapshot.flight_logic.invalid_input_count);

    /* R8R24: sign-diagnostic observability; append only. */
    T70_U32(uart_snapshot.flight_logic.horizontal_target_gated);
    T70_U32(uart_snapshot.flight_logic.rcs_v1_event_count);
    T70_U32(uart_snapshot.flight_logic.rcs_v3_event_count);
    T70_U32(uart_snapshot.flight_logic.rcs_v5_event_count);
    T70_U32(uart_snapshot.flight_logic.rcs_v7_event_count);

    /* R8R30: fixed IMU/sensor-frame -> physical rocket-frame transform diagnostics. */
    T70_U32(uart_snapshot.flight_logic.mount_cal_phase);
    T70_U32(uart_snapshot.flight_logic.mount_cal_valid);
    T70_U32(uart_snapshot.flight_logic.mount_cal_fault);
    T70_U32(uart_snapshot.flight_logic.mount_cal_upright_samples);
    T70_U32(uart_snapshot.flight_logic.mount_cal_tilt_samples);
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_cal_tilt_deg, 100.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_r00, 10000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_r01, 10000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_r02, 10000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_r10, 10000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_r11, 10000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_r12, 10000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_r20, 10000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_r21, 10000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_r22, 10000.0f));

    /* R8R27 append-only +Y pose and matrix-quality diagnostics. */
    T70_U32(uart_snapshot.flight_logic.mount_cal_y_tilt_samples);
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_cal_y_tilt_deg, 100.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_cal_xy_angle_deg, 100.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_cal_axis_agreement, 10000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_cal_ortho_error, 10000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_cal_det, 10000.0f));

    /* R8R28 append-only -X/-Y symmetric-pair quality diagnostics. */
    T70_U32(uart_snapshot.flight_logic.mount_cal_neg_x_samples);
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_cal_neg_x_tilt_deg, 100.0f));
    T70_U32(uart_snapshot.flight_logic.mount_cal_neg_y_samples);
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_cal_neg_y_tilt_deg, 100.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_cal_x_opposition, 10000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_cal_y_opposition, 10000.0f));
    T70_I32(UARTTelemetry_ScaleFloat(uart_snapshot.flight_logic.mount_cal_z_agreement, 10000.0f));

    /* R8R35 final dry-run diagnostics: expose the exact needle low-level root
     * cause and verify RCS physical pulse duration after the legacy-output cut
     * was removed. Read-only telemetry; no control-path decisions use these. */
    T70_U32((uint32_t)needle_auto.fault);
    T70_U32(needle_telem.stall_ms);
    T70_U32(needle_auto.command_reject_count);
    T70_U32(p110_state);
    T70_U32(p110_result);
    T70_U32(p110_abort_reason);
    T70_U32(p110_total_powered_ms);
    T70_U32(p110_speed_adc_s);
    T70_U32(p110_stop_distance_adc);
    T70_U32(p110_correction_count);
    {
        const SolenoidOutputStatus_t r8r35_sol = SolenoidOutput_GetStatus();
        T70_U32(r8r35_sol.pulse_complete_count);
        T70_U32(r8r35_sol.last_pulse_ms);
        T70_U32(r8r35_sol.max_pulse_ms);
    }

    /* R8R35R1 diagnostic closure: append-only observability for the two
     * remaining dry-run anomalies. These values are READ-ONLY; no control,
     * safety, scheduler, actuator or SD recovery decision uses them. */
    T70_U32(p83_diag_flags);
    T70_U32(p83_adc1_raw);
    T70_U32(p83_adc2_raw);
    T70_U32(p83_pair_diff);
    T70_U32(p83_pair_candidate);
    T70_U32(p83_median7);
    T70_U32(p83_filtered_adc);
    T70_U32(p83_feedback_valid);
    T70_U32(p83_confidence_pct);
    T70_U32(p83_pair_reject_count);
    T70_U32(p83_rate_reject_count);
    T70_U32(p83_quarantine_count);
    T70_U32(p83_reacquire_count);
    T70_U32(p83_mode);
    T70_U32(p83_acq_progress_pct);
    T70_U32(p83_adc_timeout_count);
    T70_U32(p83_win_raw_pp);
    T70_U32(p83_win_filtered_pp);
    T70_U32(p83_vref_win_pp_raw12);

    T70_U32(uart_snapshot.sd_initialized);
    T70_U32(uart_snapshot.sd_mount_ok);
    T70_U32(uart_snapshot.sd_file_open);
    T70_U32(uart_snapshot.sd_logging);
    T70_U32(uart_snapshot.sd_last_result);
    T70_U32(uart_snapshot.sd_disk_status);
    T70_U32(uart_snapshot.sd_mount_retry_count);
    T70_U32(uart_snapshot.sd_error_count);
    T70_U32(uart_snapshot.sd_write_error_count);
    T70_U32(uart_snapshot.sd_dropped_frame_count);
    T70_U32(uart_snapshot.sd_ring_overrun_count);
    T70_U32(uart_snapshot.sd_async_timeout_count);
    T70_U32(uart_snapshot.sd_ring_count);
    T70_U32(uart_snapshot.sd_ring_high_watermark);
    T70_U32(uart_snapshot.sd_backpressure_level);
    T70_U32(uart_snapshot.sd_guard_pending);
    T70_U32(uart_snapshot.sd_guard_deferred);
    T70_U32(uart_snapshot.sd_card_busy_polls);
    T70_U32(uart_snapshot.sd_runtime_reinits);
    T70_U32(uart_snapshot.sd_runtime_reinit_success);
    T70_U32(uart_snapshot.sd_runtime_reinit_failures);
    T70_U32(uart_snapshot.sd_runtime_last_error);
    T70_U32(uart_snapshot.sd_runtime_recovery_count);
    T70_U32(uart_snapshot.sd_runtime_recovery_success);
    T70_U32(uart_snapshot.sd_runtime_recovery_failures);
    T70_U32(uart_snapshot.sd_runtime_recovery_flight_aborts);
    T70_U32(uart_snapshot.sd_runtime_recovery_last_us);
    T70_U32(uart_snapshot.sd_runtime_recovery_max_us);

    /* R8R35R3 SD-only throughput closure diagnostics. */
    T70_U32(sd_logger_frame_count);
    T70_U32(sd_logger_ring_push_count);
    T70_U32(sd_logger_ring_pop_count);
    T70_U32(sd_logger_async_data_start_count);
    T70_U32(sd_logger_async_data_complete_count);
    T70_U32(v87_sd_update_count);
    T70_U32(p34_sd_defer_count);
    T70_U32(p112_sd_suppressed_count);
    T70_U32(sd_logger_drain_last_us);
    T70_U32(sd_logger_drain_max_us);
    T70_U32(sd_logger_drain_budget_yield_count);
    T70_U32(sd_logger_max_write_duration_us);

    /* R8R35R3R4: prove that short HAL host-busy windows are deferred rather
     * than promoted to FR_DISK_ERR, while real BSP/HAL start errors remain
     * fail-visible. */
    T70_U32(sd_logger_dma_retry_card_busy_count);
    T70_U32(sd_bsp_hal_state);
    T70_U32(sd_dma_start_error_count);


    /* R8R35R3R7: exact first/last HAL SD DMA-launch failure snapshots. */
    T70_U32(sd_r7_write_start_fail_count);
    T70_U32(sd_r7_first_fail_time_ms);
    T70_U32(sd_r7_first_fail_hal_status);
    T70_U32(sd_r7_first_fail_hsd_state);
    T70_U32(sd_r7_first_fail_hsd_error);
    T70_U32(sd_r7_first_fail_hsd_context);
    T70_U32(sd_r7_first_fail_dma_state);
    T70_U32(sd_r7_first_fail_dma_error);
    T70_U32(sd_r7_first_fail_sdio_sta);
    T70_U32(sd_r7_first_fail_sdio_dctrl);
    T70_U32(sd_r7_first_fail_sdio_dcount);
    T70_U32(sd_r7_last_fail_time_ms);
    T70_U32(sd_r7_last_fail_hal_status);
    T70_U32(sd_r7_last_fail_hsd_state);
    T70_U32(sd_r7_last_fail_hsd_error);
    T70_U32(sd_r7_last_fail_hsd_context);
    T70_U32(sd_r7_last_fail_dma_state);
    T70_U32(sd_r7_last_fail_dma_error);
    T70_U32(sd_r7_last_fail_sdio_sta);
    T70_U32(sd_r7_last_fail_sdio_dctrl);
    T70_U32(sd_r7_last_fail_sdio_dcount);

    /* R8R35R3R7R1: prove no millisecond HAL delay can block runtime. */
    T70_U32(RuntimeDelayGuard_GetViolationCount());
    T70_U32(RuntimeDelayGuard_GetLastRequestedMs());

    /* R8R35R3R9: verify the actuator quiet-window and the ACTUAL BSP SDIO
     * clock register used after host reset/re-init. */
    T70_U32(sd_logger_motor_write_hold_active);
    T70_U32(sd_logger_r9_hold_remaining_ms);
    T70_U32(sd_logger_r9_rcs_hold_event_count);
    T70_U32(sd_bsp_sdio_clkcr);

    /* R8R35R3R10 RAM-first flight logger. */
    T70_U32(sd_logger_r10_ram_mode);
    T70_U32(sd_logger_r10_ram_frame_count);
    T70_U32(sd_logger_r10_ram_capacity);
    T70_U32(sd_logger_r10_ram_overflow_count);
    T70_U32(sd_logger_r10_capture_frozen);
    T70_U32(sd_logger_r10_flush_active);
    T70_U32(sd_logger_r10_flush_index);
    T70_U32(sd_logger_r10_flush_complete);
    T70_U32(sd_logger_r10_tail_dma_at_entry);
    T70_U32(sd_logger_r10_physical_sd_fault_count);
    T70_U32(sd_logger_r10_postflight_reason);
    T70_U32(sd_bsp_sdio_clkcr);

    crc = UARTTelemetry_Crc16Ccitt(
        (const uint8_t *)uart_telemetry_buffer,
        position
    );
    uart_telemetry_last_crc16 = crc;

    if ((UARTTelemetry_AppendChar(&position, '*') == 0U) ||
        (UARTTelemetry_AppendHex16(&position, crc) == 0U) ||
        (UARTTelemetry_AppendChar(&position, '\r') == 0U) ||
        (UARTTelemetry_AppendChar(&position, '\n') == 0U))
    {
        return 0U;
    }

#undef T70_U32
#undef T70_I32

    *message_length = position;
    return 1U;
}

void UARTTelemetry_Update10Hz(void)
{
    static const char timing_banner[] =
        "# TGY73 R8R35R3R10R3 FINAL CANDIDATE / RX-ONLY FLIGHT / HOLD-SAFE FB | FW " APP_VERSION_STRING
        " | USART2 DMA TX | COMPACT / READ-ONLY\r\n";
    static const char timing_header[] =
        "# frame,seq,time_ms,ready,flight_active,pe9_open,auth,"
        "imu_valid,imu_age_ms,baro_valid,baro_fresh,baro_pa,baro_tcdeg,"
        "lidar_valid,lidar_fresh,lidar_age_ms,lidar_state,lidar_errors,lidar_timeouts,lidar_recoveries,"
        "lidar_rec_active,lidar_rec_step,lidar_rec_attempts,lidar_rec_success,lidar_rec_failures,lidar_rec_last_us,lidar_rec_max_us,"
        "eskf_ok,eskf_age_ms,eskf_cov_ok,eskf_cov_checks,"
        "eskf_corr_us,eskf_corr_max_us,eskf_predict_us,eskf_predict_max_us,"
        "eskf_gravity_us,eskf_gravity_max_us,eskf_stationary_us,eskf_stationary_max_us,"
        "eskf_baro_aid_us,eskf_baro_aid_max_us,eskf_lidar_aid_us,eskf_lidar_aid_max_us,"
        "eskf_public_us,eskf_public_max_us,eskf_public_count,gravity_updates,gravity_rejects,gravity_joseph_updates,gravity_joseph_faults,"
        "cpu_x100,cpu_bg_x100,cpu_imu_x100,cpu_baro_x100,cpu_lidar_x100,cpu_eskf_x100,cov_us,cov_max_us,"
        "task_imu_us,task_baro_us,task_lidar_us,task_eskf_us,task_imu_max_us,task_baro_max_us,task_lidar_max_us,task_eskf_max_us,"
        "miss_imu,miss_baro,miss_lidar,miss_eskf,system_ok,system_fault,sys_imu_fresh,sys_lidar_fresh,sys_eskf_fresh,"
        "scheduler_realigns,imu_late_us,imu_late_max_us,baro_late_us,baro_late_max_us,eskf_late_us,eskf_late_max_us,"
        "baro_defer_count,baro_defer_streak,baro_defer_streak_max,baro_defer_slack_us,"
        "eskf_defer_count,eskf_defer_streak,eskf_defer_streak_max,eskf_defer_slack_us,"
        "bg_fresh_us,bg_fresh_max_us,bg_remote_us,bg_remote_max_us,bg_control_us,bg_control_max_us,sd_update_us,sd_update_max_us,"
        "bg_uart_us,bg_uart_max_us,uart_defers,sd_ready,"
        "nrf_connected,nrf_link,nrf_flags,nrf_age_ms,nrf_rx_count,nrf_valid_count,nrf_errors,nrf_invalid,nrf_irq,"
        "nrf_status,nrf_config,nrf_channel,nrf_rf_setup,nrf_fifo,cpu_nrf_x100,task_nrf_us,task_nrf_max_us,miss_nrf,"
        "nrf_tlm_schedule,nrf_tlm_tx_start,nrf_tlm_tx_success,nrf_tlm_tx_fail,nrf_tlm_pending_replace,nrf_tlm_last_tx_us,nrf_tlm_max_tx_us,"
        "stop_latched,estop_close_active,estop_close_complete,estop_close_failed,estop_close_fail_reason,estop_close_start_count,estop_close_elapsed_ms,"
        "p110_pwm,p111_moves,p111_fault,needle_fault,rcs_mask,hardoff,isr_max_us,"
        "needle_adc,needle_closed_ref_adc,needle_target_adc,needle_error_adc,needle_ref_valid,needle_position_locked,"
        "needle_auto_state,needle_move_in_progress,p110_direction,needle_lpwm,needle_rpwm,needle_cmd_x10000,needle_pos_x10000,estop_override_used,"
        "fl_synth,fl_state,fl_step_count,fl_elapsed_ms,fl_z_cg_mm,fl_vz_mms,fl_x_mm,fl_y_mm,fl_pitch_cdeg,fl_yaw_cdeg,"
        "fl_zref_mm,fl_hover_best_ms,fl_target_force_cN,fl_valve_x10000,fl_target_pitch_cdeg,fl_target_yaw_cdeg,"
        "fl_thrust_shortage,fl_rcs_fault,fl_rcs_req_mask,fl_rcs_applied_mask,fl_rcs_pitch_mode,fl_rcs_yaw_mode,"
        "fl_pred_pitch_cdeg,fl_pred_yaw_cdeg,fl_tgo_ms,"
        "fl_real,fl_input_valid,fl_input_reject,fl_hpos_valid,fl_vpos_valid,fl_origin_zeroed,fl_eskf_inhibit,"
        "fl_input_eskf_age_ms,fl_input_imu_age_ms,fl_vx_mms,fl_vy_mms,fl_pitch_rate_cdeg_s,fl_yaw_rate_cdeg_s,fl_invalid_count,"
        "fl_horiz_gated,fl_v1_events,fl_v3_events,fl_v5_events,fl_v7_events,"
        "cal_phase,cal_valid,cal_fault,cal_upright_samples,cal_tilt_samples,cal_tilt_cdeg,"
        "cal_r00_x10000,cal_r01_x10000,cal_r02_x10000,cal_r10_x10000,cal_r11_x10000,cal_r12_x10000,"
        "cal_r20_x10000,cal_r21_x10000,cal_r22_x10000,"
        "cal_y_tilt_samples,cal_y_tilt_cdeg,cal_xy_angle_cdeg,cal_axis_agree_x10000,cal_ortho_err_x10000,cal_det_x10000,"
        "cal_neg_x_samples,cal_neg_x_tilt_cdeg,cal_neg_y_samples,cal_neg_y_tilt_cdeg,cal_x_opp_x10000,cal_y_opp_x10000,cal_z_agree_x10000,"
        "needle_auto_fault,needle_stall_ms,needle_cmd_rejects,p110_state,p110_result,p110_abort_reason,p110_powered_ms,p110_speed_adc_s,p110_stop_distance_adc,p110_corrections,"
        "rcs_pulse_complete_count,rcs_last_pulse_ms,rcs_max_pulse_ms,"
        "p83_diag_flags,p83_adc1_raw,p83_adc2_raw,p83_pair_diff,p83_pair_candidate,p83_median7,p83_filtered_adc,p83_feedback_valid,p83_confidence_pct,"
        "p83_pair_reject_count,p83_rate_reject_count,p83_quarantine_count,p83_reacquire_count,p83_mode,p83_acq_progress_pct,p83_adc_timeout_count,p83_win_raw_pp,p83_win_filtered_pp,p83_vref_win_pp_raw12,"
        "sd_initialized,sd_mount_ok,sd_file_open,sd_logging,sd_last_result,sd_disk_status,sd_mount_retries,sd_errors,sd_write_errors,sd_dropped,sd_ring_overruns,sd_async_timeouts,sd_ring_count,sd_ring_high_water,sd_backpressure_level,sd_guard_pending,sd_guard_deferred,sd_card_busy_polls,"
        "sd_runtime_reinits,sd_runtime_reinit_success,sd_runtime_reinit_failures,sd_runtime_last_error,sd_runtime_rec_count,sd_runtime_rec_success,sd_runtime_rec_failures,sd_runtime_rec_flight_aborts,sd_runtime_rec_last_us,sd_runtime_rec_max_us,"
        "sd_frames_total,sd_ring_push_total,sd_ring_pop_total,sd_dma_start_total,sd_dma_complete_total,sd_update_count_total,sd_defer_total,sd_suppressed_total,sd_drain_last_us_r3,sd_drain_max_us_r3,sd_drain_yields_total,sd_write_max_us_r3,"
        "sd_host_busy_defers_r4,sd_bsp_hal_state_r4,sd_dma_start_errors_r4,"
        "sd_r7_fail_count,"
        "sd_r7_first_time_ms,sd_r7_first_hal_status,sd_r7_first_hsd_state,sd_r7_first_hsd_error,sd_r7_first_hsd_context,sd_r7_first_dma_state,sd_r7_first_dma_error,sd_r7_first_sdio_sta,sd_r7_first_sdio_dctrl,sd_r7_first_sdio_dcount,"
        "sd_r7_last_time_ms,sd_r7_last_hal_status,sd_r7_last_hsd_state,sd_r7_last_hsd_error,sd_r7_last_hsd_context,sd_r7_last_dma_state,sd_r7_last_dma_error,sd_r7_last_sdio_sta,sd_r7_last_sdio_dctrl,sd_r7_last_sdio_dcount,"
        "runtime_hal_delay_violations,runtime_hal_delay_last_ms,"
        "sd_hold_active_r9,sd_hold_remaining_ms_r9,sd_rcs_hold_events_r9,sd_sdio_clkcr_r9,"
        "sd_r10_ram_mode,sd_r10_ram_frames,sd_r10_ram_capacity,sd_r10_ram_overflow,sd_r10_capture_frozen,"
        "sd_r10_flush_active,sd_r10_flush_index,sd_r10_flush_complete,sd_r10_tail_dma_at_entry,"
        "sd_r10_physical_sd_faults,sd_r10_postflight_reason,sd_sdio_clkcr_r10,"
        "crc16_ccitt\r\n";
    static const char banner[] =
        "# TGY UART FLIGHT DIAGNOSTICS V68 | FW " APP_VERSION_STRING
        " | USART2 PA2-TX PA3-RX | 115200 8N1 | DMA TX | RX COMMANDS OFF\r\n";
    static const char header[] =
        "# frame,seq,time_ms,preflight_state,flight_active,pe9_raw_open,"
        "pe9_debounced_open,preflight_ready,preflight_fault,flight_time_ms,"
        "connector_seen,actuator_authorized,"
        "imu_valid,imu_age_ms,imu_recovery_state,imu_recovery_step,"
        "imu_recovery_count,imu_recovery_attempts,imu_recovery_failures,"
        "imu_recovery_last_us,imu_recovery_max_us,imu_stale_count,"
        "imu_pattern_errors,imu_pattern_retries,imu_pattern_retry_success,"
        "imu_pattern_recovery_escalations,imu_fast_cfg_checks,"
        "imu_fast_cfg_attempts,imu_fast_cfg_success,imu_fast_cfg_failures,"
        "imu_fast_cfg_last_us,imu_fast_cfg_max_us,"
        "imu_stale_fast_checks,imu_stale_fast_attempts,imu_stale_fast_success,"
        "imu_stale_fast_failures,imu_stale_retry_success,imu_stale_recovery_escalations,"
        "imu_stale_fast_last_us,imu_stale_fast_max_us,"
        "imu_stale_warmup_events,imu_stale_warmup_polls,imu_stale_warmup_success,"
        "imu_stale_warmup_timeouts,imu_stale_warmup_first_ready_us,"
        "imu_stale_warmup_max_ready_us,imu_stale_warmup_last_status,"
        "imu_stale_warmup_active,"
        "imu_stale_diag_valid,imu_stale_diag_event_count,imu_stale_diag_time_ms,"
        "imu_stale_diag_age_us,imu_stale_diag_reg_valid,imu_stale_diag_whoami,"
        "imu_stale_diag_ctrl1_xl,imu_stale_diag_ctrl2_g,imu_stale_diag_ctrl3_c,"
        "imu_stale_diag_ctrl4_c,imu_invalid_samples,imu_redundant_rejects,"
        "imu_pat_valid,imu_pat_event_count,imu_pat_time_ms,imu_pat_mag,imu_pat_sign_mask,"
        "imu_pat_b0_gx,imu_pat_b0_gy,imu_pat_b0_gz,imu_pat_b0_ax,imu_pat_b0_ay,imu_pat_b0_az,"
        "imu_pat_b1_gx,imu_pat_b1_gy,imu_pat_b1_gz,imu_pat_b1_ax,imu_pat_b1_ay,imu_pat_b1_az,"
        "imu_pat_b2_gx,imu_pat_b2_gy,imu_pat_b2_gz,imu_pat_b2_ax,imu_pat_b2_ay,imu_pat_b2_az,"
        "imu_pat_reg_valid,imu_pat_whoami,imu_pat_ctrl1_xl,imu_pat_ctrl2_g,imu_pat_ctrl3_c,imu_pat_ctrl4_c,"
        "baro_valid,lidar_valid,eskf_init,eskf_ok,eskf_inhibit,eskf_reacquire,"
        "eskf_div_reason,eskf_div_count,eskf_reacq_count,eskf_div_candidate,"
        "eskf_reacq_stable,vertical_consistency_mm,vertical_reacq_target_mm,"
        "eskf_public_age_ms,eskf_public_count,eskf_reset_reason,eskf_reset_count,"
        "eskf_num_errors,eskf_public_rejects,eskf_z_jump_rejects,"
        "eskf_vz_jump_rejects,eskf_cov_ok,eskf_cov_checks,eskf_cov_faults,"
        "eskf_cov_reinits,eskf_cov_diag_min_u1e6,eskf_cov_diag_max_m1e3,"
        "eskf_cov_sym_max_u1e6,eskf_cov_fault_stage,eskf_cov_fault_state,"
        "eskf_cov_fault_other,eskf_cov_fault_raw_n1e9,eskf_cov_fault_aux_n1e9,"
        "eskf_cov_fault_time_ms,eskf_cov_rollback_last_good,eskf_cov_rollbacks,"
        "eskf_cov_roundoff_clamps,eskf_cov_roundoff_stage,eskf_cov_roundoff_state,"
        "eskf_cov_roundoff_raw_n1e9,eskf_cov_roundoff_time_ms,baro_fresh,"
        "lidar_fresh,vertical_source_mask,vertical_degraded,eskf_z_mm,"
        "eskf_vz_mmps,baro_alt_mm,lidar_mm,baro_pressure_pa,"
        "baro_ground_pa,baro_temp_cdeg,baro_ref_track_allowed,baro_ref_track_active,"
        "baro_ref_frozen,baro_ref_updates,baro_ref_error_mpa,baro_ref_step_mpa,"
        "baro_ref_last_ms,baro_ref_freezes,baro_innov_mm,"
        "lidar_innov_mm,baro_updates,lidar_updates,baro_rejects,"
        "lidar_rejects,lidar_state,lidar_age_ms,lidar_reads,lidar_errors,"
        "lidar_timeouts,lidar_wait_timeouts,lidar_recoveries,"
        "lidar_rec_active,lidar_rec_step,lidar_rec_attempts,lidar_rec_success,"
        "lidar_rec_failures,lidar_rec_last_us,lidar_rec_max_us,lidar_rec_step_max_us,"
        "roll_cdeg,pitch_cdeg,roll_rate_mdps,pitch_rate_mdps,rcs_state,"
        "rcs_fault,rcs_req_mask,rcs_applied_mask,rcs_roll_pd_us,"
        "rcs_pitch_pd_us,rcs_roll_pulse_rem_ms,rcs_pitch_pulse_rem_ms,"
        "rcs_roll_cooldown_rem_ms,rcs_pitch_cooldown_rem_ms,"
        "rcs_autodamp_mask,rcs_dry_run,rcs_event_count,"
        "rcs_flight_authorized,rcs_safety_inhibited,rcs_inhibit_count,"
        "main_cmd_x10000,"
        "main_target_force_cN,main_est_thrust_cN,main_burn_active,"
        "main_output_valid,main_physical_enabled,needle_req_x10000,"
        "needle_limited_x10000,needle_adc,needle_target_adc,needle_enabled,"
        "needle_lock,needle_fault,needle_zero_adc,needle_error_adc,"
        "needle_rpwm,needle_lpwm,needle_zero_valid,needle_homing_active,"
        "needle_homing_complete,needle_stall_ms,"
        "nrf_connected,nrf_link,nrf_cmd,nrf_flags,"
        "nrf_age_ms,nrf_rx_count,nrf_errors,nrf_invalid,nrf_irq,"
        "nrf_status,nrf_config,nrf_channel,nrf_rf_setup,nrf_fifo,stop_latched,"
        "sd_initialized,sd_mount_ok,sd_file_open,sd_ready,sd_logging,"
        "sd_last_result,sd_disk_status,sd_mount_retries,sd_errors,"
        "sd_write_errors,sd_dropped,sd_ring_overruns,sd_async_timeouts,sd_ring_count,sd_ring_high_water,sd_backpressure_level,sd_backpressure_entries,sd_backpressure_critical_entries,sd_guard_pending,sd_guard_deferred,sd_card_busy_polls,sd_write_max_us,sd_guard_max_us,"
        "sd_fifo_order_faults,sd_fifo_last_started_order,sd_fifo_next_ready_order,"
        "sd_frames,sd_source_publishes,sd_hal_init,sd_wide,sd_host_attempts,"
        "sd_host_resets,sd_hal_error,sd_last_hal_error,sd_recovered,"
        "sd_soft_recoveries,sd_runtime_reinits,sd_runtime_reinit_success,"
        "sd_runtime_reinit_failures,sd_runtime_last_error,sd_runtime_rec_count,"
        "sd_runtime_rec_success,sd_runtime_rec_failures,sd_runtime_rec_flight_aborts,"
        "sd_runtime_rec_last_us,sd_runtime_rec_max_us,"
        "cpu_x100,cpu_bg_x100,sd_update_us,sd_update_max_us,sd_drain_us,"
        "sd_drain_max_us,sd_drain_yields,cov_us,cov_max_us,cov_count,grav_joseph_updates,grav_joseph_faults,"
        "zupt_joseph_updates,zupt_joseph_faults,"
        "sched_slow_defers,bg_fresh_us,bg_remote_us,bg_control_us,bg_uart_us,"
        "bg_uart_max_us,sd_defers,control_defers,uart_defers,"
        "cpu_imu_x100,cpu_baro_x100,cpu_lidar_x100,cpu_nrf_x100,"
        "cpu_eskf_x100,task_imu_us,task_baro_us,task_lidar_us,task_nrf_us,"
        "task_eskf_us,task_imu_max_us,task_baro_max_us,task_lidar_max_us,"
        "task_nrf_max_us,task_eskf_max_us,miss_imu,miss_baro,miss_lidar,"
        "miss_nrf,miss_eskf,"
        "fast_fault,baro_raw_spike_rejects,baro_raw_step_confirms,"
        "eskf_lidar_soft_reacq_active,eskf_lidar_soft_reacq_count,"
        "eskf_lidar_soft_reacq_success,"
        "system_ok,system_fault,system_imu_fresh,system_lidar_fresh,"
        "system_eskf_fresh,scheduler_realigns,uart_dma_errors,"
        "uart_busy_skips,nrf_tlm_schedule,nrf_tlm_tx_start,"
        "nrf_tlm_tx_success,nrf_tlm_tx_fail,nrf_tlm_pending_replace,"
        "nrf_tlm_last_tx_us,nrf_tlm_max_tx_us,"
        "p83_diag_flags,p83_samples_total,p83_adc1_raw,p83_adc2_raw,"
        "p83_pair_diff,p83_pair_candidate,p83_median7,p83_filtered_adc,"
        "p83_feedback_valid,p83_confidence_pct,p83_pair_reject_count,"
        "p83_rate_reject_count,p83_quarantine_count,p83_reacquire_count,"
        "p83_mode,p83_acq_progress_pct,p83_adc_timeout_count,"
        "p83_win_raw_pp,p83_win_filtered_pp,p83_vref_win_pp_raw12,"
        "p87_sweep_samples,p87_adc1_min,p87_adc1_max,p87_adc2_min,p87_adc2_max,"
        "p87_candidate_min,p87_candidate_max,p87_adc1_max_step,p87_adc1_step_from,"
        "p87_adc1_step_to,p87_adc2_max_step,p87_adc2_step_from,p87_adc2_step_to,"
        "p87_candidate_max_step,p87_candidate_step_from,p87_candidate_step_to,"
        "p87_filtered_max_step,p87_filtered_step_from,p87_filtered_step_to,"
        "p87_raw_gap_event_count,p87_filtered_gap_event_count,p87_pair_diff_max,"
        "p99_abort_reason,p99_phase,p99_open_chunks,p99_open_drive_ms,"
        "p99_return_chunks,p99_return_drive_ms,"
        "p105_cycle,p105_completed,p105_pass_mask,p105_last_abort,"
        "p105_last_start_adc,p105_last_target_adc,p105_last_open_end_adc,"
        "p105_last_open_error_adc,p105_last_home_adc,p105_last_open_chunks,"
        "p105_last_open_drive_ms,p105_last_return_chunks,p105_last_return_drive_ms,"
        "p106_state,p106_stage,p106_test_pwm,p106_start_adc,p106_stage_start_adc,"
        "p106_end_adc,p106_delta_adc,p106_breakaway_found,p106_breakaway_pwm,p106_result,"
        "p107_state,p107_result,p107_start_adc,p107_target_adc,p107_end_adc,p107_error_adc,"
        "p107_boost_pwm,p107_sustain_pwm,p107_boost_ms,p107_sustain_ms,p107_max_open_drop_adc,p107_brake_cause,"
        "p109_state,p109_result,p109_start_adc,p109_target_adc,p109_prebrake_adc,p109_brake_entry_adc,"
        "p109_adc_100ms,p109_adc_250ms,p109_adc_500ms,p109_coast_100_adc,p109_coast_250_adc,p109_coast_500_adc,"
        "p109_final_error_adc,p109_boost_ms,p109_sustain_ms,p109_brake_cause,"
        "p110_state,p110_result,p110_direction,p110_start_adc,p110_target_adc,p110_current_adc,p110_error_adc,"
        "p110_active_pwm,p110_learned_breakaway_pwm,p110_sustain_pwm,p110_speed_adc_s,p110_stop_distance_adc,"
        "p110_brake_entry_adc,p110_coast_max_adc,p110_final_adc,p110_final_error_adc,p110_search_ms,p110_drive_ms,"
        "p110_brake_ms,p110_total_powered_ms,p110_correction_count,p110_breakaway_found,p110_abort_reason,"
        "p111_state,p111_result,p111_cycle,p111_completed,p111_pass_mask,p111_phase,p111_moves_completed,"
        "p111_baseline_adc,p111_open_target_adc,p111_learned_open_pwm,p111_learned_close_pwm,"
        "p111_learned_open_coast,p111_learned_close_coast,p111_last_open_error_adc,p111_last_close_error_adc,"
        "p111_last_open_powered_ms,p111_last_close_powered_ms,p111_last_open_breakaway_pwm,p111_last_close_breakaway_pwm,"
        "p111_last_open_stop_adc,p111_last_close_stop_adc,p111_last_open_coast_adc,p111_last_close_coast_adc,p111_abort_reason,"
        "p112_isr_control_ticks,p112_isr_adc_samples,p112_hard_off_count,p112_uart_suppressed_count,p112_sd_suppressed_count,"
        "p112_isr_last_us,p112_isr_max_us,p112_timing_critical,p112_decel_active,p112_decel_pwm,p112_initial_coast_adc,"
        "p112r11_state,p112r11_abort_reason,p112r11_abort_count,p112r11_button_debounced,p112r11_open_target_adc,"
        "crc16_ccitt\r\n";
    uint32_t now_ms = HAL_GetTick();
    uint16_t message_length;

    if (uart_telemetry_initialized == 0U)
    {
        return;
    }

    if ((uint32_t)(now_ms - uart_telemetry_previous_service_ms) <
        UART_TELEMETRY_PERIOD_MS)
    {
        return;
    }

    uart_telemetry_previous_service_ms = now_ms;
    uart_telemetry_last_service_ms = now_ms;

    if (USART2_TxDMA_IsBusy() != 0U)
    {
        uart_telemetry_busy_skip_count++;
        return;
    }

    if (uart_telemetry_stage == 0U)
    {
#if (APP_P112R12R8R8_ESKF_TIMING_MODE != 0U)
        if (UARTTelemetry_SendText(
                timing_banner,
                (uint16_t)(sizeof(timing_banner) - 1U)
            ) != 0U)
#else
        if (UARTTelemetry_SendText(
                banner,
                (uint16_t)(sizeof(banner) - 1U)
            ) != 0U)
#endif
        {
            uart_telemetry_stage = 1U;
        }
        return;
    }

    if (uart_telemetry_stage == 1U)
    {
#if (APP_P112R12R8R8_ESKF_TIMING_MODE != 0U)
        if (UARTTelemetry_SendText(
                timing_header,
                (uint16_t)(sizeof(timing_header) - 1U)
            ) != 0U)
#else
        if (UARTTelemetry_SendText(
                header,
                (uint16_t)(sizeof(header) - 1U)
            ) != 0U)
#endif
        {
            uart_telemetry_stage = 2U;
            uart_telemetry_streaming = 1U;
        }
        return;
    }

    UARTTelemetry_TakeSnapshot();

#if (APP_P112R12R8R8_ESKF_TIMING_MODE != 0U)
    if (UARTTelemetry_BuildTimingLine(&message_length) == 0U)
#else
    if (UARTTelemetry_BuildDataLine(&message_length) == 0U)
#endif
    {
        uart_telemetry_format_error_count++;
        return;
    }

    if (UARTTelemetry_SendText(
            uart_telemetry_buffer,
            message_length
        ) != 0U)
    {
        uart_telemetry_frame_sequence++;
    }
}
