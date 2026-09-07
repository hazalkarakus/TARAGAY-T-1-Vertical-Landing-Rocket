#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdint.h>

void SDLogger_Init(void);
void SDLogger_Update(void);
void SDLogger_Stop(void);
void SDLogger_RequestSync(void);

/* P32: 1 kHz path only stores the latest raw IMU sample in a 5-sample ring.
 * Heavy frame/source packing is deliberately kept out of the 1 kHz task. */
void SDLogger_PushFastIMU(void);

/* P32: called from the dedicated 200 Hz ESKF/correction task. */
void SDLogger_PublishSources(void);

/* Called only by the TIM5 200 Hz period callback. */
void SDLogger_TimerCaptureISR(void);

uint8_t SDLogger_IsReady(void);
uint8_t SDLogger_IsLogging(void);
uint8_t SDLogger_TestPassed(void);


/* V8.7 selected Live Expressions / integration diagnostics. */
extern volatile uint8_t sd_logger_initialized;
extern volatile uint8_t sd_logger_ready;
extern volatile uint8_t sd_logger_mount_ok;
extern volatile uint8_t sd_logger_test_done;
extern volatile uint8_t sd_logger_test_passed;
extern volatile uint8_t sd_logger_file_open;
extern volatile uint8_t sd_logger_logging_active;

extern volatile uint8_t sd_logger_last_result;
extern volatile uint8_t sd_logger_disk_init_status;
extern volatile uint8_t sd_logger_mount_retry_count;
extern volatile uint32_t sd_logger_error_count;
extern volatile uint32_t sd_logger_write_error_count;
extern volatile uint32_t sd_logger_total_bytes_written;

extern volatile uint32_t sd_logger_frame_count;
extern volatile uint32_t sd_logger_dropped_frame_count;
extern volatile uint32_t sd_logger_missed_period_count;
extern volatile uint32_t sd_logger_buffer_overrun_count;

extern volatile uint32_t sd_logger_ring_count;
extern volatile uint32_t sd_logger_ring_high_watermark;
extern volatile uint32_t sd_logger_ring_overrun_count;
extern volatile uint32_t sd_logger_drain_last_us;
extern volatile uint32_t sd_logger_drain_max_us;
extern volatile uint32_t sd_logger_drain_budget_yield_count;

/* P47 adaptive backpressure / deferred-guard diagnostics. */
extern volatile uint8_t sd_logger_backpressure_level;
extern volatile uint32_t sd_logger_backpressure_entry_count;
extern volatile uint32_t sd_logger_backpressure_critical_entry_count;
extern volatile uint8_t sd_logger_guard_pending;
extern volatile uint32_t sd_logger_guard_deferred_count;
extern volatile uint32_t sd_logger_async_card_busy_poll_count;
extern volatile uint32_t sd_logger_last_write_duration_us;
extern volatile uint32_t sd_logger_max_write_duration_us;
extern volatile uint32_t sd_logger_last_guard_write_duration_us;
extern volatile uint32_t sd_logger_max_guard_write_duration_us;

/* P49 chronological multi-buffer FIFO diagnostics. */
extern volatile uint32_t sd_logger_fifo_order_fault_count;
extern volatile uint32_t sd_logger_fifo_last_started_order;
extern volatile uint32_t sd_logger_fifo_next_ready_order;

extern volatile uint32_t sd_logger_async_data_start_count;
extern volatile uint32_t sd_logger_async_data_complete_count;
extern volatile uint32_t sd_logger_async_timeout_count;

extern volatile uint8_t sd_logger_capture_timer_started;
extern volatile uint32_t sd_logger_timer_irq_count;
extern volatile uint32_t sd_logger_timer_last_interval_us;
extern volatile uint32_t sd_logger_timer_min_interval_us;
extern volatile uint32_t sd_logger_timer_max_interval_us;
extern volatile uint32_t sd_logger_timer_max_isr_duration_us;

extern volatile uint32_t sd_logger_source_publish_count;
extern volatile uint32_t sd_logger_preallocation_duration_ms;
extern volatile uint32_t sd_logger_preallocated_bytes;
extern volatile uint8_t sd_logger_preallocate_ok;


/* V12 needle-valve logger snapshot. */
extern volatile uint16_t sd_logger_needle_requested_cmd_x10000;
extern volatile uint16_t sd_logger_needle_limited_cmd_x10000;
extern volatile uint16_t sd_logger_needle_raw_adc;
extern volatile uint16_t sd_logger_needle_zero_adc;
extern volatile uint16_t sd_logger_needle_target_adc;
extern volatile int16_t sd_logger_needle_error_adc;
extern volatile uint8_t sd_logger_needle_rpwm;
extern volatile uint8_t sd_logger_needle_lpwm;
extern volatile uint8_t sd_logger_needle_state_flags;
extern volatile uint8_t sd_logger_needle_fault;


/* V13 NRF + tahliye-servo logger snapshot. */
extern volatile uint8_t sd_logger_nrf_link_active;
extern volatile uint8_t sd_logger_nrf_command;
extern volatile uint8_t sd_logger_nrf_last_sequence;
extern volatile uint8_t sd_logger_servo_target_open;
extern volatile uint16_t sd_logger_servo_pulse_us;
extern volatile uint16_t sd_logger_nrf_last_packet_age_ms;
extern volatile uint32_t sd_logger_nrf_valid_packet_count;
extern volatile uint16_t sd_logger_nrf_irq_count_low;
extern volatile uint16_t sd_logger_nrf_rx_count_low;


/* V8.13 transient SDIO retry diagnostics. */
extern volatile uint32_t sd_logger_dma_retry_attempt_count;
extern volatile uint32_t sd_logger_dma_retry_success_count;
extern volatile uint32_t sd_logger_dma_retry_exhausted_count;
extern volatile uint32_t sd_logger_dma_retry_card_busy_count;
extern volatile uint32_t sd_logger_dma_retry_abort_count;
extern volatile uint8_t sd_logger_dma_retry_pending;
extern volatile uint8_t sd_logger_dma_retry_current;

/* P37 runtime SDIO recovery diagnostics. */
extern volatile uint8_t sd_logger_runtime_recovery_active;
extern volatile uint32_t sd_logger_runtime_recovery_count;
extern volatile uint32_t sd_logger_runtime_recovery_success_count;
extern volatile uint32_t sd_logger_runtime_recovery_failure_count;
extern volatile uint32_t sd_logger_runtime_recovery_flight_abort_count;
extern volatile uint32_t sd_logger_runtime_recovery_last_us;
extern volatile uint32_t sd_logger_runtime_recovery_max_us;
extern volatile uint32_t sd_logger_runtime_last_hal_error;


/* R8R35R3R10 RAM-first flight logger diagnostics. */
extern volatile uint8_t sd_logger_r10_ram_mode;
extern volatile uint32_t sd_logger_r10_ram_frame_count;
extern volatile uint32_t sd_logger_r10_ram_capacity;
extern volatile uint32_t sd_logger_r10_ram_overflow_count;
extern volatile uint8_t sd_logger_r10_capture_frozen;
extern volatile uint8_t sd_logger_r10_flush_active;
extern volatile uint32_t sd_logger_r10_flush_index;
extern volatile uint8_t sd_logger_r10_flush_complete;
extern volatile uint8_t sd_logger_r10_tail_dma_at_entry;
extern volatile uint32_t sd_logger_r10_physical_sd_fault_count;
extern volatile uint8_t sd_logger_r10_postflight_reason;
extern volatile uint32_t sd_logger_r10_entry_ms;
extern volatile uint32_t sd_logger_r10_flush_start_ms;

/* V8.15C CCM capture-ring diagnostics. */
extern volatile uint32_t sd_logger_ring_address;
extern volatile uint32_t sd_logger_ring_end_address;
extern volatile uint8_t sd_logger_ring_ccm_expected;

#endif
