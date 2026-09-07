#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdint.h>

void AppTasks_Init(void);

void Task_IMU_1kHz(void);
void Task_BarometerValveHealth_200Hz(void);
void Task_Lidar_Service_1kHz(void);
void Task_NRFMonitor_200Hz(void);
void Task_FullESKFCorrection_200Hz(void);
void Task_FullESKFCovariance_25Hz(void);
void Task_IntegrationMonitor_10Hz(void);

extern volatile uint32_t v87_imu_task_counter;
extern volatile uint32_t v87_baro_task_counter;
extern volatile uint32_t v87_lidar_task_counter;
extern volatile uint32_t v87_monitor_task_counter;

extern volatile uint8_t v87_scheduler_ok;
extern volatile uint8_t v87_imu_ok;
extern volatile uint8_t v87_baro_ok;
extern volatile uint8_t v87_lidar_ok;
extern volatile uint8_t v87_valve_tick_ok;
extern volatile uint8_t v87_adc_ok;
extern volatile uint8_t v87_integration_ok;

extern volatile uint8_t v87_sd_ok;
extern volatile uint8_t v87_sd_progress_ok;
extern volatile uint32_t v87_sd_frame_delta_100ms;
extern volatile uint32_t v87_sd_bytes_delta_100ms;

/* IMU */
extern volatile uint8_t v87_imu_connected;
extern volatile uint8_t v87_imu_device_id;
extern volatile uint8_t v87_imu_driver_sample_valid;
extern volatile uint8_t v87_imu_progress_ok;
extern volatile uint32_t v87_imu_sample_age_us;
extern volatile uint32_t v87_imu_task_delta_100ms;
extern volatile uint32_t v87_imu_task_last_exec_us;
extern volatile uint32_t v87_imu_task_max_exec_us;
extern volatile uint32_t v87_imu_task_overrun_count;
extern volatile uint32_t v87_imu_task_deadline_miss_count;

/* Barometer */
extern volatile uint8_t v87_baro_connected;
extern volatile uint8_t v87_baro_data_ready;
extern volatile uint8_t v87_baro_pressure_valid;
extern volatile uint8_t v87_baro_calibrated;
extern volatile uint8_t v87_baro_healthy;
extern volatile uint8_t v87_baro_progress_ok;
extern volatile float v87_baro_pressure_pa;
extern volatile float v87_baro_temperature_c;
extern volatile float v87_baro_filtered_altitude_m;
extern volatile uint32_t v87_baro_source_update_count;
extern volatile uint32_t v87_baro_source_delta_100ms;
extern volatile uint32_t v87_baro_task_delta_100ms;
extern volatile uint32_t v87_baro_task_last_exec_us;
extern volatile uint32_t v87_baro_task_max_exec_us;
extern volatile uint32_t v87_baro_task_overrun_count;
extern volatile uint32_t v87_baro_task_deadline_miss_count;

/* LIDAR */
extern volatile uint8_t v87_lidar_initialized;
extern volatile uint8_t v87_lidar_connected;
extern volatile uint8_t v87_lidar_data_ready;
extern volatile uint8_t v87_lidar_distance_valid;
extern volatile uint8_t v87_lidar_progress_ok;
extern volatile uint16_t v87_lidar_distance_cm;
extern volatile float v87_lidar_distance_m;
extern volatile float v87_lidar_median_distance_m;
extern volatile float v87_lidar_filtered_distance_m;
extern volatile uint8_t v87_lidar_calibration_complete;
extern volatile uint16_t v87_lidar_calibration_sample_count;
extern volatile uint32_t v87_lidar_update_count;
extern volatile uint32_t v87_lidar_update_delta_100ms;
extern volatile uint32_t v87_lidar_read_count;
extern volatile uint32_t v87_lidar_error_count;
extern volatile uint32_t v87_lidar_timeout_count;
extern volatile uint32_t v87_lidar_dma_tx_start_count;
extern volatile uint32_t v87_lidar_dma_tx_complete_count;
extern volatile uint32_t v87_lidar_dma_rx_start_count;
extern volatile uint32_t v87_lidar_dma_rx_complete_count;
extern volatile uint32_t v87_lidar_dma_error_count;
extern volatile uint32_t v87_lidar_dma_busy_count;
extern volatile uint32_t v87_lidar_last_sample_interval_us;
extern volatile uint32_t v87_lidar_min_sample_interval_us;
extern volatile uint32_t v87_lidar_max_sample_interval_us;
extern volatile uint32_t v87_lidar_sample_age_us;
extern volatile uint32_t v87_lidar_probe_attempt_count;
extern volatile uint32_t v87_lidar_probe_success_count;
extern volatile uint32_t v87_lidar_probe_failure_count;
extern volatile uint32_t v87_lidar_bus_recovery_count;
extern volatile uint32_t v87_lidar_last_i2c_error_code;
extern volatile uint32_t v87_lidar_task_delta_100ms;
extern volatile uint32_t v87_lidar_task_last_exec_us;
extern volatile uint32_t v87_lidar_task_max_exec_us;
extern volatile uint32_t v87_lidar_task_overrun_count;
extern volatile uint32_t v87_lidar_task_deadline_miss_count;

/* Needle */
extern volatile uint32_t v87_valve_tick_delta_100ms;
extern volatile uint32_t v87_adc_timeout_count;
extern volatile uint8_t v87_valve_fault_snapshot;


/* V8.9 NRF24 monitor-only diagnostics. */
extern volatile uint8_t v89_scheduler_ok;
extern volatile uint8_t v89_nrf_hw_ok;
extern volatile uint8_t v89_integration_ok;

extern volatile uint8_t v89_nrf_initialized;
extern volatile uint8_t v89_nrf_connected;
extern volatile uint8_t v89_nrf_mode;
extern volatile uint8_t v89_nrf_status_reg;
extern volatile uint8_t v89_nrf_config_reg;
extern volatile uint8_t v89_nrf_rf_ch_reg;
extern volatile uint8_t v89_nrf_rf_setup_reg;
extern volatile uint8_t v89_nrf_fifo_status_reg;

extern volatile uint8_t v89_remote_link_active;
extern volatile uint8_t v89_remote_command;
extern volatile uint8_t v89_remote_last_sequence;

extern volatile uint32_t v89_nrf_rx_count;
extern volatile uint32_t v89_nrf_tx_count;
extern volatile uint32_t v89_nrf_tx_fail_count;
extern volatile uint32_t v89_nrf_error_count;

extern volatile uint32_t v89_remote_valid_packet_count;
extern volatile uint32_t v89_remote_invalid_packet_count;
extern volatile uint32_t v89_remote_timeout_count;
extern volatile uint32_t v89_remote_irq_count;
extern volatile uint32_t v89_remote_last_packet_age_ms;

extern volatile uint32_t v89_nrf_task_counter;
extern volatile uint32_t v89_nrf_task_delta_100ms;
extern volatile uint32_t v89_nrf_task_last_exec_us;
extern volatile uint32_t v89_nrf_task_max_exec_us;
extern volatile uint32_t v89_nrf_task_overrun_count;
extern volatile uint32_t v89_nrf_task_deadline_miss_count;

extern volatile uint8_t v89_spi3_ok;
extern volatile uint8_t v89_spi3_fault;
extern volatile uint8_t v89_spi3_last_error;
extern volatile uint32_t v89_spi3_transaction_count;
extern volatile uint32_t v89_spi3_error_count;
extern volatile uint32_t v89_spi3_timeout_count;
extern volatile uint32_t v89_spi3_busy_count;
extern volatile uint32_t v89_spi3_slow_count;
extern volatile uint32_t v89_spi3_last_duration_us;
extern volatile uint32_t v89_spi3_max_duration_us;


/* V8.10 NRF -> tahliye servo diagnostics. */
extern volatile uint8_t v810_servo_ok;
extern volatile uint8_t v810_integration_ok;

extern volatile uint8_t v810_servo_init_ok;
extern volatile uint8_t v810_servo_remote_command;
extern volatile uint8_t v810_servo_link_active;
extern volatile uint8_t v810_servo_target_open;
extern volatile uint16_t v810_servo_pulse_us;

extern volatile uint32_t v810_servo_update_count;
extern volatile uint32_t v810_servo_command_change_count;


/* V8.11 cleanup diagnostics. */
extern volatile uint8_t v811_lidar_sample_rate_ok;
extern volatile uint32_t v811_lidar_sample_delta_100ms;
extern volatile uint8_t v811_integration_ok;


/* V8.12 Garmin LIDAR-Lite v3 balanced 250 Hz diagnostics. */
extern volatile uint8_t v813_lidar_profile_config_ok;
extern volatile uint8_t v813_lidar_200hz_ok;
extern volatile uint32_t v813_lidar_sample_delta_100ms;
extern volatile float v813_lidar_sample_rate_hz;
extern volatile uint8_t v813_integration_ok;


extern volatile uint8_t v813_core_without_baro_ok;
extern volatile uint8_t v813_sd_accounting_ok;


/* V8.14 LIDAR one-second diagnostics. */
extern volatile uint32_t v814_lidar_sample_delta_1s;
extern volatile float v814_lidar_sample_rate_1s_hz;
extern volatile float v814_lidar_sample_rate_1s_min_hz;
extern volatile float v814_lidar_sample_rate_1s_max_hz;
extern volatile uint32_t v814_lidar_low_rate_window_count;
extern volatile uint8_t v814_lidar_rate_1s_ok;
extern volatile uint8_t v814_integration_ok;


/* V8.15 Full-State ESKF shadow diagnostics. */
extern volatile uint8_t v815_attitude_ok;
extern volatile uint8_t v815_eskf_initialized;
extern volatile uint8_t v815_eskf_healthy;
extern volatile uint8_t v815_eskf_shadow_mode;
extern volatile uint8_t v815_eskf_lidar_reference_ready;
extern volatile uint8_t v815_eskf_baro_reference_ready;
extern volatile uint8_t v815_eskf_stationary_detected;
extern volatile uint8_t v815_eskf_rate_ok;
extern volatile uint8_t v815_eskf_shadow_core_ok;
extern volatile uint8_t v815_integration_ok;

extern volatile uint32_t v815_attitude_delta_100ms;
extern volatile uint32_t v815_eskf_predict_delta_100ms;
extern volatile uint32_t v815_eskf_public_delta_100ms;
extern volatile uint32_t v815_eskf_covariance_delta_100ms;
extern volatile uint32_t v815_eskf_correction_delta_100ms;

extern volatile uint32_t v815_eskf_correction_task_counter;
extern volatile uint32_t v815_eskf_correction_task_delta_100ms;
extern volatile uint32_t v815_eskf_correction_task_last_exec_us;
extern volatile uint32_t v815_eskf_correction_task_max_exec_us;
extern volatile uint32_t v815_eskf_correction_task_overrun_count;
extern volatile uint32_t v815_eskf_correction_task_deadline_miss_count;

extern volatile uint32_t v815_eskf_predict_max_exec_us;
extern volatile uint32_t v815_eskf_correction_max_exec_us;
extern volatile uint32_t v815_eskf_numerical_error_count;
extern volatile uint32_t v815_eskf_gap_skip_count;
extern volatile uint32_t v815_eskf_reset_count;

extern volatile float v815_eskf_position_x_m;
extern volatile float v815_eskf_position_y_m;
extern volatile float v815_eskf_position_z_m;
extern volatile float v815_eskf_velocity_x_mps;
extern volatile float v815_eskf_velocity_y_mps;
extern volatile float v815_eskf_velocity_z_mps;
extern volatile float v815_eskf_roll_deg;
extern volatile float v815_eskf_pitch_deg;
extern volatile float v815_eskf_yaw_deg;
extern volatile float v815_eskf_gyro_bias_x_dps;
extern volatile float v815_eskf_gyro_bias_y_dps;
extern volatile float v815_eskf_gyro_bias_z_dps;


/* V8.15A pointer/ABI diagnostics. */
extern volatile uint32_t v815a_magic_pre;
extern volatile uint32_t v815a_magic_post;
extern volatile uint8_t v815a_canary_ok;
extern volatile uint8_t v815a_pointer_api_ok;
extern volatile uint8_t v815a_state_finite_ok;
extern volatile uint8_t v815a_shadow_core_ok;
extern volatile uint32_t v815a_sensor_struct_size;
extern volatile uint32_t v815a_attitude_struct_size;
extern volatile uint32_t v815a_eskf_struct_size;


/* V8.15C firmware/RAM identity diagnostics. */
extern const uint32_t v815c_flash_magic;
extern volatile uint32_t v815c_ram_magic;
extern volatile uint8_t v815c_firmware_identity_ok;
extern volatile uint8_t v815c_ram_magic_ok;
extern volatile uint8_t v815c_sd_ring_in_ccm;
extern volatile uint8_t v815c_estimator_basic_sanity_ok;
extern volatile uint32_t v815c_bss_end_address;
extern volatile uint32_t v815c_msp_address;
extern volatile uint32_t v815c_main_sram_gap_bytes;
extern volatile uint32_t v815c_main_sram_gap_min_bytes;

#endif
