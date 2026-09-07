#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>

typedef enum
{
    LIDAR_STATE_UNINITIALIZED = 0,
    LIDAR_STATE_IDLE = 1,
    LIDAR_STATE_TX_START_DMA = 2,
    LIDAR_STATE_WAIT_MEASUREMENT = 3,
    LIDAR_STATE_RX_STATUS_DMA = 4,
    LIDAR_STATE_RX_DISTANCE_DMA = 5,
    LIDAR_STATE_ERROR = 6,
    LIDAR_STATE_RECOVERY = 7

} LidarState_t;

typedef struct
{
    uint8_t initialized;
    uint8_t connected;
    uint8_t data_ready;
    uint8_t distance_valid;

    uint16_t distance_cm;
    float distance_m;

    uint8_t median_initialized;
    uint16_t median_distance_cm;
    float median_distance_m;

    uint8_t filter_enabled;
    uint8_t filter_config_ok;
    uint8_t filter_initialized;
    float filtered_distance_m;

    uint32_t update_count;
    uint32_t read_count;
    uint32_t error_count;
    uint32_t timeout_count;

    uint32_t dma_tx_start_count;
    uint32_t dma_tx_complete_count;
    uint32_t dma_rx_start_count;
    uint32_t dma_rx_complete_count;
    uint32_t dma_busy_count;
    uint32_t dma_error_count;
    uint32_t wait_busy_timeout_count;
    uint32_t bus_recovery_count;

    uint8_t recovery_active;
    uint8_t recovery_step;
    uint16_t recovery_reserved;
    uint32_t recovery_attempt_count;
    uint32_t recovery_success_count;
    uint32_t recovery_failure_count;
    uint32_t recovery_last_duration_us;
    uint32_t recovery_max_duration_us;
    uint32_t recovery_step_max_us;

    uint32_t filter_update_count;
    uint32_t filter_reset_count;

    uint32_t last_sample_timestamp_us;
    uint32_t last_sample_interval_us;
    uint32_t min_sample_interval_us;
    uint32_t max_sample_interval_us;

    uint8_t state;
    uint8_t last_hal_status;

    uint32_t measurement_start_ms;
    uint32_t last_measure_duration_ms;

} LidarData_t;

void Lidar_Init(void);
void Lidar_Update(void);

uint8_t Lidar_IsConnected(void);
uint8_t Lidar_IsDataReady(void);
uint8_t Lidar_IsDistanceValid(void);

uint16_t Lidar_GetDistanceCm(void);
float Lidar_GetDistanceM(void);
float Lidar_GetMedianDistanceM(void);
float Lidar_GetFilteredDistanceM(void);

LidarData_t Lidar_GetData(void);
const LidarData_t *Lidar_GetDataPtr(void);


/* Garmin LIDAR-Lite v3 balanced-profile diagnostics (V8.12B). */
extern volatile uint8_t lidar_profile_config_ok;
extern volatile uint8_t lidar_profile_sig_count_value;
extern volatile uint8_t lidar_profile_acq_config_value;
extern volatile uint8_t lidar_profile_threshold_value;

extern volatile uint32_t lidar_bias_command_count;
extern volatile uint32_t lidar_no_bias_command_count;

extern volatile uint32_t lidar_status_poll_start_count;
extern volatile uint32_t lidar_status_poll_complete_count;
extern volatile uint32_t lidar_status_busy_count;
extern volatile uint8_t lidar_last_status_reg;

extern volatile uint32_t lidar_last_trigger_interval_us;
extern volatile uint32_t lidar_min_trigger_interval_us;
extern volatile uint32_t lidar_max_trigger_interval_us;
extern volatile uint32_t lidar_sample_pending_overrun_count;

#endif
