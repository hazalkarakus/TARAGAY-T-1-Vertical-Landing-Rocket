#ifndef BAROMETER_H
#define BAROMETER_H

#include <stdint.h>

typedef struct
{
    uint8_t initialized;
    uint8_t connected;
    uint8_t data_ready;
    uint8_t pressure_valid;
    uint8_t calibrated;
    uint8_t healthy;

    uint8_t filter_enabled;
    uint8_t filter_config_ok;
    uint8_t filter_initialized;

    float temperature_c;

    /* Raw pressure from the newest complete D1 + D2 measurement. */
    float pressure_pa;

    /* Median of the newest three accepted pressure samples. */
    float median_pressure_pa;

    /* Median-3 followed by 2nd-order Butterworth pressure output. */
    float filtered_pressure_pa;

    float ground_pressure_pa;

    /* P53 preflight datum tracking diagnostics. */
    uint8_t ground_reference_tracking_allowed;
    uint8_t ground_reference_tracking_active;
    uint8_t ground_reference_frozen;
    uint8_t ground_reference_reserved;
    float ground_reference_error_pa;
    float ground_reference_last_step_pa;
    uint32_t ground_reference_update_count;
    uint32_t ground_reference_freeze_count;
    uint32_t ground_reference_last_update_us;

    /* Raw altitude from raw pressure. */
    float altitude_m;

    /* Altitude calculated from filtered pressure. */
    float filtered_altitude_m;

    /* 2nd-order Butterworth filtered vertical speed. */
    float vertical_speed_mps;

    uint32_t d1_raw;
    uint32_t d2_raw;

    /* Accepted fresh BMP585 samples; repeated raw P/T pairs are excluded. */
    uint32_t update_count;
    uint32_t source_update_count;
    uint32_t valid_sample_count;
    uint32_t invalid_sample_count;
    uint32_t calibration_sample_count;

    uint32_t duplicate_sample_skip_count;

    uint32_t median_update_count;
    uint32_t median_rejected_spike_count;

    /* P39 raw-pressure plausibility guard diagnostics. */
    uint32_t raw_spike_reject_count;
    uint32_t raw_step_confirm_count;

    uint32_t filter_update_count;
    uint32_t filter_reset_count;

    uint32_t last_sample_timestamp_us;
    uint32_t last_sample_interval_us;
    uint32_t min_sample_interval_us;
    uint32_t max_sample_interval_us;

    uint32_t last_measure_duration_us;
    uint32_t max_measure_duration_us;

} BarometerData_t;

void Barometer_Init(void);
void Barometer_Update(void);

/* P53: app-level preflight gate. Tracking can be allowed only while the
 * estimator says the vehicle is stationary. Freeze is one-way until reset. */
void Barometer_SetGroundReferenceTrackingAllowed(uint8_t allowed);
void Barometer_FreezeGroundReference(void);

uint8_t Barometer_IsConnected(void);
uint8_t Barometer_IsDataReady(void);
uint8_t Barometer_IsPressureValid(void);
uint8_t Barometer_IsCalibrated(void);
uint8_t Barometer_IsHealthy(void);

BarometerData_t Barometer_GetData(void);
const BarometerData_t *Barometer_GetDataPtr(void);

#endif
