#ifndef APP_MODULES_ESTIMATION_VERTICAL_EKF_H
#define APP_MODULES_ESTIMATION_VERTICAL_EKF_H

#include <stdint.h>

typedef struct
{
    uint8_t enabled;
    uint8_t initialized;
    uint8_t healthy;

    uint8_t baro_reference_ready;
    uint8_t lidar_reference_ready;
    uint8_t imu_aiding_active;

    float altitude_m;
    float vertical_velocity_mps;

    /* Corrected acceleration used by prediction: input - estimated bias. */
    float vertical_acceleration_mps2;

    /* Quaternion-derived input before EKF bias correction. */
    float imu_accel_input_mps2;

    /* Online estimated residual world-Z acceleration bias. */
    float accel_bias_mps2;

    float baro_reference_m;
    float lidar_reference_m;

    float baro_innovation_m;
    float lidar_innovation_m;

    float covariance_altitude;
    float covariance_velocity;
    float covariance_accel_bias;

    uint32_t predict_count;
    uint32_t baro_update_count;
    uint32_t lidar_update_count;

    uint32_t baro_reject_count;
    uint32_t lidar_reject_count;

    uint32_t imu_aiding_accept_count;
    uint32_t imu_aiding_reject_count;

    uint32_t reset_count;
    uint32_t numerical_error_count;
    uint32_t predict_gap_reset_count;

    uint32_t baro_reference_sample_count;
    uint32_t lidar_reference_sample_count;

    uint32_t last_predict_timestamp_us;
    uint32_t last_correction_timestamp_us;
    uint32_t last_dt_us;
    uint32_t max_dt_us;

} VerticalEKFData_t;

void VerticalEKF_Init(void);
void VerticalEKF_Update(void);
void VerticalEKF_Reset(void);

uint8_t VerticalEKF_IsInitialized(void);
uint8_t VerticalEKF_IsHealthy(void);
VerticalEKFData_t VerticalEKF_GetData(void);

#endif
