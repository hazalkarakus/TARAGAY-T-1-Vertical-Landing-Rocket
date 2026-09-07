#ifndef APP_MODULES_SENSORS_SENSOR_MANAGER_H
#define APP_MODULES_SENSORS_SENSOR_MANAGER_H

#include <stdint.h>

#define SENSOR_MANAGER_HEADER_V2    1

typedef struct
{
    /* Genel bilgiler */
    uint32_t timestamp_us;
    uint32_t imu_sample_timestamp_us;
    uint32_t update_count;

    /* Sensör sağlık bilgileri */
    uint8_t imu_valid;

    uint8_t baro_valid;
    uint8_t baro_data_ready;
    uint8_t baro_pressure_valid;
    uint8_t baro_calibrated;

    /* IMU ham verileri */
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;

    /* IMU ölçeklenmiş verileri */
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float accel_norm_g;

    /*
     * 2nd-order Butterworth filtered IMU values.
     * Raw and unfiltered scaled values above are preserved for diagnostics.
     */
    float gyro_x_filtered_dps;
    float gyro_y_filtered_dps;
    float gyro_z_filtered_dps;

    float accel_x_filtered_g;
    float accel_y_filtered_g;
    float accel_z_filtered_g;

    float accel_filtered_norm_g;

    /* Barometre verileri */
    float baro_temperature_c;

    float baro_pressure_pa;
    float baro_filtered_pressure_pa;
    float baro_ground_pressure_pa;

    float baro_altitude_m;
    float baro_filtered_altitude_m;
    float baro_vertical_speed_mps;

    uint32_t baro_d1_raw;
    uint32_t baro_d2_raw;

    uint32_t baro_update_count;
    uint32_t baro_valid_sample_count;
    uint32_t baro_invalid_sample_count;
    uint32_t baro_calibration_sample_count;

} SensorData_t;

void SensorManager_Init(void);

void SensorManager_UpdateIMU(void);
void SensorManager_UpdateBarometer(void);
void SensorManager_ServiceFreshness(void);

SensorData_t SensorManager_GetData(void);
const SensorData_t *SensorManager_GetDataPtr(void);

uint8_t SensorManager_IsIMUValid(void);
uint8_t SensorManager_IsBarometerValid(void);

uint32_t SensorManager_GetUpdateCount(void);

#endif
