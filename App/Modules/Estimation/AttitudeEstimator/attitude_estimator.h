#ifndef APP_MODULES_ESTIMATION_ATTITUDE_ESTIMATOR_H
#define APP_MODULES_ESTIMATION_ATTITUDE_ESTIMATOR_H

#include <stdint.h>

typedef struct
{
    uint8_t enabled;
    uint8_t initialized;
    uint8_t healthy;
    uint8_t accel_correction_active;

    /* Quaternion rotating body-frame vectors into the world ENU frame. */
    float q_w;
    float q_x;
    float q_y;
    float q_z;

    /* Relative attitude. Yaw starts at zero and drifts without a heading aid. */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    /* Accelerometer specific force after body-to-world rotation. */
    float world_specific_force_x_g;
    float world_specific_force_y_g;
    float world_specific_force_z_g;

    /* Gravity-compensated world-frame linear acceleration. */
    float world_linear_accel_x_mps2;
    float world_linear_accel_y_mps2;
    float world_linear_accel_z_mps2;

    /* 0.0 = accelerometer correction disabled, 1.0 = full correction. */
    float accel_correction_weight;

    /* Integral feedback behaves as a slow online gyro-bias correction. */
    float integral_feedback_x_rad_s;
    float integral_feedback_y_rad_s;
    float integral_feedback_z_rad_s;

    uint32_t update_count;
    /* Increments exactly when roll_deg/pitch_deg/yaw_deg are recalculated. */
    uint32_t euler_update_count;
    uint32_t duplicate_skip_count;
    uint32_t accel_correction_count;
    uint32_t accel_reject_count;
    uint32_t reset_count;
    uint32_t numerical_error_count;
    uint32_t gap_reset_count;

    uint32_t last_sensor_update_count;
    uint32_t last_timestamp_us;
    uint32_t last_dt_us;
    uint32_t max_dt_us;

} AttitudeEstimatorData_t;

void AttitudeEstimator_Init(void);
void AttitudeEstimator_Update(void);

AttitudeEstimatorData_t AttitudeEstimator_GetData(void);
const AttitudeEstimatorData_t *AttitudeEstimator_GetDataPtr(void);
uint8_t AttitudeEstimator_IsHealthy(void);

#endif
