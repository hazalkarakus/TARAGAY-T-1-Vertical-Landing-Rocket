#ifndef APP_MODULES_CONTROL_GENERATED_FLIGHT_CONTROL_H
#define APP_MODULES_CONTROL_GENERATED_FLIGHT_CONTROL_H

#include <stdint.h>

typedef struct
{
    float pitch_deg;
    float yaw_deg;
    float x_m;
    float y_m;
    float vx_mps;
    float vy_mps;
    float z_cg_m;
    float vz_mps;
    float z_imu_m;
    float mass_kg;
    float main_pressure_bar;
    float measured_thrust_n;
    float pitch_rate_rad_s;
    float yaw_rate_rad_s;
    uint8_t vertical_valid;
    uint8_t horizontal_valid;
    uint8_t attitude_valid;
} GeneratedFlightControlInput_t;

typedef struct
{
    float rcs_v1;
    float rcs_v3;
    float rcs_v5;
    float rcs_v7;
    float main_valve_cmd;
    float vertical_200hz_valve_cmd;
    float suicide_burn_height_m;
    float vertical_target_force_n;
    float vertical_feedforward_n;
    float vertical_pi_integral_n;
    float vertical_estimated_thrust_n;
    uint8_t suicide_burn_active;
    uint8_t main_output_valid;
    uint8_t rcs_output_valid;
    uint8_t compute_only;
    uint8_t pressure_model_source;
    uint8_t thrust_eskf_estimate_source;
    uint8_t initialized;
    uint8_t vertical_sensor_source_mask;
    uint8_t vertical_sensor_degraded;
    uint8_t lidar_aiding_active;
    uint8_t baro_aiding_active;
    uint32_t step_count;
    uint32_t invalid_input_count;
    uint32_t model_reset_count;
    uint32_t vertical_sensor_transition_count;
    uint32_t vertical_dual_source_count;
    uint32_t vertical_lidar_only_count;
    uint32_t vertical_baro_only_count;
    uint32_t vertical_no_aiding_count;
} GeneratedFlightControlStatus_t;

void GeneratedFlightControl_Init(void);
void GeneratedFlightControl_Step100Hz(const GeneratedFlightControlInput_t *input);
void GeneratedFlightControl_HoldSafe(void);

/* Called after the 200 Hz ESKF correction. The slide-compatible vertical
 * outer loop runs every call; the archived Simulink base model stays 100 Hz. */
void GeneratedFlightControl_Service200Hz(void);

GeneratedFlightControlStatus_t GeneratedFlightControl_GetStatus(void);

/* Global symbol is intentionally visible for CubeIDE Live Expressions. */
extern GeneratedFlightControlStatus_t generated_fc_status;

#endif
