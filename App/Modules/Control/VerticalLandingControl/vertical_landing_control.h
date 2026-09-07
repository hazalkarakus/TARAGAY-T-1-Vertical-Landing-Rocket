#ifndef APP_MODULES_CONTROL_VERTICAL_LANDING_CONTROL_H
#define APP_MODULES_CONTROL_VERTICAL_LANDING_CONTROL_H

#include <stdint.h>

typedef struct
{
    uint8_t input_valid;

    float valve_cmd;

    float height_above_touchdown_m;
    float downward_speed_mps;
    float reference_downward_speed_mps;
    float speed_error_mps;

    float available_force_n;
    float commanded_accel_mps2;
    float required_force_n;

} VerticalLandingControlOutput_t;

void VerticalLandingControl_Init(void);

VerticalLandingControlOutput_t VerticalLandingControl_Update(
    float z_cg_m,
    float vertical_velocity_mps,
    float mass_kg,
    float main_pressure_bar
);

#endif
