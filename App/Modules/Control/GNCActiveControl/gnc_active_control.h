#ifndef APP_MODULES_CONTROL_GNC_ACTIVE_CONTROL_H
#define APP_MODULES_CONTROL_GNC_ACTIVE_CONTROL_H

#include <stdint.h>

typedef enum
{
    GNC_ACTIVE_FAULT_NONE = 0,
    GNC_ACTIVE_FAULT_CONFIG = 1,
    GNC_ACTIVE_FAULT_ESTIMATOR = 2,
    GNC_ACTIVE_FAULT_VERTICAL_STATE = 3,
    GNC_ACTIVE_FAULT_NEEDLE = 4,
    GNC_ACTIVE_FAULT_ATTITUDE_CONTROL = 5,
    GNC_ACTIVE_FAULT_COMMAND_REJECTED = 6
} GNCActiveFault_t;

void GNCActiveControl_Init(void);

/* Called from the 1 kHz IMU/ESKF propagation task. */
void GNCActiveControl_Service1kHz(void);

/* Called after the 200 Hz Full-State ESKF correction. */
void GNCActiveControl_Update200Hz(void);

/* Called from App_Run(); owns USER button PA0 in V8.16. */
void GNCActiveControl_MainLoop(void);

/* Immediate software disarm / safe demand. */
void GNCActiveControl_ForceSafe(void);

/* Reliable module globals for telemetry/fixed-address diagnostic packing. */
extern volatile uint32_t gnc_v816_magic;

extern volatile uint8_t gnc_v816_button_stage;
extern volatile uint32_t gnc_v816_button_press_count;

extern volatile uint8_t gnc_v816_armed;
extern volatile uint8_t gnc_v816_disarm_closing;
extern volatile uint8_t gnc_v816_fault_latched;
extern volatile uint8_t gnc_v816_fault_code;

extern volatile uint8_t gnc_v816_estimator_ok;
extern volatile uint8_t gnc_v816_vertical_state_ok;
extern volatile uint8_t gnc_v816_attitude_state_ok;
extern volatile uint8_t gnc_v816_needle_ok;

extern volatile float gnc_v816_height_agl_m;
extern volatile float gnc_v816_z_cg_m;
extern volatile float gnc_v816_vertical_velocity_mps;

extern volatile float gnc_v816_roll_deg;
extern volatile float gnc_v816_pitch_deg;
extern volatile float gnc_v816_yaw_deg;
extern volatile float gnc_v816_roll_rate_dps;
extern volatile float gnc_v816_pitch_rate_dps;

extern volatile float gnc_v816_vertical_valve_cmd;
extern volatile float gnc_v816_vertical_reference_speed_mps;
extern volatile float gnc_v816_vertical_speed_error_mps;

extern volatile uint8_t gnc_v816_rcs_requested_mask;
extern volatile uint8_t gnc_v816_rcs_applied_mask;
extern volatile uint8_t gnc_v816_rcs_dry_run;

extern volatile uint32_t gnc_v816_update_200hz_count;
extern volatile uint32_t gnc_v816_service_1khz_count;
extern volatile uint32_t gnc_v816_soft_invalid_count;
extern volatile uint32_t gnc_v816_fault_count;

#endif
