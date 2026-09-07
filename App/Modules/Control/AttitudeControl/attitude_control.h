#ifndef APP_MODULES_CONTROL_ATTITUDE_CONTROL_H
#define APP_MODULES_CONTROL_ATTITUDE_CONTROL_H

#include <stdint.h>

#include "Modules/Estimation/AttitudeEstimator/attitude_estimator.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"

typedef enum
{
    ATT_CTRL_STATE_DISABLED = 0,
    ATT_CTRL_STATE_WAIT_ESTIMATOR,
    ATT_CTRL_STATE_HOLD,
    ATT_CTRL_STATE_ACTIVE,
    ATT_CTRL_STATE_REVERSAL_DEADTIME,
    ATT_CTRL_STATE_SENSOR_FAULT,
    ATT_CTRL_STATE_COOLDOWN
} AttitudeControlState_t;

typedef enum
{
    ATT_CTRL_FAULT_NONE = 0,
    ATT_CTRL_FAULT_CONFIG,
    ATT_CTRL_FAULT_ESTIMATOR_UNHEALTHY,
    ATT_CTRL_FAULT_ATTITUDE_NONFINITE,
    ATT_CTRL_FAULT_SENSOR_TIMEOUT,
    ATT_CTRL_FAULT_COMMAND_CHATTER,
    ATT_CTRL_FAULT_OUTPUT_INTERLOCK,
    ATT_CTRL_FAULT_EVENT_RATE
} AttitudeControlFault_t;

typedef enum
{
    ATT_CTRL_AXIS_MODE_HOLD = 0,
    ATT_CTRL_AXIS_MODE_CORRECTING,
    ATT_CTRL_AXIS_MODE_BRAKING,
    ATT_CTRL_AXIS_MODE_REVERSAL_DEADTIME,
    ATT_CTRL_AXIS_MODE_SAFE_WAIT,
    ATT_CTRL_AXIS_MODE_COASTING,
    ATT_CTRL_AXIS_MODE_COOLDOWN
} AttitudeControlAxisMode_t;

typedef enum
{
    ATT_CTRL_TORQUE_NEGATIVE = -1,
    ATT_CTRL_TORQUE_NONE = 0,
    ATT_CTRL_TORQUE_POSITIVE = 1
} AttitudeControlTorqueCommand_t;

/* Retained for source/Live-Expression compatibility with the pulse prototype. */
typedef enum
{
    ATT_CTRL_PULSE_NONE = 0,
    ATT_CTRL_PULSE_MINIMUM,
    ATT_CTRL_PULSE_MEDIUM,
    ATT_CTRL_PULSE_MAXIMUM
} AttitudeControlPulseLevel_t;

typedef struct
{
    AttitudeControlState_t state;
    AttitudeControlFault_t fault;

    AttitudeControlAxisMode_t roll_mode;
    AttitudeControlAxisMode_t pitch_mode;
    AttitudeControlTorqueCommand_t roll_torque_command;
    AttitudeControlTorqueCommand_t pitch_torque_command;

    uint8_t config_valid;
    uint8_t estimator_healthy;
    uint8_t attitude_fresh;
    uint8_t landing_prediction_valid;

    uint8_t roll_settled;
    uint8_t pitch_settled;
    uint8_t roll_event_active;
    uint8_t pitch_event_active;
    uint8_t roll_brake_used;
    uint8_t pitch_brake_used;
    uint8_t roll_reversal_deadtime;
    uint8_t pitch_reversal_deadtime;

    uint8_t valve_demand_mask;
    uint8_t valve_applied_mask;
    uint8_t valve_demand;  /* Legacy: any valve requested. */
    uint8_t valve_applied; /* Legacy: any valve physically applied. */
    uint8_t dry_run;

    /* V49 source and signed-PD pulse diagnostics. */
    uint8_t eskf_source_active;
    uint8_t roll_auto_damping;
    uint8_t pitch_auto_damping;
    uint8_t timer_tick_active;

    float landing_height_m;
    float vertical_velocity_mps;
    float time_to_ground_s;

    float roll_deg;
    float pitch_deg;
    float roll_rate_dps;
    float pitch_rate_dps;
    float roll_trend_rate_dps;
    float pitch_trend_rate_dps;

    float roll_predicted_touch_angle_deg;
    float pitch_predicted_touch_angle_deg;
    float roll_target_angle_deg;
    float pitch_target_angle_deg;

    float roll_stopping_angle_deg;
    float pitch_stopping_angle_deg;
    float roll_braking_distance_deg;
    float pitch_braking_distance_deg;
    float roll_switch_error_deg;
    float pitch_switch_error_deg;

    float roll_pd_time_signed_ms;
    float pitch_pd_time_signed_ms;

    uint32_t roll_commanded_pulse_ms;
    uint32_t pitch_commanded_pulse_ms;
    uint32_t roll_pulse_remaining_ms;
    uint32_t pitch_pulse_remaining_ms;
    uint32_t roll_cooldown_remaining_ms;
    uint32_t pitch_cooldown_remaining_ms;
    uint32_t roll_subminimum_reject_count;
    uint32_t pitch_subminimum_reject_count;
    uint32_t roll_saturation_count;
    uint32_t pitch_saturation_count;
    uint32_t roll_cooldown_skip_count;
    uint32_t pitch_cooldown_skip_count;

    /* Legacy single-axis diagnostic aliases (roll). */
    float selected_control_angle_deg;
    float selected_control_rate_dps;
    float angle_trend_rate_dps;

    /* Pulse diagnostics; roll is retained as the legacy single-axis alias. */
    AttitudeControlPulseLevel_t pulse_level;
    uint8_t pulse_requested;
    uint8_t pulse_phase_on;
    uint32_t pulse_width_ms;
    uint32_t active_pulse_width_ms;
    uint32_t pulse_cycle_start_ms;
    uint32_t pulse_count;

    uint32_t init_ms;
    uint32_t last_attitude_ms;
    uint32_t control_update_count;
    uint32_t safety_service_count;
    uint32_t command_change_count;
    uint32_t reversal_count;
    uint32_t event_count;
} AttitudeControlStatus_t;

void AttitudeControl_Init(uint32_t now_ms);
void AttitudeControl_NotifyEstimatorReset(uint32_t now_ms);

/*
 * V49 physical path.  Update is called for every fresh Full-State ESKF public
 * output (200 Hz).  Service is called from the 1 kHz IMU task for fail-safe
 * freshness checks.  The independent TIM7 ISR owns exact pulse countdown and
 * closes the relays without blocking any scheduler task.
 */
void AttitudeControl_UpdateESKF(
    const FullStateESKFData_t *eskf,
    const SensorData_t *sensor,
    uint32_t now_ms
);
void AttitudeControl_ServiceESKF(
    uint8_t estimator_healthy,
    uint32_t now_ms
);
void AttitudeControl_TimerTickISR(void);
void AttitudeControl_ForceSafe(void);
/* R8R35: retire legacy V49 internal pulse state without touching the physical
 * SolenoidOutput mask owned by TaragayFlightLogic/RCS V7.13.4. */
void AttitudeControl_ForceSafeNoOutput(void);

/* Legacy wrappers retained so older diagnostic modules still compile. */
void AttitudeControl_Service(
    const AttitudeEstimatorData_t *attitude,
    uint8_t estimator_healthy,
    uint32_t now_ms
);

/*
 * Called for each new Euler sample (~100 Hz).
 * Landing height is the LIDAR distance to the ground. Vertical velocity is
 * positive upward / negative downward. Prediction is used only when the
 * supplied landing state is valid and the vehicle is descending.
 */
void AttitudeControl_Update(
    const AttitudeEstimatorData_t *attitude,
    uint8_t estimator_healthy,
    float roll_rate_dps,
    float pitch_rate_dps,
    float landing_height_m,
    float vertical_velocity_mps,
    uint8_t landing_state_valid,
    uint32_t now_ms
);

AttitudeControlStatus_t AttitudeControl_GetStatus(void);
const char *AttitudeControl_StateName(AttitudeControlState_t state);
const char *AttitudeControl_AxisModeName(AttitudeControlAxisMode_t mode);

#endif
