#include "Modules/Control/AttitudeControl/attitude_control.h"

#include "Common/app_config.h"
#include "Services/SolenoidOutput/solenoid_output.h"
#include "Services/SystemMonitor/system_monitor.h"
#include "main.h"

#include <stdint.h>

/*
 * V49 ESKF signed-PD RCS controller
 * ---------------------------------
 * - Fresh Full-State ESKF roll/pitch is consumed at 200 Hz.
 * - Bias-corrected filtered gyro rate supplies the derivative term.
 * - The PD result is a signed valve-open time, never PWM duty.
 * - A result below 20 ms is rejected; results above 60 ms are saturated.
 * - TIM7 decrements independent roll/pitch pulse and cooldown timers at 1 kHz.
 * - The sign may reverse before the angle reaches zero, providing active
 *   angular-rate damping without a separate blocking brake sequence.
 */

typedef struct
{
    volatile int8_t active_error_sign;
    volatile uint32_t pulse_remaining_ms;
    volatile uint32_t cooldown_remaining_ms;

    float pd_time_signed_ms;
    uint32_t commanded_pulse_ms;
    uint8_t auto_damping;

    uint32_t pulse_count;
    uint32_t subminimum_reject_count;
    uint32_t saturation_count;
    uint32_t cooldown_skip_count;
} AttitudeControlAxisRuntime_t;

static AttitudeControlStatus_t control_status;
static AttitudeControlAxisRuntime_t roll_axis;
static AttitudeControlAxisRuntime_t pitch_axis;

static uint8_t has_eskf_sample;
static uint8_t hard_fault_latched;
static uint32_t last_eskf_sample_ms;

static uint8_t AttitudeControl_IsFinite(float value)
{
    return ((value == value) &&
            (value <= 1000000.0f) &&
            (value >= -1000000.0f)) ? 1U : 0U;
}

static float AttitudeControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int8_t AttitudeControl_Sign(float value)
{
    if (value > 0.0f)
    {
        return 1;
    }

    if (value < 0.0f)
    {
        return -1;
    }

    return 0;
}

static uint32_t AttitudeControl_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void AttitudeControl_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void AttitudeControl_ClearBytes(void *object, uint32_t size)
{
    uint8_t *bytes = (uint8_t *)object;
    uint32_t index;

    for (index = 0UL; index < size; index++)
    {
        bytes[index] = 0U;
    }
}

static uint8_t AttitudeControl_ConfigValid(void)
{
    if ((ATT_CTRL_ENABLE > 1U) ||
        (ATT_CTRL_DRY_RUN > 1U) ||
        (ATT_CTRL_ROLL_ENABLE > 1U) ||
        (ATT_CTRL_PITCH_ENABLE > 1U) ||
        (APP_RCS_ESKF_SIGNED_PD_ENABLED != 1U))
    {
        return 0U;
    }

    if (((ATT_CTRL_ROLL_ANGLE_SIGN != 1) &&
         (ATT_CTRL_ROLL_ANGLE_SIGN != -1)) ||
        ((ATT_CTRL_ROLL_RATE_SIGN != 1) &&
         (ATT_CTRL_ROLL_RATE_SIGN != -1)) ||
        ((ATT_CTRL_PITCH_ANGLE_SIGN != 1) &&
         (ATT_CTRL_PITCH_ANGLE_SIGN != -1)) ||
        ((ATT_CTRL_PITCH_RATE_SIGN != 1) &&
         (ATT_CTRL_PITCH_RATE_SIGN != -1)))
    {
        return 0U;
    }

    if ((AttitudeControl_IsFinite(ATT_CTRL_PD_KP_MS_PER_DEG) == 0U) ||
        (AttitudeControl_IsFinite(ATT_CTRL_PD_KD_MS_PER_DPS) == 0U) ||
        (ATT_CTRL_PD_KP_MS_PER_DEG <= 0.0f) ||
        (ATT_CTRL_PD_KD_MS_PER_DPS <= 0.0f) ||
        (ATT_CTRL_MIN_CORRECTION_TIME_MS == 0UL) ||
        (ATT_CTRL_MAX_CORRECTION_TIME_MS <
         ATT_CTRL_MIN_CORRECTION_TIME_MS) ||
        (ATT_CTRL_COOLDOWN_MS == 0UL) ||
        (ATT_CTRL_TIMER_TICK_MS != 1UL) ||
        (ATT_CTRL_EXPECTED_UPDATE_MS != 5UL) ||
        (ATT_CTRL_SENSOR_TIMEOUT_MS == 0UL))
    {
        return 0U;
    }

    return 1U;
}

static void AttitudeControl_AbortAxis(
    AttitudeControlAxisRuntime_t *axis,
    uint8_t clear_cooldown
)
{
    axis->active_error_sign = 0;
    axis->pulse_remaining_ms = 0UL;
    axis->commanded_pulse_ms = 0UL;
    axis->auto_damping = 0U;
    axis->pd_time_signed_ms = 0.0f;

    if (clear_cooldown != 0U)
    {
        axis->cooldown_remaining_ms = 0UL;
    }
}

static AttitudeControlTorqueCommand_t AttitudeControl_TorqueFromErrorSign(
    int8_t error_sign
)
{
    /* A positive attitude error requires the negative correcting torque. */
    if (error_sign > 0)
    {
        return ATT_CTRL_TORQUE_NEGATIVE;
    }

    if (error_sign < 0)
    {
        return ATT_CTRL_TORQUE_POSITIVE;
    }

    return ATT_CTRL_TORQUE_NONE;
}

static uint8_t AttitudeControl_BuildValveMask(void)
{
    uint8_t mask = SOLENOID_VALVE_NONE;

    if (ATT_CTRL_ROLL_ENABLE != 0U)
    {
        if (roll_axis.active_error_sign > 0)
        {
            mask |= SOLENOID_VALVE_X_POS_ERROR;
        }
        else if (roll_axis.active_error_sign < 0)
        {
            mask |= SOLENOID_VALVE_X_NEG_ERROR;
        }
    }

    if (ATT_CTRL_PITCH_ENABLE != 0U)
    {
        if (pitch_axis.active_error_sign > 0)
        {
            mask |= SOLENOID_VALVE_Y_POS_ERROR;
        }
        else if (pitch_axis.active_error_sign < 0)
        {
            mask |= SOLENOID_VALVE_Y_NEG_ERROR;
        }
    }

    return mask;
}

static void AttitudeControl_SyncOutputStatus(void)
{
    SolenoidOutputStatus_t output = SolenoidOutput_GetStatus();

    control_status.valve_demand_mask = output.requested_mask;
    control_status.valve_applied_mask = output.applied_mask;
    control_status.valve_demand = output.requested_open;
    control_status.valve_applied = output.applied_open;
    control_status.dry_run = output.dry_run;
}

static AttitudeControlAxisMode_t AttitudeControl_AxisMode(
    const AttitudeControlAxisRuntime_t *axis
)
{
    if (axis->active_error_sign != 0)
    {
        return (axis->auto_damping != 0U)
            ? ATT_CTRL_AXIS_MODE_BRAKING
            : ATT_CTRL_AXIS_MODE_CORRECTING;
    }

    if (axis->cooldown_remaining_ms > 0UL)
    {
        return ATT_CTRL_AXIS_MODE_COOLDOWN;
    }

    return ATT_CTRL_AXIS_MODE_HOLD;
}

static void AttitudeControl_UpdateRuntimeStatus(void)
{
    control_status.roll_mode = AttitudeControl_AxisMode(&roll_axis);
    control_status.pitch_mode = AttitudeControl_AxisMode(&pitch_axis);

    control_status.roll_torque_command =
        AttitudeControl_TorqueFromErrorSign(roll_axis.active_error_sign);
    control_status.pitch_torque_command =
        AttitudeControl_TorqueFromErrorSign(pitch_axis.active_error_sign);

    control_status.roll_event_active =
        (roll_axis.active_error_sign != 0) ? 1U : 0U;
    control_status.pitch_event_active =
        (pitch_axis.active_error_sign != 0) ? 1U : 0U;
    control_status.roll_reversal_deadtime =
        (roll_axis.cooldown_remaining_ms > 0UL) ? 1U : 0U;
    control_status.pitch_reversal_deadtime =
        (pitch_axis.cooldown_remaining_ms > 0UL) ? 1U : 0U;
    control_status.roll_auto_damping = roll_axis.auto_damping;
    control_status.pitch_auto_damping = pitch_axis.auto_damping;

    control_status.roll_pd_time_signed_ms = roll_axis.pd_time_signed_ms;
    control_status.pitch_pd_time_signed_ms = pitch_axis.pd_time_signed_ms;
    control_status.roll_commanded_pulse_ms = roll_axis.commanded_pulse_ms;
    control_status.pitch_commanded_pulse_ms = pitch_axis.commanded_pulse_ms;
    control_status.roll_pulse_remaining_ms =
        roll_axis.pulse_remaining_ms;
    control_status.pitch_pulse_remaining_ms =
        pitch_axis.pulse_remaining_ms;
    control_status.roll_cooldown_remaining_ms =
        roll_axis.cooldown_remaining_ms;
    control_status.pitch_cooldown_remaining_ms =
        pitch_axis.cooldown_remaining_ms;
    control_status.roll_subminimum_reject_count =
        roll_axis.subminimum_reject_count;
    control_status.pitch_subminimum_reject_count =
        pitch_axis.subminimum_reject_count;
    control_status.roll_saturation_count = roll_axis.saturation_count;
    control_status.pitch_saturation_count = pitch_axis.saturation_count;
    control_status.roll_cooldown_skip_count = roll_axis.cooldown_skip_count;
    control_status.pitch_cooldown_skip_count = pitch_axis.cooldown_skip_count;

    control_status.roll_brake_used = roll_axis.auto_damping;
    control_status.pitch_brake_used = pitch_axis.auto_damping;
    control_status.roll_settled =
        ((roll_axis.active_error_sign == 0) &&
         (roll_axis.cooldown_remaining_ms == 0UL)) ? 1U : 0U;
    control_status.pitch_settled =
        ((pitch_axis.active_error_sign == 0) &&
         (pitch_axis.cooldown_remaining_ms == 0UL)) ? 1U : 0U;

    control_status.pulse_width_ms = roll_axis.commanded_pulse_ms;
    control_status.active_pulse_width_ms =
        (roll_axis.active_error_sign != 0)
        ? roll_axis.commanded_pulse_ms
        : pitch_axis.commanded_pulse_ms;
    control_status.pulse_requested =
        ((roll_axis.active_error_sign != 0) ||
         (pitch_axis.active_error_sign != 0)) ? 1U : 0U;
    control_status.pulse_phase_on = control_status.pulse_requested;

    if (control_status.fault != ATT_CTRL_FAULT_NONE)
    {
        control_status.state = ATT_CTRL_STATE_SENSOR_FAULT;
    }
    else if ((roll_axis.active_error_sign != 0) ||
             (pitch_axis.active_error_sign != 0))
    {
        control_status.state = ATT_CTRL_STATE_ACTIVE;
    }
    else if ((roll_axis.cooldown_remaining_ms > 0UL) ||
             (pitch_axis.cooldown_remaining_ms > 0UL))
    {
        control_status.state = ATT_CTRL_STATE_COOLDOWN;
    }
    else
    {
        control_status.state = ATT_CTRL_STATE_HOLD;
    }

    AttitudeControl_SyncOutputStatus();
}

static void AttitudeControl_ApplyMaskLocked(void)
{
    SolenoidOutputStatus_t output;

    if ((ATT_CTRL_ENABLE == 0U) ||
        (control_status.config_valid == 0U) ||
        (hard_fault_latched != 0U))
    {
        SolenoidOutput_ForceSafe();
        AttitudeControl_SyncOutputStatus();
        return;
    }

    SolenoidOutput_SetMask(AttitudeControl_BuildValveMask());
    output = SolenoidOutput_GetStatus();

    if (output.interlock_fault != 0U)
    {
        hard_fault_latched = 1U;
        control_status.fault = ATT_CTRL_FAULT_OUTPUT_INTERLOCK;
        AttitudeControl_AbortAxis(&roll_axis, 1U);
        AttitudeControl_AbortAxis(&pitch_axis, 1U);
        SolenoidOutput_ForceSafe();
    }

    AttitudeControl_SyncOutputStatus();
}

static void AttitudeControl_EnterFaultLocked(
    AttitudeControlFault_t fault,
    uint8_t hard_latch
)
{
    control_status.fault = fault;
    control_status.state = ATT_CTRL_STATE_SENSOR_FAULT;

    if (hard_latch != 0U)
    {
        hard_fault_latched = 1U;
    }

    AttitudeControl_AbortAxis(&roll_axis, 1U);
    AttitudeControl_AbortAxis(&pitch_axis, 1U);
    SolenoidOutput_ForceSafe();
    AttitudeControl_UpdateRuntimeStatus();
}

static uint8_t AttitudeControl_IsAutoDamping(
    float angle_deg,
    float rate_dps,
    int8_t demand_sign
)
{
    int8_t angle_sign = AttitudeControl_Sign(angle_deg);

    if ((angle_sign != 0) && (demand_sign != angle_sign))
    {
        return 1U;
    }

    if ((AttitudeControl_Abs(angle_deg) < 0.25f) &&
        (AttitudeControl_Abs(rate_dps) > 0.50f))
    {
        return 1U;
    }

    return 0U;
}

static uint8_t AttitudeControl_TryStartPulse(
    AttitudeControlAxisRuntime_t *axis,
    float angle_deg,
    float rate_dps
)
{
    float magnitude_ms;
    uint32_t pulse_ms;
    int8_t demand_sign;

    axis->pd_time_signed_ms =
        (ATT_CTRL_PD_KP_MS_PER_DEG * angle_deg) +
        (ATT_CTRL_PD_KD_MS_PER_DPS * rate_dps);

    magnitude_ms = AttitudeControl_Abs(axis->pd_time_signed_ms);

    if (axis->active_error_sign != 0)
    {
        return 0U;
    }

    if (axis->cooldown_remaining_ms > 0UL)
    {
        if (magnitude_ms >= (float)ATT_CTRL_MIN_CORRECTION_TIME_MS)
        {
            axis->cooldown_skip_count++;
        }
        return 0U;
    }

    if (magnitude_ms < (float)ATT_CTRL_MIN_CORRECTION_TIME_MS)
    {
        axis->commanded_pulse_ms = 0UL;
        axis->auto_damping = 0U;
        axis->subminimum_reject_count++;
        return 0U;
    }

    if (magnitude_ms > (float)ATT_CTRL_MAX_CORRECTION_TIME_MS)
    {
        magnitude_ms = (float)ATT_CTRL_MAX_CORRECTION_TIME_MS;
        axis->saturation_count++;
    }

    pulse_ms = (uint32_t)(magnitude_ms + 0.5f);
    if (pulse_ms < ATT_CTRL_MIN_CORRECTION_TIME_MS)
    {
        pulse_ms = ATT_CTRL_MIN_CORRECTION_TIME_MS;
    }
    if (pulse_ms > ATT_CTRL_MAX_CORRECTION_TIME_MS)
    {
        pulse_ms = ATT_CTRL_MAX_CORRECTION_TIME_MS;
    }

    demand_sign = AttitudeControl_Sign(axis->pd_time_signed_ms);
    if (demand_sign == 0)
    {
        return 0U;
    }

    axis->active_error_sign = demand_sign;
    axis->pulse_remaining_ms = pulse_ms;
    axis->commanded_pulse_ms = pulse_ms;
    axis->auto_damping =
        AttitudeControl_IsAutoDamping(angle_deg, rate_dps, demand_sign);
    axis->pulse_count++;
    return 1U;
}

void AttitudeControl_Init(uint32_t now_ms)
{
    AttitudeControl_ClearBytes(&control_status,
                               (uint32_t)sizeof(control_status));
    AttitudeControl_ClearBytes(&roll_axis, (uint32_t)sizeof(roll_axis));
    AttitudeControl_ClearBytes(&pitch_axis, (uint32_t)sizeof(pitch_axis));

    has_eskf_sample = 0U;
    hard_fault_latched = 0U;
    last_eskf_sample_ms = now_ms;

    control_status.init_ms = now_ms;
    control_status.state = ATT_CTRL_STATE_WAIT_ESTIMATOR;
    control_status.config_valid = AttitudeControl_ConfigValid();
    control_status.dry_run = (ATT_CTRL_DRY_RUN != 0U) ? 1U : 0U;
    control_status.eskf_source_active = 1U;
    control_status.timer_tick_active = 1U;
    control_status.landing_prediction_valid = 0U;

    if (control_status.config_valid == 0U)
    {
        hard_fault_latched = 1U;
        control_status.fault = ATT_CTRL_FAULT_CONFIG;
        control_status.state = ATT_CTRL_STATE_SENSOR_FAULT;
    }

    SolenoidOutput_ForceSafe();
    AttitudeControl_UpdateRuntimeStatus();
    if (control_status.config_valid != 0U)
    {
        control_status.state = ATT_CTRL_STATE_WAIT_ESTIMATOR;
    }
}

void AttitudeControl_NotifyEstimatorReset(uint32_t now_ms)
{
    uint32_t primask = AttitudeControl_EnterCritical();

    AttitudeControl_AbortAxis(&roll_axis, 1U);
    AttitudeControl_AbortAxis(&pitch_axis, 1U);
    SolenoidOutput_ForceSafe();

    has_eskf_sample = 0U;
    last_eskf_sample_ms = now_ms;
    control_status.init_ms = now_ms;
    control_status.last_attitude_ms = now_ms;
    control_status.attitude_fresh = 0U;
    control_status.estimator_healthy = 0U;

    if (AttitudeControl_ConfigValid() != 0U)
    {
        hard_fault_latched = 0U;
        control_status.config_valid = 1U;
        control_status.fault = ATT_CTRL_FAULT_NONE;
        control_status.state = ATT_CTRL_STATE_WAIT_ESTIMATOR;
    }
    else
    {
        hard_fault_latched = 1U;
        control_status.config_valid = 0U;
        control_status.fault = ATT_CTRL_FAULT_CONFIG;
        control_status.state = ATT_CTRL_STATE_SENSOR_FAULT;
    }

    AttitudeControl_UpdateRuntimeStatus();
    if (control_status.fault == ATT_CTRL_FAULT_NONE)
    {
        control_status.state = ATT_CTRL_STATE_WAIT_ESTIMATOR;
    }
    AttitudeControl_ExitCritical(primask);
}

void AttitudeControl_ForceSafe(void)
{
    uint32_t primask = AttitudeControl_EnterCritical();

    AttitudeControl_AbortAxis(&roll_axis, 1U);
    AttitudeControl_AbortAxis(&pitch_axis, 1U);
    SolenoidOutput_ForceSafe();

    if ((hard_fault_latched == 0U) &&
        (control_status.config_valid != 0U))
    {
        control_status.state = ATT_CTRL_STATE_HOLD;
    }

    AttitudeControl_UpdateRuntimeStatus();
    AttitudeControl_ExitCritical(primask);
}

void AttitudeControl_ForceSafeNoOutput(void)
{
    uint32_t primask = AttitudeControl_EnterCritical();

    /* R8R35: V49 is a retired physical authority. Reset only its internal
     * axis/pulse bookkeeping; DO NOT erase the RCS V7.13.4 physical mask. */
    AttitudeControl_AbortAxis(&roll_axis, 1U);
    AttitudeControl_AbortAxis(&pitch_axis, 1U);

    if ((hard_fault_latched == 0U) &&
        (control_status.config_valid != 0U))
    {
        control_status.state = ATT_CTRL_STATE_HOLD;
    }

    AttitudeControl_UpdateRuntimeStatus();
    AttitudeControl_ExitCritical(primask);
}

void AttitudeControl_ServiceESKF(
    uint8_t estimator_healthy,
    uint32_t now_ms
)
{
    uint32_t primask = AttitudeControl_EnterCritical();
    SolenoidOutputStatus_t output;

    control_status.safety_service_count++;
    control_status.estimator_healthy = estimator_healthy;

    if (ATT_CTRL_ENABLE == 0U)
    {
        AttitudeControl_AbortAxis(&roll_axis, 1U);
        AttitudeControl_AbortAxis(&pitch_axis, 1U);
        SolenoidOutput_ForceSafe();
        control_status.state = ATT_CTRL_STATE_DISABLED;
        control_status.fault = ATT_CTRL_FAULT_NONE;
        AttitudeControl_UpdateRuntimeStatus();
        control_status.state = ATT_CTRL_STATE_DISABLED;
        AttitudeControl_ExitCritical(primask);
        return;
    }

    if (control_status.config_valid == 0U)
    {
        AttitudeControl_EnterFaultLocked(ATT_CTRL_FAULT_CONFIG, 1U);
        AttitudeControl_ExitCritical(primask);
        return;
    }

    if (hard_fault_latched != 0U)
    {
        SolenoidOutput_ForceSafe();
        AttitudeControl_UpdateRuntimeStatus();
        AttitudeControl_ExitCritical(primask);
        return;
    }

    if (estimator_healthy == 0U)
    {
        AttitudeControl_AbortAxis(&roll_axis, 1U);
        AttitudeControl_AbortAxis(&pitch_axis, 1U);
        SolenoidOutput_ForceSafe();
        control_status.attitude_fresh = 0U;
        control_status.state = ATT_CTRL_STATE_WAIT_ESTIMATOR;

        if ((uint32_t)(now_ms - control_status.init_ms) >=
            ATT_CTRL_STARTUP_DELAY_MS)
        {
            control_status.fault = ATT_CTRL_FAULT_ESTIMATOR_UNHEALTHY;
        }

        AttitudeControl_UpdateRuntimeStatus();
        if (control_status.fault == ATT_CTRL_FAULT_NONE)
        {
            control_status.state = ATT_CTRL_STATE_WAIT_ESTIMATOR;
        }
        AttitudeControl_ExitCritical(primask);
        return;
    }

    if (has_eskf_sample == 0U)
    {
        SolenoidOutput_ForceSafe();
        control_status.attitude_fresh = 0U;
        control_status.state = ATT_CTRL_STATE_WAIT_ESTIMATOR;
        AttitudeControl_SyncOutputStatus();
        AttitudeControl_ExitCritical(primask);
        return;
    }

    if ((uint32_t)(now_ms - last_eskf_sample_ms) >
        ATT_CTRL_SENSOR_TIMEOUT_MS)
    {
        control_status.attitude_fresh = 0U;
        AttitudeControl_EnterFaultLocked(ATT_CTRL_FAULT_SENSOR_TIMEOUT, 0U);
        AttitudeControl_ExitCritical(primask);
        return;
    }

    output = SolenoidOutput_GetStatus();
    if (output.interlock_fault != 0U)
    {
        AttitudeControl_EnterFaultLocked(
            ATT_CTRL_FAULT_OUTPUT_INTERLOCK,
            1U
        );
        AttitudeControl_ExitCritical(primask);
        return;
    }

    control_status.attitude_fresh = 1U;
    if ((control_status.fault == ATT_CTRL_FAULT_ESTIMATOR_UNHEALTHY) ||
        (control_status.fault == ATT_CTRL_FAULT_SENSOR_TIMEOUT))
    {
        control_status.fault = ATT_CTRL_FAULT_NONE;
    }

    AttitudeControl_UpdateRuntimeStatus();
    AttitudeControl_ExitCritical(primask);
}

void AttitudeControl_UpdateESKF(
    const FullStateESKFData_t *eskf,
    const SensorData_t *sensor,
    uint32_t now_ms
)
{
    uint32_t primask = AttitudeControl_EnterCritical();
    float roll_angle_deg;
    float pitch_angle_deg;
    float roll_rate_dps;
    float pitch_rate_dps;
    uint8_t roll_started = 0U;
    uint8_t pitch_started = 0U;

    control_status.control_update_count++;

    if ((eskf == 0) || (sensor == 0))
    {
        AttitudeControl_EnterFaultLocked(
            ATT_CTRL_FAULT_ESTIMATOR_UNHEALTHY,
            0U
        );
        AttitudeControl_ExitCritical(primask);
        return;
    }

    if ((eskf->enabled == 0U) ||
        (eskf->initialized == 0U) ||
        (eskf->healthy == 0U) ||
        (eskf->origin_zeroed == 0U) ||
        (sensor->imu_valid == 0U))
    {
        control_status.estimator_healthy = 0U;
        AttitudeControl_EnterFaultLocked(
            ATT_CTRL_FAULT_ESTIMATOR_UNHEALTHY,
            0U
        );
        AttitudeControl_ExitCritical(primask);
        return;
    }

    roll_angle_deg =
        eskf->roll_deg * (float)ATT_CTRL_ROLL_ANGLE_SIGN;
    pitch_angle_deg =
        eskf->pitch_deg * (float)ATT_CTRL_PITCH_ANGLE_SIGN;
    roll_rate_dps =
        (sensor->gyro_x_filtered_dps - eskf->gyro_bias_x_dps) *
        (float)ATT_CTRL_ROLL_RATE_SIGN;
    pitch_rate_dps =
        (sensor->gyro_y_filtered_dps - eskf->gyro_bias_y_dps) *
        (float)ATT_CTRL_PITCH_RATE_SIGN;

    if ((AttitudeControl_IsFinite(roll_angle_deg) == 0U) ||
        (AttitudeControl_IsFinite(pitch_angle_deg) == 0U) ||
        (AttitudeControl_IsFinite(roll_rate_dps) == 0U) ||
        (AttitudeControl_IsFinite(pitch_rate_dps) == 0U))
    {
        AttitudeControl_EnterFaultLocked(
            ATT_CTRL_FAULT_ATTITUDE_NONFINITE,
            1U
        );
        AttitudeControl_ExitCritical(primask);
        return;
    }

    has_eskf_sample = 1U;
    last_eskf_sample_ms = now_ms;
    control_status.last_attitude_ms = now_ms;
    control_status.estimator_healthy = 1U;
    control_status.attitude_fresh = 1U;
    control_status.eskf_source_active = 1U;

    control_status.roll_deg = roll_angle_deg;
    control_status.pitch_deg = pitch_angle_deg;
    control_status.roll_rate_dps = roll_rate_dps;
    control_status.pitch_rate_dps = pitch_rate_dps;
    control_status.selected_control_angle_deg = roll_angle_deg;
    control_status.selected_control_rate_dps = roll_rate_dps;

    if ((uint32_t)(now_ms - control_status.init_ms) <
        ATT_CTRL_STARTUP_DELAY_MS)
    {
        SolenoidOutput_ForceSafe();
        control_status.state = ATT_CTRL_STATE_WAIT_ESTIMATOR;
        AttitudeControl_SyncOutputStatus();
        AttitudeControl_ExitCritical(primask);
        return;
    }

    if ((control_status.config_valid == 0U) ||
        (hard_fault_latched != 0U))
    {
        SolenoidOutput_ForceSafe();
        AttitudeControl_UpdateRuntimeStatus();
        AttitudeControl_ExitCritical(primask);
        return;
    }

    control_status.fault = ATT_CTRL_FAULT_NONE;

    if (ATT_CTRL_ROLL_ENABLE != 0U)
    {
        roll_started = AttitudeControl_TryStartPulse(
            &roll_axis,
            roll_angle_deg,
            roll_rate_dps
        );
    }

    if (ATT_CTRL_PITCH_ENABLE != 0U)
    {
        pitch_started = AttitudeControl_TryStartPulse(
            &pitch_axis,
            pitch_angle_deg,
            pitch_rate_dps
        );
    }

    if ((roll_started != 0U) || (pitch_started != 0U))
    {
        control_status.command_change_count +=
            (uint32_t)roll_started + (uint32_t)pitch_started;
        control_status.event_count +=
            (uint32_t)roll_started + (uint32_t)pitch_started;
        control_status.pulse_count +=
            (uint32_t)roll_started + (uint32_t)pitch_started;
        control_status.pulse_cycle_start_ms = now_ms;
    }

    AttitudeControl_ApplyMaskLocked();
    AttitudeControl_UpdateRuntimeStatus();
    AttitudeControl_ExitCritical(primask);
}

void AttitudeControl_TimerTickISR(void)
{
    uint8_t state_changed = 0U;

    control_status.timer_tick_active = 1U;

    /* P39: a critical system fault aborts the internal pulse state in the same
     * 1 ms timer cycle.  This prevents an old pulse from being re-applied if a
     * transient fault later clears. */
    if (((roll_axis.active_error_sign != 0) ||
         (pitch_axis.active_error_sign != 0)) &&
        (SystemMonitor_IsActuatorFaultActive() != 0U))
    {
        AttitudeControl_AbortAxis(&roll_axis, 1U);
        AttitudeControl_AbortAxis(&pitch_axis, 1U);
        SolenoidOutput_ForceSafe();
        control_status.estimator_healthy = 0U;
        control_status.attitude_fresh = 0U;
        control_status.state = ATT_CTRL_STATE_SENSOR_FAULT;
        AttitudeControl_UpdateRuntimeStatus();
        return;
    }

    if ((ATT_CTRL_ENABLE == 0U) ||
        (control_status.config_valid == 0U) ||
        (hard_fault_latched != 0U))
    {
        if ((roll_axis.active_error_sign != 0) ||
            (pitch_axis.active_error_sign != 0))
        {
            AttitudeControl_AbortAxis(&roll_axis, 1U);
            AttitudeControl_AbortAxis(&pitch_axis, 1U);
            SolenoidOutput_ForceSafe();
            AttitudeControl_UpdateRuntimeStatus();
        }
        return;
    }

    if (roll_axis.cooldown_remaining_ms > 0UL)
    {
        roll_axis.cooldown_remaining_ms--;
        if (roll_axis.cooldown_remaining_ms == 0UL)
        {
            state_changed = 1U;
        }
    }

    if (pitch_axis.cooldown_remaining_ms > 0UL)
    {
        pitch_axis.cooldown_remaining_ms--;
        if (pitch_axis.cooldown_remaining_ms == 0UL)
        {
            state_changed = 1U;
        }
    }

    if (roll_axis.pulse_remaining_ms > 0UL)
    {
        roll_axis.pulse_remaining_ms--;
        if (roll_axis.pulse_remaining_ms == 0UL)
        {
            roll_axis.active_error_sign = 0;
            roll_axis.cooldown_remaining_ms = ATT_CTRL_COOLDOWN_MS;
            state_changed = 1U;
        }
    }

    if (pitch_axis.pulse_remaining_ms > 0UL)
    {
        pitch_axis.pulse_remaining_ms--;
        if (pitch_axis.pulse_remaining_ms == 0UL)
        {
            pitch_axis.active_error_sign = 0;
            pitch_axis.cooldown_remaining_ms = ATT_CTRL_COOLDOWN_MS;
            state_changed = 1U;
        }
    }

    if (state_changed != 0U)
    {
        AttitudeControl_ApplyMaskLocked();
    }

    AttitudeControl_UpdateRuntimeStatus();
}

/* Legacy wrappers retained; the V49 physical path never falls back to them. */

void AttitudeControl_Service(
    const AttitudeEstimatorData_t *attitude,
    uint8_t estimator_healthy,
    uint32_t now_ms
)
{
    (void)attitude;
    AttitudeControl_ServiceESKF(estimator_healthy, now_ms);
}

void AttitudeControl_Update(
    const AttitudeEstimatorData_t *attitude,
    uint8_t estimator_healthy,
    float roll_rate_dps,
    float pitch_rate_dps,
    float landing_height_m,
    float vertical_velocity_mps,
    uint8_t landing_state_valid,
    uint32_t now_ms
)
{
    (void)attitude;
    (void)estimator_healthy;
    (void)roll_rate_dps;
    (void)pitch_rate_dps;
    (void)landing_height_m;
    (void)vertical_velocity_mps;
    (void)landing_state_valid;
    (void)now_ms;

    /* Deliberately no physical output: Full-State ESKF is mandatory in V49. */
    AttitudeControl_ForceSafe();
}

AttitudeControlStatus_t AttitudeControl_GetStatus(void)
{
    AttitudeControlStatus_t copy;
    uint32_t primask = AttitudeControl_EnterCritical();

    copy = control_status;
    AttitudeControl_ExitCritical(primask);
    return copy;
}

const char *AttitudeControl_StateName(AttitudeControlState_t state)
{
    switch (state)
    {
        case ATT_CTRL_STATE_DISABLED:
            return "DISABLED";
        case ATT_CTRL_STATE_WAIT_ESTIMATOR:
            return "WAIT_ESTIMATOR";
        case ATT_CTRL_STATE_HOLD:
            return "HOLD";
        case ATT_CTRL_STATE_ACTIVE:
            return "ACTIVE";
        case ATT_CTRL_STATE_REVERSAL_DEADTIME:
            return "REVERSAL_DEADTIME";
        case ATT_CTRL_STATE_SENSOR_FAULT:
            return "SENSOR_FAULT";
        case ATT_CTRL_STATE_COOLDOWN:
            return "COOLDOWN";
        default:
            return "UNKNOWN";
    }
}

const char *AttitudeControl_AxisModeName(AttitudeControlAxisMode_t mode)
{
    switch (mode)
    {
        case ATT_CTRL_AXIS_MODE_HOLD:
            return "HOLD";
        case ATT_CTRL_AXIS_MODE_CORRECTING:
            return "CORRECTING";
        case ATT_CTRL_AXIS_MODE_BRAKING:
            return "BRAKING";
        case ATT_CTRL_AXIS_MODE_REVERSAL_DEADTIME:
            return "REVERSAL_DEADTIME";
        case ATT_CTRL_AXIS_MODE_SAFE_WAIT:
            return "SAFE_WAIT";
        case ATT_CTRL_AXIS_MODE_COASTING:
            return "COASTING";
        case ATT_CTRL_AXIS_MODE_COOLDOWN:
            return "COOLDOWN";
        default:
            return "UNKNOWN";
    }
}
