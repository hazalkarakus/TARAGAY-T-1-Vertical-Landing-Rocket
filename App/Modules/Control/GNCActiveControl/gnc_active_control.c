#include "Modules/Control/GNCActiveControl/gnc_active_control.h"

#include "Common/app_config.h"

#include "Modules/Control/AttitudeControl/attitude_control.h"
#include "Modules/Control/NeedleValve/needle_valve_controller.h"
#include "Modules/Control/ControlInputProvider/control_input_provider.h"
#include "Modules/Control/VerticalSensorPolicy/vertical_sensor_policy.h"
#if (APP_LEGACY_VERTICAL_LANDING_RETIRED == 0U)
#include "../VerticalLandingControl/vertical_landing_control.h"
#endif

#include "Modules/Estimation/AttitudeEstimator/attitude_estimator.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"

#include "Modules/Sensors/SensorManager/sensor_manager.h"

#include "Services/SolenoidOutput/solenoid_output.h"

#include "main.h"

#include <math.h>

/* -------------------------------------------------------------------------- */
/* Reliable globals                                                           */
/* -------------------------------------------------------------------------- */

volatile uint32_t gnc_v816_magic = 0x816C16C1UL;

volatile uint8_t gnc_v816_button_stage = 0U;
volatile uint32_t gnc_v816_button_press_count = 0UL;

volatile uint8_t gnc_v816_armed = 0U;
volatile uint8_t gnc_v816_disarm_closing = 0U;
volatile uint8_t gnc_v816_fault_latched = 0U;
volatile uint8_t gnc_v816_fault_code = GNC_ACTIVE_FAULT_NONE;

volatile uint8_t gnc_v816_estimator_ok = 0U;
volatile uint8_t gnc_v816_vertical_state_ok = 0U;
volatile uint8_t gnc_v816_attitude_state_ok = 0U;
volatile uint8_t gnc_v816_needle_ok = 0U;

volatile float gnc_v816_height_agl_m = 0.0f;
volatile float gnc_v816_z_cg_m = APP_GNC_CG_TOUCH_HEIGHT_M;
volatile float gnc_v816_vertical_velocity_mps = 0.0f;

volatile float gnc_v816_roll_deg = 0.0f;
volatile float gnc_v816_pitch_deg = 0.0f;
volatile float gnc_v816_yaw_deg = 0.0f;
volatile float gnc_v816_roll_rate_dps = 0.0f;
volatile float gnc_v816_pitch_rate_dps = 0.0f;

volatile float gnc_v816_vertical_valve_cmd = 0.0f;
volatile float gnc_v816_vertical_reference_speed_mps = 0.0f;
volatile float gnc_v816_vertical_speed_error_mps = 0.0f;

volatile uint8_t gnc_v816_rcs_requested_mask = 0U;
volatile uint8_t gnc_v816_rcs_applied_mask = 0U;
volatile uint8_t gnc_v816_rcs_dry_run = 1U;

volatile uint32_t gnc_v816_update_200hz_count = 0UL;
volatile uint32_t gnc_v816_service_1khz_count = 0UL;
volatile uint32_t gnc_v816_soft_invalid_count = 0UL;
volatile uint32_t gnc_v816_fault_count = 0UL;

/* -------------------------------------------------------------------------- */
/* Private state                                                              */
/* -------------------------------------------------------------------------- */

static uint8_t last_button = 0U;
static uint32_t last_button_ms = 0UL;

static uint32_t estimator_invalid_since_ms = 0UL;
static uint8_t estimator_invalid_timer_active = 0U;

static float last_vertical_command = -1.0f;

static AttitudeEstimatorData_t attitude_adapter;
static uint32_t last_attitude_euler_update_count = 0UL;
static uint32_t last_attitude_estimator_reset_count = 0UL;

/* -------------------------------------------------------------------------- */

static void GNC_ClearAttitudeAdapter(void)
{
    uint8_t *bytes = (uint8_t *)&attitude_adapter;
    uint32_t i;

    for (i = 0UL; i < (uint32_t)sizeof(attitude_adapter); i++)
    {
        bytes[i] = 0U;
    }
}

static void GNC_ButtonInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOA, &gpio);
}

static uint8_t GNC_ButtonRead(void)
{
    return
        (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
        ? 1U
        : 0U;
}

static uint8_t GNC_IsFinite(float value)
{
    return (isfinite(value) != 0) ? 1U : 0U;
}

static uint8_t GNC_ConfigValid(void)
{
    if ((APP_GNC_ACTIVE_ENABLED > 1U) ||
        (APP_GNC_RCS_RELAY_BENCH_MODE > 1U) ||
        (APP_GNC_COMBINED_DRY_RUN_MODE > 1U) ||
        (APP_GNC_RCS_DRY_RUN > 1U) ||
        (APP_GNC_NEEDLE_PHYSICAL_ENABLED > 1U))
    {
        return 0U;
    }

#if (APP_GNC_RCS_RELAY_BENCH_MODE != 0U)
    if ((APP_GNC_ACTIVE_ENABLED == 0U) ||
        (APP_GNC_RCS_DRY_RUN != 0U) ||
        (APP_GNC_NEEDLE_PHYSICAL_ENABLED != 0U))
    {
        return 0U;
    }
#endif

#if (APP_GNC_COMBINED_DRY_RUN_MODE != 0U)
    if ((APP_GNC_ACTIVE_ENABLED == 0U) ||
        (APP_GNC_RCS_RELAY_BENCH_MODE != 0U) ||
        (APP_GNC_RCS_DRY_RUN != 0U) ||
        (APP_GNC_NEEDLE_PHYSICAL_ENABLED != 0U) ||
        (APP_GNC_VERTICAL_CONTROLLER_IMPLEMENTED == 0U))
    {
        return 0U;
    }
#endif

#if (APP_GNC_RCS_RELAY_BENCH_MODE == 0U)
    if ((GNC_IsFinite(APP_GNC_MODEL_MASS_KG) == 0U) ||
        (GNC_IsFinite(APP_GNC_MODEL_MAIN_PRESSURE_BAR) == 0U) ||
        (GNC_IsFinite(APP_GNC_CG_TOUCH_HEIGHT_M) == 0U) ||
        (APP_GNC_MODEL_MASS_KG <= 0.0f) ||
        (APP_GNC_MODEL_MAIN_PRESSURE_BAR < 0.0f) ||
        (APP_GNC_MAX_VALID_HEIGHT_AGL_M <= 0.0f))
    {
        return 0U;
    }
#endif

    return 1U;
}

static void GNC_SetFault(GNCActiveFault_t fault)
{
    if (gnc_v816_fault_latched == 0U)
    {
        gnc_v816_fault_count++;
    }

    gnc_v816_fault_latched = 1U;
    gnc_v816_fault_code = (uint8_t)fault;
    gnc_v816_armed = 0U;

    SolenoidOutput_ForceSafe();

#if (APP_GNC_NEEDLE_PHYSICAL_ENABLED != 0U)
    /*
     * Command CLOSED first. Do not blindly disable a healthy actuator while
     * it may be open.
     */
    if (needle_valve_fault == NEEDLE_VALVE_FAULT_NONE)
    {
        (void)NeedleValveController_SetCommand(0.0f);
        gnc_v816_disarm_closing = 1U;
    }
#endif
}

static uint8_t GNC_BuildEstimatorState(void)
{
    const FullStateESKFData_t *eskf = FullStateESKF_GetDataPtr();
    const AttitudeEstimatorData_t *attitude =
        AttitudeEstimator_GetDataPtr();
    const SensorData_t *sensor = SensorManager_GetDataPtr();

    float height_agl;
    VerticalSensorPolicyResult_t vertical_policy;
    uint8_t eskf_ok;
    uint8_t attitude_ok;

    if ((eskf == 0) || (attitude == 0) || (sensor == 0))
    {
        return 0U;
    }

    vertical_policy = VerticalSensorPolicy_Evaluate(eskf);

    eskf_ok =
        ((eskf->enabled == 1U) &&
         (eskf->initialized == 1U) &&
         (eskf->healthy == 1U) &&
         (eskf->output_inhibited == 0U) &&
         (eskf->numerical_error_count == 0UL)) ? 1U : 0U;

    /*
     * V8.19F attitude source:
     * Use the proven quaternion/Euler estimator from
     * TGY_FULL_SENSOR_NRF_SERVO_90 for roll, pitch and yaw.  The full ESKF
     * remains responsible for position, velocity and vertical sensor fusion.
     */
    attitude_ok =
        ((attitude->enabled == 1U) &&
         (attitude->initialized == 1U) &&
         (attitude->healthy == 1U) &&
         (attitude->numerical_error_count == 0UL)) ? 1U : 0U;

    gnc_v816_estimator_ok =
        ((eskf_ok != 0U) && (attitude_ok != 0U)) ? 1U : 0U;

    gnc_v816_attitude_state_ok =
        ((attitude_ok != 0U) &&
         (GNC_IsFinite(attitude->roll_deg) != 0U) &&
         (GNC_IsFinite(attitude->pitch_deg) != 0U) &&
         (GNC_IsFinite(attitude->yaw_deg) != 0U) &&
         (sensor->imu_valid != 0U) &&
         (GNC_IsFinite(sensor->gyro_x_filtered_dps) != 0U) &&
         (GNC_IsFinite(sensor->gyro_y_filtered_dps) != 0U)) ? 1U : 0U;

    height_agl =
        eskf->lidar_reference_m +
        eskf->position_z_m;

    if (height_agl < 0.0f)
    {
        height_agl = 0.0f;
    }

    gnc_v816_height_agl_m = height_agl;
    gnc_v816_z_cg_m =
        APP_GNC_CG_TOUCH_HEIGHT_M + height_agl;
    gnc_v816_vertical_velocity_mps =
        eskf->velocity_z_mps;

    gnc_v816_vertical_state_ok =
        ((eskf_ok != 0U) &&
         (vertical_policy.usable != 0U) &&
         (GNC_IsFinite(height_agl) != 0U) &&
         (GNC_IsFinite(eskf->velocity_z_mps) != 0U) &&
         (height_agl <= APP_GNC_MAX_VALID_HEIGHT_AGL_M)) ? 1U : 0U;

    gnc_v816_roll_deg = attitude->roll_deg;
    gnc_v816_pitch_deg = attitude->pitch_deg;
    gnc_v816_yaw_deg = attitude->yaw_deg;

    /* Match the original controller input path exactly. */
    gnc_v816_roll_rate_dps = sensor->gyro_x_filtered_dps;
    gnc_v816_pitch_rate_dps = sensor->gyro_y_filtered_dps;

    GNC_ClearAttitudeAdapter();

    attitude_adapter.enabled = attitude->enabled;
    attitude_adapter.initialized = attitude->initialized;
    attitude_adapter.healthy = attitude->healthy;

    attitude_adapter.q_w = attitude->q_w;
    attitude_adapter.q_x = attitude->q_x;
    attitude_adapter.q_y = attitude->q_y;
    attitude_adapter.q_z = attitude->q_z;

    attitude_adapter.roll_deg = attitude->roll_deg;
    attitude_adapter.pitch_deg = attitude->pitch_deg;
    attitude_adapter.yaw_deg = attitude->yaw_deg;
    attitude_adapter.euler_update_count = attitude->euler_update_count;
    attitude_adapter.reset_count = attitude->reset_count;
    attitude_adapter.last_timestamp_us = attitude->last_timestamp_us;

#if (APP_GNC_RCS_RELAY_BENCH_MODE != 0U)
    /*
     * Relay-only bench control must not depend on the needle valve, MATLAB
     * vertical controller, barometer origin or LIDAR reference.  The classic
     * RCS controller can still use its hard 10-degree limit with a valid
     * attitude even when landing prediction is unavailable.
     */
    return
        ((attitude_ok != 0U) &&
         (gnc_v816_attitude_state_ok != 0U)) ? 1U : 0U;
#else
    return
        ((gnc_v816_estimator_ok != 0U) &&
         (gnc_v816_attitude_state_ok != 0U) &&
         (gnc_v816_vertical_state_ok != 0U)) ? 1U : 0U;
#endif
}

#if ((APP_GNC_RCS_RELAY_BENCH_MODE == 0U) && \
     (APP_GNC_COMBINED_DRY_RUN_MODE == 0U))
static uint8_t GNC_NeedleReady(void)
{
    NeedleValveStatus_t needle = NeedleValveController_GetStatus();

    gnc_v816_needle_ok =
        ((needle.enabled == 1U) &&
         (needle.zero_valid == 1U) &&
         (needle.fault == NEEDLE_VALVE_FAULT_NONE)) ? 1U : 0U;

    return gnc_v816_needle_ok;
}
#endif

static void GNC_SyncRCSStatus(void)
{
    SolenoidOutputStatus_t solenoid = SolenoidOutput_GetStatus();

    gnc_v816_rcs_requested_mask = solenoid.requested_mask;
    gnc_v816_rcs_applied_mask = solenoid.applied_mask;
    gnc_v816_rcs_dry_run = solenoid.dry_run;
}

static void GNC_RequestDisarm(void)
{
    gnc_v816_armed = 0U;
    estimator_invalid_timer_active = 0U;
    estimator_invalid_since_ms = 0UL;

    SolenoidOutput_ForceSafe();

#if (APP_GNC_NEEDLE_PHYSICAL_ENABLED != 0U)
    if (needle_valve_fault == NEEDLE_VALVE_FAULT_NONE)
    {
        (void)NeedleValveController_SetCommand(0.0f);
        gnc_v816_disarm_closing = 1U;
    }
    else
    {
        NeedleValveController_Stop();
        gnc_v816_disarm_closing = 0U;
    }
#endif

    last_vertical_command = -1.0f;
}

static uint8_t GNC_TryArm(uint32_t now_ms)
{
    uint8_t estimator_ready;

    if ((APP_GNC_ACTIVE_ENABLED == 0U) ||
        (GNC_ConfigValid() == 0U))
    {
        GNC_SetFault(GNC_ACTIVE_FAULT_CONFIG);
        return 0U;
    }

    estimator_ready = GNC_BuildEstimatorState();

    if (estimator_ready == 0U)
    {
        return 0U;
    }

#if ((APP_GNC_RCS_RELAY_BENCH_MODE == 0U) && \
     (APP_GNC_COMBINED_DRY_RUN_MODE == 0U))
    if (GNC_NeedleReady() == 0U)
    {
        return 0U;
    }

#else
    /* Bench and combined dry-run arming intentionally leave needle disabled. */
    gnc_v816_needle_ok = 0U;
#endif

    gnc_v816_fault_latched = 0U;
    gnc_v816_fault_code = GNC_ACTIVE_FAULT_NONE;
    gnc_v816_disarm_closing = 0U;

    estimator_invalid_timer_active = 0U;
    estimator_invalid_since_ms = 0UL;

    /* Reset event/history state at every explicit arm. */
    AttitudeControl_Init(now_ms);

    last_vertical_command = -1.0f;

    gnc_v816_armed = 1U;

    return 1U;
}

static void GNC_HandleButton(uint32_t now_ms)
{
    gnc_v816_button_press_count++;

    if (gnc_v816_fault_latched != 0U)
    {
        /*
         * One press acknowledges the GNC latch. Needle's own hardware fault
         * remains protected by its controller and is not silently cleared.
         */
        gnc_v816_fault_latched = 0U;
        gnc_v816_fault_code = GNC_ACTIVE_FAULT_NONE;
        gnc_v816_button_stage = 0U;
        GNC_RequestDisarm();
        return;
    }

#if ((APP_GNC_RCS_RELAY_BENCH_MODE != 0U) || \
     (APP_GNC_COMBINED_DRY_RUN_MODE != 0U))
    if (gnc_v816_armed != 0U)
    {
        GNC_RequestDisarm();
        gnc_v816_button_stage = 0U;
    }
    else if (GNC_TryArm(now_ms) != 0U)
    {
        gnc_v816_button_stage = 1U;
    }

    return;
#else
    switch (gnc_v816_button_stage)
    {
        case 0U:
            if (NeedleValveController_CaptureZero() != 0U)
            {
                gnc_v816_button_stage = 1U;
            }
            break;

        case 1U:
            if (NeedleValveController_Enable() != 0U)
            {
                gnc_v816_button_stage = 2U;
            }
            break;

        case 2U:
            if (GNC_TryArm(now_ms) != 0U)
            {
                gnc_v816_button_stage = 3U;
            }
            break;

        default:
            GNC_RequestDisarm();
            gnc_v816_button_stage = 0U;
            break;
    }
#endif
}

/* -------------------------------------------------------------------------- */

void GNCActiveControl_Init(void)
{
    uint32_t now_ms = HAL_GetTick();

    GNC_ButtonInit();
#if ((APP_GNC_RCS_RELAY_BENCH_MODE == 0U) && \
     (APP_LEGACY_VERTICAL_LANDING_RETIRED == 0U))
    VerticalLandingControl_Init();
    ControlInputProvider_Init();
#endif
    AttitudeControl_Init(now_ms);

    gnc_v816_button_stage = 0U;
    gnc_v816_button_press_count = 0UL;

    gnc_v816_armed = 0U;
    gnc_v816_disarm_closing = 0U;
    gnc_v816_fault_latched = 0U;
    gnc_v816_fault_code = GNC_ACTIVE_FAULT_NONE;

    gnc_v816_estimator_ok = 0U;
    gnc_v816_vertical_state_ok = 0U;
    gnc_v816_attitude_state_ok = 0U;
    gnc_v816_needle_ok = 0U;

    gnc_v816_vertical_valve_cmd = 0.0f;
    gnc_v816_vertical_reference_speed_mps = 0.0f;
    gnc_v816_vertical_speed_error_mps = 0.0f;

    gnc_v816_rcs_requested_mask = 0U;
    gnc_v816_rcs_applied_mask = 0U;
    gnc_v816_rcs_dry_run = (APP_GNC_RCS_DRY_RUN != 0U) ? 1U : 0U;

    gnc_v816_update_200hz_count = 0UL;
    gnc_v816_service_1khz_count = 0UL;
    gnc_v816_soft_invalid_count = 0UL;
    gnc_v816_fault_count = 0UL;

    estimator_invalid_timer_active = 0U;
    estimator_invalid_since_ms = 0UL;

    last_vertical_command = -1.0f;

    GNC_ClearAttitudeAdapter();
    last_attitude_euler_update_count = 0UL;
    last_attitude_estimator_reset_count = 0UL;

    last_button = GNC_ButtonRead();
    last_button_ms = now_ms;

    GNC_RequestDisarm();
}

/* -------------------------------------------------------------------------- */

void GNCActiveControl_Service1kHz(void)
{
    uint32_t now_ms = HAL_GetTick();

    gnc_v816_service_1khz_count++;

#if (APP_LEGACY_VERTICAL_LANDING_RETIRED != 0U)
    /* R8R21 single-authority guard: legacy GNC cannot own RCS or needle. */
    SolenoidOutput_ForceSafe();
    GNC_SyncRCSStatus();
    return;
#endif

    if (gnc_v816_armed == 0U)
    {
        SolenoidOutput_ForceSafe();
        GNC_SyncRCSStatus();
        return;
    }

    if (GNC_BuildEstimatorState() == 0U)
    {
        /*
         * First response is immediately safe. Latch only if the invalidity
         * persists, so one scheduling/sample transient cannot permanently
         * kill an otherwise healthy bench run.
         */
        SolenoidOutput_ForceSafe();

        if (estimator_invalid_timer_active == 0U)
        {
            estimator_invalid_timer_active = 1U;
            estimator_invalid_since_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - estimator_invalid_since_ms) >=
                 APP_GNC_SENSOR_FAULT_CONFIRM_MS)
        {
            GNC_SetFault(GNC_ACTIVE_FAULT_ESTIMATOR);
        }

        GNC_SyncRCSStatus();
        return;
    }

    estimator_invalid_timer_active = 0U;
    estimator_invalid_since_ms = 0UL;

    AttitudeControl_Service(
        &attitude_adapter,
        1U,
        now_ms
    );

    {
        AttitudeControlStatus_t attitude_status =
            AttitudeControl_GetStatus();

        if (attitude_status.fault != ATT_CTRL_FAULT_NONE)
        {
            GNC_SetFault(GNC_ACTIVE_FAULT_ATTITUDE_CONTROL);
        }
    }

    GNC_SyncRCSStatus();
}

/* -------------------------------------------------------------------------- */

void GNCActiveControl_Update200Hz(void)
{
    uint32_t now_ms = HAL_GetTick();

#if ((APP_GNC_RCS_RELAY_BENCH_MODE == 0U) && \
     (APP_LEGACY_VERTICAL_LANDING_RETIRED == 0U))
    VerticalLandingControlOutput_t vertical;
#endif

    gnc_v816_update_200hz_count++;

#if (APP_LEGACY_VERTICAL_LANDING_RETIRED != 0U)
    /* R8R21: TaragayFlightLogic is the only mission control authority. */
    gnc_v816_vertical_valve_cmd = 0.0f;
    gnc_v816_vertical_reference_speed_mps = 0.0f;
    gnc_v816_vertical_speed_error_mps = 0.0f;
    SolenoidOutput_ForceSafe();
    GNC_SyncRCSStatus();
    return;
#else

    if (GNC_BuildEstimatorState() == 0U)
    {
        gnc_v816_soft_invalid_count++;
        gnc_v816_vertical_valve_cmd = 0.0f;

        if (gnc_v816_armed != 0U)
        {
#if (APP_GNC_NEEDLE_PHYSICAL_ENABLED != 0U)
            if (needle_valve_fault == NEEDLE_VALVE_FAULT_NONE)
            {
                (void)NeedleValveController_SetCommand(0.0f);
            }
#endif
            SolenoidOutput_ForceSafe();
        }

        GNC_SyncRCSStatus();
        return;
    }

    /*
     * Preserve the original TGY_FULL_SENSOR_NRF_SERVO_90 control cadence:
     * - classic AttitudeEstimator roll/pitch/yaw
     * - original filtered body gyro X/Y rate inputs
     * - RCS threshold logic only on a newly calculated Euler sample
     * - ESKF height/vertical speed only for touchdown prediction
     */
    if (attitude_adapter.reset_count !=
        last_attitude_estimator_reset_count)
    {
        last_attitude_estimator_reset_count =
            attitude_adapter.reset_count;
        last_attitude_euler_update_count =
            attitude_adapter.euler_update_count;
        AttitudeControl_NotifyEstimatorReset(now_ms);
    }
    else if (attitude_adapter.euler_update_count !=
             last_attitude_euler_update_count)
    {
        last_attitude_euler_update_count =
            attitude_adapter.euler_update_count;

        AttitudeControl_Update(
            &attitude_adapter,
            1U,
            gnc_v816_roll_rate_dps,
            gnc_v816_pitch_rate_dps,
            gnc_v816_height_agl_m,
            gnc_v816_vertical_velocity_mps,
            gnc_v816_vertical_state_ok,
            now_ms
        );
    }

#if ((APP_GNC_RCS_RELAY_BENCH_MODE == 0U) && \
     (APP_LEGACY_VERTICAL_LANDING_RETIRED == 0U))
    {
        ControlInputProviderData_t control_input = ControlInputProvider_Update();

        if (control_input.all_valid != 0U)
        {
            vertical = VerticalLandingControl_Update(
                control_input.z_m,
                control_input.v_mps,
                control_input.mass_kg,
                control_input.main_pressure_bar
            );
        }
        else
        {
            vertical.input_valid = 0U;
            vertical.valve_cmd = 0.0f;
            vertical.height_above_touchdown_m = 0.0f;
            vertical.downward_speed_mps = 0.0f;
            vertical.reference_downward_speed_mps = 0.0f;
            vertical.speed_error_mps = 0.0f;
            vertical.available_force_n = 0.0f;
            vertical.commanded_accel_mps2 = 0.0f;
            vertical.required_force_n = 0.0f;
        }

        gnc_v816_vertical_valve_cmd =
            (vertical.input_valid != 0U) ? vertical.valve_cmd : 0.0f;
        gnc_v816_vertical_reference_speed_mps =
            vertical.reference_downward_speed_mps;
        gnc_v816_vertical_speed_error_mps = vertical.speed_error_mps;
    }
#else
    /* R8R21: legacy landing owner is retired even if GNCActiveControl is
     * accidentally re-enabled.  This path can never command the needle. */
    gnc_v816_vertical_valve_cmd = 0.0f;
    gnc_v816_vertical_reference_speed_mps = 0.0f;
    gnc_v816_vertical_speed_error_mps = 0.0f;
#endif

    if (gnc_v816_armed == 0U)
    {
        SolenoidOutput_ForceSafe();
        GNC_SyncRCSStatus();
        return;
    }

#if (APP_GNC_RCS_RELAY_BENCH_MODE != 0U)
    /*
     * Legacy relay-only mode stops here: only the classic attitude controller
     * may drive the four relay outputs. Vertical and needle stay disconnected.
     */
    {
        AttitudeControlStatus_t attitude_status =
            AttitudeControl_GetStatus();

        if (attitude_status.fault != ATT_CTRL_FAULT_NONE)
        {
            GNC_SetFault(GNC_ACTIVE_FAULT_ATTITUDE_CONTROL);
        }
    }

    GNC_SyncRCSStatus();
    return;
#else
#if (APP_GNC_COMBINED_DRY_RUN_MODE != 0U)
    /*
     * V8.19I combined validation: RCS reaches the proven physical relay path,
     * while the vertical controller is compute-only. Needle PWM/enables are
     * never touched by the automatic controller in this build.
     */
    if (vertical.input_valid == 0U)
    {
        GNC_SetFault(GNC_ACTIVE_FAULT_VERTICAL_STATE);
        GNC_SyncRCSStatus();
        return;
    }

    {
        AttitudeControlStatus_t attitude_status =
            AttitudeControl_GetStatus();

        if (attitude_status.fault != ATT_CTRL_FAULT_NONE)
        {
            GNC_SetFault(GNC_ACTIVE_FAULT_ATTITUDE_CONTROL);
        }
    }

    GNC_SyncRCSStatus();
    return;
#else
    if ((vertical.input_valid == 0U) ||
        (GNC_NeedleReady() == 0U))
    {
        GNC_SetFault(
            (vertical.input_valid == 0U)
                ? GNC_ACTIVE_FAULT_VERTICAL_STATE
                : GNC_ACTIVE_FAULT_NEEDLE
        );

        GNC_SyncRCSStatus();
        return;
    }

#if (APP_GNC_NEEDLE_PHYSICAL_ENABLED != 0U)
    /*
     * Avoid pointlessly rewriting an identical command every 5 ms while still
     * allowing the outer control to update at 200 Hz when it changes.
     */
    if ((last_vertical_command < 0.0f) ||
        (fabsf(vertical.valve_cmd - last_vertical_command) >= 0.0005f))
    {
        if (NeedleValveController_SetCommand(vertical.valve_cmd) == 0U)
        {
            GNC_SetFault(GNC_ACTIVE_FAULT_COMMAND_REJECTED);
            GNC_SyncRCSStatus();
            return;
        }

        last_vertical_command = vertical.valve_cmd;
    }
#endif

    {
        AttitudeControlStatus_t attitude_status =
            AttitudeControl_GetStatus();

        if (attitude_status.fault != ATT_CTRL_FAULT_NONE)
        {
            GNC_SetFault(GNC_ACTIVE_FAULT_ATTITUDE_CONTROL);
        }
    }

    GNC_SyncRCSStatus();
#endif
#endif
#endif /* APP_LEGACY_VERTICAL_LANDING_RETIRED */
}

/* -------------------------------------------------------------------------- */

void GNCActiveControl_MainLoop(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t button = GNC_ButtonRead();

    if ((button != 0U) &&
        (last_button == 0U) &&
        ((uint32_t)(now_ms - last_button_ms) >=
         APP_GNC_BUTTON_DEBOUNCE_MS))
    {
        last_button_ms = now_ms;
        GNC_HandleButton(now_ms);
    }

    last_button = button;

    /*
     * On disarm/fault, keep the healthy actuator enabled until CLOSED is
     * reached, then disable the motor driver.
     */
    if (gnc_v816_disarm_closing != 0U)
    {
        NeedleValveStatus_t needle =
            NeedleValveController_GetStatus();

        if ((needle.fault != NEEDLE_VALVE_FAULT_NONE) ||
            (needle.zero_valid == 0U))
        {
            NeedleValveController_Stop();
            gnc_v816_disarm_closing = 0U;
        }
        else if ((needle.position_locked != 0U) &&
                 (needle.limited_cmd <= 0.001f) &&
                 (needle.error_adc >= -3) &&
                 (needle.error_adc <= 3))
        {
            NeedleValveController_Stop();
            gnc_v816_disarm_closing = 0U;
        }
    }

    GNC_SyncRCSStatus();
}

void GNCActiveControl_ForceSafe(void)
{
    GNC_RequestDisarm();
}
