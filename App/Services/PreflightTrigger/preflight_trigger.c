#include "Services/PreflightTrigger/preflight_trigger.h"

#include "Common/app_config.h"
#include "Modules/Control/NeedleValve/needle_valve_controller.h"
#include "Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.h"
#include "Modules/Estimation/AttitudeEstimator/attitude_estimator.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"
#include "Services/SDLogger/sd_logger.h"
#include "main.h"

PreflightTriggerStatus_t preflight_trigger_status;

static uint32_t preflight_boot_ms;
static uint32_t raw_candidate_since_ms;
static uint32_t connected_since_ms;
static uint8_t raw_candidate_open;
static uint8_t connected_timer_active;
static uint8_t needle_zero_candidate_active;
static uint32_t needle_zero_candidate_since_ms;

static uint8_t PreflightTrigger_ReadOpen(void)
{
    /* V55 P28 confirmed harness polarity:
     * LOW = connector installed/GND, HIGH = separated/open (internal pull-up).
     * This function returns the semantic OPEN state used by the state machine. */
    return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) == GPIO_PIN_SET) ? 1U : 0U;
}

static void PreflightTrigger_ConfigurePE9(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gpio);
}

static void PreflightTrigger_UpdateReadiness(void)
{
    const SensorData_t *sensor = SensorManager_GetDataPtr();
    const AttitudeEstimatorData_t *attitude = AttitudeEstimator_GetDataPtr();
    const FullStateESKFData_t *eskf = FullStateESKF_GetDataPtr();
    NeedleValveStatus_t needle =
#if (APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE != 0U)
        NeedleValveAutonomousControl_GetTelemetryStatus();
#else
        NeedleValveController_GetStatus();
#endif

    preflight_trigger_status.imu_ready =
        ((sensor != 0) && (sensor->imu_valid != 0U)) ? 1U : 0U;

    /* Classic attitude remains online for diagnostics/generated-model input,
     * but V50 physical RCS readiness is owned by Full-State ESKF below. */
    preflight_trigger_status.attitude_ready =
        ((attitude != 0) &&
         (attitude->enabled != 0U) &&
         (attitude->initialized != 0U) &&
         (attitude->healthy != 0U)) ? 1U : 0U;

    preflight_trigger_status.eskf_ready =
        ((eskf != 0) &&
         (eskf->enabled != 0U) &&
         (eskf->initialized != 0U) &&
         (eskf->healthy != 0U) &&
         (eskf->origin_zeroed != 0U) &&
         (eskf->vertical_position_valid != 0U)) ? 1U : 0U;

    preflight_trigger_status.lidar_reference_ready =
        ((eskf != 0) &&
         (eskf->lidar_reference_ready != 0U) &&
         (eskf->lidar_fresh != 0U)) ? 1U : 0U;

    preflight_trigger_status.barometer_reference_ready =
        ((eskf != 0) &&
         (eskf->baro_reference_ready != 0U) &&
         (eskf->baro_fresh != 0U)) ? 1U : 0U;

    preflight_trigger_status.needle_ready =
        ((needle.zero_valid != 0U) &&
         ((preflight_trigger_status.flight_active != 0U) ||
          (needle.enabled == 0U)) &&
         (needle.fault == NEEDLE_VALVE_FAULT_NONE)
#if (APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE != 0U)
         && (NeedleValveAutonomousControl_IsReady() != 0U)
#endif
        ) ? 1U : 0U;

#if (APP_SDLOGGER_ENABLED != 0U)
    preflight_trigger_status.sd_ready =
        ((SDLogger_IsReady() != 0U) &&
         (SDLogger_IsLogging() != 0U)) ? 1U : 0U;
#else
    preflight_trigger_status.sd_ready = 1U;
#endif

    preflight_trigger_status.preflight_ready =
        ((preflight_trigger_status.boot_elapsed_ms >=
         APP_PREFLIGHT_MIN_CALIBRATION_MS) &&
         (preflight_trigger_status.imu_ready != 0U) &&
         (preflight_trigger_status.eskf_ready != 0U) &&
         (preflight_trigger_status.lidar_reference_ready != 0U) &&
         (preflight_trigger_status.barometer_reference_ready != 0U) &&
         (preflight_trigger_status.needle_ready != 0U) &&
         (preflight_trigger_status.sd_ready != 0U)) ? 1U : 0U;
}

void PreflightTrigger_Init(void)
{
    uint8_t *bytes = (uint8_t *)&preflight_trigger_status;
    uint32_t i;

    for (i = 0UL; i < (uint32_t)sizeof(preflight_trigger_status); i++)
    {
        bytes[i] = 0U;
    }

#if (APP_PREFLIGHT_TRIGGER_ENABLED == 0U)
    preflight_boot_ms = HAL_GetTick();
    preflight_trigger_status.preflight_ready = 1U;
    preflight_trigger_status.flight_active = 1U;
    preflight_trigger_status.state = PREFLIGHT_TRIGGER_FLIGHT_ACTIVE;
    preflight_trigger_status.separation_timestamp_ms = preflight_boot_ms;
    return;
#endif

    PreflightTrigger_ConfigurePE9();
    preflight_boot_ms = HAL_GetTick();
    preflight_trigger_status.raw_open = PreflightTrigger_ReadOpen();
    preflight_trigger_status.debounced_open =
        preflight_trigger_status.raw_open;
    raw_candidate_open = preflight_trigger_status.raw_open;
    raw_candidate_since_ms = preflight_boot_ms;
    connected_since_ms = preflight_boot_ms;
    connected_timer_active = 0U;
    needle_zero_candidate_active = 0U;
    needle_zero_candidate_since_ms = preflight_boot_ms;
    preflight_trigger_status.state = PREFLIGHT_TRIGGER_BOOT_CALIBRATION;
}

void PreflightTrigger_Update(void)
{
    uint32_t now_ms = HAL_GetTick();

#if (APP_PREFLIGHT_TRIGGER_ENABLED == 0U)
    preflight_trigger_status.flight_time_ms =
        (uint32_t)(now_ms - preflight_trigger_status.separation_timestamp_ms);
    return;
#endif

    uint8_t raw_open = PreflightTrigger_ReadOpen();

    preflight_trigger_status.boot_elapsed_ms =
        (uint32_t)(now_ms - preflight_boot_ms);
    preflight_trigger_status.raw_open = raw_open;

    if (raw_open != raw_candidate_open)
    {
        if (raw_candidate_open != preflight_trigger_status.debounced_open)
        {
            preflight_trigger_status.debounce_reject_count++;
        }

        raw_candidate_open = raw_open;
        raw_candidate_since_ms = now_ms;
        preflight_trigger_status.raw_transition_count++;
    }

    if ((raw_candidate_open != preflight_trigger_status.debounced_open) &&
        ((uint32_t)(now_ms - raw_candidate_since_ms) >=
         APP_PREFLIGHT_SEPARATION_DEBOUNCE_MS))
    {
        preflight_trigger_status.debounced_open = raw_candidate_open;
    }

    if (preflight_trigger_status.debounced_open == 0U)
    {
        if (connected_timer_active == 0U)
        {
            connected_timer_active = 1U;
            connected_since_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - connected_since_ms) >=
                 APP_PREFLIGHT_CONNECTED_CONFIRM_MS)
        {
            preflight_trigger_status.connector_seen = 1U;
        }
    }
    else
    {
        connected_timer_active = 0U;
    }

#if (APP_NEEDLE_PREFLIGHT_ZERO_CAPTURE_ENABLED != 0U)
    /* The flight build never moves the motor before separation. ZERO may be
     * captured only from an already-closed, stable potentiometer reading while
     * the physical connector is installed. */
    if ((preflight_trigger_status.connector_seen != 0U) &&
        (preflight_trigger_status.debounced_open == 0U) &&
        (preflight_trigger_status.flight_active == 0U))
    {
        NeedleValveStatus_t needle =
#if (APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE != 0U)
            NeedleValveAutonomousControl_GetTelemetryStatus();
#else
            NeedleValveController_GetStatus();
#endif

        if ((needle.zero_valid == 0U) &&
            (needle.enabled == 0U) &&
            (needle.fault == NEEDLE_VALVE_FAULT_NONE) &&
            (needle.raw_adc >= NEEDLE_VALVE_SAFE_ZERO_MIN_ADC)
#if (APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE != 0U)
            && (NeedleValveAutonomousControl_IsClosedReferenceCandidate() != 0U)
#endif
           )
        {
            if (needle_zero_candidate_active == 0U)
            {
                needle_zero_candidate_active = 1U;
                needle_zero_candidate_since_ms = now_ms;
            }
            else if ((uint32_t)(now_ms - needle_zero_candidate_since_ms) >=
                     APP_NEEDLE_PREFLIGHT_ZERO_STABLE_MS)
            {
#if (APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE != 0U)
                (void)NeedleValveAutonomousControl_CaptureClosedReference();
#else
                (void)NeedleValveController_CaptureZero();
#endif
                needle_zero_candidate_active = 0U;
            }
        }
        else
        {
            needle_zero_candidate_active = 0U;
        }
    }
    else
    {
        needle_zero_candidate_active = 0U;
    }
#endif

    PreflightTrigger_UpdateReadiness();

    if ((preflight_trigger_status.flight_active != 0U) ||
        (preflight_trigger_status.fault_latched != 0U))
    {
        if (preflight_trigger_status.flight_active != 0U)
        {
            preflight_trigger_status.flight_time_ms =
                (uint32_t)(now_ms -
                           preflight_trigger_status.separation_timestamp_ms);
        }
        return;
    }

    if (preflight_trigger_status.connector_seen == 0U)
    {
        preflight_trigger_status.state =
            (preflight_trigger_status.boot_elapsed_ms <
             APP_PREFLIGHT_MIN_CALIBRATION_MS)
            ? PREFLIGHT_TRIGGER_BOOT_CALIBRATION
            : PREFLIGHT_TRIGGER_WAIT_CONNECTOR;
        return;
    }

    if (preflight_trigger_status.preflight_ready == 0U)
    {
        preflight_trigger_status.state =
            PREFLIGHT_TRIGGER_BOOT_CALIBRATION;

        if (preflight_trigger_status.debounced_open != 0U)
        {
            preflight_trigger_status.fault_latched = 1U;
            preflight_trigger_status.state =
                PREFLIGHT_TRIGGER_EARLY_SEPARATION_FAULT;
        }
        return;
    }

    preflight_trigger_status.state = PREFLIGHT_TRIGGER_WAIT_SEPARATION;

    if (preflight_trigger_status.debounced_open != 0U)
    {
        preflight_trigger_status.flight_active = 1U;
        preflight_trigger_status.separation_timestamp_ms = now_ms;
        preflight_trigger_status.flight_time_ms = 0UL;
        preflight_trigger_status.separation_event_count++;
        preflight_trigger_status.state = PREFLIGHT_TRIGGER_FLIGHT_ACTIVE;
    }
}

uint8_t PreflightTrigger_IsFlightActive(void)
{
    return preflight_trigger_status.flight_active;
}

uint8_t PreflightTrigger_HasFault(void)
{
    return preflight_trigger_status.fault_latched;
}

PreflightTriggerStatus_t PreflightTrigger_GetStatus(void)
{
    return preflight_trigger_status;
}
