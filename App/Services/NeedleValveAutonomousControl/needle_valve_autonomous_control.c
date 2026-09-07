#include "Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.h"

#include "Common/app_config.h"
#include "Services/NeedleValveIntegrationTest/needle_valve_integration_test.h"
#include "Services/PreflightTrigger/preflight_trigger.h"
#include "main.h"

/* R16 final: flight maximum commanded travel is three mechanical turns at
 * the measured 195 ADC/turn calibration: 3 x 195 = 585 ADC. */
#define P112R5_MAX_TRAVEL_ADC                  585U
#if ((APP_P112R12R8R32_REAL_FLIGHT_LOGIC_PHYSICAL_NEEDLE_REV != 0U) && \
     (P112R5_MAX_TRAVEL_ADC != APP_R8R32_NEEDLE_MAX_TRAVEL_ADC))
#error "R16 physical needle travel must match APP_R8R32_NEEDLE_MAX_TRAVEL_ADC (585 ADC / 3 turns)."
#endif
#define P112R5_TARGET_TOLERANCE_ADC              8U
#define P112R5_COMMAND_TIMEOUT_MS               50UL
#define P112R5_MIN_TARGET_ADC                   20U
/*
 * P112R12R1 same-target mechanical guard.
 *
 * The real valve/pot/coupler assembly showed roughly +/-10 ADC of combined
 * backlash, relaxation and feedback jitter around a fixed CLOSED target. A
 * fixed +/-8 release threshold therefore caused harmless jitter to be treated
 * as real position escape and repeatedly re-triggered P110.
 *
 * Rules:
 *   - a genuinely NEW GNC target (> +/-8 ADC) bypasses this guard immediately;
 *   - for the SAME target, <=12 ADC is a firm HOLD region;
 *   - 13..20 ADC is a mechanical deadband/HOLD region;
 *   - >20 ADC must persist continuously for 200 ms before a correction move.
 *
 * This guard only filters re-corrections toward an unchanged target. It does
 * not slow a real GNC retarget.
 */
#define P112R12R1_SAME_TARGET_HOLD_ADC            12U
#define P112R12R1_SAME_TARGET_DEADBAND_ADC        20U
#define P112R12R1_SAME_TARGET_CONFIRM_MS         200UL

/* R8R35R2: a completed move already measured the real mechanism stopping
 * quantum. For an unchanged target, do not launch a new breakaway event until
 * the error exceeds that measured quantum plus a small margin. This preserves
 * immediate response to a genuine GNC retarget while preventing a tiny HOLD
 * correction from becoming a 20+ ADC mechanical overshoot. */
#define P112R35R2_SETTLED_STOP_MARGIN_ADC           4U
#define P112R35R2_SETTLED_STOP_CAP_ADC             48U

/* R8R35R2: P83 quarantine while the actuator is already stationary is a
 * no-motion condition. Hold safely and allow a short recovery window; a P83
 * failure DURING a P110 move is still aborted immediately by P110. */
#define P112R35R2_HOLD_FB_INVALID_LATCH_MS        500UL

#define P112R5_SUP_RESULT_RUNNING                0U
#define P112R5_SUP_RESULT_HEALTHY                1U
#define P112R5_SUP_RESULT_FAULT                  2U
#define P112R5_SUP_RESULT_NOT_READY              3U

static volatile NeedleValveAutonomousStatus_t autonomous_status;
static volatile uint8_t autonomous_command_valid;
static volatile uint8_t autonomous_move_direction;
static volatile uint8_t autonomous_settled_hold_valid;
static volatile uint16_t autonomous_settled_hold_target_adc;
static volatile uint16_t autonomous_settled_hold_position_adc;
static volatile uint16_t autonomous_settled_hold_stop_distance_adc;
static volatile uint32_t autonomous_hold_feedback_invalid_since_ms;
static volatile uint8_t autonomous_same_target_excursion_active;
static volatile uint32_t autonomous_same_target_excursion_since_ms;

/* R8R16 one-shot E-STOP emergency-close state. Failure reasons:
 * 0 none, 1 no CLOSED reference, 2 feedback invalid, 3 reserved legacy,
 * 4 reserved legacy, 5 timeout, 6 close-command reject,
 * 7 emergency retry faulted again / non-overridable actuator fault. */
static volatile uint8_t estop_safe_close_active;
static volatile uint8_t estop_safe_close_complete;
static volatile uint8_t estop_safe_close_failed;
static volatile uint8_t estop_safe_close_fail_reason;
static volatile uint32_t estop_safe_close_start_ms;
static volatile uint32_t estop_safe_close_start_count;
static volatile uint32_t estop_safe_close_last_refresh_ms;
static volatile uint8_t estop_safe_close_override_used;
/* R8R35R3R10R2 bounded live-feedback recovery state. */
static volatile uint32_t estop_safe_close_feedback_invalid_since_ms;
static volatile uint32_t estop_safe_close_feedback_valid_since_ms;
static volatile uint32_t estop_safe_close_feedback_reacquire_count;
static volatile uint32_t flight_feedback_recovery_valid_since_ms;
static volatile uint32_t flight_feedback_recovery_count;

#if (APP_NEEDLE_P112R11_MECH_COMMISSION_MODE != 0U)
#define P112R11_COMMISSION_OPEN_COMMAND \
    ((float)APP_P112R11_OPEN_TRAVEL_ADC / (float)P112R5_MAX_TRAVEL_ADC)

#define P112R11_WAIT_SAFE   0U
#define P112R11_ARMED       1U
#define P112R11_OPENING     2U
#define P112R11_HOLD_OPEN   3U
#define P112R11_CLOSING     4U
#define P112R11_DONE        5U
#define P112R11_ABORT       6U

/* R11R1 commissioning abort reasons, exported read-only on UART. */
#define P112R11_ABORT_NONE                 0U
#define P112R11_ABORT_PE9_OR_CONNECTOR     1U
#define P112R11_ABORT_FLIGHT_ACTIVE        2U
#define P112R11_ABORT_AUTONOMOUS_FAULT     3U
#define P112R11_ABORT_FEEDBACK             4U
#define P112R11_ABORT_NEEDLE_FAULT         5U
#define P112R11_ABORT_TIMEOUT              6U
#define P112R11_ABORT_OPERATOR_PA0         7U
#define P112R11_ABORT_OPEN_SUBMIT          8U
#define P112R11_ABORT_HOLD_SUBMIT          9U
#define P112R11_ABORT_CLOSE_SUBMIT        10U

volatile uint8_t p112r11_commission_state = P112R11_WAIT_SAFE;
volatile uint8_t p112r11_button_raw = 0U;
volatile uint8_t p112r11_button_debounced = 0U;
volatile uint8_t p112r11_button_armed = 0U;
volatile uint8_t p112r11_single_shot_latched = 0U;
volatile uint8_t p112r11_done_latched = 0U;
volatile uint32_t p112r11_abort_count = 0UL;
volatile uint8_t p112r11_abort_reason = P112R11_ABORT_NONE;
volatile uint16_t p112r11_open_target_adc = 0U;

static uint8_t p112r11_button_candidate = 0U;
static uint32_t p112r11_button_candidate_since_ms = 0UL;
static uint32_t p112r11_button_release_since_ms = 0UL;
static uint32_t p112r11_sequence_start_ms = 0UL;
static uint32_t p112r11_phase_start_ms = 0UL;
static uint32_t p112r11_move_base = 0UL;

static void P112R11_ButtonInit(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static uint8_t P112R11_ButtonRead(void)
{
    return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) ? 1U : 0U;
}

static uint8_t P112R11_ButtonUpdate(uint32_t now, uint8_t raw)
{
    uint8_t press_edge = 0U;
    if (raw != p112r11_button_candidate)
    {
        p112r11_button_candidate = raw;
        p112r11_button_candidate_since_ms = now;
    }
    if ((p112r11_button_debounced != p112r11_button_candidate) &&
        ((uint32_t)(now - p112r11_button_candidate_since_ms) >=
         APP_P112R11_BUTTON_DEBOUNCE_MS))
    {
        p112r11_button_debounced = p112r11_button_candidate;
        if (p112r11_button_debounced != 0U) press_edge = 1U;
        else p112r11_button_release_since_ms = now;
    }
    return press_edge;
}

static uint8_t P112R11_CommonSafetyOk(PreflightTriggerStatus_t pf)
{
    NeedleValveAutonomousStatus_t a = NeedleValveAutonomousControl_GetStatus();
    if ((pf.connector_seen == 0U) ||
        (pf.debounced_open != 0U) ||
        (pf.flight_active != 0U) ||
        (pf.fault_latched != 0U) ||
        (a.reference_valid == 0U) ||
        (a.fault != NEEDLE_AUTONOMOUS_FAULT_NONE) ||
        (p83_feedback_valid == 0U) ||
        (p83_mode != 1U) ||
        (needle_valve_fault != (uint8_t)NEEDLE_VALVE_FAULT_NONE))
        return 0U;
    return 1U;
}

static uint8_t P112R11_SafeToStart(void)
{
    PreflightTriggerStatus_t pf = PreflightTrigger_GetStatus();
    NeedleValveAutonomousStatus_t a = NeedleValveAutonomousControl_GetStatus();
    if (P112R11_CommonSafetyOk(pf) == 0U) return 0U;
    if (pf.preflight_ready == 0U) return 0U;
    if ((uint16_t)((p83_filtered_adc >= a.closed_reference_adc) ?
         (p83_filtered_adc - a.closed_reference_adc) :
         (a.closed_reference_adc - p83_filtered_adc)) >
        (uint16_t)P112R5_TARGET_TOLERANCE_ADC) return 0U;
    return 1U;
}

static uint8_t P112R11_MotionAbortReason(void)
{
    PreflightTriggerStatus_t pf = PreflightTrigger_GetStatus();
    NeedleValveAutonomousStatus_t a = NeedleValveAutonomousControl_GetStatus();

    /*
     * R11R1: full sensor/estimator/SD preflight readiness is still mandatory
     * before arming, but once this depressurized commissioning stroke starts
     * only actuator-critical interlocks may abort motion. IMU/ESKF/LiDAR/
     * barometer/SD reference-readiness flicker is diagnostic-only here and
     * must not strand the valve at the short OPEN target.
     */
    if ((pf.connector_seen == 0U) || (pf.debounced_open != 0U))
        return P112R11_ABORT_PE9_OR_CONNECTOR;
    if (pf.flight_active != 0U)
        return P112R11_ABORT_FLIGHT_ACTIVE;
    if ((a.reference_valid == 0U) ||
        (a.fault != NEEDLE_AUTONOMOUS_FAULT_NONE))
        return P112R11_ABORT_AUTONOMOUS_FAULT;
    if ((p83_feedback_valid == 0U) || (p83_mode != 1U))
        return P112R11_ABORT_FEEDBACK;
    if (needle_valve_fault != (uint8_t)NEEDLE_VALVE_FAULT_NONE)
        return P112R11_ABORT_NEEDLE_FAULT;
    return P112R11_ABORT_NONE;
}

static void P112R11_Abort(uint8_t reason)
{
    NeedleValveAutonomousControl_RevokeAuthorization();
    NeedleValveIntegrationTest_ForceSafe(0U);
    p112r11_abort_reason =
        (reason == P112R11_ABORT_NONE) ? P112R11_ABORT_AUTONOMOUS_FAULT : reason;
    p112r11_commission_state = P112R11_ABORT;
    p112r11_button_armed = 0U;
    p112r11_single_shot_latched = 1U;
    p112r11_abort_count++;
}
#endif

static uint16_t P112R5_AbsDiffU16(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static uint16_t P112R5_CommandToTarget(float command, uint16_t zero_adc)
{
    uint32_t travel = (uint32_t)(command * (float)P112R5_MAX_TRAVEL_ADC + 0.5f);
    uint32_t target;

    if (travel > P112R5_MAX_TRAVEL_ADC) travel = P112R5_MAX_TRAVEL_ADC;
    target = (travel < zero_adc) ? ((uint32_t)zero_adc - travel) : 0UL;
    if (target < P112R5_MIN_TARGET_ADC) target = P112R5_MIN_TARGET_ADC;
    if (target > 1023UL) target = 1023UL;
    return (uint16_t)target;
}

static void P112R5_PublishCompatibilityTelemetry(void)
{
    uint8_t learned_open_pwm;
    uint8_t learned_close_pwm;
    uint16_t learned_open_coast;
    uint16_t learned_close_coast;

    NeedleValveIntegrationTest_GetAdaptiveLearning(
        &learned_open_pwm, &learned_close_pwm,
        &learned_open_coast, &learned_close_coast);

    p111_state = (uint8_t)autonomous_status.state;
    p111_result = (autonomous_status.fault == NEEDLE_AUTONOMOUS_FAULT_NONE) ?
        ((autonomous_status.reference_valid != 0U) ?
         P112R5_SUP_RESULT_HEALTHY : P112R5_SUP_RESULT_NOT_READY) :
        P112R5_SUP_RESULT_FAULT;
    p111_baseline_adc = autonomous_status.closed_reference_adc;
    p111_open_target_adc = autonomous_status.requested_target_adc;
    p111_cycle = (uint8_t)(autonomous_status.command_sequence & 0xFFUL);
    p111_completed_cycles = (uint8_t)(autonomous_status.moves_completed & 0xFFUL);
    p111_moves_completed = (uint8_t)(autonomous_status.moves_completed & 0xFFUL);
    p111_pass_mask = (autonomous_status.moves_completed == 0UL) ? 0U :
        (uint8_t)((autonomous_status.moves_completed >= 8UL) ? 0xFFU :
                  ((1UL << autonomous_status.moves_completed) - 1UL));
    p111_phase = autonomous_move_direction;
    p111_learned_open_pwm = learned_open_pwm;
    p111_learned_close_pwm = learned_close_pwm;
    p111_learned_open_coast = learned_open_coast;
    p111_learned_close_coast = learned_close_coast;
    p111_abort_reason = (uint8_t)autonomous_status.fault;
}

static void P112R5_RecordCompletedMove(void)
{
    if (autonomous_move_direction == NEEDLE_ADAPTIVE_DIR_OPEN)
    {
        p111_last_open_error_adc = p110_final_error_adc;
        p111_last_open_powered_ms = p110_total_powered_ms;
        p111_last_open_breakaway_pwm = p110_learned_breakaway_pwm;
        p111_last_open_stop_adc = p110_stop_distance_adc;
        p111_last_open_coast_adc = p110_coast_max_adc;
    }
    else
    {
        p111_last_close_error_adc = p110_final_error_adc;
        p111_last_close_powered_ms = p110_total_powered_ms;
        p111_last_close_breakaway_pwm = p110_learned_breakaway_pwm;
        p111_last_close_stop_adc = p110_stop_distance_adc;
        p111_last_close_coast_adc = p110_coast_max_adc;
    }
}

static void P112R5_LatchFault(NeedleValveAutonomousFault_t fault)
{
    autonomous_status.fault = fault;
    autonomous_status.state = NEEDLE_AUTONOMOUS_FAULT;
    autonomous_status.command_authorized = 0U;
    autonomous_status.command_fresh = 0U;
    autonomous_status.move_in_progress = 0U;
    autonomous_command_valid = 0U;
    autonomous_settled_hold_valid = 0U;
    autonomous_settled_hold_target_adc = 0U;
    autonomous_settled_hold_position_adc = 0U;
    autonomous_settled_hold_stop_distance_adc = P112R12R1_SAME_TARGET_DEADBAND_ADC;
    autonomous_hold_feedback_invalid_since_ms = 0UL;
    autonomous_same_target_excursion_active = 0U;
    autonomous_same_target_excursion_since_ms = 0UL;
    needle_valve_fault = (fault == NEEDLE_AUTONOMOUS_FAULT_FEEDBACK) ?
        (uint8_t)NEEDLE_VALVE_FAULT_ADC_INVALID :
        (uint8_t)NEEDLE_VALVE_FAULT_STALL;
    NeedleValveIntegrationTest_ForceSafe((uint8_t)fault);
    P112R5_PublishCompatibilityTelemetry();
}

void NeedleValveAutonomousControl_Init(void)
{
    estop_safe_close_active = 0U;
    estop_safe_close_complete = 0U;
    estop_safe_close_failed = 0U;
    estop_safe_close_fail_reason = 0U;
    estop_safe_close_start_ms = 0UL;
    estop_safe_close_start_count = 0UL;
    estop_safe_close_last_refresh_ms = 0UL;
    estop_safe_close_override_used = 0U;
    estop_safe_close_feedback_invalid_since_ms = 0UL;
    estop_safe_close_feedback_valid_since_ms = 0UL;
    estop_safe_close_feedback_reacquire_count = 0UL;
    flight_feedback_recovery_valid_since_ms = 0UL;
    flight_feedback_recovery_count = 0UL;
    volatile uint8_t *bytes = (volatile uint8_t *)&autonomous_status;
    uint32_t i;

    for (i = 0UL; i < (uint32_t)sizeof(autonomous_status); i++) bytes[i] = 0U;

    autonomous_status.state = NEEDLE_AUTONOMOUS_WAIT_REFERENCE;
    autonomous_status.fault = NEEDLE_AUTONOMOUS_FAULT_NONE;
    autonomous_command_valid = 0U;
    autonomous_move_direction = NEEDLE_ADAPTIVE_DIR_OPEN;
    autonomous_settled_hold_valid = 0U;
    autonomous_settled_hold_target_adc = 0U;
    autonomous_settled_hold_position_adc = 0U;
    autonomous_settled_hold_stop_distance_adc = P112R12R1_SAME_TARGET_DEADBAND_ADC;
    autonomous_hold_feedback_invalid_since_ms = 0UL;
    autonomous_same_target_excursion_active = 0U;
    autonomous_same_target_excursion_since_ms = 0UL;
    needle_valve_fault = (uint8_t)NEEDLE_VALVE_FAULT_NONE;
    NeedleValveIntegrationTest_ForceSafe(0U);
#if (APP_NEEDLE_P112R11_MECH_COMMISSION_MODE != 0U)
    P112R11_ButtonInit();
    p112r11_commission_state = P112R11_WAIT_SAFE;
    p112r11_button_raw = P112R11_ButtonRead();
    p112r11_button_debounced = p112r11_button_raw;
    p112r11_button_candidate = p112r11_button_raw;
    p112r11_button_candidate_since_ms = HAL_GetTick();
    p112r11_button_release_since_ms = HAL_GetTick();
    p112r11_button_armed = 0U;
    p112r11_single_shot_latched = 0U;
    p112r11_done_latched = 0U;
    p112r11_abort_count = 0UL;
    p112r11_abort_reason = P112R11_ABORT_NONE;
    p112r11_open_target_adc = 0U;
#endif
    P112R5_PublishCompatibilityTelemetry();
}

uint8_t NeedleValveAutonomousControl_IsClosedReferenceCandidate(void)
{
    if ((p83_feedback_valid == 0U) || (p83_mode != 1U) ||
        (p83_filtered_adc < NEEDLE_VALVE_SAFE_ZERO_MIN_ADC) ||
        (p110_state == NEEDLE_ADAPTIVE_STATE_SEARCH) ||
        (p110_state == NEEDLE_ADAPTIVE_STATE_DRIVE) ||
        (p110_state == NEEDLE_ADAPTIVE_STATE_BRAKE))
    {
        return 0U;
    }
    return 1U;
}

uint8_t NeedleValveAutonomousControl_CaptureClosedReference(void)
{
    uint32_t primask;

    if (NeedleValveAutonomousControl_IsClosedReferenceCandidate() == 0U)
    {
        autonomous_status.command_reject_count++;
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    autonomous_status.closed_reference_adc = p83_filtered_adc;
    autonomous_status.requested_target_adc = p83_filtered_adc;
    autonomous_status.requested_command = 0.0f;
    autonomous_status.reference_valid = 1U;
    autonomous_status.command_authorized = 0U;
    autonomous_status.command_fresh = 0U;
    autonomous_status.state = NEEDLE_AUTONOMOUS_READY;
    autonomous_command_valid = 0U;
    autonomous_settled_hold_valid = 0U;
    autonomous_settled_hold_target_adc = p83_filtered_adc;
    autonomous_settled_hold_position_adc = p83_filtered_adc;
    autonomous_settled_hold_stop_distance_adc = P112R12R1_SAME_TARGET_DEADBAND_ADC;
    autonomous_hold_feedback_invalid_since_ms = 0UL;
    autonomous_same_target_excursion_active = 0U;
    autonomous_same_target_excursion_since_ms = 0UL;
    if (primask == 0U) __enable_irq();

    P112R5_PublishCompatibilityTelemetry();
    return 1U;
}

uint8_t NeedleValveAutonomousControl_SubmitCommand(float command_0_to_1)
{
    uint16_t target;
    uint16_t previous_target;
    uint32_t primask;

    if ((command_0_to_1 != command_0_to_1) ||
        (command_0_to_1 < 0.0f) || (command_0_to_1 > 1.0f) ||
        (autonomous_status.reference_valid == 0U) ||
        (autonomous_status.fault != NEEDLE_AUTONOMOUS_FAULT_NONE))
    {
        autonomous_status.command_reject_count++;
        return 0U;
    }

    target = P112R5_CommandToTarget(
        command_0_to_1, autonomous_status.closed_reference_adc);
    if ((target < P112R5_MIN_TARGET_ADC) ||
        (target > autonomous_status.closed_reference_adc))
    {
        autonomous_status.command_reject_count++;
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    previous_target = autonomous_status.requested_target_adc;
    autonomous_status.requested_command = command_0_to_1;
    autonomous_status.requested_target_adc = target;
    autonomous_status.last_command_ms = HAL_GetTick();
    autonomous_status.command_sequence++;
    autonomous_status.command_authorized = 1U;
    autonomous_status.command_fresh = 1U;
    autonomous_command_valid = 1U;

    /* A real target change must react immediately. Repeated copies of the
     * same target arm/retain the mechanical jitter guard instead. */
    if (P112R5_AbsDiffU16(target, previous_target) >
        P112R5_TARGET_TOLERANCE_ADC)
    {
        autonomous_settled_hold_valid = 0U;
        autonomous_same_target_excursion_active = 0U;
        autonomous_same_target_excursion_since_ms = 0UL;
    }
    else if ((autonomous_settled_hold_valid == 0U) &&
             (autonomous_status.move_in_progress == 0U) &&
             (p83_feedback_valid != 0U) && (p83_mode == 1U))
    {
        autonomous_settled_hold_valid = 1U;
        autonomous_settled_hold_target_adc = target;
        autonomous_settled_hold_position_adc = p83_filtered_adc;
        autonomous_hold_feedback_invalid_since_ms = 0UL;
        autonomous_same_target_excursion_active = 0U;
        autonomous_same_target_excursion_since_ms = 0UL;
    }
    if (primask == 0U) __enable_irq();

    return 1U;
}

void NeedleValveAutonomousControl_RevokeAuthorization(void)
{
    uint8_t was_active;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    was_active = (uint8_t)(autonomous_status.command_authorized |
                           autonomous_status.move_in_progress);
    autonomous_status.command_authorized = 0U;
    autonomous_status.command_fresh = 0U;
    autonomous_status.move_in_progress = 0U;
    autonomous_command_valid = 0U;
    autonomous_settled_hold_valid = 0U;
    autonomous_hold_feedback_invalid_since_ms = 0UL;
    autonomous_same_target_excursion_active = 0U;
    autonomous_same_target_excursion_since_ms = 0UL;
    if ((autonomous_status.reference_valid != 0U) &&
        (autonomous_status.fault == NEEDLE_AUTONOMOUS_FAULT_NONE))
    {
        autonomous_status.state = NEEDLE_AUTONOMOUS_REVOKED;
    }
    if (primask == 0U) __enable_irq();

    if (was_active != 0U) NeedleValveIntegrationTest_ForceSafe(0U);
    P112R5_PublishCompatibilityTelemetry();
}

static void P112R16_EStopSafeCloseFail(uint8_t reason)
{
    estop_safe_close_active = 0U;
    estop_safe_close_complete = 0U;
    estop_safe_close_failed = 1U;
    estop_safe_close_fail_reason = reason;
    NeedleValveAutonomousControl_RevokeAuthorization();
    NeedleValveIntegrationTest_ForceSafe(0U);
}

static uint8_t P112R16_EStopFaultsAreTransientAndOverridable(void)
{
    /* R3R10R2: this function is reached only after EStopSafeCloseUpdate has
     * re-validated live P83 feedback continuously. A stale FEEDBACK /
     * ADC_INVALID latch may therefore be cleared once for CLOSED only.
     * Reference loss remains non-overridable. */
    if (autonomous_status.fault == NEEDLE_AUTONOMOUS_FAULT_REFERENCE)
    {
        return 0U;
    }

    if ((needle_valve_fault != (uint8_t)NEEDLE_VALVE_FAULT_NONE) &&
        (needle_valve_fault != (uint8_t)NEEDLE_VALVE_FAULT_STALL) &&
        (needle_valve_fault != (uint8_t)NEEDLE_VALVE_FAULT_TARGET_RANGE) &&
        (needle_valve_fault != (uint8_t)NEEDLE_VALVE_FAULT_ADC_INVALID))
    {
        return 0U;
    }

    return 1U;
}

static uint8_t P112R16_PrepareEmergencyCloseOverride(void)
{
    uint32_t primask;

    if (estop_safe_close_override_used != 0U) return 0U;
    if (P112R16_EStopFaultsAreTransientAndOverridable() == 0U) return 0U;

    /* Abort any stale low-level motion before re-arming exactly one CLOSED
     * attempt. This does not restore normal flight authorization. */
    NeedleValveIntegrationTest_ForceSafe(0U);

    primask = __get_PRIMASK();
    __disable_irq();
    autonomous_status.fault = NEEDLE_AUTONOMOUS_FAULT_NONE;
    autonomous_status.state = NEEDLE_AUTONOMOUS_REVOKED;
    autonomous_status.command_authorized = 0U;
    autonomous_status.command_fresh = 0U;
    autonomous_status.move_in_progress = 0U;
    autonomous_command_valid = 0U;
    autonomous_settled_hold_valid = 0U;
    autonomous_same_target_excursion_active = 0U;
    autonomous_same_target_excursion_since_ms = 0UL;
    needle_valve_fault = (uint8_t)NEEDLE_VALVE_FAULT_NONE;
    if (primask == 0U) __enable_irq();

    estop_safe_close_override_used = 1U;
    P112R5_PublishCompatibilityTelemetry();
    return 1U;
}

void NeedleValveAutonomousControl_EStopSafeCloseUpdate(void)
{
#if (APP_P112R12R8R15_ESTOP_SAFE_CLOSE_REV != 0U) && \
    (APP_P112R12R8R16_ESTOP_EMERGENCY_CLOSE_OVERRIDE_REV != 0U)
    const uint32_t now = HAL_GetTick();
    uint16_t current;

    /* One-shot behavior: once CLOSED or failed, STOP remains latched and the
     * motor remains de-energized. */
    if ((estop_safe_close_complete != 0U) ||
        (estop_safe_close_failed != 0U))
    {
        NeedleValveIntegrationTest_ForceSafe(0U);
        return;
    }

    if (estop_safe_close_active == 0U)
    {
        estop_safe_close_active = 1U;
        estop_safe_close_start_ms = now;
        estop_safe_close_start_count++;
    }

    if (autonomous_status.reference_valid == 0U)
    {
        P112R16_EStopSafeCloseFail(1U);
        return;
    }

#if (APP_P112R12R8R17_ESTOP_FEEDBACK_REACQUIRE_REV != 0U)
    /* Never move with invalid feedback. A transient P83 quarantine at the
     * instant STOP arrives is treated as a motor-OFF reacquire state instead
     * of an immediate permanent E-STOP close failure. */
    if ((p83_feedback_valid == 0U) || (p83_mode != 1U))
    {
        NeedleValveIntegrationTest_ForceSafe(0U);
        estop_safe_close_feedback_valid_since_ms = 0UL;
        if (estop_safe_close_feedback_invalid_since_ms == 0UL)
            estop_safe_close_feedback_invalid_since_ms = now;

        if ((uint32_t)(now - estop_safe_close_feedback_invalid_since_ms) >=
            APP_ESTOP_SAFE_CLOSE_REACQUIRE_TIMEOUT_MS)
        {
            P112R16_EStopSafeCloseFail(2U);
        }
        return;
    }

    if (estop_safe_close_feedback_invalid_since_ms != 0UL)
    {
        estop_safe_close_feedback_invalid_since_ms = 0UL;
        estop_safe_close_feedback_reacquire_count++;
    }
    if (estop_safe_close_feedback_valid_since_ms == 0UL)
    {
        estop_safe_close_feedback_valid_since_ms = now;
        NeedleValveIntegrationTest_ForceSafe(0U);
        return;
    }
    if ((uint32_t)(now - estop_safe_close_feedback_valid_since_ms) <
        APP_ESTOP_SAFE_CLOSE_VALID_STABLE_MS)
    {
        NeedleValveIntegrationTest_ForceSafe(0U);
        return;
    }
#else
    if ((p83_feedback_valid == 0U) || (p83_mode != 1U))
    {
        P112R16_EStopSafeCloseFail(2U);
        return;
    }
#endif

    current = p83_filtered_adc;

    /* R8R16: CLOSED is the physical safety objective. If feedback proves that
     * the valve is already CLOSED, a stale/transient actuator fault must not
     * turn a physically-safe condition into a reported safe-close failure. */
    if (P112R5_AbsDiffU16(current, autonomous_status.closed_reference_adc) <=
        APP_ESTOP_SAFE_CLOSE_TOLERANCE_ADC)
    {
        NeedleValveAutonomousControl_RevokeAuthorization();
        NeedleValveIntegrationTest_ForceSafe(0U);
        estop_safe_close_active = 0U;
        estop_safe_close_complete = 1U;
        estop_safe_close_failed = 0U;
        estop_safe_close_fail_reason = 0U;
        return;
    }

    if ((uint32_t)(now - estop_safe_close_start_ms) >
        APP_ESTOP_SAFE_CLOSE_TIMEOUT_MS)
    {
        P112R16_EStopSafeCloseFail(5U);
        return;
    }

    /* If normal actuator control faulted before E-STOP, do not simply abandon
     * an open valve. With valid reference+feedback, clear one transient
     * STALL/LOW_LEVEL condition and give CLOSED one bounded emergency retry.
     * If that retry faults again, the next pass fails and de-energizes. */
    if ((autonomous_status.fault != NEEDLE_AUTONOMOUS_FAULT_NONE) ||
        (needle_valve_fault != (uint8_t)NEEDLE_VALVE_FAULT_NONE))
    {
        if (autonomous_status.fault == NEEDLE_AUTONOMOUS_FAULT_REFERENCE)
        {
            P112R16_EStopSafeCloseFail(1U);
            return;
        }
        /* R3R10R2: FEEDBACK/ADC_INVALID may be stale historical latches.
         * The live P83 stable gate above has already re-proven position, so
         * exactly one CLOSED-only override is allowed. Any second fault below
         * fails via override_used and de-energizes the motor. */
        if ((estop_safe_close_override_used != 0U) ||
            (P112R16_PrepareEmergencyCloseOverride() == 0U))
        {
            P112R16_EStopSafeCloseFail(7U);
            return;
        }
    }

    /* Deliberate exception to normal flight authorization: after E-STOP, only
     * command 0.0/CLOSED may be refreshed. All adaptive breakaway/coast/
     * feedback protections underneath remain unchanged. */
    if ((autonomous_status.command_authorized == 0U) ||
        (autonomous_command_valid == 0U) ||
        (autonomous_status.requested_target_adc !=
         autonomous_status.closed_reference_adc) ||
        (estop_safe_close_last_refresh_ms == 0UL) ||
        ((uint32_t)(now - estop_safe_close_last_refresh_ms) >= 5UL))
    {
        if (NeedleValveAutonomousControl_SubmitCommand(0.0f) == 0U)
        {
            P112R16_EStopSafeCloseFail(6U);
            return;
        }
        estop_safe_close_last_refresh_ms = now;

        /* E-STOP must not inherit the normal 200 ms same-target jitter dwell. */
        {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            autonomous_settled_hold_valid = 0U;
            autonomous_same_target_excursion_active = 0U;
            autonomous_same_target_excursion_since_ms = 0UL;
            if (primask == 0U) __enable_irq();
        }
    }
#else
    NeedleValveAutonomousControl_RevokeAuthorization();
#endif
}

uint8_t NeedleValveAutonomousControl_IsEStopSafeCloseActive(void)
{
    return estop_safe_close_active;
}

uint8_t NeedleValveAutonomousControl_IsEStopSafeCloseComplete(void)
{
    return estop_safe_close_complete;
}

uint8_t NeedleValveAutonomousControl_HasEStopSafeCloseFailed(void)
{
    return estop_safe_close_failed;
}

uint8_t NeedleValveAutonomousControl_GetEStopSafeCloseFailReason(void)
{
    return estop_safe_close_fail_reason;
}

uint32_t NeedleValveAutonomousControl_GetEStopSafeCloseStartCount(void)
{
    return estop_safe_close_start_count;
}

uint32_t NeedleValveAutonomousControl_GetEStopSafeCloseElapsedMs(void)
{
    if (estop_safe_close_active == 0U) return 0UL;
    return (uint32_t)(HAL_GetTick() - estop_safe_close_start_ms);
}

uint8_t NeedleValveAutonomousControl_WasEStopSafeCloseOverrideUsed(void)
{
    return estop_safe_close_override_used;
}

void NeedleValveAutonomousControl_TimerTickISR(void)
{
    uint32_t now;
    uint16_t current;
    uint16_t target;

    if ((autonomous_status.reference_valid == 0U) ||
        (autonomous_status.fault != NEEDLE_AUTONOMOUS_FAULT_NONE))
    {
        return;
    }

    if ((autonomous_status.command_authorized == 0U) ||
        (autonomous_command_valid == 0U))
    {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - autonomous_status.last_command_ms) >
        P112R5_COMMAND_TIMEOUT_MS)
    {
        autonomous_status.command_timeout_count++;
        P112R5_LatchFault(NEEDLE_AUTONOMOUS_FAULT_COMMAND_TIMEOUT);
        return;
    }
    autonomous_status.command_fresh = 1U;

    if (autonomous_status.move_in_progress != 0U)
    {
        if (((p110_state == NEEDLE_ADAPTIVE_STATE_SEARCH) ||
             (p110_state == NEEDLE_ADAPTIVE_STATE_DRIVE) ||
             (p110_state == NEEDLE_ADAPTIVE_STATE_CORRECTION_DWELL)) &&
            (P112R5_AbsDiffU16(p110_target_adc,
                autonomous_status.requested_target_adc) >
             P112R5_TARGET_TOLERANCE_ADC))
        {
            (void)NeedleValveIntegrationTest_UpdateActiveTarget(
                autonomous_status.requested_target_adc);
        }

        if (p110_state != NEEDLE_ADAPTIVE_STATE_DONE) return;

        if (p110_result != NEEDLE_ADAPTIVE_RESULT_PASS)
        {
            /* P112R12R6R2 retained by R12R8: preserve the low-level abort
             * root cause. A feedback-invalid P110 abort is a P111 FEEDBACK
             * fault; unrelated low-level failures retain LOW_LEVEL. */
            if ((p110_result == NEEDLE_ADAPTIVE_RESULT_ABORT) &&
                (p110_abort_reason == NEEDLE_ADAPTIVE_ABORT_FEEDBACK_INVALID))
            {
                P112R5_LatchFault(NEEDLE_AUTONOMOUS_FAULT_FEEDBACK);
            }
            else
            {
                P112R5_LatchFault(NEEDLE_AUTONOMOUS_FAULT_LOW_LEVEL);
            }
            return;
        }

        if (p110_abort_reason != NEEDLE_ADAPTIVE_ABORT_RETARGET)
        {
            P112R5_RecordCompletedMove();
            autonomous_status.moves_completed++;
            autonomous_settled_hold_target_adc = p110_target_adc;
            autonomous_settled_hold_position_adc = p83_filtered_adc;
            autonomous_settled_hold_stop_distance_adc = p110_stop_distance_adc;
            autonomous_settled_hold_valid = 1U;
            autonomous_same_target_excursion_active = 0U;
            autonomous_same_target_excursion_since_ms = 0UL;
        }
        else
        {
            autonomous_settled_hold_valid = 0U;
            autonomous_same_target_excursion_active = 0U;
            autonomous_same_target_excursion_since_ms = 0UL;
        }
        autonomous_status.move_in_progress = 0U;
        autonomous_status.state = NEEDLE_AUTONOMOUS_HOLD;
    }

    if ((p83_feedback_valid == 0U) || (p83_mode != 1U))
    {
        /* R8R35R3R10R3: a P83 quarantine while the actuator is already
         * stationary at the last confirmed SAME-target HOLD position is not
         * evidence that a new motor move is required. Keep the motor forced
         * OFF and wait for feedback to reacquire without converting that
         * historical hold into a permanent P111 fault.
         *
         * Safety boundary: this protection applies only while the last known
         * settled target/position pair was within the measured hold cap. A
         * genuine GNC target change clears autonomous_settled_hold_valid in
         * SubmitCommand(); then the existing 500 ms invalid-feedback fault
         * path below is retained. Active P110 motion still aborts immediately
         * in P110_SequenceService(). */
        if ((autonomous_settled_hold_valid != 0U) &&
            (P112R5_AbsDiffU16(autonomous_settled_hold_position_adc,
                               autonomous_settled_hold_target_adc) <=
             P112R35R2_SETTLED_STOP_CAP_ADC))
        {
            if (autonomous_hold_feedback_invalid_since_ms == 0UL)
                autonomous_hold_feedback_invalid_since_ms = now;
            NeedleValveIntegrationTest_ForceSafe(0U);
            autonomous_status.move_in_progress = 0U;
            autonomous_status.state = NEEDLE_AUTONOMOUS_HOLD;
            return;
        }

        if (autonomous_hold_feedback_invalid_since_ms == 0UL)
            autonomous_hold_feedback_invalid_since_ms = now;
        autonomous_status.state = NEEDLE_AUTONOMOUS_HOLD;
        if ((uint32_t)(now - autonomous_hold_feedback_invalid_since_ms) >=
            P112R35R2_HOLD_FB_INVALID_LATCH_MS)
        {
            P112R5_LatchFault(NEEDLE_AUTONOMOUS_FAULT_FEEDBACK);
        }
        return;
    }
    autonomous_hold_feedback_invalid_since_ms = 0UL;

    current = p83_filtered_adc;
    target = autonomous_status.requested_target_adc;

    /*
     * P112R12R1: filter only SAME-target mechanical jitter/backlash. A real
     * GNC retarget clears this latch in SubmitCommand(), so it reaches P110 on
     * the very next control tick. For an unchanged target, a correction is
     * allowed only after error exceeds the measured stopping quantum
     * (+margin, bounded 20..48 ADC) continuously for 200 ms.
     */
    if (autonomous_settled_hold_valid != 0U)
    {
        uint16_t same_target_error = P112R5_AbsDiffU16(current, target);
        uint16_t same_target_deadband = autonomous_settled_hold_stop_distance_adc;
        if (same_target_deadband < P112R12R1_SAME_TARGET_DEADBAND_ADC)
            same_target_deadband = P112R12R1_SAME_TARGET_DEADBAND_ADC;
        if (same_target_deadband < (uint16_t)(65535U - P112R35R2_SETTLED_STOP_MARGIN_ADC))
            same_target_deadband = (uint16_t)(same_target_deadband +
                P112R35R2_SETTLED_STOP_MARGIN_ADC);
        if (same_target_deadband > P112R35R2_SETTLED_STOP_CAP_ADC)
            same_target_deadband = P112R35R2_SETTLED_STOP_CAP_ADC;

        if (P112R5_AbsDiffU16(target,
                autonomous_settled_hold_target_adc) >
            P112R5_TARGET_TOLERANCE_ADC)
        {
            autonomous_settled_hold_valid = 0U;
            autonomous_same_target_excursion_active = 0U;
            autonomous_same_target_excursion_since_ms = 0UL;
        }
        else if (same_target_error <= P112R12R1_SAME_TARGET_HOLD_ADC)
        {
            autonomous_same_target_excursion_active = 0U;
            autonomous_same_target_excursion_since_ms = 0UL;
            autonomous_status.state = NEEDLE_AUTONOMOUS_HOLD;
            return;
        }
        else if (same_target_error <= same_target_deadband)
        {
            autonomous_same_target_excursion_active = 0U;
            autonomous_same_target_excursion_since_ms = 0UL;
            autonomous_status.state = NEEDLE_AUTONOMOUS_HOLD;
            return;
        }
        else
        {
            if (autonomous_same_target_excursion_active == 0U)
            {
                autonomous_same_target_excursion_active = 1U;
                autonomous_same_target_excursion_since_ms = now;
                autonomous_status.state = NEEDLE_AUTONOMOUS_HOLD;
                return;
            }
            if ((uint32_t)(now - autonomous_same_target_excursion_since_ms) <
                P112R12R1_SAME_TARGET_CONFIRM_MS)
            {
                autonomous_status.state = NEEDLE_AUTONOMOUS_HOLD;
                return;
            }

            /* Confirmed mechanical escape: restore normal closed-loop
             * correction for this unchanged target. */
            autonomous_settled_hold_valid = 0U;
            autonomous_same_target_excursion_active = 0U;
            autonomous_same_target_excursion_since_ms = 0UL;
        }
    }

    if (P112R5_AbsDiffU16(current, target) <=
        P112R5_TARGET_TOLERANCE_ADC)
    {
        autonomous_status.state = NEEDLE_AUTONOMOUS_HOLD;
        return;
    }

    autonomous_move_direction = (target < current) ?
        NEEDLE_ADAPTIVE_DIR_OPEN : NEEDLE_ADAPTIVE_DIR_CLOSE;
    if (NeedleValveIntegrationTest_RequestAbsoluteAdaptive(target) == 0U)
    {
        P112R5_LatchFault(NEEDLE_AUTONOMOUS_FAULT_FEEDBACK);
        return;
    }

    autonomous_settled_hold_valid = 0U;
    autonomous_same_target_excursion_active = 0U;
    autonomous_same_target_excursion_since_ms = 0UL;
    autonomous_status.moves_started++;
    autonomous_status.move_in_progress = 1U;
    autonomous_status.state = NEEDLE_AUTONOMOUS_TRACKING;
    P112R5_PublishCompatibilityTelemetry();
}

void NeedleValveAutonomousControl_BackgroundUpdate(void)
{
#if (APP_P112R12R8R18_FLIGHT_FEEDBACK_RECOVERY_REV != 0U)
    /* A P110 feedback-invalid abort stops the motor immediately and latches
     * P111 FEEDBACK + ADC_INVALID. If the disturbance was transient, P83 can
     * recover to NORMAL while that historical latch remains. During active
     * flight only, with no E-STOP sequence ever started, allow a bounded
     * clear after robust feedback is continuously stable and no motor phase is
     * active. The next 200 Hz GNC command must re-authorize motion normally. */
    if ((PreflightTrigger_IsFlightActive() != 0U) &&
        (estop_safe_close_start_count == 0UL) &&
        (autonomous_status.reference_valid != 0U) &&
        (autonomous_status.fault == NEEDLE_AUTONOMOUS_FAULT_FEEDBACK) &&
        (needle_valve_fault == (uint8_t)NEEDLE_VALVE_FAULT_ADC_INVALID) &&
        (p110_state != NEEDLE_ADAPTIVE_STATE_SEARCH) &&
        (p110_state != NEEDLE_ADAPTIVE_STATE_DRIVE) &&
        (p110_state != NEEDLE_ADAPTIVE_STATE_BRAKE) &&
        (p83_feedback_valid != 0U) && (p83_mode == 1U) &&
        (flight_feedback_recovery_count < APP_FLIGHT_NEEDLE_FB_RECOVERY_MAX))
    {
        const uint32_t now = HAL_GetTick();
        if (flight_feedback_recovery_valid_since_ms == 0UL)
        {
            flight_feedback_recovery_valid_since_ms = now;
        }
        else if ((uint32_t)(now - flight_feedback_recovery_valid_since_ms) >=
                 APP_FLIGHT_NEEDLE_FB_RECOVERY_STABLE_MS)
        {
            uint32_t primask;
            NeedleValveIntegrationTest_ForceSafe(0U);
            primask = __get_PRIMASK();
            __disable_irq();
            autonomous_status.fault = NEEDLE_AUTONOMOUS_FAULT_NONE;
            autonomous_status.state = NEEDLE_AUTONOMOUS_REVOKED;
            autonomous_status.command_authorized = 0U;
            autonomous_status.command_fresh = 0U;
            autonomous_status.move_in_progress = 0U;
            autonomous_command_valid = 0U;
            autonomous_settled_hold_valid = 0U;
            autonomous_same_target_excursion_active = 0U;
            autonomous_same_target_excursion_since_ms = 0UL;
            needle_valve_fault = (uint8_t)NEEDLE_VALVE_FAULT_NONE;
            if (primask == 0U) __enable_irq();
            flight_feedback_recovery_count++;
            flight_feedback_recovery_valid_since_ms = 0UL;
        }
    }
    else
    {
        flight_feedback_recovery_valid_since_ms = 0UL;
    }
#endif
    P112R5_PublishCompatibilityTelemetry();
}

void NeedleValveAutonomousControl_CommissioningUpdate(void)
{
#if (APP_NEEDLE_P112R11_MECH_COMMISSION_MODE != 0U)
    uint32_t now = HAL_GetTick();
    uint8_t raw = P112R11_ButtonRead();
    uint8_t press_edge = P112R11_ButtonUpdate(now, raw);
    NeedleValveAutonomousStatus_t a = NeedleValveAutonomousControl_GetStatus();

    p112r11_button_raw = raw;

    if ((p112r11_commission_state == P112R11_OPENING) ||
        (p112r11_commission_state == P112R11_HOLD_OPEN) ||
        (p112r11_commission_state == P112R11_CLOSING))
    {
        uint8_t motion_abort = P112R11_MotionAbortReason();
        if (motion_abort != P112R11_ABORT_NONE)
        {
            P112R11_Abort(motion_abort);
            return;
        }
        if ((uint32_t)(now - p112r11_sequence_start_ms) >
            APP_P112R11_SEQUENCE_TIMEOUT_MS)
        {
            P112R11_Abort(P112R11_ABORT_TIMEOUT);
            return;
        }
        /* After the initiating edge, a later PA0 press is an operator abort. */
        if ((press_edge != 0U) &&
            ((uint32_t)(now - p112r11_sequence_start_ms) > 200UL))
        {
            P112R11_Abort(P112R11_ABORT_OPERATOR_PA0);
            return;
        }
    }

    switch (p112r11_commission_state)
    {
        case P112R11_WAIT_SAFE:
            NeedleValveAutonomousControl_RevokeAuthorization();
            if ((p112r11_single_shot_latched == 0U) &&
                (p112r11_button_debounced == 0U) &&
                (a.moves_completed == 0UL) &&
                (P112R11_SafeToStart() != 0U) &&
                ((uint32_t)(now - p112r11_button_release_since_ms) >=
                 APP_P112R11_RELEASE_ARM_MS))
            {
                p112r11_button_armed = 1U;
                p112r11_commission_state = P112R11_ARMED;
            }
            break;

        case P112R11_ARMED:
            if ((p112r11_single_shot_latched != 0U) ||
                (P112R11_SafeToStart() == 0U))
            {
                p112r11_button_armed = 0U;
                p112r11_commission_state = P112R11_WAIT_SAFE;
                break;
            }
            if ((press_edge != 0U) && (p112r11_button_armed != 0U))
            {
                uint16_t zero = a.closed_reference_adc;
                p112r11_open_target_adc =
                    (zero > APP_P112R11_OPEN_TRAVEL_ADC) ?
                    (uint16_t)(zero - APP_P112R11_OPEN_TRAVEL_ADC) :
                    P112R5_MIN_TARGET_ADC;
                p112r11_move_base = a.moves_completed;
                p112r11_sequence_start_ms = now;
                p112r11_phase_start_ms = now;
                p112r11_button_armed = 0U;
                p112r11_single_shot_latched = 1U;
                p112r11_commission_state = P112R11_OPENING;
                if (NeedleValveAutonomousControl_SubmitCommand(
                        P112R11_COMMISSION_OPEN_COMMAND) == 0U)
                    P112R11_Abort(P112R11_ABORT_OPEN_SUBMIT);
            }
            break;

        case P112R11_OPENING:
            if (NeedleValveAutonomousControl_SubmitCommand(
                    P112R11_COMMISSION_OPEN_COMMAND) == 0U)
            {
                P112R11_Abort(P112R11_ABORT_OPEN_SUBMIT);
                break;
            }
            a = NeedleValveAutonomousControl_GetStatus();
            if (a.moves_completed >= (p112r11_move_base + 1UL))
            {
                p112r11_phase_start_ms = now;
                p112r11_commission_state = P112R11_HOLD_OPEN;
            }
            break;

        case P112R11_HOLD_OPEN:
            if (NeedleValveAutonomousControl_SubmitCommand(
                    P112R11_COMMISSION_OPEN_COMMAND) == 0U)
            {
                P112R11_Abort(P112R11_ABORT_HOLD_SUBMIT);
                break;
            }
            if ((uint32_t)(now - p112r11_phase_start_ms) >=
                APP_P112R11_OPEN_DWELL_MS)
            {
                p112r11_commission_state = P112R11_CLOSING;
                if (NeedleValveAutonomousControl_SubmitCommand(0.0f) == 0U)
                    P112R11_Abort(P112R11_ABORT_CLOSE_SUBMIT);
            }
            break;

        case P112R11_CLOSING:
            if (NeedleValveAutonomousControl_SubmitCommand(0.0f) == 0U)
            {
                P112R11_Abort(P112R11_ABORT_CLOSE_SUBMIT);
                break;
            }
            a = NeedleValveAutonomousControl_GetStatus();
            if (a.moves_completed >= (p112r11_move_base + 2UL))
            {
                NeedleValveAutonomousControl_RevokeAuthorization();
                p112r11_done_latched = 1U;
                p112r11_commission_state = P112R11_DONE;
            }
            break;

        case P112R11_DONE:
            NeedleValveAutonomousControl_RevokeAuthorization();
            p112r11_button_armed = 0U;
            p112r11_single_shot_latched = 1U;
            p112r11_done_latched = 1U;
            break;

        case P112R11_ABORT:
        default:
            NeedleValveAutonomousControl_RevokeAuthorization();
            p112r11_button_armed = 0U;
            p112r11_single_shot_latched = 1U;
            break;
    }
#else
    /* Production/R10 profiles: intentionally inert. */
#endif
}

uint8_t NeedleValveAutonomousControl_GetCommissioningState(void)
{
#if (APP_NEEDLE_P112R11_MECH_COMMISSION_MODE != 0U)
    return p112r11_commission_state;
#else
    return 0U;
#endif
}

uint8_t NeedleValveAutonomousControl_GetCommissioningAbortReason(void)
{
#if (APP_NEEDLE_P112R11_MECH_COMMISSION_MODE != 0U)
    return p112r11_abort_reason;
#else
    return 0U;
#endif
}

uint32_t NeedleValveAutonomousControl_GetCommissioningAbortCount(void)
{
#if (APP_NEEDLE_P112R11_MECH_COMMISSION_MODE != 0U)
    return p112r11_abort_count;
#else
    return 0UL;
#endif
}

uint8_t NeedleValveAutonomousControl_GetCommissioningButtonDebounced(void)
{
#if (APP_NEEDLE_P112R11_MECH_COMMISSION_MODE != 0U)
    return p112r11_button_debounced;
#else
    return 0U;
#endif
}

uint16_t NeedleValveAutonomousControl_GetCommissioningOpenTargetAdc(void)
{
#if (APP_NEEDLE_P112R11_MECH_COMMISSION_MODE != 0U)
    return p112r11_open_target_adc;
#else
    return 0U;
#endif
}

uint8_t NeedleValveAutonomousControl_IsReady(void)
{
    return ((autonomous_status.reference_valid != 0U) &&
            (autonomous_status.fault == NEEDLE_AUTONOMOUS_FAULT_NONE) &&
            (p83_feedback_valid != 0U) && (p83_mode == 1U)) ? 1U : 0U;
}

uint8_t NeedleValveAutonomousControl_HasFault(void)
{
    return (autonomous_status.fault != NEEDLE_AUTONOMOUS_FAULT_NONE) ? 1U : 0U;
}

NeedleValveAutonomousStatus_t NeedleValveAutonomousControl_GetStatus(void)
{
    NeedleValveAutonomousStatus_t copy;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    copy = autonomous_status;
    if (primask == 0U) __enable_irq();
    return copy;
}

NeedleValveStatus_t NeedleValveAutonomousControl_GetTelemetryStatus(void)
{
    NeedleValveStatus_t s = {0};
    NeedleValveAutonomousStatus_t a = NeedleValveAutonomousControl_GetStatus();
    int32_t travel;
    uint16_t abs_error;

    s.requested_cmd = a.requested_command;
    travel = (int32_t)a.closed_reference_adc - (int32_t)p83_filtered_adc;
    if (travel < 0) travel = 0;
    if (travel > (int32_t)P112R5_MAX_TRAVEL_ADC)
        travel = (int32_t)P112R5_MAX_TRAVEL_ADC;
    s.limited_cmd = (float)travel / (float)P112R5_MAX_TRAVEL_ADC;
    s.raw_adc = p83_filtered_adc;
    s.zero_adc = a.closed_reference_adc;
    s.target_adc = a.requested_target_adc;
    s.error_adc = (int16_t)((int32_t)p83_filtered_adc -
                            (int32_t)a.requested_target_adc);
    abs_error = P112R5_AbsDiffU16(p83_filtered_adc,
                                  a.requested_target_adc);
    s.enabled = (uint8_t)((p110_state == NEEDLE_ADAPTIVE_STATE_SEARCH) ||
                          (p110_state == NEEDLE_ADAPTIVE_STATE_DRIVE) ||
                          (p110_state == NEEDLE_ADAPTIVE_STATE_BRAKE));
    s.zero_valid = a.reference_valid;
    s.position_locked = (abs_error <= P112R5_TARGET_TOLERANCE_ADC) ? 1U : 0U;
    s.homing_active = 0U;
    s.homing_complete = a.reference_valid;
    if (a.fault == NEEDLE_AUTONOMOUS_FAULT_NONE)
        s.fault = NEEDLE_VALVE_FAULT_NONE;
    else if ((a.fault == NEEDLE_AUTONOMOUS_FAULT_REFERENCE) ||
             (a.fault == NEEDLE_AUTONOMOUS_FAULT_TARGET_RANGE))
        s.fault = NEEDLE_VALVE_FAULT_ZERO_UNSAFE;
    else if (a.fault == NEEDLE_AUTONOMOUS_FAULT_FEEDBACK)
        s.fault = NEEDLE_VALVE_FAULT_ADC_INVALID;
    else
        s.fault = NEEDLE_VALVE_FAULT_STALL;
    if (s.enabled != 0U)
    {
        if (p110_direction == NEEDLE_ADAPTIVE_DIR_OPEN) s.lpwm = p110_active_pwm;
        else s.rpwm = p110_active_pwm;
    }
    s.stall_ms = p110_total_powered_ms;
    s.control_tick_count = p112_isr_control_ticks;
    s.command_update_count = a.command_sequence;
    s.target_reached_count = a.moves_completed;
    return s;
}
