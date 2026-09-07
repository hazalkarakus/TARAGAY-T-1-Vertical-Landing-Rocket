#include "needle_valve_auto_control.h"

#include "../../Modules/Control/NeedleValve/needle_valve_controller.h"
#include "main.h"

#define V819M_BOOT_SETTLE_MS                    3000UL
#define V819M_BUTTON_PRESS_STABLE_MS             500UL
#define V819M_BUTTON_RELEASE_STABLE_MS             50UL
#define V819M_BUTTON_RELEASE_ARM_MS              1000UL
#define V819M_ENDPOINT_DWELL_MS                  2000UL
#define V819M_CYCLE_COOLDOWN_MS                  5000UL
#define V819M_LEG_TIMEOUT_MS                    20000UL
#define V819M_TOTAL_TIMEOUT_MS                 600000UL
#define V819M_ENDPOINT_ERROR_ADC                    3U

volatile uint32_t v819m_auto_magic = 0x4E56414DUL; /* "NVAM" */
volatile uint8_t v819m_auto_state = V819M_AUTO_BOOT_WAIT;
volatile uint8_t v819m_endurance_phase = V819M_PHASE_IDLE;
volatile uint8_t v819m_auto_last_action_ok = 0U;
volatile uint8_t v819m_auto_ready_for_test = 0U;
volatile uint8_t v819m_endurance_target_cycles =
    V819M_ENDURANCE_TARGET_CYCLES;
volatile uint8_t v819m_endurance_current_cycle = 0U;
volatile uint8_t v819m_endurance_cycles_completed = 0U;
volatile uint16_t v819m_endurance_command_x10000 = 0U;
volatile uint32_t v819m_auto_button_press_count = 0UL;
volatile uint32_t v819m_auto_home_start_count = 0UL;
volatile uint32_t v819m_auto_home_pass_count = 0UL;
volatile uint32_t v819m_auto_abort_count = 0UL;
volatile uint32_t v819m_auto_command_accept_count = 0UL;
volatile uint32_t v819m_auto_command_reject_count = 0UL;
volatile uint32_t v819m_auto_boot_wait_remaining_ms = V819M_BOOT_SETTLE_MS;

volatile uint32_t v819m_endurance_total_elapsed_ms = 0UL;
volatile uint32_t v819m_endurance_phase_elapsed_ms = 0UL;
volatile uint32_t v819m_endurance_leg_elapsed_ms = 0UL;
volatile uint32_t v819m_endurance_cooldown_remaining_ms = 0UL;
volatile uint16_t v819m_endurance_min_raw_adc = 1023U;
volatile uint16_t v819m_endurance_max_raw_adc = 0U;

volatile uint8_t v819m_failure_reason = V819M_FAIL_NONE;
volatile uint8_t v819m_failure_cycle = 0U;
volatile uint8_t v819m_failure_phase = V819M_PHASE_IDLE;
volatile uint8_t v819m_failure_controller_fault = 0U;
volatile uint16_t v819m_failure_raw_adc = 0U;
volatile int16_t v819m_failure_error_adc = 0;

volatile uint16_t
    v819m_cycle_open_raw_adc[V819M_ENDURANCE_TARGET_CYCLES] = {0U};
volatile uint16_t
    v819m_cycle_close_raw_adc[V819M_ENDURANCE_TARGET_CYCLES] = {0U};
volatile uint32_t
    v819m_cycle_open_time_ms[V819M_ENDURANCE_TARGET_CYCLES] = {0UL};
volatile uint32_t
    v819m_cycle_close_time_ms[V819M_ENDURANCE_TARGET_CYCLES] = {0UL};

volatile uint8_t v819m_button_raw = 0U;
volatile uint8_t v819m_button_stable = 0U;
volatile uint8_t v819m_button_armed = 0U;
volatile uint32_t v819m_button_raw_transition_count = 0UL;
volatile uint32_t v819m_button_glitch_reject_count = 0UL;
volatile uint32_t v819m_button_press_event_count = 0UL;

static uint32_t init_ms = 0UL;
static uint32_t button_candidate_since_ms = 0UL;
static uint32_t button_release_since_ms = 0UL;
static uint8_t button_candidate = 0U;
static uint8_t button_press_candidate_eligible = 0U;
static uint8_t button_emergency_rise = 0U;

static uint32_t test_start_ms = 0UL;
static uint32_t phase_start_ms = 0UL;
static uint32_t leg_start_ms = 0UL;

static uint16_t V819M_AbsI16(int16_t value)
{
    return (value < 0) ? (uint16_t)(-value) : (uint16_t)value;
}

static void V819M_ButtonInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static uint8_t V819M_ButtonRead(void)
{
    return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) ? 1U : 0U;
}

static void V819M_ButtonRequireRelease(uint32_t now_ms)
{
    v819m_button_armed = 0U;
    button_press_candidate_eligible = 0U;
    button_emergency_rise = 0U;
    button_release_since_ms = now_ms;
}

static uint8_t V819M_ButtonUpdate(uint32_t now_ms)
{
    uint8_t raw = V819M_ButtonRead();
    uint8_t pressed = 0U;

    button_emergency_rise = 0U;
    v819m_button_raw = raw;

    if (raw != button_candidate)
    {
        if (button_candidate != v819m_button_stable)
        {
            v819m_button_glitch_reject_count++;
        }

        button_candidate = raw;
        button_candidate_since_ms = now_ms;
        v819m_button_raw_transition_count++;

        if (raw != 0U)
        {
            button_press_candidate_eligible = v819m_button_armed;
            button_emergency_rise = v819m_button_armed;
            v819m_button_armed = 0U;
        }
        else
        {
            button_press_candidate_eligible = 0U;
            button_release_since_ms = now_ms;
        }
    }

    if (button_candidate != v819m_button_stable)
    {
        uint32_t required_ms = (button_candidate != 0U)
            ? V819M_BUTTON_PRESS_STABLE_MS
            : V819M_BUTTON_RELEASE_STABLE_MS;

        if ((uint32_t)(now_ms - button_candidate_since_ms) >= required_ms)
        {
            v819m_button_stable = button_candidate;

            if ((v819m_button_stable != 0U) &&
                (button_press_candidate_eligible != 0U))
            {
                button_press_candidate_eligible = 0U;
                v819m_button_press_event_count++;
                pressed = 1U;
            }
        }
    }

    if ((raw == 0U) &&
        (v819m_button_stable == 0U) &&
        (v819m_button_armed == 0U) &&
        ((uint32_t)(now_ms - button_release_since_ms) >=
         V819M_BUTTON_RELEASE_ARM_MS))
    {
        v819m_button_armed = 1U;
    }

    return pressed;
}

static void V819M_ClearCycleResults(void)
{
    uint8_t i;

    for (i = 0U; i < V819M_ENDURANCE_TARGET_CYCLES; i++)
    {
        v819m_cycle_open_raw_adc[i] = 0U;
        v819m_cycle_close_raw_adc[i] = 0U;
        v819m_cycle_open_time_ms[i] = 0UL;
        v819m_cycle_close_time_ms[i] = 0UL;
    }

    v819m_endurance_current_cycle = 0U;
    v819m_endurance_cycles_completed = 0U;
    v819m_endurance_command_x10000 = 0U;
    v819m_endurance_total_elapsed_ms = 0UL;
    v819m_endurance_phase_elapsed_ms = 0UL;
    v819m_endurance_leg_elapsed_ms = 0UL;
    v819m_endurance_cooldown_remaining_ms = 0UL;
    v819m_endurance_min_raw_adc = 1023U;
    v819m_endurance_max_raw_adc = 0U;

    v819m_failure_reason = V819M_FAIL_NONE;
    v819m_failure_cycle = 0U;
    v819m_failure_phase = V819M_PHASE_IDLE;
    v819m_failure_controller_fault = 0U;
    v819m_failure_raw_adc = 0U;
    v819m_failure_error_adc = 0;
}

static void V819M_RecordFailure(
    V819M_FailureReason_t reason,
    NeedleValveStatus_t status)
{
    v819m_failure_reason = (uint8_t)reason;
    v819m_failure_cycle = v819m_endurance_current_cycle;
    v819m_failure_phase = v819m_endurance_phase;
    v819m_failure_controller_fault = (uint8_t)status.fault;
    v819m_failure_raw_adc = status.raw_adc;
    v819m_failure_error_adc = status.error_adc;
}

static void V819M_FinishFault(
    V819M_FailureReason_t reason,
    NeedleValveStatus_t status)
{
    V819M_RecordFailure(reason, status);
    NeedleValveController_Stop();
    v819m_auto_state = V819M_AUTO_FAULT;
    v819m_endurance_phase = V819M_PHASE_FAULT;
    v819m_auto_ready_for_test = 0U;
    v819m_auto_last_action_ok = 0U;
    v819m_endurance_command_x10000 = 0U;
    v819m_endurance_cooldown_remaining_ms = 0UL;
}

static void V819M_Abort(NeedleValveStatus_t status)
{
    V819M_RecordFailure(V819M_FAIL_OPERATOR_ABORT, status);
    NeedleValveController_Stop();
    v819m_auto_state = V819M_AUTO_ABORTED;
    v819m_endurance_phase = V819M_PHASE_ABORTED;
    v819m_auto_ready_for_test = 0U;
    v819m_auto_last_action_ok = 1U;
    v819m_endurance_command_x10000 = 0U;
    v819m_endurance_cooldown_remaining_ms = 0UL;
    v819m_auto_abort_count++;
}

static void V819M_FinishPass(void)
{
    NeedleValveController_Stop();
    v819m_auto_state = V819M_AUTO_TEST_PASS;
    v819m_endurance_phase = V819M_PHASE_PASS;
    v819m_auto_ready_for_test = 0U;
    v819m_auto_last_action_ok = 1U;
    v819m_endurance_command_x10000 = 0U;
    v819m_endurance_cooldown_remaining_ms = 0UL;
}

uint8_t NeedleValveAutoControl_SubmitCommand(float command_0_to_1)
{
    uint8_t accepted;

    if (v819m_auto_state != V819M_AUTO_ENDURANCE_ACTIVE)
    {
        v819m_auto_command_reject_count++;
        v819m_auto_last_action_ok = 0U;
        return 0U;
    }

    accepted = NeedleValveController_SetCommand(command_0_to_1);

    if (accepted != 0U)
    {
        v819m_auto_command_accept_count++;
        v819m_auto_last_action_ok = 1U;
    }
    else
    {
        v819m_auto_command_reject_count++;
        v819m_auto_last_action_ok = 0U;
    }

    return accepted;
}

static uint8_t V819M_StartLeg(
    float command,
    uint16_t command_x10000,
    V819M_EndurancePhase_t phase,
    uint32_t now_ms)
{
    v819m_endurance_command_x10000 = command_x10000;

    if (NeedleValveAutoControl_SubmitCommand(command) == 0U)
    {
        return 0U;
    }

    v819m_endurance_phase = (uint8_t)phase;
    phase_start_ms = now_ms;
    leg_start_ms = now_ms;
    v819m_endurance_phase_elapsed_ms = 0UL;
    v819m_endurance_leg_elapsed_ms = 0UL;
    return 1U;
}

static uint8_t V819M_StartCycle(uint32_t now_ms)
{
    NeedleValveStatus_t status = NeedleValveController_GetStatus();

    if ((status.zero_valid == 0U) ||
        (status.fault != NEEDLE_VALVE_FAULT_NONE))
    {
        return 0U;
    }

    if (NeedleValveController_Enable() == 0U)
    {
        return 0U;
    }

    v819m_endurance_current_cycle =
        (uint8_t)(v819m_endurance_cycles_completed + 1U);

    if (V819M_StartLeg(1.0f, 10000U, V819M_PHASE_OPENING, now_ms) == 0U)
    {
        NeedleValveController_Stop();
        return 0U;
    }

    return 1U;
}

static void V819M_StartTest(uint32_t now_ms)
{
    NeedleValveStatus_t status;

    V819M_ClearCycleResults();
    v819m_auto_state = V819M_AUTO_ENDURANCE_ACTIVE;
    v819m_endurance_phase = V819M_PHASE_IDLE;
    v819m_auto_ready_for_test = 0U;
    test_start_ms = now_ms;
    phase_start_ms = now_ms;
    leg_start_ms = now_ms;

    if (V819M_StartCycle(now_ms) == 0U)
    {
        status = NeedleValveController_GetStatus();
        V819M_FinishFault(V819M_FAIL_COMMAND_REJECTED, status);
    }
}

static void V819M_UpdateLiveTimes(uint32_t now_ms)
{
    v819m_endurance_total_elapsed_ms =
        (uint32_t)(now_ms - test_start_ms);
    v819m_endurance_phase_elapsed_ms =
        (uint32_t)(now_ms - phase_start_ms);

    if ((v819m_endurance_phase == V819M_PHASE_OPENING) ||
        (v819m_endurance_phase == V819M_PHASE_OPEN_DWELL) ||
        (v819m_endurance_phase == V819M_PHASE_CLOSING) ||
        (v819m_endurance_phase == V819M_PHASE_CLOSE_DWELL))
    {
        v819m_endurance_leg_elapsed_ms =
            (uint32_t)(now_ms - leg_start_ms);
    }
    else
    {
        v819m_endurance_leg_elapsed_ms = 0UL;
    }

    if (v819m_endurance_phase == V819M_PHASE_COOLDOWN)
    {
        uint32_t elapsed = (uint32_t)(now_ms - phase_start_ms);
        v819m_endurance_cooldown_remaining_ms =
            (elapsed < V819M_CYCLE_COOLDOWN_MS)
            ? (V819M_CYCLE_COOLDOWN_MS - elapsed)
            : 0UL;
    }
    else
    {
        v819m_endurance_cooldown_remaining_ms = 0UL;
    }
}

static uint8_t V819M_EndpointValid(NeedleValveStatus_t status)
{
    return ((status.position_locked != 0U) &&
            (V819M_AbsI16(status.error_adc) <=
             V819M_ENDPOINT_ERROR_ADC)) ? 1U : 0U;
}

static void V819M_RunEndurance(
    uint32_t now_ms,
    NeedleValveStatus_t status)
{
    uint8_t index;

    if (status.raw_adc < v819m_endurance_min_raw_adc)
    {
        v819m_endurance_min_raw_adc = status.raw_adc;
    }
    if (status.raw_adc > v819m_endurance_max_raw_adc)
    {
        v819m_endurance_max_raw_adc = status.raw_adc;
    }

    if (status.fault != NEEDLE_VALVE_FAULT_NONE)
    {
        V819M_FinishFault(V819M_FAIL_CONTROLLER, status);
        return;
    }

    if (status.zero_valid == 0U)
    {
        V819M_FinishFault(V819M_FAIL_ZERO_LOST, status);
        return;
    }

    if ((uint32_t)(now_ms - test_start_ms) >= V819M_TOTAL_TIMEOUT_MS)
    {
        V819M_FinishFault(V819M_FAIL_TOTAL_TIMEOUT, status);
        return;
    }

    if (button_emergency_rise != 0U)
    {
        v819m_auto_button_press_count++;
        V819M_Abort(status);
        return;
    }

    index = (v819m_endurance_current_cycle > 0U)
        ? (uint8_t)(v819m_endurance_current_cycle - 1U)
        : 0U;

    switch ((V819M_EndurancePhase_t)v819m_endurance_phase)
    {
        case V819M_PHASE_OPENING:
            if ((uint32_t)(now_ms - leg_start_ms) >= V819M_LEG_TIMEOUT_MS)
            {
                V819M_FinishFault(V819M_FAIL_LEG_TIMEOUT, status);
            }
            else if (V819M_EndpointValid(status) != 0U)
            {
                v819m_endurance_phase = V819M_PHASE_OPEN_DWELL;
                phase_start_ms = now_ms;
            }
            break;

        case V819M_PHASE_OPEN_DWELL:
            if ((uint32_t)(now_ms - leg_start_ms) >= V819M_LEG_TIMEOUT_MS)
            {
                V819M_FinishFault(V819M_FAIL_LEG_TIMEOUT, status);
            }
            else if (V819M_EndpointValid(status) == 0U)
            {
                v819m_endurance_phase = V819M_PHASE_OPENING;
                phase_start_ms = now_ms;
            }
            else if ((uint32_t)(now_ms - phase_start_ms) >=
                     V819M_ENDPOINT_DWELL_MS)
            {
                if (index < V819M_ENDURANCE_TARGET_CYCLES)
                {
                    v819m_cycle_open_raw_adc[index] = status.raw_adc;
                    v819m_cycle_open_time_ms[index] =
                        (uint32_t)(now_ms - leg_start_ms);
                }

                if (V819M_StartLeg(
                        0.0f,
                        0U,
                        V819M_PHASE_CLOSING,
                        now_ms) == 0U)
                {
                    status = NeedleValveController_GetStatus();
                    V819M_FinishFault(
                        V819M_FAIL_COMMAND_REJECTED,
                        status);
                }
            }
            break;

        case V819M_PHASE_CLOSING:
            if ((uint32_t)(now_ms - leg_start_ms) >= V819M_LEG_TIMEOUT_MS)
            {
                V819M_FinishFault(V819M_FAIL_LEG_TIMEOUT, status);
            }
            else if (V819M_EndpointValid(status) != 0U)
            {
                v819m_endurance_phase = V819M_PHASE_CLOSE_DWELL;
                phase_start_ms = now_ms;
            }
            break;

        case V819M_PHASE_CLOSE_DWELL:
            if ((uint32_t)(now_ms - leg_start_ms) >= V819M_LEG_TIMEOUT_MS)
            {
                V819M_FinishFault(V819M_FAIL_LEG_TIMEOUT, status);
            }
            else if (V819M_EndpointValid(status) == 0U)
            {
                v819m_endurance_phase = V819M_PHASE_CLOSING;
                phase_start_ms = now_ms;
            }
            else if ((uint32_t)(now_ms - phase_start_ms) >=
                     V819M_ENDPOINT_DWELL_MS)
            {
                if (index < V819M_ENDURANCE_TARGET_CYCLES)
                {
                    v819m_cycle_close_raw_adc[index] = status.raw_adc;
                    v819m_cycle_close_time_ms[index] =
                        (uint32_t)(now_ms - leg_start_ms);
                }

                v819m_endurance_cycles_completed++;

                if (v819m_endurance_cycles_completed >=
                    V819M_ENDURANCE_TARGET_CYCLES)
                {
                    V819M_FinishPass();
                }
                else
                {
                    NeedleValveController_Stop();
                    v819m_endurance_phase = V819M_PHASE_COOLDOWN;
                    v819m_endurance_command_x10000 = 0U;
                    phase_start_ms = now_ms;
                    v819m_endurance_cooldown_remaining_ms =
                        V819M_CYCLE_COOLDOWN_MS;
                }
            }
            break;

        case V819M_PHASE_COOLDOWN:
            if ((uint32_t)(now_ms - phase_start_ms) >=
                V819M_CYCLE_COOLDOWN_MS)
            {
                if (V819M_StartCycle(now_ms) == 0U)
                {
                    status = NeedleValveController_GetStatus();
                    V819M_FinishFault(
                        V819M_FAIL_COMMAND_REJECTED,
                        status);
                }
            }
            break;

        default:
            V819M_FinishFault(V819M_FAIL_INTERNAL_STATE, status);
            break;
    }
}

void NeedleValveAutoControl_Init(void)
{
    V819M_ButtonInit();

    v819m_auto_state = V819M_AUTO_BOOT_WAIT;
    v819m_endurance_phase = V819M_PHASE_IDLE;
    v819m_auto_last_action_ok = 0U;
    v819m_auto_ready_for_test = 0U;
    v819m_endurance_target_cycles = V819M_ENDURANCE_TARGET_CYCLES;
    v819m_auto_button_press_count = 0UL;
    v819m_auto_home_start_count = 0UL;
    v819m_auto_home_pass_count = 0UL;
    v819m_auto_abort_count = 0UL;
    v819m_auto_command_accept_count = 0UL;
    v819m_auto_command_reject_count = 0UL;
    v819m_auto_boot_wait_remaining_ms = V819M_BOOT_SETTLE_MS;
    V819M_ClearCycleResults();

    v819m_button_raw = V819M_ButtonRead();
    v819m_button_stable = v819m_button_raw;
    v819m_button_armed = 0U;
    v819m_button_raw_transition_count = 0UL;
    v819m_button_glitch_reject_count = 0UL;
    v819m_button_press_event_count = 0UL;

    NeedleValveController_Stop();

    init_ms = HAL_GetTick();
    button_candidate = v819m_button_raw;
    button_candidate_since_ms = init_ms;
    button_release_since_ms = init_ms;
    button_press_candidate_eligible = 0U;
    button_emergency_rise = 0U;
    test_start_ms = 0UL;
    phase_start_ms = 0UL;
    leg_start_ms = 0UL;
}

void NeedleValveAutoControl_Update(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = (uint32_t)(now_ms - init_ms);
    uint8_t button_pressed = V819M_ButtonUpdate(now_ms);
    NeedleValveStatus_t status = NeedleValveController_GetStatus();

    if ((button_pressed != 0U) &&
        ((v819m_auto_state == V819M_AUTO_BOOT_WAIT) ||
         (v819m_auto_state == V819M_AUTO_HOMING)))
    {
        v819m_auto_button_press_count++;
        V819M_Abort(status);
        return;
    }

    switch ((V819M_AutoState_t)v819m_auto_state)
    {
        case V819M_AUTO_BOOT_WAIT:
            v819m_auto_boot_wait_remaining_ms =
                (elapsed_ms < V819M_BOOT_SETTLE_MS)
                ? (V819M_BOOT_SETTLE_MS - elapsed_ms)
                : 0UL;

            if (elapsed_ms >= V819M_BOOT_SETTLE_MS)
            {
                v819m_auto_last_action_ok =
                    NeedleValveController_StartAutoHome();

                if (v819m_auto_last_action_ok != 0U)
                {
                    v819m_auto_home_start_count++;
                    v819m_auto_state = V819M_AUTO_HOMING;
                }
                else
                {
                    status = NeedleValveController_GetStatus();
                    V819M_FinishFault(
                        V819M_FAIL_COMMAND_REJECTED,
                        status);
                }
            }
            break;

        case V819M_AUTO_HOMING:
            if (status.fault != NEEDLE_VALVE_FAULT_NONE)
            {
                V819M_FinishFault(V819M_FAIL_CONTROLLER, status);
            }
            else if ((status.homing_complete != 0U) &&
                     (status.zero_valid != 0U) &&
                     (status.enabled == 0U))
            {
                v819m_auto_home_pass_count++;
                v819m_auto_state = V819M_AUTO_READY_DISABLED;
                v819m_endurance_phase = V819M_PHASE_IDLE;
                v819m_auto_ready_for_test = 0U;
                v819m_auto_last_action_ok = 1U;
                V819M_ButtonRequireRelease(now_ms);
            }
            break;

        case V819M_AUTO_READY_DISABLED:
            v819m_auto_ready_for_test = v819m_button_armed;

            if (button_pressed != 0U)
            {
                v819m_auto_button_press_count++;
                V819M_StartTest(now_ms);
            }
            break;

        case V819M_AUTO_ENDURANCE_ACTIVE:
            V819M_UpdateLiveTimes(now_ms);
            V819M_RunEndurance(now_ms, status);
            break;

        case V819M_AUTO_TEST_PASS:
        case V819M_AUTO_ABORTED:
        case V819M_AUTO_FAULT:
        default:
            NeedleValveController_Stop();
            v819m_auto_ready_for_test = 0U;
            break;
    }
}
