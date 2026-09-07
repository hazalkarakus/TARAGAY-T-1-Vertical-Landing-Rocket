#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.h"
#include "Services/NeedleValveIntegrationTest/needle_valve_integration_test.h"

uint32_t p112r5_host_tick_ms;
volatile uint8_t needle_valve_fault;
volatile uint8_t p83_feedback_valid;
volatile uint8_t p83_mode;
volatile uint16_t p83_filtered_adc;
volatile uint8_t p110_state;
volatile uint8_t p110_result;
volatile uint8_t p110_direction;
volatile uint16_t p110_target_adc;
volatile uint8_t p110_active_pwm;
volatile uint8_t p110_learned_breakaway_pwm;
volatile uint16_t p110_stop_distance_adc;
volatile uint16_t p110_coast_max_adc;
volatile int16_t p110_final_error_adc;
volatile uint16_t p110_total_powered_ms;
volatile uint8_t p110_abort_reason;
volatile uint32_t p112_isr_control_ticks;
volatile uint16_t p112_initial_coast_adc = 20U;
volatile uint8_t p111_state;
volatile uint8_t p111_result;
volatile uint8_t p111_cycle;
volatile uint8_t p111_completed_cycles;
volatile uint8_t p111_pass_mask;
volatile uint8_t p111_phase;
volatile uint8_t p111_moves_completed;
volatile uint16_t p111_baseline_adc;
volatile uint16_t p111_open_target_adc;
volatile uint8_t p111_learned_open_pwm;
volatile uint8_t p111_learned_close_pwm;
volatile uint16_t p111_learned_open_coast;
volatile uint16_t p111_learned_close_coast;
volatile int16_t p111_last_open_error_adc;
volatile int16_t p111_last_close_error_adc;
volatile uint16_t p111_last_open_powered_ms;
volatile uint16_t p111_last_close_powered_ms;
volatile uint8_t p111_last_open_breakaway_pwm;
volatile uint8_t p111_last_close_breakaway_pwm;
volatile uint16_t p111_last_open_stop_adc;
volatile uint16_t p111_last_close_stop_adc;
volatile uint16_t p111_last_open_coast_adc;
volatile uint16_t p111_last_close_coast_adc;
volatile uint8_t p111_abort_reason;

static uint16_t requested_target;
static uint32_t request_count;
static uint32_t retarget_count;
static uint32_t force_safe_count;

uint8_t NeedleValveIntegrationTest_RequestAbsoluteAdaptive(uint16_t target_adc)
{
    requested_target = target_adc;
    p110_target_adc = target_adc;
    request_count++;
    p110_direction = (target_adc < p83_filtered_adc) ?
        NEEDLE_ADAPTIVE_DIR_OPEN : NEEDLE_ADAPTIVE_DIR_CLOSE;
    p110_state = NEEDLE_ADAPTIVE_STATE_SEARCH;
    p110_result = NEEDLE_ADAPTIVE_RESULT_RUNNING;
    return 1U;
}

uint8_t NeedleValveIntegrationTest_UpdateActiveTarget(uint16_t target_adc)
{
    requested_target = target_adc;
    p110_target_adc = target_adc;
    retarget_count++;
    return 1U;
}

void NeedleValveIntegrationTest_ForceSafe(uint8_t abort_reason)
{
    (void)abort_reason;
    force_safe_count++;
    if ((p110_state != NEEDLE_ADAPTIVE_STATE_WAIT) &&
        (p110_state != NEEDLE_ADAPTIVE_STATE_DONE))
    {
        p110_state = NEEDLE_ADAPTIVE_STATE_DONE;
        p110_result = NEEDLE_ADAPTIVE_RESULT_ABORT;
    }
}

void NeedleValveIntegrationTest_GetAdaptiveLearning(
    uint8_t *open_breakaway_pwm, uint8_t *close_breakaway_pwm,
    uint16_t *open_coast_adc, uint16_t *close_coast_adc)
{
    *open_breakaway_pwm = 136U;
    *close_breakaway_pwm = 136U;
    *open_coast_adc = 32U;
    *close_coast_adc = 20U;
}

static void submit_and_tick(uint32_t ms, float command)
{
    p112r5_host_tick_ms = ms;
    assert(NeedleValveAutonomousControl_SubmitCommand(command) == 1U);
    NeedleValveAutonomousControl_TimerTickISR();
}

static void reset_closed(uint16_t adc)
{
    NeedleValveAutonomousControl_Init();
    p83_feedback_valid = 1U;
    p83_mode = 1U;
    p83_filtered_adc = adc;
    p110_state = NEEDLE_ADAPTIVE_STATE_WAIT;
    p110_result = NEEDLE_ADAPTIVE_RESULT_RUNNING;
    request_count = 0U;
    retarget_count = 0U;
    assert(NeedleValveAutonomousControl_CaptureClosedReference() == 1U);
}

int main(void)
{
    NeedleValveAutonomousStatus_t status;
    uint32_t t;

    /* 950+ CLOSED reference policy remains intact. */
    NeedleValveAutonomousControl_Init();
    p83_feedback_valid = 1U;
    p83_mode = 1U;
    p83_filtered_adc = 949U;
    assert(NeedleValveAutonomousControl_CaptureClosedReference() == 0U);
    reset_closed(950U);

    /* 1) CLOSED target repeated at 200 Hz: +/-20 ADC mechanical motion is HOLD. */
    reset_closed(995U);
    p83_filtered_adc = 989U; /* -6 */
    submit_and_tick(10U, 0.0f);
    assert(request_count == 0U);

    p83_filtered_adc = 983U; /* -12: firm hold edge */
    submit_and_tick(15U, 0.0f);
    assert(request_count == 0U);

    p83_filtered_adc = 976U; /* -19: deadband */
    submit_and_tick(20U, 0.0f);
    assert(request_count == 0U);

    /* A >20 ADC spike shorter than 200 ms must NOT launch P110. */
    p83_filtered_adc = 973U; /* -22 */
    for (t = 25U; t < 220U; t += 5U)
    {
        submit_and_tick(t, 0.0f);
        assert(request_count == 0U);
    }
    /* Recovery inside deadband resets persistence timer. */
    p83_filtered_adc = 978U; /* -17 */
    submit_and_tick(220U, 0.0f);
    assert(request_count == 0U);

    /* 2) A genuine GNC retarget bypasses the 200 ms guard immediately. */
    p83_filtered_adc = 978U;
    submit_and_tick(225U, 0.10f); /* 995 - 78 = 917 */
    assert(request_count == 1U);
    assert(requested_target == 917U);
    assert(p110_direction == NEEDLE_ADAPTIVE_DIR_OPEN);

    /* Complete the real move with PASS. */
    p83_filtered_adc = 920U;
    p110_target_adc = 917U;
    p110_final_error_adc = 3;
    p110_abort_reason = 0U;
    p110_state = NEEDLE_ADAPTIVE_STATE_DONE;
    p110_result = NEEDLE_ADAPTIVE_RESULT_PASS;
    p112r5_host_tick_ms = 230U;
    NeedleValveAutonomousControl_TimerTickISR();
    status = NeedleValveAutonomousControl_GetStatus();
    assert(status.moves_completed == 1U);

    /* Same OPEN target with 18 ADC settled/backlash drift remains held. */
    p83_filtered_adc = 935U; /* +18 from target */
    for (t = 235U; t <= 335U; t += 5U)
    {
        submit_and_tick(t, 0.10f);
        assert(request_count == 1U);
    }

    /* 3) Confirmed >20 ADC escape for the SAME target: no correction before
     * 200 ms, then exactly one correction request at/after 200 ms. */
    p83_filtered_adc = 940U; /* +23 */
    for (t = 340U; t < 540U; t += 5U)
    {
        submit_and_tick(t, 0.10f);
        assert(request_count == 1U);
    }
    submit_and_tick(540U, 0.10f);
    assert(request_count == 2U);
    assert(requested_target == 917U);

    printf("P112R12R1 same-target mechanical guard host test: PASS\n");
    printf("hold<=12 deadband<=20 confirm=200ms requests=%lu retargets=%lu force_safe=%lu\n",
           (unsigned long)request_count,
           (unsigned long)retarget_count,
           (unsigned long)force_safe_count);
    return 0;
}
