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

int main(void)
{
    NeedleValveAutonomousStatus_t status;
    uint32_t i;

    NeedleValveAutonomousControl_Init();
    p83_feedback_valid = 1U;
    p83_mode = 1U;
    p83_filtered_adc = 1023U;
    assert(NeedleValveAutonomousControl_CaptureClosedReference() == 1U);

    /* R10R2 observed target: 1023 - round(0.30*780) = 789 ADC. */
    submit_and_tick(10U, 0.30f);
    assert(request_count == 1U);
    assert(requested_target == 789U);

    /* Simulate a P110 adaptive-settled PASS at 799 ADC (+10 residual). */
    p83_filtered_adc = 799U;
    p110_target_adc = 789U;
    p110_final_error_adc = 10;
    p110_coast_max_adc = 32U;
    p110_abort_reason = 0U;
    p110_state = NEEDLE_ADAPTIVE_STATE_DONE;
    p110_result = NEEDLE_ADAPTIVE_RESULT_PASS;
    NeedleValveAutonomousControl_TimerTickISR();

    status = NeedleValveAutonomousControl_GetStatus();
    assert(status.moves_completed == 1U);
    assert(status.state == NEEDLE_AUTONOMOUS_HOLD);
    assert(request_count == 1U);

    /* Repeated 200 Hz copies of the SAME GNC target must not retrigger P110. */
    for (i = 0U; i < 20U; i++)
    {
        submit_and_tick(15U + i * 5U, 0.30f);
        assert(request_count == 1U);
        assert(p110_state == NEEDLE_ADAPTIVE_STATE_DONE);
        status = NeedleValveAutonomousControl_GetStatus();
        assert(status.moves_completed == 1U);
        assert(status.state == NEEDLE_AUTONOMOUS_HOLD);
    }

    /* Small feedback jitter around the accepted settled position also holds. */
    p83_filtered_adc = 805U; /* +6 ADC from accepted 799. */
    submit_and_tick(120U, 0.30f);
    assert(request_count == 1U);

    /* A genuinely new GNC target releases the hold and starts CLOSE. */
    submit_and_tick(125U, 0.0f);
    assert(request_count == 2U);
    assert(requested_target == 1023U);
    assert(p110_direction == NEEDLE_ADAPTIVE_DIR_CLOSE);

    /* Complete CLOSE and verify repeated CLOSED commands do not retrigger. */
    p83_filtered_adc = 1020U;
    p110_target_adc = 1023U;
    p110_final_error_adc = -3;
    p110_coast_max_adc = 20U;
    p110_abort_reason = 0U;
    p110_state = NEEDLE_ADAPTIVE_STATE_DONE;
    p110_result = NEEDLE_ADAPTIVE_RESULT_PASS;
    NeedleValveAutonomousControl_TimerTickISR();

    status = NeedleValveAutonomousControl_GetStatus();
    assert(status.moves_completed == 2U);
    assert(status.state == NEEDLE_AUTONOMOUS_HOLD);

    submit_and_tick(130U, 0.0f);
    assert(request_count == 2U);
    status = NeedleValveAutonomousControl_GetStatus();
    assert(status.moves_completed == 2U);
    assert(status.state == NEEDLE_AUTONOMOUS_HOLD);

    /* Drift beyond the accepted hold point must release the latch. */
    p83_filtered_adc = 1009U; /* 11 ADC from accepted 1020. */
    submit_and_tick(135U, 0.0f);
    assert(request_count == 3U);
    assert(requested_target == 1023U);

    printf("P112R10R3 same-target settled hold host test: PASS\n");
    printf("requests=%lu retargets=%lu force_safe=%lu moves=%lu\n",
           (unsigned long)request_count,
           (unsigned long)retarget_count,
           (unsigned long)force_safe_count,
           (unsigned long)NeedleValveAutonomousControl_GetStatus().moves_completed);
    return 0;
}
