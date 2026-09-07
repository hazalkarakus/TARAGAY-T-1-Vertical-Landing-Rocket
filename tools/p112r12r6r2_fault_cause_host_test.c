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
    *open_breakaway_pwm = 120U;
    *close_breakaway_pwm = 128U;
    *open_coast_adc = 18U;
    *close_coast_adc = 20U;
}

int main(void)
{
    NeedleValveAutonomousStatus_t status;

    /* Establish a valid CLOSED reference and start one normal move. */
    NeedleValveAutonomousControl_Init();
    p83_feedback_valid = 1U;
    p83_mode = 1U;
    p83_filtered_adc = 1020U;
    assert(NeedleValveAutonomousControl_CaptureClosedReference() == 1U);

    p112r5_host_tick_ms = 10U;
    assert(NeedleValveAutonomousControl_SubmitCommand(0.08f) == 1U);
    NeedleValveAutonomousControl_TimerTickISR();
    assert(request_count == 1U);
    assert(p110_state == NEEDLE_ADAPTIVE_STATE_SEARCH);

    /* Reproduce the physical R12R6R1 ordering: P110 sees invalid feedback
     * first and finishes ABORT/FEEDBACK_INVALID before P111's next tick. */
    p110_state = NEEDLE_ADAPTIVE_STATE_DONE;
    p110_result = NEEDLE_ADAPTIVE_RESULT_ABORT;
    p110_abort_reason = NEEDLE_ADAPTIVE_ABORT_FEEDBACK_INVALID;
    p110_active_pwm = 0U;

    NeedleValveAutonomousControl_TimerTickISR();
    status = NeedleValveAutonomousControl_GetStatus();
    assert(status.state == NEEDLE_AUTONOMOUS_FAULT);
    assert(status.fault == NEEDLE_AUTONOMOUS_FAULT_FEEDBACK);
    assert(needle_valve_fault == NEEDLE_VALVE_FAULT_ADC_INVALID);
    assert(p111_abort_reason == NEEDLE_AUTONOMOUS_FAULT_FEEDBACK);

    puts("P112R12R6R2 feedback root-cause propagation host test: PASS");
    return 0;
}
