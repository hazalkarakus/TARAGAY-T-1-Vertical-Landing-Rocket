#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.h"
#include "Services/NeedleValveIntegrationTest/needle_valve_integration_test.h"

uint32_t p112r5_host_tick_ms;
uint8_t p112r11_host_button;
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
static uint32_t force_safe_count;

uint8_t NeedleValveIntegrationTest_RequestAbsoluteAdaptive(uint16_t target_adc)
{
    requested_target = target_adc;
    p110_target_adc = target_adc;
    request_count++;
    p110_state = NEEDLE_ADAPTIVE_STATE_SEARCH;
    p110_result = NEEDLE_ADAPTIVE_RESULT_RUNNING;
    return 1U;
}
uint8_t NeedleValveIntegrationTest_UpdateActiveTarget(uint16_t target_adc)
{
    requested_target = target_adc; p110_target_adc = target_adc; return 1U;
}
void NeedleValveIntegrationTest_ForceSafe(uint8_t reason)
{
    (void)reason; force_safe_count++;
    if ((p110_state != NEEDLE_ADAPTIVE_STATE_WAIT) &&
        (p110_state != NEEDLE_ADAPTIVE_STATE_DONE)) {
        p110_state = NEEDLE_ADAPTIVE_STATE_DONE;
        p110_result = NEEDLE_ADAPTIVE_RESULT_ABORT;
    }
}
void NeedleValveIntegrationTest_GetAdaptiveLearning(uint8_t *a,uint8_t *b,uint16_t *c,uint16_t *d)
{ *a=120U; *b=128U; *c=18U; *d=20U; }


static void induce_transient_low_level_fault(void)
{
    uint32_t before = request_count;
    p83_filtered_adc = 1020U;
    assert(NeedleValveAutonomousControl_SubmitCommand(0.50f) == 1U);
    NeedleValveAutonomousControl_TimerTickISR();
    assert(request_count == before + 1U);

    /* Simulate a non-feedback low-level actuator abort/stall. */
    p110_state = NEEDLE_ADAPTIVE_STATE_DONE;
    p110_result = NEEDLE_ADAPTIVE_RESULT_ABORT;
    p110_abort_reason = 2U;
    NeedleValveAutonomousControl_TimerTickISR();
    assert(NeedleValveAutonomousControl_HasFault() == 1U);
    assert(needle_valve_fault == NEEDLE_VALVE_FAULT_STALL);
}

static void prepare_reference(uint16_t closed)
{
    p83_feedback_valid=1U; p83_mode=1U; p83_filtered_adc=closed;
    p110_state=NEEDLE_ADAPTIVE_STATE_DONE; p110_result=NEEDLE_ADAPTIVE_RESULT_PASS;
    assert(NeedleValveAutonomousControl_IsClosedReferenceCandidate()==1U);
    assert(NeedleValveAutonomousControl_CaptureClosedReference()==1U);
}

int main(void)
{
    NeedleValveAutonomousControl_Init();
    prepare_reference(1020U);

    /* Operator manually moves the valve open while the software target is
       still CLOSED. E-STOP safe-close must bypass the normal same-target
       200 ms jitter dwell and request CLOSED immediately. */
    p83_filtered_adc=700U;
    p112r5_host_tick_ms=100U;
    NeedleValveAutonomousControl_EStopSafeCloseUpdate();
    assert(NeedleValveAutonomousControl_IsEStopSafeCloseActive()==1U);
    assert(NeedleValveAutonomousControl_IsEStopSafeCloseComplete()==0U);
    assert(NeedleValveAutonomousControl_HasEStopSafeCloseFailed()==0U);
    assert(NeedleValveAutonomousControl_GetEStopSafeCloseStartCount()==1U);
    NeedleValveAutonomousControl_TimerTickISR();
    assert(request_count==1U);
    assert(requested_target==1020U);

    /* Reaching CLOSED de-energizes and seals the one-shot result. */
    p83_filtered_adc=1012U; /* within 12 ADC */
    p112r5_host_tick_ms=180U;
    NeedleValveAutonomousControl_EStopSafeCloseUpdate();
    assert(NeedleValveAutonomousControl_IsEStopSafeCloseActive()==0U);
    assert(NeedleValveAutonomousControl_IsEStopSafeCloseComplete()==1U);
    assert(NeedleValveAutonomousControl_HasEStopSafeCloseFailed()==0U);
    assert(NeedleValveAutonomousControl_GetEStopSafeCloseStartCount()==1U);
    assert(force_safe_count>0U);

    /* STOP stays one-shot: subsequent servicing cannot restart the motor. */
    p83_filtered_adc=700U;
    p112r5_host_tick_ms=250U;
    NeedleValveAutonomousControl_EStopSafeCloseUpdate();
    assert(request_count==1U);
    assert(NeedleValveAutonomousControl_GetEStopSafeCloseStartCount()==1U);

    /* R8R16 regression from the physical R8R15 log: if a transient
       LOW_LEVEL/STALL fault was already latched but feedback proves the valve
       is physically CLOSED, E-STOP must report complete instead of failing. */
    NeedleValveAutonomousControl_Init();
    prepare_reference(1020U);
    induce_transient_low_level_fault();
    p83_filtered_adc = 1016U;
    p112r5_host_tick_ms = 350U;
    NeedleValveAutonomousControl_EStopSafeCloseUpdate();
    assert(NeedleValveAutonomousControl_IsEStopSafeCloseComplete() == 1U);
    assert(NeedleValveAutonomousControl_HasEStopSafeCloseFailed() == 0U);

    /* If the valve is OPEN with the same prior transient fault, R8R16 must
       clear it exactly once and request CLOSED immediately. */
    NeedleValveAutonomousControl_Init();
    prepare_reference(1020U);
    induce_transient_low_level_fault();
    p83_filtered_adc = 700U;
    p112r5_host_tick_ms = 400U;
    {
        uint32_t before = request_count;
        NeedleValveAutonomousControl_EStopSafeCloseUpdate();
        assert(NeedleValveAutonomousControl_IsEStopSafeCloseActive() == 1U);
        assert(NeedleValveAutonomousControl_HasEStopSafeCloseFailed() == 0U);
        NeedleValveAutonomousControl_TimerTickISR();
        assert(request_count == before + 1U);
        assert(requested_target == 1020U);
    }
    p83_filtered_adc = 1014U;
    p112r5_host_tick_ms = 500U;
    NeedleValveAutonomousControl_EStopSafeCloseUpdate();
    assert(NeedleValveAutonomousControl_IsEStopSafeCloseComplete() == 1U);

    /* Missing feedback must fail closed-as-motion, i.e. motor remains safe. */
    NeedleValveAutonomousControl_Init();
    prepare_reference(1018U);
    p83_filtered_adc=700U;
    p83_feedback_valid=0U;
    p112r5_host_tick_ms=500U;
    NeedleValveAutonomousControl_EStopSafeCloseUpdate();
    assert(NeedleValveAutonomousControl_HasEStopSafeCloseFailed()==1U);
    assert(NeedleValveAutonomousControl_GetEStopSafeCloseFailReason()==2U);

    puts("P112R16 E-STOP emergency-close host test: PASS");
    return 0;
}
