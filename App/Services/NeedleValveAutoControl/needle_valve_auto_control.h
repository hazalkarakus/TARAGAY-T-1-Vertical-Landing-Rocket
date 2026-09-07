#ifndef NEEDLE_VALVE_AUTO_CONTROL_H
#define NEEDLE_VALVE_AUTO_CONTROL_H

#include <stdint.h>

#define V819M_ENDURANCE_TARGET_CYCLES 10U

typedef enum
{
    V819M_AUTO_BOOT_WAIT = 0,
    V819M_AUTO_HOMING = 1,
    V819M_AUTO_READY_DISABLED = 2,
    V819M_AUTO_ENDURANCE_ACTIVE = 3,
    V819M_AUTO_TEST_PASS = 4,
    V819M_AUTO_ABORTED = 5,
    V819M_AUTO_FAULT = 6
} V819M_AutoState_t;

typedef enum
{
    V819M_PHASE_IDLE = 0,
    V819M_PHASE_OPENING = 1,
    V819M_PHASE_OPEN_DWELL = 2,
    V819M_PHASE_CLOSING = 3,
    V819M_PHASE_CLOSE_DWELL = 4,
    V819M_PHASE_COOLDOWN = 5,
    V819M_PHASE_PASS = 6,
    V819M_PHASE_ABORTED = 7,
    V819M_PHASE_FAULT = 8
} V819M_EndurancePhase_t;

typedef enum
{
    V819M_FAIL_NONE = 0,
    V819M_FAIL_CONTROLLER = 1,
    V819M_FAIL_LEG_TIMEOUT = 2,
    V819M_FAIL_TOTAL_TIMEOUT = 3,
    V819M_FAIL_COMMAND_REJECTED = 4,
    V819M_FAIL_OPERATOR_ABORT = 5,
    V819M_FAIL_ZERO_LOST = 6,
    V819M_FAIL_INTERNAL_STATE = 7
} V819M_FailureReason_t;

void NeedleValveAutoControl_Init(void);
void NeedleValveAutoControl_Update(void);

/* Bench-only command gate used by the endurance supervisor. GNC is not
 * connected to this entry point in V8.19M. */
uint8_t NeedleValveAutoControl_SubmitCommand(float command_0_to_1);

extern volatile uint32_t v819m_auto_magic;
extern volatile uint8_t v819m_auto_state;
extern volatile uint8_t v819m_endurance_phase;
extern volatile uint8_t v819m_auto_last_action_ok;
extern volatile uint8_t v819m_auto_ready_for_test;
extern volatile uint8_t v819m_endurance_target_cycles;
extern volatile uint8_t v819m_endurance_current_cycle;
extern volatile uint8_t v819m_endurance_cycles_completed;
extern volatile uint16_t v819m_endurance_command_x10000;
extern volatile uint32_t v819m_auto_button_press_count;
extern volatile uint32_t v819m_auto_home_start_count;
extern volatile uint32_t v819m_auto_home_pass_count;
extern volatile uint32_t v819m_auto_abort_count;
extern volatile uint32_t v819m_auto_command_accept_count;
extern volatile uint32_t v819m_auto_command_reject_count;
extern volatile uint32_t v819m_auto_boot_wait_remaining_ms;

extern volatile uint32_t v819m_endurance_total_elapsed_ms;
extern volatile uint32_t v819m_endurance_phase_elapsed_ms;
extern volatile uint32_t v819m_endurance_leg_elapsed_ms;
extern volatile uint32_t v819m_endurance_cooldown_remaining_ms;
extern volatile uint16_t v819m_endurance_min_raw_adc;
extern volatile uint16_t v819m_endurance_max_raw_adc;

extern volatile uint8_t v819m_failure_reason;
extern volatile uint8_t v819m_failure_cycle;
extern volatile uint8_t v819m_failure_phase;
extern volatile uint8_t v819m_failure_controller_fault;
extern volatile uint16_t v819m_failure_raw_adc;
extern volatile int16_t v819m_failure_error_adc;

extern volatile uint16_t
    v819m_cycle_open_raw_adc[V819M_ENDURANCE_TARGET_CYCLES];
extern volatile uint16_t
    v819m_cycle_close_raw_adc[V819M_ENDURANCE_TARGET_CYCLES];
extern volatile uint32_t
    v819m_cycle_open_time_ms[V819M_ENDURANCE_TARGET_CYCLES];
extern volatile uint32_t
    v819m_cycle_close_time_ms[V819M_ENDURANCE_TARGET_CYCLES];

/* PA0 safety diagnostics. A start press is accepted only after a complete
 * LOW-release arm and a continuously stable HIGH press. Any new armed rising
 * edge while the endurance test is active causes an immediate safe abort. */
extern volatile uint8_t v819m_button_raw;
extern volatile uint8_t v819m_button_stable;
extern volatile uint8_t v819m_button_armed;
extern volatile uint32_t v819m_button_raw_transition_count;
extern volatile uint32_t v819m_button_glitch_reject_count;
extern volatile uint32_t v819m_button_press_event_count;

#endif
