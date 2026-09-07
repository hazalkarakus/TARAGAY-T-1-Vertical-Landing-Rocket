#ifndef NEEDLE_VALVE_INTEGRATION_TEST_H
#define NEEDLE_VALVE_INTEGRATION_TEST_H

#include <stdint.h>
#include "Modules/Control/NeedleValve/needle_valve_controller.h"

void NeedleValveIntegrationTest_Init(void);
void NeedleValveIntegrationTest_Update(void);
void NeedleValveIntegrationTest_FastButtonService(void);
void NeedleValveIntegrationTest_TimerTickISR(void);
uint8_t NeedleValveIntegrationTest_IsTimingCritical(void);
NeedleValveStatus_t NeedleValveIntegrationTest_GetTelemetryStatus(void);
uint8_t NeedleValveIntegrationTest_RequestAbsoluteAdaptive(uint16_t target_adc);
uint8_t NeedleValveIntegrationTest_RequestRelativeAdaptive(int16_t delta_adc);
uint8_t NeedleValveIntegrationTest_UpdateActiveTarget(uint16_t target_adc);
void NeedleValveIntegrationTest_ForceSafe(uint8_t abort_reason);
void NeedleValveIntegrationTest_GetAdaptiveLearning(
    uint8_t *open_breakaway_pwm,
    uint8_t *close_breakaway_pwm,
    uint16_t *open_coast_adc,
    uint16_t *close_coast_adc);

/* Public P112 adaptive-core state/result values used by the P112R5 autonomous
 * supervisor.  The low-level controller itself remains deterministic in TIM7. */
#define NEEDLE_ADAPTIVE_STATE_WAIT              0U
#define NEEDLE_ADAPTIVE_STATE_SEARCH            1U
#define NEEDLE_ADAPTIVE_STATE_DRIVE             2U
#define NEEDLE_ADAPTIVE_STATE_BRAKE             3U
#define NEEDLE_ADAPTIVE_STATE_CORRECTION_DWELL  4U
#define NEEDLE_ADAPTIVE_STATE_DONE              5U

#define NEEDLE_ADAPTIVE_RESULT_RUNNING          0U
#define NEEDLE_ADAPTIVE_RESULT_PASS             1U
#define NEEDLE_ADAPTIVE_RESULT_ABORT            2U
#define NEEDLE_ADAPTIVE_RESULT_OVERSHOOT        3U
#define NEEDLE_ADAPTIVE_RESULT_NO_BREAKAWAY     4U
#define NEEDLE_ADAPTIVE_RESULT_POWER_TIMEOUT    5U

#define NEEDLE_ADAPTIVE_DIR_OPEN                0U
#define NEEDLE_ADAPTIVE_DIR_CLOSE               1U
#define NEEDLE_ADAPTIVE_ABORT_FEEDBACK_INVALID  1U
#define NEEDLE_ADAPTIVE_ABORT_RETARGET          10U

/* P83 boot-acquisition + safe re-acquisition robust-feedback telemetry.
 * P112R5 retains the P112R2 median-of-3 dual-ADC PC1 sampling. Legacy p111_*
 * slots now expose the autonomous supervisor without changing UART width. */
extern volatile uint32_t p83_bench_magic;
extern volatile uint32_t p83_diag_flags;
extern volatile uint32_t p83_samples_total;
extern volatile uint16_t p83_adc1_raw;
extern volatile uint16_t p83_adc2_raw;
extern volatile uint16_t p83_pair_diff;
extern volatile uint16_t p83_pair_candidate;
extern volatile uint16_t p83_median7;
extern volatile uint16_t p83_filtered_adc;
extern volatile uint8_t p83_feedback_valid;
extern volatile uint8_t p83_confidence_pct;
extern volatile uint32_t p83_pair_reject_count;
extern volatile uint32_t p83_rate_reject_count;
extern volatile uint32_t p83_quarantine_count;
extern volatile uint32_t p83_reacquire_count;
extern volatile uint8_t p83_mode;
extern volatile uint8_t p83_acq_progress_pct;
extern volatile uint32_t p83_adc_timeout_count;
extern volatile uint16_t p83_win_raw_pp;
extern volatile uint16_t p83_win_filtered_pp;
extern volatile uint16_t p83_vref_win_pp_raw12;

/* P87 cumulative ADC sweep diagnostics, sampled internally at ~500 Hz. */
extern volatile uint32_t p87_sweep_samples;
extern volatile uint16_t p87_adc1_min;
extern volatile uint16_t p87_adc1_max;
extern volatile uint16_t p87_adc2_min;
extern volatile uint16_t p87_adc2_max;
extern volatile uint16_t p87_candidate_min;
extern volatile uint16_t p87_candidate_max;
extern volatile uint16_t p87_adc1_max_step;
extern volatile uint16_t p87_adc1_step_from;
extern volatile uint16_t p87_adc1_step_to;
extern volatile uint16_t p87_adc2_max_step;
extern volatile uint16_t p87_adc2_step_from;
extern volatile uint16_t p87_adc2_step_to;
extern volatile uint16_t p87_candidate_max_step;
extern volatile uint16_t p87_candidate_step_from;
extern volatile uint16_t p87_candidate_step_to;
extern volatile uint16_t p87_filtered_max_step;
extern volatile uint16_t p87_filtered_step_from;
extern volatile uint16_t p87_filtered_step_to;
extern volatile uint32_t p87_raw_gap_event_count;
extern volatile uint32_t p87_filtered_gap_event_count;
extern volatile uint16_t p87_pair_diff_max;

/* P99/P100/P105 exact abort/sequence telemetry. Read-only diagnostic exposure only;
 * these variables do not alter actuator timing or safety logic.
 * abort reason: 0 NONE, 1 FEEDBACK_INVALID, 2 START_RANGE, 3 ALREADY_USED,
 * 4 WRONG_DIRECTION, 5 DRIVE_TIMEOUT, 6 SEQUENCE_TIMEOUT,
 * 7 OPEN_OVERSHOOT, 8 CHUNK_LIMIT, 9 HOME_RANGE.
 * phase: 0 PREHOME, 1 PREHOME_DWELL, 2 OPEN, 3 TURNAROUND,
 * 4 RETURN, 5 DONE, 6 INTERCYCLE. */
extern volatile uint8_t p88_abort_reason;
extern volatile uint8_t p97_phase;
extern volatile uint8_t p97_open_chunk_count;
extern volatile uint16_t p97_open_drive_ms;
extern volatile uint8_t p97_return_chunk_count;
extern volatile uint16_t p97_return_drive_ms;

/* P105 three-cycle repeatability summary. */
extern volatile uint8_t p105_cycle_index;
extern volatile uint8_t p105_completed_cycles;
extern volatile uint8_t p105_pass_mask;
extern volatile uint8_t p105_last_cycle_abort;
extern volatile uint16_t p105_last_start_adc;
extern volatile uint16_t p105_last_target_adc;
extern volatile uint16_t p105_last_open_end_adc;
extern volatile int16_t p105_last_open_error_adc;
extern volatile uint16_t p105_last_home_adc;
extern volatile uint8_t p105_last_open_chunks;
extern volatile uint16_t p105_last_open_drive_ms;
extern volatile uint8_t p105_last_return_chunks;
extern volatile uint16_t p105_last_return_drive_ms;


/* P106 loaded-breakaway characterization telemetry.
 * state: 0 WAIT, 1 PULSE, 2 BRAKE, 3 OFF_DWELL, 4 DONE.
 * result: 0 RUNNING, 1 BREAKAWAY, 2 NO_BREAKAWAY, 3 ABORT. */
extern volatile uint8_t p106_state;
extern volatile uint8_t p106_stage;
extern volatile uint8_t p106_test_pwm;
extern volatile uint16_t p106_start_adc;
extern volatile uint16_t p106_stage_start_adc;
extern volatile uint16_t p106_end_adc;
extern volatile int16_t p106_delta_adc;
extern volatile uint8_t p106_breakaway_found;
extern volatile uint8_t p106_breakaway_pwm;
extern volatile uint8_t p106_result;


/* P107 loaded boost+sustain 50-ADC telemetry.
 * state: 0 WAIT, 1 BOOST, 2 SUSTAIN, 3 BRAKE, 4 DONE.
 * result: 0 RUNNING, 1 PASS, 2 TARGET_NOT_REACHED, 3 ABORT, 4 OVERSHOOT.
 * brake_cause: 0 NONE, 1 TARGET, 2 SUSTAIN_TIMEOUT, 3 OVERSHOOT. */
extern volatile uint8_t p107_state;
extern volatile uint8_t p107_result;
extern volatile uint16_t p107_start_adc;
extern volatile uint16_t p107_target_adc;
extern volatile uint16_t p107_end_adc;
extern volatile int16_t p107_error_adc;
extern volatile uint8_t p107_boost_pwm;
extern volatile uint8_t p107_sustain_pwm;
extern volatile uint16_t p107_boost_ms;
extern volatile uint16_t p107_sustain_ms;
extern volatile uint16_t p107_max_open_drop_adc;
extern volatile uint8_t p107_brake_cause;


/* P109 loaded pre-brake/coast characterization telemetry.
 * state: 0 WAIT, 1 BOOST, 2 SUSTAIN, 3 BRAKE, 4 DONE.
 * result: 0 RUNNING, 1 CHARACTERIZED, 2 PREBRAKE_NOT_REACHED, 3 ABORT, 4 OVERSHOOT.
 * coast values are signed OPEN movement after brake entry: entry_adc - sample_adc. */
extern volatile uint8_t p109_state;
extern volatile uint8_t p109_result;
extern volatile uint16_t p109_start_adc;
extern volatile uint16_t p109_target_adc;
extern volatile uint16_t p109_prebrake_adc;
extern volatile uint16_t p109_brake_entry_adc;
extern volatile uint16_t p109_adc_100ms;
extern volatile uint16_t p109_adc_250ms;
extern volatile uint16_t p109_adc_500ms;
extern volatile int16_t p109_coast_100_adc;
extern volatile int16_t p109_coast_250_adc;
extern volatile int16_t p109_coast_500_adc;
extern volatile int16_t p109_final_error_adc;
extern volatile uint16_t p109_boost_ms;
extern volatile uint16_t p109_sustain_ms;
extern volatile uint8_t p109_brake_cause;

/* P110 adaptive position controller telemetry.
 * state: 0 WAIT, 1 SEARCH, 2 DRIVE, 3 BRAKE, 4 CORRECTION_DWELL, 5 DONE.
 * result: 0 RUNNING, 1 PASS, 2 ABORT, 3 OVERSHOOT, 4 NO_BREAKAWAY, 5 POWER_TIMEOUT. */
extern volatile uint8_t p110_state;
extern volatile uint8_t p110_result;
extern volatile uint8_t p110_direction;
extern volatile uint16_t p110_start_adc;
extern volatile uint16_t p110_target_adc;
extern volatile uint16_t p110_current_adc;
extern volatile int16_t p110_error_adc;
extern volatile uint8_t p110_active_pwm;
extern volatile uint8_t p110_learned_breakaway_pwm;
extern volatile uint8_t p110_sustain_pwm;
extern volatile uint16_t p110_speed_adc_s;
extern volatile uint16_t p110_stop_distance_adc;
extern volatile uint16_t p110_brake_entry_adc;
extern volatile uint16_t p110_coast_max_adc;
extern volatile uint16_t p110_final_adc;
extern volatile int16_t p110_final_error_adc;
extern volatile uint16_t p110_search_ms;
extern volatile uint16_t p110_drive_ms;
extern volatile uint16_t p110_brake_ms;
extern volatile uint16_t p110_total_powered_ms;
extern volatile uint8_t p110_correction_count;
extern volatile uint8_t p110_breakaway_found;
extern volatile uint8_t p110_abort_reason;

/* P112R5 reuses the existing p111_* UART slots for its autonomous supervisor,
 * preserving the $TGY68 field count. See the P112R5 README for state mapping. */
extern volatile uint8_t p111_state;
extern volatile uint8_t p111_result;
extern volatile uint8_t p111_cycle;
extern volatile uint8_t p111_completed_cycles;
extern volatile uint8_t p111_pass_mask;
extern volatile uint8_t p111_phase;
extern volatile uint8_t p111_moves_completed;
extern volatile uint16_t p111_baseline_adc;
extern volatile uint16_t p111_open_target_adc;
extern volatile uint8_t p111_learned_open_pwm;
extern volatile uint8_t p111_learned_close_pwm;
extern volatile uint16_t p111_learned_open_coast;
extern volatile uint16_t p111_learned_close_coast;
extern volatile int16_t p111_last_open_error_adc;
extern volatile int16_t p111_last_close_error_adc;
extern volatile uint16_t p111_last_open_powered_ms;
extern volatile uint16_t p111_last_close_powered_ms;
extern volatile uint8_t p111_last_open_breakaway_pwm;
extern volatile uint8_t p111_last_close_breakaway_pwm;
extern volatile uint16_t p111_last_open_stop_adc;
extern volatile uint16_t p111_last_close_stop_adc;
extern volatile uint16_t p111_last_open_coast_adc;
extern volatile uint16_t p111_last_close_coast_adc;
extern volatile uint8_t p111_abort_reason;

/* P112 deterministic timing / UART-independence telemetry.
 * Controller and robust PC1 sampling are serviced from TIM7, not main-loop
 * scheduler slack. UART/SD formatting is suppressed only while the motor or
 * active brake is timing-critical; snapshots resume during dwell/DONE. */
extern volatile uint32_t p112_isr_control_ticks;
extern volatile uint32_t p112_isr_adc_samples;
extern volatile uint32_t p112_hard_off_count;
extern volatile uint32_t p112_uart_suppressed_count;
extern volatile uint32_t p112_sd_suppressed_count;
extern volatile uint16_t p112_isr_last_us;
extern volatile uint16_t p112_isr_max_us;
extern volatile uint8_t p112_timing_critical;
extern volatile uint8_t p112_decel_active;
extern volatile uint8_t p112_decel_pwm;
extern volatile uint16_t p112_initial_coast_adc;

#endif
