#include "needle_valve_integration_test.h"
#include "Common/app_config.h"
#include "Services/NeedleValveHardware/needle_valve_hw.h"
#include "Services/SolenoidOutput/solenoid_output.h"
#include "main.h"
#include "Services/Timebase/timebase.h"

/* P112R5: AUTONOMOUS ADAPTIVE ACTUATOR CORE
 * ---------------------------------------------------------------------
 * Reusable bench-only closed-loop position controller layered on the proven
 * P83/P98 robust feedback and BTS7960 hardware path. Unlike P106-P109 it does
 * not depend on a fixed breakaway PWM, fixed sustain duration, or fixed brake
 * point. Each move identifies the required breakaway level, measures filtered
 * motion speed, predicts a bounded stop distance, brakes dynamically, measures
 * coast, and can perform up to two automatic undershoot corrections.
 *
 * P112R5 keeps the serialized TIM7 ADC ownership and median-of-3 burst sampling
 * proven by P112R2. The old reset+5 s+CLOSE 80 bench supervisor is compile-time
 * disconnected. Targets now arrive only from NeedleValveAutonomousControl,
 * whose sole producer is the authorized 200 Hz generated flight-control task.
 *
 * Safety bounds:
 *   - search PWM ramps 112..192/255 in 8-count steps
 *   - breakaway search step interval 12 ms; no unbounded PWM escalation
 *   - sustain PWM is derived from learned breakaway (typically breakaway-16)
 *   - nominal target tolerance +/-8 ADC; settled acceptance guard +/-12 ADC; hard overshoot guard 12 ADC
 *   - dynamic brake prediction clamped 8..48 ADC; conservative coast seed
 *   - distance-scaled powered-time deadline, hard-capped at 2600 ms
 *   - maximum two automatic undershoot corrections
 *   - maximum sequence time 4 s
 *   - no automatic reverse correction after overshoot
 *
 * Hardware qualification is still required before flight. Keep hands/tools
 * clear. If the motor audibly hard-stalls, coupling twists/slips, alignment
 * binds, or a hard stop is hit,
 * cut motor power immediately and do not repeat.
 */
#define P83_SAMPLE_PERIOD_MS                    2UL
#define P83_PROBE_PERIOD_MS                    50UL
#define P83_WINDOW_PERIOD_MS                 1000UL

#define P83_MEDIAN_LEN                           7U
#define P83_MEDIAN_MIN_VALID                     5U
#define P83_PAIR_HARD_DIFF_ADC                  30U
#define P83_PAIR_BAD_STREAK_QUARANTINE           3U

#define P83_RAW_MAX_STEP_ADC                    80U
#define P83_MEDIAN_MAX_STEP_ADC                 20U

/* P98 BENCH-ONLY commanded-motion envelope.
 * The two ADC peripherals sample the same PC1 node sequentially; while the
 * actuator is moving, a real slope can make the samples differ. P98 relaxes
 * only the rate/pair limits when the measured trend agrees with the commanded
 * direction. Hard disagreement and opposite-direction jumps remain faults. */
#define P98_MOTION_PAIR_HARD_DIFF_ADC             60U
#define P98_MOTION_RAW_MAX_STEP_ADC              120U
#define P98_MOTION_MEDIAN_MAX_STEP_ADC            48U
#define P98_MOTION_DIR_TOL_ADC                    12U
#define P98_PROGRESS_REVERSAL_TOL_ADC             40U
#define P83_TRAJECTORY_START_ADC                24U
#define P83_TRAJECTORY_REV_TOL_ADC               4U
#define P83_TRAJECTORY_CONFIRM_SAMPLES           8U

#define P83_OUTPUT_DEADBAND_ADC                  3U
#define P83_OUTPUT_MAX_STEP_ADC                  6U
#define P83_VALID_CONFIDENCE_PCT                60U
#define P83_VREF_PP_WARN_RAW12                  15U

/* Initial anchor: ~200 ms of compact median data. */
#define P83_BOOT_ACQ_STABLE_SAMPLES            100U
#define P83_ACQ_CLUSTER_SPREAD_ADC              24U
#define P83_ACQ_MAX_STEP_ADC                    10U

/* Quarantine: last-good recovery first; far re-anchor only in motor-locked P83. */
#define P83_QUARANTINE_MIN_MS                  100UL
#define P83_NEAR_RECOVERY_BAND_ADC              25U
#define P83_NEAR_RECOVERY_MAX_STEP_ADC           8U
#define P83_NEAR_RECOVERY_STABLE_SAMPLES        25U
#define P83_FAR_REACQUIRE_STABLE_SAMPLES       250U  /* ~500 ms */

#define P83_MODE_ACQUIRE                         0U
#define P83_MODE_NORMAL                          1U
#define P83_MODE_QUARANTINE                      2U
#define P83_MODE_TRAJECTORY                      3U

#define P83_FLAG_PAIR_REJECT                (1UL << 0)
#define P83_FLAG_RATE_REJECT                (1UL << 1)
#define P83_FLAG_QUARANTINE_EVENT           (1UL << 2)
#define P83_FLAG_FEEDBACK_INVALID           (1UL << 3)
#define P83_FLAG_LOW_CONFIDENCE             (1UL << 4)
#define P83_FLAG_VREF_MOVE                  (1UL << 5)
#define P83_FLAG_ADC_TIMEOUT                (1UL << 6)
#define P83_FLAG_ACQUIRING                  (1UL << 7)
#define P83_FLAG_TRAJECTORY_PENDING         (1UL << 8)
#define P83_FLAG_RECOVERY_STABLE            (1UL << 9)
#define P83_FLAG_SAFE_REACQUIRE             (1UL << 10)
#define P83_FLAG_ACQ_RESET                  (1UL << 11)

#define P83_CURRENT_FLAG_MASK (P83_FLAG_FEEDBACK_INVALID | P83_FLAG_LOW_CONFIDENCE | \
                               P83_FLAG_ACQUIRING | P83_FLAG_TRAJECTORY_PENDING | \
                               P83_FLAG_RECOVERY_STABLE)

#define P87_RAW_GAP_EVENT_THRESHOLD_ADC            8U
#define P87_FILTERED_GAP_EVENT_THRESHOLD_ADC       6U

#define P97_TEST_PWM                            128U /* 128/255 = 50.20% */
#define P97_OPEN_DELTA_ADC                      400U
#define P97_OPEN_TOL_ADC                          8U
#define P97_MIN_START_ADC                       120U
#define P97_MAX_START_ADC                      1023U
#define P97_BRAKE_HOLD_MS                       100UL
#define P97_TURNAROUND_HOLD_MS                  300UL
#define P97_STABLE_CONFIRM_SAMPLES                5U
#define P97_AUTO_START_DELAY_MS                5000UL
#define P97_OPEN_MAX_TOTAL_DRIVE_MS             560U
#define P97_RETURN_MAX_TOTAL_DRIVE_MS          1400U
#define P97_MAX_SEQUENCE_MS                   15000UL
#define P97_OPEN_MAX_CHUNKS                      36U
#define P97_RETURN_MAX_CHUNKS                    64U
#define P97_WRONG_DIRECTION_TOL_ADC              20U
#define P97_HOME_LOW_ADC                       1015U
#define P97_HOME_HIGH_ADC                      1023U

#define P105_TOTAL_CYCLES                         3U
#define P105_INTERCYCLE_DWELL_MS               1000UL


/* P106 loaded-breakaway characterization. */
#define P106_START_MIN_ADC                       980U
#define P106_PULSE_MS                             20U
#define P106_BRAKE_HOLD_MS                       250UL
#define P106_INTERSTAGE_OFF_MS                   750UL
#define P106_BREAKAWAY_DELTA_ADC                   8U
#define P106_WRONG_DIR_DELTA_ADC                   8U
#define P106_MAX_SEQUENCE_MS                    8000UL
#define P106_PWM_STAGE_COUNT                       5U

#define P106_STATE_WAIT                            0U
#define P106_STATE_PULSE                           1U
#define P106_STATE_BRAKE                           2U
#define P106_STATE_OFF_DWELL                       3U
#define P106_STATE_DONE                            4U

#define P106_RESULT_RUNNING                        0U
#define P106_RESULT_BREAKAWAY                      1U
#define P106_RESULT_NO_BREAKAWAY                   2U
#define P106_RESULT_ABORT                          3U


/* P107 loaded boost+sustain 50-ADC characterization. */
#define P107_START_MIN_ADC                       980U
#define P107_TARGET_DELTA_ADC                     50U
#define P107_TARGET_TOL_ADC                        8U
#define P107_OVERSHOOT_ABORT_ADC                  12U
#define P107_WRONG_DIR_DELTA_ADC                   8U
#define P107_BOOST_PWM                           144U /* 56.5% */
#define P107_BOOST_MS                             20U
#define P107_SUSTAIN_PWM                         128U /* 50.2% */
#define P107_SUSTAIN_MAX_MS                       80U
#define P107_BRAKE_HOLD_MS                       300UL
#define P107_MAX_SEQUENCE_MS                    3000UL

#define P107_STATE_WAIT                            0U
#define P107_STATE_BOOST                           1U
#define P107_STATE_SUSTAIN                         2U
#define P107_STATE_BRAKE                           3U
#define P107_STATE_DONE                            4U

#define P107_RESULT_RUNNING                        0U
#define P107_RESULT_PASS                           1U
#define P107_RESULT_TARGET_NOT_REACHED             2U
#define P107_RESULT_ABORT                          3U
#define P107_RESULT_OVERSHOOT                      4U

#define P107_BRAKE_CAUSE_NONE                      0U
#define P107_BRAKE_CAUSE_TARGET                    1U
#define P107_BRAKE_CAUSE_SUSTAIN_TIMEOUT           2U
#define P107_BRAKE_CAUSE_OVERSHOOT                 3U


/* P109 pre-brake/coast characterization. */
#define P109_START_MIN_ADC                       850U
#define P109_TARGET_DELTA_ADC                     50U
#define P109_PREBRAKE_MARGIN_ADC                  20U
#define P109_HARD_OVERSHOOT_ADC                   12U
#define P109_WRONG_DIR_DELTA_ADC                   8U
#define P109_BOOST_PWM                           144U /* 56.5% */
#define P109_BOOST_MS                             20U
#define P109_SUSTAIN_PWM                         128U /* 50.2% */
#define P109_SUSTAIN_MAX_MS                      120U
#define P109_BRAKE_HOLD_MS                       500UL
#define P109_MAX_SEQUENCE_MS                    3000UL

#define P109_STATE_WAIT                            0U
#define P109_STATE_BOOST                           1U
#define P109_STATE_SUSTAIN                         2U
#define P109_STATE_BRAKE                           3U
#define P109_STATE_DONE                            4U

#define P109_RESULT_RUNNING                        0U
#define P109_RESULT_CHARACTERIZED                  1U
#define P109_RESULT_PREBRAKE_NOT_REACHED           2U
#define P109_RESULT_ABORT                          3U
#define P109_RESULT_OVERSHOOT                      4U

#define P109_BRAKE_CAUSE_NONE                      0U
#define P109_BRAKE_CAUSE_PREBRAKE                  1U
#define P109_BRAKE_CAUSE_SUSTAIN_TIMEOUT           2U
#define P109_BRAKE_CAUSE_OVERSHOOT                 3U


/* P110 adaptive position controller. */
#define P110_AUTO_START_DELAY_MS                5000UL
#define P110_AUTO_DELTA_ADC                       50U
#define P110_TARGET_TOL_ADC                        8U
/* P112R10R2 adaptive correction-feasibility guard.
 * R10/R10R1 showed that a fresh powered correction can have a minimum
 * mechanical motion quantum comparable to the just-measured brake/coast
 * distance. A fixed 12-ADC acceptance band was therefore brittle: one run
 * settled at +9 ADC, the next at +13 ADC. Keep the nominal +/-8 target and
 * the powered overshoot guard unchanged, but before starting another
 * SEARCH/DRIVE correction compare the residual undershoot with the measured
 * coast quantum from THIS move. If the residual is smaller than that quantum,
 * another breakaway is more likely to cross the target than improve it.
 * The adaptive acceptance is hard-capped at 20 ADC; larger residuals still
 * use the existing adaptive correction path. */
#define P110_SETTLED_ACCEPT_ADC                   12U
#define P110_CORRECTION_QUANTUM_MAX_ADC           20U
#define P110_HARD_OVERSHOOT_ADC                   12U
#define P110_BREAKAWAY_MOVE_ADC                    4U
#define P110_SEARCH_START_PWM                    112U
#define P110_SEARCH_STEP_PWM                       8U
#define P110_SEARCH_MAX_PWM                      192U
#define P110_SEARCH_STEP_MS                       12UL
#define P110_SUSTAIN_DROP_PWM                     16U
#define P110_SUSTAIN_MIN_PWM                     112U
#define P110_EVAL_PERIOD_MS                        5UL
#define P110_SLOW_SPEED_ADC_S                      60U
#define P110_SLOW_RAISE_MS                         20UL
#define P110_SUSTAIN_RAISE_PWM                      8U
#define P110_BRAKE_HORIZON_MS                     10UL
#define P110_STOP_BASE_ADC                         4U
#define P110_STOP_MIN_ADC                          8U
#define P110_STOP_MAX_ADC                         48U
#define P110_BRAKE_HOLD_MS                       300UL
#define P112R5_MIN_POWERED_LIMIT_MS              350U
#define P112R5_MAX_POWERED_LIMIT_MS             2600U
#define P112R5_POWERED_MS_PER_ADC                  3U
#define P110_MAX_SEQUENCE_MS                    4000UL
#define P110_MAX_CORRECTIONS                       2U
#define P110_CORRECTION_DWELL_MS                 120UL
#define P110_MIN_TARGET_ADC                       20U
#define P110_MAX_TARGET_ADC                     1023U
#define P112_INITIAL_COAST_ADC                     20U
#define P112_DECEL_ZONE_EXTRA_ADC                  18U
#define P112_DECEL_DROP_PWM                        24U
#define P112_DECEL_MIN_PWM                         96U
#define P112_ISR_ADC_DIVIDER                        2U
#define P112_VREF_ISR_DIVIDER                      50U  /* 20 Hz, serialized inside TIM7 */
#define P112_COMMISSION_BASELINE_MIN_ADC           200U  /* never auto-OPEN from near-low endpoint */

#define P110_STATE_WAIT                            0U
#define P110_STATE_SEARCH                          1U
#define P110_STATE_DRIVE                           2U
#define P110_STATE_BRAKE                           3U
#define P110_STATE_CORRECTION_DWELL                4U
#define P110_STATE_DONE                            5U

#define P110_RESULT_RUNNING                        0U
#define P110_RESULT_PASS                           1U
#define P110_RESULT_ABORT                          2U
#define P110_RESULT_OVERSHOOT                      3U
#define P110_RESULT_NO_BREAKAWAY                   4U
#define P110_RESULT_POWER_TIMEOUT                  5U
#define P110_RESULT_RETARGET                       6U

#define P110_DIR_OPEN                              0U
#define P110_DIR_CLOSE                             1U

#define P111_STATE_WAIT                            0U
#define P111_RESULT_RUNNING                        0U
#define P111_PHASE_OPEN                            0U
#define P111_PHASE_CLOSE                           1U

#define P97_PHASE_PREHOME                         0U
#define P97_PHASE_PREHOME_DWELL                   1U
#define P97_PHASE_OPEN                            2U
#define P97_PHASE_TURNAROUND                      3U
#define P97_PHASE_RETURN                          4U
#define P97_PHASE_DONE                            5U
#define P105_PHASE_INTERCYCLE                      6U

#define P97_ABORT_NONE                            0U
#define P97_ABORT_FEEDBACK_INVALID                1U
#define P97_ABORT_START_RANGE                     2U
#define P97_ABORT_ALREADY_USED                    3U
#define P97_ABORT_WRONG_DIRECTION                 4U
#define P97_ABORT_DRIVE_TIMEOUT                   5U
#define P97_ABORT_SEQUENCE_TIMEOUT                6U
#define P97_ABORT_OPEN_OVERSHOOT                  7U
#define P97_ABORT_CHUNK_LIMIT                     8U
#define P97_ABORT_HOME_RANGE                      9U

volatile uint32_t p83_bench_magic = 0x50383341UL; /* "P83A" */
volatile uint32_t p83_diag_flags = 0UL;
volatile uint32_t p83_samples_total = 0UL;
volatile uint16_t p83_adc1_raw = 0U;
volatile uint16_t p83_adc2_raw = 0U;
volatile uint16_t p83_pair_diff = 0U;
volatile uint16_t p83_pair_candidate = 0U;
volatile uint16_t p83_median7 = 0U;
volatile uint16_t p83_filtered_adc = 0U;
volatile uint8_t p83_feedback_valid = 0U;
volatile uint8_t p83_confidence_pct = 0U;
volatile uint32_t p83_pair_reject_count = 0UL;
volatile uint32_t p83_rate_reject_count = 0UL;
volatile uint32_t p83_quarantine_count = 0UL;
volatile uint32_t p83_reacquire_count = 0UL;
volatile uint8_t p83_mode = P83_MODE_ACQUIRE;
volatile uint8_t p83_acq_progress_pct = 0U;
volatile uint32_t p83_adc_timeout_count = 0UL;
volatile uint16_t p83_win_raw_pp = 0U;
volatile uint16_t p83_win_filtered_pp = 0U;
volatile uint16_t p83_vref_win_pp_raw12 = 0U;

/* P87 cumulative sweep diagnostics. Updated at the native ~500 Hz ADC sample rate,
 * so UART only needs to report the cumulative result at 10 Hz. */
volatile uint32_t p87_sweep_samples = 0UL;
volatile uint16_t p87_adc1_min = 1023U;
volatile uint16_t p87_adc1_max = 0U;
volatile uint16_t p87_adc2_min = 1023U;
volatile uint16_t p87_adc2_max = 0U;
volatile uint16_t p87_candidate_min = 1023U;
volatile uint16_t p87_candidate_max = 0U;
volatile uint16_t p87_adc1_max_step = 0U;
volatile uint16_t p87_adc1_step_from = 0U;
volatile uint16_t p87_adc1_step_to = 0U;
volatile uint16_t p87_adc2_max_step = 0U;
volatile uint16_t p87_adc2_step_from = 0U;
volatile uint16_t p87_adc2_step_to = 0U;
volatile uint16_t p87_candidate_max_step = 0U;
volatile uint16_t p87_candidate_step_from = 0U;
volatile uint16_t p87_candidate_step_to = 0U;
volatile uint16_t p87_filtered_max_step = 0U;
volatile uint16_t p87_filtered_step_from = 0U;
volatile uint16_t p87_filtered_step_to = 0U;
volatile uint32_t p87_raw_gap_event_count = 0UL;
volatile uint32_t p87_filtered_gap_event_count = 0UL;
volatile uint16_t p87_pair_diff_max = 0U;

volatile uint32_t p88_bench_magic = 0x50393741UL; /* "P97A" */
volatile uint8_t p88_jog_used = 0U;
volatile uint8_t p88_jog_active = 0U;
volatile uint8_t p88_jog_finished = 0U;
volatile uint8_t p88_abort_reason = P97_ABORT_NONE;
volatile uint16_t p88_start_adc = 0U;
volatile uint16_t p88_end_adc = 0U;
volatile int16_t p88_delta_adc = 0;
volatile uint16_t p88_pulse_elapsed_ms = 0U;

static volatile uint16_t p88_ticks_remaining = 0U;
static volatile uint8_t p88_settle_pending = 0U;
static volatile uint32_t p88_pulse_stop_ms = 0UL;
static volatile uint32_t p88_pulse_start_ms = 0UL;
static volatile uint32_t p90_service_init_ms = 0UL;
static volatile uint16_t p97_target_adc = 0U;
static volatile uint32_t p97_sequence_start_ms = 0UL;
static volatile uint32_t p97_turnaround_start_ms = 0UL;
volatile uint16_t p97_open_drive_ms = 0U;
volatile uint16_t p97_return_drive_ms = 0U;
static volatile uint8_t p97_active_pwm = 0U;
static volatile uint16_t p97_active_chunk_ms = 0U;
volatile uint8_t p97_open_chunk_count = 0U;
volatile uint8_t p97_return_chunk_count = 0U;
static volatile uint16_t p97_prev_eval_adc = 0U;
static volatile uint16_t p97_return_start_adc = 0U;
static volatile uint8_t p97_confirm_count = 0U;
static volatile uint32_t p97_last_confirm_sample_total = 0UL;
volatile uint8_t p97_phase = P97_PHASE_PREHOME;
static volatile uint8_t p97_drive_direction = 0U; /* 0=OPEN, 1=CLOSE */
static volatile uint16_t p98_best_open_adc = 1023U;
static volatile uint16_t p98_best_close_adc = 0U;

/* P105 three-cycle repeatability telemetry.  The p105_last_* fields are
 * committed only when a cycle completes (or when the current cycle aborts),
 * so every completed-cycle summary remains visible throughout the 1 s dwell. */
volatile uint8_t p105_cycle_index = 0U;
volatile uint8_t p105_completed_cycles = 0U;
volatile uint8_t p105_pass_mask = 0U;
volatile uint8_t p105_last_cycle_abort = P97_ABORT_NONE;
volatile uint16_t p105_last_start_adc = 0U;
volatile uint16_t p105_last_target_adc = 0U;
volatile uint16_t p105_last_open_end_adc = 0U;
volatile int16_t p105_last_open_error_adc = 0;
volatile uint16_t p105_last_home_adc = 0U;
volatile uint8_t p105_last_open_chunks = 0U;
volatile uint16_t p105_last_open_drive_ms = 0U;
volatile uint8_t p105_last_return_chunks = 0U;
volatile uint16_t p105_last_return_drive_ms = 0U;

static volatile uint32_t p105_intercycle_start_ms = 0UL;
static volatile uint16_t p105_current_open_target_adc = 0U;
static volatile uint16_t p105_current_open_end_adc = 0U;
static volatile int16_t p105_current_open_error_adc = 0;


/* P106 loaded-breakaway telemetry. */
volatile uint8_t p106_state = P106_STATE_WAIT;
volatile uint8_t p106_stage = 0U;                 /* 1..5 while active */
volatile uint8_t p106_test_pwm = 0U;
volatile uint16_t p106_start_adc = 0U;
volatile uint16_t p106_stage_start_adc = 0U;
volatile uint16_t p106_end_adc = 0U;
volatile int16_t p106_delta_adc = 0;
volatile uint8_t p106_breakaway_found = 0U;
volatile uint8_t p106_breakaway_pwm = 0U;
volatile uint8_t p106_result = P106_RESULT_RUNNING;


/* P107 loaded boost+sustain telemetry. */
volatile uint8_t p107_state = P107_STATE_WAIT;
volatile uint8_t p107_result = P107_RESULT_RUNNING;
volatile uint16_t p107_start_adc = 0U;
volatile uint16_t p107_target_adc = 0U;
volatile uint16_t p107_end_adc = 0U;
volatile int16_t p107_error_adc = 0;
volatile uint8_t p107_boost_pwm = P107_BOOST_PWM;
volatile uint8_t p107_sustain_pwm = P107_SUSTAIN_PWM;
volatile uint16_t p107_boost_ms = 0U;
volatile uint16_t p107_sustain_ms = 0U;
volatile uint16_t p107_max_open_drop_adc = 0U;
volatile uint8_t p107_brake_cause = P107_BRAKE_CAUSE_NONE;

static volatile uint8_t p107_pending_result = P107_RESULT_RUNNING;


/* P109 pre-brake/coast telemetry. */
volatile uint8_t p109_state = P109_STATE_WAIT;
volatile uint8_t p109_result = P109_RESULT_RUNNING;
volatile uint16_t p109_start_adc = 0U;
volatile uint16_t p109_target_adc = 0U;
volatile uint16_t p109_prebrake_adc = 0U;
volatile uint16_t p109_brake_entry_adc = 0U;
volatile uint16_t p109_adc_100ms = 0U;
volatile uint16_t p109_adc_250ms = 0U;
volatile uint16_t p109_adc_500ms = 0U;
volatile int16_t p109_coast_100_adc = 0;
volatile int16_t p109_coast_250_adc = 0;
volatile int16_t p109_coast_500_adc = 0;
volatile int16_t p109_final_error_adc = 0;
volatile uint16_t p109_boost_ms = 0U;
volatile uint16_t p109_sustain_ms = 0U;
volatile uint8_t p109_brake_cause = P109_BRAKE_CAUSE_NONE;

static volatile uint8_t p109_pending_result = P109_RESULT_RUNNING;
static volatile uint8_t p109_cap100_done = 0U;
static volatile uint8_t p109_cap250_done = 0U;
static volatile uint8_t p109_cap500_done = 0U;


/* P110 adaptive controller telemetry. */
volatile uint8_t p110_state = P110_STATE_WAIT;
volatile uint8_t p110_result = P110_RESULT_RUNNING;
volatile uint8_t p110_direction = P110_DIR_OPEN;
volatile uint16_t p110_start_adc = 0U;
volatile uint16_t p110_target_adc = 0U;
volatile uint16_t p110_current_adc = 0U;
volatile int16_t p110_error_adc = 0;
volatile uint8_t p110_active_pwm = 0U;
volatile uint8_t p110_learned_breakaway_pwm = 0U;
volatile uint8_t p110_sustain_pwm = 0U;
volatile uint16_t p110_speed_adc_s = 0U;
volatile uint16_t p110_stop_distance_adc = P110_STOP_MIN_ADC;
volatile uint16_t p110_brake_entry_adc = 0U;
volatile uint16_t p110_coast_max_adc = 0U;
volatile uint16_t p110_final_adc = 0U;
volatile int16_t p110_final_error_adc = 0;
volatile uint16_t p110_search_ms = 0U;
volatile uint16_t p110_drive_ms = 0U;
volatile uint16_t p110_brake_ms = 0U;
volatile uint16_t p110_total_powered_ms = 0U;
volatile uint8_t p110_correction_count = 0U;
volatile uint8_t p110_breakaway_found = 0U;
volatile uint8_t p110_abort_reason = P97_ABORT_NONE;

static volatile uint8_t p110_auto_requested = 0U;
static volatile uint32_t p110_state_start_ms = 0UL;
static volatile uint32_t p110_last_eval_ms = 0UL;
static volatile uint16_t p110_eval_adc = 0U;
static volatile uint16_t p110_search_origin_adc = 0U;
static volatile uint8_t p110_search_pwm = P110_SEARCH_START_PWM;
static volatile uint8_t p110_pending_result = P110_RESULT_RUNNING;
static volatile uint32_t p110_slow_since_ms = 0UL;
static volatile uint16_t p110_power_limit_ms = P112R5_MIN_POWERED_LIMIT_MS;
static volatile uint8_t p110_learned_open_pwm = 0U;
static volatile uint8_t p110_learned_close_pwm = 0U;
static volatile uint16_t p110_learned_open_coast = P112_INITIAL_COAST_ADC;
static volatile uint16_t p110_learned_close_coast = P112_INITIAL_COAST_ADC;

/* P111 supervisor telemetry. */
volatile uint8_t p111_state = P111_STATE_WAIT;
volatile uint8_t p111_result = P111_RESULT_RUNNING;
volatile uint8_t p111_cycle = 0U;
volatile uint8_t p111_completed_cycles = 0U;
volatile uint8_t p111_pass_mask = 0U;
volatile uint8_t p111_phase = P111_PHASE_OPEN;
volatile uint8_t p111_moves_completed = 0U;
volatile uint16_t p111_baseline_adc = 0U;
volatile uint16_t p111_open_target_adc = 0U;
volatile uint8_t p111_learned_open_pwm = 0U;
volatile uint8_t p111_learned_close_pwm = 0U;
volatile uint16_t p111_learned_open_coast = P112_INITIAL_COAST_ADC;
volatile uint16_t p111_learned_close_coast = P112_INITIAL_COAST_ADC;
volatile int16_t p111_last_open_error_adc = 0;
volatile int16_t p111_last_close_error_adc = 0;
volatile uint16_t p111_last_open_powered_ms = 0U;
volatile uint16_t p111_last_close_powered_ms = 0U;
volatile uint8_t p111_last_open_breakaway_pwm = 0U;
volatile uint8_t p111_last_close_breakaway_pwm = 0U;
volatile uint16_t p111_last_open_stop_adc = 0U;
volatile uint16_t p111_last_close_stop_adc = 0U;
volatile uint16_t p111_last_open_coast_adc = 0U;
volatile uint16_t p111_last_close_coast_adc = 0U;
volatile uint8_t p111_abort_reason = P97_ABORT_NONE;
volatile uint32_t p112_isr_control_ticks = 0UL;
volatile uint32_t p112_isr_adc_samples = 0UL;
volatile uint32_t p112_hard_off_count = 0UL;
volatile uint32_t p112_uart_suppressed_count = 0UL;
volatile uint32_t p112_sd_suppressed_count = 0UL;
volatile uint16_t p112_isr_last_us = 0U;
volatile uint16_t p112_isr_max_us = 0U;
volatile uint8_t p112_timing_critical = 0U;
volatile uint8_t p112_decel_active = 0U;
volatile uint8_t p112_decel_pwm = 0U;
volatile uint16_t p112_initial_coast_adc = P112_INITIAL_COAST_ADC;
static volatile uint8_t p112_adc_divider = 0U;
static volatile uint8_t p112_vref_divider = 0U;

static uint8_t p87_adc1_prev_valid = 0U;
static uint16_t p87_adc1_prev = 0U;
static uint8_t p87_adc2_prev_valid = 0U;
static uint16_t p87_adc2_prev = 0U;
static uint8_t p87_candidate_prev_valid = 0U;
static uint16_t p87_candidate_prev = 0U;

static uint32_t last_sample_ms = 0UL;
static uint32_t last_probe_ms = 0UL;
static uint32_t window_start_ms = 0UL;

static uint16_t median_ring[P83_MEDIAN_LEN];
static uint8_t median_count = 0U;
static uint8_t median_write = 0U;
static uint8_t filter_initialized = 0U;

static uint8_t raw_prev_valid = 0U;
static uint16_t raw_prev = 0U;
static uint8_t filtered_prev_valid = 0U;
static uint16_t filtered_prev = 0U;
static uint8_t median_prev_valid = 0U;
static uint16_t median_prev = 0U;

static uint8_t trajectory_pending = 0U;
static uint8_t trajectory_confirmed = 0U;
static uint8_t trajectory_count = 0U;
static int8_t trajectory_direction = 0;
static uint16_t trajectory_last_med = 0U;

static uint8_t pair_bad_streak = 0U;
static uint8_t quarantine_active = 0U;
static uint32_t quarantine_start_ms = 0UL;

/* Boot-acquisition cluster. */
static uint16_t acq_count = 0U;
static uint32_t acq_sum = 0UL;
static uint16_t acq_min = 1023U;
static uint16_t acq_max = 0U;
static uint8_t acq_prev_valid = 0U;
static uint16_t acq_prev = 0U;

/* Quarantine recovery cluster. */
static uint16_t recovery_count = 0U;
static uint32_t recovery_sum = 0UL;
static uint16_t recovery_min = 1023U;
static uint16_t recovery_max = 0U;
static uint8_t recovery_prev_valid = 0U;
static uint16_t recovery_prev = 0U;
static uint16_t near_recovery_count = 0U;

static uint16_t work_raw_min = 1023U;
static uint16_t work_raw_max = 0U;
static uint16_t work_filtered_min = 1023U;
static uint16_t work_filtered_max = 0U;
static uint8_t work_vref_valid = 0U;
static uint16_t work_vref_min = 4095U;
static uint16_t work_vref_max = 0U;

static uint16_t P83_AbsDiffU16(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static uint16_t P83_AverageU16(uint16_t a, uint16_t b)
{
    return (uint16_t)(((uint32_t)a + (uint32_t)b + 1UL) / 2UL);
}

static int8_t P83_DirectionU16(uint16_t from, uint16_t to)
{
    if (to > from) return 1;
    if (to < from) return -1;
    return 0;
}

static int8_t P98_ExpectedDirection(void)
{
    if ((p88_jog_used == 0U) || (p88_jog_finished != 0U)) return 0;

    if ((p97_phase == P97_PHASE_PREHOME) ||
        (p97_phase == P97_PHASE_PREHOME_DWELL) ||
        (p97_phase == P97_PHASE_RETURN))
    {
        return 1; /* CLOSE -> ADC should rise. */
    }

    if ((p97_phase == P97_PHASE_OPEN) ||
        (p97_phase == P97_PHASE_TURNAROUND))
    {
        return -1; /* OPEN -> ADC should fall. */
    }

    return 0;
}

static uint8_t P98_MotionContextActive(void)
{
    if ((p88_jog_used == 0U) || (p88_jog_finished != 0U)) return 0U;
    if (p97_phase == P97_PHASE_DONE) return 0U;

    /* Include the active pulse and the brake/settling interval because the
     * motor/gear train can still move after PWM is removed. */
    if ((p88_jog_active != 0U) || (p88_settle_pending != 0U) ||
        (p97_phase == P97_PHASE_PREHOME_DWELL) ||
        (p97_phase == P97_PHASE_TURNAROUND))
    {
        return 1U;
    }
    return 0U;
}

static uint8_t P98_DirectionCompatible(uint16_t from, uint16_t to, int8_t expected)
{
    uint16_t d = P83_AbsDiffU16(from, to);
    if (d <= P98_MOTION_DIR_TOL_ADC) return 1U;
    if (expected < 0) return (to < from) ? 1U : 0U;
    if (expected > 0) return (to > from) ? 1U : 0U;
    return 0U;
}

static uint8_t P98_PairSlopeCompatible(uint16_t a1, uint16_t a2, int8_t expected)
{
    uint16_t d = P83_AbsDiffU16(a1, a2);
    if (d <= P83_PAIR_HARD_DIFF_ADC) return 1U;
    if (d > P98_MOTION_PAIR_HARD_DIFF_ADC) return 0U;

    /* ADC1 is sampled first and ADC2 immediately after it. During a genuine
     * OPEN slope the second sample should be lower; during CLOSE it should be
     * higher. A small tolerance absorbs conversion/noise ordering jitter. */
    if (expected < 0) return ((uint32_t)a2 <= ((uint32_t)a1 + P98_MOTION_DIR_TOL_ADC)) ? 1U : 0U;
    if (expected > 0) return ((uint32_t)a1 <= ((uint32_t)a2 + P98_MOTION_DIR_TOL_ADC)) ? 1U : 0U;
    return 0U;
}

static void P83_ForceMotorSafe(void)
{
    NeedleValveHW_Stop();
    NeedleValveHW_SetEnabled(0U);
    NeedleValveHW_SetBenchJogArm(0U);
}

static void P97_StopBridge(void)
{
    NeedleValveHW_Stop();
    NeedleValveHW_SetEnabled(0U);
    NeedleValveHW_SetBenchJogArm(0U);
    p88_jog_active = 0U;
    p88_ticks_remaining = 0U;
}

/* Dynamic brake for the BTS7960-style dual half-bridge: keep both EN pins
 * asserted while both PWM inputs are low. */
static void P97_EnterBrake(void)
{
    NeedleValveHW_SetBenchJogArm(1U);
    NeedleValveHW_SetEnabled(1U);
    NeedleValveHW_Brake();
    p88_jog_active = 0U;
    p88_ticks_remaining = 0U;
    p88_settle_pending = 1U;
    p88_pulse_stop_ms = HAL_GetTick();
    p97_confirm_count = 0U;
    p97_last_confirm_sample_total = p83_samples_total;
}

static uint8_t P97_FeedbackReady(void)
{
    if ((p83_feedback_valid == 0U) || (p83_mode != P83_MODE_NORMAL)) return 0U;
    return 1U;
}

static int16_t P109_CoastFromEntry(uint16_t sample_adc)
{
    return (int16_t)((int32_t)p109_brake_entry_adc - (int32_t)sample_adc);
}

static void P109_Finish(uint8_t result, uint8_t abort_reason, uint16_t current_adc)
{
    P97_StopBridge();
    p88_settle_pending = 0U;
    p88_abort_reason = abort_reason;
    p88_jog_finished = 1U;
    p97_phase = P97_PHASE_DONE;
    p109_state = P109_STATE_DONE;
    p109_result = result;
    p109_adc_500ms = current_adc;
    p109_coast_500_adc = P109_CoastFromEntry(current_adc);
    p109_final_error_adc = (int16_t)((int32_t)current_adc - (int32_t)p109_target_adc);
    p88_end_adc = current_adc;
    p88_delta_adc = (int16_t)((int32_t)current_adc - (int32_t)p109_start_adc);
}

static void P109_Abort(uint8_t abort_reason)
{
    uint16_t current_adc = p83_filtered_adc;
    P109_Finish(P109_RESULT_ABORT, abort_reason, current_adc);
}

static void P109_EnterBrake(uint8_t cause, uint8_t pending_result)
{
    p109_brake_cause = cause;
    p109_pending_result = pending_result;
    p109_state = P109_STATE_BRAKE;
    p109_brake_entry_adc = p83_filtered_adc;
    p109_adc_100ms = p109_brake_entry_adc;
    p109_adc_250ms = p109_brake_entry_adc;
    p109_adc_500ms = p109_brake_entry_adc;
    p109_coast_100_adc = 0;
    p109_coast_250_adc = 0;
    p109_coast_500_adc = 0;
    p109_cap100_done = 0U;
    p109_cap250_done = 0U;
    p109_cap500_done = 0U;
    P97_EnterBrake();
}

/* Called by the 1 ms ISR after the exact 20 ms boost. No brake is inserted. */
static void __attribute__((unused)) P109_StartSustainFromISR(void)
{
    p109_state = P109_STATE_SUSTAIN;
    p97_active_pwm = P109_SUSTAIN_PWM;
    p97_active_chunk_ms = P109_SUSTAIN_MAX_MS;
    p88_ticks_remaining = P109_SUSTAIN_MAX_MS;
    p88_pulse_start_ms = HAL_GetTick();
    p88_jog_active = 1U;
    p88_settle_pending = 0U;
    p97_open_chunk_count = 2U;
    NeedleValveHW_SetBenchJogArm(1U);
    NeedleValveHW_SetEnabled(1U);
    NeedleValveHW_DriveOpen(P109_SUSTAIN_PWM);
}

static void __attribute__((unused)) P109_AutoStartService(void)
{
    uint32_t now = HAL_GetTick();
    uint16_t start_adc;

    if ((p88_jog_used != 0U) || (p88_jog_active != 0U) ||
        (p88_jog_finished != 0U) || (p88_settle_pending != 0U))
    {
        return;
    }

    if ((uint32_t)(now - p90_service_init_ms) < P97_AUTO_START_DELAY_MS) return;

    p88_jog_used = 1U;
    p88_abort_reason = P97_ABORT_NONE;
    p88_jog_finished = 0U;
    p97_sequence_start_ms = now;
    p97_open_drive_ms = 0U;
    p97_return_drive_ms = 0U;
    p97_open_chunk_count = 0U;
    p97_return_chunk_count = 0U;

    p109_state = P109_STATE_WAIT;
    p109_result = P109_RESULT_RUNNING;
    p109_pending_result = P109_RESULT_RUNNING;
    p109_brake_cause = P109_BRAKE_CAUSE_NONE;
    p109_boost_ms = 0U;
    p109_sustain_ms = 0U;
    p109_brake_entry_adc = 0U;
    p109_adc_100ms = 0U;
    p109_adc_250ms = 0U;
    p109_adc_500ms = 0U;
    p109_coast_100_adc = 0;
    p109_coast_250_adc = 0;
    p109_coast_500_adc = 0;
    p109_final_error_adc = 0;
    p109_cap100_done = 0U;
    p109_cap250_done = 0U;
    p109_cap500_done = 0U;

    if (P97_FeedbackReady() == 0U)
    {
        P109_Abort(P97_ABORT_FEEDBACK_INVALID);
        return;
    }

    start_adc = p83_filtered_adc;
    if ((start_adc < P109_START_MIN_ADC) || (start_adc > P97_MAX_START_ADC))
    {
        P109_Abort(P97_ABORT_START_RANGE);
        return;
    }

    p109_start_adc = start_adc;
    p109_target_adc = (uint16_t)(start_adc - P109_TARGET_DELTA_ADC);
    p109_prebrake_adc = (uint16_t)(p109_target_adc + P109_PREBRAKE_MARGIN_ADC);
    p109_final_error_adc = (int16_t)P109_TARGET_DELTA_ADC;

    p88_start_adc = start_adc;
    p88_end_adc = start_adc;
    p88_delta_adc = 0;
    p88_pulse_elapsed_ms = 0U;

    p97_target_adc = p109_target_adc;
    p97_active_pwm = P109_BOOST_PWM;
    p97_active_chunk_ms = P109_BOOST_MS;
    p97_drive_direction = 0U; /* OPEN */
    p97_phase = P97_PHASE_OPEN;
    p97_open_chunk_count = 1U;

    p88_ticks_remaining = P109_BOOST_MS;
    p88_pulse_start_ms = now;
    p88_jog_active = 1U;
    p88_settle_pending = 0U;
    p109_state = P109_STATE_BOOST;

    NeedleValveHW_SetBenchJogArm(1U);
    NeedleValveHW_SetEnabled(1U);
    NeedleValveHW_DriveOpen(P109_BOOST_PWM);
}

static void __attribute__((unused)) P109_SequenceService(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t brake_elapsed;
    uint16_t current_adc;
    uint16_t wrong_rise;

    if ((p88_jog_used == 0U) || (p88_jog_finished != 0U)) return;

    if ((uint32_t)(now - p97_sequence_start_ms) >= P109_MAX_SEQUENCE_MS)
    {
        P109_Abort(P97_ABORT_SEQUENCE_TIMEOUT);
        return;
    }

    if ((p109_state == P109_STATE_BOOST) || (p109_state == P109_STATE_SUSTAIN))
    {
        if (P97_FeedbackReady() == 0U)
        {
            P109_Abort(P97_ABORT_FEEDBACK_INVALID);
            return;
        }

        current_adc = p83_filtered_adc;
        p88_end_adc = current_adc;
        p88_delta_adc = (int16_t)((int32_t)current_adc - (int32_t)p109_start_adc);
        wrong_rise = (current_adc >= p109_start_adc) ?
                     (uint16_t)(current_adc - p109_start_adc) : 0U;

        if (wrong_rise >= P109_WRONG_DIR_DELTA_ADC)
        {
            P109_Abort(P97_ABORT_WRONG_DIRECTION);
            return;
        }

        /* Hard powered overshoot protection; do not wait for a normal threshold. */
        if ((uint32_t)current_adc + P109_HARD_OVERSHOOT_ADC < (uint32_t)p109_target_adc)
        {
            P109_EnterBrake(P109_BRAKE_CAUSE_OVERSHOOT, P109_RESULT_OVERSHOOT);
            return;
        }

        /* During BOOST the ISR owns the exact 20 ms handoff. */
        if (p109_state == P109_STATE_BOOST) return;

        /* Intentionally brake 20 ADC before the final 50-ADC target. */
        if (current_adc <= p109_prebrake_adc)
        {
            P109_EnterBrake(P109_BRAKE_CAUSE_PREBRAKE, P109_RESULT_CHARACTERIZED);
            return;
        }
        return;
    }

    if (p109_state == P109_STATE_BRAKE)
    {
        NeedleValveHW_SetBenchJogArm(1U);
        NeedleValveHW_SetEnabled(1U);
        NeedleValveHW_Brake();

        if (P97_FeedbackReady() == 0U)
        {
            P109_Abort(P97_ABORT_FEEDBACK_INVALID);
            return;
        }

        current_adc = p83_filtered_adc;
        brake_elapsed = (uint32_t)(now - p88_pulse_stop_ms);

        if ((p109_cap100_done == 0U) && (brake_elapsed >= 100UL))
        {
            p109_adc_100ms = current_adc;
            p109_coast_100_adc = P109_CoastFromEntry(current_adc);
            p109_cap100_done = 1U;
        }
        if ((p109_cap250_done == 0U) && (brake_elapsed >= 250UL))
        {
            p109_adc_250ms = current_adc;
            p109_coast_250_adc = P109_CoastFromEntry(current_adc);
            p109_cap250_done = 1U;
        }
        if ((p109_cap500_done == 0U) && (brake_elapsed >= P109_BRAKE_HOLD_MS))
        {
            p109_adc_500ms = current_adc;
            p109_coast_500_adc = P109_CoastFromEntry(current_adc);
            p109_final_error_adc = (int16_t)((int32_t)current_adc - (int32_t)p109_target_adc);
            p109_cap500_done = 1U;

            if ((p109_pending_result == P109_RESULT_OVERSHOOT) ||
                ((uint32_t)current_adc + P109_HARD_OVERSHOOT_ADC < (uint32_t)p109_target_adc))
            {
                P109_Finish(P109_RESULT_OVERSHOOT, P97_ABORT_OPEN_OVERSHOOT, current_adc);
            }
            else if (p109_pending_result == P109_RESULT_PREBRAKE_NOT_REACHED)
            {
                P109_Finish(P109_RESULT_PREBRAKE_NOT_REACHED, P97_ABORT_DRIVE_TIMEOUT, current_adc);
            }
            else
            {
                P109_Finish(P109_RESULT_CHARACTERIZED, P97_ABORT_NONE, current_adc);
            }
            return;
        }
    }
}


static uint16_t P110_AbsError(uint16_t current_adc)
{
    return P83_AbsDiffU16(current_adc, p110_target_adc);
}

static uint8_t P110_TargetPassed(uint16_t current_adc, uint16_t margin)
{
    if (p110_direction == P110_DIR_OPEN)
    {
        return ((uint32_t)current_adc + (uint32_t)margin < (uint32_t)p110_target_adc) ? 1U : 0U;
    }
    return ((uint32_t)current_adc > ((uint32_t)p110_target_adc + (uint32_t)margin) ? 1U : 0U);
}

static uint16_t P110_CommandedProgress(uint16_t from_adc, uint16_t to_adc)
{
    if (p110_direction == P110_DIR_OPEN)
    {
        return (from_adc > to_adc) ? (uint16_t)(from_adc - to_adc) : 0U;
    }
    return (to_adc > from_adc) ? (uint16_t)(to_adc - from_adc) : 0U;
}

static uint16_t P110_CommandedRemaining(uint16_t current_adc)
{
    if (p110_direction == P110_DIR_OPEN)
    {
        return (current_adc > p110_target_adc) ? (uint16_t)(current_adc - p110_target_adc) : 0U;
    }
    return (p110_target_adc > current_adc) ? (uint16_t)(p110_target_adc - current_adc) : 0U;
}

static void P110_ApplyDrive(uint8_t pwm)
{
    p110_active_pwm = pwm;
    p97_active_pwm = pwm;
    p97_drive_direction = p110_direction;
    p97_phase = (p110_direction == P110_DIR_OPEN) ? P97_PHASE_OPEN : P97_PHASE_RETURN;
    p88_jog_active = 1U;
    p88_settle_pending = 0U;
    NeedleValveHW_SetBenchJogArm(1U);
    NeedleValveHW_SetEnabled(1U);
    if (p110_direction == P110_DIR_OPEN) NeedleValveHW_DriveOpen(pwm);
    else NeedleValveHW_DriveClose(pwm);
}

static void P110_EnterBrake(uint8_t pending_result)
{
    p110_pending_result = pending_result;
    p110_state = P110_STATE_BRAKE;
    p110_state_start_ms = HAL_GetTick();
    p110_brake_entry_adc = p83_filtered_adc;
    p110_coast_max_adc = 0U;
    p110_brake_ms = 0U;
    p110_active_pwm = 0U;
    p112_decel_active = 0U;
    p112_decel_pwm = 0U;
    P97_EnterBrake();
}

static void P110_Finish(uint8_t result, uint8_t abort_reason)
{
    uint16_t current_adc = p83_filtered_adc;
    P97_StopBridge();
    p88_settle_pending = 0U;
    p88_jog_finished = 1U;
    p88_abort_reason = abort_reason;
    p97_phase = P97_PHASE_DONE;
    p110_state = P110_STATE_DONE;
    p110_result = result;
    p110_abort_reason = abort_reason;
    p110_current_adc = current_adc;
    p110_final_adc = current_adc;
    p110_error_adc = (int16_t)((int32_t)current_adc - (int32_t)p110_target_adc);
    p110_final_error_adc = p110_error_adc;
    p88_end_adc = current_adc;
    p88_delta_adc = (int16_t)((int32_t)current_adc - (int32_t)p110_start_adc);
}

static void P110_Abort(uint8_t result, uint8_t abort_reason)
{
    P110_Finish(result, abort_reason);
}

static uint8_t P110_RequestAbsoluteInternal(uint16_t target_adc)
{
    uint16_t current_adc;
    uint16_t move_distance;
    uint32_t power_limit;

    if ((p110_state != P110_STATE_WAIT) && (p110_state != P110_STATE_DONE)) return 0U;
    if (P97_FeedbackReady() == 0U) return 0U;
    if ((target_adc < P110_MIN_TARGET_ADC) || (target_adc > P110_MAX_TARGET_ADC)) return 0U;

    current_adc = p83_filtered_adc;
    if (P83_AbsDiffU16(current_adc, target_adc) <= P110_TARGET_TOL_ADC)
    {
        p110_start_adc = current_adc;
        p110_target_adc = target_adc;
        p110_current_adc = current_adc;
        p110_final_adc = current_adc;
        p110_error_adc = (int16_t)((int32_t)current_adc - (int32_t)target_adc);
        p110_final_error_adc = p110_error_adc;
        p110_result = P110_RESULT_PASS;
        p110_state = P110_STATE_DONE;
        return 1U;
    }

    p110_direction = (target_adc < current_adc) ? P110_DIR_OPEN : P110_DIR_CLOSE;
    p110_start_adc = current_adc;
    p110_target_adc = target_adc;
    p110_current_adc = current_adc;
    p110_error_adc = (int16_t)((int32_t)current_adc - (int32_t)target_adc);
    p110_final_adc = 0U;
    p110_final_error_adc = p110_error_adc;
    p110_speed_adc_s = 0U;
    p110_stop_distance_adc = P110_STOP_MIN_ADC;
    p110_brake_entry_adc = 0U;
    p110_coast_max_adc = 0U;
    p110_search_ms = 0U;
    p110_drive_ms = 0U;
    p110_brake_ms = 0U;
    p110_total_powered_ms = 0U;
    p110_correction_count = 0U;
    p110_breakaway_found = 0U;
    p110_abort_reason = P97_ABORT_NONE;
    p110_pending_result = P110_RESULT_RUNNING;
    p110_slow_since_ms = 0UL;
    p112_decel_active = 0U;
    p112_decel_pwm = 0U;

    move_distance = P83_AbsDiffU16(current_adc, target_adc);
    power_limit = (uint32_t)P112R5_MIN_POWERED_LIMIT_MS +
                  ((uint32_t)move_distance * P112R5_POWERED_MS_PER_ADC);
    if (power_limit > P112R5_MAX_POWERED_LIMIT_MS)
        power_limit = P112R5_MAX_POWERED_LIMIT_MS;
    p110_power_limit_ms = (uint16_t)power_limit;

    p88_jog_used = 1U;
    p88_jog_finished = 0U;
    p88_abort_reason = P97_ABORT_NONE;
    p88_start_adc = current_adc;
    p88_end_adc = current_adc;
    p88_delta_adc = 0;
    p97_target_adc = target_adc;
    p97_sequence_start_ms = HAL_GetTick();
    p97_open_drive_ms = 0U;
    p97_return_drive_ms = 0U;
    p97_open_chunk_count = 0U;
    p97_return_chunk_count = 0U;

    p110_search_origin_adc = current_adc;
    if (p110_direction == P110_DIR_OPEN)
    {
        p110_search_pwm = (p110_learned_open_pwm > P110_SUSTAIN_DROP_PWM) ?
                          (uint8_t)(p110_learned_open_pwm - P110_SUSTAIN_DROP_PWM) :
                          P110_SEARCH_START_PWM;
    }
    else
    {
        p110_search_pwm = (p110_learned_close_pwm > P110_SUSTAIN_DROP_PWM) ?
                          (uint8_t)(p110_learned_close_pwm - P110_SUSTAIN_DROP_PWM) :
                          P110_SEARCH_START_PWM;
    }
    if (p110_search_pwm < P110_SEARCH_START_PWM) p110_search_pwm = P110_SEARCH_START_PWM;
    if (p110_search_pwm > P110_SEARCH_MAX_PWM) p110_search_pwm = P110_SEARCH_MAX_PWM;

    p110_state = P110_STATE_SEARCH;
    p110_result = P110_RESULT_RUNNING;
    p110_state_start_ms = HAL_GetTick();
    p110_last_eval_ms = p110_state_start_ms;
    p110_eval_adc = current_adc;
    P110_ApplyDrive(p110_search_pwm);
    return 1U;
}

uint8_t NeedleValveIntegrationTest_RequestAbsoluteAdaptive(uint16_t target_adc)
{
    return P110_RequestAbsoluteInternal(target_adc);
}

uint8_t NeedleValveIntegrationTest_RequestRelativeAdaptive(int16_t delta_adc)
{
    int32_t target;
    if (P97_FeedbackReady() == 0U) return 0U;
    target = (int32_t)p83_filtered_adc + (int32_t)delta_adc;
    if ((target < (int32_t)P110_MIN_TARGET_ADC) || (target > (int32_t)P110_MAX_TARGET_ADC)) return 0U;
    return P110_RequestAbsoluteInternal((uint16_t)target);
}

uint8_t NeedleValveIntegrationTest_UpdateActiveTarget(uint16_t target_adc)
{
    uint16_t current_adc;
    uint16_t remaining;
    uint8_t new_direction;
    uint32_t extended_limit;

    if ((target_adc < P110_MIN_TARGET_ADC) ||
        (target_adc > P110_MAX_TARGET_ADC) ||
        (P97_FeedbackReady() == 0U)) return 0U;

    current_adc = p83_filtered_adc;

    if (p110_state == P110_STATE_CORRECTION_DWELL)
    {
        P110_Finish(P110_RESULT_PASS, NEEDLE_ADAPTIVE_ABORT_RETARGET);
        return 1U;
    }

    if ((p110_state != P110_STATE_SEARCH) &&
        (p110_state != P110_STATE_DRIVE)) return 0U;

    if (P83_AbsDiffU16(current_adc, target_adc) <= P110_TARGET_TOL_ADC)
    {
        P110_EnterBrake(P110_RESULT_RETARGET);
        return 1U;
    }

    new_direction = (target_adc < current_adc) ? P110_DIR_OPEN : P110_DIR_CLOSE;
    if (new_direction != p110_direction)
    {
        /* Preserve the old direction through BRAKE so coast learning remains
         * physically meaningful. The supervisor starts the reverse move only
         * after this bounded brake phase finishes. */
        P110_EnterBrake(P110_RESULT_RETARGET);
        return 1U;
    }

    p110_target_adc = target_adc;
    p97_target_adc = target_adc;
    p110_error_adc = (int16_t)((int32_t)current_adc - (int32_t)target_adc);

    remaining = P83_AbsDiffU16(current_adc, target_adc);
    extended_limit = (uint32_t)p110_total_powered_ms +
        (uint32_t)P112R5_MIN_POWERED_LIMIT_MS +
        ((uint32_t)remaining * P112R5_POWERED_MS_PER_ADC);
    if (extended_limit > P112R5_MAX_POWERED_LIMIT_MS)
        extended_limit = P112R5_MAX_POWERED_LIMIT_MS;
    if (extended_limit > p110_power_limit_ms)
        p110_power_limit_ms = (uint16_t)extended_limit;

    return 1U;
}

void NeedleValveIntegrationTest_ForceSafe(uint8_t abort_reason)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t was_active;

    __disable_irq();
    was_active = (uint8_t)((p110_state == P110_STATE_SEARCH) ||
                           (p110_state == P110_STATE_DRIVE) ||
                           (p110_state == P110_STATE_BRAKE) ||
                           (p110_state == P110_STATE_CORRECTION_DWELL));
    P97_StopBridge();
    p88_settle_pending = 0U;
    p112_decel_active = 0U;
    p112_decel_pwm = 0U;
    p110_active_pwm = 0U;

    if (was_active != 0U)
    {
        p88_jog_finished = 1U;
        p88_abort_reason = abort_reason;
        p110_state = P110_STATE_DONE;
        p110_result = P110_RESULT_ABORT;
        p110_abort_reason = abort_reason;
        p110_current_adc = p83_filtered_adc;
        p110_final_adc = p83_filtered_adc;
        p110_error_adc = (int16_t)((int32_t)p83_filtered_adc -
                                   (int32_t)p110_target_adc);
        p110_final_error_adc = p110_error_adc;
    }

    if (primask == 0U) __enable_irq();
}

void NeedleValveIntegrationTest_GetAdaptiveLearning(
    uint8_t *open_breakaway_pwm,
    uint8_t *close_breakaway_pwm,
    uint16_t *open_coast_adc,
    uint16_t *close_coast_adc)
{
    if (open_breakaway_pwm != 0) *open_breakaway_pwm = p110_learned_open_pwm;
    if (close_breakaway_pwm != 0) *close_breakaway_pwm = p110_learned_close_pwm;
    if (open_coast_adc != 0) *open_coast_adc = p110_learned_open_coast;
    if (close_coast_adc != 0) *close_coast_adc = p110_learned_close_coast;
}

static void P110_RestartCorrection(void)
{
    uint16_t current_adc = p83_filtered_adc;
    p110_correction_count++;
    p110_search_origin_adc = current_adc;
    p110_breakaway_found = 0U;
    p110_speed_adc_s = 0U;
    p110_search_ms = 0U;
    p110_drive_ms = 0U;
    p110_brake_ms = 0U;
    p110_slow_since_ms = 0UL;

    if (p110_direction == P110_DIR_OPEN)
    {
        p110_search_pwm = (p110_learned_open_pwm > P110_SUSTAIN_DROP_PWM) ?
                          (uint8_t)(p110_learned_open_pwm - P110_SUSTAIN_DROP_PWM) :
                          P110_SEARCH_START_PWM;
    }
    else
    {
        p110_search_pwm = (p110_learned_close_pwm > P110_SUSTAIN_DROP_PWM) ?
                          (uint8_t)(p110_learned_close_pwm - P110_SUSTAIN_DROP_PWM) :
                          P110_SEARCH_START_PWM;
    }
    if (p110_search_pwm < P110_SEARCH_START_PWM) p110_search_pwm = P110_SEARCH_START_PWM;

    p110_state = P110_STATE_SEARCH;
    p110_state_start_ms = HAL_GetTick();
    p110_last_eval_ms = p110_state_start_ms;
    p110_eval_adc = current_adc;
    P110_ApplyDrive(p110_search_pwm);
}

static void P110_UpdateSpeedAndStopDistance(uint32_t now, uint16_t current_adc)
{
    uint32_t dt;
    uint16_t progress;
    uint32_t inst_speed;
    uint32_t predicted;
    uint16_t learned_coast;

    dt = (uint32_t)(now - p110_last_eval_ms);
    if (dt < P110_EVAL_PERIOD_MS) return;
    if (dt == 0UL) dt = 1UL;

    progress = P110_CommandedProgress(p110_eval_adc, current_adc);
    inst_speed = ((uint32_t)progress * 1000UL) / dt;
    if (inst_speed > 4000UL) inst_speed = 4000UL;
    p110_speed_adc_s = (uint16_t)(((uint32_t)p110_speed_adc_s * 3UL + inst_speed) / 4UL);

    learned_coast = (p110_direction == P110_DIR_OPEN) ? p110_learned_open_coast : p110_learned_close_coast;
    predicted = (uint32_t)P110_STOP_BASE_ADC + (uint32_t)learned_coast +
                (((uint32_t)p110_speed_adc_s * P110_BRAKE_HORIZON_MS) / 1000UL);
    if (predicted < P110_STOP_MIN_ADC) predicted = P110_STOP_MIN_ADC;
    if (predicted > P110_STOP_MAX_ADC) predicted = P110_STOP_MAX_ADC;
    p110_stop_distance_adc = (uint16_t)predicted;

    p110_eval_adc = current_adc;
    p110_last_eval_ms = now;
}


static void P112_UpdateLearnedCoast(uint16_t measured)
{
    if (p110_direction == P110_DIR_OPEN)
    {
        if (measured >= p110_learned_open_coast) p110_learned_open_coast = measured;
        else p110_learned_open_coast = (uint16_t)(((uint32_t)p110_learned_open_coast * 3UL + measured + 2UL) / 4UL);
        if (p110_learned_open_coast < P110_STOP_MIN_ADC) p110_learned_open_coast = P110_STOP_MIN_ADC;
    }
    else
    {
        if (measured >= p110_learned_close_coast) p110_learned_close_coast = measured;
        else p110_learned_close_coast = (uint16_t)(((uint32_t)p110_learned_close_coast * 3UL + measured + 2UL) / 4UL);
        if (p110_learned_close_coast < P110_STOP_MIN_ADC) p110_learned_close_coast = P110_STOP_MIN_ADC;
    }
}

static void __attribute__((unused)) P110_SequenceService(void)
{
    uint32_t now = HAL_GetTick();
    uint16_t current_adc;
    uint16_t progress;
    uint16_t remaining;
    uint16_t coast;
    uint8_t learned;
    uint8_t sustain;

    if ((p110_state == P110_STATE_WAIT) || (p110_state == P110_STATE_DONE)) return;

    if ((uint32_t)(now - p97_sequence_start_ms) >= P110_MAX_SEQUENCE_MS)
    {
        P110_Abort(P110_RESULT_ABORT, P97_ABORT_SEQUENCE_TIMEOUT);
        return;
    }
    if (P97_FeedbackReady() == 0U)
    {
        P110_Abort(P110_RESULT_ABORT, P97_ABORT_FEEDBACK_INVALID);
        return;
    }

    current_adc = p83_filtered_adc;
    p110_current_adc = current_adc;
    p110_error_adc = (int16_t)((int32_t)current_adc - (int32_t)p110_target_adc);
    p88_end_adc = current_adc;
    p88_delta_adc = (int16_t)((int32_t)current_adc - (int32_t)p110_start_adc);

    if ((p110_state == P110_STATE_SEARCH) || (p110_state == P110_STATE_DRIVE))
    {
        if (P110_TargetPassed(current_adc, P110_HARD_OVERSHOOT_ADC) != 0U)
        {
            P110_EnterBrake(P110_RESULT_OVERSHOOT);
            return;
        }
        if (p110_total_powered_ms >= p110_power_limit_ms)
        {
            P110_EnterBrake(P110_RESULT_POWER_TIMEOUT);
            return;
        }
    }

    if (p110_state == P110_STATE_SEARCH)
    {
        progress = P110_CommandedProgress(p110_search_origin_adc, current_adc);
        if (progress >= P110_BREAKAWAY_MOVE_ADC)
        {
            p110_breakaway_found = 1U;
            p110_learned_breakaway_pwm = p110_search_pwm;
            if (p110_direction == P110_DIR_OPEN) p110_learned_open_pwm = p110_search_pwm;
            else p110_learned_close_pwm = p110_search_pwm;

            sustain = (p110_search_pwm > P110_SUSTAIN_DROP_PWM) ?
                      (uint8_t)(p110_search_pwm - P110_SUSTAIN_DROP_PWM) : P110_SUSTAIN_MIN_PWM;
            if (sustain < P110_SUSTAIN_MIN_PWM) sustain = P110_SUSTAIN_MIN_PWM;
            p110_sustain_pwm = sustain;
            p110_state = P110_STATE_DRIVE;
            p110_state_start_ms = now;
            p110_last_eval_ms = now;
            p110_eval_adc = current_adc;
            P110_ApplyDrive(sustain);
            return;
        }

        if ((uint32_t)(now - p110_state_start_ms) >= P110_SEARCH_STEP_MS)
        {
            p110_state_start_ms = now;
            if (p110_search_pwm >= P110_SEARCH_MAX_PWM)
            {
                P110_EnterBrake(P110_RESULT_NO_BREAKAWAY);
                return;
            }
            p110_search_pwm = (uint8_t)(p110_search_pwm + P110_SEARCH_STEP_PWM);
            if (p110_search_pwm > P110_SEARCH_MAX_PWM) p110_search_pwm = P110_SEARCH_MAX_PWM;
            P110_ApplyDrive(p110_search_pwm);
        }
        return;
    }

    if (p110_state == P110_STATE_DRIVE)
    {
        P110_UpdateSpeedAndStopDistance(now, current_adc);
        remaining = P110_CommandedRemaining(current_adc);
        if (remaining <= p110_stop_distance_adc)
        {
            p112_decel_active = 0U;
            p112_decel_pwm = 0U;
            P110_EnterBrake(P110_RESULT_RUNNING);
            return;
        }

        /* P112: reduce drive before the predicted brake point. This makes the
         * first move conservative even before coast learning has converged. */
        if (remaining <= (uint16_t)(p110_stop_distance_adc + P112_DECEL_ZONE_EXTRA_ADC))
        {
            uint8_t decel = (p110_sustain_pwm > P112_DECEL_DROP_PWM) ?
                            (uint8_t)(p110_sustain_pwm - P112_DECEL_DROP_PWM) :
                            P112_DECEL_MIN_PWM;
            if (decel < P112_DECEL_MIN_PWM) decel = P112_DECEL_MIN_PWM;
            p112_decel_active = 1U;
            p112_decel_pwm = decel;
            P110_ApplyDrive(decel);
            p110_slow_since_ms = 0UL;
            return;
        }
        p112_decel_active = 0U;
        p112_decel_pwm = 0U;

        /* If the loaded mechanism slows/stalls after breakaway, raise sustain
         * automatically instead of requiring another firmware/PWM experiment.
         * This is still bounded by P110_SEARCH_MAX_PWM. */
        if (p110_speed_adc_s < P110_SLOW_SPEED_ADC_S)
        {
            if (p110_slow_since_ms == 0UL) p110_slow_since_ms = now;
            if ((uint32_t)(now - p110_slow_since_ms) >= P110_SLOW_RAISE_MS)
            {
                if (p110_sustain_pwm < P110_SEARCH_MAX_PWM)
                {
                    uint16_t raised = (uint16_t)p110_sustain_pwm + P110_SUSTAIN_RAISE_PWM;
                    p110_sustain_pwm = (raised > P110_SEARCH_MAX_PWM) ?
                                       P110_SEARCH_MAX_PWM : (uint8_t)raised;
                }
                p110_slow_since_ms = now;
            }
        }
        else
        {
            p110_slow_since_ms = 0UL;
        }

        P110_ApplyDrive(p110_sustain_pwm);
        return;
    }

    if (p110_state == P110_STATE_BRAKE)
    {
        NeedleValveHW_SetBenchJogArm(1U);
        NeedleValveHW_SetEnabled(1U);
        NeedleValveHW_Brake();
        p110_brake_ms = (uint16_t)(now - p110_state_start_ms);
        coast = P110_CommandedProgress(p110_brake_entry_adc, current_adc);
        if (coast > p110_coast_max_adc) p110_coast_max_adc = coast;

        if ((uint32_t)(now - p110_state_start_ms) < P110_BRAKE_HOLD_MS) return;

        learned = (uint8_t)((p110_coast_max_adc > 255U) ? 255U : p110_coast_max_adc);
        P112_UpdateLearnedCoast((uint16_t)learned);

        p110_final_adc = current_adc;
        p110_final_error_adc = (int16_t)((int32_t)current_adc - (int32_t)p110_target_adc);

        if (p110_pending_result == P110_RESULT_OVERSHOOT)
        {
            P110_Finish(P110_RESULT_OVERSHOOT, P97_ABORT_OPEN_OVERSHOOT);
            return;
        }
        if (p110_pending_result == P110_RESULT_NO_BREAKAWAY)
        {
            P110_Finish(P110_RESULT_NO_BREAKAWAY, P97_ABORT_DRIVE_TIMEOUT);
            return;
        }
        if (p110_pending_result == P110_RESULT_POWER_TIMEOUT)
        {
            P110_Finish(P110_RESULT_POWER_TIMEOUT, P97_ABORT_DRIVE_TIMEOUT);
            return;
        }
        if (p110_pending_result == P110_RESULT_RETARGET)
        {
            P110_Finish(P110_RESULT_PASS, NEEDLE_ADAPTIVE_ABORT_RETARGET);
            return;
        }
        if (P110_TargetPassed(current_adc, P110_HARD_OVERSHOOT_ADC) != 0U)
        {
            P110_Finish(P110_RESULT_OVERSHOOT, P97_ABORT_OPEN_OVERSHOOT);
            return;
        }
        if (P110_AbsError(current_adc) <= P110_TARGET_TOL_ADC)
        {
            P110_Finish(P110_RESULT_PASS, P97_ABORT_NONE);
            return;
        }

        /* P112R10R2 adaptive correction-feasibility guard.
         * First retain the symmetric bounded settled guard for +/-9..12 ADC.
         * Then, only on the UNDERSHOOT side (remaining > 0), use the coast
         * measured during this brake as an estimate of the minimum useful
         * powered correction quantum. This prevents a 13-ADC residual from
         * launching a ~25-30 ADC breakaway/coast event, while still allowing
         * a correction when the residual is materially larger than the
         * mechanism's observed stopping quantum. */
        if (P110_AbsError(current_adc) <= P110_SETTLED_ACCEPT_ADC)
        {
            P110_Finish(P110_RESULT_PASS, P97_ABORT_NONE);
            return;
        }

        remaining = P110_CommandedRemaining(current_adc);
        if (remaining > 0U)
        {
            uint16_t correction_quantum_adc = p110_coast_max_adc;
            if (correction_quantum_adc < P110_SETTLED_ACCEPT_ADC)
                correction_quantum_adc = P110_SETTLED_ACCEPT_ADC;
            if (correction_quantum_adc > P110_CORRECTION_QUANTUM_MAX_ADC)
                correction_quantum_adc = P110_CORRECTION_QUANTUM_MAX_ADC;

            if (remaining <= correction_quantum_adc)
            {
                P110_Finish(P110_RESULT_PASS, P97_ABORT_NONE);
                return;
            }
        }

        if ((remaining > P110_TARGET_TOL_ADC) && (p110_correction_count < P110_MAX_CORRECTIONS))
        {
            p110_state = P110_STATE_CORRECTION_DWELL;
            p110_state_start_ms = now;
            p88_settle_pending = 0U;
            P97_StopBridge();
            return;
        }

        P110_Finish(P110_RESULT_ABORT, P97_ABORT_DRIVE_TIMEOUT);
        return;
    }

    if (p110_state == P110_STATE_CORRECTION_DWELL)
    {
        P97_StopBridge();
        if ((uint32_t)(now - p110_state_start_ms) >= P110_CORRECTION_DWELL_MS)
        {
            P110_RestartCorrection();
        }
    }
}

static void P83_ClearMedianRing(void)
{
    uint8_t i;
    for (i = 0U; i < P83_MEDIAN_LEN; ++i) median_ring[i] = 0U;
    median_count = 0U;
    median_write = 0U;
    median_prev_valid = 0U;
}

static void P83_ResetTrajectory(void)
{
    trajectory_pending = 0U;
    trajectory_confirmed = 0U;
    trajectory_count = 0U;
    trajectory_direction = 0;
    trajectory_last_med = 0U;
}

static void P83_ResetWindow(void)
{
    work_raw_min = 1023U;
    work_raw_max = 0U;
    work_filtered_min = 1023U;
    work_filtered_max = 0U;
    work_vref_valid = 0U;
    work_vref_min = 4095U;
    work_vref_max = 0U;
}

static void P83_UpdateTimeouts(void)
{
    p83_adc_timeout_count = needle_valve_hw_adc_timeout_count +
                            needle_valve_hw_adc2_timeout_count +
                            needle_valve_hw_vref_timeout_count;
    if (p83_adc_timeout_count != 0UL) p83_diag_flags |= P83_FLAG_ADC_TIMEOUT;
}

static uint16_t P83_MedianAccepted(void)
{
    uint16_t tmp[P83_MEDIAN_LEN];
    uint8_t n = median_count;
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < n; ++i) tmp[i] = median_ring[i];
    for (i = 1U; i < n; ++i)
    {
        uint16_t key = tmp[i];
        j = i;
        while ((j > 0U) && (tmp[(uint8_t)(j - 1U)] > key))
        {
            tmp[j] = tmp[(uint8_t)(j - 1U)];
            j--;
        }
        tmp[j] = key;
    }
    return tmp[n / 2U];
}

static void P83_PushMedianSample(uint16_t v)
{
    median_ring[median_write] = v;
    median_write++;
    if (median_write >= P83_MEDIAN_LEN) median_write = 0U;
    if (median_count < P83_MEDIAN_LEN) median_count++;
}

static uint8_t P83_PairConfidence(uint16_t diff)
{
    if (diff <= 8U) return 100U;
    if (diff <= 16U) return 85U;
    if (diff <= 24U) return 70U;
    if (diff <= P83_PAIR_HARD_DIFF_ADC) return 50U;
    return 0U;
}

static void P83_UpdateConfidence(uint8_t sample_score)
{
    uint32_t next;
    if (p83_samples_total <= 1UL)
    {
        p83_confidence_pct = sample_score;
        return;
    }
    next = ((uint32_t)p83_confidence_pct * 7UL + (uint32_t)sample_score + 4UL) / 8UL;
    if (next > 100UL) next = 100UL;
    p83_confidence_pct = (uint8_t)next;
}

static void P87_TrackRawSweep(uint16_t a1, uint16_t a2, uint16_t candidate, uint16_t pair_diff)
{
    uint16_t step;

    p87_sweep_samples++;
    if (a1 < p87_adc1_min) p87_adc1_min = a1;
    if (a1 > p87_adc1_max) p87_adc1_max = a1;
    if (a2 < p87_adc2_min) p87_adc2_min = a2;
    if (a2 > p87_adc2_max) p87_adc2_max = a2;
    if (candidate < p87_candidate_min) p87_candidate_min = candidate;
    if (candidate > p87_candidate_max) p87_candidate_max = candidate;
    if (pair_diff > p87_pair_diff_max) p87_pair_diff_max = pair_diff;

    if (p87_adc1_prev_valid != 0U)
    {
        step = P83_AbsDiffU16(a1, p87_adc1_prev);
        if (step > p87_adc1_max_step)
        {
            p87_adc1_max_step = step;
            p87_adc1_step_from = p87_adc1_prev;
            p87_adc1_step_to = a1;
        }
    }
    p87_adc1_prev = a1;
    p87_adc1_prev_valid = 1U;

    if (p87_adc2_prev_valid != 0U)
    {
        step = P83_AbsDiffU16(a2, p87_adc2_prev);
        if (step > p87_adc2_max_step)
        {
            p87_adc2_max_step = step;
            p87_adc2_step_from = p87_adc2_prev;
            p87_adc2_step_to = a2;
        }
    }
    p87_adc2_prev = a2;
    p87_adc2_prev_valid = 1U;

    if (p87_candidate_prev_valid != 0U)
    {
        step = P83_AbsDiffU16(candidate, p87_candidate_prev);
        if (step > p87_candidate_max_step)
        {
            p87_candidate_max_step = step;
            p87_candidate_step_from = p87_candidate_prev;
            p87_candidate_step_to = candidate;
        }
        if (step > P87_RAW_GAP_EVENT_THRESHOLD_ADC) p87_raw_gap_event_count++;
    }
    p87_candidate_prev = candidate;
    p87_candidate_prev_valid = 1U;
}

static uint16_t P83_UpdateWindowRaw(uint16_t raw)
{
    uint16_t d = 0U;
    if (raw < work_raw_min) work_raw_min = raw;
    if (raw > work_raw_max) work_raw_max = raw;
    if (raw_prev_valid != 0U) d = P83_AbsDiffU16(raw, raw_prev);
    raw_prev = raw;
    raw_prev_valid = 1U;
    return d;
}

static void P83_UpdateWindowFiltered(uint16_t filtered)
{
    if (filtered_prev_valid != 0U)
    {
        uint16_t step = P83_AbsDiffU16(filtered, filtered_prev);
        if (step > p87_filtered_max_step)
        {
            p87_filtered_max_step = step;
            p87_filtered_step_from = filtered_prev;
            p87_filtered_step_to = filtered;
        }
        if (step > P87_FILTERED_GAP_EVENT_THRESHOLD_ADC) p87_filtered_gap_event_count++;
    }
    if (filtered < work_filtered_min) work_filtered_min = filtered;
    if (filtered > work_filtered_max) work_filtered_max = filtered;
    filtered_prev = filtered;
    filtered_prev_valid = 1U;
}

static void P83_HoldFiltered(void)
{
    if (filter_initialized != 0U) P83_UpdateWindowFiltered(p83_filtered_adc);
}

static void P83_SetFilteredNormal(uint16_t target)
{
    uint16_t d;
    uint16_t proposed;

    if (filter_initialized == 0U) return;

    d = P83_AbsDiffU16(target, p83_filtered_adc);
    if (d <= P83_OUTPUT_DEADBAND_ADC)
    {
        P83_UpdateWindowFiltered(p83_filtered_adc);
        return;
    }

    proposed = (uint16_t)(((uint32_t)p83_filtered_adc * 3UL +
                           (uint32_t)target + 2UL) / 4UL);

    if (proposed > p83_filtered_adc)
    {
        uint16_t step = (uint16_t)(proposed - p83_filtered_adc);
        if (step > P83_OUTPUT_MAX_STEP_ADC) step = P83_OUTPUT_MAX_STEP_ADC;
        p83_filtered_adc = (uint16_t)(p83_filtered_adc + step);
    }
    else
    {
        uint16_t step = (uint16_t)(p83_filtered_adc - proposed);
        if (step > P83_OUTPUT_MAX_STEP_ADC) step = P83_OUTPUT_MAX_STEP_ADC;
        p83_filtered_adc = (uint16_t)(p83_filtered_adc - step);
    }
    P83_UpdateWindowFiltered(p83_filtered_adc);
}

static void P83_ResetAcquisition(uint8_t set_flag)
{
    acq_count = 0U;
    acq_sum = 0UL;
    acq_min = 1023U;
    acq_max = 0U;
    acq_prev_valid = 0U;
    acq_prev = 0U;
    p83_acq_progress_pct = 0U;
    if (set_flag != 0U) p83_diag_flags |= P83_FLAG_ACQ_RESET;
}

static void P83_AcquisitionStartWith(uint16_t med)
{
    acq_count = 1U;
    acq_sum = (uint32_t)med;
    acq_min = med;
    acq_max = med;
    acq_prev = med;
    acq_prev_valid = 1U;
    p83_acq_progress_pct = 1U;
}

static void P83_ProcessBootAcquisition(uint16_t med)
{
    uint16_t step = 0U;
    uint16_t next_min;
    uint16_t next_max;

    p83_median7 = med;

    if (acq_count == 0U)
    {
        P83_AcquisitionStartWith(med);
        return;
    }

    if (acq_prev_valid != 0U) step = P83_AbsDiffU16(med, acq_prev);
    next_min = (med < acq_min) ? med : acq_min;
    next_max = (med > acq_max) ? med : acq_max;

    if ((step > P83_ACQ_MAX_STEP_ADC) ||
        ((uint16_t)(next_max - next_min) > P83_ACQ_CLUSTER_SPREAD_ADC))
    {
        P83_ResetAcquisition(1U);
        P83_AcquisitionStartWith(med);
        return;
    }

    acq_prev = med;
    acq_prev_valid = 1U;
    acq_min = next_min;
    acq_max = next_max;
    if (acq_count < P83_BOOT_ACQ_STABLE_SAMPLES)
    {
        acq_count++;
        acq_sum += (uint32_t)med;
    }

    p83_acq_progress_pct = (uint8_t)(((uint32_t)acq_count * 100UL) /
                                      (uint32_t)P83_BOOT_ACQ_STABLE_SAMPLES);

    if (acq_count >= P83_BOOT_ACQ_STABLE_SAMPLES)
    {
        uint16_t anchor = (uint16_t)((acq_sum + (uint32_t)(acq_count / 2U)) /
                                     (uint32_t)acq_count);
        p83_filtered_adc = anchor;
        filter_initialized = 1U;
        filtered_prev_valid = 0U;
        P83_UpdateWindowFiltered(anchor);
        median_prev = med;
        median_prev_valid = 1U;
        P83_ResetTrajectory();
        p83_mode = P83_MODE_NORMAL;
        p83_acq_progress_pct = 100U;
    }
}

static void P83_ResetRecoveryCluster(void)
{
    recovery_count = 0U;
    recovery_sum = 0UL;
    recovery_min = 1023U;
    recovery_max = 0U;
    recovery_prev_valid = 0U;
    recovery_prev = 0U;
    near_recovery_count = 0U;
    p83_acq_progress_pct = 0U;
}

static void P83_RecoveryStartWith(uint16_t med)
{
    recovery_count = 1U;
    recovery_sum = (uint32_t)med;
    recovery_min = med;
    recovery_max = med;
    recovery_prev = med;
    recovery_prev_valid = 1U;
    near_recovery_count = 0U;
}

static void P83_EnterQuarantine(uint32_t now)
{
    if (filter_initialized == 0U)
    {
        P83_ResetAcquisition(1U);
        P83_ClearMedianRing();
        return;
    }

    if (quarantine_active == 0U) p83_quarantine_count++;
    quarantine_active = 1U;
    quarantine_start_ms = now;
    pair_bad_streak = 0U;
    P83_ResetTrajectory();
    P83_ClearMedianRing();
    P83_ResetRecoveryCluster();
    p83_mode = P83_MODE_QUARANTINE;
    p83_diag_flags |= P83_FLAG_QUARANTINE_EVENT | P83_FLAG_FEEDBACK_INVALID;
}

static void P83_ExitQuarantineNear(uint16_t med)
{
    quarantine_active = 0U;
    P83_ResetRecoveryCluster();
    median_prev = med;
    median_prev_valid = 1U;
    P83_ResetTrajectory();
    p83_mode = P83_MODE_NORMAL;
    p83_acq_progress_pct = 100U;
}

static void P83_ExitQuarantineReanchor(uint16_t anchor, uint16_t med)
{
    /* P83 is motor-locked. Re-anchoring while feedback is invalid is safe for
     * this diagnostic image only; never carry this unconditional rule into a
     * motor-enabled controller. */
    p83_filtered_adc = anchor;
    filter_initialized = 1U;
    filtered_prev_valid = 0U;
    P83_UpdateWindowFiltered(anchor);
    p83_reacquire_count++;
    p83_diag_flags |= P83_FLAG_SAFE_REACQUIRE;

    quarantine_active = 0U;
    P83_ResetRecoveryCluster();
    median_prev = med;
    median_prev_valid = 1U;
    P83_ResetTrajectory();
    p83_mode = P83_MODE_NORMAL;
    p83_acq_progress_pct = 100U;
}

static void P83_ProcessQuarantineMedian(uint16_t med, uint32_t now)
{
    uint16_t step = 0U;
    uint16_t next_min;
    uint16_t next_max;
    uint32_t elapsed = (uint32_t)(now - quarantine_start_ms);

    p83_median7 = med;

    if (recovery_count == 0U)
    {
        P83_RecoveryStartWith(med);
    }
    else
    {
        if (recovery_prev_valid != 0U) step = P83_AbsDiffU16(med, recovery_prev);
        next_min = (med < recovery_min) ? med : recovery_min;
        next_max = (med > recovery_max) ? med : recovery_max;

        if ((step > P83_ACQ_MAX_STEP_ADC) ||
            ((uint16_t)(next_max - next_min) > P83_ACQ_CLUSTER_SPREAD_ADC))
        {
            p83_diag_flags |= P83_FLAG_ACQ_RESET;
            P83_ResetRecoveryCluster();
            P83_RecoveryStartWith(med);
        }
        else
        {
            recovery_prev = med;
            recovery_prev_valid = 1U;
            recovery_min = next_min;
            recovery_max = next_max;
            if (recovery_count < P83_FAR_REACQUIRE_STABLE_SAMPLES)
            {
                recovery_count++;
                recovery_sum += (uint32_t)med;
            }
        }
    }

    if ((P83_AbsDiffU16(med, p83_filtered_adc) <= P83_NEAR_RECOVERY_BAND_ADC) &&
        (step <= P83_NEAR_RECOVERY_MAX_STEP_ADC))
    {
        if (near_recovery_count < P83_NEAR_RECOVERY_STABLE_SAMPLES) near_recovery_count++;
    }
    else
    {
        near_recovery_count = 0U;
    }

    if (recovery_count < P83_FAR_REACQUIRE_STABLE_SAMPLES)
    {
        p83_acq_progress_pct = (uint8_t)(((uint32_t)recovery_count * 100UL) /
                                          (uint32_t)P83_FAR_REACQUIRE_STABLE_SAMPLES);
    }
    else
    {
        p83_acq_progress_pct = 100U;
    }

    if ((elapsed >= P83_QUARANTINE_MIN_MS) &&
        (near_recovery_count >= P83_NEAR_RECOVERY_STABLE_SAMPLES))
    {
        P83_ExitQuarantineNear(med);
        P83_HoldFiltered();
        return;
    }

    if ((P98_MotionContextActive() == 0U) &&
        (elapsed >= P83_QUARANTINE_MIN_MS) &&
        (recovery_count >= P83_FAR_REACQUIRE_STABLE_SAMPLES))
    {
        uint16_t anchor = (uint16_t)((recovery_sum + (uint32_t)(recovery_count / 2U)) /
                                     (uint32_t)recovery_count);
        P83_ExitQuarantineReanchor(anchor, med);
        P83_HoldFiltered();
        return;
    }

    P83_HoldFiltered();
}

static void P83_ProcessMedian(uint16_t med, uint32_t now)
{
    uint16_t med_step = 0U;
    uint16_t from_filtered;
    uint16_t prev_med = median_prev;
    uint8_t prev_med_valid = median_prev_valid;
    uint8_t motion = P98_MotionContextActive();
    int8_t expected = P98_ExpectedDirection();
    int8_t dir;

    p83_median7 = med;

    if (filter_initialized == 0U)
    {
        P83_ProcessBootAcquisition(med);
        return;
    }

    if (quarantine_active != 0U)
    {
        P83_ProcessQuarantineMedian(med, now);
        return;
    }

    if (prev_med_valid != 0U) med_step = P83_AbsDiffU16(med, prev_med);
    median_prev = med;
    median_prev_valid = 1U;

    if (med_step > P83_MEDIAN_MAX_STEP_ADC)
    {
        uint8_t motion_ok = 0U;
        if ((motion != 0U) && (expected != 0) &&
            (med_step <= P98_MOTION_MEDIAN_MAX_STEP_ADC) &&
            (prev_med_valid != 0U) &&
            (P98_DirectionCompatible(prev_med, med, expected) != 0U))
        {
            motion_ok = 1U;
        }

        if (motion_ok == 0U)
        {
            p83_rate_reject_count++;
            p83_diag_flags |= P83_FLAG_RATE_REJECT;
            P83_EnterQuarantine(now);
            P83_HoldFiltered();
            return;
        }
    }

    from_filtered = P83_AbsDiffU16(med, p83_filtered_adc);
    if (from_filtered <= P83_TRAJECTORY_START_ADC)
    {
        P83_ResetTrajectory();
        P83_SetFilteredNormal(med);
        return;
    }

    /* During a commanded actuator move, a direction-consistent trajectory is
     * expected, not an unexplained sensor re-anchor. Follow it directly using
     * the existing output slew cap. Opposite-direction excursions still fall
     * through to the conservative P83 trajectory/quarantine logic. */
    if ((motion != 0U) && (expected != 0) &&
        (P98_DirectionCompatible(p83_filtered_adc, med, expected) != 0U))
    {
        P83_ResetTrajectory();
        P83_SetFilteredNormal(med);
        return;
    }

    dir = P83_DirectionU16(p83_filtered_adc, med);

    if (trajectory_confirmed != 0U)
    {
        if (dir == trajectory_direction)
        {
            trajectory_last_med = med;
            P83_SetFilteredNormal(med);
            return;
        }
        P83_ResetTrajectory();
    }

    if (trajectory_pending == 0U)
    {
        trajectory_pending = 1U;
        trajectory_count = 1U;
        trajectory_direction = dir;
        trajectory_last_med = med;
        P83_HoldFiltered();
        return;
    }

    if (dir != trajectory_direction)
    {
        trajectory_count = 1U;
        trajectory_direction = dir;
        trajectory_last_med = med;
        P83_HoldFiltered();
        return;
    }

    if (((trajectory_direction > 0) &&
         (((uint32_t)med + (uint32_t)P83_TRAJECTORY_REV_TOL_ADC) < (uint32_t)trajectory_last_med)) ||
        ((trajectory_direction < 0) &&
         (((uint32_t)trajectory_last_med + (uint32_t)P83_TRAJECTORY_REV_TOL_ADC) < (uint32_t)med)))
    {
        trajectory_count = 1U;
        trajectory_last_med = med;
        P83_HoldFiltered();
        return;
    }

    trajectory_last_med = med;
    if (trajectory_count < 255U) trajectory_count++;

    if (trajectory_count >= P83_TRAJECTORY_CONFIRM_SAMPLES)
    {
        trajectory_pending = 0U;
        trajectory_confirmed = 1U;
        P83_SetFilteredNormal(med);
    }
    else
    {
        P83_HoldFiltered();
    }
}

static void P83_UpdateValidity(void)
{
    p83_diag_flags &= ~P83_CURRENT_FLAG_MASK;

    if (filter_initialized == 0U)
    {
        p83_mode = P83_MODE_ACQUIRE;
        p83_feedback_valid = 0U;
        p83_diag_flags |= P83_FLAG_ACQUIRING | P83_FLAG_FEEDBACK_INVALID;
        return;
    }

    if (p83_confidence_pct < P83_VALID_CONFIDENCE_PCT)
    {
        p83_feedback_valid = 0U;
        p83_diag_flags |= P83_FLAG_LOW_CONFIDENCE | P83_FLAG_FEEDBACK_INVALID;
        return;
    }

    if (p83_adc_timeout_count != 0UL)
    {
        p83_feedback_valid = 0U;
        p83_diag_flags |= P83_FLAG_FEEDBACK_INVALID;
        return;
    }

    if (quarantine_active != 0U)
    {
        p83_mode = P83_MODE_QUARANTINE;
        p83_feedback_valid = 0U;
        p83_diag_flags |= P83_FLAG_FEEDBACK_INVALID;
        if (recovery_count >= P83_NEAR_RECOVERY_STABLE_SAMPLES)
            p83_diag_flags |= P83_FLAG_RECOVERY_STABLE;
        return;
    }

    if (trajectory_pending != 0U)
    {
        p83_mode = P83_MODE_TRAJECTORY;
        p83_feedback_valid = 0U;
        p83_diag_flags |= P83_FLAG_TRAJECTORY_PENDING | P83_FLAG_FEEDBACK_INVALID;
        return;
    }

    p83_mode = P83_MODE_NORMAL;
    p83_acq_progress_pct = 100U;
    p83_feedback_valid = 1U;
}

static void P83_SampleRobust(void)
{
    uint32_t now = HAL_GetTick();
    /* P112R5: median-of-3 on each ADC, retained during autonomous movement. */
    uint16_t a1 = NeedleValveHW_ReadPotADC10Burst3();
    uint16_t a2 = NeedleValveHW_ReadPotADC2ADC10Burst3();
    uint16_t diff = P83_AbsDiffU16(a1, a2);
    uint16_t raw_mid = P83_AverageU16(a1, a2);
    uint16_t prev_raw = raw_prev;
    uint8_t prev_raw_valid = raw_prev_valid;
    uint16_t raw_step;
    uint8_t motion = P98_MotionContextActive();
    int8_t expected = P98_ExpectedDirection();
    uint8_t pair_motion_ok = 0U;
    uint8_t raw_motion_ok = 0U;
    uint8_t score = P83_PairConfidence(diff);

    p83_adc1_raw = a1;
    p83_adc2_raw = a2;
    p83_pair_diff = diff;
    p83_pair_candidate = raw_mid;
    P87_TrackRawSweep(a1, a2, raw_mid, diff);
    p83_samples_total++;
    raw_step = P83_UpdateWindowRaw(raw_mid);

    if ((motion != 0U) && (expected != 0) &&
        (P98_PairSlopeCompatible(a1, a2, expected) != 0U))
    {
        pair_motion_ok = 1U;
        if (score == 0U) score = 50U;
    }

    if ((motion != 0U) && (expected != 0) && (prev_raw_valid != 0U) &&
        (raw_step <= P98_MOTION_RAW_MAX_STEP_ADC) &&
        (P98_DirectionCompatible(prev_raw, raw_mid, expected) != 0U))
    {
        raw_motion_ok = 1U;
    }

    if ((diff > P83_PAIR_HARD_DIFF_ADC) && (pair_motion_ok == 0U))
    {
        p83_pair_reject_count++;
        p83_diag_flags |= P83_FLAG_PAIR_REJECT;
        P83_UpdateConfidence(0U);
        if (pair_bad_streak < 255U) pair_bad_streak++;

        if (filter_initialized == 0U)
        {
            P83_ResetAcquisition(1U);
            P83_ClearMedianRing();
        }
        else if (pair_bad_streak >= P83_PAIR_BAD_STREAK_QUARANTINE)
        {
            P83_EnterQuarantine(now);
        }
        P83_HoldFiltered();
    }
    else if ((prev_raw_valid != 0U) && (raw_step > P83_RAW_MAX_STEP_ADC) &&
             (raw_motion_ok == 0U))
    {
        pair_bad_streak = 0U;
        p83_rate_reject_count++;
        p83_diag_flags |= P83_FLAG_RATE_REJECT;
        P83_UpdateConfidence(score);
        if (filter_initialized == 0U)
        {
            P83_ResetAcquisition(1U);
            P83_ClearMedianRing();
        }
        else
        {
            P83_EnterQuarantine(now);
        }
        P83_HoldFiltered();
    }
    else
    {
        pair_bad_streak = 0U;
        P83_PushMedianSample(raw_mid);
        P83_UpdateConfidence(score);
        if (median_count >= P83_MEDIAN_MIN_VALID)
            P83_ProcessMedian(P83_MedianAccepted(), now);
    }

    P83_UpdateTimeouts();
    P83_UpdateValidity();
}

static void P83_RunVrefProbe(void)
{
    uint16_t vref = NeedleValveHW_ReadVrefRaw12();
    if (vref != 0U)
    {
        if (vref < work_vref_min) work_vref_min = vref;
        if (vref > work_vref_max) work_vref_max = vref;
        work_vref_valid = 1U;
    }
    P83_UpdateTimeouts();
}

static void P83_FinalizeWindow(void)
{
    if (work_raw_max >= work_raw_min)
        p83_win_raw_pp = (uint16_t)(work_raw_max - work_raw_min);
    else
        p83_win_raw_pp = 0U;

    if ((filter_initialized != 0U) && (work_filtered_max >= work_filtered_min))
        p83_win_filtered_pp = (uint16_t)(work_filtered_max - work_filtered_min);
    else
        p83_win_filtered_pp = 0U;

    if (work_vref_valid != 0U)
    {
        p83_vref_win_pp_raw12 = (uint16_t)(work_vref_max - work_vref_min);
        if (p83_vref_win_pp_raw12 >= P83_VREF_PP_WARN_RAW12)
            p83_diag_flags |= P83_FLAG_VREF_MOVE;
    }
    else
    {
        p83_vref_win_pp_raw12 = 0U;
    }

    P83_ResetWindow();
}

NeedleValveStatus_t NeedleValveIntegrationTest_GetTelemetryStatus(void)
{
    NeedleValveStatus_t s = {0};
    s.requested_cmd = (p88_jog_active != 0U) ? ((float)p97_active_pwm / 255.0f) : 0.0f;
    s.limited_cmd = s.requested_cmd;
    s.raw_adc = p83_filtered_adc;
    s.zero_adc = p88_start_adc;
    s.target_adc = p97_target_adc;
    s.error_adc = (int16_t)((int32_t)p83_filtered_adc - (int32_t)p97_target_adc);
    s.enabled = (uint8_t)((p88_jog_active != 0U) || (p88_settle_pending != 0U));
    s.zero_valid = p88_jog_used;
    s.position_locked = p83_feedback_valid;
    s.homing_active = (uint8_t)((p88_jog_used != 0U) && (p88_jog_finished == 0U));
    s.homing_complete = p88_jog_finished;
    s.fault = (p88_abort_reason == P97_ABORT_NONE) ? NEEDLE_VALVE_FAULT_NONE : NEEDLE_VALVE_FAULT_ADC_INVALID;
    s.lpwm = ((p88_jog_active != 0U) && (p97_drive_direction == 0U)) ? p97_active_pwm : 0U;
    s.rpwm = ((p88_jog_active != 0U) && (p97_drive_direction != 0U)) ? p97_active_pwm : 0U;
    return s;
}


uint8_t NeedleValveIntegrationTest_IsTimingCritical(void)
{
    if ((p110_state == P110_STATE_SEARCH) ||
        (p110_state == P110_STATE_DRIVE) ||
        (p110_state == P110_STATE_BRAKE)) return 1U;
    return 0U;
}

void NeedleValveIntegrationTest_Init(void)
{
    uint32_t now = HAL_GetTick();

    SolenoidOutput_ForceSafe();
    NeedleValveController_Stop();
    P83_ForceMotorSafe();

    p88_jog_used = 0U;
    p88_jog_active = 0U;
    p88_jog_finished = 0U;
    p88_abort_reason = P97_ABORT_NONE;
    p88_start_adc = 0U;
    p88_end_adc = 0U;
    p88_delta_adc = 0;
    p88_pulse_elapsed_ms = 0U;
    p88_ticks_remaining = 0U;
    p88_settle_pending = 0U;
    p88_pulse_stop_ms = 0UL;
    p88_pulse_start_ms = 0UL;
    p90_service_init_ms = now;
    p97_target_adc = 0U;
    p97_sequence_start_ms = 0UL;
    p97_turnaround_start_ms = 0UL;
    p97_open_drive_ms = 0U;
    p97_return_drive_ms = 0U;
    p97_active_pwm = 0U;
    p97_active_chunk_ms = 0U;
    p97_open_chunk_count = 0U;
    p97_return_chunk_count = 0U;
    p97_prev_eval_adc = 0U;
    p97_return_start_adc = 0U;
    p97_confirm_count = 0U;
    p97_last_confirm_sample_total = 0UL;
    p97_phase = P97_PHASE_PREHOME;
    p97_drive_direction = 0U;
    p98_best_open_adc = 1023U;
    p98_best_close_adc = 0U;
    p105_cycle_index = 0U;
    p105_completed_cycles = 0U;
    p105_pass_mask = 0U;
    p105_last_cycle_abort = P97_ABORT_NONE;
    p105_last_start_adc = 0U;
    p105_last_target_adc = 0U;
    p105_last_open_end_adc = 0U;
    p105_last_open_error_adc = 0;
    p105_last_home_adc = 0U;
    p105_last_open_chunks = 0U;
    p105_last_open_drive_ms = 0U;
    p105_last_return_chunks = 0U;
    p105_last_return_drive_ms = 0U;
    p105_intercycle_start_ms = 0UL;
    p105_current_open_target_adc = 0U;
    p105_current_open_end_adc = 0U;
    p105_current_open_error_adc = 0;

    p106_state = P106_STATE_WAIT;
    p106_stage = 0U;
    p106_test_pwm = 0U;
    p106_start_adc = 0U;
    p106_stage_start_adc = 0U;
    p106_end_adc = 0U;
    p106_delta_adc = 0;
    p106_breakaway_found = 0U;
    p106_breakaway_pwm = 0U;
    p106_result = P106_RESULT_RUNNING;
    p107_state = P107_STATE_WAIT;
    p107_result = P107_RESULT_RUNNING;
    p107_start_adc = 0U;
    p107_target_adc = 0U;
    p107_end_adc = 0U;
    p107_error_adc = 0;
    p107_boost_pwm = P107_BOOST_PWM;
    p107_sustain_pwm = P107_SUSTAIN_PWM;
    p107_boost_ms = 0U;
    p107_sustain_ms = 0U;
    p107_max_open_drop_adc = 0U;
    p107_brake_cause = P107_BRAKE_CAUSE_NONE;
    p107_pending_result = P107_RESULT_RUNNING;
    p109_state = P109_STATE_WAIT;
    p109_result = P109_RESULT_RUNNING;
    p109_start_adc = 0U;
    p109_target_adc = 0U;
    p109_prebrake_adc = 0U;
    p109_brake_entry_adc = 0U;
    p109_adc_100ms = 0U;
    p109_adc_250ms = 0U;
    p109_adc_500ms = 0U;
    p109_coast_100_adc = 0;
    p109_coast_250_adc = 0;
    p109_coast_500_adc = 0;
    p109_final_error_adc = 0;
    p109_boost_ms = 0U;
    p109_sustain_ms = 0U;
    p109_brake_cause = P109_BRAKE_CAUSE_NONE;
    p109_pending_result = P109_RESULT_RUNNING;
    p109_cap100_done = 0U;
    p109_cap250_done = 0U;
    p109_cap500_done = 0U;

    p110_state = P110_STATE_WAIT;
    p110_result = P110_RESULT_RUNNING;
    p110_direction = P110_DIR_OPEN;
    p110_start_adc = 0U;
    p110_target_adc = 0U;
    p110_current_adc = 0U;
    p110_error_adc = 0;
    p110_active_pwm = 0U;
    p110_learned_breakaway_pwm = 0U;
    p110_sustain_pwm = 0U;
    p110_speed_adc_s = 0U;
    p110_stop_distance_adc = P110_STOP_MIN_ADC;
    p110_brake_entry_adc = 0U;
    p110_coast_max_adc = 0U;
    p110_final_adc = 0U;
    p110_final_error_adc = 0;
    p110_search_ms = 0U;
    p110_drive_ms = 0U;
    p110_brake_ms = 0U;
    p110_total_powered_ms = 0U;
    p110_correction_count = 0U;
    p110_breakaway_found = 0U;
    p110_abort_reason = P97_ABORT_NONE;
    p110_auto_requested = 0U;
    p110_state_start_ms = 0UL;
    p110_last_eval_ms = 0UL;
    p110_eval_adc = 0U;
    p110_search_origin_adc = 0U;
    p110_search_pwm = P110_SEARCH_START_PWM;
    p110_pending_result = P110_RESULT_RUNNING;
    p110_slow_since_ms = 0UL;
    p110_power_limit_ms = P112R5_MIN_POWERED_LIMIT_MS;
    p110_learned_open_pwm = 0U;
    p110_learned_close_pwm = 0U;
    p110_learned_open_coast = P112_INITIAL_COAST_ADC;
    p110_learned_close_coast = P112_INITIAL_COAST_ADC;

    p111_state = P111_STATE_WAIT;
    p111_result = P111_RESULT_RUNNING;
    p111_cycle = 0U;
    p111_completed_cycles = 0U;
    p111_pass_mask = 0U;
    p111_phase = P111_PHASE_CLOSE;
    p111_moves_completed = 0U;
    p111_baseline_adc = 0U;
    p111_open_target_adc = 0U;
    p111_learned_open_pwm = 0U;
    p111_learned_close_pwm = 0U;
    p111_learned_open_coast = P112_INITIAL_COAST_ADC;
    p111_learned_close_coast = P112_INITIAL_COAST_ADC;
    p111_last_open_error_adc = 0;
    p111_last_close_error_adc = 0;
    p111_last_open_powered_ms = 0U;
    p111_last_close_powered_ms = 0U;
    p111_last_open_breakaway_pwm = 0U;
    p111_last_close_breakaway_pwm = 0U;
    p111_last_open_stop_adc = 0U;
    p111_last_close_stop_adc = 0U;
    p111_last_open_coast_adc = 0U;
    p111_last_close_coast_adc = 0U;
    p111_abort_reason = P97_ABORT_NONE;
    p112_isr_control_ticks = 0UL;
    p112_isr_adc_samples = 0UL;
    p112_hard_off_count = 0UL;
    p112_uart_suppressed_count = 0UL;
    p112_sd_suppressed_count = 0UL;
    p112_isr_last_us = 0U;
    p112_isr_max_us = 0U;
    p112_timing_critical = 0U;
    p112_decel_active = 0U;
    p112_decel_pwm = 0U;
    p112_initial_coast_adc = P112_INITIAL_COAST_ADC;
    p112_adc_divider = 0U;
    p112_vref_divider = 0U;

    p83_diag_flags = 0UL;
    p83_samples_total = 0UL;
    p83_adc1_raw = 0U;
    p83_adc2_raw = 0U;
    p83_pair_diff = 0U;
    p83_pair_candidate = 0U;
    p83_median7 = 0U;
    p83_filtered_adc = 0U;
    p83_feedback_valid = 0U;
    p83_confidence_pct = 0U;
    p83_pair_reject_count = 0UL;
    p83_rate_reject_count = 0UL;
    p83_quarantine_count = 0UL;
    p83_reacquire_count = 0UL;
    p83_mode = P83_MODE_ACQUIRE;
    p83_acq_progress_pct = 0U;
    p83_adc_timeout_count = 0UL;
    p83_win_raw_pp = 0U;
    p83_win_filtered_pp = 0U;
    p83_vref_win_pp_raw12 = 0U;

    p87_sweep_samples = 0UL;
    p87_adc1_min = 1023U;
    p87_adc1_max = 0U;
    p87_adc2_min = 1023U;
    p87_adc2_max = 0U;
    p87_candidate_min = 1023U;
    p87_candidate_max = 0U;
    p87_adc1_max_step = 0U;
    p87_adc1_step_from = 0U;
    p87_adc1_step_to = 0U;
    p87_adc2_max_step = 0U;
    p87_adc2_step_from = 0U;
    p87_adc2_step_to = 0U;
    p87_candidate_max_step = 0U;
    p87_candidate_step_from = 0U;
    p87_candidate_step_to = 0U;
    p87_filtered_max_step = 0U;
    p87_filtered_step_from = 0U;
    p87_filtered_step_to = 0U;
    p87_raw_gap_event_count = 0UL;
    p87_filtered_gap_event_count = 0UL;
    p87_pair_diff_max = 0U;
    p87_adc1_prev_valid = 0U;
    p87_adc1_prev = 0U;
    p87_adc2_prev_valid = 0U;
    p87_adc2_prev = 0U;
    p87_candidate_prev_valid = 0U;
    p87_candidate_prev = 0U;

    P83_ClearMedianRing();
    filter_initialized = 0U;
    raw_prev_valid = 0U;
    filtered_prev_valid = 0U;
    filtered_prev = 0U;
    median_prev_valid = 0U;
    median_prev = 0U;
    P83_ResetTrajectory();
    pair_bad_streak = 0U;
    quarantine_active = 0U;
    quarantine_start_ms = 0UL;
    P83_ResetAcquisition(0U);
    P83_ResetRecoveryCluster();
    P83_ResetWindow();

    last_sample_ms = now - P83_SAMPLE_PERIOD_MS;
    last_probe_ms = now - P83_PROBE_PERIOD_MS;
    window_start_ms = now;

    P83_SampleRobust();
    P83_RunVrefProbe();

    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LINK_LED_GPIO_Port, LINK_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ESTOP_LED_GPIO_Port, ESTOP_LED_Pin, GPIO_PIN_RESET);
}

void NeedleValveIntegrationTest_FastButtonService(void)
{
    /* P112R1: ADC1 is TIM7-owned while commissioning is active.
     * Do NOT touch ADC1/VREF from main context: in P112 this raced with
     * P83_SampleRobust() and produced one pair reject at roughly the
     * 20 Hz VREF-probe cadence, preventing boot acquisition forever. */
    (void)last_sample_ms;
    (void)last_probe_ms;
}

void NeedleValveIntegrationTest_Update(void)
{
    uint32_t now = HAL_GetTick();

#if (APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE == 0U)
    SolenoidOutput_ForceSafe();
    if ((p88_jog_active == 0U) && (p88_settle_pending == 0U)) P83_ForceMotorSafe();
#endif

    if ((uint32_t)(now - window_start_ms) >= P83_WINDOW_PERIOD_MS)
    {
        window_start_ms = now;
        P83_FinalizeWindow();
    }
}

void NeedleValveIntegrationTest_TimerTickISR(void)
{
    uint32_t t0 = micros();
    uint32_t dt;

    p112_isr_control_ticks++;
    /* Preserve the long-standing full-system health counter even though the
     * P112 adaptive core, rather than the legacy PD loop, owns TIM7. */
    needle_valve_control_tick_count = p112_isr_control_ticks;

    /* Deterministic 500 Hz dual-ADC robust feedback acquisition. */
    p112_adc_divider++;
    if (p112_adc_divider >= P112_ISR_ADC_DIVIDER)
    {
        p112_adc_divider = 0U;
        P83_SampleRobust();
        p112_isr_adc_samples++;
    }

    /* P112R1: serialize the 20 Hz ADC1 VREF probe in the same TIM7
     * context as the PC1 dual-ADC sample. NeedleValveHW_ReadVrefRaw12()
     * temporarily changes ADC1 SQR3/SMPR1; keeping both operations in one
     * context removes the P112 ADC1 register race completely. */
    p112_vref_divider++;
    if (p112_vref_divider >= P112_VREF_ISR_DIVIDER)
    {
        p112_vref_divider = 0U;
        P83_RunVrefProbe();
    }

    /* Account powered time before the state machine makes this tick's choice. */
    if (((p110_state == P110_STATE_SEARCH) || (p110_state == P110_STATE_DRIVE)) &&
        (p88_jog_active != 0U))
    {
        if (p110_total_powered_ms < 65535U) p110_total_powered_ms++;
        if (p110_state == P110_STATE_SEARCH)
        {
            if (p110_search_ms < 65535U) p110_search_ms++;
        }
        else
        {
            if (p110_drive_ms < 65535U) p110_drive_ms++;
        }
        if (p110_direction == P110_DIR_OPEN)
        {
            if (p97_open_drive_ms < 65535U) p97_open_drive_ms++;
        }
        else
        {
            if (p97_return_drive_ms < 65535U) p97_return_drive_ms++;
        }

        /* Hard powered-time deadline independent of UART/SD/main-loop load. */
        if (p110_total_powered_ms >= p110_power_limit_ms)
        {
            p112_hard_off_count++;
            P110_EnterBrake(P110_RESULT_POWER_TIMEOUT);
        }
    }

    P110_SequenceService();

    p112_timing_critical = NeedleValveIntegrationTest_IsTimingCritical();

    if (p110_state == P110_STATE_BRAKE)
    {
        NeedleValveHW_SetBenchJogArm(1U);
        NeedleValveHW_SetEnabled(1U);
        NeedleValveHW_Brake();
    }
    else if ((p110_state != P110_STATE_SEARCH) && (p110_state != P110_STATE_DRIVE))
    {
        if (p88_settle_pending != 0U)
        {
            NeedleValveHW_SetBenchJogArm(1U);
            NeedleValveHW_SetEnabled(1U);
            NeedleValveHW_Brake();
        }
        else if (p88_jog_active == 0U)
        {
            NeedleValveHW_Stop();
            NeedleValveHW_SetEnabled(0U);
            NeedleValveHW_SetBenchJogArm(0U);
        }
    }

    dt = (uint32_t)(micros() - t0);
    if (dt > 65535UL) dt = 65535UL;
    p112_isr_last_us = (uint16_t)dt;
    if (p112_isr_last_us > p112_isr_max_us) p112_isr_max_us = p112_isr_last_us;
}
