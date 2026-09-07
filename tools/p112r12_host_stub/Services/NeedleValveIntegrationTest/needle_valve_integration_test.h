#ifndef P112R5_HOST_ADAPTIVE_H
#define P112R5_HOST_ADAPTIVE_H

#include <stdint.h>

#define NEEDLE_ADAPTIVE_STATE_WAIT              0U
#define NEEDLE_ADAPTIVE_STATE_SEARCH            1U
#define NEEDLE_ADAPTIVE_STATE_DRIVE             2U
#define NEEDLE_ADAPTIVE_STATE_BRAKE             3U
#define NEEDLE_ADAPTIVE_STATE_CORRECTION_DWELL  4U
#define NEEDLE_ADAPTIVE_STATE_DONE              5U
#define NEEDLE_ADAPTIVE_RESULT_RUNNING          0U
#define NEEDLE_ADAPTIVE_RESULT_PASS             1U
#define NEEDLE_ADAPTIVE_RESULT_ABORT            2U
#define NEEDLE_ADAPTIVE_DIR_OPEN                0U
#define NEEDLE_ADAPTIVE_DIR_CLOSE               1U
#define NEEDLE_ADAPTIVE_ABORT_FEEDBACK_INVALID  1U
#define NEEDLE_ADAPTIVE_ABORT_RETARGET          10U

uint8_t NeedleValveIntegrationTest_RequestAbsoluteAdaptive(uint16_t target_adc);
uint8_t NeedleValveIntegrationTest_UpdateActiveTarget(uint16_t target_adc);
void NeedleValveIntegrationTest_ForceSafe(uint8_t abort_reason);
void NeedleValveIntegrationTest_GetAdaptiveLearning(
    uint8_t *open_breakaway_pwm, uint8_t *close_breakaway_pwm,
    uint16_t *open_coast_adc, uint16_t *close_coast_adc);

extern volatile uint8_t p83_feedback_valid;
extern volatile uint8_t p83_mode;
extern volatile uint16_t p83_filtered_adc;
extern volatile uint8_t p110_state;
extern volatile uint8_t p110_result;
extern volatile uint8_t p110_direction;
extern volatile uint16_t p110_target_adc;
extern volatile uint8_t p110_active_pwm;
extern volatile uint8_t p110_learned_breakaway_pwm;
extern volatile uint16_t p110_stop_distance_adc;
extern volatile uint16_t p110_coast_max_adc;
extern volatile int16_t p110_final_error_adc;
extern volatile uint16_t p110_total_powered_ms;
extern volatile uint8_t p110_abort_reason;
extern volatile uint32_t p112_isr_control_ticks;
extern volatile uint16_t p112_initial_coast_adc;

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

#endif
