#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.h"
#include "Services/NeedleValveIntegrationTest/needle_valve_integration_test.h"
#include "Services/PreflightTrigger/preflight_trigger.h"

uint32_t p112r5_host_tick_ms;
uint8_t p112r11_host_button;
PreflightTriggerStatus_t p112r11_host_pf;
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
volatile uint16_t p112_initial_coast_adc=20U;
volatile uint8_t p111_state,p111_result,p111_cycle,p111_completed_cycles,p111_pass_mask,p111_phase,p111_moves_completed;
volatile uint16_t p111_baseline_adc,p111_open_target_adc;
volatile uint8_t p111_learned_open_pwm,p111_learned_close_pwm;
volatile uint16_t p111_learned_open_coast,p111_learned_close_coast;
volatile int16_t p111_last_open_error_adc,p111_last_close_error_adc;
volatile uint16_t p111_last_open_powered_ms,p111_last_close_powered_ms;
volatile uint8_t p111_last_open_breakaway_pwm,p111_last_close_breakaway_pwm;
volatile uint16_t p111_last_open_stop_adc,p111_last_close_stop_adc,p111_last_open_coast_adc,p111_last_close_coast_adc;
volatile uint8_t p111_abort_reason;
static uint32_t req_count, safe_count;

uint8_t NeedleValveIntegrationTest_RequestAbsoluteAdaptive(uint16_t t){
 p110_target_adc=t; req_count++; p110_direction=(t<p83_filtered_adc)?NEEDLE_ADAPTIVE_DIR_OPEN:NEEDLE_ADAPTIVE_DIR_CLOSE;
 p110_state=NEEDLE_ADAPTIVE_STATE_SEARCH; p110_result=NEEDLE_ADAPTIVE_RESULT_RUNNING; return 1U; }
uint8_t NeedleValveIntegrationTest_UpdateActiveTarget(uint16_t t){p110_target_adc=t;return 1U;}
void NeedleValveIntegrationTest_ForceSafe(uint8_t r){(void)r;safe_count++; if(p110_state!=NEEDLE_ADAPTIVE_STATE_DONE){p110_state=NEEDLE_ADAPTIVE_STATE_DONE;p110_result=NEEDLE_ADAPTIVE_RESULT_ABORT;}}
void NeedleValveIntegrationTest_GetAdaptiveLearning(uint8_t *o,uint8_t*c,uint16_t*oc,uint16_t*cc){*o=136;*c=128;*oc=25;*cc=26;}

static void healthy(void){
 p112r11_host_pf=(PreflightTriggerStatus_t){0}; p112r11_host_pf.connector_seen=1; p112r11_host_pf.preflight_ready=1;
 p112r11_host_pf.imu_ready=1; p112r11_host_pf.eskf_ready=1; p112r11_host_pf.lidar_reference_ready=1;
 p112r11_host_pf.barometer_reference_ready=1; p112r11_host_pf.sd_ready=1;
 p83_feedback_valid=1; p83_mode=1; p83_filtered_adc=950; needle_valve_fault=0; p112r11_host_button=0;
}
static void update(uint32_t ms){p112r5_host_tick_ms=ms; NeedleValveAutonomousControl_CommissioningUpdate();}
static void timer(uint32_t ms){p112r5_host_tick_ms=ms; NeedleValveAutonomousControl_TimerTickISR();}
static void press_and_start(void){
 update(800); assert(NeedleValveAutonomousControl_GetCommissioningState()==1U);
 p112r11_host_button=1; update(810); update(871);
 assert(NeedleValveAutonomousControl_GetCommissioningState()==2U);
 timer(871); assert(req_count==1U);
 p112r11_host_button=0; update(900); update(965);
}

int main(void){
 NeedleValveAutonomousStatus_t a;

 /* R11R2 exact floor regression: 949 must be rejected, 950 must be accepted. */
 req_count=safe_count=0; healthy(); p83_filtered_adc=949; NeedleValveAutonomousControl_Init();
 assert(NeedleValveAutonomousControl_CaptureClosedReference()==0U);
 p83_filtered_adc=950;
 assert(NeedleValveAutonomousControl_CaptureClosedReference()==1U);
 printf("P112R11R2 CLOSED floor 949 reject / 950 accept: PASS\n");

 req_count=safe_count=0; healthy(); NeedleValveAutonomousControl_Init();
 assert(NeedleValveAutonomousControl_CaptureClosedReference()==1U);
 press_and_start();
 assert(NeedleValveAutonomousControl_GetCommissioningOpenTargetAdc()==872U);
 assert(p110_target_adc==872U);

 /* Complete OPEN at 938 (+2). */
 p83_filtered_adc=874; p110_final_error_adc=2; p110_total_powered_ms=220; p110_coast_max_adc=28;
 p110_state=NEEDLE_ADAPTIVE_STATE_DONE; p110_result=NEEDLE_ADAPTIVE_RESULT_PASS; timer(970); update(975);
 a=NeedleValveAutonomousControl_GetStatus(); assert(a.moves_completed==1U);
 assert(NeedleValveAutonomousControl_GetCommissioningState()==3U);

 /* R11R1 regression: unrelated sensor/reference/SD readiness may flicker after
  * the inert stroke has started. This must NOT abort or strand at OPEN. */
 p112r11_host_pf.preflight_ready=0; p112r11_host_pf.imu_ready=0; p112r11_host_pf.eskf_ready=0;
 p112r11_host_pf.lidar_reference_ready=0; p112r11_host_pf.barometer_reference_ready=0; p112r11_host_pf.sd_ready=0;
 for(uint32_t t=1000;t<1730;t+=40){ update(t); timer(t); assert(NeedleValveAutonomousControl_GetCommissioningState()!=6U); }
 update(1735); timer(1735);
 assert(NeedleValveAutonomousControl_GetCommissioningState()==4U);
 assert(p110_target_adc==950U); assert(req_count==2U);

 /* Complete CLOSE. */
 p83_filtered_adc=948; p110_final_error_adc=-2; p110_total_powered_ms=190; p110_coast_max_adc=18;
 p110_state=NEEDLE_ADAPTIVE_STATE_DONE; p110_result=NEEDLE_ADAPTIVE_RESULT_PASS; timer(1750); update(1755);
 a=NeedleValveAutonomousControl_GetStatus(); assert(a.moves_completed==2U);
 assert(NeedleValveAutonomousControl_GetCommissioningState()==5U);
 assert(NeedleValveAutonomousControl_GetCommissioningAbortReason()==0U);
 printf("P112R11R2 baseline 950 + sensor/reference readiness flicker -> CLOSE completion: PASS\n");

 /* PE9 separation remains an immediate hard-safe abort. */
 req_count=0; healthy(); p112r5_host_tick_ms=0; NeedleValveAutonomousControl_Init();
 assert(NeedleValveAutonomousControl_CaptureClosedReference()==1U); press_and_start();
 p112r11_host_pf.debounced_open=1; update(1000);
 assert(NeedleValveAutonomousControl_GetCommissioningState()==6U);
 assert(NeedleValveAutonomousControl_GetCommissioningAbortReason()==1U);
 assert(NeedleValveAutonomousControl_GetCommissioningAbortCount()==1U); assert(safe_count>0U);
 printf("P112R11R2 PE9 separation hard-safe abort: PASS\n");

 /* P83 feedback loss remains an immediate hard-safe abort. */
 req_count=0; healthy(); p112r5_host_tick_ms=0; NeedleValveAutonomousControl_Init();
 assert(NeedleValveAutonomousControl_CaptureClosedReference()==1U); press_and_start();
 p83_feedback_valid=0; update(1000);
 assert(NeedleValveAutonomousControl_GetCommissioningState()==6U);
 assert(NeedleValveAutonomousControl_GetCommissioningAbortReason()==4U);
 printf("P112R11R2 P83 feedback-loss hard-safe abort: PASS\n");
 return 0;
}
