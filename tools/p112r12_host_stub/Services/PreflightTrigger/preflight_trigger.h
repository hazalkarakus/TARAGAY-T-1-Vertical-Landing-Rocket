#ifndef P112R11_HOST_PREFLIGHT_H
#define P112R11_HOST_PREFLIGHT_H
#include <stdint.h>
typedef struct {
 uint8_t raw_open, debounced_open, connector_seen, preflight_ready, flight_active, fault_latched;
 uint8_t imu_ready, attitude_ready, eskf_ready, lidar_reference_ready, barometer_reference_ready, needle_ready, sd_ready, state;
 uint32_t boot_elapsed_ms, separation_timestamp_ms, flight_time_ms, raw_transition_count, debounce_reject_count, separation_event_count;
} PreflightTriggerStatus_t;
extern PreflightTriggerStatus_t p112r11_host_pf;
static inline PreflightTriggerStatus_t PreflightTrigger_GetStatus(void){return p112r11_host_pf;}
#endif
