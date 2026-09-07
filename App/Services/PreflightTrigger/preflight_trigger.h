#ifndef APP_SERVICES_PREFLIGHT_TRIGGER_H
#define APP_SERVICES_PREFLIGHT_TRIGGER_H

#include <stdint.h>

typedef enum
{
    PREFLIGHT_TRIGGER_BOOT_CALIBRATION = 0,
    PREFLIGHT_TRIGGER_WAIT_CONNECTOR = 1,
    PREFLIGHT_TRIGGER_WAIT_SEPARATION = 2,
    PREFLIGHT_TRIGGER_FLIGHT_ACTIVE = 3,
    PREFLIGHT_TRIGGER_EARLY_SEPARATION_FAULT = 4
} PreflightTriggerState_t;

typedef struct
{
    uint8_t raw_open;
    uint8_t debounced_open;
    uint8_t connector_seen;
    uint8_t preflight_ready;
    uint8_t flight_active;
    uint8_t fault_latched;
    uint8_t imu_ready;
    uint8_t attitude_ready;
    uint8_t eskf_ready;
    uint8_t lidar_reference_ready;
    uint8_t barometer_reference_ready;
    uint8_t needle_ready;
    /* P48: logger must be healthy before separation; an in-flight logger
     * failure remains diagnostic and does not inhibit stabilization. */
    uint8_t sd_ready;
    uint8_t state;
    uint32_t boot_elapsed_ms;
    uint32_t separation_timestamp_ms;
    uint32_t flight_time_ms;
    uint32_t raw_transition_count;
    uint32_t debounce_reject_count;
    uint32_t separation_event_count;
} PreflightTriggerStatus_t;

void PreflightTrigger_Init(void);
void PreflightTrigger_Update(void);
uint8_t PreflightTrigger_IsFlightActive(void);
uint8_t PreflightTrigger_HasFault(void);
PreflightTriggerStatus_t PreflightTrigger_GetStatus(void);

/* Global symbol for CubeIDE Live Expressions. */
extern PreflightTriggerStatus_t preflight_trigger_status;

#endif
