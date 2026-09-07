#ifndef APP_H
#define APP_H

#include <stdint.h>

void App_Init(void);
void App_Run(void);

/* P60 centralized flight-actuator authorization.
 * Returns 1 only while the flight gate is active and every hard inhibit,
 * including the latched remote E-STOP, is clear. Ground-vent logic does not
 * use this function because it is intentionally a preflight-only path. */
uint8_t App_IsActuatorAuthorized(void);
uint8_t App_IsStopLatched(void);

extern volatile uint32_t v87_sd_update_count;
extern volatile uint32_t v87_sd_update_last_us;
extern volatile uint32_t v87_sd_update_max_us;
extern volatile uint8_t v87_sd_auto_stop_done;
extern volatile uint32_t v87_sd_auto_stop_duration_ms;


/* V8.11 SD finalize maintenance-window diagnostics. */
extern volatile uint8_t v811_sd_finalize_started;
extern volatile uint8_t v811_sd_finalize_done;
extern volatile uint32_t v811_sd_finalize_duration_ms;
extern volatile uint32_t v811_scheduler_rebase_count;

extern volatile uint32_t v811_nrf_deadline_before_finalize;
extern volatile uint32_t v811_nrf_deadline_after_finalize;
extern volatile uint32_t v811_lidar_deadline_before_finalize;
extern volatile uint32_t v811_lidar_deadline_after_finalize;

#endif
