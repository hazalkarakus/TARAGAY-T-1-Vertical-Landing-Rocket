#include <stdio.h>
#include <stdint.h>
#include "Modules/Control/TaragayFlightLogic/taragay_flight_logic.h"

int main(void)
{
    uint8_t seen[5] = {0,0,0,0,0};
    uint32_t i;
    uint32_t req_seen = 0;
    float max_valve = 0.0f;
    uint8_t max_fault = 0;
    TaragayFlightLogicStatus_t s;

    TaragayFlightLogic_Init();
    for (i = 0; i < 2600; ++i) /* 13 s @ 200 Hz service */
    {
        TaragayFlightLogic_Service200Hz();
        s = TaragayFlightLogic_GetStatus();
        if (s.mission_state < 5U) seen[s.mission_state] = 1U;
        if (s.rcs_requested_mask != 0U) req_seen = 1U;
        if (s.valve_cmd > max_valve) max_valve = s.valve_cmd;
        if (s.rcs_fault > max_fault) max_fault = s.rcs_fault;
        if (s.rcs_applied_mask != 0U)
        {
            printf("FAIL physical applied mask=%u\n", s.rcs_applied_mask);
            return 2;
        }
    }

    s = TaragayFlightLogic_GetStatus();
    printf("states=%u%u%u%u%u final=%u step=%lu elapsed=%lu maxL=%.4f req=%lu fault=%u hover=%.3f\n",
           seen[0],seen[1],seen[2],seen[3],seen[4],s.mission_state,
           (unsigned long)s.step_count,(unsigned long)s.synthetic_elapsed_ms,
           max_valve,(unsigned long)req_seen,max_fault,s.hover_best_s);

    if (!(seen[0] && seen[1] && seen[2] && seen[3] && seen[4])) return 3;
    if (s.mission_state != 4U) return 4;
    if (max_valve <= 0.0f) return 5;
    if (req_seen == 0U) return 6;
    if (max_fault != 0U) return 7;
    if (s.synthetic_input_active != 1U || s.compute_only != 1U) return 8;

    puts("PASS R8R19 flight-logic dry-run host test");
    return 0;
}
