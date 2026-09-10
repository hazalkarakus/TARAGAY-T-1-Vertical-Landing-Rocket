#ifndef APP_SERVICES_SOLENOID_OUTPUT_H
#define APP_SERVICES_SOLENOID_OUTPUT_H

#include <stdint.h>

/*
 * Valve names are the physical relay-harness channel (X/Y, matching the
 * IN1..IN4 connector labels in solenoid_output.c), NOT an attitude axis.
 * Earlier revisions named these ROLL_POS_ERROR/PITCH_POS_ERROR etc, which
 * silently disagreed with TaragayFlightLogic's own PITCH/YAW axis naming for
 * these same bits (its "pitch" axis drives the X channel here; its "yaw"
 * axis drives the Y channel) -- see the mapping note in
 * App/Modules/Control/TaragayFlightLogic/taragay_flight_logic.c near
 * rcs_step(). That three-way naming mismatch (ROLL/PITCH here vs X/Y in the
 * .c file vs PITCH/YAW in flight logic, all for the same 4 wires) is an easy
 * way to wire or reason about the wrong axis during hardware integration,
 * so the names below were changed to the neutral X/Y channel label instead
 * of guessing which attitude axis they actually correct. Whether the X
 * channel is physically mounted along the vehicle's real pitch or yaw axis
 * is a mechanical-layout fact this file cannot see -- confirm it against the
 * actual nozzle/harness build, not against a name.
 */
typedef enum
{
    SOLENOID_VALVE_NONE   = 0x00U,
    SOLENOID_VALVE_X_POS_ERROR = 0x01U, /* IN1 / PB15 / X+ */
    SOLENOID_VALVE_X_NEG_ERROR = 0x02U, /* IN2 / PE15 / X- */
    SOLENOID_VALVE_Y_POS_ERROR = 0x04U, /* IN3 / PE11 / Y+ */
    SOLENOID_VALVE_Y_NEG_ERROR = 0x08U  /* IN4 / PE7  / Y- */
} SolenoidValveMask_t;

typedef struct
{
    uint8_t requested_mask;
    uint8_t applied_mask;
    uint8_t requested_open; /* Legacy: any valve requested. */
    uint8_t applied_open;   /* Legacy: any valve physically applied. */
    uint8_t dry_run;
    uint8_t interlock_fault;
    uint8_t flight_authorized;
    uint8_t safety_inhibited;
    uint32_t update_count;
    uint32_t safety_inhibit_count;
    /* R8R35 read-only physical pulse timing diagnostics. */
    uint32_t pulse_complete_count;
    uint16_t last_pulse_ms;
    uint16_t max_pulse_ms;
} SolenoidOutputStatus_t;

void SolenoidOutput_Init(void);
void SolenoidOutput_SetMask(uint8_t valve_mask);

/* R8R25/R8R26/R8R27 dedicated pressureless relay-axis bench path. This API
 * may bypass the global non-needle INERT isolation only when the caller supplies
 * a live bench_authorized qualifier. Normal flight and legacy attitude paths
 * remain isolated; the dedicated ground-vent path below has its own interlocks. */
void SolenoidOutput_SetAxisBenchMask(uint8_t valve_mask, uint8_t bench_authorized);

/* Legacy single-channel API: maps to X_POS_ERROR / IN1 / PB15 / X+. */
void SolenoidOutput_SetDemand(uint8_t open_demand);

void SolenoidOutput_ForceSafe(void);
uint8_t SolenoidOutput_GetRequestedMask(void);
uint8_t SolenoidOutput_GetAppliedMask(void);

/* P71 dedicated manual vent path. It may bypass the normal flight-authorization
 * gate before PE9, or after PE9 only with the post-separation grounded qualifier.
 * Nonzero requests are restricted to exactly one opposing axis pair:
 * X+|X- (0x03) or Y+|Y- (0x0C). Normal RCS control still forbids opposing
 * valves. Returns 1 when the vent request was applied. */
uint8_t SolenoidOutput_SetGroundVentMask(
    uint8_t valve_mask,
    uint8_t post_separation_ground_authorized
);
SolenoidOutputStatus_t SolenoidOutput_GetStatus(void);
void SolenoidOutput_BenchTestInit(void);
void SolenoidOutput_BenchTestUpdate(void);
extern volatile uint8_t relay_bench_test_active;
extern volatile uint8_t relay_bench_test_channel;
extern volatile uint8_t relay_bench_test_last_mask;
extern volatile uint32_t relay_bench_test_start_count;
extern volatile uint32_t relay_bench_test_complete_count;
extern volatile uint32_t relay_bench_test_abort_count;

#endif
