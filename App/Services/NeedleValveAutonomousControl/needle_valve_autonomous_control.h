#ifndef NEEDLE_VALVE_AUTONOMOUS_CONTROL_H
#define NEEDLE_VALVE_AUTONOMOUS_CONTROL_H

#include <stdint.h>
#include "Modules/Control/NeedleValve/needle_valve_controller.h"

typedef enum
{
    NEEDLE_AUTONOMOUS_WAIT_REFERENCE = 0,
    NEEDLE_AUTONOMOUS_READY = 1,
    NEEDLE_AUTONOMOUS_TRACKING = 2,
    NEEDLE_AUTONOMOUS_HOLD = 3,
    NEEDLE_AUTONOMOUS_REVOKED = 4,
    NEEDLE_AUTONOMOUS_FAULT = 5
} NeedleValveAutonomousState_t;

typedef enum
{
    NEEDLE_AUTONOMOUS_FAULT_NONE = 0,
    NEEDLE_AUTONOMOUS_FAULT_REFERENCE = 1,
    NEEDLE_AUTONOMOUS_FAULT_FEEDBACK = 2,
    NEEDLE_AUTONOMOUS_FAULT_COMMAND_TIMEOUT = 3,
    NEEDLE_AUTONOMOUS_FAULT_LOW_LEVEL = 4,
    NEEDLE_AUTONOMOUS_FAULT_TARGET_RANGE = 5
} NeedleValveAutonomousFault_t;

typedef struct
{
    NeedleValveAutonomousState_t state;
    NeedleValveAutonomousFault_t fault;
    float requested_command;
    uint16_t closed_reference_adc;
    uint16_t requested_target_adc;
    uint8_t reference_valid;
    uint8_t command_authorized;
    uint8_t command_fresh;
    uint8_t move_in_progress;
    uint32_t command_sequence;
    uint32_t last_command_ms;
    uint32_t moves_started;
    uint32_t moves_completed;
    uint32_t command_timeout_count;
    uint32_t command_reject_count;
} NeedleValveAutonomousStatus_t;

void NeedleValveAutonomousControl_Init(void);
void NeedleValveAutonomousControl_TimerTickISR(void);
void NeedleValveAutonomousControl_BackgroundUpdate(void);

/* Called only by the preflight state machine while PE9 is connected.  No
 * motor output is generated; the already-closed robust ADC position becomes
 * command 0.0. */
uint8_t NeedleValveAutonomousControl_IsClosedReferenceCandidate(void);
uint8_t NeedleValveAutonomousControl_CaptureClosedReference(void);

/* Production: the 200 Hz flight task is the sole command producer.
 * P112R11 has one compile-time PA0 single-shot commissioning producer only;
 * UART remains TX-only and has no actuation route. */
uint8_t NeedleValveAutonomousControl_SubmitCommand(float command_0_to_1);
void NeedleValveAutonomousControl_RevokeAuthorization(void);

/* R8R16 latched E-STOP emergency-close. This is the sole post-STOP motor action:
 * close once to the learned CLOSED reference, then de-energize. */
void NeedleValveAutonomousControl_EStopSafeCloseUpdate(void);
uint8_t NeedleValveAutonomousControl_IsEStopSafeCloseActive(void);
uint8_t NeedleValveAutonomousControl_IsEStopSafeCloseComplete(void);
uint8_t NeedleValveAutonomousControl_HasEStopSafeCloseFailed(void);
uint8_t NeedleValveAutonomousControl_GetEStopSafeCloseFailReason(void);
uint32_t NeedleValveAutonomousControl_GetEStopSafeCloseStartCount(void);
uint32_t NeedleValveAutonomousControl_GetEStopSafeCloseElapsedMs(void);
uint8_t NeedleValveAutonomousControl_WasEStopSafeCloseOverrideUsed(void);

/* P112R11R1 DO-NOT-FLY, PE9-connected, PA0 single-shot short-travel service. */
void NeedleValveAutonomousControl_CommissioningUpdate(void);

/* R11R1 read-only commissioning diagnostics appended to UART telemetry. */
uint8_t NeedleValveAutonomousControl_GetCommissioningState(void);
uint8_t NeedleValveAutonomousControl_GetCommissioningAbortReason(void);
uint32_t NeedleValveAutonomousControl_GetCommissioningAbortCount(void);
uint8_t NeedleValveAutonomousControl_GetCommissioningButtonDebounced(void);
uint16_t NeedleValveAutonomousControl_GetCommissioningOpenTargetAdc(void);

uint8_t NeedleValveAutonomousControl_IsReady(void);
uint8_t NeedleValveAutonomousControl_HasFault(void);
NeedleValveAutonomousStatus_t NeedleValveAutonomousControl_GetStatus(void);
NeedleValveStatus_t NeedleValveAutonomousControl_GetTelemetryStatus(void);

#endif
