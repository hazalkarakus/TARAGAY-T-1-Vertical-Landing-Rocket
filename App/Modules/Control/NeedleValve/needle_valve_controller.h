#ifndef NEEDLE_VALVE_CONTROLLER_H
#define NEEDLE_VALVE_CONTROLLER_H

#include <stdint.h>

#define NEEDLE_VALVE_SAFE_ZERO_MIN_ADC 950U

typedef enum
{
    NEEDLE_VALVE_FAULT_NONE = 0,
    NEEDLE_VALVE_FAULT_ZERO_UNSAFE = 1,
    NEEDLE_VALVE_FAULT_STALL = 2,
    NEEDLE_VALVE_FAULT_TARGET_RANGE = 3,
    NEEDLE_VALVE_FAULT_ADC_INVALID = 4,
    NEEDLE_VALVE_FAULT_HOME_TIMEOUT = 5,
    NEEDLE_VALVE_FAULT_HOME_DIRECTION = 6
} NeedleValveFault_t;

typedef struct
{
    float requested_cmd;
    float limited_cmd;

    uint16_t raw_adc;
    uint16_t zero_adc;
    uint16_t target_adc;
    int16_t error_adc;

    uint8_t rpwm;
    uint8_t lpwm;

    uint8_t enabled;
    uint8_t zero_valid;
    uint8_t position_locked;
    uint8_t homing_active;
    uint8_t homing_complete;
    NeedleValveFault_t fault;

    uint32_t stall_ms;
    uint32_t homing_elapsed_ms;
    uint32_t control_tick_count;
    uint32_t command_update_count;
    uint32_t target_reached_count;
} NeedleValveStatus_t;

void NeedleValveController_Init(void);

/* Call from TIM7 IRQ exactly at 1000 Hz. */
void NeedleValveController_ControlTickISR(void);

uint8_t NeedleValveController_SetCommand(float command_0_to_1);
void NeedleValveController_Stop(void);
uint8_t NeedleValveController_Enable(void);
uint8_t NeedleValveController_CaptureZero(void);
uint8_t NeedleValveController_StartAutoHome(void);
void NeedleValveController_ClearFault(void);

NeedleValveStatus_t NeedleValveController_GetStatus(void);

/* Live Expressions / telemetry exports. */
extern volatile uint32_t needle_valve_v8_magic;
extern volatile float needle_valve_requested_cmd;
extern volatile float needle_valve_limited_cmd;
extern volatile float needle_valve_position_turns;
extern volatile float needle_valve_max_turns;
extern volatile uint16_t needle_valve_raw_adc;
extern volatile uint16_t needle_valve_zero_adc;
extern volatile uint16_t needle_valve_target_adc;
extern volatile uint16_t needle_valve_max_open_adc;
extern volatile uint16_t needle_valve_adc_per_turn;
extern volatile uint16_t needle_valve_max_travel_adc;
extern volatile int16_t needle_valve_error_adc;
extern volatile uint8_t needle_valve_rpwm;
extern volatile uint8_t needle_valve_lpwm;
extern volatile uint8_t needle_valve_enabled;
extern volatile uint8_t needle_valve_zero_valid;
extern volatile uint8_t needle_valve_lock;
extern volatile uint8_t needle_valve_homing_active;
extern volatile uint8_t needle_valve_homing_complete;
extern volatile uint8_t needle_valve_fault;
extern volatile uint32_t needle_valve_stall_ms;
extern volatile uint32_t needle_valve_homing_elapsed_ms;
extern volatile uint16_t needle_valve_homing_start_adc;
extern volatile uint32_t needle_valve_control_tick_count;
extern volatile uint32_t needle_valve_adc_invalid_count;

#endif
