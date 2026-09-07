#ifndef P112R5_HOST_NEEDLE_CONTROLLER_H
#define P112R5_HOST_NEEDLE_CONTROLLER_H

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

extern volatile uint8_t needle_valve_fault;

#endif
