#ifndef APP_MODULES_CONTROL_CONTROL_INPUT_PROVIDER_H
#define APP_MODULES_CONTROL_CONTROL_INPUT_PROVIDER_H
#include <stdint.h>

typedef enum { CONTROL_INPUT_SOURCE_INVALID=0, CONTROL_INPUT_SOURCE_ESKF=1,
               CONTROL_INPUT_SOURCE_MODEL=2, CONTROL_INPUT_SOURCE_SENSOR=3 } ControlInputSource_t;

typedef struct
{
    float z_m, v_mps, mass_kg, main_pressure_bar;
    uint8_t z_valid, v_valid, mass_valid, pressure_valid, all_valid;
    uint8_t z_source, v_source, mass_source, pressure_source;
} ControlInputProviderData_t;

void ControlInputProvider_Init(void);
ControlInputProviderData_t ControlInputProvider_Update(void);
ControlInputProviderData_t ControlInputProvider_GetData(void);
#endif
