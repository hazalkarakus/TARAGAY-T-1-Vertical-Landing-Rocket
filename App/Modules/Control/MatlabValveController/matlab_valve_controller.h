#ifndef APP_MODULES_CONTROL_MATLAB_VALVE_CONTROLLER_H
#define APP_MODULES_CONTROL_MATLAB_VALVE_CONTROLLER_H

#include <stdint.h>

typedef struct
{
    float z_m;
    float v_mps;
    float mass_kg;
    float main_pressure_bar;
    float valve_cmd;
    uint8_t input_valid;
    uint8_t algorithm_implemented;
    uint8_t output_valid;
} MatlabValveControllerStatus_t;

void MatlabValveController_Init(void);

/* Exact flight-facing MATLAB Function interface wrapper:
 * Valve_Cmd = fcn(z, v, m_guncel, P_main_bar)
 * v > 0 upward, v < 0 downward.
 */
float MatlabValveController_fcn(float z, float v, float m_guncel, float P_main_bar);

MatlabValveControllerStatus_t MatlabValveController_GetStatus(void);

#endif
