#include "Modules/Control/MatlabValveController/matlab_valve_controller.h"
#include "Common/app_config.h"
#include <math.h>

static MatlabValveControllerStatus_t g_status;

void MatlabValveController_Init(void)
{
    g_status.z_m = 0.0f;
    g_status.v_mps = 0.0f;
    g_status.mass_kg = 0.0f;
    g_status.main_pressure_bar = 0.0f;
    g_status.valve_cmd = 0.0f;
    g_status.input_valid = 0U;
    g_status.algorithm_implemented = (APP_MATLAB_VALVE_CONTROLLER_IMPLEMENTED != 0U) ? 1U : 0U;
    g_status.output_valid = 0U;
}

float MatlabValveController_fcn(float z, float v, float m_guncel, float P_main_bar)
{
    float valve_cmd = 0.0f;

    g_status.z_m = z;
    g_status.v_mps = v;
    g_status.mass_kg = m_guncel;
    g_status.main_pressure_bar = P_main_bar;
    g_status.input_valid = ((isfinite(z) != 0) && (isfinite(v) != 0) &&
                            (isfinite(m_guncel) != 0) && (isfinite(P_main_bar) != 0)) ? 1U : 0U;
    g_status.algorithm_implemented = (APP_MATLAB_VALVE_CONTROLLER_IMPLEMENTED != 0U) ? 1U : 0U;

#if (APP_MATLAB_VALVE_CONTROLLER_IMPLEMENTED != 0U)
#error "Insert the exact reviewed MATLAB controller body here before enabling it."
#else
    (void)z; (void)v; (void)m_guncel; (void)P_main_bar;
    valve_cmd = 0.0f;
#endif

    if ((g_status.input_valid == 0U) || (isfinite(valve_cmd) == 0))
    {
        valve_cmd = 0.0f;
        g_status.output_valid = 0U;
    }
    else
    {
        if (valve_cmd < 0.0f) valve_cmd = 0.0f;
        else if (valve_cmd > 1.0f) valve_cmd = 1.0f;
        g_status.output_valid = (g_status.algorithm_implemented != 0U) ? 1U : 0U;
    }

    g_status.valve_cmd = valve_cmd;
    return valve_cmd;
}

MatlabValveControllerStatus_t MatlabValveController_GetStatus(void)
{
    return g_status;
}
