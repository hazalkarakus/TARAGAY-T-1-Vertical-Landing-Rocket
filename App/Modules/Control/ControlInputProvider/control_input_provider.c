#include "Modules/Control/ControlInputProvider/control_input_provider.h"
#include "Common/app_config.h"
#include "Modules/Control/VerticalSensorPolicy/vertical_sensor_policy.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"
#include <math.h>

static ControlInputProviderData_t g_data;

void ControlInputProvider_Init(void)
{
    uint8_t *p=(uint8_t*)&g_data; uint32_t i;
    for(i=0UL;i<(uint32_t)sizeof(g_data);i++) p[i]=0U;
}

ControlInputProviderData_t ControlInputProvider_Update(void)
{
    const FullStateESKFData_t *eskf=FullStateESKF_GetDataPtr();
    VerticalSensorPolicyResult_t vertical_policy=
        VerticalSensorPolicy_Evaluate(eskf);
    float height_agl_m=0.0f;

    g_data.z_valid=g_data.v_valid=g_data.mass_valid=g_data.pressure_valid=g_data.all_valid=0U;
    g_data.z_source=g_data.v_source=g_data.mass_source=g_data.pressure_source=CONTROL_INPUT_SOURCE_INVALID;

    if(eskf!=0)
    {
        if((vertical_policy.usable!=0U)&&
           (isfinite(eskf->lidar_reference_m)!=0)&&(isfinite(eskf->position_z_m)!=0))
        {
            height_agl_m=eskf->lidar_reference_m+eskf->position_z_m;
            if(height_agl_m<0.0f) height_agl_m=0.0f;
            g_data.z_m=APP_GNC_CG_TOUCH_HEIGHT_M+height_agl_m;
            g_data.z_valid=(isfinite(g_data.z_m)!=0)?1U:0U;
            if(g_data.z_valid) g_data.z_source=CONTROL_INPUT_SOURCE_ESKF;
        }

        if((vertical_policy.usable!=0U)&&
           (isfinite(eskf->velocity_z_mps)!=0))
        {
            g_data.v_mps=eskf->velocity_z_mps;
            g_data.v_valid=1U;
            g_data.v_source=CONTROL_INPUT_SOURCE_ESKF;
        }
    }

#if (APP_CONTROL_INPUT_MASS_SOURCE_MODEL != 0U)
    g_data.mass_kg=APP_CONTROL_MODEL_MASS_KG;
    g_data.mass_valid=((isfinite(g_data.mass_kg)!=0)&&(g_data.mass_kg>0.0f))?1U:0U;
    g_data.mass_source=CONTROL_INPUT_SOURCE_MODEL;
#endif

#if (APP_CONTROL_INPUT_PRESSURE_SOURCE_MODEL != 0U)
    g_data.main_pressure_bar=APP_CONTROL_MODEL_MAIN_PRESSURE_BAR;
    g_data.pressure_valid=((isfinite(g_data.main_pressure_bar)!=0)&&(g_data.main_pressure_bar>=0.0f))?1U:0U;
    g_data.pressure_source=CONTROL_INPUT_SOURCE_MODEL;
#endif

    g_data.all_valid=(g_data.z_valid&&g_data.v_valid&&g_data.mass_valid&&g_data.pressure_valid)?1U:0U;
    return g_data;
}

ControlInputProviderData_t ControlInputProvider_GetData(void){ return g_data; }
