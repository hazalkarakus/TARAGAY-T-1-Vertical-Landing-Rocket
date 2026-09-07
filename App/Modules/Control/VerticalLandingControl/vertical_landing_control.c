#include "vertical_landing_control.h"

#include "Common/app_config.h"

#include <math.h>

/*
 * V8.16 uses the already bench/HIL-developed V10.6 actuator-aware
 * early-brake outer controller.
 *
 * Sign convention:
 *   vertical_velocity_mps > 0 : upward
 *   vertical_velocity_mps < 0 : downward
 */

static float VCtrl_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

static void VCtrl_Clear(VerticalLandingControlOutput_t *out)
{
    uint8_t *bytes = (uint8_t *)out;
    uint32_t i;

    for (i = 0UL; i < (uint32_t)sizeof(*out); i++)
    {
        bytes[i] = 0U;
    }
}

void VerticalLandingControl_Init(void)
{
    /* Stateless outer loop. */
}

VerticalLandingControlOutput_t VerticalLandingControl_Update(
    float z_cg_m,
    float vertical_velocity_mps,
    float mass_kg,
    float main_pressure_bar
)
{
    const float g = 9.80665f;

    VerticalLandingControlOutput_t out;

    float mass;
    float pressure_bar;
    float pressure_ratio;
    float available_force_n;

    float h;
    float v_down;
    float v_reference;
    float v_error;

    float a_command;
    float a_up_available;

    float required_force_n;
    float valve_cmd;

    VCtrl_Clear(&out);

    if ((isfinite(z_cg_m) == 0) ||
        (isfinite(vertical_velocity_mps) == 0) ||
        (isfinite(mass_kg) == 0) ||
        (isfinite(main_pressure_bar) == 0))
    {
        return out;
    }

    mass = VCtrl_Clamp(mass_kg, 27.5f, 35.0f);

    if (main_pressure_bar > 1.0e4f)
    {
        pressure_bar = main_pressure_bar / 1.0e5f;
    }
    else
    {
        pressure_bar = main_pressure_bar;
    }

    pressure_bar = VCtrl_Clamp(
        pressure_bar,
        0.0f,
        APP_GNC_VERT_P_MAX_VALID_BAR
    );

    pressure_ratio =
        pressure_bar / APP_GNC_VERT_P_RATED_BAR;

    pressure_ratio = VCtrl_Clamp(pressure_ratio, 0.0f, 1.0f);

    available_force_n =
        APP_GNC_VERT_F_RATED_N * pressure_ratio;

    out.available_force_n = available_force_n;

    if (available_force_n < 1.0f)
    {
        return out;
    }

    h = z_cg_m - APP_GNC_CG_TOUCH_HEIGHT_M;

    if (h < 0.0f)
    {
        h = 0.0f;
    }

    v_down = -vertical_velocity_mps;

    if (v_down < 0.0f)
    {
        v_down = 0.0f;
    }

    out.height_above_touchdown_m = h;
    out.downward_speed_mps = v_down;

    if ((h <= APP_GNC_VERT_SHUTDOWN_HEIGHT_M) &&
        (v_down <= APP_GNC_VERT_SHUTDOWN_SPEED_MPS))
    {
        out.input_valid = 1U;
        out.valve_cmd = 0.0f;
        return out;
    }

    v_reference = sqrtf(
        (APP_GNC_VERT_V_TOUCH_MPS * APP_GNC_VERT_V_TOUCH_MPS) +
        (2.0f * APP_GNC_VERT_A_PROFILE_MPS2 * h)
    );

    v_reference = VCtrl_Clamp(
        v_reference,
        APP_GNC_VERT_V_TOUCH_MPS,
        APP_GNC_VERT_V_REFERENCE_MAX_MPS
    );

    v_error = v_down - v_reference;

    out.reference_downward_speed_mps = v_reference;
    out.speed_error_mps = v_error;

    /*
     * V10.6:
     * a_command = trajectory deceleration + vertical-speed feedback.
     */
    a_command =
        APP_GNC_VERT_A_PROFILE_MPS2 +
        (APP_GNC_VERT_K_V * v_error);

    a_up_available =
        (available_force_n / mass) - g;

    a_command = VCtrl_Clamp(
        a_command,
        -g,
        a_up_available
    );

    required_force_n =
        mass * (g + a_command);

    required_force_n = VCtrl_Clamp(
        required_force_n,
        0.0f,
        available_force_n
    );

    valve_cmd =
        required_force_n / available_force_n;

    valve_cmd = VCtrl_Clamp(
        valve_cmd,
        0.0f,
        APP_GNC_VERT_VALVE_CMD_MAX
    );

    out.commanded_accel_mps2 = a_command;
    out.required_force_n = required_force_n;
    out.valve_cmd = valve_cmd;
    out.input_valid = 1U;

    return out;
}
