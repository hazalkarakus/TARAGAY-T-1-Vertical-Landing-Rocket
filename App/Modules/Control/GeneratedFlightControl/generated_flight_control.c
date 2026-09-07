#include "Modules/Control/GeneratedFlightControl/generated_flight_control.h"

#include "Common/app_config.h"
#include "Modules/Control/VerticalSensorPolicy/vertical_sensor_policy.h"
#include "Modules/Estimation/AttitudeEstimator/attitude_estimator.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"
#include "Generated/Ucus_Bilgisayari.h"

#include <math.h>

#define GENERATED_FC_DEG_TO_RAD (0.01745329251994329577f)
#define GENERATED_FC_GRAVITY_MPS2 (9.80665f)

GeneratedFlightControlStatus_t generated_fc_status;
static uint8_t generated_fc_previous_input_valid;
static uint8_t generated_fc_divider_200hz;
static uint8_t generated_fc_previous_vertical_source_mask;
static float generated_fc_vertical_integral_n;

static float GeneratedFlightControl_Clamp(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void GeneratedFlightControl_RecordVerticalPolicy(
    VerticalSensorPolicyResult_t policy)
{
    if (policy.source_mask != generated_fc_previous_vertical_source_mask)
    {
        generated_fc_status.vertical_sensor_transition_count++;
        generated_fc_previous_vertical_source_mask = policy.source_mask;
    }

    generated_fc_status.vertical_sensor_source_mask = policy.source_mask;
    generated_fc_status.vertical_sensor_degraded = policy.degraded;
    generated_fc_status.lidar_aiding_active = policy.lidar_usable;
    generated_fc_status.baro_aiding_active = policy.baro_usable;

    switch (policy.source_mask)
    {
        case VERTICAL_SENSOR_SOURCE_BOTH:
            generated_fc_status.vertical_dual_source_count++;
            break;

        case VERTICAL_SENSOR_SOURCE_LIDAR:
            generated_fc_status.vertical_lidar_only_count++;
            break;

        case VERTICAL_SENSOR_SOURCE_BARO:
            generated_fc_status.vertical_baro_only_count++;
            break;

        default:
            generated_fc_status.vertical_no_aiding_count++;
            break;
    }
}

static void GeneratedFlightControl_VerticalOuterLoop200Hz(
    const FullStateESKFData_t *eskf)
{
    VerticalSensorPolicyResult_t policy =
        VerticalSensorPolicy_Evaluate(eskf);
    float height_m;
    float descent_speed_mps;
    float speed_error_mps;
    float estimated_thrust_n;
    float force_error_n;
    float target_force_n;

    GeneratedFlightControl_RecordVerticalPolicy(policy);

    if (policy.usable == 0U)
    {
        generated_fc_status.vertical_200hz_valve_cmd = 0.0f;
        generated_fc_status.suicide_burn_height_m = 0.0f;
        generated_fc_status.vertical_target_force_n = 0.0f;
        generated_fc_status.vertical_estimated_thrust_n = 0.0f;
        generated_fc_status.suicide_burn_active = 0U;
        generated_fc_vertical_integral_n = 0.0f;
        generated_fc_status.vertical_pi_integral_n = 0.0f;
        return;
    }

#if (APP_P112R10R3_GNC_MOTOR_BENCH_MODE != 0U)
    /* P112R10R3: deterministic inert motor-bench stimulus. Only the controller
     * input is virtualized; the production 200 Hz vertical-control math below
     * remains unchanged. RCS/vent outputs stay hard-locked OFF while the main
     * needle motor follows the production actuator interlocks. */
    height_m = APP_P112R10R3_VIRTUAL_HEIGHT_M;
    descent_speed_mps = APP_P112R10R3_VIRTUAL_DESCENT_MPS;
#else
    height_m = eskf->lidar_reference_m + eskf->position_z_m;
    if (height_m < 0.0f) height_m = 0.0f;
    descent_speed_mps = (eskf->velocity_z_mps < 0.0f) ?
        -eskf->velocity_z_mps : 0.0f;
#endif

    generated_fc_status.suicide_burn_height_m =
        (descent_speed_mps * descent_speed_mps) /
        (2.0f * APP_VERTICAL_200HZ_MAX_NET_DECEL_MPS2);

    estimated_thrust_n = APP_GENERATED_FC_MODEL_MASS_KG *
        (eskf->world_linear_accel_z_mps2 + GENERATED_FC_GRAVITY_MPS2);
    estimated_thrust_n = GeneratedFlightControl_Clamp(
        estimated_thrust_n, 0.0f, APP_GENERATED_FC_THRUST_ESTIMATE_MAX_N);

    generated_fc_status.vertical_estimated_thrust_n = estimated_thrust_n;
    generated_fc_status.vertical_feedforward_n =
        APP_VERTICAL_200HZ_HOVER_FEEDFORWARD_N;

    if ((descent_speed_mps > APP_VERTICAL_200HZ_TARGET_DESCENT_MPS) &&
        (height_m <= (generated_fc_status.suicide_burn_height_m +
                      APP_VERTICAL_200HZ_BURN_MARGIN_M)))
    {
        generated_fc_status.suicide_burn_active = 1U;
        speed_error_mps = descent_speed_mps -
            APP_VERTICAL_200HZ_TARGET_DESCENT_MPS;

        /* Sensorless pressure-loss support.  If measured acceleration implies
         * less thrust than the previous target, integral feed rises. */
        force_error_n = generated_fc_status.vertical_target_force_n -
                        estimated_thrust_n;
        generated_fc_vertical_integral_n +=
            APP_VERTICAL_200HZ_KI_N_PER_NS * force_error_n * 0.005f;
        generated_fc_vertical_integral_n = GeneratedFlightControl_Clamp(
            generated_fc_vertical_integral_n,
            0.0f,
            APP_VERTICAL_200HZ_INTEGRAL_LIMIT_N);

        target_force_n = APP_VERTICAL_200HZ_HOVER_FEEDFORWARD_N +
            APP_VERTICAL_200HZ_KP_N_PER_MPS * speed_error_mps +
            generated_fc_vertical_integral_n;
        target_force_n = GeneratedFlightControl_Clamp(
            target_force_n, 0.0f, APP_VERTICAL_200HZ_MAX_THRUST_N);
    }
    else
    {
        generated_fc_status.suicide_burn_active = 0U;
        generated_fc_vertical_integral_n *= 0.995f;
        target_force_n = 0.0f;
    }

    generated_fc_status.vertical_target_force_n = target_force_n;
    generated_fc_status.vertical_pi_integral_n =
        generated_fc_vertical_integral_n;
    generated_fc_status.vertical_200hz_valve_cmd =
        target_force_n / APP_VERTICAL_200HZ_MAX_THRUST_N;
}

static void GeneratedFlightControl_ClearOutputs(void)
{
    generated_fc_status.rcs_v1 = 0.0f;
    generated_fc_status.rcs_v3 = 0.0f;
    generated_fc_status.rcs_v5 = 0.0f;
    generated_fc_status.rcs_v7 = 0.0f;
    generated_fc_status.main_valve_cmd = 0.0f;
    generated_fc_status.main_output_valid = 0U;
    generated_fc_status.rcs_output_valid = 0U;
}

static uint8_t GeneratedFlightControl_AllFinite(
    const GeneratedFlightControlInput_t *input)
{
    return
        ((isfinite(input->pitch_deg) != 0) &&
         (isfinite(input->yaw_deg) != 0) &&
         (isfinite(input->x_m) != 0) &&
         (isfinite(input->y_m) != 0) &&
         (isfinite(input->vx_mps) != 0) &&
         (isfinite(input->vy_mps) != 0) &&
         (isfinite(input->z_cg_m) != 0) &&
         (isfinite(input->vz_mps) != 0) &&
         (isfinite(input->z_imu_m) != 0) &&
         (isfinite(input->mass_kg) != 0) &&
         (isfinite(input->main_pressure_bar) != 0) &&
         (isfinite(input->measured_thrust_n) != 0) &&
         (isfinite(input->pitch_rate_rad_s) != 0) &&
         (isfinite(input->yaw_rate_rad_s) != 0)) ? 1U : 0U;
}

void GeneratedFlightControl_Init(void)
{
    uint8_t *bytes = (uint8_t *)&generated_fc_status;
    uint32_t i;

    for (i = 0UL; i < (uint32_t)sizeof(generated_fc_status); i++)
    {
        bytes[i] = 0U;
    }

    Ucus_Bilgisayari_initialize();
    generated_fc_status.compute_only =
        (APP_GENERATED_FC_PHYSICAL_OUTPUT_ENABLED == 0U) ? 1U : 0U;
    generated_fc_status.pressure_model_source = 1U;
    generated_fc_status.thrust_eskf_estimate_source = 1U;
    generated_fc_status.initialized = 1U;
    generated_fc_previous_input_valid = 0U;
    generated_fc_divider_200hz = 0U;
    generated_fc_previous_vertical_source_mask =
        VERTICAL_SENSOR_SOURCE_NONE;
    generated_fc_vertical_integral_n = 0.0f;
}

void GeneratedFlightControl_Step100Hz(const GeneratedFlightControlInput_t *input)
{
    uint8_t base_valid;

    if (input == 0)
    {
        generated_fc_status.invalid_input_count++;
        generated_fc_previous_input_valid = 0U;
        GeneratedFlightControl_ClearOutputs();
        return;
    }

    base_valid =
        ((GeneratedFlightControl_AllFinite(input) != 0U) &&
         (input->vertical_valid != 0U) &&
         (input->attitude_valid != 0U) &&
         (input->mass_kg > 0.0f) &&
         (input->main_pressure_bar >= 0.0f)) ? 1U : 0U;

    if (base_valid == 0U)
    {
        generated_fc_status.invalid_input_count++;
        generated_fc_previous_input_valid = 0U;
        GeneratedFlightControl_ClearOutputs();
        return;
    }

    if (generated_fc_previous_input_valid == 0U)
    {
        Ucus_Bilgisayari_initialize();
        generated_fc_status.model_reset_count++;
    }

    generated_fc_previous_input_valid = 1U;

    Ucus_Bilgisayari_U.Pitch = (double)input->pitch_deg;
    Ucus_Bilgisayari_U.Yaw = (double)input->yaw_deg;
    Ucus_Bilgisayari_U.yPOS = (double)input->y_m;
    Ucus_Bilgisayari_U.XPOS = (double)input->x_m;
    Ucus_Bilgisayari_U.Vy = (double)input->vy_mps;
    Ucus_Bilgisayari_U.vx = (double)input->vx_mps;
    Ucus_Bilgisayari_U.zpos = (double)input->z_cg_m;
    Ucus_Bilgisayari_U.zvel = (double)input->vz_mps;
    Ucus_Bilgisayari_U.ZposIMU = (double)input->z_imu_m;
    Ucus_Bilgisayari_U.m_guncel = (double)input->mass_kg;
    Ucus_Bilgisayari_U.P_main_bar = (double)input->main_pressure_bar;
    Ucus_Bilgisayari_U.Gercek_Itki_N = (double)input->measured_thrust_n;
    Ucus_Bilgisayari_U.pitch_rate = (double)input->pitch_rate_rad_s;
    Ucus_Bilgisayari_U.yaw_rate = (double)input->yaw_rate_rad_s;

    Ucus_Bilgisayari_output();
    Ucus_Bilgisayari_update();

    generated_fc_status.rcs_v1 = (float)Ucus_Bilgisayari_Y.V1;
    generated_fc_status.rcs_v3 = (float)Ucus_Bilgisayari_Y.V3;
    generated_fc_status.rcs_v5 = (float)Ucus_Bilgisayari_Y.V5;
    generated_fc_status.rcs_v7 = (float)Ucus_Bilgisayari_Y.V7;
    generated_fc_status.main_valve_cmd = (float)Ucus_Bilgisayari_Y.Anaitki;

    if ((isfinite(generated_fc_status.rcs_v1) == 0) ||
        (isfinite(generated_fc_status.rcs_v3) == 0) ||
        (isfinite(generated_fc_status.rcs_v5) == 0) ||
        (isfinite(generated_fc_status.rcs_v7) == 0) ||
        (isfinite(generated_fc_status.main_valve_cmd) == 0) ||
        (generated_fc_status.rcs_v1 < 0.0f) ||
        (generated_fc_status.rcs_v1 > 1.0f) ||
        (generated_fc_status.rcs_v3 < 0.0f) ||
        (generated_fc_status.rcs_v3 > 1.0f) ||
        (generated_fc_status.rcs_v5 < 0.0f) ||
        (generated_fc_status.rcs_v5 > 1.0f) ||
        (generated_fc_status.rcs_v7 < 0.0f) ||
        (generated_fc_status.rcs_v7 > 1.0f) ||
        (generated_fc_status.main_valve_cmd < 0.0f) ||
        (generated_fc_status.main_valve_cmd > 1.0f))
    {
        GeneratedFlightControl_ClearOutputs();
        generated_fc_previous_input_valid = 0U;
        return;
    }

    generated_fc_status.main_output_valid = 1U;
    generated_fc_status.rcs_output_valid =
        (input->horizontal_valid != 0U) ? 1U : 0U;
    generated_fc_status.step_count++;
}

void GeneratedFlightControl_HoldSafe(void)
{
    generated_fc_previous_input_valid = 0U;
    generated_fc_divider_200hz = 0U;
    generated_fc_vertical_integral_n = 0.0f;
    generated_fc_previous_vertical_source_mask =
        VERTICAL_SENSOR_SOURCE_NONE;
    generated_fc_status.vertical_200hz_valve_cmd = 0.0f;
    generated_fc_status.vertical_target_force_n = 0.0f;
    generated_fc_status.vertical_pi_integral_n = 0.0f;
    generated_fc_status.suicide_burn_active = 0U;
    generated_fc_status.vertical_sensor_source_mask =
        VERTICAL_SENSOR_SOURCE_NONE;
    generated_fc_status.vertical_sensor_degraded = 0U;
    generated_fc_status.lidar_aiding_active = 0U;
    generated_fc_status.baro_aiding_active = 0U;
    GeneratedFlightControl_ClearOutputs();
}

void GeneratedFlightControl_Service200Hz(void)
{
#if (APP_LEGACY_VERTICAL_LANDING_RETIRED != 0U)
    /* R8R21: legacy Simulink/vertical landing owner is retired.
     * Keep status/telemetry ABI, but it can never generate a live command. */
    GeneratedFlightControl_HoldSafe();
    return;
#endif
    const FullStateESKFData_t *eskf;
    const AttitudeEstimatorData_t *attitude;
    const SensorData_t *sensor;
    GeneratedFlightControlInput_t input;
    float height_agl_m;

#if (APP_GENERATED_FLIGHT_CONTROL_ENABLED == 0U)
    return;
#endif

    eskf = FullStateESKF_GetDataPtr();
    attitude = AttitudeEstimator_GetDataPtr();
    sensor = SensorManager_GetDataPtr();

    if ((eskf == 0) || (attitude == 0) || (sensor == 0))
    {
        generated_fc_status.invalid_input_count++;
        GeneratedFlightControl_HoldSafe();
        return;
    }

    GeneratedFlightControl_VerticalOuterLoop200Hz(eskf);

    generated_fc_divider_200hz++;
    if (generated_fc_divider_200hz < 2U)
    {
        return;
    }
    generated_fc_divider_200hz = 0U;

    height_agl_m = eskf->lidar_reference_m + eskf->position_z_m;
    if (height_agl_m < 0.0f)
    {
        height_agl_m = 0.0f;
    }

    input.pitch_deg = attitude->pitch_deg;
    input.yaw_deg = attitude->yaw_deg;
    input.x_m = eskf->position_x_m;
    input.y_m = eskf->position_y_m;
    input.vx_mps = eskf->velocity_x_mps;
    input.vy_mps = eskf->velocity_y_mps;
    input.z_cg_m = APP_GNC_CG_TOUCH_HEIGHT_M + height_agl_m;
    input.vz_mps = eskf->velocity_z_mps;
    input.z_imu_m = APP_GNC_CG_TOUCH_HEIGHT_M + eskf->position_z_m;
    input.mass_kg = APP_GENERATED_FC_MODEL_MASS_KG;
    input.main_pressure_bar = APP_GENERATED_FC_INITIAL_PRESSURE_BAR;
    input.measured_thrust_n = input.mass_kg *
        (eskf->world_linear_accel_z_mps2 + GENERATED_FC_GRAVITY_MPS2);
    if (input.measured_thrust_n < 0.0f)
    {
        input.measured_thrust_n = 0.0f;
    }
    else if (input.measured_thrust_n > APP_GENERATED_FC_THRUST_ESTIMATE_MAX_N)
    {
        input.measured_thrust_n = APP_GENERATED_FC_THRUST_ESTIMATE_MAX_N;
    }
    input.pitch_rate_rad_s =
        sensor->gyro_y_filtered_dps * GENERATED_FC_DEG_TO_RAD;
    input.yaw_rate_rad_s =
        sensor->gyro_z_filtered_dps * GENERATED_FC_DEG_TO_RAD;

    input.vertical_valid =
        (generated_fc_status.vertical_sensor_source_mask !=
         VERTICAL_SENSOR_SOURCE_NONE) ? 1U : 0U;

    input.horizontal_valid =
        (eskf->horizontal_position_valid != 0U) ? 1U : 0U;

    input.attitude_valid =
        ((attitude->enabled != 0U) &&
         (attitude->initialized != 0U) &&
         (attitude->healthy != 0U) &&
         (sensor->imu_valid != 0U)) ? 1U : 0U;

    GeneratedFlightControl_Step100Hz(&input);
}

GeneratedFlightControlStatus_t GeneratedFlightControl_GetStatus(void)
{
    return generated_fc_status;
}
