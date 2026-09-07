#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "Modules/Control/TaragayFlightLogic/taragay_flight_logic.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"

static FullStateESKFData_t g_eskf;
static SensorData_t g_sensor;
static uint32_t g_now_us = 1000000UL;

const FullStateESKFData_t *FullStateESKF_GetDataPtr(void) { return &g_eskf; }
const SensorData_t *SensorManager_GetDataPtr(void) { return &g_sensor; }
uint32_t micros(void) { return g_now_us; }
uint32_t millis(void) { return g_now_us/1000UL; }
uint8_t Timebase_HasElapsedMs(uint32_t n,uint32_t p,uint32_t i){return (uint8_t)((uint32_t)(n-p)>=i);}
uint8_t Timebase_HasElapsedUs(uint32_t n,uint32_t p,uint32_t i){return (uint8_t)((uint32_t)(n-p)>=i);}
void Timebase_Init(void) {}

int main(void)
{
    uint32_t i;
    uint8_t req_seen = 0U;
    uint8_t valid_seen = 0U;
    float max_valve = 0.0f;
    TaragayFlightLogicStatus_t s;

    memset(&g_eskf, 0, sizeof(g_eskf));
    memset(&g_sensor, 0, sizeof(g_sensor));
    g_eskf.initialized = 1U;
    g_eskf.healthy = 1U;
    g_eskf.covariance_integrity_ok = 1U;
    g_eskf.vertical_position_valid = 1U;
    g_eskf.horizontal_position_valid = 0U; /* current architecture */
    g_eskf.origin_zeroed = 1U;
    g_eskf.output_inhibited = 0U;
    g_eskf.lidar_reference_m = 0.0f;
    g_sensor.imu_valid = 1U;

    TaragayFlightLogic_Init();
    for (i = 0U; i < 2400U; ++i) /* 12 s at 200 Hz service */
    {
        float t = (float)i * 0.005f;
        g_now_us += 5000UL;
        g_eskf.last_public_output_timestamp_us = g_now_us - 1000UL;
        g_sensor.imu_sample_timestamp_us = g_now_us - 500UL;

        g_eskf.position_x_m = 0.10f;
        g_eskf.position_y_m = -0.10f;
        g_eskf.velocity_x_mps = 0.0f;
        g_eskf.velocity_y_mps = 0.0f;
        g_eskf.position_z_m = 0.0f;
        g_eskf.velocity_z_mps = 0.0f;
        if ((t >= 3.0f) && (t < 4.0f))
        {
            g_eskf.pitch_deg = 12.0f;
            g_sensor.gyro_x_filtered_dps = 0.0f;
        }
        else
        {
            g_eskf.pitch_deg = 0.0f;
            g_sensor.gyro_x_filtered_dps = 0.0f;
        }
        g_eskf.yaw_deg = 0.0f;
        g_sensor.gyro_y_filtered_dps = 0.0f;

        TaragayFlightLogic_Service200Hz();
        s = TaragayFlightLogic_GetStatus();
        if (s.input_valid) valid_seen = 1U;
        if (s.rcs_requested_mask) req_seen = 1U;
        if (s.valve_cmd > max_valve) max_valve = s.valve_cmd;
        if (s.rcs_applied_mask != 0U)
        {
            printf("FAIL applied mask=%u\n", s.rcs_applied_mask);
            return 2;
        }
    }

    s = TaragayFlightLogic_GetStatus();
    printf("real=%u synth=%u valid=%u reject=%u h/v=%u/%u origin=%u inhibit=%u step=%lu elapsed=%lu state=%u maxL=%.4f req=%u fault=%u age=%lu/%lu invalid=%lu\n",
           s.real_input_active,s.synthetic_input_active,s.input_valid,s.input_reject_reason,
           s.horizontal_position_valid,s.vertical_position_valid,s.eskf_origin_zeroed,s.eskf_output_inhibited,
           (unsigned long)s.step_count,(unsigned long)s.logic_elapsed_ms,s.mission_state,max_valve,req_seen,s.rcs_fault,
           (unsigned long)s.input_eskf_age_ms,(unsigned long)s.input_imu_age_ms,(unsigned long)s.invalid_input_count);

    if (s.real_input_active != 1U || s.synthetic_input_active != 0U) return 3;
    if (!valid_seen || s.input_valid != 1U || s.input_reject_reason != 0U) return 4;
    if (s.vertical_position_valid != 1U || s.horizontal_position_valid != 0U) return 5;
    if (s.step_count < 1000U || s.logic_elapsed_ms < 10000U) return 6;
    if (max_valve <= 0.0f) return 7;
    if (!req_seen || s.rcs_fault != 0U) return 8;
    if (s.rcs_applied_mask != 0U) return 9;
    if (s.invalid_input_count != 0U) return 10;

    /* Stale ESKF must be rejected and every compute-only command held safe. */
    g_now_us += 50000UL;
    g_sensor.imu_sample_timestamp_us = g_now_us - 500UL;
    /* Keep ESKF timestamp old on purpose. Two 200 Hz service calls => one 100 Hz step attempt. */
    TaragayFlightLogic_Service200Hz();
    TaragayFlightLogic_Service200Hz();
    s = TaragayFlightLogic_GetStatus();
    printf("stale_gate valid=%u reject=%u valve=%.4f req=%u invalid=%lu\n",
           s.input_valid,s.input_reject_reason,s.valve_cmd,s.rcs_requested_mask,
           (unsigned long)s.invalid_input_count);
    if (s.input_valid != 0U || s.input_reject_reason != 7U) return 11;
    if (s.valve_cmd != 0.0f || s.rcs_requested_mask != 0U || s.rcs_applied_mask != 0U) return 12;

    puts("PASS R8R22 real-ESKF compute-only + stale-source safety host test");
    return 0;
}
