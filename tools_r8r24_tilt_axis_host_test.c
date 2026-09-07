#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "Modules/Control/TaragayFlightLogic/taragay_flight_logic.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"

static FullStateESKFData_t g_eskf;
static SensorData_t g_sensor;
static uint32_t g_now_us = 1000000UL;
const FullStateESKFData_t *FullStateESKF_GetDataPtr(void){return &g_eskf;}
const SensorData_t *SensorManager_GetDataPtr(void){return &g_sensor;}
uint32_t micros(void){return g_now_us;}
uint32_t millis(void){return g_now_us/1000UL;}
uint8_t Timebase_HasElapsedMs(uint32_t n,uint32_t p,uint32_t i){return (uint8_t)((uint32_t)(n-p)>=i);}
uint8_t Timebase_HasElapsedUs(uint32_t n,uint32_t p,uint32_t i){return (uint8_t)((uint32_t)(n-p)>=i);}
void Timebase_Init(void){}

static void tick(float roll,float pitch,float gx,float gy,uint32_t ms)
{
    uint32_t i;
    for(i=0;i<ms*2U;i++){
        g_now_us += 5000UL;
        g_eskf.last_public_output_timestamp_us = g_now_us - 1000UL;
        g_sensor.imu_sample_timestamp_us = g_now_us - 500UL;
        g_eskf.roll_deg=roll; g_eskf.pitch_deg=pitch; g_eskf.yaw_deg=77.0f; /* heading must be ignored */
        g_sensor.gyro_x_filtered_dps=gx; g_sensor.gyro_y_filtered_dps=gy;
        TaragayFlightLogic_Service200Hz();
    }
}

int main(void)
{
    TaragayFlightLogicStatus_t s;
    memset(&g_eskf,0,sizeof(g_eskf)); memset(&g_sensor,0,sizeof(g_sensor));
    g_eskf.initialized=1U; g_eskf.healthy=1U; g_eskf.covariance_integrity_ok=1U;
    g_eskf.vertical_position_valid=1U; g_eskf.horizontal_position_valid=0U;
    g_eskf.origin_zeroed=1U; g_eskf.output_inhibited=0U; g_eskf.lidar_reference_m=0.2f;
    g_eskf.position_x_m=2.5f; g_eskf.position_y_m=-2.0f; /* deliberate invalid-hpos drift */
    g_eskf.velocity_x_mps=1.0f; g_eskf.velocity_y_mps=-1.0f;
    g_sensor.imu_valid=1U;
    TaragayFlightLogic_Init();

    tick(0,0,0,0,1000);
    s=TaragayFlightLogic_GetStatus();
    printf("base valid=%u gate=%u target=%.3f/%.3f events=%lu/%lu/%lu/%lu\n",s.input_valid,s.horizontal_target_gated,
           s.target_pitch_rad*57.2957795f,s.target_yaw_rad*57.2957795f,
           (unsigned long)s.rcs_v1_event_count,(unsigned long)s.rcs_v3_event_count,(unsigned long)s.rcs_v5_event_count,(unsigned long)s.rcs_v7_event_count);
    if(!s.input_valid || !s.horizontal_target_gated) return 2;
    if(fabsf(s.target_pitch_rad)>1e-6f || fabsf(s.target_yaw_rad)>1e-6f) return 3;

    tick(+12,0,0,0,1200); tick(0,0,0,0,500);
    tick(-12,0,0,0,1200); tick(0,0,0,0,500);
    tick(0,+12,0,0,1200); tick(0,0,0,0,500);
    tick(0,-12,0,0,1200); tick(0,0,0,0,500);
    s=TaragayFlightLogic_GetStatus();
    printf("final gate=%u target=%.3f/%.3f req/app=%u/%u events=%lu/%lu/%lu/%lu fault=%u\n",s.horizontal_target_gated,
           s.target_pitch_rad*57.2957795f,s.target_yaw_rad*57.2957795f,s.rcs_requested_mask,s.rcs_applied_mask,
           (unsigned long)s.rcs_v1_event_count,(unsigned long)s.rcs_v3_event_count,(unsigned long)s.rcs_v5_event_count,(unsigned long)s.rcs_v7_event_count,s.rcs_fault);
    if(s.rcs_applied_mask!=0U || s.rcs_fault!=0U) return 4;
    if((s.rcs_v1_event_count+s.rcs_v3_event_count+s.rcs_v5_event_count+s.rcs_v7_event_count)==0U) return 5;
    puts("PASS R8R24 roll/pitch tilt source + hpos gate + zero-ref RCS host test");
    return 0;
}
