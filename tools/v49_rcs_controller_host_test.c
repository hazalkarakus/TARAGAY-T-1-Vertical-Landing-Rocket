#include "Modules/Control/AttitudeControl/attitude_control.h"
#include "Services/SolenoidOutput/solenoid_output.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static SolenoidOutputStatus_t test_output;
static uint8_t test_fast_fault = 0U;
static uint8_t test_system_fault = 0U;

uint8_t SystemMonitor_IsFastFaultActive(void)
{
    return test_fast_fault;
}

uint8_t SystemMonitor_IsActuatorFaultActive(void)
{
    return ((test_fast_fault != 0U) || (test_system_fault != 0U)) ? 1U : 0U;
}

void SolenoidOutput_Init(void)
{
    test_output.requested_mask = 0U;
    test_output.applied_mask = 0U;
    test_output.requested_open = 0U;
    test_output.applied_open = 0U;
    test_output.dry_run = 0U;
    test_output.interlock_fault = 0U;
    test_output.update_count = 0UL;
}

void SolenoidOutput_SetMask(uint8_t mask)
{
    uint8_t roll_pair = (uint8_t)(SOLENOID_VALVE_ROLL_POS_ERROR |
                                  SOLENOID_VALVE_ROLL_NEG_ERROR);
    uint8_t pitch_pair = (uint8_t)(SOLENOID_VALVE_PITCH_POS_ERROR |
                                   SOLENOID_VALVE_PITCH_NEG_ERROR);

    test_output.interlock_fault =
        ((((mask & roll_pair) == roll_pair) ||
          ((mask & pitch_pair) == pitch_pair))) ? 1U : 0U;
    test_output.requested_mask = mask;
    test_output.applied_mask = (test_output.interlock_fault == 0U) ? mask : 0U;
    test_output.requested_open = (mask != 0U) ? 1U : 0U;
    test_output.applied_open =
        (test_output.applied_mask != 0U) ? 1U : 0U;
    test_output.update_count++;
}

void SolenoidOutput_SetDemand(uint8_t open_demand)
{
    SolenoidOutput_SetMask(
        (open_demand != 0U) ? SOLENOID_VALVE_ROLL_POS_ERROR : 0U
    );
}

void SolenoidOutput_ForceSafe(void)
{
    SolenoidOutput_SetMask(0U);
}

SolenoidOutputStatus_t SolenoidOutput_GetStatus(void)
{
    return test_output;
}

static FullStateESKFData_t HealthyESKF(void)
{
    FullStateESKFData_t eskf = {0};
    eskf.enabled = 1U;
    eskf.initialized = 1U;
    eskf.healthy = 1U;
    eskf.origin_zeroed = 1U;
    return eskf;
}

static SensorData_t HealthySensor(void)
{
    SensorData_t sensor = {0};
    sensor.imu_valid = 1U;
    return sensor;
}

static void Tick(uint32_t count)
{
    uint32_t i;
    for (i = 0UL; i < count; i++)
    {
        AttitudeControl_TimerTickISR();
    }
}

int main(void)
{
    FullStateESKFData_t eskf = HealthyESKF();
    SensorData_t sensor = HealthySensor();
    AttitudeControlStatus_t status;

    SolenoidOutput_Init();
    AttitudeControl_Init(0UL);

    /* Slide example: 2.50*3 + 3.55*10 = 43 ms. */
    eskf.roll_deg = 3.0f;
    sensor.gyro_x_filtered_dps = 10.0f;
    AttitudeControl_UpdateESKF(&eskf, &sensor, 3000UL);
    status = AttitudeControl_GetStatus();
    assert(status.roll_commanded_pulse_ms == 43UL);
    assert(status.roll_pd_time_signed_ms > 42.99f);
    assert(status.roll_pd_time_signed_ms < 43.01f);
    assert(status.valve_applied_mask == SOLENOID_VALVE_ROLL_POS_ERROR);

    Tick(42UL);
    assert(SolenoidOutput_GetStatus().applied_mask ==
           SOLENOID_VALVE_ROLL_POS_ERROR);
    Tick(1UL);
    status = AttitudeControl_GetStatus();
    assert(SolenoidOutput_GetStatus().applied_mask == 0U);
    assert(status.roll_cooldown_remaining_ms == 100UL);

    /* A command during cooldown is ignored and cannot chatter the valve. */
    AttitudeControl_UpdateESKF(&eskf, &sensor, 3045UL);
    status = AttitudeControl_GetStatus();
    assert(status.valve_applied_mask == 0U);
    assert(status.roll_cooldown_skip_count > 0UL);

    Tick(100UL);

    /* Derivative term changes sign before zero: opposite valve active brake. */
    eskf.roll_deg = 5.0f;
    sensor.gyro_x_filtered_dps = -10.0f;
    AttitudeControl_UpdateESKF(&eskf, &sensor, 3150UL);
    status = AttitudeControl_GetStatus();
    assert(status.roll_commanded_pulse_ms == 23UL);
    assert(status.roll_pd_time_signed_ms < -22.99f);
    assert(status.roll_auto_damping == 1U);
    assert(status.valve_applied_mask == SOLENOID_VALVE_ROLL_NEG_ERROR);

    Tick(23UL);
    Tick(100UL);

    /* Upper mechanical shield. */
    eskf.roll_deg = 30.0f;
    sensor.gyro_x_filtered_dps = 0.0f;
    AttitudeControl_UpdateESKF(&eskf, &sensor, 3300UL);
    status = AttitudeControl_GetStatus();
    assert(status.roll_commanded_pulse_ms == 60UL);
    assert(status.roll_saturation_count > 0UL);

    Tick(60UL);
    Tick(100UL);

    /* Lower mechanical shield: 2.5 ms request becomes exactly zero output. */
    eskf.roll_deg = 1.0f;
    sensor.gyro_x_filtered_dps = 0.0f;
    AttitudeControl_UpdateESKF(&eskf, &sensor, 3500UL);
    status = AttitudeControl_GetStatus();
    assert(status.roll_commanded_pulse_ms == 0UL);
    assert(status.valve_applied_mask == 0U);

    /* STOP/preflight override clears a live pulse and both timer states. */
    eskf.pitch_deg = 4.0f;
    sensor.gyro_y_filtered_dps = 10.0f;
    AttitudeControl_UpdateESKF(&eskf, &sensor, 3510UL);
    assert(SolenoidOutput_GetStatus().applied_mask != 0U);
    AttitudeControl_ForceSafe();
    status = AttitudeControl_GetStatus();
    assert(status.valve_applied_mask == 0U);
    assert(status.roll_pulse_remaining_ms == 0UL);
    assert(status.pitch_pulse_remaining_ms == 0UL);


    /* P39: a fast system fault must abort an already-active physical pulse
     * on the very next 1 ms timer tick. */
    AttitudeControl_Init(0UL);
    SolenoidOutput_ForceSafe();
    eskf = HealthyESKF();
    sensor = HealthySensor();
    eskf.roll_deg = 8.0f;
    sensor.gyro_x_filtered_dps = 0.0f;
    test_fast_fault = 0U;
    AttitudeControl_UpdateESKF(&eskf, &sensor, 5000UL);
    status = AttitudeControl_GetStatus();
    assert(status.roll_pulse_remaining_ms > 0UL);
    assert(SolenoidOutput_GetStatus().applied_mask != 0U);

    test_fast_fault = 1U;
    Tick(1UL);
    status = AttitudeControl_GetStatus();
    assert(SolenoidOutput_GetStatus().applied_mask == 0U);
    assert(status.valve_applied_mask == 0U);
    assert(status.roll_pulse_remaining_ms == 0UL);
    assert(status.pitch_pulse_remaining_ms == 0UL);
    test_fast_fault = 0U;

    /* P40: the slower SystemMonitor fault must keep an active pulse inhibited
     * even after the live fast fault has already cleared. */
    AttitudeControl_Init(0UL);
    SolenoidOutput_ForceSafe();
    eskf = HealthyESKF();
    sensor = HealthySensor();
    eskf.pitch_deg = 8.0f;
    sensor.gyro_y_filtered_dps = 0.0f;
    test_system_fault = 0U;
    AttitudeControl_UpdateESKF(&eskf, &sensor, 6000UL);
    assert(SolenoidOutput_GetStatus().applied_mask != 0U);
    test_system_fault = 1U;
    Tick(1UL);
    status = AttitudeControl_GetStatus();
    assert(SolenoidOutput_GetStatus().applied_mask == 0U);
    assert(status.pitch_pulse_remaining_ms == 0UL);
    test_system_fault = 0U;

    puts("V49/P40 RCS signed-PD host tests: PASS");
    return 0;
}
