#include <stdio.h>
#include <string.h>

#include "Modules/Control/GeneratedFlightControl/generated_flight_control.h"
#include "Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari.h"
#include "Modules/Estimation/AttitudeEstimator/attitude_estimator.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"

static FullStateESKFData_t test_eskf;
static AttitudeEstimatorData_t test_attitude;
static SensorData_t test_sensor;
static int failures;

ExtU_Ucus_Bilgisayari_T Ucus_Bilgisayari_U;
ExtY_Ucus_Bilgisayari_T Ucus_Bilgisayari_Y;

const FullStateESKFData_t *FullStateESKF_GetDataPtr(void)
{
    return &test_eskf;
}

const AttitudeEstimatorData_t *AttitudeEstimator_GetDataPtr(void)
{
    return &test_attitude;
}

const SensorData_t *SensorManager_GetDataPtr(void)
{
    return &test_sensor;
}

void Ucus_Bilgisayari_initialize(void)
{
    memset(&Ucus_Bilgisayari_U, 0, sizeof(Ucus_Bilgisayari_U));
    memset(&Ucus_Bilgisayari_Y, 0, sizeof(Ucus_Bilgisayari_Y));
}

void Ucus_Bilgisayari_output(void)
{
    Ucus_Bilgisayari_Y.V1 = 0.0;
    Ucus_Bilgisayari_Y.V3 = 0.0;
    Ucus_Bilgisayari_Y.V5 = 0.0;
    Ucus_Bilgisayari_Y.V7 = 0.0;
    Ucus_Bilgisayari_Y.Anaitki = 0.0;
}

void Ucus_Bilgisayari_update(void) {}
void Ucus_Bilgisayari_terminate(void) {}

static void Expect(int condition, const char *name)
{
    if (!condition)
    {
        printf("FAIL: %s\n", name);
        failures++;
    }
}

static void SetupHealthyBase(void)
{
    memset(&test_eskf, 0, sizeof(test_eskf));
    memset(&test_attitude, 0, sizeof(test_attitude));
    memset(&test_sensor, 0, sizeof(test_sensor));

    test_eskf.enabled = 1U;
    test_eskf.initialized = 1U;
    test_eskf.healthy = 1U;
    test_eskf.origin_zeroed = 1U;
    test_eskf.vertical_position_valid = 1U;
    test_eskf.covariance_integrity_ok = 1U;
    test_eskf.output_inhibited = 0U;
    test_eskf.numerical_error_count = 52UL;
    test_eskf.lidar_reference_ready = 1U;
    test_eskf.lidar_reference_m = 0.20f;
    test_eskf.position_z_m = 0.0f;
    test_eskf.velocity_z_mps = -3.0f;
    test_eskf.world_linear_accel_z_mps2 = 0.0f;

    test_attitude.enabled = 1U;
    test_attitude.initialized = 1U;
    test_attitude.healthy = 1U;
    test_sensor.imu_valid = 1U;
}

int main(void)
{
    SetupHealthyBase();
    GeneratedFlightControl_Init();

    /* Thrust-induced LIDAR dropout: barometer must keep control alive. */
    test_eskf.lidar_fresh = 0U;
    test_eskf.baro_reference_ready = 1U;
    test_eskf.baro_fresh = 1U;
    GeneratedFlightControl_Service200Hz();
    Expect(generated_fc_status.vertical_sensor_source_mask == 2U,
           "barometer-only source mask");
    Expect(generated_fc_status.vertical_sensor_degraded == 1U,
           "barometer-only degraded flag");
    Expect(generated_fc_status.vertical_200hz_valve_cmd > 0.0f,
           "LIDAR loss alone must not zero vertical command");

    /* LIDAR reacquisition restores normal dual-aided mode. */
    test_eskf.lidar_fresh = 1U;
    GeneratedFlightControl_Service200Hz();
    Expect(generated_fc_status.vertical_sensor_source_mask == 3U,
           "dual source after LIDAR reacquisition");
    Expect(generated_fc_status.vertical_sensor_degraded == 0U,
           "dual source normal flag");

    /* Barometer loss is also survivable while LIDAR is healthy. */
    test_eskf.baro_fresh = 0U;
    GeneratedFlightControl_Service200Hz();
    Expect(generated_fc_status.vertical_sensor_source_mask == 1U,
           "LIDAR-only source mask");
    Expect(generated_fc_status.vertical_200hz_valve_cmd > 0.0f,
           "barometer loss alone must not zero vertical command");

    /* Both aids unavailable is the explicit safe boundary. */
    test_eskf.lidar_fresh = 0U;
    test_eskf.vertical_position_valid = 0U;
    GeneratedFlightControl_Service200Hz();
    Expect(generated_fc_status.vertical_sensor_source_mask == 0U,
           "no-aiding source mask");
    Expect(generated_fc_status.vertical_200hz_valve_cmd == 0.0f,
           "both aids invalid forces vertical command safe");
    Expect(generated_fc_status.vertical_pi_integral_n == 0.0f,
           "both aids invalid clears PI memory");

    if (failures != 0)
    {
        printf("V50 generated vertical fallback tests: %d failure(s)\n",
               failures);
        return 1;
    }

    printf("V50 generated vertical fallback host tests: PASS\n");
    return 0;
}
