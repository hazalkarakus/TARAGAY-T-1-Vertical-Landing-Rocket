#include <stdio.h>
#include <string.h>

#include "Modules/Control/VerticalSensorPolicy/vertical_sensor_policy.h"

static int failures;

static void Expect(int condition, const char *name)
{
    if (!condition)
    {
        printf("FAIL: %s\n", name);
        failures++;
    }
}

static FullStateESKFData_t HealthyBase(void)
{
    FullStateESKFData_t eskf;

    memset(&eskf, 0, sizeof(eskf));
    eskf.enabled = 1U;
    eskf.initialized = 1U;
    eskf.healthy = 1U;
    eskf.origin_zeroed = 1U;
    eskf.vertical_position_valid = 1U;
    eskf.covariance_integrity_ok = 1U;
    eskf.output_inhibited = 0U;
    eskf.lidar_reference_ready = 1U;
    /* Historical count is intentionally non-zero in P49; recovered errors
     * must not permanently disable vertical aiding. */
    eskf.numerical_error_count = 52UL;

    return eskf;
}

int main(void)
{
    FullStateESKFData_t eskf = HealthyBase();
    VerticalSensorPolicyResult_t policy;

    policy = VerticalSensorPolicy_Evaluate(0);
    Expect(policy.usable == 0U, "null ESKF is invalid");

    eskf.lidar_fresh = 1U;
    policy = VerticalSensorPolicy_Evaluate(&eskf);
    Expect(policy.usable == 1U, "LIDAR-only remains usable");
    Expect(policy.source_mask == VERTICAL_SENSOR_SOURCE_LIDAR,
           "LIDAR-only mask");
    Expect(policy.degraded == 1U, "LIDAR-only is degraded redundancy");

    eskf.lidar_fresh = 0U;
    eskf.baro_reference_ready = 1U;
    eskf.baro_fresh = 1U;
    policy = VerticalSensorPolicy_Evaluate(&eskf);
    Expect(policy.usable == 1U,
           "barometer continues after thrust-induced LIDAR dropout");
    Expect(policy.source_mask == VERTICAL_SENSOR_SOURCE_BARO,
           "barometer-only mask");
    Expect(policy.degraded == 1U, "barometer-only is degraded redundancy");

    eskf.lidar_fresh = 1U;
    policy = VerticalSensorPolicy_Evaluate(&eskf);
    Expect(policy.usable == 1U, "dual source usable");
    Expect(policy.source_mask == VERTICAL_SENSOR_SOURCE_BOTH,
           "dual source mask");
    Expect(policy.degraded == 0U, "dual source is not degraded");

    eskf.lidar_fresh = 0U;
    eskf.baro_fresh = 0U;
    policy = VerticalSensorPolicy_Evaluate(&eskf);
    Expect(policy.usable == 0U, "no aiding source is invalid");

    eskf.baro_fresh = 1U;
    eskf.lidar_reference_ready = 0U;
    policy = VerticalSensorPolicy_Evaluate(&eskf);
    Expect(policy.usable == 0U,
           "AGL control requires preflight LIDAR datum");

    eskf.lidar_reference_ready = 1U;

    eskf.numerical_error_count = 999UL;
    eskf.baro_fresh = 1U;
    policy = VerticalSensorPolicy_Evaluate(&eskf);
    Expect(policy.usable == 1U,
           "historical numerical errors do not permanently inhibit recovered ESKF");

    eskf.covariance_integrity_ok = 0U;
    policy = VerticalSensorPolicy_Evaluate(&eskf);
    Expect(policy.usable == 0U, "current covariance fault is invalid");
    eskf.covariance_integrity_ok = 1U;

    eskf.output_inhibited = 1U;
    policy = VerticalSensorPolicy_Evaluate(&eskf);
    Expect(policy.usable == 0U, "current output inhibit is invalid");
    eskf.output_inhibited = 0U;

    eskf.healthy = 0U;
    policy = VerticalSensorPolicy_Evaluate(&eskf);
    Expect(policy.usable == 0U, "unhealthy ESKF is invalid");

    if (failures != 0)
    {
        printf("V50 vertical sensor policy tests: %d failure(s)\n", failures);
        return 1;
    }

    printf("V50 vertical sensor policy host tests: PASS\n");
    return 0;
}
