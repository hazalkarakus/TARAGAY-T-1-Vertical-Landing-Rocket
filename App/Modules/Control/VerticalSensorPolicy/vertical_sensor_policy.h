#ifndef APP_MODULES_CONTROL_VERTICAL_SENSOR_POLICY_H
#define APP_MODULES_CONTROL_VERTICAL_SENSOR_POLICY_H

#include <stdint.h>

#include "Common/app_config.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"

/* Bit mask intentionally remains stable for Live Expressions and SD logs. */
#define VERTICAL_SENSOR_SOURCE_NONE   0U
#define VERTICAL_SENSOR_SOURCE_LIDAR  (1U << 0)
#define VERTICAL_SENSOR_SOURCE_BARO   (1U << 1)
#define VERTICAL_SENSOR_SOURCE_BOTH   \
    (VERTICAL_SENSOR_SOURCE_LIDAR | VERTICAL_SENSOR_SOURCE_BARO)

typedef struct
{
    uint8_t usable;
    uint8_t source_mask;
    uint8_t degraded;
    uint8_t lidar_usable;
    uint8_t baro_usable;
} VerticalSensorPolicyResult_t;

/*
 * This policy does not start or stop either sensor. The scheduler continuously
 * services LIDAR and barometer acquisition. It only decides whether the ESKF
 * vertical state is sufficiently aided for the main-motor outer loop.
 *
 * AGL height remains:
 *
 *     startup_lidar_reference + ESKF position_z
 *
 * During a thrust-induced LIDAR dropout, barometer corrections continue to
 * constrain ESKF position_z while the IMU provides high-rate propagation.
 */
static inline VerticalSensorPolicyResult_t
VerticalSensorPolicy_Evaluate(const FullStateESKFData_t *eskf)
{
    VerticalSensorPolicyResult_t result = {0U, 0U, 0U, 0U, 0U};
    uint8_t base_valid;

    if (eskf == 0)
    {
        return result;
    }

    /* P49: numerical_error_count is cumulative diagnostics, not a live
     * inhibit. P48 proved that using it here permanently killed vertical
     * aiding after a successfully recovered covariance rollback. Current
     * healthy/covariance/output flags are the authoritative live gate. */
    base_valid =
        ((APP_V50_DUAL_VERTICAL_FUSION_ENABLED != 0U) &&
         (eskf->enabled != 0U) &&
         (eskf->initialized != 0U) &&
         (eskf->healthy != 0U) &&
         (eskf->origin_zeroed != 0U) &&
         (eskf->vertical_position_valid != 0U) &&
         (eskf->covariance_integrity_ok != 0U) &&
         (eskf->output_inhibited == 0U)) ? 1U : 0U;

    if (base_valid == 0U)
    {
        return result;
    }

#if (APP_V50_REQUIRE_STARTUP_LIDAR_REFERENCE != 0U)
    if (eskf->lidar_reference_ready == 0U)
    {
        return result;
    }
#endif

#if (APP_V50_ALLOW_LIDAR_ONLY_VERTICAL != 0U)
    result.lidar_usable =
        ((eskf->lidar_reference_ready != 0U) &&
         (eskf->lidar_fresh != 0U)) ? 1U : 0U;
#endif

#if (APP_V50_ALLOW_BARO_ONLY_VERTICAL != 0U)
    result.baro_usable =
        ((eskf->baro_reference_ready != 0U) &&
         (eskf->baro_fresh != 0U)) ? 1U : 0U;
#endif

    if (result.lidar_usable != 0U)
    {
        result.source_mask |= VERTICAL_SENSOR_SOURCE_LIDAR;
    }

    if (result.baro_usable != 0U)
    {
        result.source_mask |= VERTICAL_SENSOR_SOURCE_BARO;
    }

    result.usable =
        (result.source_mask != VERTICAL_SENSOR_SOURCE_NONE) ? 1U : 0U;
    result.degraded =
        ((result.usable != 0U) &&
         (result.source_mask != VERTICAL_SENSOR_SOURCE_BOTH)) ? 1U : 0U;

    return result;
}

#endif
