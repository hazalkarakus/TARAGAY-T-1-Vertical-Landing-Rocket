#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#define IMU_WHO_AM_I_ISM330DLC      0x6A
#define IMU_WHO_AM_I_ALT            0x6B

/* IMU_GetRecoveryState() return values. */
#define IMU_RECOVERY_STATE_NORMAL       0U
#define IMU_RECOVERY_STATE_REQUESTED    1U
#define IMU_RECOVERY_STATE_RECOVERING   2U
#define IMU_RECOVERY_STATE_VALIDATING   3U
#define IMU_RECOVERY_STATE_FAILED       4U

typedef struct
{
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;

} IMU_RawData_t;

typedef struct
{
    uint8_t valid;
    uint8_t sign_mask;
    uint16_t repeated_magnitude_raw;
    uint32_t event_count;
    uint32_t timestamp_us;

    IMU_RawData_t burst[3];

    /* P54: register snapshot captured after the first repeated-word triplet
     * and bus re-synchronization, before the deferred retry. */
    uint8_t register_snapshot_valid;
    uint8_t whoami;
    uint8_t ctrl1_xl;
    uint8_t ctrl2_g;
    uint8_t ctrl3_c;
    uint8_t ctrl4_c;
} IMU_PatternDiagnostic_t;

/* P56/P57: last exact-six-axis stale episode. The register snapshot is
 * captured after a soft SPI resync and before any configuration write. P57
 * additionally exposes bounded STATUS_REG warmup counters via getters below. */
typedef struct
{
    uint8_t valid;
    uint32_t event_count;
    uint32_t timestamp_us;
    uint32_t stale_age_us;

    uint8_t register_snapshot_valid;
    uint8_t whoami;
    uint8_t ctrl1_xl;
    uint8_t ctrl2_g;
    uint8_t ctrl3_c;
    uint8_t ctrl4_c;
} IMU_StaleDiagnostic_t;

typedef struct
{
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

} IMU_Data_t;

void IMU_Init(void);

/* V42/V43: one-shot physical SPI1/IMU line diagnostic.
 * V43 also fingerprints control registers and tests direct SPI1 register access.
 * This only touches PA4/PA5/PA6/PA7 temporarily and restores SPI1 afterwards. */
void IMU_RunPhysicalLineDiagnostic(void);

uint8_t IMU_ReadWhoAmI(void);
uint8_t IMU_IsConnected(void);
uint8_t IMU_GetDeviceID(void);

uint8_t IMU_IsLastSampleValid(void);
uint32_t IMU_GetLastSampleTimestampUs(void);

/* Runtime watchdog/recovery diagnostics for SD logging and Live Expressions. */
uint32_t IMU_GetStaleCount(void);
uint32_t IMU_GetPatternErrorCount(void);
uint32_t IMU_GetPatternRetryCount(void);
uint32_t IMU_GetPatternRetrySuccessCount(void);
uint32_t IMU_GetPatternRecoveryEscalationCount(void);
uint32_t IMU_GetFastConfigCheckCount(void);
uint32_t IMU_GetFastConfigRepairAttemptCount(void);
uint32_t IMU_GetFastConfigRepairSuccessCount(void);
uint32_t IMU_GetFastConfigRepairFailureCount(void);
uint32_t IMU_GetFastConfigRepairLastDurationUs(void);
uint32_t IMU_GetFastConfigRepairMaxDurationUs(void);
uint8_t IMU_GetPatternDiagnostic(IMU_PatternDiagnostic_t *diag);
uint32_t IMU_GetStaleFastConfigCheckCount(void);
uint32_t IMU_GetStaleFastConfigRepairAttemptCount(void);
uint32_t IMU_GetStaleFastConfigRepairSuccessCount(void);
uint32_t IMU_GetStaleFastConfigRepairFailureCount(void);
uint32_t IMU_GetStaleRetrySuccessCount(void);
uint32_t IMU_GetStaleRecoveryEscalationCount(void);
uint32_t IMU_GetStaleFastConfigRepairLastDurationUs(void);
uint32_t IMU_GetStaleFastConfigRepairMaxDurationUs(void);
uint32_t IMU_GetStaleWarmupEventCount(void);
uint32_t IMU_GetStaleWarmupPollCount(void);
uint32_t IMU_GetStaleWarmupSuccessCount(void);
uint32_t IMU_GetStaleWarmupTimeoutCount(void);
uint32_t IMU_GetStaleWarmupFirstReadyUs(void);
uint32_t IMU_GetStaleWarmupMaxReadyUs(void);
uint8_t IMU_GetStaleWarmupLastStatus(void);
uint8_t IMU_GetStaleWarmupActive(void);
uint8_t IMU_GetStaleDiagnostic(IMU_StaleDiagnostic_t *diag);
uint32_t IMU_GetDmaTimeoutCount(void);
uint32_t IMU_GetRecoveryCount(void);
uint8_t IMU_GetRecoveryState(void);
uint8_t IMU_GetRecoveryStep(void);
uint32_t IMU_GetRecoveryAttemptCount(void);
uint32_t IMU_GetRecoveryFailureCount(void);
uint32_t IMU_GetRecoveryLastDurationUs(void);
uint32_t IMU_GetRecoveryMaxDurationUs(void);
uint32_t IMU_GetInvalidSampleCount(void);
uint32_t IMU_GetRedundantRejectCount(void);
uint32_t IMU_GetRegisterErrorCount(void);
uint8_t IMU_ShouldInhibitRCS(void);

uint8_t IMU_ReadRawSnapshot(
    IMU_RawData_t *raw,
    uint32_t *sample_timestamp_us,
    uint32_t *sample_generation
);
void IMU_ReadRaw(IMU_RawData_t *raw);
void IMU_ReadScaled(IMU_Data_t *data);

#endif
