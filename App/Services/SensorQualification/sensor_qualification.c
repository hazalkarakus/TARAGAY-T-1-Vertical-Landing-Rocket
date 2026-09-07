#include "Services/SensorQualification/sensor_qualification.h"

#include "Common/app_config.h"
#include "Modules/Sensors/IMU/imu.h"
#include "Modules/Sensors/Barometer/barometer.h"
#include "Modules/Sensors/Barometer/ms5611_spi.h"
#include "Modules/Sensors/Lidar/Lidar.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"
#include "Core/Scheduler/scheduler.h"
#include "Services/Timebase/timebase.h"

#include <math.h>
#include <limits.h>

#define SENSOR_QUAL_DIAG_MAGIC        0x819C19E1UL
#define SENSOR_QUAL_DIAG_VERSION      0x0008190EUL
#define SENSOR_QUAL_DIAG_WORDS        64U
#define SENSOR_QUAL_DIAG_BYTES        (SENSOR_QUAL_DIAG_WORDS * 4U)

#define BARO_DIAG_MAGIC               0xB58519E1UL
#define BARO_DIAG_VERSION             0x0008190EUL
#define BARO_DIAG_WORDS               64U
#define BARO_DIAG_BYTES               (BARO_DIAG_WORDS * 4U)

/* Existing detailed diagnostics exported by their implementation modules. */
extern volatile uint32_t imu_dma_error_count;
extern volatile uint32_t imu_redundant_disagreement_count;
extern volatile uint32_t imu_redundant_reject_count;
extern volatile uint32_t imu_bit8_correction_count;

extern volatile uint8_t sensor_imu_calibration_complete;
extern volatile uint32_t sensor_imu_calibration_sample_count;
extern volatile float sensor_imu_calibration_gyro_stddev_x_dps;
extern volatile float sensor_imu_calibration_gyro_stddev_y_dps;
extern volatile float sensor_imu_calibration_gyro_stddev_z_dps;
extern volatile float sensor_imu_calibration_accel_stddev_g;
extern volatile uint8_t sensor_imu_filter_config_ok;
extern volatile uint8_t sensor_imu_filter_initialized;
extern volatile uint32_t sensor_imu_valid_update_count;
extern volatile uint32_t sensor_imu_filter_max_gap_us;

extern volatile uint8_t bmp585_chip_id;
extern volatile uint8_t bmp585_config_ok;
extern volatile uint8_t bmp585_direct_read_mode_ok;
extern volatile uint32_t bmp585_fresh_sample_count;

volatile uint8_t sensor_qual_state = SENSOR_QUAL_STATE_WARMUP;
volatile uint8_t sensor_qual_pass = 0U;
volatile uint8_t sensor_qual_good_windows = 0U;
volatile uint32_t sensor_qual_flags = 0UL;
volatile float sensor_qual_imu_rate_hz = 0.0f;
volatile float sensor_qual_baro_rate_hz = 0.0f;
volatile float sensor_qual_lidar_rate_hz = 0.0f;

/* Raw fixed-address block. Live Expressions is not required to validate it. */
volatile uint32_t sensor_qual_diag[SENSOR_QUAL_DIAG_WORDS]
    __attribute__((section(".diag_sensor"), aligned(4), used));

/* V8.19E raw barometer diagnostic block at 0x1000F300. */
volatile uint32_t baro_diag[BARO_DIAG_WORDS]
    __attribute__((section(".diag_baro"), aligned(4), used));

static uint32_t window_start_us = 0UL;
static uint32_t window_count = 0UL;

static uint32_t last_imu_updates = 0UL;
static uint32_t last_baro_updates = 0UL;
static uint32_t last_lidar_updates = 0UL;

static uint32_t last_imu_dma_error = 0UL;
static uint32_t last_imu_dma_timeout = 0UL;
static uint32_t last_imu_stale = 0UL;
static uint32_t last_imu_pattern = 0UL;
static uint32_t last_imu_recovery = 0UL;
static uint32_t last_imu_register_error = 0UL;
static uint32_t last_imu_reject = 0UL;

static uint32_t last_baro_invalid = 0UL;
static uint32_t last_baro_comm_error = 0UL;

static uint32_t last_lidar_error = 0UL;
static uint32_t last_lidar_timeout = 0UL;
static uint32_t last_lidar_dma_error = 0UL;
static uint32_t last_lidar_pending_overrun = 0UL;

static uint32_t window_imu_error_delta = 0UL;
static uint32_t window_baro_error_delta = 0UL;
static uint32_t window_lidar_error_delta = 0UL;

static uint32_t diag_sequence = 0UL;
static uint32_t baro_diag_sequence = 0UL;

static uint32_t last_baro_task_runs = 0UL;
static uint32_t last_bmp585_service_calls = 0UL;
static uint32_t last_bmp585_direct_reads = 0UL;
static uint32_t last_bmp585_duplicates = 0UL;
static uint32_t last_bmp585_fresh_samples = 0UL;

static uint32_t baro_diag_task_rate_x10 = 0UL;
static uint32_t baro_diag_service_rate_x10 = 0UL;
static uint32_t baro_diag_direct_read_rate_x10 = 0UL;
static uint32_t baro_diag_duplicate_rate_x10 = 0UL;
static uint32_t baro_diag_fresh_rate_x10 = 0UL;

static uint8_t SensorQual_Finite(float value)
{
    return (isfinite(value) != 0) ? 1U : 0U;
}

static int32_t SensorQual_ScaleSigned(float value, float scale)
{
    float scaled;

    if (SensorQual_Finite(value) == 0U)
    {
        return INT32_MIN;
    }

    scaled = value * scale;

    if (scaled > 2147483000.0f)
    {
        return INT32_MAX;
    }

    if (scaled < -2147483000.0f)
    {
        return INT32_MIN + 1;
    }

    return (int32_t)scaled;
}

static uint32_t SensorQual_RateX10(float rate_hz)
{
    if ((SensorQual_Finite(rate_hz) == 0U) || (rate_hz < 0.0f))
    {
        return 0xFFFFFFFFUL;
    }

    if (rate_hz > 429496000.0f)
    {
        return 0xFFFFFFFFUL;
    }

    return (uint32_t)(rate_hz * 10.0f + 0.5f);
}

static uint32_t SensorQual_CountRateX10(uint32_t delta, float seconds)
{
    if ((SensorQual_Finite(seconds) == 0U) || (seconds <= 0.0f))
    {
        return 0xFFFFFFFFUL;
    }

    return SensorQual_RateX10((float)delta / seconds);
}

static uint32_t SensorQual_AbsDiffU32(uint32_t a, uint32_t b)
{
    return (a >= b) ? (a - b) : (b - a);
}

static uint8_t SensorQual_RateInside(float rate, float minimum, float maximum)
{
    return
        ((SensorQual_Finite(rate) != 0U) &&
         (rate >= minimum) &&
         (rate <= maximum)) ? 1U : 0U;
}

static uint32_t SensorQual_Checksum(void)
{
    uint32_t checksum = 0x18C1A55AUL;
    uint32_t i;

    /* word 60 = checksum, word 61 = inverse, 62..63 reserved. */
    for (i = 0UL; i < 60UL; i++)
    {
        checksum ^= sensor_qual_diag[i];
    }

    return checksum;
}

static uint32_t BaroDiag_Checksum(uint32_t published_sequence)
{
    uint32_t checksum = 0xB4A019B5UL;
    uint32_t i;

    for (i = 0UL; i < 60UL; i++)
    {
        /* word 3 is published last. Calculate its final even value while the
         * memory block still advertises an in-progress odd sequence. */
        checksum ^=
            (i == 3UL) ? published_sequence : baro_diag[i];
    }

    return checksum;
}

static void SensorQual_WriteDiagnostic(
    const SensorData_t *sensor,
    const BarometerData_t *baro,
    const MS5611_SPI_Data_t *baro_low,
    const LidarData_t *lidar,
    uint32_t now_us
)
{
    uint32_t seq = diag_sequence + 2UL;
    uint32_t checksum;
    uint32_t imu_age = 0xFFFFFFFFUL;
    uint32_t baro_age = 0xFFFFFFFFUL;
    uint32_t lidar_age = 0xFFFFFFFFUL;

    if ((seq & 1UL) != 0UL)
    {
        seq++;
    }
    diag_sequence = seq;

    if (IMU_GetLastSampleTimestampUs() != 0UL)
    {
        imu_age = now_us - IMU_GetLastSampleTimestampUs();
    }
    if (baro->last_sample_timestamp_us != 0UL)
    {
        baro_age = now_us - baro->last_sample_timestamp_us;
    }
    if (lidar->last_sample_timestamp_us != 0UL)
    {
        lidar_age = now_us - lidar->last_sample_timestamp_us;
    }

    sensor_qual_diag[0] = SENSOR_QUAL_DIAG_MAGIC;
    sensor_qual_diag[1] = SENSOR_QUAL_DIAG_VERSION;
    sensor_qual_diag[2] = SENSOR_QUAL_DIAG_BYTES;
    sensor_qual_diag[3] = seq | 1UL;

    sensor_qual_diag[4] = sensor_qual_flags;
    sensor_qual_diag[5] = sensor_qual_state;
    sensor_qual_diag[6] = sensor_qual_good_windows;
    sensor_qual_diag[7] = window_count;

    sensor_qual_diag[8] = SensorQual_RateX10(sensor_qual_imu_rate_hz);
    sensor_qual_diag[9] = SensorQual_RateX10(sensor_qual_baro_rate_hz);
    sensor_qual_diag[10] = SensorQual_RateX10(sensor_qual_lidar_rate_hz);
    sensor_qual_diag[11] = (uint32_t)IMU_GetDeviceID();
    sensor_qual_diag[12] = imu_age;

    sensor_qual_diag[13] = (uint32_t)SensorQual_ScaleSigned(sensor->accel_norm_g, 1000.0f);
    sensor_qual_diag[14] = (uint32_t)SensorQual_ScaleSigned(sensor->gyro_x_dps, 1000.0f);
    sensor_qual_diag[15] = (uint32_t)SensorQual_ScaleSigned(sensor->gyro_y_dps, 1000.0f);
    sensor_qual_diag[16] = (uint32_t)SensorQual_ScaleSigned(sensor->gyro_z_dps, 1000.0f);

    sensor_qual_diag[17] = sensor_imu_calibration_sample_count;
    sensor_qual_diag[18] = (uint32_t)SensorQual_ScaleSigned(sensor_imu_calibration_gyro_stddev_x_dps, 1000.0f);
    sensor_qual_diag[19] = (uint32_t)SensorQual_ScaleSigned(sensor_imu_calibration_gyro_stddev_y_dps, 1000.0f);
    sensor_qual_diag[20] = (uint32_t)SensorQual_ScaleSigned(sensor_imu_calibration_gyro_stddev_z_dps, 1000.0f);
    sensor_qual_diag[21] = (uint32_t)SensorQual_ScaleSigned(sensor_imu_calibration_accel_stddev_g, 1000.0f);

    sensor_qual_diag[22] = imu_dma_error_count;
    sensor_qual_diag[23] = IMU_GetDmaTimeoutCount();
    sensor_qual_diag[24] = IMU_GetStaleCount();
    sensor_qual_diag[25] = IMU_GetPatternErrorCount();
    sensor_qual_diag[26] = IMU_GetRecoveryCount();
    sensor_qual_diag[27] = IMU_GetRegisterErrorCount();
    sensor_qual_diag[28] = imu_redundant_disagreement_count;
    sensor_qual_diag[29] = imu_bit8_correction_count;

    sensor_qual_diag[30] = (uint32_t)SensorQual_ScaleSigned(baro->pressure_pa, 1.0f);
    sensor_qual_diag[31] = (uint32_t)SensorQual_ScaleSigned(baro->temperature_c, 1000.0f);
    sensor_qual_diag[32] = (uint32_t)SensorQual_ScaleSigned(baro->filtered_altitude_m, 1000.0f);
    sensor_qual_diag[33] = (uint32_t)SensorQual_ScaleSigned(baro->vertical_speed_mps, 1000.0f);
    sensor_qual_diag[34] = baro_age;
    sensor_qual_diag[35] = baro->update_count;
    sensor_qual_diag[36] = baro->invalid_sample_count;
    sensor_qual_diag[37] = baro_low->communication_error_count;
    sensor_qual_diag[38] = (uint32_t)bmp585_chip_id;
    sensor_qual_diag[39] = (uint32_t)bmp585_config_ok;
    sensor_qual_diag[40] = bmp585_fresh_sample_count;

    sensor_qual_diag[41] = (uint32_t)SensorQual_ScaleSigned(lidar->distance_m, 1000.0f);
    sensor_qual_diag[42] = (uint32_t)SensorQual_ScaleSigned(lidar->filtered_distance_m, 1000.0f);
    sensor_qual_diag[43] = lidar_age;
    sensor_qual_diag[44] = lidar->update_count;
    sensor_qual_diag[45] = lidar->error_count;
    sensor_qual_diag[46] = lidar->timeout_count;
    sensor_qual_diag[47] = lidar->dma_error_count;
    sensor_qual_diag[48] = (uint32_t)lidar_profile_config_ok;
    sensor_qual_diag[49] = lidar_sample_pending_overrun_count;

    sensor_qual_diag[50] = window_baro_error_delta;
    sensor_qual_diag[51] = window_lidar_error_delta;
    sensor_qual_diag[52] = window_imu_error_delta;
    sensor_qual_diag[53] = sensor_imu_filter_max_gap_us;
    sensor_qual_diag[54] = baro->last_sample_interval_us;
    sensor_qual_diag[55] = lidar->last_sample_interval_us;
    sensor_qual_diag[56] = baro->max_sample_interval_us;
    sensor_qual_diag[57] = lidar->max_sample_interval_us;
    sensor_qual_diag[58] = baro_low->update_count;

    sensor_qual_diag[59] = seq;

    /* Publish begin last: begin=end and even means coherent snapshot. */
    sensor_qual_diag[3] = seq;
    checksum = SensorQual_Checksum();
    sensor_qual_diag[60] = checksum;
    sensor_qual_diag[61] = ~checksum;
    sensor_qual_diag[62] = 0x53454E53UL; /* 'SENS' */
    sensor_qual_diag[63] = 0x56313945UL; /* 'V19E' */
}

static void SensorQual_WriteBarometerDiagnostic(
    const BarometerData_t *baro,
    const MS5611_SPI_Data_t *baro_low,
    const Task_t *baro_task,
    uint32_t now_us
)
{
    uint32_t seq = baro_diag_sequence + 2UL;
    uint32_t checksum;
    uint32_t flags = 1UL;
    uint32_t task_run_count = 0UL;
    uint32_t task_last_exec_us = 0UL;
    uint32_t task_max_exec_us = 0UL;
    uint32_t task_overrun_count = 0UL;
    uint32_t task_deadline_miss_count = 0UL;
    uint8_t standby_ok;

    if ((seq & 1UL) != 0UL)
    {
        seq++;
    }
    baro_diag_sequence = seq;

    standby_ok =
        (((bmp585_standby_config_reg & 0x83U) == 0x80U) ? 1U : 0U);

    if (baro_task != 0)
    {
        task_run_count = baro_task->run_count;
        task_last_exec_us = baro_task->last_exec_us;
        task_max_exec_us = baro_task->max_exec_us;
        task_overrun_count = baro_task->overrun_count;
        task_deadline_miss_count = baro_task->deadline_miss_count;
        flags |= (1UL << 1);
    }

    if (bmp585_direct_read_mode_ok != 0U)            flags |= (1UL << 24);
    if (baro_low->initialized != 0U)                 flags |= (1UL << 2);
    if (baro_low->connected != 0U)                   flags |= (1UL << 3);
    if (bmp585_config_ok != 0U)                      flags |= (1UL << 4);
    if (bmp585_direct_read_mode_ok != 0U)            flags |= (1UL << 5);
    if (bmp585_register_readback_ok != 0U)           flags |= (1UL << 6);

    if (SensorQual_AbsDiffU32(
            task_run_count,
            bmp585_service_call_count) <= 1UL)       flags |= (1UL << 7);
    if (SensorQual_AbsDiffU32(
            bmp585_service_call_count,
            bmp585_measurement_read_count) <= 1UL)   flags |= (1UL << 8);
    if (SensorQual_AbsDiffU32(
            bmp585_fresh_sample_count,
            baro_low->update_count) <= 1UL)          flags |= (1UL << 9);
    if (SensorQual_AbsDiffU32(
            baro_low->update_count,
            baro->source_update_count) <= 1UL)       flags |= (1UL << 10);

    if ((baro_diag_task_rate_x10 >= 1950UL) &&
        (baro_diag_task_rate_x10 <= 2050UL))         flags |= (1UL << 11);
    if ((baro_diag_direct_read_rate_x10 >= 1950UL) &&
        (baro_diag_direct_read_rate_x10 <= 2050UL))  flags |= (1UL << 12);
    if ((baro_diag_fresh_rate_x10 >= 1900UL) &&
        (baro_diag_fresh_rate_x10 <= 2050UL))        flags |= (1UL << 13);

    if (bmp585_int_status_read_error_count == 0UL)   flags |= (1UL << 14);
    if (bmp585_measurement_read_error_count == 0UL)  flags |= (1UL << 15);
    if (bmp585_register_snapshot_error_count == 0UL) flags |= (1UL << 16);
    if (bmp585_register_mismatch_count == 0UL)       flags |= (1UL << 17);
    if ((bmp585_status_reg & 0x04U) == 0U)           flags |= (1UL << 18);
    if (bmp585_odr_config_reg == 0x85U)              flags |= (1UL << 19);
    if (bmp585_osr_config_reg == 0x50U)              flags |= (1UL << 20);
    if (bmp585_osr_eff == 0x90U)                     flags |= (1UL << 21);
    if ((bmp585_int_source_reg & 0x01U) == 0U)       flags |= (1UL << 22);
    if (window_count > 0UL)                          flags |= (1UL << 23);

    baro_diag[0] = BARO_DIAG_MAGIC;
    baro_diag[1] = BARO_DIAG_VERSION;
    baro_diag[2] = BARO_DIAG_BYTES;
    baro_diag[3] = seq | 1UL;

    baro_diag[4] = flags;
    baro_diag[5] = task_run_count;
    baro_diag[6] = bmp585_service_call_count;
    baro_diag[7] = bmp585_measurement_read_count;
    baro_diag[8] = bmp585_measurement_read_error_count;
    baro_diag[9] = bmp585_duplicate_raw_count;
    baro_diag[10] = bmp585_fresh_sample_count;
    baro_diag[11] = baro_low->update_count;
    baro_diag[12] = baro->update_count;
    baro_diag[13] = baro->source_update_count;
    baro_diag[14] = baro->duplicate_sample_skip_count;
    baro_diag[15] = bmp585_drdy_count; /* R8R4: synchronized DRDY count. */

    baro_diag[16] = baro_diag_task_rate_x10;
    baro_diag[17] = baro_diag_service_rate_x10;
    baro_diag[18] = baro_diag_direct_read_rate_x10;
    baro_diag[19] = baro_diag_duplicate_rate_x10;
    baro_diag[20] = baro_diag_fresh_rate_x10;
    baro_diag[21] = SensorQual_RateX10(sensor_qual_baro_rate_hz);

    baro_diag[22] = task_last_exec_us;
    baro_diag[23] = task_max_exec_us;
    baro_diag[24] = task_overrun_count;
    baro_diag[25] = task_deadline_miss_count;

    baro_diag[26] = bmp585_last_service_interval_us;
    baro_diag[27] = bmp585_min_service_interval_us;
    baro_diag[28] = bmp585_max_service_interval_us;
    baro_diag[29] = bmp585_last_fresh_interval_us;
    baro_diag[30] = bmp585_min_fresh_interval_us;
    baro_diag[31] = bmp585_max_fresh_interval_us;

    baro_diag[32] = (uint32_t)bmp585_chip_id;
    baro_diag[33] = (uint32_t)bmp585_int_source_reg;
    baro_diag[34] = (uint32_t)bmp585_odr_config_reg;
    baro_diag[35] = (uint32_t)bmp585_osr_config_reg;
    baro_diag[36] = (uint32_t)bmp585_osr_eff;
    baro_diag[37] = (uint32_t)bmp585_status_reg;
    /* Low byte: latest INT_STATUS; next byte: verified standby readback. */
    baro_diag[38] =
        (uint32_t)bmp585_int_status |
        ((uint32_t)bmp585_standby_config_reg << 8);
    baro_diag[39] = bmp585_register_snapshot_count;
    baro_diag[40] = bmp585_register_snapshot_error_count;
    baro_diag[41] = bmp585_register_mismatch_count;
    baro_diag[42] = bmp585_last_register_snapshot_us;

    baro_diag[43] = baro_low->communication_error_count;
    baro_diag[44] = baro_low->invalid_adc_count;
    baro_diag[45] = baro_low->adc_read_count;
    baro_diag[46] = baro->source_update_count;
    baro_diag[47] = baro->update_count;
    baro_diag[48] = SensorQual_AbsDiffU32(
        task_run_count,
        bmp585_service_call_count
    );
    baro_diag[49] = SensorQual_AbsDiffU32(
        bmp585_fresh_sample_count,
        baro_low->update_count
    );
    baro_diag[50] = SensorQual_RateX10(sensor_qual_baro_rate_hz);
    baro_diag[51] = now_us;
    baro_diag[52] = baro->last_sample_interval_us;
    baro_diag[53] = baro->min_sample_interval_us;
    baro_diag[54] = baro->max_sample_interval_us;
    baro_diag[55] = (uint32_t)bmp585_config_ok;
    baro_diag[56] = (uint32_t)bmp585_direct_read_mode_ok; /* R8R5 direct atomic OK */
    baro_diag[57] = (uint32_t)bmp585_register_readback_ok;
    /* V8.19E init/readback paketi:
     * bits 7:0   INT_CONFIG yazma oncesi
     * bits 15:8  INT_CONFIG yazilmak istenen
     * bits 23:16 INT_CONFIG anlik readback
     * bits 27:24 konfigurasyon asamasi (8 = runtime hazir)
     * bit 28     STANDBY 0x80 dogrulandi
     * bit 29     INT_CONFIG bilerek degistirilmedi
     * bit 30     DRDY kapali direct-read modu dogrulandi
     * bit 31     zorunlu runtime konfigurasyonu dogrulandi. */
    baro_diag[58] =
        (uint32_t)bmp585_int_config_before_reg |
        ((uint32_t)bmp585_int_config_written_reg << 8) |
        ((uint32_t)bmp585_int_config_reg << 16) |
        (((uint32_t)bmp585_config_stage & 0x0FUL) << 24) |
        ((uint32_t)standby_ok << 28) |
        ((uint32_t)bmp585_int_config_ok << 29) |
        ((uint32_t)bmp585_direct_read_mode_ok << 30) |
        ((uint32_t)bmp585_config_ok << 31);
    baro_diag[59] = seq;

    /* Build the complete snapshot and checksum while word 3 stays odd.
     * Publishing the final even sequence last prevents a debugger refresh
     * from accepting a block whose checksum still belongs to the old frame. */
    checksum = BaroDiag_Checksum(seq);
    baro_diag[60] = checksum;
    baro_diag[61] = ~checksum;
    baro_diag[62] = 0x4241524FUL; /* 'BARO' */
    baro_diag[63] = 0x56313945UL; /* 'V19E' */
    __DMB();
    baro_diag[3] = seq;

}

void SensorQualification_Init(void)
{
    uint32_t i;
    BarometerData_t baro = Barometer_GetData();
    MS5611_SPI_Data_t baro_low = MS5611_SPI_GetData();
    LidarData_t lidar = Lidar_GetData();
    Task_t *baro_task = Scheduler_GetTask(1UL);

    for (i = 0UL; i < SENSOR_QUAL_DIAG_WORDS; i++)
    {
        sensor_qual_diag[i] = 0UL;
    }

    for (i = 0UL; i < BARO_DIAG_WORDS; i++)
    {
        baro_diag[i] = 0UL;
    }

    sensor_qual_state = SENSOR_QUAL_STATE_WARMUP;
    sensor_qual_pass = 0U;
    sensor_qual_good_windows = 0U;
    sensor_qual_flags = 0UL;

    sensor_qual_imu_rate_hz = 0.0f;
    sensor_qual_baro_rate_hz = 0.0f;
    sensor_qual_lidar_rate_hz = 0.0f;

    window_start_us = micros();
    window_count = 0UL;

    last_imu_updates = sensor_imu_valid_update_count;
    last_baro_updates = baro.update_count;
    last_lidar_updates = lidar.update_count;

    last_imu_dma_error = imu_dma_error_count;
    last_imu_dma_timeout = IMU_GetDmaTimeoutCount();
    last_imu_stale = IMU_GetStaleCount();
    last_imu_pattern = IMU_GetPatternErrorCount();
    last_imu_recovery = IMU_GetRecoveryCount();
    last_imu_register_error = IMU_GetRegisterErrorCount();
    last_imu_reject = imu_redundant_reject_count;

    last_baro_invalid = baro.invalid_sample_count;
    last_baro_comm_error = baro_low.communication_error_count;

    last_lidar_error = lidar.error_count;
    last_lidar_timeout = lidar.timeout_count;
    last_lidar_dma_error = lidar.dma_error_count;
    last_lidar_pending_overrun = lidar_sample_pending_overrun_count;

    window_imu_error_delta = 0UL;
    window_baro_error_delta = 0UL;
    window_lidar_error_delta = 0UL;

    diag_sequence = 0UL;
    baro_diag_sequence = 0UL;

    last_baro_task_runs = (baro_task != 0) ? baro_task->run_count : 0UL;
    last_bmp585_service_calls = bmp585_service_call_count;
    last_bmp585_direct_reads = bmp585_measurement_read_count;
    last_bmp585_duplicates = bmp585_duplicate_raw_count;
    last_bmp585_fresh_samples = bmp585_fresh_sample_count;

    baro_diag_task_rate_x10 = 0UL;
    baro_diag_service_rate_x10 = 0UL;
    baro_diag_direct_read_rate_x10 = 0UL;
    baro_diag_duplicate_rate_x10 = 0UL;
    baro_diag_fresh_rate_x10 = 0UL;
}

void SensorQualification_Update10Hz(void)
{
    const SensorData_t *sensor = SensorManager_GetDataPtr();
    const BarometerData_t *baro = Barometer_GetDataPtr();
    const LidarData_t *lidar = Lidar_GetDataPtr();
    MS5611_SPI_Data_t baro_low = MS5611_SPI_GetData();
    Task_t *baro_task = Scheduler_GetTask(1UL);

    uint32_t now_us = micros();
    uint32_t elapsed_us = now_us - window_start_us;
    uint32_t flags = 0UL;

    uint8_t imu_rate_ok = 0U;
    uint8_t baro_rate_ok = 0U;
    uint8_t lidar_rate_ok = 0U;
    uint8_t values_ok = 0U;
    uint8_t ages_ok = 0U;
    uint8_t imu_rest_ok = 0U;
    uint8_t baro_fresh_coherent = 0U;
    uint8_t instant_ok = 0U;

    uint8_t imu_window_clean = (window_imu_error_delta == 0UL) ? 1U : 0U;
    uint8_t baro_window_clean = (window_baro_error_delta == 0UL) ? 1U : 0U;
    uint8_t lidar_window_clean = (window_lidar_error_delta == 0UL) ? 1U : 0U;

    uint32_t imu_age = 0xFFFFFFFFUL;
    uint32_t baro_age = 0xFFFFFFFFUL;
    uint32_t lidar_age = 0xFFFFFFFFUL;

    if (IMU_GetLastSampleTimestampUs() != 0UL)
    {
        imu_age = now_us - IMU_GetLastSampleTimestampUs();
    }
    if (baro->last_sample_timestamp_us != 0UL)
    {
        baro_age = now_us - baro->last_sample_timestamp_us;
    }
    if (lidar->last_sample_timestamp_us != 0UL)
    {
        lidar_age = now_us - lidar->last_sample_timestamp_us;
    }

    if (elapsed_us >= 1000000UL)
    {
        float seconds = (float)elapsed_us * 1.0e-6f;

        uint32_t imu_now = sensor_imu_valid_update_count;
        uint32_t baro_now = baro->update_count;
        uint32_t lidar_now = lidar->update_count;

        uint32_t imu_dma_now = imu_dma_error_count;
        uint32_t imu_dma_timeout_now = IMU_GetDmaTimeoutCount();
        uint32_t imu_stale_now = IMU_GetStaleCount();
        uint32_t imu_pattern_now = IMU_GetPatternErrorCount();
        uint32_t imu_recovery_now = IMU_GetRecoveryCount();
        uint32_t imu_register_now = IMU_GetRegisterErrorCount();
        uint32_t imu_reject_now = imu_redundant_reject_count;

        uint32_t baro_task_runs_now =
            (baro_task != 0) ? baro_task->run_count : last_baro_task_runs;
        uint32_t bmp585_service_now = bmp585_service_call_count;
        uint32_t bmp585_direct_read_now = bmp585_measurement_read_count;
        uint32_t bmp585_duplicate_now = bmp585_duplicate_raw_count;
        uint32_t bmp585_fresh_now = bmp585_fresh_sample_count;

        uint32_t baro_invalid_now = baro->invalid_sample_count;
        uint32_t baro_comm_now = baro_low.communication_error_count;

        uint32_t lidar_error_now = lidar->error_count;
        uint32_t lidar_timeout_now = lidar->timeout_count;
        uint32_t lidar_dma_error_now = lidar->dma_error_count;
        uint32_t lidar_overrun_now = lidar_sample_pending_overrun_count;

        sensor_qual_imu_rate_hz =
            (float)(imu_now - last_imu_updates) / seconds;
        sensor_qual_baro_rate_hz =
            (float)(baro_now - last_baro_updates) / seconds;
        sensor_qual_lidar_rate_hz =
            (float)(lidar_now - last_lidar_updates) / seconds;

        baro_diag_task_rate_x10 = SensorQual_CountRateX10(
            baro_task_runs_now - last_baro_task_runs,
            seconds
        );
        baro_diag_service_rate_x10 = SensorQual_CountRateX10(
            bmp585_service_now - last_bmp585_service_calls,
            seconds
        );
        baro_diag_direct_read_rate_x10 = SensorQual_CountRateX10(
            bmp585_direct_read_now - last_bmp585_direct_reads,
            seconds
        );
        baro_diag_duplicate_rate_x10 = SensorQual_CountRateX10(
            bmp585_duplicate_now - last_bmp585_duplicates,
            seconds
        );
        baro_diag_fresh_rate_x10 = SensorQual_CountRateX10(
            bmp585_fresh_now - last_bmp585_fresh_samples,
            seconds
        );

        /*
         * A redundant triplet is sampled while the ISM330DLC itself runs at
         * 1.666 kHz. Consecutive raw values are therefore allowed to differ.
         * Only transport/validation faults and a fully rejected triplet make
         * the qualification window dirty. Corrected disagreements remain in
         * the diagnostic counters but are not flight-data failures.
         */
        window_imu_error_delta =
            (imu_dma_now - last_imu_dma_error) +
            (imu_dma_timeout_now - last_imu_dma_timeout) +
            (imu_stale_now - last_imu_stale) +
            (imu_pattern_now - last_imu_pattern) +
            (imu_recovery_now - last_imu_recovery) +
            (imu_register_now - last_imu_register_error) +
            (imu_reject_now - last_imu_reject);

        window_baro_error_delta =
            (baro_invalid_now - last_baro_invalid) +
            (baro_comm_now - last_baro_comm_error);

        window_lidar_error_delta =
            (lidar_error_now - last_lidar_error) +
            (lidar_timeout_now - last_lidar_timeout) +
            (lidar_dma_error_now - last_lidar_dma_error) +
            (lidar_overrun_now - last_lidar_pending_overrun);

        last_imu_updates = imu_now;
        last_baro_updates = baro_now;
        last_lidar_updates = lidar_now;

        last_imu_dma_error = imu_dma_now;
        last_imu_dma_timeout = imu_dma_timeout_now;
        last_imu_stale = imu_stale_now;
        last_imu_pattern = imu_pattern_now;
        last_imu_recovery = imu_recovery_now;
        last_imu_register_error = imu_register_now;
        last_imu_reject = imu_reject_now;

        last_baro_task_runs = baro_task_runs_now;
        last_bmp585_service_calls = bmp585_service_now;
        last_bmp585_direct_reads = bmp585_direct_read_now;
        last_bmp585_duplicates = bmp585_duplicate_now;
        last_bmp585_fresh_samples = bmp585_fresh_now;

        last_baro_invalid = baro_invalid_now;
        last_baro_comm_error = baro_comm_now;

        last_lidar_error = lidar_error_now;
        last_lidar_timeout = lidar_timeout_now;
        last_lidar_dma_error = lidar_dma_error_now;
        last_lidar_pending_overrun = lidar_overrun_now;

        window_start_us = now_us;
        window_count++;

        imu_window_clean = (window_imu_error_delta == 0UL) ? 1U : 0U;
        baro_window_clean = (window_baro_error_delta == 0UL) ? 1U : 0U;
        lidar_window_clean = (window_lidar_error_delta == 0UL) ? 1U : 0U;
    }

    imu_rate_ok = SensorQual_RateInside(
        sensor_qual_imu_rate_hz,
        APP_SENSOR_QUAL_IMU_RATE_MIN_HZ,
        APP_SENSOR_QUAL_IMU_RATE_MAX_HZ
    );

    baro_rate_ok = SensorQual_RateInside(
        sensor_qual_baro_rate_hz,
        APP_SENSOR_QUAL_BARO_RATE_MIN_HZ,
        APP_SENSOR_QUAL_BARO_RATE_MAX_HZ
    );

    lidar_rate_ok = SensorQual_RateInside(
        sensor_qual_lidar_rate_hz,
        APP_SENSOR_QUAL_LIDAR_RATE_MIN_HZ,
        APP_SENSOR_QUAL_LIDAR_RATE_MAX_HZ
    );

    values_ok =
        ((SensorQual_Finite(sensor->accel_norm_g) != 0U) &&
         (SensorQual_Finite(sensor->gyro_x_dps) != 0U) &&
         (SensorQual_Finite(sensor->gyro_y_dps) != 0U) &&
         (SensorQual_Finite(sensor->gyro_z_dps) != 0U) &&
         (SensorQual_Finite(baro->pressure_pa) != 0U) &&
         (baro->pressure_pa >= APP_BARO_PRESSURE_MIN_PA) &&
         (baro->pressure_pa <= APP_BARO_PRESSURE_MAX_PA) &&
         (SensorQual_Finite(lidar->distance_m) != 0U) &&
         (lidar->distance_m >= 0.06f) &&
         (lidar->distance_m <= 40.0f)) ? 1U : 0U;

    ages_ok =
        ((imu_age <= 5000UL) &&
         (baro_age <= APP_SENSOR_QUAL_BARO_FRESH_US) &&
         (lidar_age <= APP_SENSOR_QUAL_LIDAR_FRESH_US)) ? 1U : 0U;

    imu_rest_ok =
        ((sensor->accel_norm_g >= APP_SENSOR_QUAL_IMU_REST_ACCEL_MIN_G) &&
         (sensor->accel_norm_g <= APP_SENSOR_QUAL_IMU_REST_ACCEL_MAX_G) &&
         (fabsf(sensor->gyro_x_dps) <= APP_SENSOR_QUAL_IMU_REST_GYRO_MAX_DPS) &&
         (fabsf(sensor->gyro_y_dps) <= APP_SENSOR_QUAL_IMU_REST_GYRO_MAX_DPS) &&
         (fabsf(sensor->gyro_z_dps) <= APP_SENSOR_QUAL_IMU_REST_GYRO_MAX_DPS)) ? 1U : 0U;

    {
        uint32_t fresh = bmp585_fresh_sample_count;
        uint32_t accepted = baro_low.update_count;
        uint32_t difference = (fresh >= accepted)
            ? (fresh - accepted)
            : (accepted - fresh);

        baro_fresh_coherent = (difference <= 1UL) ? 1U : 0U;
    }

    flags |= (1UL << 0); /* alive */

#if (((APP_V49_ESKF_RCS_PHYSICAL_ENABLED != 0U) && \
      (APP_GNC_ACTIVE_ENABLED == 0U) && \
      (APP_GNC_RCS_DRY_RUN == 0U) && \
      (APP_GNC_NEEDLE_PHYSICAL_ENABLED == 0U) && \
      (APP_VENT_SERVO_ENABLED == 0U)) || \
     ((APP_GNC_ACTIVE_ENABLED == 0U) && \
      (APP_GNC_RCS_DRY_RUN != 0U) && \
      (APP_GNC_NEEDLE_PHYSICAL_ENABLED == 0U) && \
      (APP_VENT_SERVO_ENABLED == 0U)) || \
     ((APP_GNC_RCS_RELAY_BENCH_MODE != 0U) && \
      (APP_GNC_ACTIVE_ENABLED != 0U) && \
      (APP_GNC_RCS_DRY_RUN == 0U) && \
      (APP_GNC_NEEDLE_PHYSICAL_ENABLED == 0U) && \
      (APP_VENT_SERVO_ENABLED == 0U)) || \
     ((APP_GNC_COMBINED_DRY_RUN_MODE != 0U) && \
      (APP_GNC_RCS_RELAY_BENCH_MODE == 0U) && \
      (APP_GNC_ACTIVE_ENABLED != 0U) && \
      (APP_GNC_RCS_DRY_RUN == 0U) && \
      (APP_GNC_NEEDLE_PHYSICAL_ENABLED == 0U) && \
      (APP_GNC_VERTICAL_CONTROLLER_IMPLEMENTED != 0U) && \
      (APP_VENT_SERVO_ENABLED == 0U)))
    flags |= (1UL << 1); /* selected actuator configuration is coherent */
#endif

    if (IMU_IsConnected() != 0U)                 flags |= (1UL << 2);
    if (IMU_IsLastSampleValid() != 0U)           flags |= (1UL << 3);
    if (sensor_imu_calibration_complete != 0U)   flags |= (1UL << 4);
    if ((sensor_imu_filter_config_ok != 0U) &&
        (sensor_imu_filter_initialized != 0U))    flags |= (1UL << 5);
    if (imu_rate_ok != 0U)                       flags |= (1UL << 6);
    if (imu_window_clean != 0U)                  flags |= (1UL << 7);

    if (baro->connected != 0U)                   flags |= (1UL << 8);
    if (baro->healthy != 0U)                     flags |= (1UL << 9);
    if (baro->calibrated != 0U)                  flags |= (1UL << 10);
    if ((baro->filter_config_ok != 0U) &&
        (baro->filter_initialized != 0U))         flags |= (1UL << 11);
    if (baro_rate_ok != 0U)                      flags |= (1UL << 12);
    if (baro_window_clean != 0U)                 flags |= (1UL << 13);
    if (bmp585_chip_id == 0x51U)                 flags |= (1UL << 14);
    if (bmp585_config_ok != 0U)                  flags |= (1UL << 15);

    if (lidar->connected != 0U)                  flags |= (1UL << 16);
    if (lidar->distance_valid != 0U)             flags |= (1UL << 17);
    if ((lidar->filter_config_ok != 0U) &&
        (lidar->filter_initialized != 0U))        flags |= (1UL << 18);
    if (lidar_profile_config_ok != 0U)            flags |= (1UL << 19);
    if (lidar_rate_ok != 0U)                     flags |= (1UL << 20);
    if (lidar_window_clean != 0U)                flags |= (1UL << 21);

#if (APP_LIDAR_STARTUP_CALIBRATION_ENABLED == 0U)
    flags |= (1UL << 22);
#endif

    if (values_ok != 0U)                         flags |= (1UL << 23);
    if (ages_ok != 0U)                           flags |= (1UL << 24);

    if ((IMU_GetDeviceID() == IMU_WHO_AM_I_ISM330DLC) ||
        (IMU_GetDeviceID() == IMU_WHO_AM_I_ALT)) flags |= (1UL << 27);

    /* Direct-read fresh samples and accepted low-level samples must match. */
    if (baro_fresh_coherent != 0U)               flags |= (1UL << 28);

    /* V8.19 bench qualification is intentionally stationary. This catches
     * wrong axis/gravity scaling and residual gyro bias before control work. */
    if (imu_rest_ok != 0U)                       flags |= (1UL << 29);

    /* P112R12R8R5: direct 6-byte atomic path, no DRDY acquisition gate. */
    if (bmp585_direct_read_mode_ok != 0U)         flags |= (1UL << 30);

    instant_ok =
        (((flags & (1UL << 1)) != 0UL) &&
         ((flags & (1UL << 2)) != 0UL) &&
         ((flags & (1UL << 3)) != 0UL) &&
         ((flags & (1UL << 4)) != 0UL) &&
         ((flags & (1UL << 5)) != 0UL) &&
         ((flags & (1UL << 6)) != 0UL) &&
         ((flags & (1UL << 7)) != 0UL) &&
         ((flags & (1UL << 8)) != 0UL) &&
         ((flags & (1UL << 9)) != 0UL) &&
         ((flags & (1UL << 10)) != 0UL) &&
         ((flags & (1UL << 11)) != 0UL) &&
         ((flags & (1UL << 12)) != 0UL) &&
         ((flags & (1UL << 13)) != 0UL) &&
         ((flags & (1UL << 14)) != 0UL) &&
         ((flags & (1UL << 15)) != 0UL) &&
         ((flags & (1UL << 16)) != 0UL) &&
         ((flags & (1UL << 17)) != 0UL) &&
         ((flags & (1UL << 18)) != 0UL) &&
         ((flags & (1UL << 19)) != 0UL) &&
         ((flags & (1UL << 20)) != 0UL) &&
         ((flags & (1UL << 21)) != 0UL) &&
         ((flags & (1UL << 22)) != 0UL) &&
         ((flags & (1UL << 23)) != 0UL) &&
         ((flags & (1UL << 24)) != 0UL) &&
         ((flags & (1UL << 27)) != 0UL) &&
         ((flags & (1UL << 28)) != 0UL) &&
         ((flags & (1UL << 29)) != 0UL) &&
         ((flags & (1UL << 30)) != 0UL)) ? 1U : 0U;

    if (instant_ok != 0U)
    {
        flags |= (1UL << 25);
    }

    /* Change the qualification verdict only on completed 1-second windows. */
    if (elapsed_us >= 1000000UL)
    {
        if (instant_ok != 0U)
        {
            if (sensor_qual_good_windows < 255U)
            {
                sensor_qual_good_windows++;
            }
        }
        else
        {
            sensor_qual_good_windows = 0U;
        }

        if (sensor_qual_good_windows >= APP_SENSOR_QUAL_GOOD_WINDOWS_REQUIRED)
        {
            sensor_qual_state = SENSOR_QUAL_STATE_PASS;
            sensor_qual_pass = 1U;
        }
        else if (instant_ok != 0U)
        {
            sensor_qual_state = SENSOR_QUAL_STATE_QUALIFYING;
            sensor_qual_pass = 0U;
        }
        else
        {
            sensor_qual_state = SENSOR_QUAL_STATE_FAIL;
            sensor_qual_pass = 0U;
        }
    }
    else if (window_count == 0UL)
    {
        sensor_qual_state = SENSOR_QUAL_STATE_WARMUP;
        sensor_qual_pass = 0U;
    }

    if (sensor_qual_pass != 0U)
    {
        flags |= (1UL << 26);
    }

    sensor_qual_flags = flags;

    SensorQual_WriteDiagnostic(sensor, baro, &baro_low, lidar, now_us);
    SensorQual_WriteBarometerDiagnostic(baro, &baro_low, baro_task, now_us);
}
