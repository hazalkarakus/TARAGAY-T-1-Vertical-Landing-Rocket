#include "Modules/Sensors/Lidar/Lidar.h"

#include "Common/app_config.h"
#include "Common/Filters/butterworth_filter.h"
#include "Services/Timebase/timebase.h"

#include "i2c.h"
#include "stm32f4xx_hal.h"

#define LIDAR_I2C_ADDRESS_7BIT       0x62U
#define LIDAR_I2C_ADDRESS            (LIDAR_I2C_ADDRESS_7BIT << 1)
#define LIDAR_REG_ACQ_COMMAND        0x00U
#define LIDAR_REG_DISTANCE_AUTO      0x8FU
#define LIDAR_CMD_START_MEASUREMENT  0x04U
#define LIDAR_MEASUREMENT_TIME_MS    5UL
#define LIDAR_DMA_TIMEOUT_MS         20UL
#define LIDAR_WAIT_BUSY_TIMEOUT_MS   30UL
#define LIDAR_SAMPLE_WATCHDOG_MS     100UL
#define LIDAR_RUNTIME_RETRY_MS       50UL
#define LIDAR_RECOVERY_SETTLE_MS     2UL
#define LIDAR_RECOVERY_MAX_ATTEMPTS  3U
#define LIDAR_FIRST_SAMPLE_GRACE_MS   150UL
#define LIDAR_BUS_CLEAR_CLOCKS         9U
#define LIDAR_BUS_CLEAR_HALF_US        5UL

#define LIDAR_RECOVERY_STEP_IDLE       0U
#define LIDAR_RECOVERY_STEP_QUIESCE    1U
#define LIDAR_RECOVERY_STEP_DMA_WAIT   2U
#define LIDAR_RECOVERY_STEP_HOST_RESET 3U
#define LIDAR_RECOVERY_STEP_SETTLE     4U
#define LIDAR_RECOVERY_STEP_RESTART    5U

/*
 * LIDAR-Lite may not acknowledge immediately after the MCU starts.
 * Initialization is therefore retried non-blockingly instead of permanently
 * disabling the driver after one failed HAL_I2C_IsDeviceReady call.
 */
#define LIDAR_STARTUP_DELAY_MS       100UL
#define LIDAR_RETRY_INTERVAL_MS      500UL

#define LIDAR_MIN_DISTANCE_CM        5U
#define LIDAR_MAX_DISTANCE_CM        4000U
#define LIDAR_MEDIAN_WINDOW_SIZE     3U

static LidarData_t lidar_data;
static Butterworth2LPF_t lidar_distance_filter;
static uint16_t lidar_median_window[LIDAR_MEDIAN_WINDOW_SIZE];
static uint8_t lidar_median_index;

static uint8_t lidar_tx_value = LIDAR_CMD_START_MEASUREMENT;
static uint8_t lidar_rx_dma_buffer[2];
static volatile uint8_t lidar_sample_pending;
static volatile uint8_t lidar_pending_high;
static volatile uint8_t lidar_pending_low;
static volatile uint32_t lidar_dma_operation_start_ms;
static volatile uint32_t lidar_next_retry_ms;
static uint8_t lidar_recovery_step_state = LIDAR_RECOVERY_STEP_IDLE;
static uint8_t lidar_recovery_attempt_in_cycle = 0U;
static uint32_t lidar_recovery_start_us = 0UL;
static uint32_t lidar_recovery_due_ms = 0UL;
/* P38: stale watchdog is intentionally disarmed across reconnect/recovery.
 * It is armed only after a real, validated distance sample is published. */
static uint8_t lidar_watchdog_armed = 0U;
static uint8_t lidar_recovery_confirmation_pending = 0U;
static uint32_t lidar_first_sample_deadline_ms = 0UL;


/* Startup calibration state; processed only from Lidar_Update task context. */
static uint8_t lidar_calibration_complete_state = 0U;
static uint16_t lidar_calibration_sample_count_state = 0U;
static float lidar_calibration_sum_m = 0.0f;
static float lidar_calibration_sum_square_m = 0.0f;
static float lidar_calibration_last_m = 0.0f;
static float lidar_calibration_offset_m_state = 0.0f;

volatile uint8_t lidar_initialized;
volatile uint8_t lidar_connected;
volatile uint8_t lidar_data_ready;
volatile uint8_t lidar_distance_valid;
volatile uint16_t lidar_distance_cm;
volatile float lidar_distance_m;

volatile uint16_t lidar_raw_distance_cm;
volatile float lidar_raw_distance_m;
volatile uint8_t lidar_calibration_complete;
volatile uint16_t lidar_calibration_sample_count;
volatile uint32_t lidar_calibration_reset_count;
volatile float lidar_calibration_reference_m;
volatile float lidar_calibration_offset_m;
volatile float lidar_calibration_stddev_m;
volatile uint8_t lidar_calibration_last_reject_reason;
volatile uint8_t lidar_median_initialized;
volatile uint16_t lidar_median_distance_cm;
volatile float lidar_median_distance_m;
volatile uint8_t lidar_filter_enabled;
volatile uint8_t lidar_filter_config_ok;
volatile uint8_t lidar_filter_initialized;
volatile float lidar_filtered_distance_m;
volatile uint32_t lidar_update_count;
volatile uint32_t lidar_read_count;
volatile uint32_t lidar_error_count;
volatile uint32_t lidar_timeout_count;
volatile uint32_t lidar_dma_tx_start_count;
volatile uint32_t lidar_dma_tx_complete_count;
volatile uint32_t lidar_dma_rx_start_count;
volatile uint32_t lidar_dma_rx_complete_count;
volatile uint32_t lidar_dma_busy_count;
volatile uint32_t lidar_dma_error_count;
volatile uint32_t lidar_wait_busy_timeout_count;

volatile uint32_t lidar_probe_attempt_count;
volatile uint32_t lidar_probe_success_count;
volatile uint32_t lidar_probe_failure_count;
volatile uint32_t lidar_reconnect_count;
volatile uint32_t lidar_bus_recovery_count;
volatile uint32_t lidar_last_i2c_error_code;
volatile uint32_t lidar_next_retry_timestamp_ms;
volatile uint8_t lidar_recovery_active = 0U;
volatile uint8_t lidar_recovery_step = LIDAR_RECOVERY_STEP_IDLE;
volatile uint32_t lidar_recovery_attempt_count = 0UL;
volatile uint32_t lidar_recovery_success_count = 0UL;
volatile uint32_t lidar_recovery_failure_count = 0UL;
volatile uint32_t lidar_recovery_last_duration_us = 0UL;
volatile uint32_t lidar_recovery_max_duration_us = 0UL;
volatile uint32_t lidar_recovery_step_max_us = 0UL;

volatile uint32_t lidar_filter_update_count;
volatile uint32_t lidar_filter_reset_count;
volatile uint32_t lidar_last_sample_timestamp_us;
volatile uint32_t lidar_last_sample_interval_us;
volatile uint32_t lidar_min_sample_interval_us;
volatile uint32_t lidar_max_sample_interval_us;
volatile uint8_t lidar_state;
volatile uint8_t lidar_last_hal_status;
volatile uint32_t lidar_measurement_start_ms;
volatile uint32_t lidar_last_measure_duration_ms;
volatile uint8_t lidar_last_high_byte;
volatile uint8_t lidar_last_low_byte;
volatile float lidar_filter_b0;
volatile float lidar_filter_b1;
volatile float lidar_filter_b2;
volatile float lidar_filter_a1;
volatile float lidar_filter_a2;

/* -------------------------------------------------------------------------- */
/* V8.19M compatibility diagnostics                                           */
/* -------------------------------------------------------------------------- */
/*
 * The small-board proven driver intentionally uses the Garmin default
 * acquisition behavior and does not poll STATUS between every measurement.
 * These symbols keep the richer V8.19M qualification/telemetry interface
 * link-compatible while preserving the proven I2C/DMA transaction sequence.
 */
volatile uint8_t lidar_profile_config_ok = 0U;
volatile uint8_t lidar_profile_sig_count_value = 0x80U;
volatile uint8_t lidar_profile_acq_config_value = 0x08U;
volatile uint8_t lidar_profile_threshold_value = 0x00U;

volatile uint32_t lidar_bias_command_count = 0UL;
volatile uint32_t lidar_no_bias_command_count = 0UL;
volatile uint32_t lidar_status_poll_start_count = 0UL;
volatile uint32_t lidar_status_poll_complete_count = 0UL;
volatile uint32_t lidar_status_busy_count = 0UL;
volatile uint8_t lidar_last_status_reg = 0U;

volatile uint32_t lidar_last_trigger_interval_us = 0UL;
volatile uint32_t lidar_min_trigger_interval_us = 0UL;
volatile uint32_t lidar_max_trigger_interval_us = 0UL;
volatile uint32_t lidar_sample_pending_overrun_count = 0UL;

static uint32_t lidar_v819_last_trigger_us = 0UL;

static uint8_t Lidar_StartMeasurementDMA(void);


static void Lidar_UpdateLiveDebug(void)
{
    lidar_initialized = lidar_data.initialized;
    lidar_connected = lidar_data.connected;
    lidar_data_ready = lidar_data.data_ready;
    lidar_distance_valid = lidar_data.distance_valid;
    lidar_distance_cm = lidar_data.distance_cm;
    lidar_distance_m = lidar_data.distance_m;
    lidar_calibration_complete = lidar_calibration_complete_state;
    lidar_calibration_sample_count = lidar_calibration_sample_count_state;
    lidar_calibration_reference_m = APP_LIDAR_CALIBRATION_REFERENCE_M;
    lidar_calibration_offset_m = lidar_calibration_offset_m_state;
    lidar_median_initialized = lidar_data.median_initialized;
    lidar_median_distance_cm = lidar_data.median_distance_cm;
    lidar_median_distance_m = lidar_data.median_distance_m;
    lidar_filter_enabled = lidar_data.filter_enabled;
    lidar_filter_config_ok = lidar_data.filter_config_ok;
    lidar_filter_initialized = lidar_data.filter_initialized;
    lidar_filtered_distance_m = lidar_data.filtered_distance_m;
    lidar_update_count = lidar_data.update_count;
    lidar_read_count = lidar_data.read_count;
    lidar_error_count = lidar_data.error_count;
    lidar_timeout_count = lidar_data.timeout_count;
    lidar_dma_tx_start_count = lidar_data.dma_tx_start_count;
    lidar_dma_tx_complete_count = lidar_data.dma_tx_complete_count;
    lidar_dma_rx_start_count = lidar_data.dma_rx_start_count;
    lidar_dma_rx_complete_count = lidar_data.dma_rx_complete_count;
    lidar_dma_busy_count = lidar_data.dma_busy_count;
    lidar_dma_error_count = lidar_data.dma_error_count;
    lidar_wait_busy_timeout_count = lidar_data.wait_busy_timeout_count;
    lidar_data.recovery_active = lidar_recovery_active;
    lidar_data.recovery_step = lidar_recovery_step;
    lidar_data.recovery_attempt_count = lidar_recovery_attempt_count;
    lidar_data.recovery_success_count = lidar_recovery_success_count;
    lidar_data.recovery_failure_count = lidar_recovery_failure_count;
    lidar_data.recovery_last_duration_us = lidar_recovery_last_duration_us;
    lidar_data.recovery_max_duration_us = lidar_recovery_max_duration_us;
    lidar_data.recovery_step_max_us = lidar_recovery_step_max_us;
    lidar_filter_update_count = lidar_data.filter_update_count;
    lidar_filter_reset_count = lidar_data.filter_reset_count;
    lidar_last_sample_timestamp_us = lidar_data.last_sample_timestamp_us;
    lidar_last_sample_interval_us = lidar_data.last_sample_interval_us;
    lidar_min_sample_interval_us = lidar_data.min_sample_interval_us;
    lidar_max_sample_interval_us = lidar_data.max_sample_interval_us;
    lidar_state = lidar_data.state;
    lidar_last_hal_status = lidar_data.last_hal_status;
    lidar_measurement_start_ms = lidar_data.measurement_start_ms;
    lidar_last_measure_duration_ms = lidar_data.last_measure_duration_ms;
}

static void Lidar_ResetData(void)
{
    uint8_t i;
    lidar_data = (LidarData_t){0};
    lidar_data.state = LIDAR_STATE_UNINITIALIZED;
    lidar_median_index = 0U;
    lidar_sample_pending = 0U;
    lidar_pending_high = 0U;
    lidar_pending_low = 0U;
    lidar_rx_dma_buffer[0] = 0U;
    lidar_rx_dma_buffer[1] = 0U;
    lidar_next_retry_ms = 0UL;
    lidar_recovery_step_state = LIDAR_RECOVERY_STEP_IDLE;
    lidar_recovery_attempt_in_cycle = 0U;
    lidar_recovery_start_us = 0UL;
    lidar_recovery_due_ms = 0UL;
    lidar_watchdog_armed = 0U;
    lidar_recovery_confirmation_pending = 0U;
    lidar_first_sample_deadline_ms = 0UL;
    lidar_recovery_active = 0U;
    lidar_recovery_step = LIDAR_RECOVERY_STEP_IDLE;
    lidar_recovery_attempt_count = 0UL;
    lidar_recovery_success_count = 0UL;
    lidar_recovery_failure_count = 0UL;
    lidar_recovery_last_duration_us = 0UL;
    lidar_recovery_max_duration_us = 0UL;
    lidar_recovery_step_max_us = 0UL;

    lidar_profile_config_ok = 0U;
    lidar_profile_sig_count_value = 0x80U;
    lidar_profile_acq_config_value = 0x08U;
    lidar_profile_threshold_value = 0x00U;
    lidar_bias_command_count = 0UL;
    lidar_no_bias_command_count = 0UL;
    lidar_status_poll_start_count = 0UL;
    lidar_status_poll_complete_count = 0UL;
    lidar_status_busy_count = 0UL;
    lidar_last_status_reg = 0U;
    lidar_last_trigger_interval_us = 0UL;
    lidar_min_trigger_interval_us = 0UL;
    lidar_max_trigger_interval_us = 0UL;
    lidar_sample_pending_overrun_count = 0UL;
    lidar_v819_last_trigger_us = 0UL;

#if (APP_LIDAR_STARTUP_CALIBRATION_ENABLED != 0U)
    lidar_calibration_complete_state = 0U;
#else
    lidar_calibration_complete_state = 1U;
#endif
    lidar_calibration_sample_count_state = 0U;
    lidar_calibration_sum_m = 0.0f;
    lidar_calibration_sum_square_m = 0.0f;
    lidar_calibration_last_m = 0.0f;
    lidar_calibration_offset_m_state = 0.0f;
    lidar_calibration_stddev_m = 0.0f;
    lidar_calibration_last_reject_reason = 0U;
    lidar_calibration_reset_count = 0UL;
    lidar_raw_distance_cm = 0U;
    lidar_raw_distance_m = 0.0f;

    lidar_probe_attempt_count = 0UL;
    lidar_probe_success_count = 0UL;
    lidar_probe_failure_count = 0UL;
    lidar_reconnect_count = 0UL;
    lidar_bus_recovery_count = 0UL;
    lidar_last_i2c_error_code = 0UL;
    lidar_next_retry_timestamp_ms = 0UL;

    for (i = 0U; i < LIDAR_MEDIAN_WINDOW_SIZE; i++) lidar_median_window[i] = 0U;
    Lidar_UpdateLiveDebug();
}

static uint8_t Lidar_TimeReached(
    uint32_t now_ms,
    uint32_t target_ms
)
{
    return (((int32_t)(now_ms - target_ms)) >= 0) ? 1U : 0U;
}

static void Lidar_ScheduleRetry(uint32_t delay_ms)
{
    lidar_next_retry_ms = HAL_GetTick() + delay_ms;
    lidar_next_retry_timestamp_ms = lidar_next_retry_ms;

    lidar_watchdog_armed = 0U;
    lidar_recovery_confirmation_pending = 0U;
    lidar_first_sample_deadline_ms = 0UL;
    lidar_data.connected = 0U;
    lidar_data.distance_valid = 0U;
    lidar_data.data_ready = 0U;
    lidar_data.last_sample_timestamp_us = 0UL;
    lidar_data.state = LIDAR_STATE_ERROR;
}

static void Lidar_ScheduleRuntimeRestart(uint32_t delay_ms)
{
    lidar_next_retry_ms = HAL_GetTick() + delay_ms;
    lidar_next_retry_timestamp_ms = lidar_next_retry_ms;

    /* I2C has just been reinitialized in task context.  Keep the fast restart
     * path free of the blocking IsDeviceReady probe; a failed DMA start falls
     * back to the normal slow reconnect path. */
    lidar_watchdog_armed = 0U;
    lidar_recovery_confirmation_pending = 0U;
    lidar_first_sample_deadline_ms = 0UL;
    lidar_data.connected = 1U;
    lidar_data.distance_valid = 0U;
    lidar_data.data_ready = 0U;
    lidar_data.last_sample_timestamp_us = 0UL;
    lidar_data.state = LIDAR_STATE_ERROR;
}

static void Lidar_BeginFirstSampleGrace(uint8_t recovery_confirmation)
{
    /* Do not compare a newly restarted acquisition against the timestamp from
     * the previous I2C session.  A real sample must arrive before the stale
     * watchdog is re-armed. */
    lidar_watchdog_armed = 0U;
    lidar_recovery_confirmation_pending = recovery_confirmation;
    lidar_first_sample_deadline_ms = HAL_GetTick() + LIDAR_FIRST_SAMPLE_GRACE_MS;
    lidar_data.last_sample_timestamp_us = 0UL;
}

static void Lidar_RecordRecoveryDuration(void)
{
    uint32_t duration_us = micros() - lidar_recovery_start_us;
    lidar_recovery_last_duration_us = duration_us;
    if (duration_us > lidar_recovery_max_duration_us)
    {
        lidar_recovery_max_duration_us = duration_us;
    }
}

static void Lidar_BusClearDelayUs(uint32_t delay_us)
{
    uint32_t start_us = micros();
    while ((uint32_t)(micros() - start_us) < delay_us)
    {
        /* Bounded task-context delay: 5 us half-cycle only. */
    }
}

/*
 * R12R8R2: true I2C2 bus-unwedge before HAL_I2C_Init().
 *
 * The R12R8/R12R8R1 physical logs showed a single runtime timeout followed by
 * a permanently stuck recovery loop.  Each HOST_RESET then took ~36 ms and
 * every restart attempt failed.  That signature is consistent with the I2C
 * BUSY condition remaining asserted while a slave holds SDA low.  Re-running
 * HAL_I2C_Init() alone cannot clear that electrical bus state.
 *
 * Recovery is already in task context, I2C2/DMA are quiesced, and I2C2 is used
 * only by the LiDAR in this firmware.  Temporarily take PB10/PB11 as open-drain
 * GPIO, clock SCL up to nine times, generate a STOP, verify both lines released,
 * then return the pins to MX_I2C2_Init().  Total intentional pulse time is
 * bounded below 120 us.
 */
static uint8_t Lidar_BusClearGPIO(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t i;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Release both lines first. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
    Lidar_BusClearDelayUs(LIDAR_BUS_CLEAR_HALF_US);

    /* If SDA is held low, give the slave enough clocks to finish a byte. */
    for (i = 0U;
         (i < LIDAR_BUS_CLEAR_CLOCKS) &&
         (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET);
         i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        Lidar_BusClearDelayUs(LIDAR_BUS_CLEAR_HALF_US);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        Lidar_BusClearDelayUs(LIDAR_BUS_CLEAR_HALF_US);
    }

    /* Generate an explicit STOP: SDA low -> SCL released high -> SDA released. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    Lidar_BusClearDelayUs(LIDAR_BUS_CLEAR_HALF_US);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    Lidar_BusClearDelayUs(LIDAR_BUS_CLEAR_HALF_US);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    Lidar_BusClearDelayUs(LIDAR_BUS_CLEAR_HALF_US);

    if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) &&
        (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET))
    {
        return 1U;
    }

    return 0U;
}

static void Lidar_RequestRecovery(uint32_t delay_ms)
{
    if (lidar_recovery_active == 0U)
    {
        lidar_recovery_active = 1U;
        lidar_recovery_step_state = LIDAR_RECOVERY_STEP_QUIESCE;
        lidar_recovery_step = lidar_recovery_step_state;
        lidar_recovery_attempt_in_cycle = 0U;
        lidar_recovery_start_us = micros();
    }

    lidar_recovery_due_ms = HAL_GetTick() + delay_ms;
    lidar_watchdog_armed = 0U;
    lidar_recovery_confirmation_pending = 0U;
    lidar_first_sample_deadline_ms = 0UL;
    lidar_data.last_sample_timestamp_us = 0UL;
    lidar_data.connected = 0U;
    lidar_data.distance_valid = 0U;
    lidar_data.data_ready = 0U;
    lidar_data.state = LIDAR_STATE_RECOVERY;
}

static void Lidar_ServiceRecovery(void)
{
    uint32_t step_start_us;
    uint32_t step_us;

    if (lidar_recovery_active == 0U)
    {
        return;
    }

    if ((int32_t)(HAL_GetTick() - lidar_recovery_due_ms) < 0)
    {
        return;
    }

    step_start_us = micros();

    switch (lidar_recovery_step_state)
    {
        case LIDAR_RECOVERY_STEP_QUIESCE:
            lidar_recovery_attempt_in_cycle++;
            lidar_recovery_attempt_count++;

            /* P37: stop DMA without waiting.  A later 1 kHz service verifies
             * that the stream EN bits are really low before HAL cleanup. */
            __HAL_I2C_DISABLE(&hi2c2);
            if ((hi2c2.hdmatx != NULL) &&
                (hi2c2.hdmatx->State == HAL_DMA_STATE_BUSY))
            {
                __HAL_DMA_DISABLE(hi2c2.hdmatx);
            }
            if ((hi2c2.hdmarx != NULL) &&
                (hi2c2.hdmarx->State == HAL_DMA_STATE_BUSY))
            {
                __HAL_DMA_DISABLE(hi2c2.hdmarx);
            }

            lidar_sample_pending = 0U;
            lidar_recovery_step_state = LIDAR_RECOVERY_STEP_DMA_WAIT;
            break;

        case LIDAR_RECOVERY_STEP_DMA_WAIT:
        {
            uint8_t tx_stopped = 1U;
            uint8_t rx_stopped = 1U;

            if (hi2c2.hdmatx != NULL)
            {
                tx_stopped = ((hi2c2.hdmatx->Instance->CR & DMA_SxCR_EN) == 0U) ? 1U : 0U;
            }
            if (hi2c2.hdmarx != NULL)
            {
                rx_stopped = ((hi2c2.hdmarx->Instance->CR & DMA_SxCR_EN) == 0U) ? 1U : 0U;
            }

            if ((tx_stopped == 0U) || (rx_stopped == 0U))
            {
                /* No spin-loop: try again on the next 1 kHz service. */
                break;
            }

            /* EN is already clear, so HAL_DMA_Abort cannot sit in its timeout
             * loop; this only normalizes HAL state and clears DMA flags. */
            if ((hi2c2.hdmatx != NULL) &&
                (hi2c2.hdmatx->State == HAL_DMA_STATE_BUSY))
            {
                (void)HAL_DMA_Abort(hi2c2.hdmatx);
            }
            if ((hi2c2.hdmarx != NULL) &&
                (hi2c2.hdmarx->State == HAL_DMA_STATE_BUSY))
            {
                (void)HAL_DMA_Abort(hi2c2.hdmarx);
            }

            lidar_recovery_step_state = LIDAR_RECOVERY_STEP_HOST_RESET;
            break;
        }

        case LIDAR_RECOVERY_STEP_HOST_RESET:
            /* R12R8R2: deinit first, electrically clear PB10/PB11, then
             * recreate I2C2.  Do not enter HAL_I2C_Init while the bus is
             * still physically held low: that was the ~36 ms blocking path
             * seen in the R12R8R1 log. */
            (void)HAL_I2C_DeInit(&hi2c2);

            if (Lidar_BusClearGPIO() == 0U)
            {
                /* Keep the recovery bounded and retry forever.  No motor/RCS
                 * behavior depends on this path. */
                if (lidar_recovery_attempt_in_cycle >= LIDAR_RECOVERY_MAX_ATTEMPTS)
                {
                    lidar_recovery_failure_count++;
                    Lidar_RecordRecoveryDuration();
                    lidar_recovery_attempt_in_cycle = 0U;
                    lidar_recovery_due_ms = HAL_GetTick() + LIDAR_RETRY_INTERVAL_MS;
                }
                else
                {
                    lidar_recovery_due_ms = HAL_GetTick() + LIDAR_RUNTIME_RETRY_MS;
                }
                lidar_recovery_step_state = LIDAR_RECOVERY_STEP_QUIESCE;
                break;
            }

            __HAL_RCC_I2C2_FORCE_RESET();
            __HAL_RCC_I2C2_RELEASE_RESET();
            MX_I2C2_Init();
            lidar_data.bus_recovery_count++;
            lidar_bus_recovery_count++;
            lidar_recovery_due_ms = HAL_GetTick() + LIDAR_RECOVERY_SETTLE_MS;
            lidar_recovery_step_state = LIDAR_RECOVERY_STEP_SETTLE;
            break;

        case LIDAR_RECOVERY_STEP_SETTLE:
            lidar_recovery_step_state = LIDAR_RECOVERY_STEP_RESTART;
            break;

        case LIDAR_RECOVERY_STEP_RESTART:
            /* A real DMA write is the non-blocking connectivity probe. */
            lidar_data.connected = 1U;
            lidar_data.state = LIDAR_STATE_IDLE;
            if (Lidar_StartMeasurementDMA() != 0U)
            {
                /* P38: a DMA command being accepted proves only that the host
                 * transaction started.  Recovery is confirmed only after a
                 * real validated RX distance sample is published. */
                lidar_recovery_active = 0U;
                lidar_recovery_step_state = LIDAR_RECOVERY_STEP_IDLE;
                lidar_recovery_step = LIDAR_RECOVERY_STEP_IDLE;
                lidar_profile_config_ok = 1U;
                Lidar_BeginFirstSampleGrace(1U);
            }
            else if (lidar_recovery_attempt_in_cycle < LIDAR_RECOVERY_MAX_ATTEMPTS)
            {
                lidar_recovery_step_state = LIDAR_RECOVERY_STEP_QUIESCE;
                lidar_recovery_due_ms = HAL_GetTick() + LIDAR_RUNTIME_RETRY_MS;
            }
            else
            {
                lidar_recovery_failure_count++;
                Lidar_RecordRecoveryDuration();
                lidar_recovery_attempt_in_cycle = 0U;
                lidar_recovery_step_state = LIDAR_RECOVERY_STEP_QUIESCE;
                lidar_recovery_due_ms = HAL_GetTick() + LIDAR_RETRY_INTERVAL_MS;
            }
            break;

        default:
            lidar_recovery_step_state = LIDAR_RECOVERY_STEP_QUIESCE;
            break;
    }

    lidar_recovery_step = lidar_recovery_step_state;
    step_us = micros() - step_start_us;
    if (step_us > lidar_recovery_step_max_us)
    {
        lidar_recovery_step_max_us = step_us;
    }
}

static uint8_t Lidar_TryConnect(void)
{
    /* P37: do not use HAL_I2C_IsDeviceReady; it can block for tens of ms.
     * The acquisition DMA command itself is the probe. */
    lidar_probe_attempt_count++;
    lidar_data.connected = 1U;
    lidar_data.state = LIDAR_STATE_IDLE;

    if (Lidar_StartMeasurementDMA() != 0U)
    {
        lidar_probe_success_count++;
        lidar_reconnect_count++;
        lidar_profile_config_ok = 1U;
        Lidar_BeginFirstSampleGrace(0U);
        return 1U;
    }

    lidar_probe_failure_count++;
    Lidar_RequestRecovery(LIDAR_RUNTIME_RETRY_MS);
    return 0U;
}

static uint8_t Lidar_InitFilter(void)
{
#if (APP_LIDAR_FILTER_ENABLED != 0U)
    uint8_t ok = Butterworth2LPF_Init(&lidar_distance_filter,
        APP_LIDAR_FILTER_SAMPLE_RATE_HZ, APP_LIDAR_LPF_CUTOFF_HZ);
    lidar_data.filter_enabled = 1U;
    lidar_data.filter_config_ok = ok;
    lidar_filter_b0 = lidar_distance_filter.b0;
    lidar_filter_b1 = lidar_distance_filter.b1;
    lidar_filter_b2 = lidar_distance_filter.b2;
    lidar_filter_a1 = lidar_distance_filter.a1;
    lidar_filter_a2 = lidar_distance_filter.a2;
    return ok;
#else
    lidar_data.filter_enabled = 0U;
    lidar_data.filter_config_ok = 1U;
    return 1U;
#endif
}

static uint16_t Lidar_MedianOfThree(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t t;
    if (a > b) { t=a; a=b; b=t; }
    if (b > c) { t=b; b=c; c=t; }
    if (a > b) { t=a; a=b; b=t; }
    return b;
}

static uint16_t Lidar_UpdateMedian(uint16_t value)
{
    uint8_t i;
    if (lidar_data.median_initialized == 0U)
    {
        for (i=0U; i<LIDAR_MEDIAN_WINDOW_SIZE; i++) lidar_median_window[i]=value;
        lidar_data.median_initialized=1U;
        lidar_median_index=0U;
    }
    else
    {
        lidar_median_window[lidar_median_index]=value;
        lidar_median_index=(uint8_t)((lidar_median_index+1U)%LIDAR_MEDIAN_WINDOW_SIZE);
    }
    return Lidar_MedianOfThree(lidar_median_window[0], lidar_median_window[1], lidar_median_window[2]);
}

static void Lidar_SaveSampleInterval(uint32_t now_us)
{
    uint32_t dt;
    if (lidar_data.last_sample_timestamp_us == 0UL) return;
    dt=now_us-lidar_data.last_sample_timestamp_us;
    lidar_data.last_sample_interval_us=dt;
    if ((lidar_data.min_sample_interval_us==0UL)||(dt<lidar_data.min_sample_interval_us)) lidar_data.min_sample_interval_us=dt;
    if (dt>lidar_data.max_sample_interval_us) lidar_data.max_sample_interval_us=dt;
}

static void Lidar_UpdateFilter(float value, uint32_t now_us)
{
#if (APP_LIDAR_FILTER_ENABLED != 0U)
    uint32_t gap=now_us-lidar_data.last_sample_timestamp_us;
    if ((lidar_data.filter_enabled==0U)||(lidar_data.filter_config_ok==0U))
    {
        lidar_data.filtered_distance_m=value;
        return;
    }

    if ((lidar_data.filter_initialized==0U)||(lidar_data.last_sample_timestamp_us==0UL)||(gap>APP_LIDAR_FILTER_RESET_GAP_US))
    {
        Butterworth2LPF_Reset(&lidar_distance_filter,value);
        lidar_data.filtered_distance_m=value;
        lidar_data.filter_initialized=1U;
        lidar_data.filter_reset_count++;
    }
    else lidar_data.filtered_distance_m=Butterworth2LPF_Process(&lidar_distance_filter,value);
    lidar_data.filter_update_count++;
#else
    (void)now_us;
    lidar_data.filtered_distance_m=value;
#endif
}

static uint8_t Lidar_StartMeasurementDMA(void)
{
    HAL_StatusTypeDef st;
    uint32_t now_us = micros();

    if (lidar_v819_last_trigger_us != 0UL)
    {
        uint32_t interval_us = now_us - lidar_v819_last_trigger_us;
        lidar_last_trigger_interval_us = interval_us;
        if ((lidar_min_trigger_interval_us == 0UL) ||
            (interval_us < lidar_min_trigger_interval_us))
        {
            lidar_min_trigger_interval_us = interval_us;
        }
        if (interval_us > lidar_max_trigger_interval_us)
        {
            lidar_max_trigger_interval_us = interval_us;
        }
    }
    lidar_v819_last_trigger_us = now_us;

    /* P112R12R8R5: STM32F4 HAL Mem_*_DMA can spin on I2C_FLAG_BUSY before
     * it ever starts DMA. Never enter that bounded-but-long HAL wait from the
     * cooperative scheduler. Existing WAIT_BUSY watchdog/recovery will service
     * a genuinely stuck bus non-blockingly. */
    if ((HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) ||
        (__HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY) != RESET))
    {
        lidar_data.last_hal_status = (uint8_t)HAL_BUSY;
        lidar_data.dma_busy_count++;
        return 0U;
    }

    st=HAL_I2C_Mem_Write_DMA(&hi2c2,LIDAR_I2C_ADDRESS,LIDAR_REG_ACQ_COMMAND,
        I2C_MEMADD_SIZE_8BIT,&lidar_tx_value,1U);
    lidar_data.last_hal_status=(uint8_t)st;
    if (st==HAL_OK)
    {
        lidar_data.state=LIDAR_STATE_TX_START_DMA;
        lidar_data.dma_tx_start_count++;
        lidar_bias_command_count++;
        lidar_dma_operation_start_ms=HAL_GetTick();
        return 1U;
    }
    if (st==HAL_BUSY) lidar_data.dma_busy_count++;
    else { lidar_data.error_count++; lidar_data.dma_error_count++; }
    return 0U;
}

static uint8_t Lidar_StartReadDMA(void)
{
    HAL_StatusTypeDef st;

    /* Same non-blocking BUSY preflight as the acquisition-command path. */
    if ((HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) ||
        (__HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY) != RESET))
    {
        lidar_data.last_hal_status = (uint8_t)HAL_BUSY;
        lidar_data.dma_busy_count++;
        return 0U;
    }

    st=HAL_I2C_Mem_Read_DMA(&hi2c2,LIDAR_I2C_ADDRESS,LIDAR_REG_DISTANCE_AUTO,
        I2C_MEMADD_SIZE_8BIT,lidar_rx_dma_buffer,2U);
    lidar_data.last_hal_status=(uint8_t)st;
    if (st==HAL_OK)
    {
        lidar_data.state=LIDAR_STATE_RX_DISTANCE_DMA;
        lidar_data.dma_rx_start_count++;
        lidar_dma_operation_start_ms=HAL_GetTick();
        return 1U;
    }
    if (st==HAL_BUSY) lidar_data.dma_busy_count++;
    else { lidar_data.error_count++; lidar_data.dma_error_count++; }
    return 0U;
}

static float Lidar_FastSqrtApprox(float value)
{
    float guess;

    if (value <= 0.0f)
    {
        return 0.0f;
    }

    guess = value;
    for (uint8_t i = 0U; i < 6U; i++)
    {
        guess = 0.5f * (guess + (value / guess));
    }

    return guess;
}

static void Lidar_ResetCalibrationWindow(void)
{
    if (lidar_calibration_sample_count_state != 0U)
    {
        lidar_calibration_reset_count++;
    }

    lidar_calibration_sample_count_state = 0U;
    lidar_calibration_sum_m = 0.0f;
    lidar_calibration_sum_square_m = 0.0f;
    lidar_calibration_last_m = 0.0f;
}

static uint8_t Lidar_UpdateStartupCalibration(float median_raw_distance_m)
{
#if (APP_LIDAR_STARTUP_CALIBRATION_ENABLED != 0U)
    float reference_error;
    float sample_jump;

    if (lidar_calibration_complete_state != 0U)
    {
        return 0U;
    }

    reference_error =
        median_raw_distance_m - APP_LIDAR_CALIBRATION_REFERENCE_M;
    if (reference_error < 0.0f)
    {
        reference_error = -reference_error;
    }

    sample_jump = median_raw_distance_m - lidar_calibration_last_m;
    if (sample_jump < 0.0f)
    {
        sample_jump = -sample_jump;
    }

    if ((reference_error > APP_LIDAR_CALIBRATION_MAX_ERROR_M) ||
        ((lidar_calibration_sample_count_state != 0U) &&
         (sample_jump > APP_LIDAR_CALIBRATION_MAX_JUMP_M)))
    {
        lidar_calibration_last_reject_reason = 1U;
        Lidar_ResetCalibrationWindow();
        return 0U;
    }

    lidar_calibration_sum_m += median_raw_distance_m;
    lidar_calibration_sum_square_m +=
        median_raw_distance_m * median_raw_distance_m;
    lidar_calibration_last_m = median_raw_distance_m;
    lidar_calibration_sample_count_state++;

    if (lidar_calibration_sample_count_state >=
        APP_LIDAR_CALIBRATION_SAMPLE_COUNT)
    {
        const float sample_count =
            (float)lidar_calibration_sample_count_state;
        const float average_distance_m =
            lidar_calibration_sum_m / sample_count;
        float variance_m2 =
            (lidar_calibration_sum_square_m / sample_count) -
            (average_distance_m * average_distance_m);

        if (variance_m2 < 0.0f)
        {
            variance_m2 = 0.0f;
        }

        lidar_calibration_stddev_m = Lidar_FastSqrtApprox(variance_m2);

        if (lidar_calibration_stddev_m >
            APP_LIDAR_CALIBRATION_STDDEV_MAX_M)
        {
            lidar_calibration_last_reject_reason = 2U;
            Lidar_ResetCalibrationWindow();
            return 0U;
        }

        lidar_calibration_offset_m_state =
            average_distance_m - APP_LIDAR_CALIBRATION_REFERENCE_M;
        lidar_calibration_complete_state = 1U;
        lidar_calibration_last_reject_reason = 0U;

        /* Corrected data starts with a fresh filter state. */
        lidar_data.filter_initialized = 0U;
        return 1U;
    }
#else
    (void)median_raw_distance_m;
    lidar_calibration_complete_state = 1U;
#endif

    return 0U;
}

static void Lidar_ProcessPendingSample(void)
{
    uint16_t raw;
    uint16_t med;
    uint32_t now_us;
    float raw_distance_m;
    float median_raw_distance_m;
    float published_distance_m;
    float published_median_distance_m;

    if (lidar_sample_pending == 0U)
    {
        return;
    }

    __disable_irq();
    lidar_last_high_byte = lidar_pending_high;
    lidar_last_low_byte = lidar_pending_low;
    lidar_sample_pending = 0U;
    __enable_irq();

    raw =
        ((uint16_t)lidar_last_high_byte << 8) |
        (uint16_t)lidar_last_low_byte;

    lidar_data.read_count++;

    if ((raw < LIDAR_MIN_DISTANCE_CM) ||
        (raw > LIDAR_MAX_DISTANCE_CM))
    {
        lidar_data.distance_valid = 0U;
        lidar_data.error_count++;
        return;
    }

    now_us = micros();
    raw_distance_m = (float)raw * 0.01f;
    med = Lidar_UpdateMedian(raw);
    median_raw_distance_m = (float)med * 0.01f;

    lidar_raw_distance_cm = raw;
    lidar_raw_distance_m = raw_distance_m;

    (void)Lidar_UpdateStartupCalibration(median_raw_distance_m);

    /*
     * Before calibration completes, publish the real sensor distance unchanged.
     * This keeps startup and debugger data valid. Once complete, apply the
     * fixed 9 m reference offset to both direct and median values.
     */
    if (lidar_calibration_complete_state != 0U)
    {
        published_distance_m =
            raw_distance_m - lidar_calibration_offset_m_state;
        published_median_distance_m =
            median_raw_distance_m - lidar_calibration_offset_m_state;
    }
    else
    {
        published_distance_m = raw_distance_m;
        published_median_distance_m = median_raw_distance_m;
    }

    if ((published_distance_m < 0.0f) ||
        (published_distance_m > ((float)LIDAR_MAX_DISTANCE_CM * 0.01f)) ||
        (published_median_distance_m < 0.0f) ||
        (published_median_distance_m >
         ((float)LIDAR_MAX_DISTANCE_CM * 0.01f)))
    {
        lidar_data.distance_valid = 0U;
        lidar_data.error_count++;
        return;
    }

    lidar_data.distance_m = published_distance_m;
    lidar_data.distance_cm =
        (uint16_t)((published_distance_m * 100.0f) + 0.5f);
    lidar_data.median_distance_m = published_median_distance_m;
    lidar_data.median_distance_cm =
        (uint16_t)((published_median_distance_m * 100.0f) + 0.5f);

    Lidar_SaveSampleInterval(now_us);
    Lidar_UpdateFilter(lidar_data.median_distance_m, now_us);
    lidar_data.last_sample_timestamp_us = now_us;
    lidar_data.distance_valid = 1U;
    lidar_data.data_ready = 1U;
    lidar_data.connected = 1U;
    lidar_data.update_count++;
    lidar_data.last_measure_duration_ms =
        HAL_GetTick() - lidar_data.measurement_start_ms;

    /* P38: only a validated physical distance sample re-arms freshness. */
    lidar_watchdog_armed = 1U;
    lidar_first_sample_deadline_ms = 0UL;
    if (lidar_recovery_confirmation_pending != 0U)
    {
        lidar_recovery_confirmation_pending = 0U;
        lidar_recovery_success_count++;
        Lidar_RecordRecoveryDuration();
    }
}

void Lidar_Init(void)
{
    Lidar_ResetData();
    (void)Lidar_InitFilter();

    /*
     * Driver initialization is successful even when the sensor has not yet
     * completed its own power-up. Lidar_Update will probe and reconnect.
     */
    lidar_data.initialized = 1U;
    lidar_data.connected = 0U;
    lidar_data.state = LIDAR_STATE_ERROR;

    Lidar_ScheduleRetry(LIDAR_STARTUP_DELAY_MS);
    Lidar_UpdateLiveDebug();
}

void Lidar_Update(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t recovery_started = 0U;

    if (lidar_data.initialized == 0U)
    {
        return;
    }

    if (lidar_recovery_active != 0U)
    {
        Lidar_ServiceRecovery();
        Lidar_UpdateLiveDebug();
        return;
    }

    /*
     * A failed startup probe no longer disables the LIDAR permanently.
     * Retry at a low rate until the sensor acknowledges.
     */
    if (lidar_data.connected == 0U)
    {
        if (Lidar_TimeReached(now_ms, lidar_next_retry_ms) != 0U)
        {
            /* Lidar_TryConnect() already starts exactly one measurement DMA.
             * P37 started a second DMA here and could manufacture HAL_BUSY. */
            (void)Lidar_TryConnect();
        }

        Lidar_UpdateLiveDebug();
        return;
    }

    if (lidar_data.state == LIDAR_STATE_ERROR)
    {
        if (Lidar_TimeReached(now_ms, lidar_next_retry_ms) != 0U)
        {
            lidar_data.state = LIDAR_STATE_IDLE;

            if (Lidar_StartMeasurementDMA() == 0U)
            {
                Lidar_ScheduleRetry(LIDAR_RETRY_INTERVAL_MS);
            }
            else
            {
                Lidar_BeginFirstSampleGrace(0U);
            }
        }

        Lidar_UpdateLiveDebug();
        return;
    }

    Lidar_ProcessPendingSample();

    /* P38 recovery/startup grace: while waiting for the first validated sample
     * the stale watchdog is deliberately disarmed.  If no real sample arrives
     * within the bounded grace window, start another non-blocking recovery. */
    if ((lidar_watchdog_armed == 0U) &&
        (lidar_first_sample_deadline_ms != 0UL) &&
        (Lidar_TimeReached(now_ms, lidar_first_sample_deadline_ms) != 0U))
    {
        if (lidar_recovery_confirmation_pending != 0U)
        {
            lidar_recovery_confirmation_pending = 0U;
            lidar_recovery_failure_count++;
            Lidar_RecordRecoveryDuration();
        }
        lidar_data.timeout_count++;
        lidar_data.error_count++;
        lidar_data.distance_valid = 0U;
        Lidar_RequestRecovery(LIDAR_RUNTIME_RETRY_MS);
        recovery_started = 1U;
    }

    /* A completed DMA path can still freeze without raising an I2C callback.
     * Once at least one real sample exists, contain that silent freeze here
     * and keep reconnecting forever without blocking the scheduler. */
    if ((recovery_started == 0U) &&
        (lidar_watchdog_armed != 0U) &&
        (lidar_data.last_sample_timestamp_us != 0UL) &&
        ((uint32_t)(micros() - lidar_data.last_sample_timestamp_us) >
         (LIDAR_SAMPLE_WATCHDOG_MS * 1000UL)))
    {
        lidar_data.timeout_count++;
        lidar_data.error_count++;
        lidar_data.distance_valid = 0U;
        Lidar_RequestRecovery(LIDAR_RUNTIME_RETRY_MS);
        recovery_started = 1U;
    }

    if ((recovery_started == 0U) &&
        (lidar_data.state == LIDAR_STATE_WAIT_MEASUREMENT) &&
        ((now_ms - lidar_data.measurement_start_ms) >=
         LIDAR_MEASUREMENT_TIME_MS))
    {
        if ((Lidar_StartReadDMA() == 0U) &&
            ((now_ms - lidar_data.measurement_start_ms) >=
             LIDAR_WAIT_BUSY_TIMEOUT_MS))
        {
            /* Covers the previously unbounded WAIT_MEASUREMENT/HAL_BUSY
             * condition observed on the powered RCS test. */
            lidar_data.wait_busy_timeout_count++;
            lidar_data.timeout_count++;
            lidar_data.error_count++;
            lidar_data.distance_valid = 0U;
            Lidar_RequestRecovery(LIDAR_RUNTIME_RETRY_MS);
        }
    }
    else if ((recovery_started == 0U) &&
             (lidar_data.state == LIDAR_STATE_IDLE) &&
             (lidar_sample_pending == 0U))
    {
        (void)Lidar_StartMeasurementDMA();
    }
    else if ((recovery_started == 0U) &&
             ((lidar_data.state == LIDAR_STATE_TX_START_DMA) ||
              (lidar_data.state == LIDAR_STATE_RX_DISTANCE_DMA)) &&
             ((now_ms - lidar_dma_operation_start_ms) >
              LIDAR_DMA_TIMEOUT_MS))
    {
        lidar_data.timeout_count++;
        lidar_data.error_count++;

        Lidar_RequestRecovery(LIDAR_RUNTIME_RETRY_MS);
    }

    Lidar_UpdateLiveDebug();
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance!=I2C2) return;
    lidar_data.dma_tx_complete_count++;
    lidar_data.connected = 1U;
    lidar_data.measurement_start_ms=HAL_GetTick();
    lidar_data.state=LIDAR_STATE_WAIT_MEASUREMENT;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance!=I2C2) return;
    if (lidar_sample_pending != 0U)
    {
        lidar_sample_pending_overrun_count++;
    }
    lidar_pending_high=lidar_rx_dma_buffer[0];
    lidar_pending_low=lidar_rx_dma_buffer[1];
    lidar_sample_pending=1U;
    lidar_data.dma_rx_complete_count++;
    /* Start the next conversion immediately; keeps approximately 200 Hz cadence. */
    lidar_data.state=LIDAR_STATE_IDLE;
    (void)Lidar_StartMeasurementDMA();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C2)
    {
        return;
    }

    lidar_data.last_hal_status = (uint8_t)HAL_ERROR;
    lidar_last_i2c_error_code = HAL_I2C_GetError(hi2c);

    lidar_data.error_count++;
    lidar_data.dma_error_count++;

    lidar_sample_pending = 0U;
    lidar_profile_config_ok = 0U;
    Lidar_RequestRecovery(LIDAR_RUNTIME_RETRY_MS);
}

uint8_t Lidar_IsConnected(void){return lidar_data.connected;}
uint8_t Lidar_IsDataReady(void){return lidar_data.data_ready;}
uint8_t Lidar_IsDistanceValid(void){return lidar_data.distance_valid;}
uint16_t Lidar_GetDistanceCm(void){return lidar_data.distance_cm;}
float Lidar_GetDistanceM(void){return lidar_data.distance_m;}
float Lidar_GetMedianDistanceM(void){return lidar_data.median_distance_m;}
float Lidar_GetFilteredDistanceM(void){return lidar_data.filtered_distance_m;}
LidarData_t Lidar_GetData(void){return lidar_data;}
const LidarData_t *Lidar_GetDataPtr(void){return &lidar_data;}
