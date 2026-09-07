#include "Modules/Sensors/IMU/imu.h"

#include "Common/app_config.h"

#include "Platform/board_handles.h"
#include "Platform/board_pins.h"
#include "Services/Timebase/timebase.h"

#include "main.h"

/* -------------------------------------------------------------------------- */
/* ISM330DLC Registers                                                        */
/* -------------------------------------------------------------------------- */

#define IMU_REG_WHO_AM_I              0x0F
#define IMU_REG_CTRL1_XL              0x10
#define IMU_REG_CTRL2_G               0x11
#define IMU_REG_CTRL3_C               0x12
#define IMU_REG_CTRL4_C               0x13
#define IMU_REG_STATUS_REG            0x1E
#define IMU_REG_OUTX_L_G              0x22

/* Config: ODR 1.66 kHz, accel +/-8 g, gyro +/-1000 dps */
#define IMU_CTRL1_XL_1666HZ_8G         0x8CU
#define IMU_CTRL2_G_1666HZ_1000DPS     0x88U

#define IMU_CTRL3_C_SW_RESET            0x01U
#define IMU_CTRL3_C_BDU_IF_INC          0x44U
#define IMU_CTRL4_C_I2C_DISABLE          0x04U
#define IMU_STATUS_XLDA                 0x01U
#define IMU_STATUS_GDA                  0x02U

/* -------------------------------------------------------------------------- */
/* SPI / DMA                                                                  */
/* -------------------------------------------------------------------------- */

#define IMU_SPI_TIMEOUT_MS            20U

#define IMU_DMA_RAW_LENGTH            13U
#define IMU_DMA_MAX_LENGTH            33U

#define IMU_DMA_TRANSFER_NONE         0U
#define IMU_DMA_TRANSFER_RAW          1U
#define IMU_DMA_TRANSFER_BLOCKING     2U

#define IMU_REDUNDANT_BURST_COUNT     3U
#define IMU_RAW_AXIS_COUNT            6U
#define IMU_BIT8_MASK                 0x0100U
#define IMU_REDUNDANT_GYRO_RESIDUAL_MAX_RAW  512L
#define IMU_REDUNDANT_ACCEL_RESIDUAL_MAX_RAW 1024L
#define IMU_CONTINUITY_SWITCH_MARGIN_RAW      24L

/* Runtime data-integrity watchdog. Values are deliberately time based so the
 * protection remains valid if the task frequency changes.  P39 aligns exact
 * six-axis repeat detection with the 20 ms flight freshness limit; a frozen
 * bus sample can no longer be propagated as valid for ~100 ms. */
#define IMU_STALE_TIMEOUT_US                   20000UL
#define IMU_REGISTER_CHECK_INTERVAL_US       1000000UL
#define IMU_RECOVERY_RETRY_DELAY_US           500000UL
#define IMU_RECOVERY_INVALID_LIMIT            10UL
#define IMU_RECOVERY_VALID_SAMPLE_COUNT       20U
#define IMU_REPEATED_WORD_MIN_ABS_RAW         128U

/* P55 repeated-pattern fast configuration repair result. */
#define IMU_FAST_CONFIG_REPAIR_CONTINUE         1U
#define IMU_FAST_CONFIG_REPAIR_ESCALATE         0U
/* P56/P57 stale-config repair can defer acquisition while a newly enabled
 * 1.66 kHz ODR is warming up. P57 waits non-blockingly for both accel and
 * gyro DATA_READY instead of judging the sensor after one fixed 1 ms tick. */
#define IMU_FAST_CONFIG_REPAIR_DEFER             2U
#define IMU_STALE_WARMUP_TIMEOUT_US          40000UL

/* P36 runtime recovery: no HAL_Delay is permitted after App_Init. */
#define IMU_RECOVERY_MAX_ATTEMPTS              3U
#define IMU_RECOVERY_RESET_TIMEOUT_US          50000UL
#define IMU_RECOVERY_RESET_POLL_US             1000UL
#define IMU_RECOVERY_SETTLE_US                 40000UL

#define IMU_RECOVERY_STEP_IDLE                 0U
#define IMU_RECOVERY_STEP_BUS_RESET            1U
#define IMU_RECOVERY_STEP_WHOAMI               2U
#define IMU_RECOVERY_STEP_RESET_WRITE          3U
#define IMU_RECOVERY_STEP_RESET_WAIT           4U
#define IMU_RECOVERY_STEP_CONFIGURE            5U
#define IMU_RECOVERY_STEP_SETTLE_WAIT          6U
#define IMU_RECOVERY_STEP_VERIFY               7U

#define IMU_CS_LOW()                  HAL_GPIO_WritePin(BOARD_IMU_CS_PORT, BOARD_IMU_CS_PIN, GPIO_PIN_RESET)
#define IMU_CS_HIGH()                 HAL_GPIO_WritePin(BOARD_IMU_CS_PORT, BOARD_IMU_CS_PIN, GPIO_PIN_SET)

/* -------------------------------------------------------------------------- */
/* Private data                                                               */
/* -------------------------------------------------------------------------- */

static uint8_t imu_initialized = 0U;
static uint8_t imu_connected = 0U;
static uint8_t imu_device_id = 0U;

static IMU_RawData_t imu_cached_raw;

/*
 * imu_last_sample_valid describes the newest completed RAW DMA packet.
 * When a corrupt all-zero packet is rejected, imu_cached_raw is preserved.
 */
static volatile uint8_t imu_last_sample_valid = 0U;
static volatile uint32_t imu_last_sample_timestamp_us = 0UL;
static volatile uint32_t imu_sample_generation = 0UL;

/* The observed bus fault toggles bit 8 of individual 16-bit output words.
 * Three complete DMA bursts plus temporal continuity are used to validate
 * every sample before it reaches calibration or estimation. */
static uint8_t imu_continuity_initialized = 0U;
static IMU_RawData_t imu_previous_raw;
static IMU_RawData_t imu_previous_previous_raw;

static uint8_t imu_dma_tx[IMU_DMA_MAX_LENGTH];
static uint8_t imu_dma_rx[IMU_DMA_MAX_LENGTH];

static volatile uint8_t imu_dma_busy = 0U;
static volatile uint8_t imu_dma_done = 0U;
static volatile uint8_t imu_dma_error = 0U;
static volatile uint8_t imu_dma_transfer_type = IMU_DMA_TRANSFER_NONE;

/* Runtime watchdog/recovery state. */
static uint8_t imu_stale_reference_initialized = 0U;
static IMU_RawData_t imu_stale_reference_raw;
static uint32_t imu_last_raw_change_timestamp_us = 0UL;
static uint32_t imu_last_register_check_timestamp_us = 0UL;
static uint32_t imu_recovery_retry_timestamp_us = 0UL;
static uint8_t imu_recovery_valid_streak = 0U;
static uint8_t imu_recovery_step = IMU_RECOVERY_STEP_IDLE;
static uint8_t imu_recovery_attempt_in_cycle = 0U;
static uint32_t imu_recovery_step_deadline_us = 0UL;
static uint32_t imu_recovery_next_poll_us = 0UL;
static uint32_t imu_recovery_start_timestamp_us = 0UL;
static uint8_t imu_last_commit_pattern_error = 0U;
/* P44: a repeated-word triplet is quarantined until the next 1 ms IMU task.
 * Before arming that retry, the SPI transaction boundary is re-synchronized
 * without resetting the sensor or blocking the scheduler. */
static uint8_t imu_pattern_retry_pending = 0U;
/* P54: after the first bad triplet, snapshot the configuration registers on
 * the next 1 kHz service call before the one allowed deferred retry. */
static uint8_t imu_pattern_register_snapshot_pending = 0U;

/* P56/P57: an exact-six-axis stale episode first gets a register-only repair.
 * P57 then enters a bounded, non-blocking STATUS_REG warmup gate. The IMU task
 * polls XLDA|GDA once per scheduler call and only retries the raw triplet once
 * fresh accel+gyro output is advertised. A 40 ms timeout retains the existing
 * full reset recovery as the final safety net. */
static uint8_t imu_stale_retry_pending = 0U;
static uint8_t imu_stale_register_snapshot_pending = 0U;
static uint8_t imu_stale_warmup_active = 0U;
static uint8_t imu_stale_warmup_first_ready_recorded = 0U;
static uint32_t imu_stale_warmup_start_us = 0UL;

/* P53/P54: persist the most recent repeated-word event so the bench UART can show
 * the exact three raw SPI bursts and the corrected word pattern that triggered
 * quarantine. This does not participate in flight decisions. */
static IMU_PatternDiagnostic_t imu_pattern_diagnostic;
static IMU_StaleDiagnostic_t imu_stale_diagnostic;

/* Forward declarations used by the recovery service. */
static uint8_t IMU_ReadReg(uint8_t reg);
static void IMU_WriteReg(uint8_t reg, uint8_t value);
static uint8_t IMU_ResetAndConfigure(void);
static uint8_t IMU_CheckRuntimeRegisters(void);
static void IMU_ServiceRecovery(uint32_t now_us);
static void IMU_StartRecovery(uint32_t now_us);
static void IMU_RecoveryAttemptFailed(uint32_t now_us);
static uint8_t IMU_ApplySPIProfile(uint32_t polarity, uint32_t phase, uint32_t prescaler);
static uint8_t IMU_AutoProbeSPI(void);
static void IMU_SelectRuntimeSPI(void);
static void IMU_SW_SPI_ConfigureGPIO(void);
static void IMU_SPI_ResynchronizeBus(void);
static uint8_t IMU_ServicePatternFastConfigRepair(void);
static uint8_t IMU_ServiceStaleFastConfigRepair(void);
static HAL_StatusTypeDef IMU_SW_SPI_Transfer(uint8_t *tx, uint8_t *rx, uint16_t length);
static uint8_t IMU_SW_SPI_EnableIfPresent(void);

/* -------------------------------------------------------------------------- */
/* Live Expressions                                                           */
/* -------------------------------------------------------------------------- */

volatile uint8_t imu_drv_initialized = 0U;
volatile uint8_t imu_drv_connected = 0U;
volatile uint8_t imu_drv_device_id = 0U;
volatile uint8_t imu_drv_last_whoami = 0U;

volatile uint32_t imu_drv_error_count = 0UL;
volatile uint32_t imu_drv_read_count = 0UL;
volatile uint32_t imu_drv_write_count = 0UL;

volatile uint8_t imu_drv_last_reg = 0U;
volatile uint8_t imu_drv_last_value = 0U;

volatile uint8_t imu_drv_ctrl1_xl_readback = 0U;
volatile uint8_t imu_drv_ctrl2_g_readback = 0U;
volatile uint8_t imu_drv_ctrl3_c_readback = 0U;
volatile uint8_t imu_drv_ctrl4_c_readback = 0U;
volatile uint8_t imu_drv_i2c_disable_ok = 0U;
volatile uint8_t imu_drv_status_reg = 0U;
volatile uint8_t imu_drv_fullscale_config_ok = 0U;
volatile uint8_t imu_drv_bdu_if_inc_ok = 0U;
volatile uint32_t imu_drv_reset_count = 0UL;
volatile uint32_t imu_drv_config_retry_count = 0UL;

volatile uint8_t imu_drv_last_hal_status = 0U;
volatile uint8_t imu_drv_last_rx0 = 0U;
volatile uint8_t imu_drv_last_rx1 = 0U;

/* V41: SPI1 startup auto-probe diagnostics.
 * mode: 3 = CPOL1/CPHA1, 0 = CPOL0/CPHA0
 * prescaler_code: 32/64/128/256 (human-readable divider)
 */
volatile uint8_t imu_spi_profile_found = 0U;
volatile uint8_t imu_spi_mode_selected = 0xFFU;
volatile uint16_t imu_spi_prescaler_selected = 0U;
volatile uint32_t imu_spi_probe_attempt_count = 0UL;
volatile uint32_t imu_spi_poll_read_count = 0UL;
volatile uint32_t imu_spi_poll_error_count = 0UL;

/* V42: physical-line / bit-bang diagnostics.
 * These are intentionally diagnostic-only and never feed the flight estimator. */
volatile uint8_t imu_phy_diag_done = 0U;
volatile uint8_t imu_phy_cs_high_read = 0U;
volatile uint8_t imu_phy_cs_low_read = 0U;
volatile uint8_t imu_phy_sck_high_read = 0U;
volatile uint8_t imu_phy_sck_low_read = 0U;
volatile uint8_t imu_phy_mosi_high_read = 0U;
volatile uint8_t imu_phy_mosi_low_read = 0U;
volatile uint8_t imu_phy_miso_pullup_read = 0U;
volatile uint8_t imu_phy_miso_pulldown_read = 0U;
volatile uint8_t imu_phy_bitbang_mode0_who = 0U;
volatile uint8_t imu_phy_bitbang_mode3_who = 0U;
volatile uint8_t imu_phy_gpio_drive_ok = 0U;
volatile uint8_t imu_phy_miso_free = 0U;
volatile uint8_t imu_phy_spi_response_ok = 0U;

/* V43: bit-bang register map + direct SPI1 peripheral diagnostics.
 * These values are diagnostic only; they do not alter estimator/control data. */
volatile uint8_t imu_v43_bb_selected_mode = 0xFFU;
volatile uint8_t imu_v43_bb_who = 0U;
volatile uint8_t imu_v43_bb_ctrl1_xl = 0U;
volatile uint8_t imu_v43_bb_ctrl2_g = 0U;
volatile uint8_t imu_v43_bb_ctrl3_c = 0U;
volatile uint8_t imu_v43_bb_ctrl4_c = 0U;
volatile uint8_t imu_v43_bb_status = 0U;
volatile uint8_t imu_v43_direct_mode0_who = 0U;
volatile uint8_t imu_v43_direct_mode3_who = 0U;
volatile uint8_t imu_v43_direct_response_ok = 0U;
volatile uint32_t imu_v43_direct_transfer_count = 0UL;
volatile uint32_t imu_v43_direct_error_count = 0UL;
volatile uint32_t imu_v43_reinit_attempt_count = 0UL;

/* V44: software-SPI fallback.
 * V43 proved the IMU responds to GPIO bit-bang while the STM32 SPI1 peripheral
 * reads 0x00. Only the IMU bus uses this fallback; no other peripheral changes.
 */
volatile uint8_t imu_swspi_active = 0U;
volatile uint8_t imu_swspi_whoami = 0U;
volatile uint32_t imu_swspi_probe_count = 0UL;
volatile uint32_t imu_swspi_transfer_count = 0UL;
volatile uint32_t imu_swspi_error_count = 0UL;

static uint32_t imu_spi_selected_polarity = SPI_POLARITY_HIGH;
static uint32_t imu_spi_selected_phase = SPI_PHASE_2EDGE;

volatile uint32_t imu_dma_start_count = 0UL;
volatile uint32_t imu_dma_complete_count = 0UL;
volatile uint32_t imu_dma_error_count = 0UL;
volatile uint32_t imu_dma_timeout_count = 0UL;
volatile uint32_t imu_dma_busy_count = 0UL;

/* Logger/Live Expressions diagnostics requested for the IMU watchdog. */
volatile uint8_t imu_sample_valid = 0U;
volatile uint32_t imu_stale_count = 0UL;
volatile uint32_t imu_pattern_error_count = 0UL;
volatile uint32_t imu_pattern_retry_count = 0UL;
volatile uint32_t imu_pattern_retry_success_count = 0UL;
volatile uint32_t imu_pattern_recovery_escalation_count = 0UL;
/* P55: repeated-word fast configuration repair diagnostics. */
volatile uint32_t imu_fast_config_check_count = 0UL;
volatile uint32_t imu_fast_config_repair_attempt_count = 0UL;
volatile uint32_t imu_fast_config_repair_success_count = 0UL;
volatile uint32_t imu_fast_config_repair_failure_count = 0UL;
volatile uint32_t imu_fast_config_repair_last_duration_us = 0UL;
volatile uint32_t imu_fast_config_repair_max_duration_us = 0UL;
/* P56: stale short-path diagnostics. */
volatile uint32_t imu_stale_fast_config_check_count = 0UL;
volatile uint32_t imu_stale_fast_config_repair_attempt_count = 0UL;
volatile uint32_t imu_stale_fast_config_repair_success_count = 0UL;
volatile uint32_t imu_stale_fast_config_repair_failure_count = 0UL;
volatile uint32_t imu_stale_retry_success_count = 0UL;
volatile uint32_t imu_stale_recovery_escalation_count = 0UL;
volatile uint32_t imu_stale_fast_config_repair_last_duration_us = 0UL;
volatile uint32_t imu_stale_fast_config_repair_max_duration_us = 0UL;
/* P57: non-blocking stale ODR warmup / DATA_READY diagnostics. */
volatile uint32_t imu_stale_warmup_event_count = 0UL;
volatile uint32_t imu_stale_warmup_poll_count = 0UL;
volatile uint32_t imu_stale_warmup_success_count = 0UL;
volatile uint32_t imu_stale_warmup_timeout_count = 0UL;
volatile uint32_t imu_stale_warmup_first_ready_us = 0UL;
volatile uint32_t imu_stale_warmup_max_ready_us = 0UL;
volatile uint8_t imu_stale_warmup_last_status = 0U;
volatile uint8_t imu_stale_warmup_active_debug = 0U;
volatile uint32_t imu_recovery_count = 0UL;
volatile uint8_t imu_recovery_state = IMU_RECOVERY_STATE_NORMAL;
volatile uint8_t imu_recovery_step_debug = IMU_RECOVERY_STEP_IDLE;
volatile uint32_t imu_recovery_attempt_count = 0UL;
volatile uint32_t imu_recovery_failure_count = 0UL;
volatile uint32_t imu_recovery_last_duration_us = 0UL;
volatile uint32_t imu_recovery_max_duration_us = 0UL;
volatile uint32_t imu_register_error_count = 0UL;

/* IMU data-quality diagnostics */
volatile uint8_t imu_last_sample_valid_debug = 0U;
volatile uint32_t imu_valid_sample_count = 0UL;
volatile uint32_t imu_invalid_sample_count = 0UL;
volatile uint32_t imu_all_zero_sample_count = 0UL;
volatile uint32_t imu_consecutive_invalid_sample_count = 0UL;
volatile uint32_t imu_max_consecutive_invalid_sample_count = 0UL;
volatile uint32_t imu_last_valid_sample_timestamp_us = 0UL;
volatile uint32_t imu_last_invalid_sample_timestamp_us = 0UL;
volatile uint32_t imu_sample_generation_debug = 0UL;

volatile uint8_t imu_dma_debug_busy = 0U;
volatile uint8_t imu_dma_debug_done = 0U;
volatile uint8_t imu_dma_debug_error = 0U;
volatile uint8_t imu_dma_debug_type = 0U;

volatile uint8_t imu_dma_rx0 = 0U;
volatile uint8_t imu_dma_rx1 = 0U;
volatile uint8_t imu_dma_rx2 = 0U;
volatile uint8_t imu_dma_rx3 = 0U;

volatile uint32_t imu_spi_cr1 = 0UL;
volatile uint32_t imu_spi_cr2 = 0UL;
volatile uint32_t imu_spi_sr = 0UL;

volatile uint32_t imu_spi_idle_timeout_count = 0UL;

volatile uint32_t imu_redundant_set_count = 0UL;
volatile uint32_t imu_redundant_burst_read_count = 0UL;
volatile uint32_t imu_redundant_disagreement_count = 0UL;
volatile uint32_t imu_redundant_reject_count = 0UL;
volatile uint32_t imu_bit8_correction_count = 0UL;
volatile uint32_t imu_bit8_correction_gyro_x_count = 0UL;
volatile uint32_t imu_bit8_correction_gyro_y_count = 0UL;
volatile uint32_t imu_bit8_correction_gyro_z_count = 0UL;
volatile uint32_t imu_bit8_correction_accel_x_count = 0UL;
volatile uint32_t imu_bit8_correction_accel_y_count = 0UL;
volatile uint32_t imu_bit8_correction_accel_z_count = 0UL;
volatile int16_t imu_last_triplet_gyro_x[3] = {0, 0, 0};
volatile int16_t imu_last_triplet_accel_z[3] = {0, 0, 0};

/* -------------------------------------------------------------------------- */
/* Debug                                                                      */
/* -------------------------------------------------------------------------- */

static void IMU_UpdateLiveDebug(void)
{
    imu_drv_initialized = imu_initialized;
    imu_drv_connected = imu_connected;
    imu_drv_device_id = imu_device_id;

    imu_dma_debug_busy = imu_dma_busy;
    imu_dma_debug_done = imu_dma_done;
    imu_dma_debug_error = imu_dma_error;
    imu_dma_debug_type = imu_dma_transfer_type;

    imu_last_sample_valid_debug = imu_last_sample_valid;
    imu_sample_valid = imu_last_sample_valid;
    imu_sample_generation_debug = imu_sample_generation;
    imu_stale_warmup_active_debug = imu_stale_warmup_active;

    imu_dma_rx0 = imu_dma_rx[0];
    imu_dma_rx1 = imu_dma_rx[1];
    imu_dma_rx2 = imu_dma_rx[2];
    imu_dma_rx3 = imu_dma_rx[3];

    if (hspi1.Instance != 0)
    {
        imu_spi_cr1 = hspi1.Instance->CR1;
        imu_spi_cr2 = hspi1.Instance->CR2;
        imu_spi_sr = hspi1.Instance->SR;
    }
}

/* -------------------------------------------------------------------------- */
/* Small waits                                                                */
/* -------------------------------------------------------------------------- */

static void IMU_DelayShort(void)
{
    for (volatile uint32_t i = 0UL; i < 16UL; i++)
    {
        __NOP();
    }
}

static void IMU_WaitSPIIdle(void)
{
    uint32_t timeout = 100000UL;

    while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) != RESET)
    {
        if (timeout == 0UL)
        {
            imu_spi_idle_timeout_count++;
            imu_drv_error_count++;
            IMU_UpdateLiveDebug();
            break;
        }

        timeout--;
    }
}

/* -------------------------------------------------------------------------- */
/* CS                                                                         */
/* -------------------------------------------------------------------------- */

static void IMU_ConfigureCS(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = BOARD_IMU_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(BOARD_IMU_CS_PORT, &GPIO_InitStruct);

    IMU_CS_HIGH();
}

/* -------------------------------------------------------------------------- */
/* SPI init                                                                   */
/* -------------------------------------------------------------------------- */

static uint8_t IMU_HardwareSPI_DMA_Init(void)
{
    /*
     * SPI1 artık CubeMX tarafından MX_SPI1_Init() ile başlatılıyor.
     * Bu driver SPI'yi tekrar DeInit/Init yapmaz.
     * Sadece CS pinini güvenli seviyeye alır ve handle kontrolü yapar.
     */

    IMU_ConfigureCS();

    if (hspi1.Instance != SPI1)
    {
        imu_drv_last_hal_status = 101U;
        imu_drv_error_count++;
        IMU_UpdateLiveDebug();
        return 0U;
    }

    if ((hspi1.hdmarx == 0) || (hspi1.hdmatx == 0))
    {
        imu_drv_last_hal_status = 102U;
        imu_drv_error_count++;
        IMU_UpdateLiveDebug();
        return 0U;
    }

    /*
     * Eğer önceki bir işlem yarım kaldıysa temizle.
     */
    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    {
        (void)HAL_SPI_Abort(&hspi1);
    }

    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    {
        imu_drv_last_hal_status = 103U;
        imu_drv_error_count++;
        IMU_UpdateLiveDebug();
        return 0U;
    }

    SET_BIT(hspi1.Instance->CR1, SPI_CR1_SSI);

    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);

    IMU_CS_HIGH();

    /* P36: no blocking settle delay here. Startup already has its own delay;
     * runtime recovery is driven by explicit microsecond deadlines. */
    imu_drv_last_hal_status = 0U;

    IMU_UpdateLiveDebug();

    return 1U;
}



/* -------------------------------------------------------------------------- */
/* V44 - IMU-only software SPI fallback                                       */
/* -------------------------------------------------------------------------- */

/* P33: fast deterministic software-SPI timing.
 *
 * P32 used HAL_GPIO_WritePin/HAL_GPIO_ReadPin plus a 40-iteration delay on
 * every clock edge. On the target board that made one validated 3-burst IMU
 * sample take about 2.6 ms, so the nominal 1 kHz task could only execute at
 * roughly 300-330 Hz.
 *
 * Keep the proven Mode-3 software backend and the full 3-burst integrity
 * validation, but drive GPIOA through BSRR/IDR directly. The fixed NOP delay
 * is intentionally conservative for ISM330DLC while removing HAL call
 * overhead from the hot path.
 */
#define IMU_SW_SPI_EDGE_DELAY() do { \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
} while (0)

#define IMU_SW_SCK_HIGH()  (GPIOA->BSRR = (uint32_t)GPIO_PIN_5)
#define IMU_SW_SCK_LOW()   (GPIOA->BSRR = ((uint32_t)GPIO_PIN_5 << 16U))
#define IMU_SW_MOSI_HIGH() (GPIOA->BSRR = (uint32_t)GPIO_PIN_7)
#define IMU_SW_MOSI_LOW()  (GPIOA->BSRR = ((uint32_t)GPIO_PIN_7 << 16U))
#define IMU_SW_CS_HIGH()   (GPIOA->BSRR = (uint32_t)GPIO_PIN_4)
#define IMU_SW_CS_LOW()    (GPIOA->BSRR = ((uint32_t)GPIO_PIN_4 << 16U))
#define IMU_SW_MISO_HIGH() ((GPIOA->IDR & (uint32_t)GPIO_PIN_6) != 0UL)

static inline void IMU_SW_SPI_Delay(void)
{
    IMU_SW_SPI_EDGE_DELAY();
}

static void IMU_SW_SPI_ConfigureGPIO(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Release PA5/6/7 from the SPI1 peripheral. */
    __HAL_SPI_DISABLE(&hspi1);

    /* PA4 = CS, PA5 = SCK, PA7 = MOSI. Mode-3 idles SCK HIGH. */
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PA6 = MISO. */
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    IMU_SW_SPI_Delay();
}

/* P44 soft bus recovery. CS remains deasserted while sixteen clocks clear any
 * partial software-SPI bit position. No HAL_Delay, sensor reset, or register
 * transaction is performed here; the next 1 ms task owns the validation read. */
static void IMU_SPI_ResynchronizeBus(void)
{
    if (imu_swspi_active != 0U)
    {
        IMU_SW_SPI_ConfigureGPIO();
        IMU_SW_CS_HIGH();
        IMU_SW_MOSI_LOW();

        for (uint8_t pulse = 0U; pulse < 16U; pulse++)
        {
            IMU_SW_SCK_LOW();
            IMU_SW_SPI_EDGE_DELAY();
            IMU_SW_SCK_HIGH();
            IMU_SW_SPI_EDGE_DELAY();
        }

        IMU_SW_CS_HIGH();
        IMU_SW_SCK_HIGH();
        IMU_SW_MOSI_LOW();
        IMU_SW_SPI_EDGE_DELAY();
    }
    else
    {
        (void)HAL_SPI_Abort(&hspi1);
        IMU_WaitSPIIdle();
        IMU_CS_HIGH();
    }

    imu_dma_busy = 0U;
    imu_dma_done = 0U;
    imu_dma_error = 0U;
    imu_dma_transfer_type = IMU_DMA_TRANSFER_NONE;
}

static inline uint8_t IMU_SW_SPI_Byte(uint8_t tx)
{
    uint8_t rx = 0U;

    /* SPI mode 3: idle HIGH, update MOSI on falling edge, sample on rising. */
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
        const uint8_t mask = (uint8_t)(0x80U >> bit);

        IMU_SW_SCK_LOW();
        if ((tx & mask) != 0U)
        {
            IMU_SW_MOSI_HIGH();
        }
        else
        {
            IMU_SW_MOSI_LOW();
        }
        IMU_SW_SPI_EDGE_DELAY();

        IMU_SW_SCK_HIGH();
        IMU_SW_SPI_EDGE_DELAY();

        if (IMU_SW_MISO_HIGH())
        {
            rx |= mask;
        }
    }

    return rx;
}

static HAL_StatusTypeDef IMU_SW_SPI_Transfer(
    uint8_t *tx,
    uint8_t *rx,
    uint16_t length
)
{
    if ((tx == 0) || (rx == 0) || (length == 0U))
    {
        imu_swspi_error_count++;
        return HAL_ERROR;
    }

    IMU_SW_SCK_HIGH();
    IMU_SW_MOSI_LOW();
    IMU_SW_CS_HIGH();
    IMU_SW_SPI_EDGE_DELAY();

    IMU_SW_CS_LOW();
    IMU_SW_SPI_EDGE_DELAY();

    for (uint16_t i = 0U; i < length; ++i)
    {
        rx[i] = IMU_SW_SPI_Byte(tx[i]);
    }

    IMU_SW_CS_HIGH();
    IMU_SW_SCK_HIGH();
    IMU_SW_SPI_EDGE_DELAY();

    imu_swspi_transfer_count++;
    return HAL_OK;
}

static uint8_t IMU_SW_SPI_EnableIfPresent(void)
{
    uint8_t valid_count = 0U;
    uint8_t who = 0U;

    IMU_SW_SPI_ConfigureGPIO();
    imu_swspi_probe_count++;

    for (uint8_t attempt = 0U; attempt < 3U; ++attempt)
    {
        uint8_t tx[2] = {
            (uint8_t)(0x80U | IMU_REG_WHO_AM_I),
            0x00U
        };
        uint8_t rx[2] = {0U, 0U};

        if (IMU_SW_SPI_Transfer(tx, rx, 2U) == HAL_OK)
        {
            who = rx[1];

            if ((who == IMU_WHO_AM_I_ISM330DLC) ||
                (who == IMU_WHO_AM_I_ALT))
            {
                valid_count++;
            }
        }

        HAL_Delay(2U);
    }

    imu_swspi_whoami = who;

    if (valid_count >= 2U)
    {
        imu_swspi_active = 1U;

        /* Re-assert the exact software-SPI GPIO configuration after probing. */
        IMU_SW_SPI_ConfigureGPIO();

        imu_spi_profile_found = 1U;
        imu_spi_mode_selected = 3U;
        imu_spi_prescaler_selected = 0U; /* 0 = software SPI fallback */
        imu_spi_selected_polarity = SPI_POLARITY_HIGH;
        imu_spi_selected_phase = SPI_PHASE_2EDGE;

        return 1U;
    }

    imu_swspi_active = 0U;

    /* Restore normal SPI1 pins so the legacy hardware path can still try. */
    {
        GPIO_InitTypeDef gpio = {0};

        gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &gpio);

        (void)HAL_SPI_Init(&hspi1);
        SET_BIT(hspi1.Instance->CR1, SPI_CR1_SSI);
        __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
    }

    return 0U;
}

/* -------------------------------------------------------------------------- */
/* V41 - robust SPI1 startup / register access                                */
/* -------------------------------------------------------------------------- */

static uint16_t IMU_PrescalerToDivider(uint32_t prescaler)
{
    switch (prescaler)
    {
        case SPI_BAUDRATEPRESCALER_32:  return 32U;
        case SPI_BAUDRATEPRESCALER_64:  return 64U;
        case SPI_BAUDRATEPRESCALER_128: return 128U;
        case SPI_BAUDRATEPRESCALER_256: return 256U;
        default:                        return 0U;
    }
}

static uint8_t IMU_ApplySPIProfile(
    uint32_t polarity,
    uint32_t phase,
    uint32_t prescaler
)
{
    IMU_CS_HIGH();

    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    {
        (void)HAL_SPI_Abort(&hspi1);
    }

    hspi1.Init.CLKPolarity = polarity;
    hspi1.Init.CLKPhase = phase;
    hspi1.Init.BaudRatePrescaler = prescaler;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        imu_drv_last_hal_status = 210U;
        imu_spi_poll_error_count++;
        return 0U;
    }

    SET_BIT(hspi1.Instance->CR1, SPI_CR1_SSI);
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
    IMU_CS_HIGH();

    HAL_Delay(2U);
    return 1U;
}

static HAL_StatusTypeDef IMU_V43_DirectTransfer(
    uint8_t *tx,
    uint8_t *rx,
    uint16_t length
)
{
    SPI_TypeDef *spi = hspi1.Instance;

    if ((tx == 0) || (rx == 0) || (length == 0U) || (spi != SPI1))
    {
        imu_v43_direct_error_count++;
        return HAL_ERROR;
    }

    /* Register transactions are only issued when the IMU DMA path is idle. */
    if (imu_dma_busy != 0U)
    {
        imu_v43_direct_error_count++;
        return HAL_BUSY;
    }

    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    {
        (void)HAL_SPI_Abort(&hspi1);
    }

    /* Drain stale RX data / overrun before asserting CS. */
    while ((spi->SR & SPI_SR_RXNE) != 0U)
    {
        volatile uint8_t dump = *(__IO uint8_t *)&spi->DR;
        (void)dump;
    }
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);

    SET_BIT(spi->CR1, SPI_CR1_SSI);
    SET_BIT(spi->CR1, SPI_CR1_SPE);

    IMU_CS_HIGH();
    IMU_DelayShort();
    IMU_CS_LOW();
    IMU_DelayShort();

    for (uint16_t i = 0U; i < length; ++i)
    {
        uint32_t timeout = 100000UL;

        while ((spi->SR & SPI_SR_TXE) == 0U)
        {
            if (timeout-- == 0UL)
            {
                IMU_CS_HIGH();
                imu_v43_direct_error_count++;
                return HAL_TIMEOUT;
            }
        }

        *(__IO uint8_t *)&spi->DR = tx[i];

        timeout = 100000UL;
        while ((spi->SR & SPI_SR_RXNE) == 0U)
        {
            if (timeout-- == 0UL)
            {
                IMU_CS_HIGH();
                imu_v43_direct_error_count++;
                return HAL_TIMEOUT;
            }
        }

        rx[i] = *(__IO uint8_t *)&spi->DR;
    }

    {
        uint32_t timeout = 100000UL;
        while ((spi->SR & SPI_SR_BSY) != 0U)
        {
            if (timeout-- == 0UL)
            {
                IMU_CS_HIGH();
                imu_v43_direct_error_count++;
                return HAL_TIMEOUT;
            }
        }
    }

    IMU_CS_HIGH();
    imu_v43_direct_transfer_count++;
    return HAL_OK;
}

static HAL_StatusTypeDef IMU_PollingTransfer(
    uint8_t *tx,
    uint8_t *rx,
    uint16_t length
)
{
    HAL_StatusTypeDef status;

    if ((tx == 0) || (rx == 0) || (length == 0U))
    {
        return HAL_ERROR;
    }

    /* V44: V43 proved GPIO bit-bang works while the SPI1 peripheral returns
     * 0x00. Once software SPI is enabled, use it for all IMU register access.
     * Other buses/peripherals are untouched. */
    if (imu_swspi_active != 0U)
    {
        status = IMU_SW_SPI_Transfer(tx, rx, length);
    }
    else
    {
        status = IMU_V43_DirectTransfer(tx, rx, length);
    }

    imu_drv_last_hal_status = (uint8_t)status;

    if (status != HAL_OK)
    {
        imu_spi_poll_error_count++;
    }

    return status;
}

static uint8_t IMU_AutoProbeSPI(void)
{
    typedef struct
    {
        uint32_t polarity;
        uint32_t phase;
        uint32_t prescaler;
        uint8_t mode_number;
    } IMU_SPIProfile_t;

    /* Start deliberately slow. ISM330DLC supports SPI mode 0 and mode 3.
     * This allows the same firmware to recover from clock-edge sensitivity on
     * the small-board harness without touching any other peripheral. */
    static const IMU_SPIProfile_t profiles[] =
    {
        {SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_BAUDRATEPRESCALER_256, 3U},
        {SPI_POLARITY_LOW,  SPI_PHASE_1EDGE, SPI_BAUDRATEPRESCALER_256, 0U},
        {SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_BAUDRATEPRESCALER_128, 3U},
        {SPI_POLARITY_LOW,  SPI_PHASE_1EDGE, SPI_BAUDRATEPRESCALER_128, 0U},
        {SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_BAUDRATEPRESCALER_64,  3U},
        {SPI_POLARITY_LOW,  SPI_PHASE_1EDGE, SPI_BAUDRATEPRESCALER_64,  0U},
        {SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_BAUDRATEPRESCALER_32,  3U},
        {SPI_POLARITY_LOW,  SPI_PHASE_1EDGE, SPI_BAUDRATEPRESCALER_32,  0U}
    };

    for (uint32_t i = 0UL; i < (sizeof(profiles) / sizeof(profiles[0])); ++i)
    {
        uint8_t good_count = 0U;
        uint8_t who = 0U;

        imu_spi_probe_attempt_count++;

        if (IMU_ApplySPIProfile(
                profiles[i].polarity,
                profiles[i].phase,
                profiles[i].prescaler) == 0U)
        {
            continue;
        }

        /* Require two valid responses; a single accidental byte is not enough. */
        for (uint8_t n = 0U; n < 3U; ++n)
        {
            who = IMU_ReadReg(IMU_REG_WHO_AM_I);

            if ((who == IMU_WHO_AM_I_ISM330DLC) ||
                (who == IMU_WHO_AM_I_ALT))
            {
                good_count++;
            }

            HAL_Delay(2U);
        }

        if (good_count >= 2U)
        {
            imu_spi_profile_found = 1U;
            imu_spi_mode_selected = profiles[i].mode_number;
            imu_spi_prescaler_selected =
                IMU_PrescalerToDivider(profiles[i].prescaler);
            imu_spi_selected_polarity = profiles[i].polarity;
            imu_spi_selected_phase = profiles[i].phase;
            imu_device_id = who;
            imu_drv_last_whoami = who;
            return 1U;
        }
    }

    imu_spi_profile_found = 0U;
    return 0U;
}

static void IMU_SelectRuntimeSPI(void)
{
    /* Prefer /32; if the physical link cannot repeat WHO_AM_I reliably,
     * automatically downshift to /64, /128, then /256. */
    static const uint32_t runtime_prescalers[] =
    {
        SPI_BAUDRATEPRESCALER_32,
        SPI_BAUDRATEPRESCALER_64,
        SPI_BAUDRATEPRESCALER_128,
        SPI_BAUDRATEPRESCALER_256
    };

    for (uint32_t i = 0UL;
         i < (sizeof(runtime_prescalers) / sizeof(runtime_prescalers[0]));
         ++i)
    {
        uint8_t valid = 0U;

        if (IMU_ApplySPIProfile(
                imu_spi_selected_polarity,
                imu_spi_selected_phase,
                runtime_prescalers[i]) == 0U)
        {
            continue;
        }

        for (uint8_t n = 0U; n < 3U; ++n)
        {
            uint8_t who = IMU_ReadReg(IMU_REG_WHO_AM_I);

            if ((who == IMU_WHO_AM_I_ISM330DLC) ||
                (who == IMU_WHO_AM_I_ALT))
            {
                valid++;
            }
        }

        if (valid >= 2U)
        {
            imu_spi_prescaler_selected =
                IMU_PrescalerToDivider(runtime_prescalers[i]);
            return;
        }
    }

    /* If upshift validation failed, return to the slowest robust profile. */
    (void)IMU_ApplySPIProfile(
        imu_spi_selected_polarity,
        imu_spi_selected_phase,
        SPI_BAUDRATEPRESCALER_256
    );
    imu_spi_prescaler_selected = 256U;
}

/* -------------------------------------------------------------------------- */
/* V42 - physical SPI1 line diagnostic                                        */
/* -------------------------------------------------------------------------- */

static void IMU_V42_DelayGPIO(void)
{
    /* Deliberately slow bit-bang timing. Exact frequency is unimportant here. */
    for (volatile uint32_t i = 0UL; i < 400UL; ++i)
    {
        __NOP();
    }
}

static uint8_t IMU_V42_BitBangByte(uint8_t tx, uint8_t mode3)
{
    uint8_t rx = 0U;

    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
        uint8_t mask = (uint8_t)(0x80U >> bit);

        if (mode3 == 0U)
        {
            /* Mode 0: idle LOW, data valid before rising edge, sample rising. */
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,
                ((tx & mask) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            IMU_V42_DelayGPIO();

            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            IMU_V42_DelayGPIO();

            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET)
            {
                rx |= mask;
            }

            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            IMU_V42_DelayGPIO();
        }
        else
        {
            /* Mode 3: idle HIGH, change on falling edge, sample rising edge. */
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,
                ((tx & mask) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            IMU_V42_DelayGPIO();

            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            IMU_V42_DelayGPIO();

            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET)
            {
                rx |= mask;
            }
        }
    }

    return rx;
}

static uint8_t IMU_V43_BitBangReadReg(uint8_t reg, uint8_t mode3)
{
    uint8_t value;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,
        (mode3 != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    IMU_V42_DelayGPIO();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    IMU_V42_DelayGPIO();

    (void)IMU_V42_BitBangByte((uint8_t)(0x80U | reg), mode3);
    value = IMU_V42_BitBangByte(0x00U, mode3);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    IMU_V42_DelayGPIO();

    return value;
}

static uint8_t IMU_V42_BitBangWhoAmI(uint8_t mode3)
{
    return IMU_V43_BitBangReadReg(IMU_REG_WHO_AM_I, mode3);
}

void IMU_RunPhysicalLineDiagnostic(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t saved_polarity = hspi1.Init.CLKPolarity;
    uint32_t saved_phase = hspi1.Init.CLKPhase;
    uint32_t saved_prescaler = hspi1.Init.BaudRatePrescaler;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Do not disturb a healthy IMU. This diagnostic is for failed startup only. */
    if (imu_connected != 0U)
    {
        imu_phy_diag_done = 1U;
        imu_phy_spi_response_ok = 1U;
        return;
    }

    (void)HAL_SPI_Abort(&hspi1);
    __HAL_SPI_DISABLE(&hspi1);

    /* PA4 CS, PA5 SCK, PA7 MOSI as plain GPIO outputs. */
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* CS pin drive/readback. */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    IMU_V42_DelayGPIO();
    imu_phy_cs_high_read =
        (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) ? 1U : 0U;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    IMU_V42_DelayGPIO();
    imu_phy_cs_low_read =
        (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) ? 1U : 0U;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /* SCK output drive/readback. */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    IMU_V42_DelayGPIO();
    imu_phy_sck_high_read =
        (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) ? 1U : 0U;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    IMU_V42_DelayGPIO();
    imu_phy_sck_low_read =
        (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) ? 1U : 0U;

    /* MOSI output drive/readback. */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    IMU_V42_DelayGPIO();
    imu_phy_mosi_high_read =
        (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET) ? 1U : 0U;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    IMU_V42_DelayGPIO();
    imu_phy_mosi_low_read =
        (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET) ? 1U : 0U;

    imu_phy_gpio_drive_ok =
        ((imu_phy_cs_high_read == 1U) && (imu_phy_cs_low_read == 0U) &&
         (imu_phy_sck_high_read == 1U) && (imu_phy_sck_low_read == 0U) &&
         (imu_phy_mosi_high_read == 1U) && (imu_phy_mosi_low_read == 0U)) ? 1U : 0U;

    /* With CS high, a healthy SPI MISO is normally high impedance. See whether
     * PA6 follows weak internal pull-up and pull-down. If not, the line may be
     * externally held/shorted. */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);
    IMU_V42_DelayGPIO();
    imu_phy_miso_pullup_read =
        (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) ? 1U : 0U;

    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &gpio);
    IMU_V42_DelayGPIO();
    imu_phy_miso_pulldown_read =
        (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) ? 1U : 0U;

    imu_phy_miso_free =
        ((imu_phy_miso_pullup_read == 1U) &&
         (imu_phy_miso_pulldown_read == 0U)) ? 1U : 0U;

    /* MISO floating input for actual bit-bang transactions. */
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    imu_phy_bitbang_mode0_who = IMU_V42_BitBangWhoAmI(0U);
    HAL_Delay(2U);
    imu_phy_bitbang_mode3_who = IMU_V42_BitBangWhoAmI(1U);

    /* Pick a stable bit-bang mode and capture a small register fingerprint.
     * 0x6A is the project's expected ISM330DLC ID; 0x6B is kept as the
     * alternate ID already accepted by the existing project. */
    if ((imu_phy_bitbang_mode3_who == IMU_WHO_AM_I_ISM330DLC) ||
        (imu_phy_bitbang_mode3_who == IMU_WHO_AM_I_ALT))
    {
        imu_v43_bb_selected_mode = 3U;
    }
    else if ((imu_phy_bitbang_mode0_who == IMU_WHO_AM_I_ISM330DLC) ||
             (imu_phy_bitbang_mode0_who == IMU_WHO_AM_I_ALT))
    {
        imu_v43_bb_selected_mode = 0U;
    }
    else
    {
        imu_v43_bb_selected_mode = 0xFFU;
    }

    if (imu_v43_bb_selected_mode != 0xFFU)
    {
        uint8_t m3 = (imu_v43_bb_selected_mode == 3U) ? 1U : 0U;
        imu_v43_bb_who = IMU_V43_BitBangReadReg(IMU_REG_WHO_AM_I, m3);
        imu_v43_bb_ctrl1_xl = IMU_V43_BitBangReadReg(IMU_REG_CTRL1_XL, m3);
        imu_v43_bb_ctrl2_g = IMU_V43_BitBangReadReg(IMU_REG_CTRL2_G, m3);
        imu_v43_bb_ctrl3_c = IMU_V43_BitBangReadReg(IMU_REG_CTRL3_C, m3);
        imu_v43_bb_ctrl4_c = IMU_V43_BitBangReadReg(IMU_REG_CTRL4_C, m3);
        imu_v43_bb_status = IMU_V43_BitBangReadReg(IMU_REG_STATUS_REG, m3);
    }

    imu_phy_spi_response_ok =
        ((imu_phy_bitbang_mode0_who == IMU_WHO_AM_I_ISM330DLC) ||
         (imu_phy_bitbang_mode0_who == IMU_WHO_AM_I_ALT) ||
         (imu_phy_bitbang_mode3_who == IMU_WHO_AM_I_ISM330DLC) ||
         (imu_phy_bitbang_mode3_who == IMU_WHO_AM_I_ALT)) ? 1U : 0U;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /* Restore PA5/6/7 to SPI1 AF5 exactly as the project used before V42. */
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /* V43: after AF5 has been restored, test SPI1 itself without the HAL
     * TransmitReceive state machine. This separates GPIO wiring from the SPI1
     * peripheral path. */
    if (IMU_ApplySPIProfile(
            SPI_POLARITY_LOW, SPI_PHASE_1EDGE, SPI_BAUDRATEPRESCALER_256) != 0U)
    {
        uint8_t tx[2] = {(uint8_t)(0x80U | IMU_REG_WHO_AM_I), 0x00U};
        uint8_t rx[2] = {0U, 0U};
        if (IMU_V43_DirectTransfer(tx, rx, 2U) == HAL_OK)
        {
            imu_v43_direct_mode0_who = rx[1];
        }
    }

    if (IMU_ApplySPIProfile(
            SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_BAUDRATEPRESCALER_256) != 0U)
    {
        uint8_t tx[2] = {(uint8_t)(0x80U | IMU_REG_WHO_AM_I), 0x00U};
        uint8_t rx[2] = {0U, 0U};
        if (IMU_V43_DirectTransfer(tx, rx, 2U) == HAL_OK)
        {
            imu_v43_direct_mode3_who = rx[1];
        }
    }

    imu_v43_direct_response_ok =
        ((imu_v43_direct_mode0_who == IMU_WHO_AM_I_ISM330DLC) ||
         (imu_v43_direct_mode0_who == IMU_WHO_AM_I_ALT) ||
         (imu_v43_direct_mode3_who == IMU_WHO_AM_I_ISM330DLC) ||
         (imu_v43_direct_mode3_who == IMU_WHO_AM_I_ALT)) ? 1U : 0U;

    hspi1.Init.CLKPolarity = saved_polarity;
    hspi1.Init.CLKPhase = saved_phase;
    hspi1.Init.BaudRatePrescaler = saved_prescaler;
    (void)HAL_SPI_Init(&hspi1);
    SET_BIT(hspi1.Instance->CR1, SPI_CR1_SSI);
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);

    imu_phy_diag_done = 1U;
    IMU_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */
/* DMA transfer                                                               */
/* -------------------------------------------------------------------------- */

static HAL_StatusTypeDef IMU_DMA_StartTransfer(
    uint8_t *tx,
    uint8_t *rx,
    uint16_t length,
    uint8_t transfer_type
)
{
    HAL_StatusTypeDef status;

    if ((tx == 0) || (rx == 0) || (length == 0U))
    {
        imu_drv_error_count++;
        return HAL_ERROR;
    }

    if (imu_dma_busy != 0U)
    {
        imu_dma_busy_count++;
        return HAL_BUSY;
    }

    imu_dma_busy = 1U;
    imu_dma_done = 0U;
    imu_dma_error = 0U;
    imu_dma_transfer_type = transfer_type;

    imu_dma_start_count++;

    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);

    IMU_CS_HIGH();
    IMU_DelayShort();

    IMU_CS_LOW();
    IMU_DelayShort();

    status = HAL_SPI_TransmitReceive_DMA(
        &hspi1,
        tx,
        rx,
        length
    );

    imu_drv_last_hal_status = (uint8_t)status;

    if (status != HAL_OK)
    {
        IMU_WaitSPIIdle();
        IMU_CS_HIGH();

        imu_dma_busy = 0U;
        imu_dma_done = 0U;
        imu_dma_error = 1U;
        imu_dma_transfer_type = IMU_DMA_TRANSFER_NONE;

        imu_drv_error_count++;
        imu_dma_error_count++;

        IMU_UpdateLiveDebug();

        return status;
    }

    IMU_UpdateLiveDebug();

    return HAL_OK;
}

static HAL_StatusTypeDef IMU_DMA_TransferBlocking(
    uint8_t *tx,
    uint8_t *rx,
    uint16_t length,
    uint32_t timeout_ms
)
{
    uint32_t start_tick = HAL_GetTick();

    /* V44: preserve the existing validated/raw-data pipeline, but replace only
     * the broken SPI1 electrical-transfer backend with the proven GPIO path.
     * Counters are kept compatible with existing UART diagnostics. */
    if (imu_swspi_active != 0U)
    {
        HAL_StatusTypeDef sw_status;

        imu_dma_start_count++;
        imu_dma_busy = 1U;
        imu_dma_done = 0U;
        imu_dma_error = 0U;
        imu_dma_transfer_type = IMU_DMA_TRANSFER_BLOCKING;

        sw_status = IMU_SW_SPI_Transfer(tx, rx, length);

        imu_dma_busy = 0U;
        imu_dma_transfer_type = IMU_DMA_TRANSFER_NONE;

        if (sw_status == HAL_OK)
        {
            imu_dma_done = 1U;
            imu_dma_complete_count++;
            imu_drv_last_hal_status = (uint8_t)HAL_OK;
            /* P34: do not copy debug/register state after every one of the
             * three software-SPI bursts. IMU_AcquireValidatedSample() commits
             * the complete triplet and performs one coherent debug refresh. */
        }
        else
        {
            imu_dma_done = 0U;
            imu_dma_error = 1U;
            imu_dma_error_count++;
            imu_drv_error_count++;
            imu_drv_last_hal_status = (uint8_t)sw_status;
            IMU_UpdateLiveDebug();
        }

        return sw_status;
    }

    HAL_StatusTypeDef status = IMU_DMA_StartTransfer(
        tx,
        rx,
        length,
        IMU_DMA_TRANSFER_BLOCKING
    );

    if (status != HAL_OK)
    {
        return status;
    }

    while ((imu_dma_done == 0U) && (imu_dma_error == 0U))
    {
        if ((HAL_GetTick() - start_tick) >= timeout_ms)
        {
            HAL_SPI_Abort(&hspi1);

            IMU_WaitSPIIdle();
            IMU_CS_HIGH();

            imu_dma_busy = 0U;
            imu_dma_done = 0U;
            imu_dma_error = 1U;
            imu_dma_transfer_type = IMU_DMA_TRANSFER_NONE;

            imu_dma_timeout_count++;
            imu_drv_error_count++;
            imu_drv_last_hal_status = 200U;

            IMU_UpdateLiveDebug();

            return HAL_TIMEOUT;
        }
    }

    if (imu_dma_error != 0U)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* -------------------------------------------------------------------------- */
/* Parser                                                                     */
/* -------------------------------------------------------------------------- */

static void IMU_ParseRawBufferToStruct(
    const uint8_t *buffer,
    IMU_RawData_t *candidate
)
{
    if ((buffer == 0) || (candidate == 0))
    {
        return;
    }

    candidate->gyro_x_raw =
        (int16_t)(((uint16_t)buffer[1] << 8) |
                  (uint16_t)buffer[0]);

    candidate->gyro_y_raw =
        (int16_t)(((uint16_t)buffer[3] << 8) |
                  (uint16_t)buffer[2]);

    candidate->gyro_z_raw =
        (int16_t)(((uint16_t)buffer[5] << 8) |
                  (uint16_t)buffer[4]);

    candidate->accel_x_raw =
        (int16_t)(((uint16_t)buffer[7] << 8) |
                  (uint16_t)buffer[6]);

    candidate->accel_y_raw =
        (int16_t)(((uint16_t)buffer[9] << 8) |
                  (uint16_t)buffer[8]);

    candidate->accel_z_raw =
        (int16_t)(((uint16_t)buffer[11] << 8) |
                  (uint16_t)buffer[10]);
}

static int16_t IMU_Median3Int16(int16_t a, int16_t b, int16_t c)
{
    int16_t t;

    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) { t = a; a = b; b = t; }

    return b;
}

static int16_t IMU_ToggleBit8(int16_t value)
{
    return (int16_t)(((uint16_t)value) ^ IMU_BIT8_MASK);
}

static int32_t IMU_Abs32(int32_t value)
{
    return (value < 0L) ? -value : value;
}

static uint16_t IMU_Abs16U(int16_t value)
{
    int32_t extended = (int32_t)value;

    if (extended < 0L)
    {
        extended = -extended;
    }

    return (uint16_t)extended;
}

static uint8_t IMU_RawDataEqual(
    const IMU_RawData_t *a,
    const IMU_RawData_t *b
)
{
    if ((a == 0) || (b == 0))
    {
        return 0U;
    }

    return ((a->gyro_x_raw == b->gyro_x_raw) &&
            (a->gyro_y_raw == b->gyro_y_raw) &&
            (a->gyro_z_raw == b->gyro_z_raw) &&
            (a->accel_x_raw == b->accel_x_raw) &&
            (a->accel_y_raw == b->accel_y_raw) &&
            (a->accel_z_raw == b->accel_z_raw))
           ? 1U
           : 0U;
}

/* Detect the observed bus-corruption signature where one 16-bit word is
 * copied into every gyro and accelerometer axis, possibly with sign changes. */
static uint8_t IMU_HasRepeatedWordPattern(const IMU_RawData_t *raw)
{
    uint16_t magnitude;

    if (raw == 0)
    {
        return 0U;
    }

    magnitude = IMU_Abs16U(raw->gyro_x_raw);

    if (magnitude < IMU_REPEATED_WORD_MIN_ABS_RAW)
    {
        return 0U;
    }

    return ((IMU_Abs16U(raw->gyro_y_raw) == magnitude) &&
            (IMU_Abs16U(raw->gyro_z_raw) == magnitude) &&
            (IMU_Abs16U(raw->accel_x_raw) == magnitude) &&
            (IMU_Abs16U(raw->accel_y_raw) == magnitude) &&
            (IMU_Abs16U(raw->accel_z_raw) == magnitude))
           ? 1U
           : 0U;
}

static void IMU_CapturePatternDiagnostic(
    const IMU_RawData_t samples[IMU_REDUNDANT_BURST_COUNT],
    const IMU_RawData_t *corrected
)
{
    uint16_t magnitude;
    uint8_t sign_mask = 0U;
    uint8_t i;

    if ((samples == 0) || (corrected == 0))
    {
        return;
    }

    for (i = 0U; i < IMU_REDUNDANT_BURST_COUNT; i++)
    {
        imu_pattern_diagnostic.burst[i] = samples[i];
    }

    /* On the first triplet of a pattern episode, invalidate the previous
     * register snapshot and request a fresh one.  The second consecutive bad
     * triplet must not erase the snapshot captured before its retry. */
    if (imu_pattern_retry_pending == 0U)
    {
        imu_pattern_diagnostic.register_snapshot_valid = 0U;
        imu_pattern_diagnostic.whoami = 0U;
        imu_pattern_diagnostic.ctrl1_xl = 0U;
        imu_pattern_diagnostic.ctrl2_g = 0U;
        imu_pattern_diagnostic.ctrl3_c = 0U;
        imu_pattern_diagnostic.ctrl4_c = 0U;
        imu_pattern_register_snapshot_pending = 1U;
    }

    magnitude = IMU_Abs16U(corrected->gyro_x_raw);

    if (corrected->gyro_x_raw < 0) { sign_mask |= 0x01U; }
    if (corrected->gyro_y_raw < 0) { sign_mask |= 0x02U; }
    if (corrected->gyro_z_raw < 0) { sign_mask |= 0x04U; }
    if (corrected->accel_x_raw < 0) { sign_mask |= 0x08U; }
    if (corrected->accel_y_raw < 0) { sign_mask |= 0x10U; }
    if (corrected->accel_z_raw < 0) { sign_mask |= 0x20U; }

    imu_pattern_diagnostic.valid = 1U;
    imu_pattern_diagnostic.sign_mask = sign_mask;
    imu_pattern_diagnostic.repeated_magnitude_raw = magnitude;
    imu_pattern_diagnostic.event_count = imu_pattern_error_count + 1UL;
    imu_pattern_diagnostic.timestamp_us = micros();
}

static uint8_t IMU_ServicePatternFastConfigRepair(void)
{
    uint32_t start_us = micros();
    uint32_t duration_us;
    uint8_t whoami;
    uint8_t ctrl1_xl;
    uint8_t ctrl2_g;
    uint8_t ctrl3_c;
    uint8_t ctrl4_c;
    uint8_t whoami_valid;
    uint8_t config_valid;

    /* P55: run only after a repeated-word triplet and the P44 soft SPI
     * re-synchronization.  The first five reads are kept as the pre-repair
     * evidence in imu_pattern_diagnostic.  If WHO_AM_I is still valid but a
     * critical configuration register drifted, restore only those registers
     * and verify them immediately.  No sensor reset and no HAL_Delay. */
    whoami = IMU_ReadReg(IMU_REG_WHO_AM_I);
    ctrl1_xl = IMU_ReadReg(IMU_REG_CTRL1_XL);
    ctrl2_g = IMU_ReadReg(IMU_REG_CTRL2_G);
    ctrl3_c = IMU_ReadReg(IMU_REG_CTRL3_C);
    ctrl4_c = IMU_ReadReg(IMU_REG_CTRL4_C);

    imu_pattern_diagnostic.whoami = whoami;
    imu_pattern_diagnostic.ctrl1_xl = ctrl1_xl;
    imu_pattern_diagnostic.ctrl2_g = ctrl2_g;
    imu_pattern_diagnostic.ctrl3_c = ctrl3_c;
    imu_pattern_diagnostic.ctrl4_c = ctrl4_c;
    imu_pattern_diagnostic.register_snapshot_valid = 1U;

    imu_fast_config_check_count++;

    whoami_valid =
        ((whoami == IMU_WHO_AM_I_ISM330DLC) ||
         (whoami == IMU_WHO_AM_I_ALT)) ? 1U : 0U;

    config_valid =
        ((ctrl1_xl == IMU_CTRL1_XL_1666HZ_8G) &&
         (ctrl2_g == IMU_CTRL2_G_1666HZ_1000DPS) &&
         ((ctrl3_c & IMU_CTRL3_C_BDU_IF_INC) ==
          IMU_CTRL3_C_BDU_IF_INC) &&
         ((ctrl4_c & IMU_CTRL4_C_I2C_DISABLE) != 0U)) ? 1U : 0U;

    if (whoami_valid == 0U)
    {
        /* A bad WHO_AM_I means this is not a safe register-only repair. */
        imu_fast_config_repair_failure_count++;
        duration_us = (uint32_t)(micros() - start_us);
        imu_fast_config_repair_last_duration_us = duration_us;
        if (duration_us > imu_fast_config_repair_max_duration_us)
        {
            imu_fast_config_repair_max_duration_us = duration_us;
        }
        return IMU_FAST_CONFIG_REPAIR_ESCALATE;
    }

    if (config_valid != 0U)
    {
        /* Pure bus/framing upset: the existing one-shot deferred triplet retry
         * is sufficient, so do not write healthy configuration registers. */
        duration_us = (uint32_t)(micros() - start_us);
        imu_fast_config_repair_last_duration_us = duration_us;
        if (duration_us > imu_fast_config_repair_max_duration_us)
        {
            imu_fast_config_repair_max_duration_us = duration_us;
        }
        return IMU_FAST_CONFIG_REPAIR_CONTINUE;
    }

    imu_register_error_count++;
    imu_fast_config_repair_attempt_count++;

    IMU_WriteReg(IMU_REG_CTRL3_C, IMU_CTRL3_C_BDU_IF_INC);
    IMU_WriteReg(IMU_REG_CTRL4_C,
                 (uint8_t)(ctrl4_c | IMU_CTRL4_C_I2C_DISABLE));
    IMU_WriteReg(IMU_REG_CTRL1_XL, IMU_CTRL1_XL_1666HZ_8G);
    IMU_WriteReg(IMU_REG_CTRL2_G, IMU_CTRL2_G_1666HZ_1000DPS);

    whoami = IMU_ReadReg(IMU_REG_WHO_AM_I);
    ctrl1_xl = IMU_ReadReg(IMU_REG_CTRL1_XL);
    ctrl2_g = IMU_ReadReg(IMU_REG_CTRL2_G);
    ctrl3_c = IMU_ReadReg(IMU_REG_CTRL3_C);
    ctrl4_c = IMU_ReadReg(IMU_REG_CTRL4_C);

    imu_drv_last_whoami = whoami;
    imu_drv_ctrl1_xl_readback = ctrl1_xl;
    imu_drv_ctrl2_g_readback = ctrl2_g;
    imu_drv_ctrl3_c_readback = ctrl3_c;
    imu_drv_ctrl4_c_readback = ctrl4_c;
    imu_drv_i2c_disable_ok =
        ((ctrl4_c & IMU_CTRL4_C_I2C_DISABLE) != 0U) ? 1U : 0U;
    imu_drv_bdu_if_inc_ok =
        ((ctrl3_c & IMU_CTRL3_C_BDU_IF_INC) == IMU_CTRL3_C_BDU_IF_INC)
        ? 1U : 0U;
    imu_drv_fullscale_config_ok =
        (((whoami == IMU_WHO_AM_I_ISM330DLC) ||
          (whoami == IMU_WHO_AM_I_ALT)) &&
         (ctrl1_xl == IMU_CTRL1_XL_1666HZ_8G) &&
         (ctrl2_g == IMU_CTRL2_G_1666HZ_1000DPS) &&
         (imu_drv_bdu_if_inc_ok != 0U) &&
         (imu_drv_i2c_disable_ok != 0U)) ? 1U : 0U;

    duration_us = (uint32_t)(micros() - start_us);
    imu_fast_config_repair_last_duration_us = duration_us;
    if (duration_us > imu_fast_config_repair_max_duration_us)
    {
        imu_fast_config_repair_max_duration_us = duration_us;
    }

    if (imu_drv_fullscale_config_ok != 0U)
    {
        imu_fast_config_repair_success_count++;
        imu_last_register_check_timestamp_us = micros();
        return IMU_FAST_CONFIG_REPAIR_CONTINUE;
    }

    imu_fast_config_repair_failure_count++;
    imu_drv_error_count++;
    return IMU_FAST_CONFIG_REPAIR_ESCALATE;
}

static uint8_t IMU_ServiceStaleFastConfigRepair(void)
{
    uint32_t start_us = micros();
    uint32_t duration_us;
    uint8_t whoami;
    uint8_t ctrl1_xl;
    uint8_t ctrl2_g;
    uint8_t ctrl3_c;
    uint8_t ctrl4_c;
    uint8_t whoami_valid;
    uint8_t config_valid;

    /* P56: identical to the proven P55 register-only repair policy, but
     * triggered only by a time-qualified exact-six-axis stale sample. The
     * snapshot below is preserved before any write so UART can tell whether
     * the stale event was another register-loss episode or only a bus freeze. */
    whoami = IMU_ReadReg(IMU_REG_WHO_AM_I);
    ctrl1_xl = IMU_ReadReg(IMU_REG_CTRL1_XL);
    ctrl2_g = IMU_ReadReg(IMU_REG_CTRL2_G);
    ctrl3_c = IMU_ReadReg(IMU_REG_CTRL3_C);
    ctrl4_c = IMU_ReadReg(IMU_REG_CTRL4_C);

    imu_stale_diagnostic.whoami = whoami;
    imu_stale_diagnostic.ctrl1_xl = ctrl1_xl;
    imu_stale_diagnostic.ctrl2_g = ctrl2_g;
    imu_stale_diagnostic.ctrl3_c = ctrl3_c;
    imu_stale_diagnostic.ctrl4_c = ctrl4_c;
    imu_stale_diagnostic.register_snapshot_valid = 1U;

    imu_stale_fast_config_check_count++;

    whoami_valid =
        ((whoami == IMU_WHO_AM_I_ISM330DLC) ||
         (whoami == IMU_WHO_AM_I_ALT)) ? 1U : 0U;

    config_valid =
        ((ctrl1_xl == IMU_CTRL1_XL_1666HZ_8G) &&
         (ctrl2_g == IMU_CTRL2_G_1666HZ_1000DPS) &&
         ((ctrl3_c & IMU_CTRL3_C_BDU_IF_INC) ==
          IMU_CTRL3_C_BDU_IF_INC) &&
         ((ctrl4_c & IMU_CTRL4_C_I2C_DISABLE) != 0U)) ? 1U : 0U;

    if (whoami_valid == 0U)
    {
        imu_stale_fast_config_repair_failure_count++;
        duration_us = (uint32_t)(micros() - start_us);
        imu_stale_fast_config_repair_last_duration_us = duration_us;
        if (duration_us > imu_stale_fast_config_repair_max_duration_us)
        {
            imu_stale_fast_config_repair_max_duration_us = duration_us;
        }
        return IMU_FAST_CONFIG_REPAIR_ESCALATE;
    }

    if (config_valid != 0U)
    {
        /* Healthy registers: the soft SPI re-sync already performed at stale
         * detection is the only intervention. The next triplet must actually
         * change; otherwise the one-shot stale retry escalates immediately. */
        duration_us = (uint32_t)(micros() - start_us);
        imu_stale_fast_config_repair_last_duration_us = duration_us;
        if (duration_us > imu_stale_fast_config_repair_max_duration_us)
        {
            imu_stale_fast_config_repair_max_duration_us = duration_us;
        }
        return IMU_FAST_CONFIG_REPAIR_CONTINUE;
    }

    imu_register_error_count++;
    imu_stale_fast_config_repair_attempt_count++;

    IMU_WriteReg(IMU_REG_CTRL3_C, IMU_CTRL3_C_BDU_IF_INC);
    IMU_WriteReg(IMU_REG_CTRL4_C,
                 (uint8_t)(ctrl4_c | IMU_CTRL4_C_I2C_DISABLE));
    IMU_WriteReg(IMU_REG_CTRL1_XL, IMU_CTRL1_XL_1666HZ_8G);
    IMU_WriteReg(IMU_REG_CTRL2_G, IMU_CTRL2_G_1666HZ_1000DPS);

    whoami = IMU_ReadReg(IMU_REG_WHO_AM_I);
    ctrl1_xl = IMU_ReadReg(IMU_REG_CTRL1_XL);
    ctrl2_g = IMU_ReadReg(IMU_REG_CTRL2_G);
    ctrl3_c = IMU_ReadReg(IMU_REG_CTRL3_C);
    ctrl4_c = IMU_ReadReg(IMU_REG_CTRL4_C);

    imu_drv_last_whoami = whoami;
    imu_drv_ctrl1_xl_readback = ctrl1_xl;
    imu_drv_ctrl2_g_readback = ctrl2_g;
    imu_drv_ctrl3_c_readback = ctrl3_c;
    imu_drv_ctrl4_c_readback = ctrl4_c;
    imu_drv_i2c_disable_ok =
        ((ctrl4_c & IMU_CTRL4_C_I2C_DISABLE) != 0U) ? 1U : 0U;
    imu_drv_bdu_if_inc_ok =
        ((ctrl3_c & IMU_CTRL3_C_BDU_IF_INC) == IMU_CTRL3_C_BDU_IF_INC)
        ? 1U : 0U;
    imu_drv_fullscale_config_ok =
        (((whoami == IMU_WHO_AM_I_ISM330DLC) ||
          (whoami == IMU_WHO_AM_I_ALT)) &&
         (ctrl1_xl == IMU_CTRL1_XL_1666HZ_8G) &&
         (ctrl2_g == IMU_CTRL2_G_1666HZ_1000DPS) &&
         (imu_drv_bdu_if_inc_ok != 0U) &&
         (imu_drv_i2c_disable_ok != 0U)) ? 1U : 0U;

    duration_us = (uint32_t)(micros() - start_us);
    imu_stale_fast_config_repair_last_duration_us = duration_us;
    if (duration_us > imu_stale_fast_config_repair_max_duration_us)
    {
        imu_stale_fast_config_repair_max_duration_us = duration_us;
    }

    if (imu_drv_fullscale_config_ok != 0U)
    {
        uint32_t warmup_now_us = micros();

        imu_stale_fast_config_repair_success_count++;
        imu_last_register_check_timestamp_us = warmup_now_us;

        /* P57: CTRL1/CTRL2 were observed in power-down and have just been
         * restored. Do not assume a fixed 1 ms turn-on time. Arm a bounded
         * non-blocking warmup state; subsequent 1 kHz calls poll STATUS_REG
         * until both XLDA and GDA are asserted, then attempt a validated
         * triplet. No HAL_Delay and no busy-wait are used. */
        imu_stale_warmup_active = 1U;
        imu_stale_warmup_active_debug = 1U;
        imu_stale_warmup_first_ready_recorded = 0U;
        imu_stale_warmup_start_us = warmup_now_us;
        imu_stale_warmup_last_status = 0U;
        imu_stale_warmup_event_count++;
        return IMU_FAST_CONFIG_REPAIR_DEFER;
    }

    imu_stale_fast_config_repair_failure_count++;
    imu_drv_error_count++;
    return IMU_FAST_CONFIG_REPAIR_ESCALATE;
}

static uint8_t IMU_IsStaleSample(
    const IMU_RawData_t *raw,
    uint32_t now_us
)
{
    if (raw == 0)
    {
        return 1U;
    }

    if (imu_stale_reference_initialized == 0U)
    {
        imu_stale_reference_raw = *raw;
        imu_last_raw_change_timestamp_us = now_us;
        imu_stale_reference_initialized = 1U;
        return 0U;
    }

    if (IMU_RawDataEqual(raw, &imu_stale_reference_raw) == 0U)
    {
        imu_stale_reference_raw = *raw;
        imu_last_raw_change_timestamp_us = now_us;
        return 0U;
    }

    return (((uint32_t)(now_us - imu_last_raw_change_timestamp_us)) >=
            IMU_STALE_TIMEOUT_US)
           ? 1U
           : 0U;
}

static void IMU_RecordInvalidSample(uint32_t now_us)
{
    imu_last_sample_valid = 0U;
    imu_sample_valid = 0U;
    imu_invalid_sample_count++;
    imu_consecutive_invalid_sample_count++;
    imu_last_invalid_sample_timestamp_us = now_us;
    imu_recovery_valid_streak = 0U;

    if (imu_consecutive_invalid_sample_count >
        imu_max_consecutive_invalid_sample_count)
    {
        imu_max_consecutive_invalid_sample_count =
            imu_consecutive_invalid_sample_count;
    }

    if ((imu_consecutive_invalid_sample_count >=
         IMU_RECOVERY_INVALID_LIMIT) &&
        (imu_recovery_state != IMU_RECOVERY_STATE_RECOVERING))
    {
        imu_recovery_state = IMU_RECOVERY_STATE_REQUESTED;
    }
}

static int16_t IMU_SelectContinuousWord(
    int16_t voted_value,
    int32_t reference_raw,
    volatile uint32_t *axis_correction_count
)
{
    const int16_t alternate_value = IMU_ToggleBit8(voted_value);
    const int32_t voted_error =
        IMU_Abs32((int32_t)voted_value - reference_raw);
    const int32_t alternate_error =
        IMU_Abs32((int32_t)alternate_value - reference_raw);

    if ((alternate_error + IMU_CONTINUITY_SWITCH_MARGIN_RAW) < voted_error)
    {
        imu_bit8_correction_count++;

        if (axis_correction_count != 0)
        {
            (*axis_correction_count)++;
        }

        return alternate_value;
    }

    return voted_value;
}

static int32_t IMU_PredictAxisRaw(
    int16_t previous_raw,
    int16_t previous_previous_raw
)
{
    /*
     * Use the last accepted value, not a derivative extrapolation. The bus
     * error is exactly 256 counts; derivative prediction can land at the
     * 128-count midpoint and latch onto the wrong band after one noisy sample.
     */
    (void)previous_previous_raw;
    return (int32_t)previous_raw;
}

static int32_t IMU_MinBit8EquivalentError(
    int16_t measured,
    int16_t accepted
)
{
    const int32_t direct_error =
        IMU_Abs32((int32_t)measured - (int32_t)accepted);
    const int32_t toggled_error =
        IMU_Abs32((int32_t)IMU_ToggleBit8(measured) -
                  (int32_t)accepted);

    return (direct_error < toggled_error) ?
           direct_error :
           toggled_error;
}

static uint8_t IMU_ValidateRedundantAxis(
    int16_t a,
    int16_t b,
    int16_t c,
    int16_t accepted,
    int32_t residual_limit_raw
)
{
    if ((IMU_MinBit8EquivalentError(a, accepted) > residual_limit_raw) ||
        (IMU_MinBit8EquivalentError(b, accepted) > residual_limit_raw) ||
        (IMU_MinBit8EquivalentError(c, accepted) > residual_limit_raw))
    {
        return 0U;
    }

    return 1U;
}

static uint8_t IMU_CommitValidatedTriplet(
    const IMU_RawData_t samples[IMU_REDUNDANT_BURST_COUNT]
)
{
    IMU_RawData_t voted;
    IMU_RawData_t corrected;
    uint32_t now_us;
    int32_t gyro_reference[3];
    int32_t accel_reference[3];

    if (samples == 0)
    {
        return 0U;
    }

    imu_last_commit_pattern_error = 0U;

    voted.gyro_x_raw = IMU_Median3Int16(
        samples[0].gyro_x_raw,
        samples[1].gyro_x_raw,
        samples[2].gyro_x_raw);
    voted.gyro_y_raw = IMU_Median3Int16(
        samples[0].gyro_y_raw,
        samples[1].gyro_y_raw,
        samples[2].gyro_y_raw);
    voted.gyro_z_raw = IMU_Median3Int16(
        samples[0].gyro_z_raw,
        samples[1].gyro_z_raw,
        samples[2].gyro_z_raw);

    voted.accel_x_raw = IMU_Median3Int16(
        samples[0].accel_x_raw,
        samples[1].accel_x_raw,
        samples[2].accel_x_raw);
    voted.accel_y_raw = IMU_Median3Int16(
        samples[0].accel_y_raw,
        samples[1].accel_y_raw,
        samples[2].accel_y_raw);
    voted.accel_z_raw = IMU_Median3Int16(
        samples[0].accel_z_raw,
        samples[1].accel_z_raw,
        samples[2].accel_z_raw);

    imu_last_triplet_gyro_x[0] = samples[0].gyro_x_raw;
    imu_last_triplet_gyro_x[1] = samples[1].gyro_x_raw;
    imu_last_triplet_gyro_x[2] = samples[2].gyro_x_raw;
    imu_last_triplet_accel_z[0] = samples[0].accel_z_raw;
    imu_last_triplet_accel_z[1] = samples[1].accel_z_raw;
    imu_last_triplet_accel_z[2] = samples[2].accel_z_raw;

    if ((samples[0].gyro_x_raw != samples[1].gyro_x_raw) ||
        (samples[1].gyro_x_raw != samples[2].gyro_x_raw) ||
        (samples[0].gyro_y_raw != samples[1].gyro_y_raw) ||
        (samples[1].gyro_y_raw != samples[2].gyro_y_raw) ||
        (samples[0].gyro_z_raw != samples[1].gyro_z_raw) ||
        (samples[1].gyro_z_raw != samples[2].gyro_z_raw) ||
        (samples[0].accel_x_raw != samples[1].accel_x_raw) ||
        (samples[1].accel_x_raw != samples[2].accel_x_raw) ||
        (samples[0].accel_y_raw != samples[1].accel_y_raw) ||
        (samples[1].accel_y_raw != samples[2].accel_y_raw) ||
        (samples[0].accel_z_raw != samples[1].accel_z_raw) ||
        (samples[1].accel_z_raw != samples[2].accel_z_raw))
    {
        imu_redundant_disagreement_count++;
    }

    if (imu_continuity_initialized != 0U)
    {
        gyro_reference[0] = IMU_PredictAxisRaw(
            imu_previous_raw.gyro_x_raw,
            imu_previous_previous_raw.gyro_x_raw);
        gyro_reference[1] = IMU_PredictAxisRaw(
            imu_previous_raw.gyro_y_raw,
            imu_previous_previous_raw.gyro_y_raw);
        gyro_reference[2] = IMU_PredictAxisRaw(
            imu_previous_raw.gyro_z_raw,
            imu_previous_previous_raw.gyro_z_raw);

        accel_reference[0] = IMU_PredictAxisRaw(
            imu_previous_raw.accel_x_raw,
            imu_previous_previous_raw.accel_x_raw);
        accel_reference[1] = IMU_PredictAxisRaw(
            imu_previous_raw.accel_y_raw,
            imu_previous_previous_raw.accel_y_raw);
        accel_reference[2] = IMU_PredictAxisRaw(
            imu_previous_raw.accel_z_raw,
            imu_previous_previous_raw.accel_z_raw);
    }
    else
    {
        /* Connector calibration pose: stationary, sensor +Z approximately +1 g. */
        gyro_reference[0] = 0L;
        gyro_reference[1] = 0L;
        gyro_reference[2] = 0L;
        accel_reference[0] = 0L;
        accel_reference[1] = 0L;
        accel_reference[2] =
            (int32_t)((1.0f / APP_IMU_ACCEL_SCALE_G) + 0.5f);
    }

    corrected.gyro_x_raw = IMU_SelectContinuousWord(
        voted.gyro_x_raw,
        gyro_reference[0],
        &imu_bit8_correction_gyro_x_count);
    corrected.gyro_y_raw = IMU_SelectContinuousWord(
        voted.gyro_y_raw,
        gyro_reference[1],
        &imu_bit8_correction_gyro_y_count);
    corrected.gyro_z_raw = IMU_SelectContinuousWord(
        voted.gyro_z_raw,
        gyro_reference[2],
        &imu_bit8_correction_gyro_z_count);

    corrected.accel_x_raw = IMU_SelectContinuousWord(
        voted.accel_x_raw,
        accel_reference[0],
        &imu_bit8_correction_accel_x_count);
    corrected.accel_y_raw = IMU_SelectContinuousWord(
        voted.accel_y_raw,
        accel_reference[1],
        &imu_bit8_correction_accel_y_count);
    corrected.accel_z_raw = IMU_SelectContinuousWord(
        voted.accel_z_raw,
        accel_reference[2],
        &imu_bit8_correction_accel_z_count);

    if ((corrected.gyro_x_raw == 0) &&
        (corrected.gyro_y_raw == 0) &&
        (corrected.gyro_z_raw == 0) &&
        (corrected.accel_x_raw == 0) &&
        (corrected.accel_y_raw == 0) &&
        (corrected.accel_z_raw == 0))
    {
        return 0U;
    }

    if (IMU_HasRepeatedWordPattern(&corrected) != 0U)
    {
        IMU_CapturePatternDiagnostic(samples, &corrected);
        imu_pattern_error_count++;
        imu_last_commit_pattern_error = 1U;
        return 0U;
    }

    if ((IMU_ValidateRedundantAxis(
             samples[0].gyro_x_raw,
             samples[1].gyro_x_raw,
             samples[2].gyro_x_raw,
             corrected.gyro_x_raw,
             IMU_REDUNDANT_GYRO_RESIDUAL_MAX_RAW) == 0U) ||
        (IMU_ValidateRedundantAxis(
             samples[0].gyro_y_raw,
             samples[1].gyro_y_raw,
             samples[2].gyro_y_raw,
             corrected.gyro_y_raw,
             IMU_REDUNDANT_GYRO_RESIDUAL_MAX_RAW) == 0U) ||
        (IMU_ValidateRedundantAxis(
             samples[0].gyro_z_raw,
             samples[1].gyro_z_raw,
             samples[2].gyro_z_raw,
             corrected.gyro_z_raw,
             IMU_REDUNDANT_GYRO_RESIDUAL_MAX_RAW) == 0U) ||
        (IMU_ValidateRedundantAxis(
             samples[0].accel_x_raw,
             samples[1].accel_x_raw,
             samples[2].accel_x_raw,
             corrected.accel_x_raw,
             IMU_REDUNDANT_ACCEL_RESIDUAL_MAX_RAW) == 0U) ||
        (IMU_ValidateRedundantAxis(
             samples[0].accel_y_raw,
             samples[1].accel_y_raw,
             samples[2].accel_y_raw,
             corrected.accel_y_raw,
             IMU_REDUNDANT_ACCEL_RESIDUAL_MAX_RAW) == 0U) ||
        (IMU_ValidateRedundantAxis(
             samples[0].accel_z_raw,
             samples[1].accel_z_raw,
             samples[2].accel_z_raw,
             corrected.accel_z_raw,
             IMU_REDUNDANT_ACCEL_RESIDUAL_MAX_RAW) == 0U))
    {
        return 0U;
    }

    now_us = micros();

    if (IMU_IsStaleSample(&corrected, now_us) != 0U)
    {
        /* P56: do not jump straight to the ~50 ms sensor reset. Arm exactly
         * one deferred short-path attempt. The frozen sample itself remains
         * quarantined and is never published to ESKF/control. A failed retry
         * is still the same stale episode, so the public stale counter is not
         * incremented twice. */
        if (imu_stale_retry_pending == 0U)
        {
            imu_stale_count++;
            imu_stale_diagnostic.valid = 1U;
            imu_stale_diagnostic.event_count = imu_stale_count;
            imu_stale_diagnostic.timestamp_us = now_us;
            imu_stale_diagnostic.stale_age_us =
                (uint32_t)(now_us - imu_last_raw_change_timestamp_us);
            imu_stale_diagnostic.register_snapshot_valid = 0U;
            imu_stale_diagnostic.whoami = 0U;
            imu_stale_diagnostic.ctrl1_xl = 0U;
            imu_stale_diagnostic.ctrl2_g = 0U;
            imu_stale_diagnostic.ctrl3_c = 0U;
            imu_stale_diagnostic.ctrl4_c = 0U;

            IMU_SPI_ResynchronizeBus();
            imu_stale_retry_pending = 1U;
            imu_stale_register_snapshot_pending = 1U;
        }

        return 0U;
    }

    imu_previous_previous_raw = imu_previous_raw;
    imu_previous_raw = corrected;
    imu_continuity_initialized = 1U;

    imu_cached_raw = corrected;
    imu_last_sample_timestamp_us = now_us;
    imu_sample_generation++;

    imu_valid_sample_count++;
    imu_consecutive_invalid_sample_count = 0UL;
    imu_last_valid_sample_timestamp_us = now_us;
    imu_redundant_set_count++;

    if (imu_recovery_state == IMU_RECOVERY_STATE_VALIDATING)
    {
        if (imu_recovery_valid_streak < 255U)
        {
            imu_recovery_valid_streak++;
        }

        if (imu_recovery_valid_streak >= IMU_RECOVERY_VALID_SAMPLE_COUNT)
        {
            imu_recovery_state = IMU_RECOVERY_STATE_NORMAL;
            imu_last_sample_valid = 1U;
            imu_sample_valid = 1U;
        }
        else
        {
            /* Cache is refreshed, but the sample is withheld from the flight
             * stack until a stable run of valid packets has been observed. */
            imu_last_sample_valid = 0U;
            imu_sample_valid = 0U;
        }
    }
    else
    {
        imu_last_sample_valid = 1U;
        imu_sample_valid = 1U;
    }

    return 1U;
}

static uint8_t IMU_ReadOneRawBurstDMA(IMU_RawData_t *sample)
{
    uint8_t tx[IMU_DMA_RAW_LENGTH] = {0U};
    uint8_t rx[IMU_DMA_RAW_LENGTH] = {0U};

    if (sample == 0)
    {
        return 0U;
    }

    tx[0] = IMU_REG_OUTX_L_G | 0x80U;

    if (IMU_DMA_TransferBlocking(
            tx,
            rx,
            IMU_DMA_RAW_LENGTH,
            IMU_SPI_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }

    for (uint8_t i = 0U; i < IMU_DMA_RAW_LENGTH; i++)
    {
        imu_dma_rx[i] = rx[i];
    }

    IMU_ParseRawBufferToStruct(&rx[1], sample);
    imu_redundant_burst_read_count++;

    return 1U;
}

static uint8_t IMU_AcquireValidatedSample(void)
{
    IMU_RawData_t samples[IMU_REDUNDANT_BURST_COUNT];
    uint8_t stale_retry_was_pending;

    if (imu_stale_register_snapshot_pending != 0U)
    {
        uint32_t now_us = micros();
        uint8_t fast_repair_result;

        fast_repair_result = IMU_ServiceStaleFastConfigRepair();
        imu_stale_register_snapshot_pending = 0U;

        if (fast_repair_result == IMU_FAST_CONFIG_REPAIR_ESCALATE)
        {
            imu_stale_retry_pending = 0U;
            imu_stale_recovery_escalation_count++;
            IMU_StartRecovery(now_us);
            IMU_RecordInvalidSample(now_us);
            IMU_UpdateLiveDebug();
            return 0U;
        }

        if (fast_repair_result == IMU_FAST_CONFIG_REPAIR_DEFER)
        {
            /* P57 warmup is now armed. Return immediately; subsequent 1 kHz
             * calls poll STATUS_REG without blocking until fresh data is ready. */
            IMU_RecordInvalidSample(now_us);
            IMU_UpdateLiveDebug();
            return 0U;
        }
    }

    /* P57: after a successful stale register repair, wait for hardware
     * DATA_READY rather than retrying after one arbitrary scheduler tick.
     * One STATUS_REG read is performed per 1 kHz call. The raw sample remains
     * quarantined throughout the warmup. */
    if (imu_stale_warmup_active != 0U)
    {
        uint32_t now_us = micros();
        uint32_t elapsed_us = (uint32_t)(now_us - imu_stale_warmup_start_us);
        uint8_t status = IMU_ReadReg(IMU_REG_STATUS_REG);

        imu_stale_warmup_poll_count++;
        imu_stale_warmup_last_status = status;
        imu_drv_status_reg = status;

        if ((status & (IMU_STATUS_XLDA | IMU_STATUS_GDA)) !=
            (IMU_STATUS_XLDA | IMU_STATUS_GDA))
        {
            if (elapsed_us >= IMU_STALE_WARMUP_TIMEOUT_US)
            {
                imu_stale_warmup_timeout_count++;
                imu_stale_warmup_active = 0U;
                imu_stale_warmup_active_debug = 0U;
                imu_stale_retry_pending = 0U;
                imu_stale_register_snapshot_pending = 0U;
                imu_stale_recovery_escalation_count++;
                IMU_StartRecovery(now_us);
            }

            IMU_RecordInvalidSample(now_us);
            IMU_UpdateLiveDebug();
            return 0U;
        }

        if (imu_stale_warmup_first_ready_recorded == 0U)
        {
            imu_stale_warmup_first_ready_recorded = 1U;
            imu_stale_warmup_first_ready_us = elapsed_us;
            if (elapsed_us > imu_stale_warmup_max_ready_us)
            {
                imu_stale_warmup_max_ready_us = elapsed_us;
            }
        }
        /* Keep warmup_active set until a genuinely changed validated triplet
         * is committed. If DATA_READY was asserted for an unchanged output,
         * the next scheduler call may try again until the same bounded timeout. */
    }

    stale_retry_was_pending = imu_stale_retry_pending;

    if (imu_pattern_register_snapshot_pending != 0U)
    {
        uint32_t now_us = micros();
        uint8_t fast_repair_result;

        fast_repair_result = IMU_ServicePatternFastConfigRepair();
        imu_pattern_register_snapshot_pending = 0U;

        if (fast_repair_result == IMU_FAST_CONFIG_REPAIR_ESCALATE)
        {
            /* P55: WHO_AM_I loss or failed register verification is not kept in
             * the short path.  Fall straight into the existing bounded full
             * recovery state machine; its first reset step runs next call. */
            imu_pattern_retry_pending = 0U;
            imu_pattern_recovery_escalation_count++;
            IMU_StartRecovery(now_us);
            IMU_RecordInvalidSample(now_us);
            IMU_UpdateLiveDebug();
            return 0U;
        }
    }

    for (uint8_t i = 0U; i < IMU_REDUNDANT_BURST_COUNT; i++)
    {
        if (IMU_ReadOneRawBurstDMA(&samples[i]) == 0U)
        {
            uint32_t now_us = micros();

            if (stale_retry_was_pending != 0U)
            {
                imu_stale_retry_pending = 0U;
                imu_stale_register_snapshot_pending = 0U;
                imu_stale_warmup_active = 0U;
                imu_stale_warmup_active_debug = 0U;
                imu_stale_recovery_escalation_count++;
                IMU_StartRecovery(now_us);
            }

            IMU_RecordInvalidSample(now_us);
            imu_redundant_reject_count++;

            IMU_UpdateLiveDebug();
            return 0U;
        }
    }

    if (IMU_CommitValidatedTriplet(samples) == 0U)
    {
        /* P57: a DATA_READY indication may precede a genuinely changed
         * six-axis output by a scheduler tick. During the bounded warmup only
         * an ordinary unchanged/invalid triplet is allowed to retry; a
         * repeated-word corruption or timeout still escalates immediately. */
        if (stale_retry_was_pending != 0U)
        {
            uint32_t retry_now_us = micros();
            uint32_t warmup_elapsed_us =
                (uint32_t)(retry_now_us - imu_stale_warmup_start_us);

            if ((imu_stale_warmup_active != 0U) &&
                (imu_last_commit_pattern_error == 0U) &&
                (warmup_elapsed_us < IMU_STALE_WARMUP_TIMEOUT_US))
            {
                /* Keep the stale episode armed. Next 1 kHz call polls
                 * STATUS_REG again and retries only when XLDA|GDA is ready. */
            }
            else
            {
                if ((imu_stale_warmup_active != 0U) &&
                    (warmup_elapsed_us >= IMU_STALE_WARMUP_TIMEOUT_US))
                {
                    imu_stale_warmup_timeout_count++;
                }
                imu_stale_retry_pending = 0U;
                imu_stale_register_snapshot_pending = 0U;
                imu_stale_warmup_active = 0U;
                imu_stale_warmup_active_debug = 0U;
                imu_stale_recovery_escalation_count++;
                IMU_StartRecovery(retry_now_us);
            }
        }
        /* P44: do not perform a second 3-burst transaction in this task call.
         * Re-establish the SPI transaction boundary, then arm one deferred
         * retry for the next 1 ms IMU task. */
        else if (imu_last_commit_pattern_error != 0U)
        {
            if (imu_pattern_retry_pending == 0U)
            {
                IMU_SPI_ResynchronizeBus();
                imu_pattern_retry_pending = 1U;
                imu_pattern_retry_count++;
            }
            else
            {
                /* P45: a second consecutive bad triplet is not soft-resynced
                 * again. Start the existing full recovery state machine now;
                 * the first bus-reset step is still serviced non-blockingly on
                 * the next 1 kHz call. */
                imu_pattern_retry_pending = 0U;
                imu_pattern_recovery_escalation_count++;
                IMU_StartRecovery(micros());
            }
        }
        else if (imu_pattern_retry_pending != 0U)
        {
            /* P45: any bad second sample after the one allowed soft-resync is
             * an immediate escalation to the full non-blocking recovery path. */
            imu_pattern_retry_pending = 0U;
            imu_pattern_recovery_escalation_count++;
            IMU_StartRecovery(micros());
        }

        IMU_RecordInvalidSample(micros());
        imu_redundant_reject_count++;

        if ((samples[0].gyro_x_raw == 0) &&
            (samples[0].gyro_y_raw == 0) &&
            (samples[0].gyro_z_raw == 0) &&
            (samples[0].accel_x_raw == 0) &&
            (samples[0].accel_y_raw == 0) &&
            (samples[0].accel_z_raw == 0))
        {
            imu_all_zero_sample_count++;
        }

        IMU_UpdateLiveDebug();
        return 0U;
    }

    if (stale_retry_was_pending != 0U)
    {
        imu_stale_retry_pending = 0U;
        imu_stale_register_snapshot_pending = 0U;
        if (imu_stale_warmup_active != 0U)
        {
            imu_stale_warmup_success_count++;
        }
        imu_stale_warmup_active = 0U;
        imu_stale_warmup_active_debug = 0U;
        imu_stale_retry_success_count++;
    }

    if (imu_pattern_retry_pending != 0U)
    {
        imu_pattern_retry_pending = 0U;
        imu_pattern_retry_success_count++;
    }

    IMU_UpdateLiveDebug();
    return 1U;
}
/* -------------------------------------------------------------------------- */
/* Register access                                                            */
/* -------------------------------------------------------------------------- */

static uint8_t IMU_ReadReg(uint8_t reg)
{
    uint8_t tx[2] = {0U, 0U};
    uint8_t rx[2] = {0U, 0U};

    imu_drv_last_reg = reg;

    tx[0] = reg | 0x80U;
    tx[1] = 0x00U;

    if (IMU_PollingTransfer(tx, rx, 2U) != HAL_OK)
    {
        return 0U;
    }

    imu_drv_last_rx0 = rx[0];
    imu_drv_last_rx1 = rx[1];
    imu_drv_last_value = rx[1];

    imu_drv_read_count++;
    imu_spi_poll_read_count++;

    IMU_UpdateLiveDebug();

    return rx[1];
}

static void IMU_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {0U, 0U};
    uint8_t rx[2] = {0U, 0U};

    imu_drv_last_reg = reg;
    imu_drv_last_value = value;

    tx[0] = reg & 0x7FU;
    tx[1] = value;

    if (IMU_PollingTransfer(tx, rx, 2U) != HAL_OK)
    {
        return;
    }

    imu_drv_last_rx0 = rx[0];
    imu_drv_last_rx1 = rx[1];

    imu_drv_write_count++;

    IMU_UpdateLiveDebug();
}

static void IMU_ReadMultiBlocking(uint8_t start_reg, uint8_t *buffer, uint8_t length)
{
    uint8_t tx[IMU_DMA_MAX_LENGTH] = {0U};
    uint8_t rx[IMU_DMA_MAX_LENGTH] = {0U};

    if ((buffer == 0) || (length == 0U) || (length > 32U))
    {
        imu_drv_error_count++;
        return;
    }

    imu_drv_last_reg = start_reg;

    tx[0] = start_reg | 0x80U;

    for (uint8_t i = 1U; i <= length; i++)
    {
        tx[i] = 0x00U;
    }

    if (IMU_DMA_TransferBlocking(
            tx,
            rx,
            (uint16_t)(length + 1U),
            IMU_SPI_TIMEOUT_MS) != HAL_OK)
    {
        return;
    }

    for (uint8_t i = 0U; i < length; i++)
    {
        buffer[i] = rx[i + 1U];
    }

    imu_drv_last_rx0 = buffer[0];
    imu_drv_last_rx1 = buffer[1];

    imu_drv_read_count++;

    IMU_UpdateLiveDebug();
}

static void IMU_StartRawDMARead(void)
{
    if (imu_dma_busy != 0U)
    {
        imu_dma_busy_count++;
        return;
    }

    for (uint8_t i = 0U; i < IMU_DMA_RAW_LENGTH; i++)
    {
        imu_dma_tx[i] = 0x00U;
        imu_dma_rx[i] = 0x00U;
    }

    imu_dma_tx[0] = IMU_REG_OUTX_L_G | 0x80U;

    (void)IMU_DMA_StartTransfer(
        imu_dma_tx,
        imu_dma_rx,
        IMU_DMA_RAW_LENGTH,
        IMU_DMA_TRANSFER_RAW
    );
}

/* -------------------------------------------------------------------------- */
/* Reset and verified configuration                                           */
/* -------------------------------------------------------------------------- */

static uint8_t IMU_ResetAndConfigure(void)
{
    for (uint8_t attempt = 0U; attempt < 3U; attempt++)
    {
        uint8_t reset_cleared = 0U;

        /* A debugger reset does not power-cycle the sensor. Restore a known
         * register state first, then configure BDU/auto-increment explicitly. */
        IMU_WriteReg(IMU_REG_CTRL3_C, IMU_CTRL3_C_SW_RESET);
        imu_drv_reset_count++;

        for (uint8_t poll = 0U; poll < 50U; poll++)
        {
            HAL_Delay(1U);
            imu_drv_ctrl3_c_readback = IMU_ReadReg(IMU_REG_CTRL3_C);

            if ((imu_drv_ctrl3_c_readback & IMU_CTRL3_C_SW_RESET) == 0U)
            {
                reset_cleared = 1U;
                break;
            }
        }

        if (reset_cleared == 0U)
        {
            imu_drv_config_retry_count++;
            continue;
        }

        IMU_WriteReg(IMU_REG_CTRL3_C, IMU_CTRL3_C_BDU_IF_INC);

        /* Disable the unused I2C front-end while operating in 4-wire SPI. */
        imu_drv_ctrl4_c_readback = IMU_ReadReg(IMU_REG_CTRL4_C);
        IMU_WriteReg(
            IMU_REG_CTRL4_C,
            (uint8_t)(imu_drv_ctrl4_c_readback |
                      IMU_CTRL4_C_I2C_DISABLE)
        );

        IMU_WriteReg(IMU_REG_CTRL1_XL, IMU_CTRL1_XL_1666HZ_8G);
        IMU_WriteReg(IMU_REG_CTRL2_G, IMU_CTRL2_G_1666HZ_1000DPS);

        /* Sensor turn-on time is specified in milliseconds. */
        HAL_Delay(40U);

        imu_drv_ctrl3_c_readback = IMU_ReadReg(IMU_REG_CTRL3_C);
        imu_drv_ctrl4_c_readback = IMU_ReadReg(IMU_REG_CTRL4_C);
        imu_drv_ctrl1_xl_readback = IMU_ReadReg(IMU_REG_CTRL1_XL);
        imu_drv_ctrl2_g_readback = IMU_ReadReg(IMU_REG_CTRL2_G);
        imu_drv_status_reg = IMU_ReadReg(IMU_REG_STATUS_REG);

        imu_drv_i2c_disable_ok =
            ((imu_drv_ctrl4_c_readback & IMU_CTRL4_C_I2C_DISABLE) != 0U)
            ? 1U
            : 0U;

        imu_drv_bdu_if_inc_ok =
            ((imu_drv_ctrl3_c_readback & IMU_CTRL3_C_BDU_IF_INC) ==
             IMU_CTRL3_C_BDU_IF_INC)
            ? 1U
            : 0U;

        imu_drv_fullscale_config_ok =
            ((imu_drv_ctrl1_xl_readback == IMU_CTRL1_XL_1666HZ_8G) &&
             (imu_drv_ctrl2_g_readback == IMU_CTRL2_G_1666HZ_1000DPS) &&
             (imu_drv_bdu_if_inc_ok != 0U) &&
             (imu_drv_i2c_disable_ok != 0U))
            ? 1U
            : 0U;

        if (imu_drv_fullscale_config_ok != 0U)
        {
            return 1U;
        }

        imu_drv_config_retry_count++;
    }

    return 0U;
}

static uint8_t IMU_CheckRuntimeRegisters(void)
{
    uint8_t whoami;
    uint8_t ctrl1_xl;
    uint8_t ctrl2_g;
    uint8_t ctrl3_c;
    uint8_t ctrl4_c;

    whoami = IMU_ReadReg(IMU_REG_WHO_AM_I);
    ctrl1_xl = IMU_ReadReg(IMU_REG_CTRL1_XL);
    ctrl2_g = IMU_ReadReg(IMU_REG_CTRL2_G);
    ctrl3_c = IMU_ReadReg(IMU_REG_CTRL3_C);
    ctrl4_c = IMU_ReadReg(IMU_REG_CTRL4_C);

    imu_drv_last_whoami = whoami;
    imu_drv_ctrl1_xl_readback = ctrl1_xl;
    imu_drv_ctrl2_g_readback = ctrl2_g;
    imu_drv_ctrl3_c_readback = ctrl3_c;
    imu_drv_ctrl4_c_readback = ctrl4_c;

    if (((whoami != IMU_WHO_AM_I_ISM330DLC) &&
         (whoami != IMU_WHO_AM_I_ALT)) ||
        (ctrl1_xl != IMU_CTRL1_XL_1666HZ_8G) ||
        (ctrl2_g != IMU_CTRL2_G_1666HZ_1000DPS) ||
        ((ctrl3_c & IMU_CTRL3_C_BDU_IF_INC) !=
         IMU_CTRL3_C_BDU_IF_INC) ||
        ((ctrl4_c & IMU_CTRL4_C_I2C_DISABLE) == 0U))
    {
        imu_register_error_count++;
        imu_drv_error_count++;
        return 0U;
    }

    return 1U;
}

static void IMU_RecoveryFinishDuration(uint32_t now_us)
{
    uint32_t duration_us = (uint32_t)(now_us - imu_recovery_start_timestamp_us);
    imu_recovery_last_duration_us = duration_us;
    if (duration_us > imu_recovery_max_duration_us)
    {
        imu_recovery_max_duration_us = duration_us;
    }
}

static void IMU_RecoveryAttemptFailed(uint32_t now_us)
{
    imu_recovery_failure_count++;
    imu_drv_config_retry_count++;

    if (imu_recovery_attempt_in_cycle < IMU_RECOVERY_MAX_ATTEMPTS)
    {
        /* Try again on a later 1 kHz service call; never loop here. */
        imu_recovery_step = IMU_RECOVERY_STEP_BUS_RESET;
        imu_recovery_step_debug = imu_recovery_step;
        imu_recovery_step_deadline_us = 0UL;
        imu_recovery_next_poll_us = 0UL;
        return;
    }

    imu_recovery_state = IMU_RECOVERY_STATE_FAILED;
    imu_recovery_step = IMU_RECOVERY_STEP_IDLE;
    imu_recovery_step_debug = imu_recovery_step;
    imu_recovery_retry_timestamp_us = now_us;
    IMU_RecoveryFinishDuration(now_us);
}

static void IMU_StartRecovery(uint32_t now_us)
{
    imu_recovery_state = IMU_RECOVERY_STATE_RECOVERING;
    imu_recovery_step = IMU_RECOVERY_STEP_BUS_RESET;
    imu_recovery_step_debug = imu_recovery_step;
    imu_recovery_attempt_in_cycle = 0U;
    imu_recovery_step_deadline_us = 0UL;
    imu_recovery_next_poll_us = 0UL;
    imu_recovery_start_timestamp_us = now_us;
    imu_last_sample_valid = 0U;
    imu_sample_valid = 0U;
    imu_recovery_valid_streak = 0U;
    imu_pattern_retry_pending = 0U;
    imu_pattern_register_snapshot_pending = 0U;
    imu_stale_retry_pending = 0U;
    imu_stale_register_snapshot_pending = 0U;
    imu_stale_warmup_active = 0U;
    imu_stale_warmup_active_debug = 0U;
    imu_stale_warmup_first_ready_recorded = 0U;
}

static void IMU_ServiceRecovery(uint32_t now_us)
{
    uint8_t whoami;

    if (imu_recovery_state == IMU_RECOVERY_STATE_REQUESTED)
    {
        IMU_StartRecovery(now_us);
    }
    else if (imu_recovery_state == IMU_RECOVERY_STATE_FAILED)
    {
        if (((uint32_t)(now_us - imu_recovery_retry_timestamp_us)) <
            IMU_RECOVERY_RETRY_DELAY_US)
        {
            return;
        }
        IMU_StartRecovery(now_us);
    }

    if (imu_recovery_state != IMU_RECOVERY_STATE_RECOVERING)
    {
        return;
    }

    switch (imu_recovery_step)
    {
        case IMU_RECOVERY_STEP_BUS_RESET:
            imu_recovery_attempt_in_cycle++;
            imu_recovery_attempt_count++;

            if (imu_swspi_active != 0U)
            {
                IMU_SW_SPI_ConfigureGPIO();
            }
            else
            {
                (void)HAL_SPI_Abort(&hspi1);
                IMU_WaitSPIIdle();
                IMU_CS_HIGH();
            }

            imu_dma_busy = 0U;
            imu_dma_done = 0U;
            imu_dma_error = 0U;
            imu_dma_transfer_type = IMU_DMA_TRANSFER_NONE;

            if ((imu_swspi_active == 0U) &&
                (IMU_HardwareSPI_DMA_Init() == 0U))
            {
                IMU_RecoveryAttemptFailed(now_us);
                break;
            }

            imu_recovery_step = IMU_RECOVERY_STEP_WHOAMI;
            break;

        case IMU_RECOVERY_STEP_WHOAMI:
            whoami = IMU_ReadReg(IMU_REG_WHO_AM_I);
            imu_drv_last_whoami = whoami;

            if ((whoami != IMU_WHO_AM_I_ISM330DLC) &&
                (whoami != IMU_WHO_AM_I_ALT))
            {
                imu_connected = 0U;
                imu_drv_error_count++;
                IMU_RecoveryAttemptFailed(now_us);
                break;
            }

            imu_connected = 1U;
            imu_device_id = whoami;
            imu_recovery_step = IMU_RECOVERY_STEP_RESET_WRITE;
            break;

        case IMU_RECOVERY_STEP_RESET_WRITE:
            IMU_WriteReg(IMU_REG_CTRL3_C, IMU_CTRL3_C_SW_RESET);
            imu_drv_reset_count++;
            imu_recovery_step_deadline_us = now_us + IMU_RECOVERY_RESET_TIMEOUT_US;
            imu_recovery_next_poll_us = now_us + IMU_RECOVERY_RESET_POLL_US;
            imu_recovery_step = IMU_RECOVERY_STEP_RESET_WAIT;
            break;

        case IMU_RECOVERY_STEP_RESET_WAIT:
            if ((int32_t)(now_us - imu_recovery_step_deadline_us) >= 0)
            {
                IMU_RecoveryAttemptFailed(now_us);
                break;
            }

            if ((int32_t)(now_us - imu_recovery_next_poll_us) < 0)
            {
                break;
            }

            imu_recovery_next_poll_us = now_us + IMU_RECOVERY_RESET_POLL_US;
            imu_drv_ctrl3_c_readback = IMU_ReadReg(IMU_REG_CTRL3_C);
            if ((imu_drv_ctrl3_c_readback & IMU_CTRL3_C_SW_RESET) == 0U)
            {
                imu_recovery_step = IMU_RECOVERY_STEP_CONFIGURE;
            }
            break;

        case IMU_RECOVERY_STEP_CONFIGURE:
            IMU_WriteReg(IMU_REG_CTRL3_C, IMU_CTRL3_C_BDU_IF_INC);
            imu_drv_ctrl4_c_readback = IMU_ReadReg(IMU_REG_CTRL4_C);
            IMU_WriteReg(IMU_REG_CTRL4_C,
                         (uint8_t)(imu_drv_ctrl4_c_readback |
                                   IMU_CTRL4_C_I2C_DISABLE));
            IMU_WriteReg(IMU_REG_CTRL1_XL, IMU_CTRL1_XL_1666HZ_8G);
            IMU_WriteReg(IMU_REG_CTRL2_G, IMU_CTRL2_G_1666HZ_1000DPS);
            imu_recovery_step_deadline_us = now_us + IMU_RECOVERY_SETTLE_US;
            imu_recovery_step = IMU_RECOVERY_STEP_SETTLE_WAIT;
            break;

        case IMU_RECOVERY_STEP_SETTLE_WAIT:
            if ((int32_t)(now_us - imu_recovery_step_deadline_us) >= 0)
            {
                imu_recovery_step = IMU_RECOVERY_STEP_VERIFY;
            }
            break;

        case IMU_RECOVERY_STEP_VERIFY:
            whoami = IMU_ReadReg(IMU_REG_WHO_AM_I);
            imu_drv_last_whoami = whoami;
            imu_drv_ctrl3_c_readback = IMU_ReadReg(IMU_REG_CTRL3_C);
            imu_drv_ctrl4_c_readback = IMU_ReadReg(IMU_REG_CTRL4_C);
            imu_drv_ctrl1_xl_readback = IMU_ReadReg(IMU_REG_CTRL1_XL);
            imu_drv_ctrl2_g_readback = IMU_ReadReg(IMU_REG_CTRL2_G);
            imu_drv_status_reg = IMU_ReadReg(IMU_REG_STATUS_REG);

            imu_drv_i2c_disable_ok =
                ((imu_drv_ctrl4_c_readback & IMU_CTRL4_C_I2C_DISABLE) != 0U)
                ? 1U : 0U;
            imu_drv_bdu_if_inc_ok =
                ((imu_drv_ctrl3_c_readback & IMU_CTRL3_C_BDU_IF_INC) ==
                 IMU_CTRL3_C_BDU_IF_INC) ? 1U : 0U;
            imu_drv_fullscale_config_ok =
                (((whoami == IMU_WHO_AM_I_ISM330DLC) ||
                  (whoami == IMU_WHO_AM_I_ALT)) &&
                 (imu_drv_ctrl1_xl_readback == IMU_CTRL1_XL_1666HZ_8G) &&
                 (imu_drv_ctrl2_g_readback == IMU_CTRL2_G_1666HZ_1000DPS) &&
                 (imu_drv_bdu_if_inc_ok != 0U) &&
                 (imu_drv_i2c_disable_ok != 0U)) ? 1U : 0U;

            if (imu_drv_fullscale_config_ok == 0U)
            {
                imu_drv_error_count++;
                IMU_RecoveryAttemptFailed(now_us);
                break;
            }

            imu_continuity_initialized = 0U;
            imu_previous_raw = (IMU_RawData_t){0};
            imu_previous_previous_raw = (IMU_RawData_t){0};
            imu_stale_reference_initialized = 0U;
            imu_stale_reference_raw = (IMU_RawData_t){0};
            imu_last_raw_change_timestamp_us = now_us;
            imu_last_register_check_timestamp_us = now_us;
            imu_consecutive_invalid_sample_count = 0UL;
            imu_recovery_valid_streak = 0U;
            imu_recovery_count++;
            imu_recovery_state = IMU_RECOVERY_STATE_VALIDATING;
            imu_recovery_step = IMU_RECOVERY_STEP_IDLE;
            IMU_RecoveryFinishDuration(now_us);
            break;

        default:
            imu_recovery_step = IMU_RECOVERY_STEP_BUS_RESET;
            break;
    }

    imu_recovery_step_debug = imu_recovery_step;
    IMU_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void IMU_Init(void)
{
    imu_initialized = 0U;
    imu_connected = 0U;
    imu_device_id = 0U;

    imu_cached_raw.gyro_x_raw = 0;
    imu_cached_raw.gyro_y_raw = 0;
    imu_cached_raw.gyro_z_raw = 0;

    imu_cached_raw.accel_x_raw = 0;
    imu_cached_raw.accel_y_raw = 0;
    imu_cached_raw.accel_z_raw = 0;

    imu_last_sample_valid = 0U;
    imu_last_sample_timestamp_us = 0UL;
    imu_sample_generation = 0UL;
    imu_continuity_initialized = 0U;
    imu_previous_raw = (IMU_RawData_t){0};
    imu_previous_previous_raw = (IMU_RawData_t){0};

    imu_stale_reference_initialized = 0U;
    imu_stale_reference_raw = (IMU_RawData_t){0};
    imu_last_raw_change_timestamp_us = 0UL;
    imu_last_register_check_timestamp_us = 0UL;
    imu_recovery_retry_timestamp_us = 0UL;
    imu_recovery_valid_streak = 0U;
    imu_recovery_step = IMU_RECOVERY_STEP_IDLE;
    imu_recovery_attempt_in_cycle = 0U;
    imu_recovery_step_deadline_us = 0UL;
    imu_recovery_next_poll_us = 0UL;
    imu_recovery_start_timestamp_us = 0UL;
    imu_pattern_retry_pending = 0U;
    imu_pattern_register_snapshot_pending = 0U;
    imu_stale_retry_pending = 0U;
    imu_stale_register_snapshot_pending = 0U;
    imu_stale_warmup_active = 0U;
    imu_stale_warmup_first_ready_recorded = 0U;
    imu_stale_warmup_start_us = 0UL;

    imu_sample_valid = 0U;
    imu_stale_count = 0UL;
    imu_pattern_error_count = 0UL;
    imu_pattern_retry_count = 0UL;
    imu_pattern_retry_success_count = 0UL;
    imu_pattern_recovery_escalation_count = 0UL;
    imu_fast_config_check_count = 0UL;
    imu_fast_config_repair_attempt_count = 0UL;
    imu_fast_config_repair_success_count = 0UL;
    imu_fast_config_repair_failure_count = 0UL;
    imu_fast_config_repair_last_duration_us = 0UL;
    imu_fast_config_repair_max_duration_us = 0UL;
    imu_stale_fast_config_check_count = 0UL;
    imu_stale_fast_config_repair_attempt_count = 0UL;
    imu_stale_fast_config_repair_success_count = 0UL;
    imu_stale_fast_config_repair_failure_count = 0UL;
    imu_stale_retry_success_count = 0UL;
    imu_stale_recovery_escalation_count = 0UL;
    imu_stale_fast_config_repair_last_duration_us = 0UL;
    imu_stale_fast_config_repair_max_duration_us = 0UL;
    imu_stale_warmup_event_count = 0UL;
    imu_stale_warmup_poll_count = 0UL;
    imu_stale_warmup_success_count = 0UL;
    imu_stale_warmup_timeout_count = 0UL;
    imu_stale_warmup_first_ready_us = 0UL;
    imu_stale_warmup_max_ready_us = 0UL;
    imu_stale_warmup_last_status = 0U;
    imu_stale_warmup_active_debug = 0U;
    imu_pattern_diagnostic = (IMU_PatternDiagnostic_t){0};
    imu_stale_diagnostic = (IMU_StaleDiagnostic_t){0};
    imu_recovery_count = 0UL;
    imu_recovery_state = IMU_RECOVERY_STATE_NORMAL;
    imu_recovery_step_debug = IMU_RECOVERY_STEP_IDLE;
    imu_recovery_attempt_count = 0UL;
    imu_recovery_failure_count = 0UL;
    imu_recovery_last_duration_us = 0UL;
    imu_recovery_max_duration_us = 0UL;
    imu_register_error_count = 0UL;

    imu_valid_sample_count = 0UL;
    imu_invalid_sample_count = 0UL;
    imu_all_zero_sample_count = 0UL;
    imu_consecutive_invalid_sample_count = 0UL;
    imu_max_consecutive_invalid_sample_count = 0UL;
    imu_last_valid_sample_timestamp_us = 0UL;
    imu_last_invalid_sample_timestamp_us = 0UL;

    imu_drv_last_whoami = 0U;
    imu_drv_error_count = 0UL;
    imu_drv_read_count = 0UL;
    imu_drv_write_count = 0UL;

    imu_drv_last_reg = 0U;
    imu_drv_last_value = 0U;

    imu_drv_last_hal_status = 0U;
    imu_drv_last_rx0 = 0U;
    imu_drv_last_rx1 = 0U;

    imu_spi_profile_found = 0U;
    imu_spi_mode_selected = 0xFFU;
    imu_spi_prescaler_selected = 0U;
    imu_spi_probe_attempt_count = 0UL;
    imu_spi_poll_read_count = 0UL;
    imu_spi_poll_error_count = 0UL;
    imu_v43_direct_transfer_count = 0UL;
    imu_v43_direct_error_count = 0UL;
    imu_swspi_active = 0U;
    imu_swspi_whoami = 0U;
    imu_swspi_probe_count = 0UL;
    imu_swspi_transfer_count = 0UL;
    imu_swspi_error_count = 0UL;
    imu_spi_selected_polarity = SPI_POLARITY_HIGH;
    imu_spi_selected_phase = SPI_PHASE_2EDGE;

    imu_drv_ctrl1_xl_readback = 0U;
    imu_drv_ctrl2_g_readback = 0U;
    imu_drv_ctrl3_c_readback = 0U;
    imu_drv_ctrl4_c_readback = 0U;
    imu_drv_i2c_disable_ok = 0U;
    imu_drv_status_reg = 0U;
    imu_drv_fullscale_config_ok = 0U;
    imu_drv_bdu_if_inc_ok = 0U;
    imu_drv_reset_count = 0UL;
    imu_drv_config_retry_count = 0UL;

    imu_dma_start_count = 0UL;
    imu_dma_complete_count = 0UL;
    imu_dma_error_count = 0UL;
    imu_dma_timeout_count = 0UL;
    imu_dma_busy_count = 0UL;

    imu_dma_busy = 0U;
    imu_dma_done = 0U;
    imu_dma_error = 0U;
    imu_dma_transfer_type = IMU_DMA_TRANSFER_NONE;

    imu_spi_idle_timeout_count = 0UL;
    imu_redundant_set_count = 0UL;
    imu_redundant_burst_read_count = 0UL;
    imu_redundant_disagreement_count = 0UL;
    imu_redundant_reject_count = 0UL;
    imu_bit8_correction_count = 0UL;
    imu_bit8_correction_gyro_x_count = 0UL;
    imu_bit8_correction_gyro_y_count = 0UL;
    imu_bit8_correction_gyro_z_count = 0UL;
    imu_bit8_correction_accel_x_count = 0UL;
    imu_bit8_correction_accel_y_count = 0UL;
    imu_bit8_correction_accel_z_count = 0UL;

    if (IMU_HardwareSPI_DMA_Init() == 0U)
    {
        imu_initialized = 0U;
        imu_connected = 0U;
        IMU_UpdateLiveDebug();
        return;
    }

    HAL_Delay(100U);

    /* V44: first use the software-SPI path already proven by V42/V43.
     * If it cannot identify the IMU, retain the legacy hardware auto-probe as
     * a fallback. No other peripheral is reconfigured. */
    if (IMU_SW_SPI_EnableIfPresent() == 0U)
    {
        if (IMU_AutoProbeSPI() == 0U)
        {
            imu_initialized = 0U;
            imu_connected = 0U;
            imu_drv_error_count++;
            IMU_UpdateLiveDebug();
            return;
        }
    }

    for (uint8_t attempt = 0U; attempt < 10U; attempt++)
    {
        imu_device_id = IMU_ReadReg(IMU_REG_WHO_AM_I);
        imu_drv_last_whoami = imu_device_id;

        if ((imu_device_id == IMU_WHO_AM_I_ISM330DLC) ||
            (imu_device_id == IMU_WHO_AM_I_ALT))
        {
            imu_connected = 1U;
            break;
        }

        HAL_Delay(10U);
    }

    if (imu_connected == 0U)
    {
        imu_initialized = 0U;
        imu_drv_error_count++;
        IMU_UpdateLiveDebug();
        return;
    }

    if (IMU_ResetAndConfigure() == 0U)
    {
        imu_initialized = 0U;
        imu_connected = 0U;
        imu_drv_error_count++;
        IMU_UpdateLiveDebug();
        return;
    }

    /* Hardware SPI can still select a runtime speed. Software-SPI mode keeps
     * its proven Mode-3 GPIO backend and does not touch SPI1 again. */
    if (imu_swspi_active == 0U)
    {
        IMU_SelectRuntimeSPI();
    }

    imu_initialized = 1U;
    imu_last_register_check_timestamp_us = micros();

    /* Prime the coherent cache with the same validated path used in flight. */
    (void)IMU_AcquireValidatedSample();

    /*
     * İlk raw DMA transferini task tarafı başlatacak.
     * Burada başlatmıyoruz ki AppTasks_Init içindeki IMU_ReadWhoAmI ile çakışmasın.
     */

    IMU_UpdateLiveDebug();
}

uint8_t IMU_ReadWhoAmI(void)
{
    uint8_t whoami = IMU_ReadReg(IMU_REG_WHO_AM_I);

    imu_drv_last_whoami = whoami;

    if ((whoami == IMU_WHO_AM_I_ISM330DLC) ||
        (whoami == IMU_WHO_AM_I_ALT))
    {
        imu_connected = 1U;
        imu_device_id = whoami;
    }
    else
    {
        imu_connected = 0U;
    }

    IMU_UpdateLiveDebug();

    return whoami;
}

uint8_t IMU_IsConnected(void)
{
    return imu_connected;
}

uint8_t IMU_GetDeviceID(void)
{
    return imu_device_id;
}

uint8_t IMU_IsLastSampleValid(void)
{
    return imu_last_sample_valid;
}

uint32_t IMU_GetLastSampleTimestampUs(void)
{
    return imu_last_sample_timestamp_us;
}

uint32_t IMU_GetStaleCount(void)
{
    return imu_stale_count;
}

uint32_t IMU_GetPatternErrorCount(void)
{
    return imu_pattern_error_count;
}

uint32_t IMU_GetPatternRetryCount(void)
{
    return imu_pattern_retry_count;
}

uint32_t IMU_GetPatternRetrySuccessCount(void)
{
    return imu_pattern_retry_success_count;
}

uint32_t IMU_GetPatternRecoveryEscalationCount(void)
{
    return imu_pattern_recovery_escalation_count;
}

uint32_t IMU_GetFastConfigCheckCount(void)
{
    return imu_fast_config_check_count;
}

uint32_t IMU_GetFastConfigRepairAttemptCount(void)
{
    return imu_fast_config_repair_attempt_count;
}

uint32_t IMU_GetFastConfigRepairSuccessCount(void)
{
    return imu_fast_config_repair_success_count;
}

uint32_t IMU_GetFastConfigRepairFailureCount(void)
{
    return imu_fast_config_repair_failure_count;
}

uint32_t IMU_GetFastConfigRepairLastDurationUs(void)
{
    return imu_fast_config_repair_last_duration_us;
}

uint32_t IMU_GetFastConfigRepairMaxDurationUs(void)
{
    return imu_fast_config_repair_max_duration_us;
}

uint8_t IMU_GetPatternDiagnostic(IMU_PatternDiagnostic_t *diag)
{
    if (diag == 0)
    {
        return 0U;
    }

    *diag = imu_pattern_diagnostic;
    return imu_pattern_diagnostic.valid;
}

uint32_t IMU_GetStaleFastConfigCheckCount(void)
{
    return imu_stale_fast_config_check_count;
}

uint32_t IMU_GetStaleFastConfigRepairAttemptCount(void)
{
    return imu_stale_fast_config_repair_attempt_count;
}

uint32_t IMU_GetStaleFastConfigRepairSuccessCount(void)
{
    return imu_stale_fast_config_repair_success_count;
}

uint32_t IMU_GetStaleFastConfigRepairFailureCount(void)
{
    return imu_stale_fast_config_repair_failure_count;
}

uint32_t IMU_GetStaleRetrySuccessCount(void)
{
    return imu_stale_retry_success_count;
}

uint32_t IMU_GetStaleRecoveryEscalationCount(void)
{
    return imu_stale_recovery_escalation_count;
}

uint32_t IMU_GetStaleFastConfigRepairLastDurationUs(void)
{
    return imu_stale_fast_config_repair_last_duration_us;
}

uint32_t IMU_GetStaleFastConfigRepairMaxDurationUs(void)
{
    return imu_stale_fast_config_repair_max_duration_us;
}

uint32_t IMU_GetStaleWarmupEventCount(void)
{
    return imu_stale_warmup_event_count;
}

uint32_t IMU_GetStaleWarmupPollCount(void)
{
    return imu_stale_warmup_poll_count;
}

uint32_t IMU_GetStaleWarmupSuccessCount(void)
{
    return imu_stale_warmup_success_count;
}

uint32_t IMU_GetStaleWarmupTimeoutCount(void)
{
    return imu_stale_warmup_timeout_count;
}

uint32_t IMU_GetStaleWarmupFirstReadyUs(void)
{
    return imu_stale_warmup_first_ready_us;
}

uint32_t IMU_GetStaleWarmupMaxReadyUs(void)
{
    return imu_stale_warmup_max_ready_us;
}

uint8_t IMU_GetStaleWarmupLastStatus(void)
{
    return imu_stale_warmup_last_status;
}

uint8_t IMU_GetStaleWarmupActive(void)
{
    return imu_stale_warmup_active;
}

uint8_t IMU_GetStaleDiagnostic(IMU_StaleDiagnostic_t *diag)
{
    if (diag == 0)
    {
        return 0U;
    }

    *diag = imu_stale_diagnostic;
    return imu_stale_diagnostic.valid;
}

uint32_t IMU_GetDmaTimeoutCount(void)
{
    return imu_dma_timeout_count;
}

uint32_t IMU_GetRecoveryCount(void)
{
    return imu_recovery_count;
}

uint8_t IMU_GetRecoveryState(void)
{
    return imu_recovery_state;
}

uint8_t IMU_GetRecoveryStep(void)
{
    return imu_recovery_step_debug;
}

uint32_t IMU_GetRecoveryAttemptCount(void)
{
    return imu_recovery_attempt_count;
}

uint32_t IMU_GetRecoveryFailureCount(void)
{
    return imu_recovery_failure_count;
}

uint32_t IMU_GetRecoveryLastDurationUs(void)
{
    return imu_recovery_last_duration_us;
}

uint32_t IMU_GetRecoveryMaxDurationUs(void)
{
    return imu_recovery_max_duration_us;
}

uint32_t IMU_GetInvalidSampleCount(void)
{
    return imu_invalid_sample_count;
}

uint32_t IMU_GetRedundantRejectCount(void)
{
    return imu_redundant_reject_count;
}

uint32_t IMU_GetRegisterErrorCount(void)
{
    return imu_register_error_count;
}

uint8_t IMU_ShouldInhibitRCS(void)
{
    return ((imu_sample_valid == 0U) ||
            (imu_recovery_state != IMU_RECOVERY_STATE_NORMAL))
           ? 1U
           : 0U;
}

uint8_t IMU_ReadRawSnapshot(
    IMU_RawData_t *raw,
    uint32_t *sample_timestamp_us,
    uint32_t *sample_generation
)
{
    uint8_t valid;
    uint32_t primask;
    uint32_t now_us;

    if (raw == 0)
    {
        imu_drv_error_count++;
        return 0U;
    }

    /* Always return a defined value, even while recovery is in progress. */
    *raw = imu_cached_raw;

    now_us = micros();

    /* P36: runtime recovery is a bounded state-machine service. One call does
     * at most a handful of register transfers and never waits for milliseconds. */
    IMU_ServiceRecovery(now_us);

    if ((imu_initialized == 0U) ||
        (imu_connected == 0U) ||
        (imu_recovery_state == IMU_RECOVERY_STATE_REQUESTED) ||
        (imu_recovery_state == IMU_RECOVERY_STATE_RECOVERING) ||
        (imu_recovery_state == IMU_RECOVERY_STATE_FAILED))
    {
        imu_last_sample_valid = 0U;
        imu_sample_valid = 0U;

        if (sample_timestamp_us != 0)
        {
            *sample_timestamp_us = imu_last_sample_timestamp_us;
        }

        if (sample_generation != 0)
        {
            *sample_generation = imu_sample_generation;
        }

        IMU_UpdateLiveDebug();
        return 0U;
    }

    (void)IMU_AcquireValidatedSample();

    now_us = micros();

    if ((imu_recovery_state == IMU_RECOVERY_STATE_NORMAL) &&
        (((uint32_t)(now_us - imu_last_register_check_timestamp_us)) >=
         IMU_REGISTER_CHECK_INTERVAL_US))
    {
        imu_last_register_check_timestamp_us = now_us;

        if (IMU_CheckRuntimeRegisters() == 0U)
        {
            imu_recovery_state = IMU_RECOVERY_STATE_REQUESTED;
            IMU_RecordInvalidSample(now_us);
        }
    }

    primask = __get_PRIMASK();
    __disable_irq();

    *raw = imu_cached_raw;
    valid = imu_last_sample_valid;

    if (sample_timestamp_us != 0)
    {
        *sample_timestamp_us = imu_last_sample_timestamp_us;
    }

    if (sample_generation != 0)
    {
        *sample_generation = imu_sample_generation;
    }

    if (primask == 0UL)
    {
        __enable_irq();
    }

    IMU_UpdateLiveDebug();
    return valid;
}

void IMU_ReadRaw(IMU_RawData_t *raw)
{
    (void)IMU_ReadRawSnapshot(raw, 0, 0);
}

void IMU_ReadScaled(IMU_Data_t *data)
{
    IMU_RawData_t raw;

    if (data == 0)
    {
        imu_drv_error_count++;
        return;
    }

    IMU_ReadRaw(&raw);

    data->gyro_x_dps =
        raw.gyro_x_raw * APP_IMU_GYRO_SCALE_DPS;

    data->gyro_y_dps =
        raw.gyro_y_raw * APP_IMU_GYRO_SCALE_DPS;

    data->gyro_z_dps =
        raw.gyro_z_raw * APP_IMU_GYRO_SCALE_DPS;

    data->accel_x_g =
        raw.accel_x_raw * APP_IMU_ACCEL_SCALE_G;

    data->accel_y_g =
        raw.accel_y_raw * APP_IMU_ACCEL_SCALE_G;

    data->accel_z_g =
        raw.accel_z_raw * APP_IMU_ACCEL_SCALE_G;
}

/* -------------------------------------------------------------------------- */
/* HAL callbacks                                                              */
/* -------------------------------------------------------------------------- */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    uint8_t completed_transfer_type;

    if (hspi->Instance != SPI1)
    {
        return;
    }

    completed_transfer_type = imu_dma_transfer_type;

    /*
     * CS kaldırmadan önce SPI tamamen idle olsun.
     */
    IMU_WaitSPIIdle();
    IMU_CS_HIGH();

    /* Raw data are parsed only after the complete redundant read set. */

    imu_dma_busy = 0U;
    imu_dma_done = 1U;
    imu_dma_error = 0U;
    imu_dma_transfer_type = IMU_DMA_TRANSFER_NONE;

    imu_dma_complete_count++;

    /*
     * Three blocking bursts are completed for every IMU sample. Avoid a large
     * live-debug copy in every DMA IRQ; the task updates it once per validated
     * sample instead.
     */
    if (completed_transfer_type != IMU_DMA_TRANSFER_BLOCKING)
    {
        IMU_UpdateLiveDebug();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1)
    {
        return;
    }

    IMU_WaitSPIIdle();
    IMU_CS_HIGH();

    imu_dma_busy = 0U;
    imu_dma_done = 0U;
    imu_dma_error = 1U;
    imu_dma_transfer_type = IMU_DMA_TRANSFER_NONE;

    imu_dma_error_count++;
    imu_drv_error_count++;
    imu_drv_last_hal_status = 201U;

    HAL_SPI_Abort(hspi);

    IMU_UpdateLiveDebug();
}
