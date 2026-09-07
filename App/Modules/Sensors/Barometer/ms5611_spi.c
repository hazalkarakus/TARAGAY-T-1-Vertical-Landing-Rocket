/*
 * ---------------------------------------------------------------------------
 *  ms5611_spi.c  ->  BMP585 / BMP581 SPI SURUCUSU  (datasheet uyumlu)
 * ---------------------------------------------------------------------------
 *  Sensor: Bosch BMP585 (BMP581 ayni register haritasi -> ayni surucu).
 *  Dosya adi ve MS5611_* isimleri barometer.c ile uyumluluk icin korundu.
 *  Ust katman (filtre / medyan / irtifa / EKF) HIC degismez.
 *
 *  Donanim: SPI2 (&hspi2), CS = PB12 (BOARD_BARO_CS), SPI mode 0 (CPOL0/CPHA0).
 *
 *  Datasheet'e gore kritik noktalar (bu surumde uygulandi):
 *   1) SPI okuma:  (reg | 0x80) gonder, ARDINDAN dogrudan veri gelir.
 *      BMP581/585'te DUMMY BYTE YOKTUR (BMP388'den farki budur). Bu yuzden
 *      chip_id rx[1]'de okunur. (Kullanicinin bir kez 0x51 gormesi bunu dogrular.)
 *   2) Guc acilisinda ilk erisim bir "dummy read" olmali -> arayuz SPI'a kilitlenir.
 *      Ilk CS-LOW islemi (soft reset yazimi) zaten SPI'i secer; ek olarak
 *      chip_id'yi birkac kez okuyup ilk cop okumayi atiyoruz.
 *   3) Konfig register'lari (OSR/ODR-odr/DSP) yalnizca STANDBY modda yazilabilir.
 *      Sira: standby -> OSR_CONFIG -> DSP_IIR(bypass) -> ODR_CONFIG(normal).
 *   4) STATUS.nvm_rdy=1 ve nvm_err=0 dogrulanir; INT_STATUS.por ile reset teyidi.
 *   5) Basinc = ham/64 [Pa], Sicaklik = ham/65536 [°C] (24-bit, sicaklik isaretli).
 *   6) V8.19E'de veri kabul yolu DRDY kullanmiyordu; 200 Hz task measurement
 *      registerlarini dogrudan okuyordu.
 *   7) P112R12R8R3: sicaklik+basinc daima TEK 6-byte atomic SPI burst ile
 *      okunur; ayrik tek-register measurement fallback kaldirildi.
 *   8) P112R12R8R5: fiziksel R8R4 logunda INT_STATUS.DRDY acquisition gate
 *      sik valid/fresh dropout uretti. R8R3'te kanitlanan 200 Hz direct atomic
 *      6-byte okuma geri getirildi; ayni raw P/T cifti duplicate olarak atilir.
 *      R8R4 bounded stale-recovery ve upper-layer freshness hold korunur.
 * ---------------------------------------------------------------------------
 */

#include "Modules/Sensors/Barometer/ms5611_spi.h"

#include "Platform/board_handles.h"
#include "Platform/board_pins.h"
#include "Services/Timebase/timebase.h"

#include "main.h"

/* -------------------------------------------------------------------------- */
/* BMP585 register haritasi                                                   */
/* -------------------------------------------------------------------------- */

#define BMP585_REG_CHIP_ID          0x01U
#define BMP585_REG_REV_ID           0x02U
#define BMP585_REG_CHIP_STATUS      0x11U
#define BMP585_REG_INT_CONFIG       0x14U
#define BMP585_REG_INT_SOURCE       0x15U
#define BMP585_REG_TEMP_DATA_XLSB   0x1DU  /* temp[7:0]   */
#define BMP585_REG_TEMP_DATA_LSB    0x1EU  /* temp[15:8]  */
#define BMP585_REG_TEMP_DATA_MSB    0x1FU  /* temp[23:16] */
#define BMP585_REG_PRESS_DATA_XLSB  0x20U  /* press[7:0]  */
#define BMP585_REG_PRESS_DATA_LSB   0x21U  /* press[15:8] */
#define BMP585_REG_PRESS_DATA_MSB   0x22U  /* press[23:16]*/
#define BMP585_REG_INT_STATUS       0x27U
#define BMP585_REG_STATUS           0x28U
#define BMP585_REG_DSP_CONFIG       0x30U
#define BMP585_REG_DSP_IIR          0x31U
#define BMP585_REG_OSR_CONFIG       0x36U
#define BMP585_REG_ODR_CONFIG       0x37U
#define BMP585_REG_OSR_EFF          0x38U
#define BMP585_REG_CMD              0x7EU

/* Chip ID'ler */
#define BMP585_CHIP_ID_585          0x51U  /* BMP585 */
#define BMP585_CHIP_ID_581          0x50U  /* BMP581 (ayni surucu) */

/* Soft reset komutu (CMD register'ina yazilir) */
#define BMP585_CMD_SOFT_RESET       0xB6U

/* STATUS (0x28) bitleri */
#define BMP585_STATUS_CORE_RDY      0x01U
#define BMP585_STATUS_NVM_RDY       0x02U
#define BMP585_STATUS_NVM_ERR       0x04U

/* INT_STATUS (0x27) bitleri */
#define BMP585_INT_DRDY             0x01U  /* data ready */
#define BMP585_INT_POR              0x10U  /* power-on / soft-reset complete */
#define BMP585_INT_SOURCE_DRDY_EN   0x01U  /* INT_SOURCE.drdy_data_reg_en */
#define BMP585_INT_SOURCE_CONFIG_VALUE 0x00U

/*
 * INT_CONFIG (0x14), Bosch SensorAPI ile ayni duzen:
 * pulsed + active-high + push-pull + interrupt enable.
 * Ust nibbledeki pad drive ayari read-modify-write ile korunur.
 */
#define BMP585_INT_CONFIG_CONTROL_MASK   0x0FU
#define BMP585_INT_CONFIG_REQUIRED_VALUE 0x0AU

/* OSR_CONFIG (0x36): osr_t[2:0], osr_p[5:3], press_en[6] */
#define BMP585_PRESS_EN             0x40U
#define BMP585_OSR_1X               0x00U
#define BMP585_OSR_2X               0x01U
#define BMP585_OSR_4X               0x02U
#define BMP585_OSR_8X               0x03U
#define BMP585_OSR_16X              0x04U
#define BMP585_OSR_32X              0x05U
#define BMP585_OSR_64X              0x06U
#define BMP585_OSR_128X             0x07U

/* ODR_CONFIG (0x37): pwr_mode[1:0], odr[6:2], deep_dis[7] */
#define BMP585_PWR_STANDBY          0x00U
#define BMP585_PWR_NORMAL           0x01U
#define BMP585_PWR_FORCED           0x02U
#define BMP585_PWR_CONTINUOUS       0x03U
#define BMP585_DEEP_DIS             0x80U  /* deep standby kapali */
#define BMP585_ODR_218_5HZ_CODE     0x01U  /* 218.5 Hz nominal -> [6:2] field */

/* -------------------------------------------------------------------------- */
/* Kullanici konfigurasyonu (istersen buradan degistir)                        */
/* -------------------------------------------------------------------------- */

/*
 * V8.19E: pressure x4, temperature x1.
 * Bosch NORMAL-mode timing table permits up to 220 Hz with this pair. The
 * selected 218.5 Hz sensor ODR stays above the 200 Hz host read rate and
 * preserves the highest pressure oversampling that still provides this margin.
 */
#define BMP585_OSR_CONFIG_VALUE  \
    (uint8_t)(BMP585_PRESS_EN | (BMP585_OSR_4X << 3) | (BMP585_OSR_1X << 0))

/* Normal mode + 218.5 Hz nominal ODR + deep standby disabled. */
#define BMP585_ODR_CONFIG_VALUE  \
    (uint8_t)(BMP585_DEEP_DIS | (BMP585_ODR_218_5HZ_CODE << 2) | BMP585_PWR_NORMAL)

/*
 * Kesin STANDBY: power mode = 0 ve deep standby devre disi.
 * 0x80, Bosch'un normal/forced moda gecmeden once kullandigi guvenli standby
 * kosulunu acikca kurar; onceki 0x00 gecisinin belirsizligini ortadan kaldirir.
 */
#define BMP585_STANDBY_CONFIG_VALUE \
    (uint8_t)(BMP585_DEEP_DIS | BMP585_PWR_STANDBY)

/* IIR bypass: ust katmanda zaten Butterworth var. */
#define BMP585_DSP_IIR_VALUE     0x00U

/* -------------------------------------------------------------------------- */
/* Yerel ayarlar                                                              */
/* -------------------------------------------------------------------------- */

#define BMP585_SPI_TIMEOUT_MS        2U
#define BMP585_READ_RETRY           3U    /* okuma tekrar sayisi */
#define BMP585_CHIP_ID_TRIES        10U   /* chip_id yakalama denemesi */
#define BMP585_REGISTER_DIAG_PERIOD_US 5000000UL
#define BMP585_RECOVERY_STALE_US       25000UL
#define BMP585_RECOVERY_COOLDOWN_US   500000UL
#define BMP585_RECOVERY_ERROR_LIMIT        3U
#define BMP585_RECOVERY_STANDBY_WAIT_US  3000UL
#define BMP585_RECOVERY_NORMAL_WAIT_US   5000UL

/* P112R12R8R3 measurement hardening. Bosch BMP5 SensorAPI reads the six
 * temperature+pressure bytes as one burst. A suspicious decoded burst is
 * retried as another complete atomic burst; data bytes are never pieced
 * together from separate CS transactions. */
#define BMP585_MEASUREMENT_ATOMIC_ATTEMPTS  3U
#define BMP585_TEMPERATURE_MIN_C          (-40.0f)
#define BMP585_TEMPERATURE_MAX_C           (85.0f)
#define BMP585_TEMPERATURE_MAX_STEP_C       (5.0f)
#define BMP585_PRESSURE_MAX_STEP_PA        (64.0f)

#define BMP585_MEASUREMENT_ERROR       0U
#define BMP585_MEASUREMENT_FRESH       1U
#define BMP585_MEASUREMENT_DUPLICATE   2U

#define BMP585_CS_LOW()   HAL_GPIO_WritePin(BOARD_BARO_CS_PORT, BOARD_BARO_CS_PIN, GPIO_PIN_RESET)
#define BMP585_CS_HIGH()  HAL_GPIO_WritePin(BOARD_BARO_CS_PORT, BOARD_BARO_CS_PIN, GPIO_PIN_SET)

/* Cikis olcekleme */
#define BMP585_PRESSURE_LSB_TO_PA   (1.0f / 64.0f)     /* press / 2^6 */
#define BMP585_TEMP_LSB_TO_C        (1.0f / 65536.0f)  /* temp  / 2^16 */

/* -------------------------------------------------------------------------- */
/* Private data                                                               */
/* -------------------------------------------------------------------------- */

static SPI_HandleTypeDef *bmp585_hspi = 0;
static MS5611_SPI_Data_t ms5611_data;

/* -------------------------------------------------------------------------- */
/* Live Expressions (MS5611 isimleri + BMP585 teshis alanlari)                */
/* -------------------------------------------------------------------------- */

volatile uint8_t ms5611_initialized = 0U;
volatile uint8_t ms5611_connected = 0U;

volatile uint8_t ms5611_prom_ok = 0U;
volatile uint8_t ms5611_prom_coefficients_ok = 0U;
volatile uint8_t ms5611_prom_crc_ok = 0U;

volatile uint8_t ms5611_adc_ok = 0U;
volatile uint8_t ms5611_data_ready = 0U;

volatile uint8_t ms5611_state = 0U;
volatile uint8_t ms5611_last_hal_status = 0U;

volatile uint32_t ms5611_d1_raw = 0UL;
volatile uint32_t ms5611_d2_raw = 0UL;

volatile float ms5611_temperature_c = 0.0f;
volatile float ms5611_pressure_pa = 0.0f;

volatile uint32_t ms5611_update_count = 0UL;
volatile uint32_t ms5611_adc_read_count = 0UL;
volatile uint32_t ms5611_invalid_adc_count = 0UL;
volatile uint32_t ms5611_communication_error_count = 0UL;

/* BMP585'e ozel teshis */
volatile uint8_t  bmp585_chip_id = 0U;
volatile uint8_t  bmp585_status_reg = 0U;
volatile uint8_t  bmp585_int_status = 0U;
volatile uint8_t  bmp585_osr_eff = 0U;
volatile uint8_t  bmp585_config_ok = 0U;
volatile uint32_t bmp585_drdy_count = 0UL;
volatile uint32_t bmp585_por_seen = 0UL;

/* V8.19E register-level direct-read diagnostics */
volatile uint8_t  bmp585_int_config_reg = 0U;
volatile uint8_t  bmp585_int_config_ok = 0U;
volatile uint8_t  bmp585_int_config_before_reg = 0U;
volatile uint8_t  bmp585_int_config_written_reg = 0U;
volatile uint8_t  bmp585_standby_config_reg = 0U;
volatile uint8_t  bmp585_config_stage = 0U;
volatile uint8_t  bmp585_int_source_reg = 0U;
volatile uint8_t  bmp585_drdy_source_ok = 0U;
volatile uint8_t  bmp585_odr_config_reg = 0U;
volatile uint8_t  bmp585_osr_config_reg = 0U;
volatile uint8_t  bmp585_register_readback_ok = 0U;
volatile uint8_t  bmp585_direct_read_mode_ok = 0U;

volatile uint32_t bmp585_service_call_count = 0UL;
/* Legacy V8.19D counters are retained for debugger compatibility. They stay
 * zero because V8.19E does not poll INT_STATUS or gate data on DRDY. */
volatile uint32_t bmp585_int_status_poll_count = 0UL;
volatile uint32_t bmp585_int_status_read_error_count = 0UL;
volatile uint32_t bmp585_no_drdy_poll_count = 0UL;
volatile uint32_t bmp585_measurement_read_count = 0UL;
volatile uint32_t bmp585_measurement_read_error_count = 0UL;
volatile uint32_t bmp585_fresh_sample_count = 0UL;
volatile uint32_t bmp585_duplicate_raw_count = 0UL;
volatile uint32_t bmp585_register_snapshot_count = 0UL;
volatile uint32_t bmp585_register_snapshot_error_count = 0UL;
volatile uint32_t bmp585_register_mismatch_count = 0UL;
volatile uint32_t bmp585_last_register_snapshot_us = 0UL;

volatile uint32_t bmp585_last_service_interval_us = 0UL;
volatile uint32_t bmp585_min_service_interval_us = 0UL;
volatile uint32_t bmp585_max_service_interval_us = 0UL;
volatile uint32_t bmp585_last_fresh_interval_us = 0UL;
volatile uint32_t bmp585_min_fresh_interval_us = 0UL;
volatile uint32_t bmp585_max_fresh_interval_us = 0UL;

static uint32_t bmp585_previous_service_us = 0UL;
static uint32_t bmp585_previous_fresh_us = 0UL;
static uint32_t bmp585_previous_press_raw = 0UL;
static int32_t  bmp585_previous_temp_raw = 0;
static uint8_t  bmp585_previous_raw_valid = 0U;

typedef enum
{
    BMP585_RECOVERY_IDLE = 0U,
    BMP585_RECOVERY_REQUESTED = 1U,
    BMP585_RECOVERY_WAIT_STANDBY = 2U,
    BMP585_RECOVERY_WAIT_NORMAL = 3U
} BMP585_RecoveryState_t;

static BMP585_RecoveryState_t bmp585_recovery_state = BMP585_RECOVERY_IDLE;
static uint32_t bmp585_recovery_deadline_us = 0UL;
static uint32_t bmp585_last_recovery_attempt_us = 0UL;
static uint32_t bmp585_last_good_or_init_us = 0UL;
static uint8_t bmp585_consecutive_read_errors = 0U;

volatile uint8_t  bmp585_recovery_active = 0U;
volatile uint8_t  bmp585_recovery_reason = 0U;
volatile uint32_t bmp585_recovery_request_count = 0UL;
volatile uint32_t bmp585_recovery_attempt_count = 0UL;
volatile uint32_t bmp585_recovery_success_count = 0UL;
volatile uint32_t bmp585_recovery_failure_count = 0UL;
volatile uint32_t bmp585_spi_abort_count = 0UL;
volatile uint32_t bmp585_spi_reinit_count = 0UL;

/* Ham teshis byte'lari */
volatile uint8_t bmp585_raw0 = 0U;
volatile uint8_t bmp585_raw1 = 0U;
volatile uint8_t bmp585_raw2 = 0U;
volatile uint8_t bmp585_raw3 = 0U;

/* V45: Olcum burst'u bu kartta kararsizsa otomatik tek-register fallback. */
volatile uint8_t  bmp585_single_register_fallback_active = 0U;
volatile uint32_t bmp585_burst_invalid_count = 0UL;
volatile uint32_t bmp585_single_register_read_count = 0UL;
volatile uint32_t bmp585_single_register_success_count = 0UL;
volatile uint32_t bmp585_single_register_error_count = 0UL;

/* P112R12R8R3 atomic-burst diagnostics. */
volatile uint32_t bmp585_atomic_retry_count = 0UL;
volatile uint32_t bmp585_atomic_retry_success_count = 0UL;
volatile uint32_t bmp585_plausibility_reject_count = 0UL;

/* -------------------------------------------------------------------------- */

static void MS5611_UpdateLive(void)
{
    ms5611_initialized = ms5611_data.initialized;
    ms5611_connected   = ms5611_data.connected;

    ms5611_prom_ok              = ms5611_data.prom_ok;
    ms5611_prom_coefficients_ok = ms5611_data.prom_coefficients_ok;
    ms5611_prom_crc_ok          = ms5611_data.prom_crc_ok;

    ms5611_adc_ok     = ms5611_data.adc_ok;
    ms5611_data_ready = ms5611_data.data_ready;

    ms5611_state = ms5611_data.state;

    ms5611_d1_raw = ms5611_data.d1_raw;
    ms5611_d2_raw = ms5611_data.d2_raw;

    ms5611_temperature_c = ms5611_data.temperature_c;
    ms5611_pressure_pa   = ms5611_data.pressure_pa;

    ms5611_update_count            = ms5611_data.update_count;
    ms5611_adc_read_count          = ms5611_data.adc_read_count;
    ms5611_invalid_adc_count       = ms5611_data.invalid_adc_count;
    ms5611_communication_error_count = ms5611_data.communication_error_count;
}

/* -------------------------------------------------------------------------- */
/* Zamanlama                                                                  */
/* -------------------------------------------------------------------------- */

static void BMP585_SaveDuration(uint32_t start_us)
{
    uint32_t duration_us = micros() - start_us;

    ms5611_data.last_transaction_duration_us = duration_us;

    if (duration_us > ms5611_data.max_transaction_duration_us)
    {
        ms5611_data.max_transaction_duration_us = duration_us;
    }
}

/* -------------------------------------------------------------------------- */
/* CS pin init                                                                */
/* -------------------------------------------------------------------------- */

static void BMP585_CS_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin   = BOARD_BARO_CS_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_Init(BOARD_BARO_CS_PORT, &GPIO_InitStruct);

    BMP585_CS_HIGH();
}

/* -------------------------------------------------------------------------- */
/* Dusuk seviye SPI                                                           */
/* -------------------------------------------------------------------------- */

static uint8_t BMP585_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    HAL_StatusTypeDef status;
    uint32_t start_us = micros();

    tx[0] = (uint8_t)(reg & 0x7FU);  /* yazma: MSB = 0 */
    tx[1] = value;

    BMP585_CS_LOW();
    status = HAL_SPI_Transmit(bmp585_hspi, tx, 2U, BMP585_SPI_TIMEOUT_MS);
    BMP585_CS_HIGH();

    ms5611_last_hal_status = (uint8_t)status;
    BMP585_SaveDuration(start_us);

    if (status != HAL_OK)
    {
        ms5611_data.communication_error_count++;
        MS5611_UpdateLive();
        return 0U;
    }

    return 1U;
}

/*
 * BMP581/585 SPI okuma: [reg|0x80] gonder, sonra dogrudan veri.
 * DUMMY BYTE YOK. tx: [reg|0x80][0x00...]  rx: [addr-eko][data0][data1]...
 */
static uint8_t BMP585_ReadRegsOnce(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t tx[8] = {0U};
    uint8_t rx[8] = {0U};
    uint16_t total;
    HAL_StatusTypeDef status;
    uint32_t start_us = micros();

    if ((buffer == 0) || (length == 0U) || (length > 7U))
    {
        return 0U;
    }

    total = (uint16_t)(1U + length);   /* addr + veri (dummy yok) */
    tx[0] = (uint8_t)(reg | 0x80U);    /* okuma: MSB = 1 */

    BMP585_CS_LOW();
    status = HAL_SPI_TransmitReceive(bmp585_hspi, tx, rx, total,
                                     BMP585_SPI_TIMEOUT_MS);
    BMP585_CS_HIGH();

    ms5611_last_hal_status = (uint8_t)status;
    BMP585_SaveDuration(start_us);

    bmp585_raw0 = rx[0];
    bmp585_raw1 = rx[1];
    bmp585_raw2 = rx[2];
    bmp585_raw3 = rx[3];

    if (status != HAL_OK)
    {
        return 0U;
    }

    for (uint8_t i = 0U; i < length; i++)
    {
        buffer[i] = rx[1U + i];        /* veri rx[1]'den baslar */
    }

    return 1U;
}

static uint8_t BMP585_ReadRegs(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    for (uint8_t attempt = 0U; attempt < BMP585_READ_RETRY; attempt++)
    {
        if (BMP585_ReadRegsOnce(reg, buffer, length) != 0U)
        {
            return 1U;
        }
    }

    ms5611_data.communication_error_count++;
    MS5611_UpdateLive();
    return 0U;
}

static uint8_t BMP585_ReadReg(uint8_t reg, uint8_t *value)
{
    return BMP585_ReadRegs(reg, value, 1U);
}

static void BMP585_RequestRecovery(uint8_t reason, uint32_t now_us)
{
    if (bmp585_recovery_state != BMP585_RECOVERY_IDLE) return;
    if ((bmp585_last_recovery_attempt_us != 0UL) &&
        ((now_us - bmp585_last_recovery_attempt_us) < BMP585_RECOVERY_COOLDOWN_US)) return;
    bmp585_recovery_reason = reason;
    bmp585_recovery_state = BMP585_RECOVERY_REQUESTED;
    bmp585_recovery_active = 1U;
    bmp585_recovery_request_count++;
}

static uint8_t BMP585_RecoverSPIPeripheral(void)
{
    if ((bmp585_hspi == 0) || (bmp585_hspi->Instance != SPI2)) return 0U;
    BMP585_CS_HIGH();
    if ((HAL_SPI_GetState(bmp585_hspi) != HAL_SPI_STATE_READY) ||
        (bmp585_hspi->ErrorCode != HAL_SPI_ERROR_NONE))
    {
        (void)HAL_SPI_Abort(bmp585_hspi);
        bmp585_spi_abort_count++;
    }
    if (HAL_SPI_DeInit(bmp585_hspi) != HAL_OK) return 0U;
    if (HAL_SPI_Init(bmp585_hspi) != HAL_OK) return 0U;
    bmp585_spi_reinit_count++;
    BMP585_CS_HIGH();
    return 1U;
}

/* Runtime recovery is non-blocking: required sensor settle times are modeled
 * as deadlines and never with HAL_Delay(). */
static void BMP585_ServiceRecovery(uint32_t now_us)
{
    uint8_t id=0U, status_reg=0U, int_source=0U, odr_config=0U, osr_config=0U, osr_eff=0U;
    uint8_t ok=1U;
    switch (bmp585_recovery_state)
    {
        case BMP585_RECOVERY_IDLE: return;
        case BMP585_RECOVERY_REQUESTED:
            bmp585_last_recovery_attempt_us=now_us;
            bmp585_recovery_attempt_count++;
            if ((BMP585_RecoverSPIPeripheral()==0U) ||
                (BMP585_WriteReg(BMP585_REG_ODR_CONFIG, BMP585_STANDBY_CONFIG_VALUE)==0U))
            {
                bmp585_recovery_failure_count++;
                bmp585_recovery_state=BMP585_RECOVERY_IDLE; bmp585_recovery_active=0U; return;
            }
            bmp585_recovery_deadline_us=now_us+BMP585_RECOVERY_STANDBY_WAIT_US;
            bmp585_recovery_state=BMP585_RECOVERY_WAIT_STANDBY; return;
        case BMP585_RECOVERY_WAIT_STANDBY:
            if ((int32_t)(now_us-bmp585_recovery_deadline_us)<0) return;
            ok &= BMP585_WriteReg(BMP585_REG_OSR_CONFIG, BMP585_OSR_CONFIG_VALUE);
            ok &= BMP585_WriteReg(BMP585_REG_DSP_IIR, BMP585_DSP_IIR_VALUE);
            ok &= BMP585_WriteReg(BMP585_REG_INT_SOURCE, BMP585_INT_SOURCE_CONFIG_VALUE);
            ok &= BMP585_WriteReg(BMP585_REG_ODR_CONFIG, BMP585_ODR_CONFIG_VALUE);
            if (ok==0U) { bmp585_recovery_failure_count++; bmp585_recovery_state=BMP585_RECOVERY_IDLE; bmp585_recovery_active=0U; return; }
            bmp585_recovery_deadline_us=now_us+BMP585_RECOVERY_NORMAL_WAIT_US;
            bmp585_recovery_state=BMP585_RECOVERY_WAIT_NORMAL; return;
        case BMP585_RECOVERY_WAIT_NORMAL:
            if ((int32_t)(now_us-bmp585_recovery_deadline_us)<0) return;
            ok &= BMP585_ReadReg(BMP585_REG_CHIP_ID,&id);
            ok &= BMP585_ReadReg(BMP585_REG_STATUS,&status_reg);
            ok &= BMP585_ReadReg(BMP585_REG_INT_SOURCE,&int_source);
            ok &= BMP585_ReadReg(BMP585_REG_ODR_CONFIG,&odr_config);
            ok &= BMP585_ReadReg(BMP585_REG_OSR_CONFIG,&osr_config);
            ok &= BMP585_ReadReg(BMP585_REG_OSR_EFF,&osr_eff);
            if ((ok!=0U) && ((id==BMP585_CHIP_ID_585)||(id==BMP585_CHIP_ID_581)) &&
                ((status_reg&BMP585_STATUS_NVM_ERR)==0U) &&
                ((int_source&BMP585_INT_SOURCE_DRDY_EN)==0U) &&
                ((odr_config&0x03U)==BMP585_PWR_NORMAL) &&
                ((osr_config&BMP585_PRESS_EN)!=0U))
            {
                bmp585_chip_id=id; bmp585_status_reg=status_reg; bmp585_int_source_reg=int_source;
                bmp585_drdy_source_ok=0U; bmp585_direct_read_mode_ok=1U;
                bmp585_odr_config_reg=odr_config; bmp585_osr_config_reg=osr_config;
                bmp585_osr_eff=osr_eff; bmp585_config_ok=1U;
                bmp585_previous_raw_valid=0U; bmp585_consecutive_read_errors=0U;
                bmp585_last_good_or_init_us=now_us; ms5611_data.adc_ok=0U; ms5611_data.data_ready=0U;
                bmp585_recovery_success_count++;
            }
            else { bmp585_config_ok=0U; ms5611_data.adc_ok=0U; bmp585_recovery_failure_count++; }
            bmp585_recovery_state=BMP585_RECOVERY_IDLE; bmp585_recovery_active=0U; MS5611_UpdateLive(); return;
        default: bmp585_recovery_state=BMP585_RECOVERY_IDLE; bmp585_recovery_active=0U; return;
    }
}

/* -------------------------------------------------------------------------- */
/* V8.19E non-invasive timing/configuration diagnostics                       */
/* -------------------------------------------------------------------------- */

static void BMP585_RecordServiceTiming(uint32_t now_us)
{
    if (bmp585_previous_service_us != 0UL)
    {
        uint32_t interval_us = now_us - bmp585_previous_service_us;

        bmp585_last_service_interval_us = interval_us;

        if ((bmp585_min_service_interval_us == 0UL) ||
            (interval_us < bmp585_min_service_interval_us))
        {
            bmp585_min_service_interval_us = interval_us;
        }

        if (interval_us > bmp585_max_service_interval_us)
        {
            bmp585_max_service_interval_us = interval_us;
        }
    }

    bmp585_previous_service_us = now_us;
}

static void BMP585_RecordFreshTiming(uint32_t now_us)
{
    if (bmp585_previous_fresh_us != 0UL)
    {
        uint32_t interval_us = now_us - bmp585_previous_fresh_us;

        bmp585_last_fresh_interval_us = interval_us;

        if ((bmp585_min_fresh_interval_us == 0UL) ||
            (interval_us < bmp585_min_fresh_interval_us))
        {
            bmp585_min_fresh_interval_us = interval_us;
        }

        if (interval_us > bmp585_max_fresh_interval_us)
        {
            bmp585_max_fresh_interval_us = interval_us;
        }
    }

    bmp585_previous_fresh_us = now_us;
}

static void BMP585_UpdatePeriodicRegisterSnapshot(uint32_t now_us)
{
    uint8_t int_config = bmp585_int_config_reg;
    uint8_t int_source = bmp585_int_source_reg;
    uint8_t odr_config = bmp585_odr_config_reg;
    uint8_t osr_config = bmp585_osr_config_reg;
    uint8_t osr_eff = bmp585_osr_eff;
    uint8_t status_reg = bmp585_status_reg;
    uint8_t read_ok = 1U;
    uint8_t matches_expected;

    if ((bmp585_last_register_snapshot_us != 0UL) &&
        ((now_us - bmp585_last_register_snapshot_us) <
         BMP585_REGISTER_DIAG_PERIOD_US))
    {
        return;
    }

    bmp585_last_register_snapshot_us = now_us;
    bmp585_register_snapshot_count++;

    if (BMP585_ReadReg(BMP585_REG_INT_CONFIG, &int_config) == 0U)
    {
        read_ok = 0U;
    }
    if (BMP585_ReadReg(BMP585_REG_INT_SOURCE, &int_source) == 0U)
    {
        read_ok = 0U;
    }
    if (BMP585_ReadReg(BMP585_REG_ODR_CONFIG, &odr_config) == 0U)
    {
        read_ok = 0U;
    }
    if (BMP585_ReadReg(BMP585_REG_OSR_CONFIG, &osr_config) == 0U)
    {
        read_ok = 0U;
    }
    if (BMP585_ReadReg(BMP585_REG_OSR_EFF, &osr_eff) == 0U)
    {
        read_ok = 0U;
    }
    if (BMP585_ReadReg(BMP585_REG_STATUS, &status_reg) == 0U)
    {
        read_ok = 0U;
    }

    if (read_ok == 0U)
    {
        bmp585_register_readback_ok = 0U;
        bmp585_config_ok = 0U;
        bmp585_register_snapshot_error_count++;
        BMP585_RequestRecovery(3U, now_us);
        return;
    }

    bmp585_int_config_reg = int_config;
    bmp585_int_source_reg = int_source;
    bmp585_odr_config_reg = odr_config;
    bmp585_osr_config_reg = osr_config;
    bmp585_osr_eff = osr_eff;
    bmp585_status_reg = status_reg;
    bmp585_drdy_source_ok =
        ((int_source & BMP585_INT_SOURCE_DRDY_EN) != 0U) ? 1U : 0U;
    /* R8R5 direct atomic mode: DRDY source is intentionally disabled and
     * INT_STATUS is not used as an acquisition gate. */
    bmp585_direct_read_mode_ok =
        ((int_source & BMP585_INT_SOURCE_DRDY_EN) == 0U) ? 1U : 0U;
    bmp585_int_config_ok = 1U;

    /*
     * R8R5 expected runtime: direct atomic reads, DRDY source disabled,
     * NORMAL mode at 218.5 Hz, P x4 / T x1.
     */
    matches_expected =
        (((int_source & BMP585_INT_SOURCE_DRDY_EN) == 0U) &&
         (odr_config == BMP585_ODR_CONFIG_VALUE) &&
         (osr_config == BMP585_OSR_CONFIG_VALUE) &&
         ((osr_eff & 0x80U) != 0U) &&
         (((osr_eff >> 3) & 0x07U) == BMP585_OSR_4X) &&
         ((osr_eff & 0x07U) == BMP585_OSR_1X) &&
         ((status_reg & BMP585_STATUS_NVM_ERR) == 0U)) ? 1U : 0U;

    bmp585_register_readback_ok = matches_expected;
    bmp585_config_ok = matches_expected;

    if (matches_expected == 0U)
    {
        bmp585_register_mismatch_count++;
        BMP585_RequestRecovery(3U, now_us);
    }
}

/* -------------------------------------------------------------------------- */
/* Soft reset + POR teyidi                                                    */
/* -------------------------------------------------------------------------- */

static uint8_t BMP585_SoftReset(void)
{
    uint8_t int_status = 0U;

    (void)BMP585_WriteReg(BMP585_REG_CMD, BMP585_CMD_SOFT_RESET);
    HAL_Delay(5U);   /* datasheet: reset sonrasi ~2 ms; guvenli tarafta 5 ms */

    /* POR biti reset tamamlandigini gosterir (okuyunca temizlenir). */
    if (BMP585_ReadReg(BMP585_REG_INT_STATUS, &int_status) != 0U)
    {
        bmp585_int_status = int_status;
        if ((int_status & BMP585_INT_POR) != 0U)
        {
            bmp585_por_seen++;
            return 1U;
        }
    }

    /* POR bitini yakalayamasak bile devam edebiliriz; chip_id son karari verir. */
    return 1U;
}

/* -------------------------------------------------------------------------- */
/* Chip ID (birkac deneme; aralikli okumada bir kez yakalamak yeterli)         */
/* -------------------------------------------------------------------------- */

static uint8_t BMP585_ReadChipId(uint8_t *out_id)
{
    uint8_t id = 0U;

    /* Ilk okuma guc acilisi sonrasi arayuzu SPI'a kilitleyen dummy read'dir. */
    (void)BMP585_ReadReg(BMP585_REG_CHIP_ID, &id);

    for (uint8_t i = 0U; i < BMP585_CHIP_ID_TRIES; i++)
    {
        if (BMP585_ReadReg(BMP585_REG_CHIP_ID, &id) != 0U)
        {
            if ((id == BMP585_CHIP_ID_585) || (id == BMP585_CHIP_ID_581))
            {
                *out_id = id;
                return 1U;
            }
        }
        HAL_Delay(2U);
    }

    *out_id = id;   /* son okunan (teshis icin) */
    return 0U;
}

/* -------------------------------------------------------------------------- */
/* Konfigurasyon (yalnizca STANDBY'da yazilir)                                */
/* -------------------------------------------------------------------------- */

static uint8_t BMP585_Configure(void)
{
    uint8_t status_reg = 0U;
    uint8_t osr_eff = 0U;
    uint8_t int_config_before = 0U;
    uint8_t int_source = 0U;
    uint8_t standby_config = 0U;
    uint8_t odr_config = 0U;
    uint8_t osr_config = 0U;
    uint8_t ok = 1U;

    /* NVM hazir mi / hata var mi? */
    (void)BMP585_ReadReg(BMP585_REG_STATUS, &status_reg);
    bmp585_status_reg = status_reg;

    if ((status_reg & BMP585_STATUS_NVM_RDY) == 0U)
    {
        return 0U;
    }
    if ((status_reg & BMP585_STATUS_NVM_ERR) != 0U)
    {
        return 0U;
    }

    bmp585_config_stage = 1U; /* NVM kontrolu gecti. */

    /* 1) STANDBY moda al.
     * BMP585 konfigurasyon register'larini guvenli sekilde burada yaziyoruz.
     */
    ok &= BMP585_WriteReg(
        BMP585_REG_ODR_CONFIG,
        BMP585_STANDBY_CONFIG_VALUE
    );
    HAL_Delay(3U);

    if (BMP585_ReadReg(BMP585_REG_ODR_CONFIG, &standby_config) == 0U)
    {
        ok = 0U;
    }

    bmp585_standby_config_reg = standby_config;

    if ((standby_config & (BMP585_DEEP_DIS | 0x03U)) !=
        BMP585_STANDBY_CONFIG_VALUE)
    {
        ok = 0U;
    }

    if (ok == 0U)
    {
        return 0U;
    }

    bmp585_config_stage = 2U; /* Gercek STANDBY readback dogrulandi. */

    /* 2) Pressure x4 + temperature x1 + pressure enable. */
    ok &= BMP585_WriteReg(BMP585_REG_OSR_CONFIG, BMP585_OSR_CONFIG_VALUE);

    /* 3) Sensorun dahili IIR'i bypass; ust katmanda filtre var. */
    ok &= BMP585_WriteReg(BMP585_REG_DSP_IIR, BMP585_DSP_IIR_VALUE);
    bmp585_config_stage = 3U;

    /*
     * 4) P112R12R8R5 direct atomic acquisition.
     *
     * Fiziksel INT pini ve INT_STATUS.DRDY acquisition gate kullanilmaz.
     * INT_SOURCE.DRDY kapali tutulur; 200 Hz host task tek 6-byte atomic P/T
     * burst okur ve ayni raw ciftini duplicate olarak yayinlamaz.
     */
    if (BMP585_ReadReg(BMP585_REG_INT_CONFIG, &int_config_before) == 0U)
    {
        int_config_before = 0U;
    }
    bmp585_int_config_before_reg = int_config_before;

    ok &= BMP585_WriteReg(BMP585_REG_INT_SOURCE, BMP585_INT_SOURCE_CONFIG_VALUE);
    bmp585_int_config_written_reg = int_config_before;
    bmp585_int_config_reg = int_config_before;
    bmp585_int_config_ok = 1U; /* physical INT pin config intentionally untouched */
    bmp585_config_stage = 4U;

    if (ok == 0U)
    {
        return 0U;
    }

    bmp585_config_stage = 5U; /* INT_SOURCE configured for direct-read mode. */

    /* 5) Normal mod + 218.5 Hz nominal ODR + deep standby disabled. */
    ok &= BMP585_WriteReg(BMP585_REG_ODR_CONFIG, BMP585_ODR_CONFIG_VALUE);
    HAL_Delay(5U);

    /*
     * Register read-back:
     * sadece yazmanin HAL_OK donmesine degil, sensorun gercekte kabul ettigi
     * ayarlara bakiyoruz.
     */
    if (BMP585_ReadReg(BMP585_REG_OSR_EFF, &osr_eff) == 0U)
    {
        ok = 0U;
    }

    if (BMP585_ReadReg(BMP585_REG_INT_SOURCE, &int_source) == 0U)
    {
        ok = 0U;
    }

    if (BMP585_ReadReg(BMP585_REG_ODR_CONFIG, &odr_config) == 0U)
    {
        ok = 0U;
    }

    if (BMP585_ReadReg(BMP585_REG_OSR_CONFIG, &osr_config) == 0U)
    {
        ok = 0U;
    }

    bmp585_osr_eff = osr_eff;
    bmp585_int_source_reg = int_source;
    bmp585_odr_config_reg = odr_config;
    bmp585_osr_config_reg = osr_config;

    bmp585_drdy_source_ok =
        ((int_source & BMP585_INT_SOURCE_DRDY_EN) != 0U) ? 1U : 0U;
    bmp585_direct_read_mode_ok =
        ((int_source & BMP585_INT_SOURCE_DRDY_EN) == 0U) ? 1U : 0U;
    bmp585_config_stage = 7U;

    /*
     * Expected read-back:
     * - ODR valid bit = 1
     * - P OSR = x4
     * - T OSR = x1
     * - pressure enabled
     * - ODR code = 0x01 (218.5 Hz nominal)
     * - power mode = NORMAL
     * - deep standby disabled
     * - DRDY source disabled (direct atomic-read mode)
     */
    if ((ok != 0U) &&
        ((osr_eff & 0x80U) != 0U) &&
        (((osr_eff >> 3) & 0x07U) == BMP585_OSR_4X) &&
        ((osr_eff & 0x07U) == BMP585_OSR_1X) &&
        ((osr_config & BMP585_PRESS_EN) != 0U) &&
        (((osr_config >> 3) & 0x07U) == BMP585_OSR_4X) &&
        ((osr_config & 0x07U) == BMP585_OSR_1X) &&
        (((odr_config >> 2) & 0x1FU) == BMP585_ODR_218_5HZ_CODE) &&
        ((odr_config & 0x03U) == BMP585_PWR_NORMAL) &&
        ((odr_config & BMP585_DEEP_DIS) != 0U) &&
        (bmp585_direct_read_mode_ok != 0U))
    {
        bmp585_config_ok = 1U;
        bmp585_config_stage = 8U;
    }
    else
    {
        bmp585_config_ok = 0U;
    }

    return bmp585_config_ok;
}

/* -------------------------------------------------------------------------- */
/* Ham veri okuma + Pa / °C                                                   */
/* -------------------------------------------------------------------------- */

static uint8_t BMP585_DecodeMeasurementBytes(
    const uint8_t raw[6],
    uint32_t *press_raw_out,
    int32_t *temp_raw_out,
    float *pressure_pa_out,
    float *temperature_c_out
)
{
    uint32_t press_raw;
    int32_t temp_raw;
    float pressure_pa;
    float temperature_c;

    if ((raw == 0) ||
        (press_raw_out == 0) ||
        (temp_raw_out == 0) ||
        (pressure_pa_out == 0) ||
        (temperature_c_out == 0))
    {
        return 0U;
    }

    temp_raw = (int32_t)(((uint32_t)raw[2] << 16) |
                         ((uint32_t)raw[1] << 8)  |
                         ((uint32_t)raw[0]));

    press_raw = ((uint32_t)raw[5] << 16) |
                ((uint32_t)raw[4] << 8)  |
                ((uint32_t)raw[3]);

    /* Sicaklik isaretli 24-bit: 24. bit set ise negatife genislet. */
    if ((temp_raw & 0x00800000) != 0)
    {
        temp_raw |= (int32_t)0xFF000000;
    }

    temperature_c = (float)temp_raw * BMP585_TEMP_LSB_TO_C;
    pressure_pa = (float)press_raw * BMP585_PRESSURE_LSB_TO_PA;

    /* Bosch BMP585 full-accuracy basinç araligi ile uyumlu makul kontrol. */
    if ((press_raw == 0UL) ||
        (pressure_pa < 30000.0f) ||
        (pressure_pa > 125000.0f))
    {
        return 0U;
    }

    *press_raw_out = press_raw;
    *temp_raw_out = temp_raw;
    *pressure_pa_out = pressure_pa;
    *temperature_c_out = temperature_c;
    return 1U;
}

/*
 * P112R12R8R3 atomic measurement path.
 *
 * Bosch BMP5 SensorAPI reads TEMP_DATA_XLSB..PRESS_DATA_MSB as one 6-byte
 * burst. The six bytes belong to one coherent register snapshot only when CS
 * stays asserted across the complete read. The old V45 fallback read those
 * bytes in six separate CS frames and could therefore splice two consecutive
 * physical samples together. That mode is intentionally disabled.
 */
static float BMP585_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t BMP585_MeasurementPlausible(
    float pressure_pa,
    float temperature_c
)
{
    if ((temperature_c < BMP585_TEMPERATURE_MIN_C) ||
        (temperature_c > BMP585_TEMPERATURE_MAX_C))
    {
        return 0U;
    }

    if (bmp585_previous_raw_valid != 0U)
    {
        const float previous_pressure_pa =
            (float)bmp585_previous_press_raw * BMP585_PRESSURE_LSB_TO_PA;
        const float previous_temperature_c =
            (float)bmp585_previous_temp_raw * BMP585_TEMP_LSB_TO_C;

        /* 5 C or 64 Pa between adjacent ~5 ms samples is far beyond the
         * physically credible change rate for this atmospheric sensor on the
         * vehicle. Treat it as a torn/corrupt transfer and reread atomically. */
        if (BMP585_AbsFloat(temperature_c - previous_temperature_c) >
            BMP585_TEMPERATURE_MAX_STEP_C)
        {
            return 0U;
        }

        if (BMP585_AbsFloat(pressure_pa - previous_pressure_pa) >
            BMP585_PRESSURE_MAX_STEP_PA)
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t BMP585_ReadMeasurement(void)
{
    uint8_t raw[6] = {0U};
    uint32_t press_raw = 0UL;
    int32_t temp_raw = 0;
    float pressure_pa = 0.0f;
    float temperature_c = 0.0f;
    uint8_t decoded_ok = 0U;

    /* Every attempt is a complete, coherent 6-byte burst. No per-byte
     * fallback is allowed. This also reduces SPI transaction overhead and CPU
     * load compared with six independent register reads. */
    for (uint8_t attempt = 0U;
         attempt < BMP585_MEASUREMENT_ATOMIC_ATTEMPTS;
         attempt++)
    {
        if (attempt != 0U)
        {
            bmp585_atomic_retry_count++;
        }

        if (BMP585_ReadRegs(BMP585_REG_TEMP_DATA_XLSB, raw, 6U) == 0U)
        {
            continue;
        }

        if (BMP585_DecodeMeasurementBytes(
                raw,
                &press_raw,
                &temp_raw,
                &pressure_pa,
                &temperature_c) == 0U)
        {
            bmp585_burst_invalid_count++;
            continue;
        }

        if (BMP585_MeasurementPlausible(pressure_pa, temperature_c) == 0U)
        {
            bmp585_burst_invalid_count++;
            bmp585_plausibility_reject_count++;
            continue;
        }

        decoded_ok = 1U;
        if (attempt != 0U)
        {
            bmp585_atomic_retry_success_count++;
        }
        break;
    }

    /* Legacy V45 fields are intentionally held disabled/zero. */
    bmp585_single_register_fallback_active = 0U;

    if (decoded_ok == 0U)
    {
        ms5611_data.adc_ok = 0U;
        ms5611_data.invalid_adc_count++;
        return BMP585_MEASUREMENT_ERROR;
    }

    /* Host 200 Hz, sensor 218.5 Hz nominal: do not publish the same physical
     * P/T pair twice. */
    if ((bmp585_previous_raw_valid != 0U) &&
        (press_raw == bmp585_previous_press_raw) &&
        (temp_raw == bmp585_previous_temp_raw))
    {
        ms5611_data.adc_ok = 1U;
        bmp585_duplicate_raw_count++;
        return BMP585_MEASUREMENT_DUPLICATE;
    }

    bmp585_previous_press_raw = press_raw;
    bmp585_previous_temp_raw = temp_raw;
    bmp585_previous_raw_valid = 1U;

    ms5611_data.d2_raw = (uint32_t)temp_raw;
    ms5611_data.d1_raw = press_raw;
    ms5611_data.temperature_c = temperature_c;
    ms5611_data.pressure_pa = pressure_pa;
    ms5611_data.adc_ok = 1U;
    ms5611_data.adc_read_count++;
    return BMP585_MEASUREMENT_FRESH;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

uint8_t MS5611_SPI_Init(SPI_HandleTypeDef *hspi)
{
    uint8_t chip_id = 0U;

    /* Veri yapisini sifirla */
    {
        uint8_t *p = (uint8_t *)&ms5611_data;
        for (uint32_t i = 0UL; i < sizeof(ms5611_data); i++)
        {
            p[i] = 0U;
        }
    }
    ms5611_data.state = MS5611_STATE_UNINITIALIZED;

    bmp585_service_call_count = 0UL;
    bmp585_int_status_poll_count = 0UL;
    bmp585_int_status_read_error_count = 0UL;
    bmp585_no_drdy_poll_count = 0UL;
    bmp585_measurement_read_count = 0UL;
    bmp585_measurement_read_error_count = 0UL;
    bmp585_fresh_sample_count = 0UL;
    bmp585_duplicate_raw_count = 0UL;
    bmp585_register_snapshot_count = 0UL;
    bmp585_register_snapshot_error_count = 0UL;
    bmp585_register_mismatch_count = 0UL;
    bmp585_last_register_snapshot_us = 0UL;
    bmp585_register_readback_ok = 0U;
    bmp585_direct_read_mode_ok = 0U;
    bmp585_int_config_reg = 0U;
    bmp585_int_config_ok = 0U;
    bmp585_int_config_before_reg = 0U;
    bmp585_int_config_written_reg = 0U;
    bmp585_standby_config_reg = 0U;
    bmp585_config_stage = 0U;

    bmp585_last_service_interval_us = 0UL;
    bmp585_min_service_interval_us = 0UL;
    bmp585_max_service_interval_us = 0UL;
    bmp585_last_fresh_interval_us = 0UL;
    bmp585_min_fresh_interval_us = 0UL;
    bmp585_max_fresh_interval_us = 0UL;
    bmp585_previous_service_us = 0UL;
    bmp585_previous_fresh_us = 0UL;
    bmp585_previous_press_raw = 0UL;
    bmp585_previous_temp_raw = 0;
    bmp585_previous_raw_valid = 0U;
    bmp585_recovery_state = BMP585_RECOVERY_IDLE;
    bmp585_recovery_deadline_us = 0UL;
    bmp585_last_recovery_attempt_us = 0UL;
    bmp585_last_good_or_init_us = 0UL;
    bmp585_consecutive_read_errors = 0U;
    bmp585_recovery_active = 0U;
    bmp585_recovery_reason = 0U;
    bmp585_recovery_request_count = 0UL;
    bmp585_recovery_attempt_count = 0UL;
    bmp585_recovery_success_count = 0UL;
    bmp585_recovery_failure_count = 0UL;
    bmp585_spi_abort_count = 0UL;
    bmp585_spi_reinit_count = 0UL;

    bmp585_single_register_fallback_active = 0U;
    bmp585_burst_invalid_count = 0UL;
    bmp585_single_register_read_count = 0UL;
    bmp585_single_register_success_count = 0UL;
    bmp585_single_register_error_count = 0UL;
    bmp585_atomic_retry_count = 0UL;
    bmp585_atomic_retry_success_count = 0UL;
    bmp585_plausibility_reject_count = 0UL;

    bmp585_hspi = hspi;

    BMP585_CS_Init();

    if ((bmp585_hspi == 0) || (bmp585_hspi->Instance != SPI2))
    {
        MS5611_UpdateLive();
        return 0U;
    }

    if (HAL_SPI_GetState(bmp585_hspi) != HAL_SPI_STATE_READY)
    {
        (void)HAL_SPI_Abort(bmp585_hspi);
    }

    BMP585_CS_HIGH();
    HAL_Delay(5U);   /* guc oturma */

    /* Soft reset -> arayuz SPI'a kilitlenir, POR teyidi */
    (void)BMP585_SoftReset();

    /* Chip ID (0x51 BMP585 / 0x50 BMP581) */
    if (BMP585_ReadChipId(&chip_id) == 0U)
    {
        bmp585_chip_id = chip_id;
        ms5611_data.prom[0] = (uint16_t)chip_id;
        ms5611_data.connected = 0U;
        ms5611_data.initialized = 0U;
        ms5611_data.state = MS5611_STATE_ERROR;
        MS5611_UpdateLive();
        return 0U;
    }

    bmp585_chip_id = chip_id;
    ms5611_data.prom[0] = (uint16_t)chip_id;   /* MS5611 uyumlulugu icin sakla */

    /* Konfigurasyon */
    if (BMP585_Configure() == 0U)
    {
        ms5611_data.connected = 0U;
        ms5611_data.initialized = 0U;
        ms5611_data.state = MS5611_STATE_ERROR;
        MS5611_UpdateLive();
        return 0U;
    }

    ms5611_data.initialized = 1U;
    ms5611_data.connected   = 1U;

    /* BMP585'te PROM/CRC yok -> ust katman icin hepsini gecerli say */
    ms5611_data.prom_ok              = 1U;
    ms5611_data.prom_coefficients_ok = 1U;
    ms5611_data.prom_crc_ok          = 1U;

    ms5611_data.state = MS5611_STATE_IDLE;
    bmp585_last_good_or_init_us = micros();

    MS5611_UpdateLive();
    return 1U;
}

/* -------------------------------------------------------------------------- */

void MS5611_SPI_Update(void)
{
    uint8_t measurement_result;
    uint32_t now_us;

    if ((ms5611_data.initialized == 0U) ||
        (ms5611_data.connected == 0U))
    {
        return;
    }

    now_us = micros();
    bmp585_service_call_count++;
    BMP585_RecordServiceTiming(now_us);

    if (bmp585_recovery_state != BMP585_RECOVERY_IDLE)
    {
        BMP585_ServiceRecovery(now_us);
        return;
    }

    /* Start recovery before the upper 50 ms freshness guard expires.
     * At 218.5 Hz, 25 ms means several physical samples were genuinely lost. */
    if ((bmp585_last_good_or_init_us != 0UL) &&
        ((now_us - bmp585_last_good_or_init_us) > BMP585_RECOVERY_STALE_US))
    {
        BMP585_RequestRecovery(2U, now_us);
        BMP585_ServiceRecovery(now_us);
        return;
    }

    BMP585_UpdatePeriodicRegisterSnapshot(now_us);
    if (bmp585_recovery_state != BMP585_RECOVERY_IDLE)
    {
        BMP585_ServiceRecovery(now_us);
        return;
    }

    /* P112R12R8R5: direct 200 Hz host read, exactly one 6-byte atomic P/T
     * transaction. The sensor runs at 218.5 Hz; an unchanged raw pair is a
     * benign duplicate and is not published twice. INT_STATUS.DRDY is not an
     * acquisition gate. This restores the physically more stable R8R3 path
     * while retaining R8R4 bounded recovery/freshness semantics. */
    bmp585_measurement_read_count++;
    measurement_result = BMP585_ReadMeasurement();

    if (measurement_result == BMP585_MEASUREMENT_ERROR)
    {
        bmp585_measurement_read_error_count++;
        ms5611_data.state = MS5611_STATE_IDLE;
        if (bmp585_consecutive_read_errors < 255U)
        {
            bmp585_consecutive_read_errors++;
        }
        if (bmp585_consecutive_read_errors >= BMP585_RECOVERY_ERROR_LIMIT)
        {
            BMP585_RequestRecovery(1U, now_us);
        }
        MS5611_UpdateLive();
        return;
    }

    if (measurement_result == BMP585_MEASUREMENT_DUPLICATE)
    {
        ms5611_data.state = MS5611_STATE_IDLE;
        bmp585_consecutive_read_errors = 0U;
        MS5611_UpdateLive();
        return;
    }

    bmp585_consecutive_read_errors = 0U;
    bmp585_last_good_or_init_us = now_us;
    bmp585_fresh_sample_count++;
    BMP585_RecordFreshTiming(now_us);
    ms5611_data.data_ready = 1U;
    ms5611_data.update_count++;
    ms5611_data.state = MS5611_STATE_IDLE;
    MS5611_UpdateLive();
}

/* -------------------------------------------------------------------------- */

uint8_t MS5611_SPI_IsConnected(void)
{
    return ms5611_data.connected;
}

uint8_t MS5611_SPI_IsDataReady(void)
{
    return ms5611_data.data_ready;
}

MS5611_SPI_Data_t MS5611_SPI_GetData(void)
{
    return ms5611_data;
}
