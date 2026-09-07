#ifndef MS5611_SPI_H
#define MS5611_SPI_H

/*
 * -------------------------------------------------------------------------
 *  BMP585 SPI surucusu
 * -------------------------------------------------------------------------
 *  DIKKAT: Dosya adi ve fonksiyon/tip isimleri (MS5611_*) GERIYE DONUK
 *  UYUMLULUK icin korunmustur. Sensor artik Bosch BMP585'tir.
 *  barometer.c bu arayuzu oldugu gibi kullanmaya devam eder; boylece
 *  ust katman (filtre, medyan, irtifa, EKF) hic degismez.
 *
 *  Ayni donanim hatti: SPI2 (&hspi2), CS = PB12 (BOARD_BARO_CS).
 * -------------------------------------------------------------------------
 */

#include <stdint.h>
#include "main.h"

typedef enum
{
    MS5611_STATE_UNINITIALIZED = 0,
    MS5611_STATE_IDLE = 1,
    MS5611_STATE_WAIT_D1 = 2,
    MS5611_STATE_READ_D1 = 3,
    MS5611_STATE_WAIT_D2 = 4,
    MS5611_STATE_READ_D2 = 5,
    MS5611_STATE_ERROR = 6

} MS5611_State_t;

typedef struct
{
    uint8_t initialized;
    uint8_t connected;

    /* MS5611 uyumluluk alanlari: BMP585'te PROM yok, connected ile 1 tutulur */
    uint8_t prom_ok;
    uint8_t prom_coefficients_ok;
    uint8_t prom_crc_ok;

    uint8_t adc_ok;
    uint8_t data_ready;

    uint8_t selected_spi_mode;
    uint8_t state;

    /* MS5611 PROM alani; BMP585'te kullanilmaz, chip id vb. tutulabilir */
    uint16_t prom[8];

    /* Ham 24-bit basinc / sicaklik ADC degerleri (BMP585 register cikisi) */
    uint32_t d1_raw;   /* = ham basinc word'u */
    uint32_t d2_raw;   /* = ham sicaklik word'u */

    float temperature_c;
    float pressure_pa;

    uint32_t update_count;
    uint32_t adc_read_count;
    uint32_t invalid_adc_count;
    uint32_t communication_error_count;
    uint32_t prom_crc_error_count;

    uint32_t conversion_start_us;

    uint32_t last_transaction_duration_us;
    uint32_t max_transaction_duration_us;

} MS5611_SPI_Data_t;

uint8_t MS5611_SPI_Init(SPI_HandleTypeDef *hspi);
void MS5611_SPI_Update(void);

uint8_t MS5611_SPI_IsConnected(void);
uint8_t MS5611_SPI_IsDataReady(void);

MS5611_SPI_Data_t MS5611_SPI_GetData(void);

/* BMP585 V8.19E register-level and 200 Hz direct-read diagnostics */
extern volatile uint8_t  bmp585_chip_id;
extern volatile uint8_t  bmp585_config_ok;
extern volatile uint8_t  bmp585_status_reg;
extern volatile uint8_t  bmp585_osr_eff;
extern volatile uint8_t  bmp585_int_status;
extern volatile uint32_t bmp585_drdy_count;
extern volatile uint8_t  bmp585_int_config_reg;
extern volatile uint8_t  bmp585_int_config_ok;
extern volatile uint8_t  bmp585_int_config_before_reg;
extern volatile uint8_t  bmp585_int_config_written_reg;
extern volatile uint8_t  bmp585_standby_config_reg;
extern volatile uint8_t  bmp585_config_stage;
extern volatile uint8_t  bmp585_int_source_reg;
extern volatile uint8_t  bmp585_drdy_source_ok;
extern volatile uint8_t  bmp585_odr_config_reg;
extern volatile uint8_t  bmp585_osr_config_reg;
extern volatile uint8_t  bmp585_register_readback_ok;
extern volatile uint8_t  bmp585_direct_read_mode_ok;

extern volatile uint32_t bmp585_service_call_count;
extern volatile uint32_t bmp585_int_status_poll_count;
extern volatile uint32_t bmp585_int_status_read_error_count;
extern volatile uint32_t bmp585_no_drdy_poll_count;
extern volatile uint32_t bmp585_measurement_read_count;
extern volatile uint32_t bmp585_measurement_read_error_count;
extern volatile uint32_t bmp585_fresh_sample_count;
extern volatile uint32_t bmp585_duplicate_raw_count;
extern volatile uint32_t bmp585_register_snapshot_count;
extern volatile uint32_t bmp585_register_snapshot_error_count;
extern volatile uint32_t bmp585_register_mismatch_count;
extern volatile uint32_t bmp585_last_register_snapshot_us;

extern volatile uint32_t bmp585_last_service_interval_us;
extern volatile uint32_t bmp585_min_service_interval_us;
extern volatile uint32_t bmp585_max_service_interval_us;
extern volatile uint32_t bmp585_last_fresh_interval_us;
extern volatile uint32_t bmp585_min_fresh_interval_us;
extern volatile uint32_t bmp585_max_fresh_interval_us;

/* P112R12R8R3 atomic measurement diagnostics. Legacy single-register
 * fallback counters remain in the C file for debugger ABI compatibility but
 * the measurement path no longer uses that mode. */
extern volatile uint32_t bmp585_atomic_retry_count;
extern volatile uint32_t bmp585_atomic_retry_success_count;
extern volatile uint32_t bmp585_plausibility_reject_count;

#endif
