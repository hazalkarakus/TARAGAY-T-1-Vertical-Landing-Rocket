#include "Services/BusManager/bus_manager.h"
#include "Services/Timebase/timebase.h"

static BusStatus_t bus_status[BUS_ID_COUNT];

/*
 * Live Expressions için SPI1 debug değişkenleri
 */

volatile uint8_t bm_spi1_ok = 0;
volatile uint8_t bm_spi1_busy = 0;
volatile uint8_t bm_spi1_fault = 0;
volatile uint8_t bm_spi1_owner = 0;
volatile uint8_t bm_spi1_last_error = 0;

volatile uint32_t bm_spi1_transaction_count = 0;
volatile uint32_t bm_spi1_error_count = 0;
volatile uint32_t bm_spi1_timeout_count = 0;
volatile uint32_t bm_spi1_busy_count = 0;
volatile uint32_t bm_spi1_slow_count = 0;

volatile uint32_t bm_spi1_last_duration_us = 0;
volatile uint32_t bm_spi1_max_duration_us = 0;
volatile uint32_t bm_spi1_last_error_time_ms = 0;
volatile uint32_t bm_spi1_last_error_line = 0;

/*
 * Live Expressions için SPI2 debug değişkenleri
 */

volatile uint8_t bm_spi2_ok = 0;
volatile uint8_t bm_spi2_busy = 0;
volatile uint8_t bm_spi2_fault = 0;
volatile uint8_t bm_spi2_owner = 0;
volatile uint8_t bm_spi2_last_error = 0;

volatile uint32_t bm_spi2_transaction_count = 0;
volatile uint32_t bm_spi2_error_count = 0;
volatile uint32_t bm_spi2_timeout_count = 0;
volatile uint32_t bm_spi2_busy_count = 0;
volatile uint32_t bm_spi2_slow_count = 0;

volatile uint32_t bm_spi2_last_duration_us = 0;
volatile uint32_t bm_spi2_max_duration_us = 0;
volatile uint32_t bm_spi2_last_error_time_ms = 0;
volatile uint32_t bm_spi2_last_error_line = 0;

static void BusManager_UpdateLiveDebug(void)
{
    BusStatus_t *spi1 = &bus_status[BUS_ID_SPI1];

    bm_spi1_ok = spi1->ok;
    bm_spi1_busy = spi1->busy;
    bm_spi1_fault = spi1->fault;
    bm_spi1_owner = (uint8_t)spi1->owner_device;
    bm_spi1_last_error = (uint8_t)spi1->last_error;

    bm_spi1_transaction_count = spi1->transaction_count;
    bm_spi1_error_count = spi1->error_count;
    bm_spi1_timeout_count = spi1->timeout_count;
    bm_spi1_busy_count = spi1->busy_count;
    bm_spi1_slow_count = spi1->slow_count;

    bm_spi1_last_duration_us = spi1->last_duration_us;
    bm_spi1_max_duration_us = spi1->max_duration_us;
    bm_spi1_last_error_time_ms = spi1->last_error_time_ms;
    bm_spi1_last_error_line = spi1->last_error_line;

    BusStatus_t *spi2 = &bus_status[BUS_ID_SPI2];

    bm_spi2_ok = spi2->ok;
    bm_spi2_busy = spi2->busy;
    bm_spi2_fault = spi2->fault;
    bm_spi2_owner = (uint8_t)spi2->owner_device;
    bm_spi2_last_error = (uint8_t)spi2->last_error;

    bm_spi2_transaction_count = spi2->transaction_count;
    bm_spi2_error_count = spi2->error_count;
    bm_spi2_timeout_count = spi2->timeout_count;
    bm_spi2_busy_count = spi2->busy_count;
    bm_spi2_slow_count = spi2->slow_count;

    bm_spi2_last_duration_us = spi2->last_duration_us;
    bm_spi2_max_duration_us = spi2->max_duration_us;
    bm_spi2_last_error_time_ms = spi2->last_error_time_ms;
    bm_spi2_last_error_line = spi2->last_error_line;
}

static BusError_t BusManager_HalToBusError(HAL_StatusTypeDef hal_status)
{
    if (hal_status == HAL_OK)
    {
        return BUS_ERROR_NONE;
    }

    if (hal_status == HAL_BUSY)
    {
        return BUS_ERROR_HAL_BUSY;
    }

    if (hal_status == HAL_TIMEOUT)
    {
        return BUS_ERROR_HAL_TIMEOUT;
    }

    return BUS_ERROR_HAL_ERROR;
}

static void BusManager_SetHardFault(
    BusId_t bus_id,
    DeviceId_t device_id,
    BusError_t error,
    const char *file,
    uint32_t line
)
{
    if (bus_id >= BUS_ID_COUNT)
    {
        return;
    }

    BusStatus_t *bus = &bus_status[bus_id];

    bus->ok = 0;
    bus->fault = 1;
    bus->last_error = error;
    bus->owner_device = device_id;

    bus->error_count++;
    bus->last_error_time_ms = millis();
    bus->last_error_file = file;
    bus->last_error_line = line;

    if (error == BUS_ERROR_HAL_TIMEOUT)
    {
        bus->timeout_count++;
    }
    else if (error == BUS_ERROR_HAL_BUSY)
    {
        bus->busy_count++;
    }

    BusManager_UpdateLiveDebug();
}

static void BusManager_SetSlowWarning(
    BusId_t bus_id,
    DeviceId_t device_id,
    const char *file,
    uint32_t line
)
{
    if (bus_id >= BUS_ID_COUNT)
    {
        return;
    }

    BusStatus_t *bus = &bus_status[bus_id];

    bus->ok = 1;
    bus->fault = 0;
    bus->last_error = BUS_ERROR_SLOW_TRANSACTION;
    bus->owner_device = device_id;

    bus->slow_count++;
    bus->last_error_time_ms = millis();
    bus->last_error_file = file;
    bus->last_error_line = line;

    BusManager_UpdateLiveDebug();
}

void BusManager_Init(void)
{
    for (uint32_t i = 0; i < BUS_ID_COUNT; i++)
    {
        bus_status[i].ok = 1;
        bus_status[i].busy = 0;
        bus_status[i].fault = 0;

        bus_status[i].bus_id = (BusId_t)i;
        bus_status[i].owner_device = DEVICE_ID_NONE;

        bus_status[i].last_error = BUS_ERROR_NONE;

        bus_status[i].transaction_count = 0UL;
        bus_status[i].error_count = 0UL;
        bus_status[i].timeout_count = 0UL;
        bus_status[i].busy_count = 0UL;
        bus_status[i].slow_count = 0UL;

        bus_status[i].last_duration_us = 0UL;
        bus_status[i].max_duration_us = 0UL;

        bus_status[i].last_start_us = 0UL;
        bus_status[i].last_end_us = 0UL;

        bus_status[i].last_error_time_ms = 0UL;

        bus_status[i].last_error_file = 0;
        bus_status[i].last_error_line = 0UL;
    }

    BusManager_UpdateLiveDebug();
}

BusStatus_t BusManager_GetStatus(BusId_t bus_id)
{
    BusStatus_t empty = {0};

    if (bus_id >= BUS_ID_COUNT)
    {
        return empty;
    }

    return bus_status[bus_id];
}

HAL_StatusTypeDef BusManager_SPI_TransmitReceive(
    BusId_t bus_id,
    DeviceId_t device_id,
    SPI_HandleTypeDef *hspi,
    uint8_t *tx,
    uint8_t *rx,
    uint16_t size,
    uint32_t timeout_ms,
    uint32_t slow_threshold_us,
    const char *file,
    uint32_t line
)
{
    if ((bus_id >= BUS_ID_COUNT) ||
        (hspi == 0) ||
        (tx == 0) ||
        (rx == 0) ||
        (size == 0U))
    {
        BusManager_SetHardFault(bus_id, device_id, BUS_ERROR_INVALID_ARGUMENT, file, line);
        return HAL_ERROR;
    }

    BusStatus_t *bus = &bus_status[bus_id];

    if (bus->busy != 0)
    {
        BusManager_SetHardFault(bus_id, device_id, BUS_ERROR_HAL_BUSY, file, line);
        return HAL_BUSY;
    }

    bus->busy = 1;
    bus->owner_device = device_id;
    bus->last_start_us = micros();

    BusManager_UpdateLiveDebug();

    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(
        hspi,
        tx,
        rx,
        size,
        timeout_ms
    );

    bus->last_end_us = micros();
    bus->last_duration_us = bus->last_end_us - bus->last_start_us;

    if (bus->last_duration_us > bus->max_duration_us)
    {
        bus->max_duration_us = bus->last_duration_us;
    }

    bus->transaction_count++;
    bus->busy = 0;

    if (result != HAL_OK)
    {
        BusError_t error = BusManager_HalToBusError(result);
        BusManager_SetHardFault(bus_id, device_id, error, file, line);
    }
    else if (bus->last_duration_us > slow_threshold_us)
    {
        BusManager_SetSlowWarning(bus_id, device_id, file, line);
    }
    else
    {
        bus->ok = 1;
        bus->fault = 0;
        bus->last_error = BUS_ERROR_NONE;
    }

    BusManager_UpdateLiveDebug();

    return result;
}

HAL_StatusTypeDef BusManager_SPI_Transmit(
    BusId_t bus_id,
    DeviceId_t device_id,
    SPI_HandleTypeDef *hspi,
    uint8_t *tx,
    uint16_t size,
    uint32_t timeout_ms,
    uint32_t slow_threshold_us,
    const char *file,
    uint32_t line
)
{
    if ((bus_id >= BUS_ID_COUNT) ||
        (hspi == 0) ||
        (tx == 0) ||
        (size == 0U))
    {
        BusManager_SetHardFault(bus_id, device_id, BUS_ERROR_INVALID_ARGUMENT, file, line);
        return HAL_ERROR;
    }

    BusStatus_t *bus = &bus_status[bus_id];

    if (bus->busy != 0)
    {
        BusManager_SetHardFault(bus_id, device_id, BUS_ERROR_HAL_BUSY, file, line);
        return HAL_BUSY;
    }

    bus->busy = 1;
    bus->owner_device = device_id;
    bus->last_start_us = micros();

    BusManager_UpdateLiveDebug();

    HAL_StatusTypeDef result = HAL_SPI_Transmit(
        hspi,
        tx,
        size,
        timeout_ms
    );

    bus->last_end_us = micros();
    bus->last_duration_us = bus->last_end_us - bus->last_start_us;

    if (bus->last_duration_us > bus->max_duration_us)
    {
        bus->max_duration_us = bus->last_duration_us;
    }

    bus->transaction_count++;
    bus->busy = 0;

    if (result != HAL_OK)
    {
        BusError_t error = BusManager_HalToBusError(result);
        BusManager_SetHardFault(bus_id, device_id, error, file, line);
    }
    else if (bus->last_duration_us > slow_threshold_us)
    {
        BusManager_SetSlowWarning(bus_id, device_id, file, line);
    }
    else
    {
        bus->ok = 1;
        bus->fault = 0;
        bus->last_error = BUS_ERROR_NONE;
    }

    BusManager_UpdateLiveDebug();

    return result;
}
