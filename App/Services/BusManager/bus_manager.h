#ifndef BUS_MANAGER_H
#define BUS_MANAGER_H

#include "main.h"
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* Bus IDs                                                                    */
/* -------------------------------------------------------------------------- */

typedef enum
{
    BUS_ID_SPI1 = 0,
    BUS_ID_SPI2,
    BUS_ID_SPI3,

    BUS_ID_I2C2,
    BUS_ID_UART3,
    BUS_ID_SDIO,
    BUS_ID_GPIO,

    BUS_ID_COUNT

} BusId_t;

/* -------------------------------------------------------------------------- */
/* Device IDs                                                                 */
/* -------------------------------------------------------------------------- */

typedef enum
{
    DEVICE_ID_NONE = 0,

    DEVICE_ID_IMU,
    DEVICE_ID_BAROMETER,
    DEVICE_ID_NRF24,

    DEVICE_ID_LIDAR,
    DEVICE_ID_ODRIVE,
    DEVICE_ID_SD_CARD,
    DEVICE_ID_SOLENOID,
    DEVICE_ID_SEPARATION_INPUT

} DeviceId_t;

/* -------------------------------------------------------------------------- */
/* Bus Errors                                                                 */
/* -------------------------------------------------------------------------- */

typedef enum
{
    BUS_ERROR_NONE = 0,
    BUS_ERROR_HAL_ERROR,
    BUS_ERROR_HAL_BUSY,
    BUS_ERROR_HAL_TIMEOUT,
    BUS_ERROR_SLOW_TRANSACTION,
    BUS_ERROR_INVALID_ARGUMENT

} BusError_t;

/* -------------------------------------------------------------------------- */
/* Bus Status                                                                 */
/* -------------------------------------------------------------------------- */

typedef struct
{
    uint8_t ok;
    uint8_t busy;
    uint8_t fault;

    BusId_t bus_id;
    DeviceId_t owner_device;

    BusError_t last_error;

    uint32_t transaction_count;
    uint32_t error_count;
    uint32_t timeout_count;
    uint32_t busy_count;
    uint32_t slow_count;

    uint32_t last_duration_us;
    uint32_t max_duration_us;

    uint32_t last_start_us;
    uint32_t last_end_us;

    uint32_t last_error_time_ms;

    const char *last_error_file;
    uint32_t last_error_line;

} BusStatus_t;

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void BusManager_Init(void);

BusStatus_t BusManager_GetStatus(BusId_t bus_id);

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
);

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
);

/* -------------------------------------------------------------------------- */
/* Debug Macros                                                               */
/* -------------------------------------------------------------------------- */

#define BUS_SPI_TRANSMIT_RECEIVE(bus_id, device_id, hspi, tx, rx, size, timeout_ms, slow_threshold_us) \
    BusManager_SPI_TransmitReceive(                                                                  \
        bus_id,                                                                                       \
        device_id,                                                                                    \
        hspi,                                                                                         \
        tx,                                                                                           \
        rx,                                                                                           \
        size,                                                                                         \
        timeout_ms,                                                                                   \
        slow_threshold_us,                                                                            \
        __FILE__,                                                                                     \
        __LINE__                                                                                      \
    )

#define BUS_SPI_TRANSMIT(bus_id, device_id, hspi, tx, size, timeout_ms, slow_threshold_us) \
    BusManager_SPI_Transmit(                                                              \
        bus_id,                                                                            \
        device_id,                                                                         \
        hspi,                                                                              \
        tx,                                                                                \
        size,                                                                              \
        timeout_ms,                                                                        \
        slow_threshold_us,                                                                 \
        __FILE__,                                                                          \
        __LINE__                                                                           \
    )

#endif
