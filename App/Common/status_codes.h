#ifndef STATUS_CODES_H
#define STATUS_CODES_H

#include <stdint.h>

/*
 * Genel uygulama sonuç kodları.
 */

typedef enum
{
    APP_STATUS_OK = 0,
    APP_STATUS_ERROR,
    APP_STATUS_BUSY,
    APP_STATUS_TIMEOUT,
    APP_STATUS_INVALID_PARAM,
    APP_STATUS_NOT_INITIALIZED,
    APP_STATUS_NOT_READY,
    APP_STATUS_NOT_SUPPORTED

} AppStatus_t;

/*
 * Sistem fault kodları.
 *
 * Şimdilik system_monitor.h içindeki fault enum'u aktif kalabilir.
 * İleride system_monitor fault enum'unu buraya merkezileştireceğiz.
 */

typedef enum
{
    APP_FAULT_NONE = 0,

    APP_FAULT_IMU_NOT_CONNECTED,
    APP_FAULT_BAROMETER_NOT_CONNECTED,

    APP_FAULT_SPI1_BUS_ERROR,
    APP_FAULT_SPI2_BUS_ERROR,

    APP_FAULT_SCHEDULER_STALLED,
    APP_FAULT_TASK_OVERRUN,
    APP_FAULT_TASK_DEADLINE_MISS,

    APP_FAULT_SENSOR_DATA_INVALID,

    APP_FAULT_UNKNOWN = 255

} AppFaultCode_t;

#endif
