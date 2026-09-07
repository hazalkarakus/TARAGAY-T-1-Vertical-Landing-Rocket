#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>
#include <stddef.h>

/*
 * Genel boolean tipi.
 */

typedef enum
{
    APP_FALSE = 0,
    APP_TRUE  = 1

} AppBool_t;

/*
 * Genel modül durumu.
 */

typedef enum
{
    APP_MODULE_STATE_UNINIT = 0,
    APP_MODULE_STATE_INIT,
    APP_MODULE_STATE_READY,
    APP_MODULE_STATE_RUNNING,
    APP_MODULE_STATE_FAULT

} AppModuleState_t;

/*
 * Genel health durumu.
 */

typedef enum
{
    APP_HEALTH_UNKNOWN = 0,
    APP_HEALTH_OK,
    APP_HEALTH_WARNING,
    APP_HEALTH_FAULT

} AppHealth_t;

/*
 * 3 eksenli float vektör.
 */

typedef struct
{
    float x;
    float y;
    float z;

} Vector3f_t;

/*
 * 3 eksenli int16 raw vektör.
 */

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;

} Vector3i16_t;

/*
 * Genel zaman damgası.
 */

typedef struct
{
    uint32_t timestamp_us;
    uint32_t timestamp_ms;

} AppTimestamp_t;

#endif
