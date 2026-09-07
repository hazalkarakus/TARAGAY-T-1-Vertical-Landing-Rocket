#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include <stdint.h>

typedef enum
{
    SYS_FAULT_NONE = 0,

    SYS_FAULT_IMU_NOT_CONNECTED = 1,

    SYS_FAULT_BAROMETER_NOT_CONNECTED = 2,
    SYS_FAULT_BAROMETER_NO_DATA = 3,
    SYS_FAULT_BAROMETER_PRESSURE_INVALID = 4,
    SYS_FAULT_BAROMETER_NOT_CALIBRATED = 5,

    SYS_FAULT_SCHEDULER_STALLED = 6,

    SYS_FAULT_IMU_OVERRUN = 7,
    SYS_FAULT_IMU_DEADLINE_MISS = 8,

    SYS_FAULT_BAROMETER_OVERRUN = 9,
    SYS_FAULT_BAROMETER_DEADLINE_MISS = 10,

    SYS_FAULT_SPI1_BUS_ERROR = 11,
    SYS_FAULT_SPI2_BUS_ERROR = 12,

    SYS_FAULT_IMU_STALE = 13,
    SYS_FAULT_LIDAR_STALE = 14,
    SYS_FAULT_ESKF_STALE = 15,
    SYS_FAULT_ESKF_DIVERGENCE = 16,
    SYS_FAULT_SD_LOGGING = 17

} SystemFaultCode_t;

typedef struct
{
    uint8_t imu_ok;
    uint8_t imu_fresh;
    uint8_t lidar_fresh;
    uint8_t eskf_public_fresh;

    uint8_t barometer_ok;
    uint8_t barometer_connected;
    uint8_t barometer_data_ready;
    uint8_t barometer_pressure_valid;
    uint8_t barometer_calibrated;

    uint8_t scheduler_ok;
    uint8_t lidar_task_ok;
    uint8_t eskf_task_ok;
    uint8_t bus_ok;
    uint8_t system_ok;

    SystemFaultCode_t fault_code;

    uint32_t uptime_ms;
    uint32_t imu_sample_age_us;
    uint32_t lidar_sample_age_us;
    uint32_t eskf_public_output_age_us;

    uint32_t imu_run_count;
    uint32_t imu_last_exec_us;
    uint32_t imu_max_exec_us;
    uint32_t imu_overrun_count;
    uint32_t imu_deadline_miss_count;

    uint32_t barometer_run_count;
    uint32_t barometer_last_exec_us;
    uint32_t barometer_max_exec_us;
    uint32_t barometer_overrun_count;
    uint32_t barometer_deadline_miss_count;

    uint32_t lidar_run_count;
    uint32_t eskf_run_count;

    uint32_t spi1_transaction_count;
    uint32_t spi1_error_count;
    uint32_t spi1_timeout_count;
    uint32_t spi1_busy_count;
    uint32_t spi1_slow_count;
    uint32_t spi1_last_duration_us;
    uint32_t spi1_max_duration_us;

    uint32_t spi2_transaction_count;
    uint32_t spi2_error_count;
    uint32_t spi2_timeout_count;
    uint32_t spi2_busy_count;
    uint32_t spi2_slow_count;
    uint32_t spi2_last_duration_us;
    uint32_t spi2_max_duration_us;

    uint32_t monitor_update_count;

} SystemStatus_t;

void SystemMonitor_Init(void);
void SystemMonitor_Update(void);

SystemStatus_t SystemMonitor_GetStatus(void);

/* Hard real-time actuator gate. Unlike the 10 Hz diagnostic snapshot, this
 * evaluates current sensor/ESKF freshness on demand. P48 keeps SD mandatory
 * before flight but non-actuator-critical after separation, and honors V50
 * LIDAR/barometer degraded vertical fallback in flight. */
SystemFaultCode_t SystemMonitor_GetFastFaultCode(void);
uint8_t SystemMonitor_IsFastFaultActive(void);
/* P40 physical-actuator inhibit: live fast fault OR latched 10 Hz system fault. */
uint8_t SystemMonitor_IsActuatorFaultActive(void);

#endif
