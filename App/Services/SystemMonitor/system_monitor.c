#include "Services/SystemMonitor/system_monitor.h"

#include <string.h>

#include "Common/app_config.h"
#include "Modules/Sensors/IMU/imu.h"
#include "Modules/Sensors/Barometer/barometer.h"
#include "Modules/Sensors/Lidar/Lidar.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"
#include "Modules/Control/VerticalSensorPolicy/vertical_sensor_policy.h"
#include "Services/PreflightTrigger/preflight_trigger.h"
#include "Services/SDLogger/sd_logger.h"

#include "Core/Scheduler/scheduler.h"

#include "Services/Timebase/timebase.h"

/* -------------------------------------------------------------------------- */
/* Task index                                                                 */
/* -------------------------------------------------------------------------- */

#define IMU_TASK_INDEX                         0UL
#define BAROMETER_TASK_INDEX                   1UL
#define LIDAR_TASK_INDEX                       2UL
#define FULL_ESKF_TASK_INDEX                   4UL

/*
 * Eğer task run_count bu süre içinde hiç artmazsa scheduler stalled kabul edilir.
 * Bit-bang test sürümünde deadline/overrun sayaçlarını fatal yapmıyoruz.
 */
#define SCHEDULER_STALL_TIMEOUT_MS             500UL

/* -------------------------------------------------------------------------- */
/* Private data                                                               */
/* -------------------------------------------------------------------------- */

static SystemStatus_t system_status;

static uint32_t last_imu_run_count = 0UL;
static uint32_t last_barometer_run_count = 0UL;
static uint32_t last_lidar_run_count = 0UL;
static uint32_t last_eskf_run_count = 0UL;

static uint32_t last_imu_progress_ms = 0UL;
static uint32_t last_barometer_progress_ms = 0UL;
static uint32_t last_lidar_progress_ms = 0UL;
static uint32_t last_eskf_progress_ms = 0UL;

/* -------------------------------------------------------------------------- */
/* Live Expressions                                                           */
/* -------------------------------------------------------------------------- */

volatile uint8_t sm_imu_ok = 0U;
volatile uint8_t sm_imu_fresh = 0U;
volatile uint8_t sm_lidar_fresh = 0U;
volatile uint8_t sm_eskf_public_fresh = 0U;
volatile uint32_t sm_imu_sample_age_us = 0xFFFFFFFFUL;
volatile uint32_t sm_lidar_sample_age_us = 0xFFFFFFFFUL;
volatile uint32_t sm_eskf_public_output_age_us = 0xFFFFFFFFUL;

volatile uint8_t sm_barometer_ok = 0U;
volatile uint8_t sm_barometer_connected = 0U;
volatile uint8_t sm_barometer_data_ready = 0U;
volatile uint8_t sm_barometer_pressure_valid = 0U;
volatile uint8_t sm_barometer_calibrated = 0U;

volatile uint8_t sm_scheduler_ok = 0U;
volatile uint8_t sm_bus_ok = 0U;
volatile uint8_t sm_system_ok = 0U;

volatile uint8_t sm_fault_code = 0U;

volatile uint32_t sm_uptime_ms = 0UL;

volatile uint32_t sm_imu_run_count = 0UL;
volatile uint32_t sm_imu_last_exec_us = 0UL;
volatile uint32_t sm_imu_max_exec_us = 0UL;
volatile uint32_t sm_imu_overrun_count = 0UL;
volatile uint32_t sm_imu_deadline_miss_count = 0UL;

volatile uint32_t sm_barometer_run_count = 0UL;
volatile uint32_t sm_barometer_last_exec_us = 0UL;
volatile uint32_t sm_barometer_max_exec_us = 0UL;
volatile uint32_t sm_barometer_overrun_count = 0UL;
volatile uint32_t sm_barometer_deadline_miss_count = 0UL;

volatile uint32_t sm_spi1_transaction_count = 0UL;
volatile uint32_t sm_spi1_error_count = 0UL;
volatile uint32_t sm_spi1_timeout_count = 0UL;
volatile uint32_t sm_spi1_busy_count = 0UL;
volatile uint32_t sm_spi1_slow_count = 0UL;
volatile uint32_t sm_spi1_last_duration_us = 0UL;
volatile uint32_t sm_spi1_max_duration_us = 0UL;

volatile uint32_t sm_spi2_transaction_count = 0UL;
volatile uint32_t sm_spi2_error_count = 0UL;
volatile uint32_t sm_spi2_timeout_count = 0UL;
volatile uint32_t sm_spi2_busy_count = 0UL;
volatile uint32_t sm_spi2_slow_count = 0UL;
volatile uint32_t sm_spi2_last_duration_us = 0UL;
volatile uint32_t sm_spi2_max_duration_us = 0UL;

volatile uint32_t sm_monitor_update_count = 0UL;

/* Extra debug */
volatile uint32_t sm_last_imu_progress_ms = 0UL;
volatile uint32_t sm_last_barometer_progress_ms = 0UL;
volatile uint32_t sm_last_lidar_progress_ms = 0UL;
volatile uint32_t sm_last_eskf_progress_ms = 0UL;

/* -------------------------------------------------------------------------- */

static void SystemMonitor_UpdateLiveDebug(void)
{
    sm_imu_ok = system_status.imu_ok;
    sm_imu_fresh = system_status.imu_fresh;
    sm_lidar_fresh = system_status.lidar_fresh;
    sm_eskf_public_fresh = system_status.eskf_public_fresh;
    sm_imu_sample_age_us = system_status.imu_sample_age_us;
    sm_lidar_sample_age_us = system_status.lidar_sample_age_us;
    sm_eskf_public_output_age_us =
        system_status.eskf_public_output_age_us;

    sm_barometer_ok = system_status.barometer_ok;
    sm_barometer_connected = system_status.barometer_connected;
    sm_barometer_data_ready = system_status.barometer_data_ready;
    sm_barometer_pressure_valid = system_status.barometer_pressure_valid;
    sm_barometer_calibrated = system_status.barometer_calibrated;

    sm_scheduler_ok = system_status.scheduler_ok;
    sm_bus_ok = system_status.bus_ok;
    sm_system_ok = system_status.system_ok;

    sm_fault_code = (uint8_t)system_status.fault_code;

    sm_uptime_ms = system_status.uptime_ms;

    sm_imu_run_count = system_status.imu_run_count;
    sm_imu_last_exec_us = system_status.imu_last_exec_us;
    sm_imu_max_exec_us = system_status.imu_max_exec_us;
    sm_imu_overrun_count = system_status.imu_overrun_count;
    sm_imu_deadline_miss_count = system_status.imu_deadline_miss_count;

    sm_barometer_run_count = system_status.barometer_run_count;
    sm_barometer_last_exec_us = system_status.barometer_last_exec_us;
    sm_barometer_max_exec_us = system_status.barometer_max_exec_us;
    sm_barometer_overrun_count = system_status.barometer_overrun_count;
    sm_barometer_deadline_miss_count = system_status.barometer_deadline_miss_count;

    sm_spi1_transaction_count = system_status.spi1_transaction_count;
    sm_spi1_error_count = system_status.spi1_error_count;
    sm_spi1_timeout_count = system_status.spi1_timeout_count;
    sm_spi1_busy_count = system_status.spi1_busy_count;
    sm_spi1_slow_count = system_status.spi1_slow_count;
    sm_spi1_last_duration_us = system_status.spi1_last_duration_us;
    sm_spi1_max_duration_us = system_status.spi1_max_duration_us;

    sm_spi2_transaction_count = system_status.spi2_transaction_count;
    sm_spi2_error_count = system_status.spi2_error_count;
    sm_spi2_timeout_count = system_status.spi2_timeout_count;
    sm_spi2_busy_count = system_status.spi2_busy_count;
    sm_spi2_slow_count = system_status.spi2_slow_count;
    sm_spi2_last_duration_us = system_status.spi2_last_duration_us;
    sm_spi2_max_duration_us = system_status.spi2_max_duration_us;

    sm_monitor_update_count = system_status.monitor_update_count;

    sm_last_imu_progress_ms = last_imu_progress_ms;
    sm_last_barometer_progress_ms = last_barometer_progress_ms;
    sm_last_lidar_progress_ms = last_lidar_progress_ms;
    sm_last_eskf_progress_ms = last_eskf_progress_ms;
}

/* -------------------------------------------------------------------------- */

static void SystemMonitor_ResetStatus(void)
{
    memset(&system_status, 0, sizeof(system_status));

    system_status.fault_code = SYS_FAULT_NONE;

    last_imu_run_count = 0UL;
    last_barometer_run_count = 0UL;
    last_lidar_run_count = 0UL;
    last_eskf_run_count = 0UL;

    last_imu_progress_ms = millis();
    last_barometer_progress_ms = millis();
    last_lidar_progress_ms = millis();
    last_eskf_progress_ms = millis();

    SystemMonitor_UpdateLiveDebug();
}

/* -------------------------------------------------------------------------- */
/* Sensor health                                                              */
/* -------------------------------------------------------------------------- */

static uint32_t SystemMonitor_AgeUs(
    uint32_t now_us,
    uint32_t timestamp_us
)
{
    if (timestamp_us == 0UL)
    {
        return 0xFFFFFFFFUL;
    }

    return (uint32_t)(now_us - timestamp_us);
}

static void SystemMonitor_UpdateSensorHealth(void)
{
    uint32_t now_us = micros();
    const SensorData_t *sensor = SensorManager_GetDataPtr();
    const LidarData_t *lidar = Lidar_GetDataPtr();
    const FullStateESKFData_t *eskf = FullStateESKF_GetDataPtr();

    system_status.imu_ok = IMU_IsConnected();
    system_status.imu_sample_age_us = SystemMonitor_AgeUs(
        now_us,
        sensor->imu_sample_timestamp_us);
    system_status.lidar_sample_age_us = SystemMonitor_AgeUs(
        now_us,
        lidar->last_sample_timestamp_us);
    system_status.eskf_public_output_age_us = SystemMonitor_AgeUs(
        now_us,
        eskf->last_public_output_timestamp_us);

    system_status.imu_fresh =
        (system_status.imu_sample_age_us <=
         APP_SYSTEM_MONITOR_IMU_STALE_US) ? 1U : 0U;
    system_status.lidar_fresh =
        (system_status.lidar_sample_age_us <=
         APP_SYSTEM_MONITOR_LIDAR_STALE_US) ? 1U : 0U;
    system_status.eskf_public_fresh =
        (system_status.eskf_public_output_age_us <=
         APP_SYSTEM_MONITOR_ESKF_PUBLIC_STALE_US) ? 1U : 0U;

    system_status.barometer_connected = Barometer_IsConnected();
    system_status.barometer_data_ready = Barometer_IsDataReady();
    system_status.barometer_pressure_valid = Barometer_IsPressureValid();
    system_status.barometer_calibrated = Barometer_IsCalibrated();
    system_status.barometer_ok = Barometer_IsHealthy();
}

/* -------------------------------------------------------------------------- */
/* Scheduler health                                                           */
/* -------------------------------------------------------------------------- */

static void SystemMonitor_UpdateSchedulerHealth(void)
{
    uint32_t now_ms = millis();

    uint8_t imu_task_ok = 0U;
    uint8_t barometer_task_ok = 0U;
    uint8_t lidar_task_ok = 0U;
    uint8_t eskf_task_ok = 0U;

    Task_t *imu_task = Scheduler_GetTask(IMU_TASK_INDEX);

    if (imu_task != 0)
    {
        system_status.imu_run_count = imu_task->run_count;
        system_status.imu_last_exec_us = imu_task->last_exec_us;
        system_status.imu_max_exec_us = imu_task->max_exec_us;
        system_status.imu_overrun_count = imu_task->overrun_count;
        system_status.imu_deadline_miss_count = imu_task->deadline_miss_count;

        if (imu_task->run_count != last_imu_run_count)
        {
            last_imu_run_count = imu_task->run_count;
            last_imu_progress_ms = now_ms;
        }

        if ((now_ms - last_imu_progress_ms) < SCHEDULER_STALL_TIMEOUT_MS)
        {
            imu_task_ok = 1U;
        }
    }

    Task_t *barometer_task = Scheduler_GetTask(BAROMETER_TASK_INDEX);

    if (barometer_task != 0)
    {
        system_status.barometer_run_count = barometer_task->run_count;
        system_status.barometer_last_exec_us = barometer_task->last_exec_us;
        system_status.barometer_max_exec_us = barometer_task->max_exec_us;
        system_status.barometer_overrun_count = barometer_task->overrun_count;
        system_status.barometer_deadline_miss_count = barometer_task->deadline_miss_count;

        if (barometer_task->run_count != last_barometer_run_count)
        {
            last_barometer_run_count = barometer_task->run_count;
            last_barometer_progress_ms = now_ms;
        }

        if ((now_ms - last_barometer_progress_ms) < SCHEDULER_STALL_TIMEOUT_MS)
        {
            barometer_task_ok = 1U;
        }
    }

    Task_t *lidar_task = Scheduler_GetTask(LIDAR_TASK_INDEX);

    if (lidar_task != 0)
    {
        system_status.lidar_run_count = lidar_task->run_count;

        if (lidar_task->run_count != last_lidar_run_count)
        {
            last_lidar_run_count = lidar_task->run_count;
            last_lidar_progress_ms = now_ms;
        }

        if ((now_ms - last_lidar_progress_ms) < SCHEDULER_STALL_TIMEOUT_MS)
        {
            lidar_task_ok = 1U;
        }
    }

    Task_t *eskf_task = Scheduler_GetTask(FULL_ESKF_TASK_INDEX);

    if (eskf_task != 0)
    {
        system_status.eskf_run_count = eskf_task->run_count;

        if (eskf_task->run_count != last_eskf_run_count)
        {
            last_eskf_run_count = eskf_task->run_count;
            last_eskf_progress_ms = now_ms;
        }

        if ((now_ms - last_eskf_progress_ms) < SCHEDULER_STALL_TIMEOUT_MS)
        {
            eskf_task_ok = 1U;
        }
    }

    system_status.lidar_task_ok = lidar_task_ok;
    system_status.eskf_task_ok = eskf_task_ok;

    /*
     * Önemli:
     * Bu bit-bang test sürümünde overrun/deadline sayaçları fatal değil.
     * Çünkü barometre dönüşümünde bekleme var.
     * Fatal kabul ettiğimiz şey task'ların tamamen durması.
     */

    if ((imu_task_ok != 0U) &&
        (barometer_task_ok != 0U) &&
        (lidar_task_ok != 0U) &&
        (eskf_task_ok != 0U))
    {
        system_status.scheduler_ok = 1U;
    }
    else
    {
        system_status.scheduler_ok = 0U;
    }
}

/* -------------------------------------------------------------------------- */
/* Bus health                                                                 */
/* -------------------------------------------------------------------------- */

static void SystemMonitor_UpdateBusHealth(void)
{
    /*
     * Şu an IMU ve barometre bit-bang SPI ile çalışıyor.
     * BusManager SPI istatistikleri bu sürümde kullanılmıyor.
     */

    system_status.bus_ok = 1U;

    system_status.spi1_transaction_count = 0UL;
    system_status.spi1_error_count = 0UL;
    system_status.spi1_timeout_count = 0UL;
    system_status.spi1_busy_count = 0UL;
    system_status.spi1_slow_count = 0UL;
    system_status.spi1_last_duration_us = 0UL;
    system_status.spi1_max_duration_us = 0UL;

    system_status.spi2_transaction_count = 0UL;
    system_status.spi2_error_count = 0UL;
    system_status.spi2_timeout_count = 0UL;
    system_status.spi2_busy_count = 0UL;
    system_status.spi2_slow_count = 0UL;
    system_status.spi2_last_duration_us = 0UL;
    system_status.spi2_max_duration_us = 0UL;
}

/* -------------------------------------------------------------------------- */
/* Fault code                                                                 */
/* -------------------------------------------------------------------------- */

static void SystemMonitor_UpdateFaultCode(void)
{
    const FullStateESKFData_t *eskf = FullStateESKF_GetDataPtr();
    uint8_t flight_active = PreflightTrigger_IsFlightActive();
    VerticalSensorPolicyResult_t vertical = VerticalSensorPolicy_Evaluate(eskf);

    system_status.fault_code = SYS_FAULT_NONE;

    if (system_status.imu_ok == 0U)
    {
        system_status.fault_code = SYS_FAULT_IMU_NOT_CONNECTED;
        return;
    }

    /* P48 flight policy: both vertical sensors are mandatory before flight,
     * but once PE9 has committed flight, V50 intentionally permits either
     * fresh LIDAR or fresh barometer aiding.  Do not defeat that redundancy by
     * latching the failed member as a global actuator fault. */
    if (flight_active == 0U)
    {
        if (system_status.barometer_connected == 0U)
        {
            system_status.fault_code = SYS_FAULT_BAROMETER_NOT_CONNECTED;
            return;
        }

        if (system_status.barometer_data_ready == 0U)
        {
            system_status.fault_code = SYS_FAULT_BAROMETER_NO_DATA;
            return;
        }

        if (system_status.barometer_pressure_valid == 0U)
        {
            system_status.fault_code = SYS_FAULT_BAROMETER_PRESSURE_INVALID;
            return;
        }

        if (system_status.barometer_calibrated == 0U)
        {
            system_status.fault_code = SYS_FAULT_BAROMETER_NOT_CALIBRATED;
            return;
        }
    }

    if (system_status.scheduler_ok == 0U)
    {
        system_status.fault_code = SYS_FAULT_SCHEDULER_STALLED;
        return;
    }

    if ((eskf == 0) || (eskf->output_inhibited != 0U))
    {
        system_status.fault_code = SYS_FAULT_ESKF_DIVERGENCE;
        return;
    }

#if (APP_SDLOGGER_ENABLED != 0U)
    /* SD logging is a preflight requirement.  In flight it remains fully
     * diagnosed by SDLogger telemetry, but losing a recorder must never close
     * the needle valve or kill RCS stabilization. */
    if ((flight_active == 0U) &&
        (system_status.uptime_ms >= APP_SYSTEM_MONITOR_STARTUP_GRACE_MS) &&
        ((SDLogger_IsReady() == 0U) || (SDLogger_IsLogging() == 0U)))
    {
        system_status.fault_code = SYS_FAULT_SD_LOGGING;
        return;
    }
#endif

    if (system_status.uptime_ms >= APP_SYSTEM_MONITOR_STARTUP_GRACE_MS)
    {
        if (system_status.imu_fresh == 0U)
        {
            system_status.fault_code = SYS_FAULT_IMU_STALE;
            return;
        }

        if ((eskf->healthy == 0U) ||
            (system_status.eskf_public_fresh == 0U))
        {
            system_status.fault_code = SYS_FAULT_ESKF_STALE;
            return;
        }

        if (flight_active != 0U)
        {
            if (vertical.usable == 0U)
            {
                /* Existing code value retained for telemetry compatibility;
                 * semantically this means all valid vertical aiding is lost. */
                system_status.fault_code = SYS_FAULT_LIDAR_STALE;
                return;
            }
        }
        else if (system_status.lidar_fresh == 0U)
        {
            system_status.fault_code = SYS_FAULT_LIDAR_STALE;
            return;
        }
    }

    /* Historical deadline counters remain diagnostics, not fatal latches. */
}

/* -------------------------------------------------------------------------- */
/* P39 fast actuator-safety fault path                                        */
/* -------------------------------------------------------------------------- */

SystemFaultCode_t SystemMonitor_GetFastFaultCode(void)
{
    uint32_t now_ms = millis();
    uint32_t now_us = micros();
    const BarometerData_t *baro = Barometer_GetDataPtr();
    const LidarData_t *lidar = Lidar_GetDataPtr();
    const FullStateESKFData_t *eskf = FullStateESKF_GetDataPtr();
    uint8_t flight_active = PreflightTrigger_IsFlightActive();
    VerticalSensorPolicyResult_t vertical = VerticalSensorPolicy_Evaluate(eskf);

    if (IMU_IsConnected() == 0U)
    {
        return SYS_FAULT_IMU_NOT_CONNECTED;
    }

    /* Strict dual-sensor qualification before flight. */
    if (flight_active == 0U)
    {
        if ((baro == 0) || (baro->connected == 0U))
        {
            return SYS_FAULT_BAROMETER_NOT_CONNECTED;
        }
        if (baro->data_ready == 0U)
        {
            return SYS_FAULT_BAROMETER_NO_DATA;
        }
        if (baro->pressure_valid == 0U)
        {
            return SYS_FAULT_BAROMETER_PRESSURE_INVALID;
        }
        if (baro->calibrated == 0U)
        {
            return SYS_FAULT_BAROMETER_NOT_CALIBRATED;
        }
    }

    if ((now_ms >= APP_SYSTEM_MONITOR_STARTUP_GRACE_MS) &&
        (system_status.scheduler_ok == 0U))
    {
        return SYS_FAULT_SCHEDULER_STALLED;
    }

    if ((eskf != 0) && (eskf->output_inhibited != 0U))
    {
        return SYS_FAULT_ESKF_DIVERGENCE;
    }

#if (APP_SDLOGGER_ENABLED != 0U)
    /* P48: recorder health blocks arming/preflight, never active stabilization. */
    if ((flight_active == 0U) &&
        (now_ms >= APP_SYSTEM_MONITOR_STARTUP_GRACE_MS) &&
        ((SDLogger_IsReady() == 0U) || (SDLogger_IsLogging() == 0U)))
    {
        return SYS_FAULT_SD_LOGGING;
    }
#endif

    if (now_ms >= APP_SYSTEM_MONITOR_STARTUP_GRACE_MS)
    {
        uint32_t imu_timestamp_us = IMU_GetLastSampleTimestampUs();
        uint32_t imu_age_us = (imu_timestamp_us != 0UL)
            ? (uint32_t)(now_us - imu_timestamp_us) : 0xFFFFFFFFUL;
        uint32_t lidar_age_us = ((lidar != 0) &&
                                 (lidar->last_sample_timestamp_us != 0UL))
            ? (uint32_t)(now_us - lidar->last_sample_timestamp_us)
            : 0xFFFFFFFFUL;
        uint32_t eskf_age_us = ((eskf != 0) &&
                                (eskf->last_public_output_timestamp_us != 0UL))
            ? (uint32_t)(now_us - eskf->last_public_output_timestamp_us)
            : 0xFFFFFFFFUL;

        if ((IMU_IsLastSampleValid() == 0U) ||
            (imu_age_us > APP_SYSTEM_MONITOR_IMU_STALE_US))
        {
            return SYS_FAULT_IMU_STALE;
        }

        if ((eskf == 0) || (eskf->healthy == 0U) ||
            (eskf_age_us > APP_SYSTEM_MONITOR_ESKF_PUBLIC_STALE_US))
        {
            return SYS_FAULT_ESKF_STALE;
        }

        if (flight_active != 0U)
        {
            if (vertical.usable == 0U)
            {
                return SYS_FAULT_LIDAR_STALE;
            }
        }
        else if ((lidar == 0) || (lidar->distance_valid == 0U) ||
                 (lidar_age_us > APP_SYSTEM_MONITOR_LIDAR_STALE_US))
        {
            return SYS_FAULT_LIDAR_STALE;
        }
    }

    return SYS_FAULT_NONE;
}

uint8_t SystemMonitor_IsFastFaultActive(void)
{
    return (SystemMonitor_GetFastFaultCode() != SYS_FAULT_NONE) ? 1U : 0U;
}

uint8_t SystemMonitor_IsActuatorFaultActive(void)
{
    SystemFaultCode_t fast_fault = SystemMonitor_GetFastFaultCode();
    SystemFaultCode_t latched_fault = system_status.fault_code;

    if (fast_fault != SYS_FAULT_NONE)
    {
        return 1U;
    }

    /* A 10 Hz diagnostic snapshot can lag a flight transition/recovery by up
     * to one monitor period.  During active flight, ignore only faults that
     * P48 explicitly defines as non-actuator-critical when the live fast path
     * above is already healthy.  All IMU/ESKF/scheduler faults stay fatal. */
    if (PreflightTrigger_IsFlightActive() != 0U)
    {
        if (latched_fault == SYS_FAULT_SD_LOGGING)
        {
            return 0U;
        }

        if ((latched_fault == SYS_FAULT_BAROMETER_NOT_CONNECTED) ||
            (latched_fault == SYS_FAULT_BAROMETER_NO_DATA) ||
            (latched_fault == SYS_FAULT_BAROMETER_PRESSURE_INVALID) ||
            (latched_fault == SYS_FAULT_BAROMETER_NOT_CALIBRATED) ||
            (latched_fault == SYS_FAULT_LIDAR_STALE))
        {
            VerticalSensorPolicyResult_t vertical =
                VerticalSensorPolicy_Evaluate(FullStateESKF_GetDataPtr());

            if (vertical.usable != 0U)
            {
                return 0U;
            }
        }
    }

    return (latched_fault != SYS_FAULT_NONE) ? 1U : 0U;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void SystemMonitor_Init(void)
{
    SystemMonitor_ResetStatus();
}

void SystemMonitor_Update(void)
{
    system_status.uptime_ms = millis();

    SystemMonitor_UpdateSensorHealth();
    SystemMonitor_UpdateSchedulerHealth();
    SystemMonitor_UpdateBusHealth();
    SystemMonitor_UpdateFaultCode();

    {
        uint8_t flight_active = PreflightTrigger_IsFlightActive();
        VerticalSensorPolicyResult_t vertical =
            VerticalSensorPolicy_Evaluate(FullStateESKF_GetDataPtr());
        uint8_t vertical_ok = (flight_active != 0U)
            ? vertical.usable
            : ((system_status.barometer_ok != 0U) &&
               (system_status.lidar_fresh != 0U));

        system_status.system_ok =
            (system_status.imu_ok != 0U) &&
            (system_status.scheduler_ok != 0U) &&
            (system_status.bus_ok != 0U) &&
            (system_status.fault_code == SYS_FAULT_NONE) &&
            ((system_status.uptime_ms < APP_SYSTEM_MONITOR_STARTUP_GRACE_MS) ||
             ((system_status.imu_fresh != 0U) &&
              (vertical_ok != 0U) &&
              (system_status.eskf_public_fresh != 0U)));
    }

    system_status.monitor_update_count++;

    SystemMonitor_UpdateLiveDebug();
}

SystemStatus_t SystemMonitor_GetStatus(void)
{
    return system_status;
}
