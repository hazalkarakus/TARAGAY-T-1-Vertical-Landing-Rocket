#include "Services/DebugDashboard/debug_dashboard.h"

#include "Core/Scheduler/scheduler.h"

/*
 * Live Expressions için scheduler debug değişkenleri.
 *
 * Bunlar artık main.c içinde değil.
 */

volatile uint32_t scheduler_task_count = 0;

volatile uint32_t imu_last_exec_us = 0;
volatile uint32_t imu_max_exec_us = 0;
volatile uint32_t imu_overrun_count = 0;
volatile uint32_t imu_deadline_miss_count = 0;
volatile uint32_t imu_run_count = 0;

volatile uint32_t barometer_last_exec_us = 0;
volatile uint32_t barometer_max_exec_us = 0;
volatile uint32_t barometer_overrun_count = 0;
volatile uint32_t barometer_deadline_miss_count = 0;
volatile uint32_t barometer_run_count = 0;

volatile uint32_t sysmon_last_exec_us = 0;
volatile uint32_t sysmon_max_exec_us = 0;
volatile uint32_t sysmon_overrun_count = 0;
volatile uint32_t sysmon_deadline_miss_count = 0;
volatile uint32_t sysmon_run_count = 0;

void DebugDashboard_Init(void)
{
    scheduler_task_count = 0UL;

    imu_last_exec_us = 0UL;
    imu_max_exec_us = 0UL;
    imu_overrun_count = 0UL;
    imu_deadline_miss_count = 0UL;
    imu_run_count = 0UL;

    barometer_last_exec_us = 0UL;
    barometer_max_exec_us = 0UL;
    barometer_overrun_count = 0UL;
    barometer_deadline_miss_count = 0UL;
    barometer_run_count = 0UL;

    sysmon_last_exec_us = 0UL;
    sysmon_max_exec_us = 0UL;
    sysmon_overrun_count = 0UL;
    sysmon_deadline_miss_count = 0UL;
    sysmon_run_count = 0UL;
}

void DebugDashboard_Update(void)
{
    scheduler_task_count = Scheduler_GetTaskCount();

    /*
     * Task 0 = IMU_1KHZ
     */
    Task_t *imu_task = Scheduler_GetTask(0UL);

    if (imu_task != 0)
    {
        imu_last_exec_us = imu_task->last_exec_us;
        imu_max_exec_us = imu_task->max_exec_us;
        imu_overrun_count = imu_task->overrun_count;
        imu_deadline_miss_count = imu_task->deadline_miss_count;
        imu_run_count = imu_task->run_count;
    }

    /*
     * Task 1 = BAROMETER_100HZ
     */
    Task_t *barometer_task = Scheduler_GetTask(1UL);

    if (barometer_task != 0)
    {
        barometer_last_exec_us = barometer_task->last_exec_us;
        barometer_max_exec_us = barometer_task->max_exec_us;
        barometer_overrun_count = barometer_task->overrun_count;
        barometer_deadline_miss_count = barometer_task->deadline_miss_count;
        barometer_run_count = barometer_task->run_count;
    }

    /*
     * Task 2 = SYSTEM_MONITOR_10HZ
     */
    Task_t *sysmon_task = Scheduler_GetTask(2UL);

    if (sysmon_task != 0)
    {
        sysmon_last_exec_us = sysmon_task->last_exec_us;
        sysmon_max_exec_us = sysmon_task->max_exec_us;
        sysmon_overrun_count = sysmon_task->overrun_count;
        sysmon_deadline_miss_count = sysmon_task->deadline_miss_count;
        sysmon_run_count = sysmon_task->run_count;
    }
}
