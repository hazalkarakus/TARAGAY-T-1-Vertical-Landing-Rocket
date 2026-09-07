#include "Core/Scheduler/scheduler.h"

#include "Core/Tasks/app_tasks.h"

#include "Common/app_config.h"

#include "Services/Timebase/timebase.h"

/* -------------------------------------------------------------------------- */
/* CPU monitor config                                                         */
/* -------------------------------------------------------------------------- */

#define CPU_MONITOR_WINDOW_US              1000000UL

#define CPU_STATE_IDLE                     0U
#define CPU_STATE_TASK_RUNNING             1U

/* -------------------------------------------------------------------------- */
/* Tasks                                                                      */
/* -------------------------------------------------------------------------- */

static Task_t tasks[] =
{
    /* Task 0: IMU */
    {
        "IMU_1KHZ",
        Task_IMU_1kHz,
        APP_TASK_IMU_PERIOD_US,
        0UL,
        0UL,
        APP_TASK_IMU_BUDGET_US,
        0UL, 0UL, 0UL, 0UL, 0UL, 0UL
    },

    /* Task 1: BMP585 + needle health */
    {
        "BARO_VALVE_200HZ",
        Task_BarometerValveHealth_200Hz,
        APP_TASK_BAROMETER_PERIOD_US,
        2000UL,
        0UL,
        APP_TASK_BAROMETER_BUDGET_US,
        0UL, 0UL, 0UL, 0UL, 0UL, 0UL
    },

    /* Task 2: LIDAR-Lite I2C2 DMA state machine */
    {
        "LIDAR_SERVICE_1KHZ_BEST_EFFORT",
        Task_Lidar_Service_1kHz,
        APP_TASK_LIDAR_PERIOD_US,
        3000UL,
        0UL,
        APP_TASK_LIDAR_BUDGET_US,
        0UL, 0UL, 0UL, 0UL, 0UL, 0UL
    },

    /* Task 3: NRF24 / SPI3 monitor-only receiver */
    {
        "NRF_SERVO_200HZ",
        Task_NRFMonitor_200Hz,
        APP_TASK_NRF_PERIOD_US,
        4000UL,
        0UL,
        APP_TASK_NRF_BUDGET_US,
        0UL, 0UL, 0UL, 0UL, 0UL, 0UL
    },

    /* Task 4: Full-State ESKF correction / covariance service */
    {
        "FULL_ESKF_CORR_200HZ",
        Task_FullESKFCorrection_200Hz,
        APP_TASK_FULL_ESKF_CORRECTION_PERIOD_US,
        1000UL,
        0UL,
        APP_TASK_FULL_ESKF_CORRECTION_BUDGET_US,
        0UL, 0UL, 0UL, 0UL, 0UL, 0UL
    },

    /* Task 5: coexistence monitor */
    {
        "V815_MONITOR_10HZ",
        Task_IntegrationMonitor_10Hz,
        APP_TASK_SYSMON_PERIOD_US,
        4000UL,
        0UL,
        APP_TASK_SYSMON_BUDGET_US,
        0UL, 0UL, 0UL, 0UL, 0UL, 0UL
    },

    /* Task 6: heavy covariance propagation, hard-separated from 200 Hz public output */
    {
        "FULL_ESKF_COV_25HZ",
        Task_FullESKFCovariance_25Hz,
        APP_TASK_FULL_ESKF_COVARIANCE_PERIOD_US,
        3000UL,
        0UL,
        APP_TASK_FULL_ESKF_COVARIANCE_BUDGET_US,
        0UL, 0UL, 0UL, 0UL, 0UL, 0UL
    }
};

#define TASK_COUNT   (sizeof(tasks) / sizeof(tasks[0]))

/* P44: IMU and the 200 Hz public ESKF output remain first. The measured
 * 450 us covariance service is admitted before best-effort LiDAR once every
 * 40 ms; LiDAR keeps the proven one-generation carry/skip protection.
 * Task-array/UART indices and every period/phase remain unchanged.
 * IMU -> ESKF -> COV -> LIDAR -> BARO -> NRF -> MON. */
static const uint8_t scheduler_priority_order[TASK_COUNT] =
{
    0U, 4U, 6U, 2U, 1U, 3U, 5U
};

/* -------------------------------------------------------------------------- */
/* CPU monitor private variables                                              */
/* -------------------------------------------------------------------------- */

static uint32_t cpu_window_start_us = 0UL;
static uint32_t cpu_busy_accum_us = 0UL;
static uint32_t cpu_external_busy_accum_us = 0UL;

static uint32_t cpu_task_run_accum = 0UL;
static uint32_t cpu_loop_accum = 0UL;
static uint32_t cpu_idle_loop_accum = 0UL;
static uint32_t cpu_active_loop_accum = 0UL;

/* Task bazlı yük */
static uint32_t cpu_task_busy_accum_us[TASK_COUNT];

/* P35: once a slow task is found too close to the next IMU release,
 * checking it again in the same 1 ms IMU generation cannot improve the
 * available slack.  Latch that defer until task-0 runs again. */
static uint32_t slow_task_blocked_imu_generation[TASK_COUNT];
/* P39: a LiDAR service release deferred for IMU safety is carried across the
 * next IMU slot instead of being discarded. This preserves deterministic IMU
 * timing while restoring enough state-machine service cadence for ~200 Hz
 * physical ranging. */
static uint8_t lidar_intentional_defer_pending = 0U;
static uint32_t lidar_intentional_defer_imu_generation = 0UL;

/* -------------------------------------------------------------------------- */
/* Live Expressions - CPU Monitor                                             */
/* -------------------------------------------------------------------------- */

/*
 * Percent x100:
 *
 * 5814 = 58.14%
 * 100  = 1.00%
 */

volatile uint32_t cpu_load_percent_x100 = 0UL;
volatile uint32_t cpu_idle_percent_x100 = 10000UL;

volatile float cpu_load_percent = 0.0f;
volatile float cpu_idle_percent = 100.0f;

volatile uint32_t cpu_busy_time_us_1s = 0UL;
volatile uint32_t cpu_idle_time_us_1s = 0UL;
volatile uint32_t cpu_window_time_us = 0UL;

volatile uint32_t cpu_task_run_count_1s = 0UL;
volatile uint32_t cpu_loop_count_1s = 0UL;
volatile uint32_t cpu_idle_loop_count_1s = 0UL;
volatile uint32_t cpu_active_loop_count_1s = 0UL;

volatile uint32_t cpu_total_loop_count = 0UL;
volatile uint32_t cpu_total_idle_loop_count = 0UL;
volatile uint32_t cpu_total_active_loop_count = 0UL;
volatile uint32_t cpu_total_task_run_count = 0UL;

volatile uint8_t cpu_current_state = CPU_STATE_IDLE;

volatile uint32_t cpu_last_task_index = 0UL;
volatile uint32_t cpu_last_task_exec_us = 0UL;
volatile uint32_t cpu_last_task_over_budget = 0UL;

volatile uint32_t cpu_max_task_exec_us = 0UL;
volatile uint32_t cpu_total_busy_time_us = 0UL;
volatile uint32_t scheduler_release_realign_count = 0UL;

/* R8R10 observer-only scheduler diagnostics. */
volatile uint32_t scheduler_task_last_lateness_us[TASK_COUNT];
volatile uint32_t scheduler_task_max_lateness_us[TASK_COUNT];
volatile uint32_t scheduler_task_defer_count[TASK_COUNT];
volatile uint32_t scheduler_task_defer_streak[TASK_COUNT];
volatile uint32_t scheduler_task_max_defer_streak[TASK_COUNT];
volatile uint32_t scheduler_task_last_defer_slack_us[TASK_COUNT];
volatile uint32_t scheduler_slow_task_defer_count = 0UL;
/* P43: LiDAR is a best-effort 1 kHz state-machine service, not a hard 1 kHz
 * physical ranging deadline. These counters distinguish a one-IMU-slot carry
 * from a service release that had to be skipped/re-anchored. */
volatile uint32_t lidar_scheduler_carry_count = 0UL;
volatile uint32_t lidar_scheduler_carry_served_count = 0UL;
volatile uint32_t lidar_scheduler_best_effort_skip_count = 0UL;
volatile uint32_t cpu_external_load_percent_x100 = 0UL;

/* Task bazlı CPU load */
volatile uint32_t cpu_task0_load_percent_x100 = 0UL;
volatile uint32_t cpu_task1_load_percent_x100 = 0UL;
volatile uint32_t cpu_task2_load_percent_x100 = 0UL;
volatile uint32_t cpu_task3_load_percent_x100 = 0UL;
volatile uint32_t cpu_task4_load_percent_x100 = 0UL;

volatile float cpu_task0_load_percent = 0.0f;
volatile float cpu_task1_load_percent = 0.0f;
volatile float cpu_task2_load_percent = 0.0f;

volatile uint32_t cpu_task0_busy_time_us_1s = 0UL;
volatile uint32_t cpu_task1_busy_time_us_1s = 0UL;
volatile uint32_t cpu_task2_busy_time_us_1s = 0UL;

/* -------------------------------------------------------------------------- */
/* CPU monitor                                                                */
/* -------------------------------------------------------------------------- */

static void Scheduler_CPU_ResetWindow(uint32_t now_us)
{
    cpu_window_start_us = now_us;

    cpu_busy_accum_us = 0UL;
    cpu_external_busy_accum_us = 0UL;

    cpu_task_run_accum = 0UL;
    cpu_loop_accum = 0UL;
    cpu_idle_loop_accum = 0UL;
    cpu_active_loop_accum = 0UL;

    for (uint32_t i = 0UL; i < TASK_COUNT; i++)
    {
        cpu_task_busy_accum_us[i] = 0UL;
        slow_task_blocked_imu_generation[i] = 0xFFFFFFFFUL;
    }
}

/* -------------------------------------------------------------------------- */

static uint32_t Scheduler_CPU_ComputePercentX100(uint32_t part_us, uint32_t total_us)
{
    if (total_us == 0UL)
    {
        return 0UL;
    }

    uint32_t result =
        (uint32_t)(((uint64_t)part_us * 10000ULL) /
                   (uint64_t)total_us);

    if (result > 10000UL)
    {
        result = 10000UL;
    }

    return result;
}

/* -------------------------------------------------------------------------- */

static void Scheduler_CPU_UpdateWindow(uint32_t now_us)
{
    uint32_t elapsed_us =
        now_us - cpu_window_start_us;

    if (elapsed_us < CPU_MONITOR_WINDOW_US)
    {
        return;
    }

    uint32_t busy_us = cpu_busy_accum_us;

    if (busy_us > elapsed_us)
    {
        busy_us = elapsed_us;
    }

    uint32_t idle_us = elapsed_us - busy_us;

    cpu_window_time_us = elapsed_us;

    cpu_busy_time_us_1s = busy_us;
    cpu_idle_time_us_1s = idle_us;

    cpu_task_run_count_1s = cpu_task_run_accum;

    cpu_loop_count_1s = cpu_loop_accum;
    cpu_idle_loop_count_1s = cpu_idle_loop_accum;
    cpu_active_loop_count_1s = cpu_active_loop_accum;

    cpu_load_percent_x100 =
        Scheduler_CPU_ComputePercentX100(busy_us, elapsed_us);

    cpu_idle_percent_x100 =
        10000UL - cpu_load_percent_x100;

    cpu_load_percent =
        (float)cpu_load_percent_x100 * 0.01f;

    cpu_idle_percent =
        (float)cpu_idle_percent_x100 * 0.01f;

    /*
     * Task bazlı CPU load.
     */

    cpu_task0_busy_time_us_1s = cpu_task_busy_accum_us[0];
    cpu_task1_busy_time_us_1s = cpu_task_busy_accum_us[1];
    cpu_task2_busy_time_us_1s = cpu_task_busy_accum_us[2];

    cpu_external_load_percent_x100 =
        Scheduler_CPU_ComputePercentX100(cpu_external_busy_accum_us, elapsed_us);

    cpu_task0_load_percent_x100 =
        Scheduler_CPU_ComputePercentX100(cpu_task_busy_accum_us[0], elapsed_us);

    cpu_task1_load_percent_x100 =
        Scheduler_CPU_ComputePercentX100(cpu_task_busy_accum_us[1], elapsed_us);

    cpu_task2_load_percent_x100 =
        Scheduler_CPU_ComputePercentX100(cpu_task_busy_accum_us[2], elapsed_us);
    cpu_task3_load_percent_x100 =
        Scheduler_CPU_ComputePercentX100(cpu_task_busy_accum_us[3], elapsed_us);
    cpu_task4_load_percent_x100 =
        Scheduler_CPU_ComputePercentX100(cpu_task_busy_accum_us[4], elapsed_us);

    cpu_task0_load_percent =
        (float)cpu_task0_load_percent_x100 * 0.01f;

    cpu_task1_load_percent =
        (float)cpu_task1_load_percent_x100 * 0.01f;

    cpu_task2_load_percent =
        (float)cpu_task2_load_percent_x100 * 0.01f;

    Scheduler_CPU_ResetWindow(now_us);
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void Scheduler_Init(void)
{
    uint32_t now = micros();

    for (uint32_t i = 0UL; i < TASK_COUNT; i++)
    {
        tasks[i].next_run_us = now + tasks[i].phase_us;

        tasks[i].last_exec_us = 0UL;
        tasks[i].max_exec_us = 0UL;

        tasks[i].run_count = 0UL;
        tasks[i].overrun_count = 0UL;
        tasks[i].deadline_miss_count = 0UL;
        tasks[i].release_realign_count = 0UL;
        scheduler_task_last_lateness_us[i] = 0UL;
        scheduler_task_max_lateness_us[i] = 0UL;
        scheduler_task_defer_count[i] = 0UL;
        scheduler_task_defer_streak[i] = 0UL;
        scheduler_task_max_defer_streak[i] = 0UL;
        scheduler_task_last_defer_slack_us[i] = 0UL;

        cpu_task_busy_accum_us[i] = 0UL;
    }

    cpu_load_percent_x100 = 0UL;
    cpu_idle_percent_x100 = 10000UL;

    cpu_load_percent = 0.0f;
    cpu_idle_percent = 100.0f;

    cpu_busy_time_us_1s = 0UL;
    cpu_idle_time_us_1s = 0UL;
    cpu_window_time_us = 0UL;

    cpu_task_run_count_1s = 0UL;
    cpu_loop_count_1s = 0UL;
    cpu_idle_loop_count_1s = 0UL;
    cpu_active_loop_count_1s = 0UL;

    cpu_total_loop_count = 0UL;
    cpu_total_idle_loop_count = 0UL;
    cpu_total_active_loop_count = 0UL;
    cpu_total_task_run_count = 0UL;

    cpu_current_state = CPU_STATE_IDLE;

    cpu_last_task_index = 0UL;
    cpu_last_task_exec_us = 0UL;
    cpu_last_task_over_budget = 0UL;

    cpu_max_task_exec_us = 0UL;
    cpu_total_busy_time_us = 0UL;
    scheduler_release_realign_count = 0UL;
    scheduler_slow_task_defer_count = 0UL;
    lidar_intentional_defer_pending = 0U;
    lidar_intentional_defer_imu_generation = 0UL;
    lidar_scheduler_carry_count = 0UL;
    lidar_scheduler_carry_served_count = 0UL;
    lidar_scheduler_best_effort_skip_count = 0UL;
    cpu_external_load_percent_x100 = 0UL;

    cpu_task0_load_percent_x100 = 0UL;
    cpu_task1_load_percent_x100 = 0UL;
    cpu_task2_load_percent_x100 = 0UL;
    cpu_task3_load_percent_x100 = 0UL;
    cpu_task4_load_percent_x100 = 0UL;

    cpu_task0_load_percent = 0.0f;
    cpu_task1_load_percent = 0.0f;
    cpu_task2_load_percent = 0.0f;

    cpu_task0_busy_time_us_1s = 0UL;
    cpu_task1_busy_time_us_1s = 0UL;
    cpu_task2_busy_time_us_1s = 0UL;

    Scheduler_CPU_ResetWindow(now);
}

/* -------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------- */
/*
 * Rebase task release times after an intentional blocking maintenance window
 * (for example final FATFS truncate/sync/close after a completed bench run).
 *
 * Runtime counters are preserved. Only future release timestamps and the CPU
 * monitor window are re-anchored, so the scheduler does not "catch up" by
 * repeatedly reporting deadlines that occurred while maintenance was active.
 */
void Scheduler_Rebase(void)
{
    uint32_t now = micros();

    for (uint32_t i = 0UL; i < TASK_COUNT; i++)
    {
        tasks[i].next_run_us = now + tasks[i].phase_us;
    }

    cpu_current_state = CPU_STATE_IDLE;
    cpu_last_task_over_budget = 0UL;
    /* An intentional maintenance rebase ends any outstanding best-effort
     * LiDAR carry. Runtime counters remain preserved. */
    lidar_intentional_defer_pending = 0U;
    lidar_intentional_defer_imu_generation = tasks[0].run_count;

    Scheduler_CPU_ResetWindow(now);
}

static uint32_t Scheduler_TaskSlotReserveUs(uint32_t index)
{
    switch (index)
    {
        case 1UL: return APP_P34_BARO_SLOT_RESERVE_US;
        case 2UL: return APP_P34_LIDAR_SLOT_RESERVE_US;
        case 3UL: return APP_P34_NRF_SLOT_RESERVE_US;
        case 4UL: return APP_P34_ESKF_SLOT_RESERVE_US;
        case 5UL: return APP_P34_SYSMON_SLOT_RESERVE_US;
        case 6UL: return APP_P44_COVARIANCE_SLOT_RESERVE_US;
        default:  return 0UL;
    }
}

uint32_t Scheduler_GetTimeUntilIMUReleaseUs(void)
{
    uint32_t now = micros();
    int32_t delta = (int32_t)(tasks[0].next_run_us - now);

    return (delta > 0) ? (uint32_t)delta : 0UL;
}

uint8_t Scheduler_HasIMUSlack(uint32_t required_us)
{
    uint32_t slack = Scheduler_GetTimeUntilIMUReleaseUs();
    return (slack > (required_us + APP_P34_IMU_GUARD_US)) ? 1U : 0U;
}

void Scheduler_RecordExternalBusyTime(uint32_t exec_us)
{
    cpu_external_busy_accum_us += exec_us;
    cpu_busy_accum_us += exec_us;
    cpu_total_busy_time_us += exec_us;
}

void Scheduler_Run(void)
{
    uint8_t any_task_ran = 0U;

    uint32_t now = micros();

    cpu_total_loop_count++;
    cpu_loop_accum++;


    for (uint32_t order_index = 0UL; order_index < TASK_COUNT; order_index++)
    {
        uint32_t i = scheduler_priority_order[order_index];
        now = micros();

        if ((int32_t)(now - tasks[i].next_run_us) >= 0)
        {
            /* P34: task 0 is the hard 1 kHz owner. A slower cooperative task
             * stays pending when its normal execution cost would intrude into
             * the next IMU release; it is serviced immediately after that IMU
             * slot instead of creating an avoidable IMU deadline miss. */
            if (i != 0UL)
            {
                uint32_t imu_generation = tasks[0].run_count;

                if (slow_task_blocked_imu_generation[i] == imu_generation)
                {
                    continue;
                }

                uint32_t slot_reserve_us = Scheduler_TaskSlotReserveUs(i);
                uint32_t imu_slack_us = Scheduler_GetTimeUntilIMUReleaseUs();

                if (imu_slack_us <=
                    (slot_reserve_us + APP_P34_IMU_GUARD_US))
                {
                    scheduler_slow_task_defer_count++;
                    scheduler_task_defer_count[i]++;
                    scheduler_task_defer_streak[i]++;
                    if (scheduler_task_defer_streak[i] >
                        scheduler_task_max_defer_streak[i])
                    {
                        scheduler_task_max_defer_streak[i] =
                            scheduler_task_defer_streak[i];
                    }
                    scheduler_task_last_defer_slack_us[i] = imu_slack_us;
                    slow_task_blocked_imu_generation[i] = imu_generation;

                    /* P43: a LiDAR release may be carried across exactly one
                     * protected IMU generation. If the carried release still
                     * cannot fit after the next IMU, discard it and re-anchor
                     * instead of hiding an unbounded number of missed service
                     * opportunities behind one boolean flag. */
                    if (i == 2UL)
                    {
                        if (lidar_intentional_defer_pending == 0U)
                        {
                            lidar_intentional_defer_pending = 1U;
                            lidar_intentional_defer_imu_generation =
                                imu_generation;
                            lidar_scheduler_carry_count++;
                        }
                        else if (imu_generation !=
                                 lidar_intentional_defer_imu_generation)
                        {
                            tasks[i].next_run_us = now + tasks[i].period_us;
                            tasks[i].release_realign_count++;
                            lidar_scheduler_best_effort_skip_count++;
                            lidar_intentional_defer_pending = 0U;
                        }
                    }
                    continue;
                }

                slow_task_blocked_imu_generation[i] = 0xFFFFFFFFUL;
            }

            any_task_ran = 1U;

            uint32_t lateness_us =
                (uint32_t)(now - tasks[i].next_run_us);

            scheduler_task_last_lateness_us[i] = lateness_us;
            if (lateness_us > scheduler_task_max_lateness_us[i])
            {
                scheduler_task_max_lateness_us[i] = lateness_us;
            }
            scheduler_task_defer_streak[i] = 0UL;

            if (i == 2UL)
            {
                /* LiDAR's 1 kHz rate is a best-effort service cadence. Never
                 * replay old releases and never report a hard task deadline
                 * miss for a service call that does not own a 1 kHz sample.
                 * A protected carry is serviced once and then re-anchored. */
                if (lidar_intentional_defer_pending != 0U)
                {
                    tasks[i].next_run_us = now + tasks[i].period_us;
                    lidar_intentional_defer_pending = 0U;
                    lidar_scheduler_carry_served_count++;
                }
                else if (lateness_us >= tasks[i].period_us)
                {
                    uint32_t skipped_releases =
                        lateness_us / tasks[i].period_us;

                    if (skipped_releases == 0UL)
                    {
                        skipped_releases = 1UL;
                    }

                    tasks[i].release_realign_count++;
                    lidar_scheduler_best_effort_skip_count += skipped_releases;
                    tasks[i].next_run_us = now + tasks[i].period_us;
                }
                else
                {
                    tasks[i].next_run_us += tasks[i].period_us;
                }
            }
            else
            {
                if (lateness_us >= tasks[i].period_us)
                {
                    tasks[i].deadline_miss_count++;
                    tasks[i].release_realign_count++;
                    scheduler_release_realign_count++;

                    /* P29: discard missed releases and anchor the next one to
                     * real time.  A saturated CPU must not spend the rest of
                     * run executing an ever-growing historical backlog. */
                    tasks[i].next_run_us = now + tasks[i].period_us;
                }
                else
                {
                    /* Small jitter preserves the original phase. */
                    tasks[i].next_run_us += tasks[i].period_us;
                }
            }

            cpu_current_state = CPU_STATE_TASK_RUNNING;
            cpu_last_task_index = i;

            uint32_t start_us = micros();

            tasks[i].function();

            uint32_t end_us = micros();

            uint32_t exec_us = end_us - start_us;

            tasks[i].last_exec_us = exec_us;

            if (exec_us > tasks[i].max_exec_us)
            {
                tasks[i].max_exec_us = exec_us;
            }

            if (exec_us > tasks[i].budget_us)
            {
                tasks[i].overrun_count++;
                cpu_last_task_over_budget = 1UL;
            }
            else
            {
                cpu_last_task_over_budget = 0UL;
            }

            tasks[i].run_count++;

            cpu_last_task_exec_us = exec_us;

            if (exec_us > cpu_max_task_exec_us)
            {
                cpu_max_task_exec_us = exec_us;
            }

            cpu_busy_accum_us += exec_us;
            cpu_total_busy_time_us += exec_us;

            cpu_task_busy_accum_us[i] += exec_us;

            cpu_task_run_accum++;
            cpu_total_task_run_count++;
        }
    }

    cpu_current_state = CPU_STATE_IDLE;

    if (any_task_ran != 0U)
    {
        cpu_total_active_loop_count++;
        cpu_active_loop_accum++;
    }
    else
    {
        cpu_total_idle_loop_count++;
        cpu_idle_loop_accum++;
    }

    Scheduler_CPU_UpdateWindow(micros());
}

/* -------------------------------------------------------------------------- */

uint32_t Scheduler_GetTaskCount(void)
{
    return TASK_COUNT;
}

/* -------------------------------------------------------------------------- */

Task_t *Scheduler_GetTask(uint32_t index)
{
    if (index >= TASK_COUNT)
    {
        return 0;
    }

    return &tasks[index];
}
