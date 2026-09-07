#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "main.h"
#include <stdint.h>

typedef void (*TaskFunction_t)(void);

typedef struct
{
    const char *name;

    TaskFunction_t function;

    uint32_t period_us;
    uint32_t phase_us;
    uint32_t next_run_us;
    uint32_t budget_us;

    uint32_t last_exec_us;
    uint32_t max_exec_us;

    uint32_t run_count;
    uint32_t overrun_count;
    uint32_t deadline_miss_count;
    uint32_t release_realign_count;

} Task_t;

void Scheduler_Init(void);
void Scheduler_Run(void);
void Scheduler_Rebase(void);
uint32_t Scheduler_GetTimeUntilIMUReleaseUs(void);
uint8_t Scheduler_HasIMUSlack(uint32_t required_us);
void Scheduler_RecordExternalBusyTime(uint32_t exec_us);

uint32_t Scheduler_GetTaskCount(void);
Task_t *Scheduler_GetTask(uint32_t index);

/* Live CPU-load metrics produced by scheduler.c. */
extern volatile uint32_t cpu_load_percent_x100;
extern volatile uint32_t cpu_idle_percent_x100;
extern volatile float cpu_load_percent;
extern volatile float cpu_idle_percent;
extern volatile uint32_t scheduler_release_realign_count;

/* P112R12R8R10 root-cause diagnostics only.  These counters do not alter
 * admission, release, priority or task execution semantics. */
extern volatile uint32_t scheduler_task_last_lateness_us[7];
extern volatile uint32_t scheduler_task_max_lateness_us[7];
extern volatile uint32_t scheduler_task_defer_count[7];
extern volatile uint32_t scheduler_task_defer_streak[7];
extern volatile uint32_t scheduler_task_max_defer_streak[7];
extern volatile uint32_t scheduler_task_last_defer_slack_us[7];
extern volatile uint32_t scheduler_slow_task_defer_count;
extern volatile uint32_t lidar_scheduler_carry_count;
extern volatile uint32_t lidar_scheduler_carry_served_count;
extern volatile uint32_t lidar_scheduler_best_effort_skip_count;
extern volatile uint32_t cpu_external_load_percent_x100;
extern volatile uint32_t cpu_task0_load_percent_x100;
extern volatile uint32_t cpu_task1_load_percent_x100;
extern volatile uint32_t cpu_task2_load_percent_x100;
extern volatile uint32_t cpu_task3_load_percent_x100;
extern volatile uint32_t cpu_task4_load_percent_x100;

#endif
