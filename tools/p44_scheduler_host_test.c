#include <stdint.h>
#include <stdio.h>

#include "Core/Scheduler/scheduler.h"

static uint32_t host_now_us;
static uint32_t imu_exec_us;
static uint32_t covariance_exec_us;
static uint32_t lidar_runs;
static uint32_t covariance_runs;
static int failures;

uint32_t micros(void)
{
    return host_now_us;
}

void Task_IMU_1kHz(void)
{
    host_now_us += imu_exec_us;
}

void Task_Lidar_Service_1kHz(void)
{
    lidar_runs++;
    host_now_us += 20UL;
}

void Task_BarometerValveHealth_200Hz(void) {}
void Task_NRFMonitor_200Hz(void) {}
void Task_FullESKFCorrection_200Hz(void) {}
void Task_FullESKFCovariance_25Hz(void)
{
    covariance_runs++;
    host_now_us += covariance_exec_us;
}
void Task_IntegrationMonitor_10Hz(void) {}

static void Expect(int condition, const char *name)
{
    if (!condition)
    {
        printf("FAIL: %s\n", name);
        failures++;
    }
}

static void ParkAllTasks(uint32_t future_us)
{
    for (uint32_t i = 0UL; i < Scheduler_GetTaskCount(); ++i)
    {
        Scheduler_GetTask(i)->next_run_us = future_us;
    }
}

static void TestOneGenerationCarryServed(void)
{
    Task_t *imu;
    Task_t *lidar;

    host_now_us = 0UL;
    imu_exec_us = 750UL;
    lidar_runs = 0UL;
    Scheduler_Init();
    ParkAllTasks(100000UL);

    imu = Scheduler_GetTask(0UL);
    lidar = Scheduler_GetTask(2UL);
    imu->next_run_us = 10000UL;
    lidar->next_run_us = 10000UL;

    host_now_us = 10000UL;
    Scheduler_Run();
    Expect(lidar_runs == 0UL, "LiDAR first release is carried");
    Expect(lidar_scheduler_carry_count == 1UL, "one carry armed");

    imu_exec_us = 100UL;
    host_now_us = 11000UL;
    Scheduler_Run();
    Expect(lidar_runs == 1UL, "carried LiDAR release is serviced once");
    Expect(lidar_scheduler_carry_served_count == 1UL,
           "carry-served counter increments");
    Expect(lidar_scheduler_best_effort_skip_count == 0UL,
           "served carry does not count as skip");
    Expect(lidar->deadline_miss_count == 0UL,
           "served carry is not a hard deadline miss");
}

static void TestSecondGenerationBecomesSkip(void)
{
    Task_t *imu;
    Task_t *lidar;

    host_now_us = 0UL;
    imu_exec_us = 750UL;
    lidar_runs = 0UL;
    Scheduler_Init();
    ParkAllTasks(100000UL);

    imu = Scheduler_GetTask(0UL);
    lidar = Scheduler_GetTask(2UL);
    imu->next_run_us = 20000UL;
    lidar->next_run_us = 20000UL;

    host_now_us = 20000UL;
    Scheduler_Run();
    host_now_us = 21000UL;
    Scheduler_Run();

    Expect(lidar_runs == 0UL, "second protected slot still does not run LiDAR");
    Expect(lidar_scheduler_carry_count == 1UL, "only one carry is armed");
    Expect(lidar_scheduler_carry_served_count == 0UL,
           "unserved carry is not reported served");
    Expect(lidar_scheduler_best_effort_skip_count == 1UL,
           "second-generation defer becomes one best-effort skip");
    Expect(lidar->deadline_miss_count == 0UL,
           "best-effort skip is not a hard deadline miss");
    Expect(lidar->release_realign_count == 1UL,
           "best-effort skip re-anchors the release");
}

static void TestLateReleaseReanchorsWithoutHardMiss(void)
{
    Task_t *imu;
    Task_t *lidar;

    host_now_us = 0UL;
    imu_exec_us = 0UL;
    lidar_runs = 0UL;
    Scheduler_Init();
    ParkAllTasks(100000UL);

    imu = Scheduler_GetTask(0UL);
    lidar = Scheduler_GetTask(2UL);
    imu->next_run_us = 33000UL;
    lidar->next_run_us = 30000UL;

    host_now_us = 32050UL;
    Scheduler_Run();

    Expect(lidar_runs == 1UL, "late best-effort LiDAR release still services");
    Expect(lidar_scheduler_best_effort_skip_count == 2UL,
           "two elapsed service releases are counted");
    Expect(lidar->deadline_miss_count == 0UL,
           "late best-effort service is not a hard deadline miss");
    Expect(lidar->release_realign_count == 1UL,
           "late best-effort service re-anchors once");
    Expect(lidar->next_run_us == 33050UL,
           "next LiDAR service is anchored to dispatch time");
}

static void TestCovariancePreemptsBestEffortLidar(void)
{
    Task_t *imu;
    Task_t *lidar;
    Task_t *covariance;

    host_now_us = 0UL;
    imu_exec_us = 452UL;
    covariance_exec_us = 450UL;
    lidar_runs = 0UL;
    covariance_runs = 0UL;
    Scheduler_Init();
    ParkAllTasks(100000UL);

    imu = Scheduler_GetTask(0UL);
    lidar = Scheduler_GetTask(2UL);
    covariance = Scheduler_GetTask(6UL);
    imu->next_run_us = 10000UL;
    lidar->next_run_us = 10000UL;
    covariance->next_run_us = 10000UL;

    host_now_us = 10000UL;
    Scheduler_Run();

    Expect(covariance_runs == 1UL,
           "covariance runs in measured worst-case IMU slot");
    Expect(covariance->deadline_miss_count == 0UL,
           "covariance dispatch has no deadline miss");
    Expect(lidar_runs == 0UL,
           "LiDAR yields to due covariance");
    Expect(lidar_scheduler_carry_count == 1UL,
           "displaced LiDAR release is carried");

    imu_exec_us = 100UL;
    host_now_us = 11000UL;
    Scheduler_Run();

    Expect(lidar_runs == 1UL,
           "carried LiDAR runs in the next IMU generation");
    Expect(lidar_scheduler_carry_served_count == 1UL,
           "covariance displacement uses normal carry accounting");
}

static void TestTwoSecondCovarianceCadence(void)
{
    host_now_us = 0UL;
    imu_exec_us = 322UL;
    covariance_exec_us = 450UL;
    lidar_runs = 0UL;
    covariance_runs = 0UL;
    Scheduler_Init();

    while (host_now_us < 2000000UL)
    {
        uint32_t before_us = host_now_us;
        Scheduler_Run();
        if (host_now_us == before_us)
        {
            host_now_us++;
        }
    }

    Expect((covariance_runs >= 49UL) && (covariance_runs <= 50UL),
           "covariance sustains approximately 25 Hz for two seconds");
    Expect(Scheduler_GetTask(6UL)->deadline_miss_count == 0UL,
           "sustained covariance has zero deadline misses");
    Expect(lidar_runs >= 1900UL,
           "best-effort LiDAR retains more than 950 Hz service cadence");
    Expect(Scheduler_GetTask(0UL)->deadline_miss_count == 0UL,
           "sustained covariance creates no IMU deadline miss");
}

int main(void)
{
    TestOneGenerationCarryServed();
    TestSecondGenerationBecomesSkip();
    TestLateReleaseReanchorsWithoutHardMiss();
    TestCovariancePreemptsBestEffortLidar();
    TestTwoSecondCovarianceCadence();

    if (failures != 0)
    {
        printf("P44 scheduler host tests: %d failure(s)\n", failures);
        return 1;
    }

    printf("P44 scheduler host tests: PASS\n");
    return 0;
}
