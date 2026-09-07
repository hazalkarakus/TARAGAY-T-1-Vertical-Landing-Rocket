#include "runtime_delay_guard.h"
#include "stm32f4xx_hal.h"

static volatile uint8_t s_runtime_started = 0U;
static volatile uint32_t s_violation_count = 0UL;
static volatile uint32_t s_last_requested_ms = 0UL;

void RuntimeDelayGuard_Init(void)
{
    s_runtime_started = 0U;
    s_violation_count = 0UL;
    s_last_requested_ms = 0UL;
}

void RuntimeDelayGuard_MarkRuntimeStarted(void)
{
    s_runtime_started = 1U;
}

uint32_t RuntimeDelayGuard_GetViolationCount(void)
{
    return s_violation_count;
}

uint32_t RuntimeDelayGuard_GetLastRequestedMs(void)
{
    return s_last_requested_ms;
}

/* Strong override of the STM32 HAL weak implementation.
 *
 * Before App_Run starts, legacy hardware-initialization code may still need
 * datasheet power/reset settling. Preserve the normal boot-time semantics.
 *
 * After App_Init marks runtime started, HAL_Delay() is forbidden: record the
 * violation and return immediately. This guarantees that an accidental HAL,
 * USB, sensor, SD or radio path cannot stall the deterministic flight loop. */
void HAL_Delay(uint32_t Delay)
{
    uint32_t tickstart;
    uint32_t wait;

    if (s_runtime_started != 0U)
    {
        s_last_requested_ms = Delay;
        s_violation_count++;
        return;
    }

    tickstart = HAL_GetTick();
    wait = Delay;
    if (wait < HAL_MAX_DELAY)
    {
        wait += (uint32_t)uwTickFreq;
    }
    while ((HAL_GetTick() - tickstart) < wait)
    {
        /* Boot-only compatibility path. Never reachable after App_Init. */
    }
}
