#include "Services/Timebase/timebase.h"

#include "tim.h"

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* Live Expressions                                                           */
/* -------------------------------------------------------------------------- */

volatile uint8_t timebase_initialized = 0U;

volatile uint32_t timebase_timer_clock_hz = 0UL;
volatile uint32_t timebase_timer_prescaler = 0UL;

volatile uint32_t timebase_micros_debug = 0UL;
volatile uint32_t timebase_millis_debug = 0UL;

volatile uint32_t timebase_start_error_count = 0UL;

/* -------------------------------------------------------------------------- */

void Timebase_Init(void)
{
    uint32_t pclk1_hz;
    uint32_t timer_clock_hz;

    timebase_initialized = 0U;

    /*
     * CubeMX önce MX_TIM2_Init() çağırmış olmalı.
     */
    if (htim2.Instance != TIM2)
    {
        timebase_start_error_count++;
        return;
    }

    pclk1_hz = HAL_RCC_GetPCLK1Freq();
    timer_clock_hz = pclk1_hz;

    /*
     * APB1 prescaler 1 değilse timer clock = 2 x PCLK1.
     */
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
    {
        timer_clock_hz *= 2UL;
    }

    timebase_timer_clock_hz = timer_clock_hz;
    timebase_timer_prescaler = htim2.Init.Prescaler;

    /*
     * Debugger CPU'yu durdurduğunda SysTick de durur. TIM2/TIM5 çalışmaya
     * devam ederse micros() bir anda ileri sıçrar ve filtreler gereksiz reset
     * olur. Bu bitler yalnızca debug-halt sırasında etkilidir; normal uçuş
     * çalışmasını değiştirmez.
     */
#if defined(__HAL_DBGMCU_FREEZE_TIM2)
    __HAL_DBGMCU_FREEZE_TIM2();
#endif
#if defined(__HAL_DBGMCU_FREEZE_TIM5)
    __HAL_DBGMCU_FREEZE_TIM5();
#endif

    __HAL_TIM_SET_COUNTER(&htim2, 0UL);

    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
    {
        timebase_start_error_count++;
        return;
    }

    timebase_micros_debug = 0UL;
    timebase_millis_debug = HAL_GetTick();

    timebase_initialized = 1U;
}

/* -------------------------------------------------------------------------- */

uint32_t millis(void)
{
    uint32_t now_ms = HAL_GetTick();

    timebase_millis_debug = now_ms;

    return now_ms;
}

/* -------------------------------------------------------------------------- */

uint32_t micros(void)
{
    uint32_t now_us;

    if (timebase_initialized == 0U)
    {
        return 0UL;
    }

    /*
     * TIM2:
     * APB1 timer clock = 84 MHz
     * Prescaler = 83
     * Counter frequency = 1 MHz
     *
     * 1 count = 1 us
     */
    now_us = __HAL_TIM_GET_COUNTER(&htim2);

    timebase_micros_debug = now_us;

    return now_us;
}

/* -------------------------------------------------------------------------- */

uint8_t Timebase_HasElapsedMs(
    uint32_t now,
    uint32_t previous,
    uint32_t interval_ms
)
{
    return ((uint32_t)(now - previous) >= interval_ms);
}

/* -------------------------------------------------------------------------- */

uint8_t Timebase_HasElapsedUs(
    uint32_t now,
    uint32_t previous,
    uint32_t interval_us
)
{
    return ((uint32_t)(now - previous) >= interval_us);
}
