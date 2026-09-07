#include "Services/ServoOutput/servo_output.h"

#include "Common/app_config.h"
#include "main.h"

/* PB6 is TIM4_CH1 on AF2 for STM32F407. */
#define VENT_SERVO_GPIO_PORT          GPIOB
#define VENT_SERVO_GPIO_PIN           GPIO_PIN_6
#define VENT_SERVO_GPIO_AF            GPIO_AF2_TIM4
#define VENT_SERVO_TIM_CHANNEL        TIM_CHANNEL_1

static TIM_HandleTypeDef vent_servo_htim4;
static uint8_t vent_servo_last_target_open = 0xFFU;

static uint8_t vent_servo_link_loss_grace_active_internal = 0U;
static uint32_t vent_servo_link_loss_start_ms = 0UL;

volatile uint8_t vent_servo_init_ok = 0U;
volatile uint8_t vent_servo_remote_command = 0U;
volatile uint8_t vent_servo_link_active = 0U;
volatile uint8_t vent_servo_target_open = 0U;
volatile uint16_t vent_servo_pulse_us = 0U;
volatile uint32_t vent_servo_update_count = 0UL;
volatile uint32_t vent_servo_command_change_count = 0UL;

/*
 * transition_reason:
 * 0 none/init
 * 1 command OPEN
 * 2 command CLOSED
 * 3 confirmed RF link-loss CLOSED
 * 4 ForceClosed()
 */
volatile uint8_t vent_servo_transition_reason = 0U;
volatile uint32_t vent_servo_link_loss_grace_count = 0UL;
volatile uint32_t vent_servo_link_glitch_suppressed_count = 0UL;
volatile uint8_t vent_servo_link_loss_grace_active = 0U;
volatile uint32_t vent_servo_link_loss_grace_age_ms = 0UL;

static uint16_t ServoOutput_ClampPulse(uint16_t pulse_us)
{
    if (pulse_us < APP_VENT_SERVO_MIN_PULSE_US)
    {
        return APP_VENT_SERVO_MIN_PULSE_US;
    }

    if (pulse_us > APP_VENT_SERVO_MAX_PULSE_US)
    {
        return APP_VENT_SERVO_MAX_PULSE_US;
    }

    return pulse_us;
}

static uint32_t ServoOutput_GetTim4ClockHz(void)
{
    RCC_ClkInitTypeDef clock_config = {0};
    uint32_t flash_latency = 0UL;
    uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();

    HAL_RCC_GetClockConfig(&clock_config, &flash_latency);

    /* APB prescaler != 1 ise timer clock APB clock'un 2 katidir. */
    if (clock_config.APB1CLKDivider != RCC_HCLK_DIV1)
    {
        return pclk1_hz * 2UL;
    }

    return pclk1_hz;
}

static void ServoOutput_WritePulse(uint16_t pulse_us)
{
    uint16_t safe_pulse = ServoOutput_ClampPulse(pulse_us);

    __HAL_TIM_SET_COMPARE(
        &vent_servo_htim4,
        VENT_SERVO_TIM_CHANNEL,
        (uint32_t)safe_pulse
    );

    vent_servo_pulse_us = safe_pulse;
}

void ServoOutput_Init(void)
{
#if (APP_VENT_SERVO_ENABLED != 0U)
    GPIO_InitTypeDef gpio_init = {0};
    TIM_OC_InitTypeDef pwm_config = {0};
    uint32_t timer_clock_hz;
    uint32_t prescaler;

    vent_servo_init_ok = 0U;
    vent_servo_remote_command = 0U;
    vent_servo_link_active = 0U;
    vent_servo_target_open = 0U;
    vent_servo_update_count = 0UL;
    vent_servo_command_change_count = 0UL;
    vent_servo_last_target_open = 0xFFU;

    vent_servo_link_loss_grace_active_internal = 0U;
    vent_servo_link_loss_start_ms = 0UL;

    vent_servo_transition_reason = 0U;
    vent_servo_link_loss_grace_count = 0UL;
    vent_servo_link_glitch_suppressed_count = 0UL;
    vent_servo_link_loss_grace_active = 0U;
    vent_servo_link_loss_grace_age_ms = 0UL;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    gpio_init.Pin = VENT_SERVO_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_init.Alternate = VENT_SERVO_GPIO_AF;
    HAL_GPIO_Init(VENT_SERVO_GPIO_PORT, &gpio_init);

    timer_clock_hz = ServoOutput_GetTim4ClockHz();
    if (timer_clock_hz < 1000000UL)
    {
        return;
    }

    /* 1 us timer tick. */
    prescaler = (timer_clock_hz / 1000000UL) - 1UL;

    vent_servo_htim4.Instance = TIM4;
    vent_servo_htim4.Init.Prescaler = prescaler;
    vent_servo_htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    vent_servo_htim4.Init.Period = APP_VENT_SERVO_PWM_PERIOD_US - 1UL;
    vent_servo_htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    vent_servo_htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&vent_servo_htim4) != HAL_OK)
    {
        return;
    }

    pwm_config.OCMode = TIM_OCMODE_PWM1;
    pwm_config.Pulse = ServoOutput_ClampPulse(APP_VENT_SERVO_CLOSED_PULSE_US);
    pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm_config.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(
            &vent_servo_htim4,
            &pwm_config,
            VENT_SERVO_TIM_CHANNEL) != HAL_OK)
    {
        return;
    }

    ServoOutput_WritePulse(APP_VENT_SERVO_CLOSED_PULSE_US);

    if (HAL_TIM_PWM_Start(&vent_servo_htim4, VENT_SERVO_TIM_CHANNEL) != HAL_OK)
    {
        return;
    }

    vent_servo_last_target_open = 0U;
    vent_servo_init_ok = 1U;
#else
    vent_servo_init_ok = 0U;
#endif
}

void ServoOutput_Update(uint8_t remote_command, uint8_t link_active)
{
#if (APP_VENT_SERVO_ENABLED != 0U)
    uint8_t requested_open;
    uint16_t requested_pulse;

#if (APP_P112R12_INERT_OUTPUT_ISOLATION_MODE != 0U)
    remote_command = 0U;
    link_active = 0U;
#endif

    vent_servo_remote_command = remote_command;
    vent_servo_link_active = link_active;
    vent_servo_update_count++;

    if (vent_servo_init_ok == 0U)
    {
        return;
    }

    /*
     * nRF protocol:
     *   0 = CLOSED
     *   1 = OPEN
     *
     * RemoteControl already has a 500 ms hard link timeout. A short additional
     * servo grace suppresses the observed CLOSE->OPEN chatter when the radio
     * link momentarily times out and immediately reacquires.
     */
#if (APP_P112R12_INERT_OUTPUT_ISOLATION_MODE != 0U)
    requested_open = 0U;
    vent_servo_link_loss_grace_active_internal = 0U;
    vent_servo_link_loss_grace_active = 0U;
    vent_servo_link_loss_grace_age_ms = 0UL;
#else
    if (link_active != 0U)
    {
        if (vent_servo_link_loss_grace_active_internal != 0U)
        {
            vent_servo_link_glitch_suppressed_count++;
        }

        vent_servo_link_loss_grace_active_internal = 0U;
        vent_servo_link_loss_grace_active = 0U;
        vent_servo_link_loss_grace_age_ms = 0UL;

        requested_open = (remote_command == 1U) ? 1U : 0U;
    }
    else
    {
        if ((vent_servo_last_target_open == 1U) &&
            (vent_servo_link_loss_grace_active_internal == 0U))
        {
            vent_servo_link_loss_grace_active_internal = 1U;
            vent_servo_link_loss_start_ms = HAL_GetTick();
            vent_servo_link_loss_grace_count++;
        }

        if (vent_servo_link_loss_grace_active_internal != 0U)
        {
            uint32_t grace_age =
                (uint32_t)(HAL_GetTick() - vent_servo_link_loss_start_ms);

            vent_servo_link_loss_grace_active = 1U;
            vent_servo_link_loss_grace_age_ms = grace_age;

            if (grace_age < APP_VENT_SERVO_LINK_LOSS_GRACE_MS)
            {
                /* Hold OPEN during the short anti-chatter grace. */
                requested_open = 1U;
            }
            else
            {
                requested_open = 0U;
                vent_servo_link_loss_grace_active_internal = 0U;
                vent_servo_link_loss_grace_active = 0U;
            }
        }
        else
        {
            requested_open = 0U;
        }
    }
#endif

    requested_pulse =
        (requested_open != 0U)
            ? APP_VENT_SERVO_OPEN_PULSE_US
            : APP_VENT_SERVO_CLOSED_PULSE_US;

    if (requested_open != vent_servo_last_target_open)
    {
        if (requested_open != 0U)
        {
            vent_servo_transition_reason = 1U;
        }
        else if (link_active != 0U)
        {
            vent_servo_transition_reason = 2U;
        }
        else
        {
            vent_servo_transition_reason = 3U;
        }

        ServoOutput_WritePulse(requested_pulse);
        vent_servo_last_target_open = requested_open;
        vent_servo_command_change_count++;
    }

    vent_servo_target_open = requested_open;
#else
    (void)remote_command;
    (void)link_active;
#endif
}

void ServoOutput_ForceClosed(void)
{
#if (APP_VENT_SERVO_ENABLED != 0U)
    if (vent_servo_init_ok != 0U)
    {
        ServoOutput_WritePulse(APP_VENT_SERVO_CLOSED_PULSE_US);
        vent_servo_last_target_open = 0U;
        vent_servo_target_open = 0U;
        vent_servo_transition_reason = 4U;
        vent_servo_link_loss_grace_active_internal = 0U;
        vent_servo_link_loss_grace_active = 0U;
        vent_servo_link_loss_grace_age_ms = 0UL;
    }
#endif
}
