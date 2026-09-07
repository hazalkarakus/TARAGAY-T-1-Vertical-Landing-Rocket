#include "Services/NeedleValveHardware/needle_valve_hw.h"

#include "Common/app_config.h"
#include "main.h"

/*
 * STM32F407VG needle-valve hardware mapping
 * -----------------------------------------
 * PC1  : potentiometer wiper, ADC1_IN11 + ADC2_IN11
 * PB0  : TIM3_CH3 -> BTS7960 LPWM -> OPEN
 * PB1  : TIM3_CH4 -> BTS7960 RPWM -> CLOSE
 * PC4  : BTS7960 L_EN
 * PC5  : BTS7960 R_EN
 *
 * P82 diagnostic ADC strategy
 * ---------------------------
 * - ADC1 long sample (480 cycles) keeps the production/P79 path.
 * - ADC2 independently reads the exact same PC1 / channel 11 signal.
 * - ADC1 can also read PC1 with the shortest 3-cycle sample time.
 * - ADC1 periodically reads internal VREFINT (channel 17) so VDDA/reference
 *   motion can be separated from an external PC1/potentiometer problem.
 *
 * All P82 functions are single-shot/polled and are only used in the inert
 * bench diagnostic profile. Motor hardware remains compile-time locked OFF.
 */

#define NV_POT_PORT                   GPIOC
#define NV_POT_PIN                    GPIO_PIN_1

#define NV_LPWM_PORT                  GPIOB
#define NV_LPWM_PIN                   GPIO_PIN_0
#define NV_LPWM_AF                    GPIO_AF2_TIM3
#define NV_LPWM_CHANNEL               TIM_CHANNEL_3

#define NV_RPWM_PORT                  GPIOB
#define NV_RPWM_PIN                   GPIO_PIN_1
#define NV_RPWM_AF                    GPIO_AF2_TIM3
#define NV_RPWM_CHANNEL               TIM_CHANNEL_4

#define NV_LEN_PORT                   GPIOC
#define NV_LEN_PIN                    GPIO_PIN_4
#define NV_REN_PORT                   GPIOC
#define NV_REN_PIN                    GPIO_PIN_5

#define NV_PWM_TIMER_HZ               1000000UL
#define NV_PWM_CARRIER_HZ             1000UL
#define NV_PWM_PERIOD_COUNTS          (NV_PWM_TIMER_HZ / NV_PWM_CARRIER_HZ)

#define NV_ADC_GUARD                  250000UL
#define NV_ADC_PC1_CHANNEL            11UL
#define NV_ADC_VREF_CHANNEL           17UL
#define NV_VREFINT_TYP_MV             1210UL

static uint8_t hw_initialized = 0U;
static uint16_t adc_last_10bit = 0U;
static uint16_t adc2_last_10bit = 0U;
static uint16_t adc1_short_last_10bit = 0U;
static uint16_t vref_last_raw12 = 0U;
static volatile uint8_t p88_bench_jog_arm = 0U;

volatile uint16_t needle_valve_hw_adc_raw12 = 0U;
volatile uint16_t needle_valve_hw_adc_mv = 0U;
volatile uint32_t needle_valve_hw_adc_ok_count = 0UL;
volatile uint32_t needle_valve_hw_adc_timeout_count = 0UL;

volatile uint16_t needle_valve_hw_adc2_raw12 = 0U;
volatile uint32_t needle_valve_hw_adc2_ok_count = 0UL;
volatile uint32_t needle_valve_hw_adc2_timeout_count = 0UL;

volatile uint16_t needle_valve_hw_adc1_short_raw12 = 0U;
volatile uint32_t needle_valve_hw_adc1_short_ok_count = 0UL;
volatile uint32_t needle_valve_hw_adc1_short_timeout_count = 0UL;

volatile uint16_t needle_valve_hw_vref_raw12 = 0U;
volatile uint16_t needle_valve_hw_vdda_mv_est = 0U;
volatile uint32_t needle_valve_hw_vref_ok_count = 0UL;
volatile uint32_t needle_valve_hw_vref_timeout_count = 0UL;

static uint32_t NeedleValveHW_GetTim3ClockHz(void)
{
    RCC_ClkInitTypeDef clock_config = {0};
    uint32_t flash_latency = 0UL;
    uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();

    HAL_RCC_GetClockConfig(&clock_config, &flash_latency);

    if (clock_config.APB1CLKDivider != RCC_HCLK_DIV1)
    {
        return pclk1_hz * 2UL;
    }

    return pclk1_hz;
}

#if ((APP_NEEDLE_ADC_DIAGNOSTIC_MODE == 0U) || (APP_NEEDLE_P91_ADC_DELTA_400_MODE != 0U))
static uint32_t NeedleValveHW_PWMToCCR(uint8_t pwm_0_255)
{
    uint32_t counts =
        ((uint32_t)pwm_0_255 * NV_PWM_PERIOD_COUNTS + 127UL) / 255UL;

    if (counts >= NV_PWM_PERIOD_COUNTS)
    {
        counts = NV_PWM_PERIOD_COUNTS - 1UL;
    }

    return counts;
}
#endif

static uint8_t NeedleValveHW_ConvertADC1(uint32_t channel, uint16_t *raw12)
{
    uint32_t guard = NV_ADC_GUARD;

    ADC1->SQR3 = channel;
    ADC1->SR = 0UL;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    while (((ADC1->SR & ADC_SR_EOC) == 0UL) && (guard > 0UL))
    {
        guard--;
    }

    if ((ADC1->SR & ADC_SR_EOC) == 0UL)
    {
        return 0U;
    }

    *raw12 = (uint16_t)(ADC1->DR & 0x0FFFUL);
    return 1U;
}

static uint8_t NeedleValveHW_ConvertADC2(uint16_t *raw12)
{
    uint32_t guard = NV_ADC_GUARD;

    ADC2->SR = 0UL;
    ADC2->CR2 |= ADC_CR2_SWSTART;

    while (((ADC2->SR & ADC_SR_EOC) == 0UL) && (guard > 0UL))
    {
        guard--;
    }

    if ((ADC2->SR & ADC_SR_EOC) == 0UL)
    {
        return 0U;
    }

    *raw12 = (uint16_t)(ADC2->DR & 0x0FFFUL);
    return 1U;
}

static uint16_t NeedleValveHW_Median3U16(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}

static uint16_t NeedleValveHW_Raw12ToADC10(uint16_t raw12)
{
    return (uint16_t)(((uint32_t)raw12 * 1023UL + 2047UL) / 4095UL);
}

static void NeedleValveHW_ConfigADC(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();

    gpio.Pin = NV_POT_PIN;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(NV_POT_PORT, &gpio);

    /* PCLK2 84 MHz / 4 = 21 MHz ADC clock. Keep ADCs independent. */
    ADC->CCR &= ~(ADC_CCR_ADCPRE | ADC_CCR_MULTI);
    ADC->CCR |= ADC_CCR_ADCPRE_0;

    /* Enable the internal temperature/VREF block. P82 only consumes VREFINT. */
    ADC->CCR |= ADC_CCR_TSVREFE;

    ADC1->CR1 = 0UL;
    ADC1->CR2 = 0UL;
    ADC1->SMPR1 = 0UL;
    ADC1->SMPR2 = 0UL;
    ADC1->SMPR1 |= ADC_SMPR1_SMP11; /* 480 cycles on PC1 */
    ADC1->SMPR1 |= ADC_SMPR1_SMP17; /* 480 cycles on VREFINT */
    ADC1->SQR1 = 0UL;
    ADC1->SQR2 = 0UL;
    ADC1->SQR3 = NV_ADC_PC1_CHANNEL;
    ADC1->SR = 0UL;
    ADC1->CR2 |= ADC_CR2_ADON;

    ADC2->CR1 = 0UL;
    ADC2->CR2 = 0UL;
    ADC2->SMPR1 = 0UL;
    ADC2->SMPR2 = 0UL;
    ADC2->SMPR1 |= ADC_SMPR1_SMP11; /* 480 cycles on same PC1 input */
    ADC2->SQR1 = 0UL;
    ADC2->SQR2 = 0UL;
    ADC2->SQR3 = NV_ADC_PC1_CHANNEL;
    ADC2->SR = 0UL;
    ADC2->CR2 |= ADC_CR2_ADON;

    /* ADC enable + VREFINT startup settling. */
    for (volatile uint32_t i = 0UL; i < 20000UL; ++i)
    {
        __NOP();
    }
}

static void NeedleValveHW_ConfigPWM(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t tim_clk_hz;
    uint32_t prescaler;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    gpio.Pin = NV_LPWM_PIN | NV_RPWM_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = NV_LPWM_AF;
    HAL_GPIO_Init(GPIOB, &gpio);

    tim_clk_hz = NeedleValveHW_GetTim3ClockHz();
    prescaler = (tim_clk_hz / NV_PWM_TIMER_HZ) - 1UL;

    TIM3->CR1 = 0UL;
    TIM3->PSC = prescaler;
    TIM3->ARR = NV_PWM_PERIOD_COUNTS - 1UL;
    TIM3->CCR3 = 0UL;
    TIM3->CCR4 = 0UL;

    TIM3->CCMR2 =
        TIM_CCMR2_OC3PE |
        TIM_CCMR2_OC3M_1 |
        TIM_CCMR2_OC3M_2 |
        TIM_CCMR2_OC4PE |
        TIM_CCMR2_OC4M_1 |
        TIM_CCMR2_OC4M_2;

    TIM3->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void NeedleValveHW_ConfigEnablePins(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(NV_LEN_PORT, NV_LEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(NV_REN_PORT, NV_REN_PIN, GPIO_PIN_RESET);

    gpio.Pin = NV_LEN_PIN | NV_REN_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
}

void NeedleValveHW_Init(void)
{
    hw_initialized = 0U;
    adc_last_10bit = 0U;
    adc2_last_10bit = 0U;
    adc1_short_last_10bit = 0U;
    vref_last_raw12 = 0U;
    p88_bench_jog_arm = 0U;

    needle_valve_hw_adc_raw12 = 0U;
    needle_valve_hw_adc_mv = 0U;
    needle_valve_hw_adc_ok_count = 0UL;
    needle_valve_hw_adc_timeout_count = 0UL;
    needle_valve_hw_adc2_raw12 = 0U;
    needle_valve_hw_adc2_ok_count = 0UL;
    needle_valve_hw_adc2_timeout_count = 0UL;
    needle_valve_hw_adc1_short_raw12 = 0U;
    needle_valve_hw_adc1_short_ok_count = 0UL;
    needle_valve_hw_adc1_short_timeout_count = 0UL;
    needle_valve_hw_vref_raw12 = 0U;
    needle_valve_hw_vdda_mv_est = 0U;
    needle_valve_hw_vref_ok_count = 0UL;
    needle_valve_hw_vref_timeout_count = 0UL;

    NeedleValveHW_ConfigEnablePins();
    NeedleValveHW_ConfigPWM();
    NeedleValveHW_ConfigADC();

    NeedleValveHW_Stop();
    hw_initialized = 1U;
}

uint16_t NeedleValveHW_ReadPotADC10(void)
{
    uint16_t raw12;

    if (hw_initialized == 0U)
    {
        return adc_last_10bit;
    }

    /* Production/P79-compatible path: ADC1, channel 11, 480 cycles. */
    ADC1->SMPR1 |= ADC_SMPR1_SMP11;
    if (NeedleValveHW_ConvertADC1(NV_ADC_PC1_CHANNEL, &raw12) == 0U)
    {
        needle_valve_hw_adc_timeout_count++;
        return adc_last_10bit;
    }

    needle_valve_hw_adc_raw12 = raw12;
    needle_valve_hw_adc_mv =
        (uint16_t)(((uint32_t)raw12 * 3300UL + 2047UL) / 4095UL);
    adc_last_10bit = NeedleValveHW_Raw12ToADC10(raw12);
    needle_valve_hw_adc_ok_count++;
    return adc_last_10bit;
}

uint16_t NeedleValveHW_ReadPotADC10Burst3(void)
{
    uint16_t a = NeedleValveHW_ReadPotADC10();
    uint16_t b = NeedleValveHW_ReadPotADC10();
    uint16_t c = NeedleValveHW_ReadPotADC10();
    return NeedleValveHW_Median3U16(a, b, c);
}

uint16_t NeedleValveHW_ReadPotADC2ADC10(void)
{
    uint16_t raw12;

    if (hw_initialized == 0U)
    {
        return adc2_last_10bit;
    }

    if (NeedleValveHW_ConvertADC2(&raw12) == 0U)
    {
        needle_valve_hw_adc2_timeout_count++;
        return adc2_last_10bit;
    }

    needle_valve_hw_adc2_raw12 = raw12;
    adc2_last_10bit = NeedleValveHW_Raw12ToADC10(raw12);
    needle_valve_hw_adc2_ok_count++;
    return adc2_last_10bit;
}

uint16_t NeedleValveHW_ReadPotADC2ADC10Burst3(void)
{
    uint16_t a = NeedleValveHW_ReadPotADC2ADC10();
    uint16_t b = NeedleValveHW_ReadPotADC2ADC10();
    uint16_t c = NeedleValveHW_ReadPotADC2ADC10();
    return NeedleValveHW_Median3U16(a, b, c);
}

uint16_t NeedleValveHW_ReadPotADC1ShortADC10(void)
{
    uint16_t raw12;
    uint32_t saved_smpr1;

    if (hw_initialized == 0U)
    {
        return adc1_short_last_10bit;
    }

    saved_smpr1 = ADC1->SMPR1;
    ADC1->SMPR1 &= ~ADC_SMPR1_SMP11; /* 000 = 3 ADC cycles */

    if (NeedleValveHW_ConvertADC1(NV_ADC_PC1_CHANNEL, &raw12) == 0U)
    {
        ADC1->SMPR1 = saved_smpr1;
        needle_valve_hw_adc1_short_timeout_count++;
        return adc1_short_last_10bit;
    }

    ADC1->SMPR1 = saved_smpr1;
    needle_valve_hw_adc1_short_raw12 = raw12;
    adc1_short_last_10bit = NeedleValveHW_Raw12ToADC10(raw12);
    needle_valve_hw_adc1_short_ok_count++;
    return adc1_short_last_10bit;
}

uint16_t NeedleValveHW_ReadVrefRaw12(void)
{
    uint16_t raw12;
    uint32_t saved_sqr3;
    uint32_t saved_smpr1;

    if (hw_initialized == 0U)
    {
        return vref_last_raw12;
    }

    saved_sqr3 = ADC1->SQR3;
    saved_smpr1 = ADC1->SMPR1;
    ADC1->SMPR1 |= ADC_SMPR1_SMP17;

    if (NeedleValveHW_ConvertADC1(NV_ADC_VREF_CHANNEL, &raw12) == 0U)
    {
        ADC1->SQR3 = saved_sqr3;
        ADC1->SMPR1 = saved_smpr1;
        needle_valve_hw_vref_timeout_count++;
        return vref_last_raw12;
    }

    ADC1->SQR3 = saved_sqr3;
    ADC1->SMPR1 = saved_smpr1;

    vref_last_raw12 = raw12;
    needle_valve_hw_vref_raw12 = raw12;
    if (raw12 != 0U)
    {
        /* Diagnostic estimate only. Absolute accuracy is not the purpose;
         * relative movement reveals VDDA/reference instability. */
        uint32_t vdda = (NV_VREFINT_TYP_MV * 4095UL + ((uint32_t)raw12 / 2UL)) /
                        (uint32_t)raw12;
        if (vdda > 65535UL) vdda = 65535UL;
        needle_valve_hw_vdda_mv_est = (uint16_t)vdda;
    }
    needle_valve_hw_vref_ok_count++;
    return vref_last_raw12;
}

void NeedleValveHW_SetBenchJogArm(uint8_t armed)
{
#if (APP_NEEDLE_P91_ADC_DELTA_400_MODE != 0U)
    p88_bench_jog_arm = (armed != 0U) ? 1U : 0U;
#else
    (void)armed;
    p88_bench_jog_arm = 0U;
#endif

    if (p88_bench_jog_arm == 0U)
    {
        NeedleValveHW_Stop();
        HAL_GPIO_WritePin(NV_LEN_PORT, NV_LEN_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(NV_REN_PORT, NV_REN_PIN, GPIO_PIN_RESET);
    }
}

void NeedleValveHW_SetEnabled(uint8_t enabled)
{
#if ((APP_NEEDLE_ADC_DIAGNOSTIC_MODE != 0U) && (APP_NEEDLE_P91_ADC_DELTA_400_MODE == 0U))
    enabled = 0U;
#endif
#if (APP_NEEDLE_P91_ADC_DELTA_400_MODE != 0U)
    if (p88_bench_jog_arm == 0U) enabled = 0U;
#endif

    GPIO_PinState state = (enabled != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    if (enabled == 0U)
    {
        NeedleValveHW_Stop();
    }

    HAL_GPIO_WritePin(NV_LEN_PORT, NV_LEN_PIN, state);
    HAL_GPIO_WritePin(NV_REN_PORT, NV_REN_PIN, state);
}

void NeedleValveHW_Stop(void)
{
    TIM3->CCR3 = 0UL;
    TIM3->CCR4 = 0UL;
}

void NeedleValveHW_Brake(void)
{
    TIM3->CCR3 = 0UL;
    TIM3->CCR4 = 0UL;
}

void NeedleValveHW_DriveOpen(uint8_t pwm_0_255)
{
#if ((APP_NEEDLE_ADC_DIAGNOSTIC_MODE != 0U) && (APP_NEEDLE_P91_ADC_DELTA_400_MODE == 0U))
    (void)pwm_0_255;
    NeedleValveHW_Stop();
    return;
#else
#if (APP_NEEDLE_P91_ADC_DELTA_400_MODE != 0U)
    if (p88_bench_jog_arm == 0U)
    {
        NeedleValveHW_Stop();
        return;
    }
#endif
    uint32_t ccr = NeedleValveHW_PWMToCCR(pwm_0_255);
    TIM3->CCR4 = 0UL;
    TIM3->CCR3 = ccr;
#endif
}

void NeedleValveHW_DriveClose(uint8_t pwm_0_255)
{
#if ((APP_NEEDLE_ADC_DIAGNOSTIC_MODE != 0U) && (APP_NEEDLE_P91_ADC_DELTA_400_MODE == 0U))
    (void)pwm_0_255;
    NeedleValveHW_Stop();
    return;
#else
#if (APP_NEEDLE_P91_ADC_DELTA_400_MODE != 0U)
    if (p88_bench_jog_arm == 0U)
    {
        NeedleValveHW_Stop();
        return;
    }
#endif
    uint32_t ccr = NeedleValveHW_PWMToCCR(pwm_0_255);
    TIM3->CCR3 = 0UL;
    TIM3->CCR4 = ccr;
#endif
}

uint8_t NeedleValveHW_IsInitialized(void)
{
    return hw_initialized;
}
