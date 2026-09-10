#include "Services/SolenoidOutput/solenoid_output.h"

#include "Common/app_config.h"
#include "Services/PreflightTrigger/preflight_trigger.h"
#include "Services/SystemMonitor/system_monitor.h"
#include "main.h"

/* All four channels use the previously verified active-LOW relay logic. */
#define SOLENOID_ACTIVE_STATE       GPIO_PIN_RESET
#define SOLENOID_SAFE_STATE         GPIO_PIN_SET

/* Final verified relay harness supplied by the mechanical/electrical team. */
#define SOLENOID_IN1_X_POS_PORT     GPIOB
#define SOLENOID_IN1_X_POS_PIN      GPIO_PIN_15
#define SOLENOID_IN2_X_NEG_PORT     GPIOE
#define SOLENOID_IN2_X_NEG_PIN      GPIO_PIN_15
#define SOLENOID_IN3_Y_POS_PORT     GPIOE
#define SOLENOID_IN3_Y_POS_PIN      GPIO_PIN_11
#define SOLENOID_IN4_Y_NEG_PORT     GPIOE
#define SOLENOID_IN4_Y_NEG_PIN      GPIO_PIN_7

#define SOLENOID_ALL_MASK ((uint8_t)( \
    SOLENOID_VALVE_X_POS_ERROR  | \
    SOLENOID_VALVE_X_NEG_ERROR  | \
    SOLENOID_VALVE_Y_POS_ERROR | \
    SOLENOID_VALVE_Y_NEG_ERROR))

static SolenoidOutputStatus_t solenoid_status;
/* R8R35: start time of the currently applied nonzero physical mask. */
static uint32_t r8r35_pulse_start_ms = 0UL;


static uint8_t SolenoidOutput_SanitizeMask(uint8_t requested_mask);
#if (ATT_CTRL_DRY_RUN == 0U)
static void SolenoidOutput_WritePhysical(uint8_t valve_mask);
#endif

#define RELAY_BENCH_STEP_MS 500UL
#define RELAY_BENCH_DEBOUNCE_MS 50UL
#define RELAY_BENCH_REARM_MS 250UL
volatile uint8_t relay_bench_test_active=0U;
volatile uint8_t relay_bench_test_channel=0U;
volatile uint8_t relay_bench_test_last_mask=0U;
volatile uint32_t relay_bench_test_start_count=0UL;
volatile uint32_t relay_bench_test_complete_count=0UL;
volatile uint32_t relay_bench_test_abort_count=0UL;
#if (APP_RELAY_SEQUENCE_TEST_MODE != 0U)
static uint32_t relay_step_start_ms, relay_candidate_since_ms, relay_last_accept_ms;
static uint8_t relay_raw_last, relay_debounced;
static uint8_t BenchMask(uint8_t c){ switch(c){case 1U:return SOLENOID_VALVE_X_POS_ERROR;case 2U:return SOLENOID_VALVE_X_NEG_ERROR;case 3U:return SOLENOID_VALVE_Y_POS_ERROR;case 4U:return SOLENOID_VALVE_Y_NEG_ERROR;default:return SOLENOID_VALVE_NONE;} }
static uint8_t BenchButton(void){return (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET)?1U:0U;}
static void BenchButtonInit(void){GPIO_InitTypeDef g={0};__HAL_RCC_GPIOA_CLK_ENABLE();g.Pin=GPIO_PIN_0;g.Mode=GPIO_MODE_INPUT;g.Pull=GPIO_PULLDOWN;g.Speed=GPIO_SPEED_FREQ_LOW;HAL_GPIO_Init(GPIOA,&g);}
static void BenchApply(uint8_t mask){uint8_t m=SolenoidOutput_SanitizeMask(mask);solenoid_status.requested_mask=m;solenoid_status.requested_open=(m!=0U);solenoid_status.update_count++;solenoid_status.safety_inhibited=0U;
#if (ATT_CTRL_DRY_RUN == 0U)
if(m!=solenoid_status.applied_mask){SolenoidOutput_WritePhysical(m);solenoid_status.applied_mask=m;}
#else
solenoid_status.applied_mask=0U;
#endif
solenoid_status.applied_open=(solenoid_status.applied_mask!=0U);relay_bench_test_last_mask=solenoid_status.applied_mask;}
#endif

static uint8_t SolenoidOutput_SanitizeMask(uint8_t requested_mask)
{
    uint8_t sanitized = (uint8_t)(requested_mask & SOLENOID_ALL_MASK);
    uint8_t interlock_fault = 0U;

    /* Opposing valves on one axis are never permitted simultaneously. */
    if ((sanitized & (uint8_t)(SOLENOID_VALVE_X_POS_ERROR |
                               SOLENOID_VALVE_X_NEG_ERROR)) ==
        (uint8_t)(SOLENOID_VALVE_X_POS_ERROR |
                  SOLENOID_VALVE_X_NEG_ERROR))
    {
        sanitized &= (uint8_t)~(uint8_t)(SOLENOID_VALVE_X_POS_ERROR |
                                         SOLENOID_VALVE_X_NEG_ERROR);
        interlock_fault = 1U;
    }

    if ((sanitized & (uint8_t)(SOLENOID_VALVE_Y_POS_ERROR |
                               SOLENOID_VALVE_Y_NEG_ERROR)) ==
        (uint8_t)(SOLENOID_VALVE_Y_POS_ERROR |
                  SOLENOID_VALVE_Y_NEG_ERROR))
    {
        sanitized &= (uint8_t)~(uint8_t)(SOLENOID_VALVE_Y_POS_ERROR |
                                         SOLENOID_VALVE_Y_NEG_ERROR);
        interlock_fault = 1U;
    }

    solenoid_status.interlock_fault = interlock_fault;
    return sanitized;
}

#if (ATT_CTRL_DRY_RUN == 0U)
static void SolenoidOutput_WritePin(
    GPIO_TypeDef *port,
    uint16_t pin,
    uint8_t active
)
{
    HAL_GPIO_WritePin(
        port,
        pin,
        (active != 0U) ? SOLENOID_ACTIVE_STATE : SOLENOID_SAFE_STATE
    );
}

static void SolenoidOutput_WritePhysical(uint8_t valve_mask)
{
    SolenoidOutput_WritePin(
        SOLENOID_IN1_X_POS_PORT,
        SOLENOID_IN1_X_POS_PIN,
        (uint8_t)(valve_mask & SOLENOID_VALVE_X_POS_ERROR)
    );
    SolenoidOutput_WritePin(
        SOLENOID_IN2_X_NEG_PORT,
        SOLENOID_IN2_X_NEG_PIN,
        (uint8_t)(valve_mask & SOLENOID_VALVE_X_NEG_ERROR)
    );
    SolenoidOutput_WritePin(
        SOLENOID_IN3_Y_POS_PORT,
        SOLENOID_IN3_Y_POS_PIN,
        (uint8_t)(valve_mask & SOLENOID_VALVE_Y_POS_ERROR)
    );
    SolenoidOutput_WritePin(
        SOLENOID_IN4_Y_NEG_PORT,
        SOLENOID_IN4_Y_NEG_PIN,
        (uint8_t)(valve_mask & SOLENOID_VALVE_Y_NEG_ERROR)
    );
}

static void SolenoidOutput_ConfigureHardware(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Preload safe HIGH before changing pin mode to avoid startup pulses. */
    HAL_GPIO_WritePin(
        GPIOE,
        GPIO_PIN_7 | GPIO_PIN_11 | GPIO_PIN_15,
        SOLENOID_SAFE_STATE
    );
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, SOLENOID_SAFE_STATE);

    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

    gpio_init.Pin = GPIO_PIN_7 | GPIO_PIN_11 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &gpio_init);

    gpio_init.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    SolenoidOutput_WritePhysical(SOLENOID_VALVE_NONE);
}
#endif

void SolenoidOutput_Init(void)
{
    solenoid_status.requested_mask = SOLENOID_VALVE_NONE;
    solenoid_status.applied_mask = SOLENOID_VALVE_NONE;
    solenoid_status.requested_open = 0U;
    solenoid_status.applied_open = 0U;
    solenoid_status.dry_run = (ATT_CTRL_DRY_RUN != 0U) ? 1U : 0U;
    solenoid_status.interlock_fault = 0U;
    solenoid_status.flight_authorized = 0U;
    solenoid_status.safety_inhibited = 0U;
    solenoid_status.update_count = 0UL;
    solenoid_status.safety_inhibit_count = 0UL;
    solenoid_status.pulse_complete_count = 0UL;
    solenoid_status.last_pulse_ms = 0U;
    solenoid_status.max_pulse_ms = 0U;
    r8r35_pulse_start_ms = 0UL;

#if (ATT_CTRL_DRY_RUN == 0U)
    SolenoidOutput_ConfigureHardware();
#endif
}

void SolenoidOutput_SetMask(uint8_t valve_mask)
{
#if (((APP_P112R12_INERT_OUTPUT_ISOLATION_MODE != 0U) && \
      (APP_P112R12R8R33_REAL_FLIGHT_LOGIC_PHYSICAL_RCS_REV == 0U)) || \
     (APP_P112R10R3_GNC_MOTOR_BENCH_MODE != 0U))
    valve_mask = SOLENOID_VALVE_NONE;
#endif
    uint8_t sanitized_mask = SolenoidOutput_SanitizeMask(valve_mask);
    uint8_t applied_candidate = sanitized_mask;

#if (APP_ACTUATOR_FLIGHT_INTERLOCK_ENABLED != 0U)
    solenoid_status.flight_authorized =
        ((PreflightTrigger_IsFlightActive() != 0U) &&
         (PreflightTrigger_HasFault() == 0U)) ? 1U : 0U;

    if ((sanitized_mask != SOLENOID_VALVE_NONE) &&
        ((solenoid_status.flight_authorized == 0U) ||
         (SystemMonitor_IsActuatorFaultActive() != 0U)))
    {
        applied_candidate = SOLENOID_VALVE_NONE;
        solenoid_status.safety_inhibited = 1U;
        solenoid_status.safety_inhibit_count++;
    }
    else
    {
        solenoid_status.safety_inhibited = 0U;
    }
#else
    solenoid_status.flight_authorized = 1U;
    solenoid_status.safety_inhibited = 0U;
#endif

    solenoid_status.requested_mask = sanitized_mask;
    solenoid_status.requested_open = (sanitized_mask != 0U) ? 1U : 0U;
    solenoid_status.update_count++;

#if (ATT_CTRL_DRY_RUN == 0U)
    if (applied_candidate != solenoid_status.applied_mask)
    {
        const uint8_t previous_mask = solenoid_status.applied_mask;
        const uint32_t now_ms = HAL_GetTick();

        SolenoidOutput_WritePhysical(applied_candidate);
        solenoid_status.applied_mask = applied_candidate;

        if ((previous_mask == SOLENOID_VALVE_NONE) &&
            (applied_candidate != SOLENOID_VALVE_NONE))
        {
            r8r35_pulse_start_ms = now_ms;
        }
        else if ((previous_mask != SOLENOID_VALVE_NONE) &&
                 (applied_candidate == SOLENOID_VALVE_NONE))
        {
            uint32_t pulse_ms = now_ms - r8r35_pulse_start_ms;
            if (pulse_ms > 65535UL) pulse_ms = 65535UL;
            solenoid_status.last_pulse_ms = (uint16_t)pulse_ms;
            if (solenoid_status.last_pulse_ms > solenoid_status.max_pulse_ms)
                solenoid_status.max_pulse_ms = solenoid_status.last_pulse_ms;
            solenoid_status.pulse_complete_count++;
            r8r35_pulse_start_ms = 0UL;
        }
        else if ((previous_mask != SOLENOID_VALVE_NONE) &&
                 (applied_candidate != SOLENOID_VALVE_NONE))
        {
            /* Break-before-make should normally insert zero. If a direct
             * nonzero mask change occurs, treat it as a new pulse epoch. */
            r8r35_pulse_start_ms = now_ms;
        }
    }
#else
    solenoid_status.applied_mask = SOLENOID_VALVE_NONE;
#endif

    solenoid_status.applied_open =
        (solenoid_status.applied_mask != 0U) ? 1U : 0U;
}


void SolenoidOutput_SetAxisBenchMask(uint8_t valve_mask, uint8_t bench_authorized)
{
#if ((APP_P112R12R8R25_RCS_RELAY_AXIS_BENCH_REV != 0U) || \
     (APP_P112R12R8R26_IMU_ROCKET_FRAME_CAL_REV != 0U) || \
     (APP_P112R12R8R27_THREE_POSE_FRAME_CAL_REV != 0U) || \
     (APP_P112R12R8R28_FIVE_POSE_FRAME_CAL_REV != 0U) || \
     (APP_P112R12R8R30_FIXED_IMU_ROCKET_FRAME_REV != 0U))
    uint8_t sanitized_mask = SolenoidOutput_SanitizeMask(valve_mask);
    uint8_t applied_candidate = SOLENOID_VALVE_NONE;

    /* Dedicated pressureless relay bench: do not require PE9 separation, but
     * require the calling flight-logic adapter to prove preflight READY, real
     * ESKF input validity, no STOP latch, and no system/actuator fault. */
    if ((bench_authorized != 0U) &&
        (SystemMonitor_IsActuatorFaultActive() == 0U) &&
        (PreflightTrigger_HasFault() == 0U))
    {
        applied_candidate = sanitized_mask;
        solenoid_status.safety_inhibited = 0U;
        solenoid_status.flight_authorized = 1U; /* bench-authorized telemetry */
    }
    else
    {
        applied_candidate = SOLENOID_VALVE_NONE;
        solenoid_status.safety_inhibited = (sanitized_mask != SOLENOID_VALVE_NONE) ? 1U : 0U;
        if (solenoid_status.safety_inhibited != 0U)
        {
            solenoid_status.safety_inhibit_count++;
        }
        solenoid_status.flight_authorized = 0U;
    }

    solenoid_status.requested_mask = sanitized_mask;
    solenoid_status.requested_open = (sanitized_mask != SOLENOID_VALVE_NONE) ? 1U : 0U;
    solenoid_status.update_count++;

#if (ATT_CTRL_DRY_RUN == 0U)
    if (applied_candidate != solenoid_status.applied_mask)
    {
        SolenoidOutput_WritePhysical(applied_candidate);
        solenoid_status.applied_mask = applied_candidate;
    }
#else
    solenoid_status.applied_mask = SOLENOID_VALVE_NONE;
#endif
    solenoid_status.applied_open = (solenoid_status.applied_mask != SOLENOID_VALVE_NONE) ? 1U : 0U;
#else
    (void)valve_mask;
    (void)bench_authorized;
    SolenoidOutput_ForceSafe();
#endif
}

void SolenoidOutput_SetDemand(uint8_t open_demand)
{
    SolenoidOutput_SetMask(
        (open_demand != 0U)
            ? SOLENOID_VALVE_X_POS_ERROR
            : SOLENOID_VALVE_NONE
    );
}

void SolenoidOutput_ForceSafe(void)
{
    SolenoidOutput_SetMask(SOLENOID_VALVE_NONE);
}

uint8_t SolenoidOutput_SetGroundVentMask(
    uint8_t valve_mask,
    uint8_t post_separation_ground_authorized
)
{
    const uint8_t roll_pair = (uint8_t)(
        SOLENOID_VALVE_X_POS_ERROR | SOLENOID_VALVE_X_NEG_ERROR);
    const uint8_t pitch_pair = (uint8_t)(
        SOLENOID_VALVE_Y_POS_ERROR | SOLENOID_VALVE_Y_NEG_ERROR);
    uint8_t sanitized_mask = (uint8_t)(valve_mask & SOLENOID_ALL_MASK);

#if (APP_P112R10R3_GNC_MOTOR_BENCH_MODE != 0U)
    sanitized_mask = SOLENOID_VALVE_NONE;
#endif

    /* P71: opposing valves are permitted ONLY in this dedicated vent path,
     * and ONLY as one complete axis pair (X+/X- or Y+/Y-).  The normal RCS
     * sanitizer and flight control keep the opposing-valve interlock intact.
     * All four simultaneously, partial multi-valve masks and cross-axis pairs
     * remain rejected. */
    if ((sanitized_mask != SOLENOID_VALVE_NONE) &&
        (sanitized_mask != roll_pair) &&
        (sanitized_mask != pitch_pair))
    {
        solenoid_status.interlock_fault = 1U;
        SolenoidOutput_ForceSafe();
        return 0U;
    }

    /* P70/P71 dedicated manual depressurization path. A nonzero output may
     * bypass the normal flight-only gate before PE9, or after PE9 only when
     * app.c has independently qualified a grounded/stationary state. Zero is
     * always permitted so any lost qualification closes immediately. */
    if ((PreflightTrigger_HasFault() != 0U) ||
        (SystemMonitor_IsActuatorFaultActive() != 0U) ||
        ((sanitized_mask != SOLENOID_VALVE_NONE) &&
         (PreflightTrigger_IsFlightActive() != 0U) &&
         (post_separation_ground_authorized == 0U)))
    {
        SolenoidOutput_ForceSafe();
        return 0U;
    }

    solenoid_status.interlock_fault = 0U;
    solenoid_status.requested_mask = sanitized_mask;
    solenoid_status.requested_open =
        (sanitized_mask != SOLENOID_VALVE_NONE) ? 1U : 0U;
    solenoid_status.flight_authorized = 0U;
    solenoid_status.safety_inhibited = 0U;
    solenoid_status.update_count++;

#if (ATT_CTRL_DRY_RUN == 0U)
    if (sanitized_mask != solenoid_status.applied_mask)
    {
        SolenoidOutput_WritePhysical(sanitized_mask);
        solenoid_status.applied_mask = sanitized_mask;
    }
#else
    solenoid_status.applied_mask = SOLENOID_VALVE_NONE;
#endif

    solenoid_status.applied_open =
        (solenoid_status.applied_mask != SOLENOID_VALVE_NONE) ? 1U : 0U;

    return 1U;
}

void SolenoidOutput_BenchTestInit(void)
{
    relay_bench_test_active=0U; relay_bench_test_channel=0U; relay_bench_test_last_mask=0U;
#if (APP_RELAY_SEQUENCE_TEST_MODE != 0U)
    BenchButtonInit(); relay_raw_last=BenchButton(); relay_debounced=relay_raw_last; relay_candidate_since_ms=HAL_GetTick(); relay_last_accept_ms=HAL_GetTick(); relay_step_start_ms=HAL_GetTick(); BenchApply(0U);
#else
    SolenoidOutput_ForceSafe();
#endif
}
void SolenoidOutput_BenchTestUpdate(void)
{
#if (APP_RELAY_SEQUENCE_TEST_MODE != 0U)
    uint32_t now=HAL_GetTick(); uint8_t raw=BenchButton(), rising=0U;
    if((PreflightTrigger_IsFlightActive()!=0U)||(PreflightTrigger_HasFault()!=0U)){if(relay_bench_test_active)relay_bench_test_abort_count++;relay_bench_test_active=0U;relay_bench_test_channel=0U;BenchApply(0U);return;}
    if(raw!=relay_raw_last){relay_raw_last=raw;relay_candidate_since_ms=now;}
    else if((raw!=relay_debounced)&&((uint32_t)(now-relay_candidate_since_ms)>=RELAY_BENCH_DEBOUNCE_MS)){relay_debounced=raw;if(relay_debounced&&((uint32_t)(now-relay_last_accept_ms)>=RELAY_BENCH_REARM_MS)){rising=1U;relay_last_accept_ms=now;}}
    if(rising){if(relay_bench_test_active){relay_bench_test_active=0U;relay_bench_test_channel=0U;relay_bench_test_abort_count++;BenchApply(0U);return;}relay_bench_test_active=1U;relay_bench_test_channel=1U;relay_bench_test_start_count++;relay_step_start_ms=now;BenchApply(BenchMask(1U));}
    if(relay_bench_test_active&&((uint32_t)(now-relay_step_start_ms)>=RELAY_BENCH_STEP_MS)){relay_step_start_ms=now;relay_bench_test_channel++;if(relay_bench_test_channel>4U){relay_bench_test_active=0U;relay_bench_test_channel=0U;relay_bench_test_complete_count++;BenchApply(0U);}else BenchApply(BenchMask(relay_bench_test_channel));}
#endif
}

SolenoidOutputStatus_t SolenoidOutput_GetStatus(void)
{
    return solenoid_status;
}

uint8_t SolenoidOutput_GetRequestedMask(void)
{
    return solenoid_status.requested_mask;
}

uint8_t SolenoidOutput_GetAppliedMask(void)
{
    return solenoid_status.applied_mask;
}
