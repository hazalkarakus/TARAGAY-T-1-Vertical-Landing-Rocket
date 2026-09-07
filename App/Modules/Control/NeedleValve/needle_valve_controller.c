#include "Modules/Control/NeedleValve/needle_valve_controller.h"

#include "Services/NeedleValveHardware/needle_valve_hw.h"
#include "Services/PreflightTrigger/preflight_trigger.h"

#include "main.h"


/* -------------------------------------------------------------------------- */
/* V8.19L measured calibration / tuning                                      */
/* -------------------------------------------------------------------------- */

#define NV_CONTROL_PERIOD_MS               1UL

#define NV_ADC_PER_TURN                    195U
#define NV_MAX_TURNS                       3.0f
#define NV_MAX_TRAVEL_ADC                  (3U * NV_ADC_PER_TURN)
#define NV_SAFE_ZERO_MIN_ADC              NEEDLE_VALVE_SAFE_ZERO_MIN_ADC
#define NV_MIN_TARGET_ADC  (NV_SAFE_ZERO_MIN_ADC - NV_MAX_TRAVEL_ADC)
#define NV_MAX_TARGET_ADC                 1023U

/* R16 final limits commanded main-needle travel to 3.0 mechanical turns.
 * The previously validated physical target rates are unchanged:
 * open 3.85 turn/s, close 3.30 turn/s. */
#define NV_RISE_STEP_PER_TICK         0.0009625f  /* +0.9625 /s @ 1000 Hz */
#define NV_FALL_STEP_PER_TICK         0.0008250f  /* -0.8250 /s @ 1000 Hz */

#define NV_POSITION_TOLERANCE_ADC           15
#define NV_POSITION_RELEASE_ADC             18
#define NV_POSITION_STABLE_MS              250UL

#define NV_OPEN_FAR_ZONE_ADC               150
#define NV_OPEN_MID_ZONE_ADC                65
#define NV_OPEN_PULSE_ZONE_ADC              25

#define NV_CLOSE_FAR_ZONE_ADC              120
#define NV_CLOSE_MID_ZONE_ADC               60
#define NV_CLOSE_PULSE_ZONE_ADC              25

#define NV_OPEN_PWM_FAR                     120U
#define NV_OPEN_PWM_MID                      85U
#define NV_OPEN_PWM_NEAR                     65U

#define NV_CLOSE_PWM_FAR                    110U
#define NV_CLOSE_PWM_MID                     80U
#define NV_CLOSE_PWM_NEAR                    65U

/* Effective 5 ms scheduler quantization of the proven Arduino pulse timing. */
#define NV_OPEN_PULSE_ON_TICKS               10U
#define NV_OPEN_PULSE_OFF_TICKS              35U
#define NV_CLOSE_PULSE_ON_TICKS              15U
#define NV_CLOSE_PULSE_OFF_TICKS             45U

/* 1 kHz inner-loop PD law in the Arduino-compatible 0..1023 domain. */
#define NV_PD_KP_PWM_PER_ADC                0.55f
#define NV_PD_KD_PWM_PER_ADC                4.00f
#define NV_PD_MIN_PWM                         65U
#define NV_ADC_EMA_ALPHA                    0.12f

/* Stop before the feedback can travel materially beyond the configured
 * three-turn endpoint. */
#define NV_TRAVEL_GUARD_ADC                   20U

#define NV_STALL_TIMEOUT_MS                  800UL
#define NV_STALL_MOVEMENT_ADC                  2

/* Consecutive near-zero readings are treated as an open/disconnected wiper. */
#define NV_ADC_INVALID_MAX_RAW                  3U
#define NV_ADC_INVALID_CONFIRM_TICKS            3U

/* Five 1 kHz ticks (5 ms) of zero output before reversing direction. */
#define NV_REVERSAL_DEADTIME_TICKS             5U

/* Automatic closing reference. ADC must increase while closing. */
#define NV_HOME_CLOSED_THRESHOLD_ADC          1015U
#define NV_HOME_CLOSE_PWM                       65U
#define NV_HOME_STABLE_MS                      100UL
#define NV_HOME_TIMEOUT_MS                   15000UL
#define NV_HOME_NO_PROGRESS_MS                1000UL
#define NV_HOME_PROGRESS_ADC                     2U
#define NV_HOME_WRONG_DIRECTION_ADC              8U

/* -------------------------------------------------------------------------- */
/* Live Expressions                                                           */
/* -------------------------------------------------------------------------- */

volatile uint32_t needle_valve_v8_magic = 0x4E56394BUL; /* "NV9K" */
volatile float needle_valve_requested_cmd = 0.0f;
volatile float needle_valve_limited_cmd = 0.0f;
volatile float needle_valve_position_turns = 0.0f;
volatile float needle_valve_max_turns = NV_MAX_TURNS;
volatile uint16_t needle_valve_raw_adc = 0U;
volatile uint16_t needle_valve_zero_adc = 0U;
volatile uint16_t needle_valve_target_adc = 0U;
volatile uint16_t needle_valve_max_open_adc = 0U;
volatile uint16_t needle_valve_adc_per_turn = NV_ADC_PER_TURN;
volatile uint16_t needle_valve_max_travel_adc = NV_MAX_TRAVEL_ADC;
volatile int16_t needle_valve_error_adc = 0;
volatile uint8_t needle_valve_rpwm = 0U;
volatile uint8_t needle_valve_lpwm = 0U;
volatile uint8_t needle_valve_enabled = 0U;
volatile uint8_t needle_valve_zero_valid = 0U;
volatile uint8_t needle_valve_lock = 0U;
volatile uint8_t needle_valve_homing_active = 0U;
volatile uint8_t needle_valve_homing_complete = 0U;
volatile uint8_t needle_valve_fault = 0U;
volatile uint32_t needle_valve_stall_ms = 0UL;
volatile uint32_t needle_valve_homing_elapsed_ms = 0UL;
volatile uint16_t needle_valve_homing_start_adc = 0U;
volatile uint32_t needle_valve_control_tick_count = 0UL;
volatile uint32_t needle_valve_adc_invalid_count = 0UL;

/* -------------------------------------------------------------------------- */
/* Private state                                                              */
/* -------------------------------------------------------------------------- */

static NeedleValveStatus_t status;

static uint16_t raw_history[4];
static uint8_t raw_history_index = 0U;
static uint8_t raw_history_count = 0U;
static float raw_ema = 0.0f;
static uint8_t raw_ema_valid = 0U;
static int32_t previous_position_error = 0;

static uint8_t pulse_direction = 0U; /* 0=none, 1=open, 2=close */
static uint8_t pulse_phase_tick = 0U;

static int8_t last_drive_direction = 0; /* +1=open, -1=close, 0=off */
static uint8_t reversal_hold_ticks = 0U;

static uint16_t stall_anchor_raw = 0U;
static uint32_t stable_ms = 0UL;
static uint8_t reached_reported = 0U;
static uint8_t adc_invalid_ticks = 0U;
static uint32_t home_stable_ms = 0UL;
static uint32_t home_no_progress_ms = 0UL;
static uint16_t home_progress_anchor_raw = 0U;
static uint16_t home_start_raw = 0U;

static void NV_ClearStatus(NeedleValveStatus_t *s)
{
    uint8_t *bytes = (uint8_t *)s;

    for (uint32_t i = 0UL; i < (uint32_t)sizeof(*s); i++)
    {
        bytes[i] = 0U;
    }
}

static void NV_ClearRawHistory(void)
{
    for (uint8_t i = 0U; i < 4U; i++)
    {
        raw_history[i] = 0U;
    }
}

/* -------------------------------------------------------------------------- */

static float NV_Clamp01(float value)
{
    if (value < 0.0f)
    {
        return 0.0f;
    }

    if (value > 1.0f)
    {
        return 1.0f;
    }

    return value;
}

static uint16_t NV_ReadFilteredRaw(void)
{
    uint16_t raw = NeedleValveHW_ReadPotADC10();
    uint16_t filtered_input = raw;

    raw_history[raw_history_index] = raw;
    raw_history_index = (uint8_t)((raw_history_index + 1U) % 3U);
    if (raw_history_count < 3U)
    {
        raw_history_count++;
    }

    if (raw_history_count >= 3U)
    {
        uint16_t a = raw_history[0];
        uint16_t b = raw_history[1];
        uint16_t c = raw_history[2];
        uint16_t t;

        if (a > b) { t = a; a = b; b = t; }
        if (b > c) { t = b; b = c; c = t; }
        if (a > b) { t = a; a = b; b = t; }
        filtered_input = b;
    }

    if (raw_ema_valid == 0U)
    {
        raw_ema = (float)filtered_input;
        raw_ema_valid = 1U;
    }
    else
    {
        raw_ema += NV_ADC_EMA_ALPHA * ((float)filtered_input - raw_ema);
    }

    return (uint16_t)(raw_ema + 0.5f);
}

static uint8_t NV_SelectPwmLimit(int32_t error, int32_t abs_error)
{
    if (error > 0)
    {
        if (abs_error > NV_OPEN_FAR_ZONE_ADC) return NV_OPEN_PWM_FAR;
        if (abs_error > NV_OPEN_MID_ZONE_ADC) return NV_OPEN_PWM_MID;
        return NV_OPEN_PWM_NEAR;
    }

    if (abs_error > NV_CLOSE_FAR_ZONE_ADC) return NV_CLOSE_PWM_FAR;
    if (abs_error > NV_CLOSE_MID_ZONE_ADC) return NV_CLOSE_PWM_MID;
    return NV_CLOSE_PWM_NEAR;
}

static uint8_t NV_ComputeDampedPwm(int32_t error)
{
    int32_t abs_error = (error < 0) ? -error : error;
    int32_t derivative = error - previous_position_error;
    float signed_effort;
    float effort;
    uint8_t pwm_limit;

    previous_position_error = error;

    /* Signed derivative is essential: while the error is shrinking quickly,
     * the D term opposes P and commands braking instead of more throttle. */
    signed_effort = ((float)error * NV_PD_KP_PWM_PER_ADC) +
                    ((float)derivative * NV_PD_KD_PWM_PER_ADC);

    if (((error > 0) && (signed_effort <= 0.0f)) ||
        ((error < 0) && (signed_effort >= 0.0f)))
    {
        return 0U;
    }

    effort = (signed_effort < 0.0f) ? -signed_effort : signed_effort;
    pwm_limit = NV_SelectPwmLimit(error, abs_error);

    if (effort < (float)NV_PD_MIN_PWM) effort = (float)NV_PD_MIN_PWM;
    if (effort > (float)pwm_limit) effort = (float)pwm_limit;
    return (uint8_t)(effort + 0.5f);
}

static void NV_StopMotor(void)
{
    NeedleValveHW_Stop();
    status.rpwm = 0U;
    status.lpwm = 0U;
    last_drive_direction = 0;
}

static void NV_SetFault(NeedleValveFault_t fault)
{
    if (status.homing_active != 0U)
    {
        status.homing_complete = 0U;
    }

    status.homing_active = 0U;
    status.fault = fault;
    status.enabled = 0U;
    status.requested_cmd = 0.0f;

    NV_StopMotor();
    NeedleValveHW_SetEnabled(0U);
}

static uint16_t NV_CommandToTarget(float command)
{
    float travel = command * (float)NV_MAX_TRAVEL_ADC;
    int32_t rounded_travel = (int32_t)(travel + 0.5f);
    int32_t target = (int32_t)status.zero_adc - rounded_travel;

    if (target < (int32_t)NV_MIN_TARGET_ADC)
    {
        target = (int32_t)NV_MIN_TARGET_ADC;
    }

    if (target > (int32_t)NV_MAX_TARGET_ADC)
    {
        target = (int32_t)NV_MAX_TARGET_ADC;
    }

    return (uint16_t)target;
}

static uint8_t NV_RequestDirection(int8_t direction, uint8_t pwm)
{
    if ((direction != 1) && (direction != -1))
    {
        NV_StopMotor();
        return 0U;
    }

    if ((last_drive_direction != 0) &&
        (last_drive_direction != direction))
    {
        NV_StopMotor();
        reversal_hold_ticks = NV_REVERSAL_DEADTIME_TICKS;
        return 0U;
    }

    if (reversal_hold_ticks > 0U)
    {
        NV_StopMotor();
        reversal_hold_ticks--;
        return 0U;
    }

    if (direction > 0)
    {
        NeedleValveHW_DriveOpen(pwm);
        status.lpwm = pwm;
        status.rpwm = 0U;
        last_drive_direction = 1;
    }
    else
    {
        NeedleValveHW_DriveClose(pwm);
        status.rpwm = pwm;
        status.lpwm = 0U;
        last_drive_direction = -1;
    }

    return 1U;
}

static void NV_ResetPulse(void)
{
    pulse_direction = 0U;
    pulse_phase_tick = 0U;
}

static uint8_t NV_RunPulse(uint8_t direction, uint8_t pwm)
{
    uint8_t on_ticks;
    uint8_t off_ticks;
    uint8_t cycle_ticks;
    uint8_t output_active;

    if (direction == 1U)
    {
        on_ticks = NV_OPEN_PULSE_ON_TICKS;
        off_ticks = NV_OPEN_PULSE_OFF_TICKS;
    }
    else
    {
        on_ticks = NV_CLOSE_PULSE_ON_TICKS;
        off_ticks = NV_CLOSE_PULSE_OFF_TICKS;
    }

    if (pulse_direction != direction)
    {
        pulse_direction = direction;
        pulse_phase_tick = 0U;
    }

    cycle_ticks = (uint8_t)(on_ticks + off_ticks);
    output_active = (pulse_phase_tick < on_ticks) ? 1U : 0U;

    pulse_phase_tick++;

    if (pulse_phase_tick >= cycle_ticks)
    {
        pulse_phase_tick = 0U;
    }

    if (output_active != 0U)
    {
        return NV_RequestDirection((direction == 1U) ? 1 : -1, pwm);
    }

    NV_StopMotor();
    return 0U;
}

static void NV_UpdateStall(uint8_t motor_powered)
{
    int32_t movement =
        (int32_t)status.raw_adc - (int32_t)stall_anchor_raw;

    if (movement < 0)
    {
        movement = -movement;
    }

    if (movement >= NV_STALL_MOVEMENT_ADC)
    {
        stall_anchor_raw = status.raw_adc;
        status.stall_ms = 0UL;
        return;
    }

    /* Pulse-OFF time intentionally neither accumulates nor resets stall. */
    if (motor_powered != 0U)
    {
        status.stall_ms += NV_CONTROL_PERIOD_MS;

        if (status.stall_ms >= NV_STALL_TIMEOUT_MS)
        {
            NV_SetFault(NEEDLE_VALVE_FAULT_STALL);
        }
    }
}

static void NV_PublishLive(void)
{
    float turns = 0.0f;

    if ((status.zero_valid != 0U) &&
        (status.raw_adc <= status.zero_adc))
    {
        turns =
            (float)((uint16_t)(status.zero_adc - status.raw_adc)) /
            (float)NV_ADC_PER_TURN;

        if (turns > NV_MAX_TURNS)
        {
            turns = NV_MAX_TURNS;
        }
    }

    needle_valve_requested_cmd = status.requested_cmd;
    needle_valve_limited_cmd = status.limited_cmd;
    needle_valve_position_turns = turns;
    needle_valve_max_turns = NV_MAX_TURNS;
    needle_valve_raw_adc = status.raw_adc;
    needle_valve_zero_adc = status.zero_adc;
    needle_valve_target_adc = status.target_adc;
    needle_valve_max_open_adc =
        ((status.zero_valid != 0U) &&
         (status.zero_adc >= NV_MAX_TRAVEL_ADC))
        ? (uint16_t)(status.zero_adc - NV_MAX_TRAVEL_ADC)
        : 0U;
    needle_valve_adc_per_turn = NV_ADC_PER_TURN;
    needle_valve_max_travel_adc = NV_MAX_TRAVEL_ADC;
    needle_valve_error_adc = status.error_adc;
    needle_valve_rpwm = status.rpwm;
    needle_valve_lpwm = status.lpwm;
    needle_valve_enabled = status.enabled;
    needle_valve_zero_valid = status.zero_valid;
    needle_valve_lock = status.position_locked;
    needle_valve_homing_active = status.homing_active;
    needle_valve_homing_complete = status.homing_complete;
    needle_valve_fault = (uint8_t)status.fault;
    needle_valve_stall_ms = status.stall_ms;
    needle_valve_homing_elapsed_ms = status.homing_elapsed_ms;
    needle_valve_homing_start_adc = home_start_raw;
    needle_valve_control_tick_count = status.control_tick_count;
}


static void NV_ConfigControlTimer1kHz(void)
{
    uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();
    uint32_t timer_hz = pclk1_hz;
    uint32_t prescaler;

    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
    {
        timer_hz *= 2UL;
    }

    __HAL_RCC_TIM7_CLK_ENABLE();

    /* 1 MHz timer tick, 1000 counts -> 1000 Hz update IRQ. */
    prescaler = (timer_hz / 1000000UL) - 1UL;

    TIM7->CR1 = 0U;
    TIM7->PSC = prescaler;
    TIM7->ARR = 999UL;
    TIM7->CNT = 0UL;
    TIM7->SR = 0U;
    TIM7->DIER = TIM_DIER_UIE;
    TIM7->EGR = TIM_EGR_UG;

#if defined(__HAL_DBGMCU_FREEZE_TIM7)
    __HAL_DBGMCU_FREEZE_TIM7();
#endif

    /* Above sensor DMA / TIM5 logging, but control ISR remains very short. */
    HAL_NVIC_SetPriority(TIM7_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

    TIM7->CR1 = TIM_CR1_CEN;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void NeedleValveController_Init(void)
{
    uint32_t sum = 0UL;

    NV_ClearStatus(&status);
    NV_ClearRawHistory();

    raw_history_index = 0U;
    raw_history_count = 0U;
    raw_ema = 0.0f;
    raw_ema_valid = 0U;
    previous_position_error = 0;
    pulse_direction = 0U;
    pulse_phase_tick = 0U;
    last_drive_direction = 0;
    reversal_hold_ticks = 0U;
    stable_ms = 0UL;
    reached_reported = 0U;
    adc_invalid_ticks = 0U;
    home_stable_ms = 0UL;
    home_no_progress_ms = 0UL;
    home_progress_anchor_raw = 0U;
    home_start_raw = 0U;
    needle_valve_adc_invalid_count = 0UL;

    NeedleValveHW_Init();

    /* HARD SAFE BOOT: driver enables stay LOW. V55 may only capture the
     * already-closed ZERO while PE9 is attached; ENABLE is flight-gated. */
    NeedleValveHW_SetEnabled(0U);

    /* Fresh synchronous ADC samples. No automatic zero capture. */
    for (uint8_t i = 0U; i < 16U; i++)
    {
        sum += NeedleValveHW_ReadPotADC10();
    }

    status.raw_adc = (uint16_t)(sum / 16UL);
    status.zero_adc = 0U;
    status.target_adc = status.raw_adc;
    status.error_adc = 0;
    status.requested_cmd = 0.0f;
    status.limited_cmd = 0.0f;
    status.zero_valid = 0U;
    status.enabled = 0U;
    status.position_locked = 0U;
    status.homing_active = 0U;
    status.homing_complete = 0U;
    status.fault = NEEDLE_VALVE_FAULT_NONE;
    status.stall_ms = 0UL;
    status.homing_elapsed_ms = 0UL;
    stall_anchor_raw = status.raw_adc;

    if (status.raw_adc <= NV_ADC_INVALID_MAX_RAW)
    {
        status.fault = NEEDLE_VALVE_FAULT_ADC_INVALID;
        needle_valve_adc_invalid_count++;
    }

    NV_PublishLive();

    /* 1 kHz sampling/control starts, but actuation remains disabled. */
    NV_ConfigControlTimer1kHz();
}

uint8_t NeedleValveController_SetCommand(float command_0_to_1)
{
    /* NaN check without libm dependency. */
    if (command_0_to_1 != command_0_to_1)
    {
        return 0U;
    }

    if ((command_0_to_1 < 0.0f) || (command_0_to_1 > 1.0f))
    {
        return 0U;
    }

    if ((status.zero_valid == 0U) ||
        (status.enabled == 0U) ||
        (status.homing_active != 0U) ||
        (status.fault != NEEDLE_VALVE_FAULT_NONE))
    {
        return 0U;
    }

#if ((APP_ACTUATOR_FLIGHT_INTERLOCK_ENABLED != 0U) && \
     (APP_NEEDLE_FOUR_TURN_TEST_MODE == 0U))
    if ((PreflightTrigger_IsFlightActive() == 0U) ||
        (PreflightTrigger_HasFault() != 0U))
    {
        return 0U;
    }
#endif

    status.requested_cmd = command_0_to_1;
    status.command_update_count++;
    NV_PublishLive();
    return 1U;
}

void NeedleValveController_Stop(void)
{
    if (status.homing_active != 0U)
    {
        status.homing_complete = 0U;
    }

    status.homing_active = 0U;
    status.requested_cmd = 0.0f;
    status.enabled = 0U;
    stable_ms = 0UL;
    reached_reported = 0U;
    NV_ResetPulse();
    NV_StopMotor();
    NeedleValveHW_SetEnabled(0U);
    NV_PublishLive();
}

uint8_t NeedleValveController_Enable(void)
{
    if ((status.zero_valid == 0U) ||
        (status.fault != NEEDLE_VALVE_FAULT_NONE))
    {
        return 0U;
    }

#if ((APP_ACTUATOR_FLIGHT_INTERLOCK_ENABLED != 0U) && \
     (APP_NEEDLE_FOUR_TURN_TEST_MODE == 0U))
    if ((PreflightTrigger_IsFlightActive() == 0U) ||
        (PreflightTrigger_HasFault() != 0U))
    {
        return 0U;
    }
#endif

    status.enabled = 1U;
    stall_anchor_raw = status.raw_adc;
    status.stall_ms = 0UL;
    previous_position_error = 0;
    NeedleValveHW_SetEnabled(1U);
    NV_PublishLive();
    return 1U;
}

uint8_t NeedleValveController_CaptureZero(void)
{
    uint16_t raw;
    uint32_t primask;

    NV_StopMotor();
    NeedleValveHW_SetEnabled(0U);

    /* TIM7 already samples and filters PC1 at 1 kHz. Re-use that stable
     * reading instead of starting synchronous ADC conversions concurrently
     * with the ISR. The preflight supervisor requires 250 ms stability before
     * this function is called. */
    primask = __get_PRIMASK();
    __disable_irq();
    raw = status.raw_adc;

    if (raw <= NV_ADC_INVALID_MAX_RAW)
    {
        status.raw_adc = raw;
        status.zero_valid = 0U;
        status.enabled = 0U;
        status.fault = NEEDLE_VALVE_FAULT_ADC_INVALID;
        needle_valve_adc_invalid_count++;
        NV_PublishLive();
        if (primask == 0U) __enable_irq();
        return 0U;
    }

    if (raw < NV_SAFE_ZERO_MIN_ADC)
    {
        status.raw_adc = raw;
        status.zero_valid = 0U;
        status.enabled = 0U;
        status.fault = NEEDLE_VALVE_FAULT_ZERO_UNSAFE;
        NV_PublishLive();
        if (primask == 0U) __enable_irq();
        return 0U;
    }

    NV_ClearRawHistory();
    raw_history_index = 0U;
    raw_history_count = 0U;

    status.raw_adc = raw;
    status.zero_adc = raw;
    status.target_adc = raw;
    status.requested_cmd = 0.0f;
    status.limited_cmd = 0.0f;
    status.error_adc = 0;
    status.zero_valid = 1U;
    status.homing_active = 0U;
    status.homing_complete = 1U;
    status.homing_elapsed_ms = 0UL;

    /* ZERO capture does not energize BTS7960. ENABLE remains a separate,
     * flight-authorized operation. */
    status.enabled = 0U;
    status.position_locked = 1U;
    status.fault = NEEDLE_VALVE_FAULT_NONE;
    status.stall_ms = 0UL;
    stall_anchor_raw = raw;
    stable_ms = 0UL;
    reached_reported = 0U;
    adc_invalid_ticks = 0U;
    NV_ResetPulse();

    NeedleValveHW_SetEnabled(0U);
    NV_PublishLive();
    if (primask == 0U) __enable_irq();
    return 1U;
}

uint8_t NeedleValveController_StartAutoHome(void)
{
#if ((APP_ACTUATOR_FLIGHT_INTERLOCK_ENABLED != 0U) && \
     (APP_NEEDLE_FOUR_TURN_TEST_MODE == 0U))
    /* Flight profile: preflight homing is forbidden because it moves the
     * actuator while the separation connector is still installed. */
    return 0U;
#else
    if ((status.fault != NEEDLE_VALVE_FAULT_NONE) ||
        (status.raw_adc <= NV_ADC_INVALID_MAX_RAW) ||
        (status.homing_active != 0U))
    {
        return 0U;
    }

    NV_StopMotor();
    NV_ResetPulse();

    status.requested_cmd = 0.0f;
    status.limited_cmd = 0.0f;
    status.zero_adc = 0U;
    status.zero_valid = 0U;
    status.target_adc = NV_HOME_CLOSED_THRESHOLD_ADC;
    status.error_adc =
        (int16_t)((int32_t)status.raw_adc -
                  (int32_t)NV_HOME_CLOSED_THRESHOLD_ADC);
    status.position_locked = 0U;
    status.homing_active = 1U;
    status.homing_complete = 0U;
    status.homing_elapsed_ms = 0UL;
    status.enabled = 1U;
    status.stall_ms = 0UL;

    home_start_raw = status.raw_adc;
    home_progress_anchor_raw = status.raw_adc;
    home_stable_ms = 0UL;
    home_no_progress_ms = 0UL;
    stall_anchor_raw = status.raw_adc;
    adc_invalid_ticks = 0U;

    NeedleValveHW_SetEnabled(1U);
    NV_PublishLive();
    return 1U;
#endif
}

void NeedleValveController_ClearFault(void)
{
    /* Clearing a fault never enables the motor and never fabricates ZERO. */
    if (status.raw_adc > NV_ADC_INVALID_MAX_RAW)
    {
        status.fault = NEEDLE_VALVE_FAULT_NONE;
    }

    status.enabled = 0U;
    NeedleValveHW_SetEnabled(0U);
    NV_StopMotor();
    NV_PublishLive();
}

void NeedleValveController_ControlTickISR(void)
{
    int32_t error;
    int32_t abs_error;
    uint8_t motor_powered = 0U;

    status.control_tick_count++;
    status.raw_adc = NV_ReadFilteredRaw();

#if ((APP_ACTUATOR_FLIGHT_INTERLOCK_ENABLED != 0U) && \
     (APP_NEEDLE_FOUR_TURN_TEST_MODE == 0U))
    if ((status.enabled != 0U) &&
        ((PreflightTrigger_IsFlightActive() == 0U) ||
         (PreflightTrigger_HasFault() != 0U)))
    {
        status.requested_cmd = 0.0f;
        status.enabled = 0U;
        status.position_locked = 0U;
        NV_StopMotor();
        NeedleValveHW_SetEnabled(0U);
        NV_PublishLive();
        return;
    }
#endif

    if (status.raw_adc <= NV_ADC_INVALID_MAX_RAW)
    {
        if (adc_invalid_ticks < 255U)
        {
            adc_invalid_ticks++;
        }

        needle_valve_adc_invalid_count++;

        if (adc_invalid_ticks >= NV_ADC_INVALID_CONFIRM_TICKS)
        {
            status.zero_valid = 0U;
            NV_SetFault(NEEDLE_VALVE_FAULT_ADC_INVALID);
            NV_PublishLive();
            return;
        }
    }
    else
    {
        adc_invalid_ticks = 0U;
    }

    if (status.homing_active != 0U)
    {
        status.homing_elapsed_ms += NV_CONTROL_PERIOD_MS;
        status.target_adc = NV_HOME_CLOSED_THRESHOLD_ADC;
        status.error_adc =
            (int16_t)((int32_t)status.raw_adc -
                      (int32_t)NV_HOME_CLOSED_THRESHOLD_ADC);

        if (status.homing_elapsed_ms >= NV_HOME_TIMEOUT_MS)
        {
            NV_SetFault(NEEDLE_VALVE_FAULT_STALL);
            NV_PublishLive();
            return;
        }

        if (status.raw_adc >= NV_HOME_CLOSED_THRESHOLD_ADC)
        {
            NV_StopMotor();
            status.stall_ms = 0UL;
            home_no_progress_ms = 0UL;
            home_stable_ms += NV_CONTROL_PERIOD_MS;

            if (home_stable_ms >= NV_HOME_STABLE_MS)
            {
                status.zero_adc = status.raw_adc;
                status.target_adc = status.raw_adc;
                status.error_adc = 0;
                status.requested_cmd = 0.0f;
                status.limited_cmd = 0.0f;
                status.zero_valid = 1U;
                status.position_locked = 1U;
                status.homing_active = 0U;
                status.homing_complete = 1U;
                status.enabled = 0U;
                status.stall_ms = 0UL;
                stable_ms = 0UL;
                reached_reported = 0U;
                NeedleValveHW_SetEnabled(0U);
            }

            NV_PublishLive();
            return;
        }

        home_stable_ms = 0UL;

        if (((uint32_t)status.raw_adc + NV_HOME_WRONG_DIRECTION_ADC) <
            (uint32_t)home_start_raw)
        {
            NV_SetFault(NEEDLE_VALVE_FAULT_HOME_DIRECTION);
            NV_PublishLive();
            return;
        }

        motor_powered = NV_RequestDirection(-1, NV_HOME_CLOSE_PWM);

        if (status.raw_adc >=
            (uint16_t)(home_progress_anchor_raw + NV_HOME_PROGRESS_ADC))
        {
            home_progress_anchor_raw = status.raw_adc;
            home_no_progress_ms = 0UL;
            status.stall_ms = 0UL;
        }
        else if (motor_powered != 0U)
        {
            home_no_progress_ms += NV_CONTROL_PERIOD_MS;
            status.stall_ms = home_no_progress_ms;

            if (home_no_progress_ms >= NV_HOME_NO_PROGRESS_MS)
            {
                NV_SetFault(NEEDLE_VALVE_FAULT_STALL);
            }
        }

        NV_PublishLive();
        return;
    }

    if ((status.enabled == 0U) ||
        (status.zero_valid == 0U) ||
        (status.fault != NEEDLE_VALVE_FAULT_NONE))
    {
        NV_StopMotor();
        NV_PublishLive();
        return;
    }

    /* V55: 4-turn local 1 kHz target ramp. PD, PWM limits,
     * reversal dead-time, stall detection and active braking are unchanged. */
    if (status.requested_cmd > status.limited_cmd)
    {
        float delta = status.requested_cmd - status.limited_cmd;
        status.limited_cmd +=
            (delta > NV_RISE_STEP_PER_TICK) ? NV_RISE_STEP_PER_TICK : delta;
    }
    else if (status.requested_cmd < status.limited_cmd)
    {
        float delta = status.limited_cmd - status.requested_cmd;
        status.limited_cmd -=
            (delta > NV_FALL_STEP_PER_TICK) ? NV_FALL_STEP_PER_TICK : delta;
    }

    status.limited_cmd = NV_Clamp01(status.limited_cmd);
    status.target_adc = NV_CommandToTarget(status.limited_cmd);

    if ((status.target_adc < NV_MIN_TARGET_ADC) ||
        (status.target_adc > NV_MAX_TARGET_ADC))
    {
        NV_SetFault(NEEDLE_VALVE_FAULT_TARGET_RANGE);
        NV_PublishLive();
        return;
    }


    if ((status.zero_valid != 0U) &&
        ((uint32_t)status.raw_adc + NV_TRAVEL_GUARD_ADC <
         ((uint32_t)status.zero_adc - NV_MAX_TRAVEL_ADC)))
    {
        NV_SetFault(NEEDLE_VALVE_FAULT_TARGET_RANGE);
        NV_PublishLive();
        return;
    }

    error = (int32_t)status.raw_adc - (int32_t)status.target_adc;
    status.error_adc = (int16_t)error;
    abs_error = (error < 0) ? -error : error;

    /* Slide requirement: active lock inside +/-15 ADC with hysteresis. */
    if (status.position_locked != 0U)
    {
        if (abs_error > NV_POSITION_RELEASE_ADC)
        {
            status.position_locked = 0U;
        }
    }
    else if (abs_error <= NV_POSITION_TOLERANCE_ADC)
    {
        status.position_locked = 1U;
    }

    if (status.position_locked != 0U)
    {
        /* Driver is still enabled here: both bridge inputs LOW short the
         * winding and suppress overshoot instead of coasting. */
        NeedleValveHW_Brake();
        status.rpwm = 0U;
        status.lpwm = 0U;
        last_drive_direction = 0;
        previous_position_error = error;
        NV_ResetPulse();
        status.stall_ms = 0UL;
        stall_anchor_raw = status.raw_adc;

        if (abs_error <= NV_POSITION_TOLERANCE_ADC)
        {
            stable_ms += NV_CONTROL_PERIOD_MS;

            if ((stable_ms >= NV_POSITION_STABLE_MS) &&
                (reached_reported == 0U))
            {
                status.target_reached_count++;
                reached_reported = 1U;
            }
        }
        else
        {
            stable_ms = 0UL;
            reached_reported = 0U;
        }

        NV_PublishLive();
        return;
    }

    stable_ms = 0UL;
    reached_reported = 0U;

    if (error > 0)
    {
        uint8_t pwm = NV_ComputeDampedPwm(error);

        if (pwm == 0U)
        {
            NV_ResetPulse();
            NeedleValveHW_Brake();
            status.rpwm = 0U;
            status.lpwm = 0U;
            last_drive_direction = 0;
        }
        else if (abs_error <= NV_OPEN_PULSE_ZONE_ADC)
        {
            motor_powered = NV_RunPulse(1U, pwm);
        }
        else
        {
            NV_ResetPulse();
            motor_powered = NV_RequestDirection(1, pwm);
        }
    }
    else if (error < 0)
    {
        uint8_t pwm = NV_ComputeDampedPwm(error);

        if (pwm == 0U)
        {
            NV_ResetPulse();
            NeedleValveHW_Brake();
            status.rpwm = 0U;
            status.lpwm = 0U;
            last_drive_direction = 0;
        }
        else if (abs_error <= NV_CLOSE_PULSE_ZONE_ADC)
        {
            motor_powered = NV_RunPulse(2U, pwm);
        }
        else
        {
            NV_ResetPulse();
            motor_powered = NV_RequestDirection(-1, pwm);
        }
    }
    else
    {
        NV_StopMotor();
        NV_ResetPulse();
    }

    NV_UpdateStall(motor_powered);
    NV_PublishLive();
}

NeedleValveStatus_t NeedleValveController_GetStatus(void)
{
    NeedleValveStatus_t copy;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    copy = status;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return copy;
}
