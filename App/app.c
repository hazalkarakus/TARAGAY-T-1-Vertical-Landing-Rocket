#include "app.h"
#include "Common/runtime_delay_guard.h"
#include "usart.h"

#include "Common/app_config.h"

#include "Platform/board.h"
#include "Platform/board_pins.h"
#include "Core/Scheduler/scheduler.h"
#include "Core/Tasks/app_tasks.h"

#include "Services/BusManager/bus_manager.h"
#include "Services/SolenoidOutput/solenoid_output.h"
#include "Services/ServoOutput/servo_output.h"
#include "Services/SDLogger/sd_logger.h"
#include "Services/Timebase/timebase.h"
#include "Services/UARTTelemetry/uart_telemetry.h"
#include "Services/SystemMonitor/system_monitor.h"
#include "Services/NRFTelemetry/nrf_telemetry.h"

#include "Modules/Control/NeedleValve/needle_valve_controller.h"
#include "Modules/Control/AttitudeControl/attitude_control.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"
#include "./Services/NeedleValveIntegrationTest/needle_valve_integration_test.h"
#include "./Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.h"
#include "./Services/NeedleValveAutoControl/needle_valve_auto_control.h"
#include "./Services/PreflightTrigger/preflight_trigger.h"
#include "Modules/Control/GNCActiveControl/gnc_active_control.h"
#include "Modules/RemoteControl/remote_control.h"
#include "Modules/Sensors/Lidar/Lidar.h"
#include "Modules/Sensors/IMU/imu.h"
#include "Modules/NRF24/nrf24.h"

/* V46 SDIO startup diagnostics from bsp_driver_sd.c */
extern volatile uint8_t sd_bsp_hal_init_status;
extern volatile uint8_t sd_bsp_wide_bus_status;
extern volatile uint32_t sd_bsp_hal_error_code;
extern volatile uint32_t sd_bsp_sdio_power;
extern volatile uint32_t sd_bsp_sdio_clkcr;
extern volatile uint32_t sd_bsp_sdio_sta;
extern volatile uint32_t sd_bsp_sdio_resp1;
extern volatile uint32_t sd_v47_init_attempt_count;
extern volatile uint32_t sd_v47_host_reset_count;
extern volatile uint8_t sd_v47_cmd_idle_level;
extern volatile uint8_t sd_v47_d0_idle_level;
extern volatile uint8_t sd_v47_d1_idle_level;
extern volatile uint8_t sd_v47_d2_idle_level;
extern volatile uint8_t sd_v47_d3_idle_level;
extern volatile uint32_t sd_v47_last_attempt_error;
extern volatile uint8_t sd_v47_recovered;

/*
 * V8.7 staged integration
 * -----------------------
 *
 * ACTIVE:
 * - IMU 1 kHz
 * - BMP585 service 200 Hz
 * - Garmin LIDAR-Lite v3 service 200 Hz
 * - Full-State ESKF RCS command 200 Hz
 * - Needle valve and RCS pulse timers on exact TIM7 1 kHz
 * - SD logger:
 *      TIM5 exact 200 Hz capture
 *      288-byte binary V13 frames
 *      RAM ring
 *      double writer buffers
 *      raw SDIO multi-sector DMA
 * - P112R5 robust-feedback autonomous adaptive needle actuator
 *
 * P59 keeps NRF24 reception enabled and adds an explicit remote-safety layer:
 * - Switch-1 / packet bit0 = maintained GROUND VENT request. The four RCS
 *   solenoids vent as balanced opposing pairs: X+|X-, then Y+|Y-, with
 *   break-before-make deadtime between pairs.
 * - Switch-2 / packet bit1 = one-way emergency STOP latch. STOP forces all RCS
 *   outputs safe and disables/stops the needle actuator until reset/power-cycle.
 *
 * The remote command never writes the generated landing Valve_Cmd directly.
 *
 * P112R5 captures the already-closed needle reference without moving the
 * motor while PE9 is connected. After the complete preflight gate and PE9
 * separation, the generated 200 Hz vertical command is converted to an ADC
 * target by the autonomous supervisor. Breakaway, speed, stop distance, coast
 * learning and bounded corrections execute in TIM7 without UART or PA0.
 *
 * SDLogger_Update() is serviced in the main loop, not in a fixed scheduler
 * slot. This keeps its async DMA state machine responsive without blocking
 * the 1 kHz / 200 Hz cooperative tasks.
 */

volatile uint32_t v87_sd_update_count = 0UL;
volatile uint32_t v87_sd_update_last_us = 0UL;
volatile uint32_t v87_sd_update_max_us = 0UL;

volatile uint8_t v87_sd_auto_stop_done = 0U;
volatile uint32_t v87_sd_auto_stop_duration_ms = 0UL;


volatile uint8_t v811_sd_finalize_started = 0U;
volatile uint8_t v811_sd_finalize_done = 0U;
volatile uint32_t v811_sd_finalize_duration_ms = 0UL;
volatile uint32_t v811_scheduler_rebase_count = 0UL;

volatile uint32_t v811_nrf_deadline_before_finalize = 0UL;
volatile uint32_t v811_nrf_deadline_after_finalize = 0UL;
volatile uint32_t v811_lidar_deadline_before_finalize = 0UL;
volatile uint32_t v811_lidar_deadline_after_finalize = 0UL;

static uint8_t v87_profile_was_running = 0U;

/* P34 UART-first deterministic-loop diagnostics. */
volatile uint32_t p34_bg_fresh_last_us = 0UL;
volatile uint32_t p34_bg_fresh_max_us = 0UL;
volatile uint32_t p34_bg_remote_last_us = 0UL;
volatile uint32_t p34_bg_remote_max_us = 0UL;
volatile uint32_t p34_bg_control_last_us = 0UL;
volatile uint32_t p34_bg_control_max_us = 0UL;
volatile uint32_t p34_bg_uart_last_us = 0UL;
volatile uint32_t p34_bg_uart_max_us = 0UL;
volatile uint32_t p34_covariance_last_us = 0UL;
volatile uint32_t p34_covariance_max_us = 0UL;
volatile uint32_t p34_covariance_service_count = 0UL;
volatile uint32_t p34_sd_defer_count = 0UL;
volatile uint32_t p34_control_defer_count = 0UL;
volatile uint32_t p34_uart_defer_count = 0UL;

static void P34_RecordExternal(uint32_t start_us, volatile uint32_t *last_us,
                               volatile uint32_t *max_us)
{
    uint32_t elapsed = (uint32_t)(micros() - start_us);
    if (last_us != 0) *last_us = elapsed;
    if ((max_us != 0) && (elapsed > *max_us)) *max_us = elapsed;
    Scheduler_RecordExternalBusyTime(elapsed);
}


/* V40 driver-level UART diagnostics (superseded by the V55 telemetry owner)
 * were removed here; see git history before this commit if ever needed. */

/* Still used by the V43 retry path in App_Init(). */
extern volatile uint8_t imu_phy_spi_response_ok;
extern volatile uint32_t imu_v43_reinit_attempt_count;

/* -------------------------------------------------------------------------- */
/* P59 two-switch remote safety: ground vent + one-way emergency STOP         */
/* -------------------------------------------------------------------------- */

#define V30_SWITCH1_MASK            0x01U
#define V30_SWITCH2_MASK            0x02U
#define V30_GREEN_LED_PORT          BOARD_STATUS_LED_PORT
#define V30_GREEN_LED_PIN           BOARD_STATUS_LED_PIN
#define V30_ORANGE_LED_PORT         BOARD_LINK_LED_PORT
#define V30_ORANGE_LED_PIN          BOARD_LINK_LED_PIN
#define V30_RED_LED_PORT            BOARD_ESTOP_LED_PORT
#define V30_RED_LED_PIN             BOARD_ESTOP_LED_PIN

/* Existing names are retained because UART/Live-Expression tooling already
 * uses them. P59 upgrades the old indicator-only vent into a bounded physical
 * ground-vent sequencer without changing IMU/ESKF/SD/needle algorithms. */
volatile uint8_t v30_switch1_vent_request = 0U;
volatile uint8_t v30_vent_pulse_active = 0U;
volatile uint32_t v30_vent_pulse_count = 0UL;
volatile uint8_t v30_vent_physical_enabled = APP_REMOTE_VENT_PHYSICAL_ENABLED;
volatile uint8_t v30_vent_link_fresh = 0U;
volatile uint32_t v30_vent_stale_block_count = 0UL;
volatile uint8_t v30_vent_override_active = 0U;
volatile uint8_t v30_vent_channel = 0U;
volatile uint8_t v30_vent_applied_mask = 0U;
volatile uint32_t v30_vent_start_count = 0UL;
volatile uint32_t v30_vent_complete_count = 0UL;
volatile uint32_t v30_vent_abort_count = 0UL;
volatile uint32_t v30_vent_blocked_flight_count = 0UL;
volatile uint32_t v30_vent_blocked_stop_count = 0UL;

/* P70 post-PE9 grounded vent diagnostics.  These are deliberately not part of
 * flight authorization; they only decide whether the manual vent override may
 * be used after the PE9 separation latch has made flight_active permanent. */
volatile uint8_t v30_postsep_ground_candidate = 0U;
volatile uint8_t v30_postsep_ground_safe = 0U;
volatile uint32_t v30_postsep_ground_stable_ms = 0UL;
volatile uint32_t v30_postsep_ground_reject_count = 0UL;
volatile uint32_t v30_postsep_vent_start_count = 0UL;

volatile uint8_t v30_switch2_stop_request = 0U;
volatile uint8_t v30_stop_latched = 0U;

/* P60: one canonical authorization decision for every normal flight actuator
 * path.  The remote STOP latch is intentionally part of this decision rather
 * than being only a last-moment output clamp.  This gives two independent
 * layers: authorization is revoked first, and the existing RCS/needle safe
 * forcing remains in place as the final output guard. */
uint8_t App_IsActuatorAuthorized(void)
{
    const FullStateESKFData_t *eskf = FullStateESKF_GetDataPtr();

    if (v30_stop_latched != 0U)
    {
        return 0U;
    }

    if ((PreflightTrigger_IsFlightActive() == 0U) ||
        (PreflightTrigger_HasFault() != 0U) ||
        (SystemMonitor_IsActuatorFaultActive() != 0U))
    {
        return 0U;
    }

    if ((eskf == 0) || (eskf->output_inhibited != 0U))
    {
        return 0U;
    }

    return 1U;
}

uint8_t App_IsStopLatched(void)
{
    return (v30_stop_latched != 0U) ? 1U : 0U;
}
volatile uint32_t v30_stop_latch_count = 0UL;
volatile uint8_t v30_remote_flags = 0U;

static uint32_t v30_vent_phase_start_ms = 0UL;
static uint32_t v30_vent_request_since_ms = 0UL;
static uint8_t v30_vent_request_qualified = 0U;
static uint8_t v30_prev_vent_request = 0U;

/* P71 vent-only pairing:
 *   pair 0 = X+ and X- together (ROLL opposing pair)
 *   pair 1 = Y+ and Y- together (PITCH opposing pair)
 *
 * The normal attitude-control path still forbids opposing valves.  Only the
 * dedicated manual vent path may request these two balanced pairs. */
static uint8_t V30_VentMaskForChannel(uint8_t channel)
{
    switch (channel)
    {
        case 0U:
            return (uint8_t)(SOLENOID_VALVE_ROLL_POS_ERROR |
                             SOLENOID_VALVE_ROLL_NEG_ERROR);
        case 1U:
            return (uint8_t)(SOLENOID_VALVE_PITCH_POS_ERROR |
                             SOLENOID_VALVE_PITCH_NEG_ERROR);
        default:
            return SOLENOID_VALVE_NONE;
    }
}

static uint32_t v30_postsep_ground_candidate_since_ms = 0UL;

static float V30_AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t V30_UpdatePostSeparationGroundQualification(uint32_t now_ms)
{
#if (APP_REMOTE_VENT_POSTSEP_GROUNDED_ENABLE != 0U)
    const LidarData_t *lidar = Lidar_GetDataPtr();
    const FullStateESKFData_t *eskf = FullStateESKF_GetDataPtr();
    uint8_t candidate = 0U;

    if (PreflightTrigger_IsFlightActive() != 0U)
    {
        if ((lidar != 0) && (eskf != 0) &&
            (lidar->initialized != 0U) &&
            (lidar->connected != 0U) &&
            (lidar->distance_valid != 0U) &&
            (lidar->last_sample_timestamp_us != 0UL) &&
            ((uint32_t)(micros() - lidar->last_sample_timestamp_us) <=
             APP_REMOTE_VENT_POSTSEP_MAX_LIDAR_AGE_US) &&
            (lidar->filtered_distance_m >= APP_REMOTE_VENT_POSTSEP_MIN_LIDAR_M) &&
            (lidar->filtered_distance_m <= APP_REMOTE_VENT_POSTSEP_MAX_LIDAR_M) &&
            (eskf->initialized != 0U) &&
            (eskf->healthy != 0U) &&
            (eskf->output_inhibited == 0U) &&
            (eskf->lidar_fresh != 0U) &&
            (V30_AbsF(eskf->velocity_z_mps) <=
             APP_REMOTE_VENT_POSTSEP_MAX_ABS_VZ_MPS))
        {
            candidate = 1U;
        }
    }

    if (candidate != 0U)
    {
        if (v30_postsep_ground_candidate == 0U)
        {
            v30_postsep_ground_candidate_since_ms = now_ms;
        }

        v30_postsep_ground_candidate = 1U;
        v30_postsep_ground_stable_ms =
            (uint32_t)(now_ms - v30_postsep_ground_candidate_since_ms);
        v30_postsep_ground_safe =
            (v30_postsep_ground_stable_ms >=
             APP_REMOTE_VENT_POSTSEP_GROUND_STABLE_MS) ? 1U : 0U;
    }
    else
    {
        if (v30_postsep_ground_candidate != 0U)
        {
            v30_postsep_ground_reject_count++;
        }
        v30_postsep_ground_candidate = 0U;
        v30_postsep_ground_safe = 0U;
        v30_postsep_ground_stable_ms = 0UL;
        v30_postsep_ground_candidate_since_ms = now_ms;
    }

    return v30_postsep_ground_safe;
#else
    (void)now_ms;
    v30_postsep_ground_candidate = 0U;
    v30_postsep_ground_safe = 0U;
    v30_postsep_ground_stable_ms = 0UL;
    return 0U;
#endif
}

static void V30_ConfigureSafetyLeds(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();

    HAL_GPIO_WritePin(
        GPIOD,
        V30_GREEN_LED_PIN | V30_ORANGE_LED_PIN | V30_RED_LED_PIN,
        GPIO_PIN_RESET
    );

    gpio.Pin = V30_GREEN_LED_PIN | V30_ORANGE_LED_PIN | V30_RED_LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &gpio);
}

static void V30_StopVentSequence(uint8_t count_abort)
{
    uint8_t was_active = v30_vent_override_active;

    if (was_active != 0U)
    {
        (void)SolenoidOutput_SetGroundVentMask(SOLENOID_VALVE_NONE, v30_postsep_ground_safe);
        if (count_abort != 0U)
        {
            v30_vent_abort_count++;
        }
        else
        {
            v30_vent_complete_count++;
        }
    }
    else
    {
        SolenoidOutput_ForceSafe();
    }

    v30_vent_override_active = 0U;
    v30_vent_pulse_active = 0U;
    v30_vent_channel = 0U;
    v30_vent_applied_mask = 0U;
    v30_vent_request_qualified = 0U;
    HAL_GPIO_WritePin(V30_GREEN_LED_PORT, V30_GREEN_LED_PIN, GPIO_PIN_RESET);
}

static void V30_StartVentSequence(uint32_t now)
{
    /* Clear any in-progress attitude pulse BEFORE the manual vent takes
     * ownership. While override_active is set, app_tasks.c does not re-apply
     * automatic RCS masks. */
    v30_vent_override_active = 1U;
    AttitudeControl_ForceSafe();
    SolenoidOutput_ForceSafe();

    v30_vent_channel = 0U;
    v30_vent_applied_mask = 0U;
    v30_vent_pulse_active = 0U;
    v30_vent_phase_start_ms = now;
    v30_vent_start_count++;
    if (PreflightTrigger_IsFlightActive() != 0U)
    {
        v30_postsep_vent_start_count++;
    }
}

static void V30_ServiceVentSequence(uint32_t now)
{
    if (v30_vent_override_active == 0U)
    {
        V30_StartVentSequence(now);
        return;
    }

    if (v30_vent_pulse_active != 0U)
    {
        if ((uint32_t)(now - v30_vent_phase_start_ms) >=
            APP_REMOTE_VENT_SOLENOID_ON_MS)
        {
            (void)SolenoidOutput_SetGroundVentMask(SOLENOID_VALVE_NONE, v30_postsep_ground_safe);
            v30_vent_applied_mask = 0U;
            v30_vent_pulse_active = 0U;
            v30_vent_phase_start_ms = now;
            v30_vent_channel = (uint8_t)((v30_vent_channel + 1U) & 0x01U);
            HAL_GPIO_WritePin(
                V30_GREEN_LED_PORT, V30_GREEN_LED_PIN, GPIO_PIN_RESET
            );
        }
    }
    else if ((uint32_t)(now - v30_vent_phase_start_ms) >=
             APP_REMOTE_VENT_DEADTIME_MS)
    {
        uint8_t mask = V30_VentMaskForChannel(v30_vent_channel);

        if (SolenoidOutput_SetGroundVentMask(mask, v30_postsep_ground_safe) != 0U)
        {
            v30_vent_applied_mask = mask;
            v30_vent_pulse_active = 1U;
            v30_vent_pulse_count++;
            v30_vent_phase_start_ms = now;
            HAL_GPIO_WritePin(
                V30_GREEN_LED_PORT, V30_GREEN_LED_PIN, GPIO_PIN_SET
            );
        }
        else
        {
            V30_StopVentSequence(1U);
        }
    }
}

static void V30_UpdateRemoteSafety(void)
{
    uint32_t now = millis();
    uint8_t flags = RemoteControl_GetCommandFlags();
    uint8_t link = RemoteControl_IsLinkActive();
    uint8_t vent_requested_raw;

    /* Always refresh the post-separation ground qualifier, even when no vent
     * command is present, so a bench/landed vehicle is ready before Switch-1
     * begins its existing hold-to-confirm timer. */
    (void)V30_UpdatePostSeparationGroundQualification(now);

    if (link == 0U)
    {
        flags = 0U;
    }

    /* A maintained vent command is more conservative than the generic radio
     * link indicator. If valid packets stop arriving, the physical vent is
     * released after this shorter freshness window even though the orange
     * link LED may remain ON until APP_REMOTE_LINK_TIMEOUT_MS. */
    v30_vent_link_fresh =
        ((link != 0U) &&
         (remote_rx_last_packet_age_ms <= APP_REMOTE_VENT_MAX_PACKET_AGE_MS))
            ? 1U : 0U;

    v30_remote_flags = flags;
    vent_requested_raw = ((flags & V30_SWITCH1_MASK) != 0U) ? 1U : 0U;
    v30_switch1_vent_request =
        ((vent_requested_raw != 0U) && (v30_vent_link_fresh != 0U)) ? 1U : 0U;
    v30_switch2_stop_request =
        ((flags & V30_SWITCH2_MASK) != 0U) ? 1U : 0U;

    if ((vent_requested_raw != 0U) && (v30_vent_link_fresh == 0U))
    {
        if (v30_prev_vent_request != 0U)
        {
            v30_vent_stale_block_count++;
        }
    }

    /* Switch-2 is a one-way STOP latch. A valid packet sets it immediately;
     * releasing the switch or losing RF can never clear it. */
    if ((v30_switch2_stop_request != 0U) && (v30_stop_latched == 0U))
    {
        v30_stop_latched = 1U;
        v30_stop_latch_count++;
    }

    HAL_GPIO_WritePin(
        V30_RED_LED_PORT,
        V30_RED_LED_PIN,
        (v30_stop_latched != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET
    );

    /* STOP always wins. It also terminates a vent pulse before forcing the
     * normal attitude and needle paths safe. */
    if (v30_stop_latched != 0U)
    {
        if (v30_vent_override_active != 0U)
        {
            v30_vent_blocked_stop_count++;
            V30_StopVentSequence(1U);
        }

        AttitudeControl_ForceSafe();
        SolenoidOutput_ForceSafe();
#if (APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE != 0U)
        NeedleValveAutonomousControl_EStopSafeCloseUpdate();
#else
        NeedleValveController_Stop();
#endif
        return;
    }

    /* Vent is a maintained command: release or RF timeout closes immediately. */
    if (v30_switch1_vent_request == 0U)
    {
        if (v30_vent_override_active != 0U)
        {
            V30_StopVentSequence(0U);
        }

        v30_prev_vent_request = 0U;
        v30_vent_request_qualified = 0U;
        return;
    }

    /* New ON edge starts a deliberate hold-to-confirm timer. */
    if (v30_prev_vent_request == 0U)
    {
        v30_prev_vent_request = 1U;
        v30_vent_request_since_ms = now;
        v30_vent_request_qualified = 0U;
    }

#if (APP_REMOTE_VENT_GROUND_ONLY != 0U)
    if ((PreflightTrigger_IsFlightActive() != 0U) &&
        (v30_postsep_ground_safe == 0U))
    {
        if (v30_vent_override_active != 0U)
        {
            V30_StopVentSequence(1U);
        }
        v30_vent_blocked_flight_count++;
        return;
    }
#endif

    if (v30_vent_physical_enabled == 0U)
    {
        return;
    }

    if (v30_vent_request_qualified == 0U)
    {
        if ((uint32_t)(now - v30_vent_request_since_ms) <
            APP_REMOTE_VENT_HOLD_CONFIRM_MS)
        {
            return;
        }
        v30_vent_request_qualified = 1U;
    }

    V30_ServiceVentSequence(now);
}

void App_Init(void)
{
    RuntimeDelayGuard_Init();
    SolenoidOutput_Init();
    V30_ConfigureSafetyLeds();

    Board_Init();
    BusManager_Init();
    SystemMonitor_Init();

    /*
     * Known-working small-board nRF startup order:
     * bring the receiver up before the sensor/application task initialization.
     * This touches SPI3 only and does not alter IMU/SPI1, BARO/SPI2, SDIO,
     * relay, servo or needle-valve drivers.
     */
#if (APP_NRF24_ENABLED != 0U)
    RemoteControl_Init();
#if (APP_NRF_ROCKET_DOWNLINK_ENABLED != 0U)
    NRFTelemetry_Init();
#endif
#endif

    NeedleValveController_Init();
    PreflightTrigger_Init();
#if (APP_RELAY_SEQUENCE_TEST_MODE != 0U)
    SolenoidOutput_BenchTestInit();
#endif

    /*
     * Existing tahliye servo implementation from the earlier full project:
     * PB6 / TIM4_CH1 / 50 Hz.
     * ServoOutput_Init() always starts at CLOSED pulse.
     */
    ServoOutput_Init();

    AppTasks_Init();

    /* V43: if normal IMU startup fails, fingerprint the physical SPI lines and
     * registers, restore AF5, then retry IMU initialization once. No other
     * sensor/control subsystem is reinitialized or modified. */
    IMU_RunPhysicalLineDiagnostic();
    if ((IMU_IsConnected() == 0U) && (imu_phy_spi_response_ok != 0U))
    {
        imu_v43_reinit_attempt_count++;
        IMU_Init();
    }

#if (APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE != 0U)
    NeedleValveIntegrationTest_Init();
    NeedleValveAutonomousControl_Init();
#elif (APP_NEEDLE_AUTO_CONTROL_MODE != 0U)
    NeedleValveAutoControl_Init();
#elif (APP_NEEDLE_COMMISSIONING_MODE != 0U)
    NeedleValveIntegrationTest_Init();
#endif

    /*
     * P112R5 SD boot isolation:
     * NeedleValveController_Init() has already configured and started TIM7 at
     * 1 kHz. The blocking SD mount/preallocation path must not run concurrently
     * with the actuator/RCS control tick during boot. Save the exact TIM7 state,
     * stop the timer + update interrupt for the whole SD startup sequence, then
     * restart only after SDLogger_Init() has completely finished.
     *
     * Runtime SDLogger_Update() DMA/defer service and the full-autonomous
     * actuator ISR/background logic are intentionally unchanged.
     */
#if (APP_SDLOGGER_ENABLED != 0U)
    {
        uint8_t sd_boot_attempt;
        uint32_t tim7_cr1_before_sd = TIM7->CR1;
        uint32_t tim7_dier_before_sd = TIM7->DIER;
        uint32_t tim7_nvic_enabled_before_sd = NVIC_GetEnableIRQ(TIM7_IRQn);

        /* Quiesce the 1 kHz actuator/RCS control tick before SD startup. */
        NVIC_DisableIRQ(TIM7_IRQn);
        TIM7->DIER &= ~TIM_DIER_UIE;
        TIM7->CR1 &= ~TIM_CR1_CEN;
        __DSB();
        __ISB();

        /* Remove any update request that was latched while stopping TIM7. */
        TIM7->SR &= ~TIM_SR_UIF;
        NVIC_ClearPendingIRQ(TIM7_IRQn);

        /* P32: SD initialization is allowed to block only before the scheduler
         * starts. Some cards need more than one complete power/host negotiation.
         * Never perform this retry loop after flight scheduling begins. */
        for (sd_boot_attempt = 0U; sd_boot_attempt < 3U; sd_boot_attempt++)
        {
            SDLogger_Init();
            if (SDLogger_IsReady() != 0U)
            {
                break;
            }
            HAL_Delay(250U);
        }

        /*
         * SD startup is now completely finished (success or exhausted retries).
         * Start TIM7 from a clean phase so no stale boot-time update interrupt
         * can execute an actuator/RCS tick immediately after SD init.
         */
        TIM7->CNT = 0UL;
        TIM7->SR &= ~TIM_SR_UIF;
        NVIC_ClearPendingIRQ(TIM7_IRQn);

        /* Restore the exact pre-SD interrupt configuration while still stopped. */
        TIM7->DIER = tim7_dier_before_sd;
        if (tim7_nvic_enabled_before_sd != 0UL)
        {
            NVIC_EnableIRQ(TIM7_IRQn);
        }
        else
        {
            NVIC_DisableIRQ(TIM7_IRQn);
        }

        /* CR1 is restored last; if TIM7 was running, this is the restart point. */
        TIM7->CR1 = tim7_cr1_before_sd;
        __DSB();
        __ISB();
    }
#endif

    Scheduler_Init();

    p34_bg_fresh_last_us = 0UL;
    p34_bg_fresh_max_us = 0UL;
    p34_bg_remote_last_us = 0UL;
    p34_bg_remote_max_us = 0UL;
    p34_bg_control_last_us = 0UL;
    p34_bg_control_max_us = 0UL;
    p34_bg_uart_last_us = 0UL;
    p34_bg_uart_max_us = 0UL;
    p34_covariance_last_us = 0UL;
    p34_covariance_max_us = 0UL;
    p34_covariance_service_count = 0UL;
    p34_sd_defer_count = 0UL;
    p34_control_defer_count = 0UL;
    p34_uart_defer_count = 0UL;

    v87_sd_update_count = 0UL;
    v87_sd_update_last_us = 0UL;
    v87_sd_update_max_us = 0UL;
    v87_sd_auto_stop_done = 0U;
    v87_sd_auto_stop_duration_ms = 0UL;
    v87_profile_was_running = 0U;

    v811_sd_finalize_started = 0U;
    v811_sd_finalize_done = 0U;
    v811_sd_finalize_duration_ms = 0UL;
    v811_scheduler_rebase_count = 0UL;

    v811_nrf_deadline_before_finalize = 0UL;
    v811_nrf_deadline_after_finalize = 0UL;
    v811_lidar_deadline_before_finalize = 0UL;
    v811_lidar_deadline_after_finalize = 0UL;

    /* USART2 is initialized by main.c. V55 owns the DMA stream from here on;
     * the legacy multiline V40 dump is intentionally not scheduled because it
     * would contaminate the machine-readable CRC protected CSV stream. */
    UARTTelemetry_Init();

    /* R8R35R3R7R1R1: App_Init is complete. From this point onward any
     * accidental HAL_Delay() call is recorded and returns immediately. */
    RuntimeDelayGuard_MarkRuntimeStarted();
}

void App_Run(void)
{
    uint32_t t0;
    uint32_t dt;
    static uint32_t last_fresh_service_us = 0UL;
    static uint32_t last_remote_service_us = 0UL;
    static uint32_t last_sd_service_us = 0UL;
    static uint32_t last_control_service_us = 0UL;
    static uint32_t last_uart_attempt_us = 0UL;

    /* P34 hard rule: service every due IMU release before any main-context
     * background work. Re-enter the scheduler between background groups so a
     * newly due 1 kHz release never waits for the remainder of App_Run(). */
    Scheduler_Run();

    /* Freshness, preflight and remote safety only need millisecond latency.
     * P33 called them on every spin of the while(1) loop; P34 caps this work at
     * 1 kHz while still checking the hard scheduler continuously. */
    if (((uint32_t)(micros() - last_fresh_service_us) >= 1000UL) &&
        (Scheduler_HasIMUSlack(APP_P36_FRESH_MIN_SLACK_US) != 0U))
    {
        last_fresh_service_us = micros();
        t0 = micros();
        SensorManager_ServiceFreshness();
        PreflightTrigger_Update();
        P34_RecordExternal(t0, &p34_bg_fresh_last_us, &p34_bg_fresh_max_us);
        Scheduler_Run();
    }

    /* R8R35R3R1 UART visibility hotfix: the SD-only closure moved a 1 kHz
     * SD slice ahead of UART while UART still required 600 us IMU slack.
     * On the current load this could leave the final UART background group
     * permanently deferred even though control continued normally.
     * Give UART its 10 Hz diagnostic opportunity before SD; this does not
     * change UART's 600 us safety admission rule or any control authority. */
    /* R8R35R3R2 UART phase-retry hotfix:
     * Do NOT advance last_uart_attempt_us when the 600 us admission check
     * fails. R3R1 advanced it on every defer, which could phase-lock the
     * UART attempt to the same bad point in each 1 ms IMU cycle forever.
     * With this form, a deferred UART attempt is retried on subsequent
     * main-loop spins until a safe >=600 us window appears. The actual
     * UART formatter still remains internally limited to 10 Hz. */
    if ((uint32_t)(micros() - last_uart_attempt_us) >= 1000UL)
    {
        if (NeedleValveIntegrationTest_IsTimingCritical() != 0U)
        {
            last_uart_attempt_us = micros();
            p112_uart_suppressed_count++;
            p34_uart_defer_count++;
        }
        else if (Scheduler_HasIMUSlack(APP_P34_UART_MIN_SLACK_US) != 0U)
        {
            last_uart_attempt_us = micros();
            t0 = micros();
            UARTTelemetry_Update10Hz();
            P34_RecordExternal(t0, &p34_bg_uart_last_us, &p34_bg_uart_max_us);
            Scheduler_Run();
        }
        else
        {
            /* Intentionally keep last_uart_attempt_us unchanged so the
             * next main-loop spin can retry at a different scheduler phase. */
            p34_uart_defer_count++;
        }
    }

    /* R8R35R3: SD-only service closure. Keep SD before the comparatively expensive nRF/remote
     * background group. R8R35R1 proved that keeping SD after remote starved
     * the 200 Hz capture ring (127/127 with continuous overruns) even though
     * the card/DMA writer itself remained healthy. Sensor freshness/preflight
     * still runs first; hard 1 kHz releases are serviced between groups. */
#if (APP_SDLOGGER_ENABLED != 0U)
    if ((SDLogger_IsReady() != 0U) && (SDLogger_IsLogging() != 0U) &&
        ((uint32_t)(micros() - last_sd_service_us) >= 1000UL))
    {
        /* R8R35R3R6: P71 known-good SD stack retained; R3R3 phase-retry admission retained:
         * Do NOT advance last_sd_service_us when the IMU-slack admission
         * check fails. R8R35R3/R3R2 advanced the timestamp before admission,
         * so every failed attempt waited another 1 ms and could phase-lock
         * to the same busy point of the IMU cycle. The physical log proved
         * the 200 Hz capture ISR was healthy (push+drop ~= 200 Hz) while the
         * main-context SD update/pop path ran only ~41 Hz.
         *
         * A deferred SD slice is now retried on subsequent main-loop spins
         * until a safe >= APP_P34_SD_MIN_SLACK_US window appears. P71 adaptive ring drain and 220 us SD admission are restored; IMU guard,
         * 200 Hz capture rate and every control/actuator path remain unchanged. */
        if (Scheduler_HasIMUSlack(APP_P34_SD_MIN_SLACK_US) != 0U)
        {
            last_sd_service_us = micros();
            t0 = micros();
            SDLogger_Update();
            dt = (uint32_t)(micros() - t0);
            v87_sd_update_last_us = dt;
            if (dt > v87_sd_update_max_us) v87_sd_update_max_us = dt;
            v87_sd_update_count++;
            Scheduler_RecordExternalBusyTime(dt);
            Scheduler_Run();
        }
        else
        {
            /* Keep last_sd_service_us unchanged so the next main-loop spin
             * retries instead of waiting another millisecond in the same phase. */
            p34_sd_defer_count++;
        }
    }
#endif

    if (((uint32_t)(micros() - last_remote_service_us) >= 1000UL) &&
        (Scheduler_HasIMUSlack(APP_P36_REMOTE_MIN_SLACK_US) != 0U))
    {
        last_remote_service_us = micros();
        t0 = micros();
#if (APP_NRF24_ENABLED != 0U)
        RemoteControl_Update();
#if (APP_NRF_ROCKET_DOWNLINK_ENABLED != 0U)
        NRFTelemetry_Service();
#endif
        V30_UpdateRemoteSafety();
        ServoOutput_Update(
            RemoteControl_GetCommand(),
            RemoteControl_IsLinkActive()
        );
#else
        ServoOutput_Update(0U, 0U);
#endif
        P34_RecordExternal(t0, &p34_bg_remote_last_us, &p34_bg_remote_max_us);
        Scheduler_Run();
    }

    /* P36: covariance is a dedicated scheduler task at 25 Hz. */


#if (APP_NEEDLE_RAW_CHARACTERIZATION_MODE != 0U)
    /* P112R5: the function is a no-op in main context; all ADC register
     * ownership remains serialized inside TIM7. */
    NeedleValveIntegrationTest_FastButtonService();
#endif

    /* Control background bookkeeping is limited to 1 kHz and only starts when
     * there is room before the next IMU slot. Flight needle/RCS commands remain
     * owned by the native 200 Hz ESKF task and exact TIM7 control timer. */
    if ((uint32_t)(micros() - last_control_service_us) >= 1000UL)
    {
        last_control_service_us = micros();
        if (Scheduler_HasIMUSlack(APP_P34_CONTROL_MIN_SLACK_US) != 0U)
        {
            t0 = micros();
#if (APP_RELAY_SEQUENCE_TEST_MODE != 0U)
            SolenoidOutput_BenchTestUpdate();
#elif (APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE != 0U)
            NeedleValveIntegrationTest_Update();
            NeedleValveAutonomousControl_BackgroundUpdate();
#elif (APP_NEEDLE_AUTO_CONTROL_MODE != 0U)
            if ((PreflightTrigger_IsFlightActive() == 0U) &&
                (PreflightTrigger_HasFault() == 0U)) NeedleValveAutoControl_Update();
            else NeedleValveController_Stop();
#elif (APP_NEEDLE_COMMISSIONING_MODE != 0U)
            NeedleValveIntegrationTest_Update();
#else
            GNCActiveControl_MainLoop();
#endif
            P34_RecordExternal(t0, &p34_bg_control_last_us, &p34_bg_control_max_us);
            Scheduler_Run();
        }
        else
        {
            p34_control_defer_count++;
        }
    }

    if (v30_stop_latched != 0U)
    {
        AttitudeControl_ForceSafe();
#if (APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE != 0U)
        NeedleValveAutonomousControl_EStopSafeCloseUpdate();
#else
        NeedleValveController_Stop();
#endif
    }

    Scheduler_Run();
}
