#include "Modules/RemoteControl/remote_control.h"

#include "Common/app_config.h"
#include "Modules/NRF24/nrf24.h"
#include "Platform/board_pins.h"
#include "Services/Timebase/timebase.h"
#include "Services/PreflightTrigger/preflight_trigger.h"

#define REMOTE_PACKET_MAGIC      0xA5U
#define REMOTE_PACKET_CHECK_XOR  0x5AU
#define REMOTE_COMMAND_OFF       0U
#define REMOTE_COMMAND_ON        1U
#define REMOTE_FLAG_SWITCH1      0x01U
#define REMOTE_FLAG_SWITCH2      0x02U
#define REMOTE_FLAGS_ALLOWED     (REMOTE_FLAG_SWITCH1 | REMOTE_FLAG_SWITCH2)

static volatile uint8_t remote_irq_pending = 0U;
static uint8_t remote_command = REMOTE_COMMAND_OFF;
static uint8_t remote_command_flags = 0U;
static uint8_t remote_link_active = 0U;
static uint8_t remote_last_sequence = 0U;
static uint32_t remote_last_packet_ms = 0UL;
static uint32_t remote_last_poll_ms = 0UL;
static uint32_t remote_last_led_toggle_ms = 0UL;
static uint32_t remote_last_rx_rearm_ms = 0UL;

/* CubeIDE Live Expressions */
volatile uint8_t remote_rx_command = 0U;
volatile uint8_t remote_rx_command_flags = 0U;
volatile uint8_t remote_rx_switch2_command = 0U;
volatile uint8_t remote_rx_link_active = 0U;
volatile uint8_t remote_rx_last_sequence = 0U;
volatile uint8_t remote_rx_last_packet_valid = 0U;
volatile uint32_t remote_rx_valid_packet_count = 0UL;
volatile uint32_t remote_rx_invalid_packet_count = 0UL;
volatile uint32_t remote_rx_timeout_count = 0UL;
volatile uint32_t remote_rx_irq_count = 0UL;
volatile uint32_t remote_rx_last_packet_age_ms = 0xFFFFFFFFUL;
volatile uint32_t remote_rx_flight_rearm_count = 0UL;

volatile uint8_t remote_rx_candidate_command = 0U;
volatile uint32_t remote_rx_candidate_age_ms = 0UL;
volatile uint32_t remote_rx_command_change_count = 0UL;
volatile uint32_t remote_rx_command_chatter_count = 0UL;

/* P59 packet rejection diagnostics. nRF hardware CRC remains enabled; these
 * counters diagnose application-protocol mismatches without accepting them. */
volatile uint32_t remote_rx_bad_magic_count = 0UL;
volatile uint32_t remote_rx_bad_flags_count = 0UL;
volatile uint32_t remote_rx_bad_checksum_count = 0UL;

static uint8_t RemoteControl_Checksum(uint8_t command_flags, uint8_t sequence)
{
    return (uint8_t)(REMOTE_PACKET_MAGIC ^ command_flags ^ sequence ^ REMOTE_PACKET_CHECK_XOR);
}

static uint8_t RemoteControl_IsPacketValid(const uint8_t packet[APP_NRF24_PAYLOAD_SIZE])
{
    if (packet[0] != REMOTE_PACKET_MAGIC)
    {
        remote_rx_bad_magic_count++;
        return 0U;
    }

    if ((packet[1] & (uint8_t)~REMOTE_FLAGS_ALLOWED) != 0U)
    {
        remote_rx_bad_flags_count++;
        return 0U;
    }

    if (packet[3] != RemoteControl_Checksum(packet[1], packet[2]))
    {
        remote_rx_bad_checksum_count++;
        return 0U;
    }

    return 1U;
}

static void RemoteControl_ApplyLed(uint32_t now)
{
    (void)now;

    /* P59: keep indication ownership unambiguous.
     * Orange PD13 = valid remote link. Green PD12 is driven only by the
     * physical vent sequencer; red PD14 is driven only by the STOP latch. */
    HAL_GPIO_WritePin(
        BOARD_LINK_LED_PORT,
        BOARD_LINK_LED_PIN,
        (remote_link_active != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}

void RemoteControl_Init(void)
{
    uint32_t now = millis();

    HAL_GPIO_WritePin(BOARD_STATUS_LED_PORT, BOARD_STATUS_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_LINK_LED_PORT, BOARD_LINK_LED_PIN, GPIO_PIN_RESET);

    remote_irq_pending = 0U;
    remote_command = REMOTE_COMMAND_OFF;
    remote_command_flags = 0U;
    remote_link_active = 0U;
    remote_last_sequence = 0U;
    remote_last_packet_ms = now;
    remote_last_poll_ms = now;
    remote_last_led_toggle_ms = now;
    remote_last_rx_rearm_ms = now;

    remote_rx_command = REMOTE_COMMAND_OFF;
    remote_rx_command_flags = 0U;
    remote_rx_switch2_command = 0U;
    remote_rx_link_active = 0U;
    remote_rx_last_sequence = 0U;
    remote_rx_last_packet_valid = 0U;
    remote_rx_valid_packet_count = 0UL;
    remote_rx_invalid_packet_count = 0UL;
    remote_rx_timeout_count = 0UL;
    remote_rx_irq_count = 0UL;
    remote_rx_last_packet_age_ms = 0xFFFFFFFFUL;
    remote_rx_flight_rearm_count = 0UL;

    remote_rx_candidate_command = REMOTE_COMMAND_OFF;
    remote_rx_candidate_age_ms = 0UL;
    remote_rx_command_change_count = 0UL;
    remote_rx_command_chatter_count = 0UL;
    remote_rx_bad_magic_count = 0UL;
    remote_rx_bad_flags_count = 0UL;
    remote_rx_bad_checksum_count = 0UL;

    NRF24_Init();
}

void RemoteControl_Update(void)
{
    uint32_t now = millis();
    uint8_t should_poll = 0U;

    if (remote_irq_pending != 0U)
    {
        remote_irq_pending = 0U;
        should_poll = 1U;
    }

    if ((uint32_t)(now - remote_last_poll_ms) >= APP_REMOTE_POLL_PERIOD_MS)
    {
        remote_last_poll_ms = now;
        should_poll = 1U;
    }

    if (should_poll != 0U)
    {
        uint8_t read_limit = 0U;

        while ((NRF24_IsPayloadAvailable() != 0U) && (read_limit < 3U))
        {
            uint8_t packet[APP_NRF24_PAYLOAD_SIZE] = {0U};
            read_limit++;

            if (NRF24_ReadPayload(packet, APP_NRF24_PAYLOAD_SIZE) == 0U)
            {
                break;
            }

            if (RemoteControl_IsPacketValid(packet) != 0U)
            {
                uint8_t packet_flags = packet[1];
                uint8_t packet_command =
                    ((packet_flags & REMOTE_FLAG_SWITCH1) != 0U) ? REMOTE_COMMAND_ON : REMOTE_COMMAND_OFF;

                if (packet_flags != remote_command_flags)
                {
                    remote_rx_command_change_count++;
                }

                remote_command_flags = packet_flags;
                remote_command = packet_command;
                remote_last_sequence = packet[2];
                remote_link_active = 1U;
                remote_last_packet_ms = now;

                /* Retain V23 live-expression diagnostics without gating command. */
                remote_rx_candidate_command = packet_command;
                remote_rx_candidate_age_ms = 0UL;
                remote_rx_command_flags = remote_command_flags;
                remote_rx_switch2_command =
                    ((remote_command_flags & REMOTE_FLAG_SWITCH2) != 0U) ? 1U : 0U;
                remote_rx_command = remote_command;
                remote_rx_last_sequence = remote_last_sequence;
                remote_rx_link_active = 1U;
                remote_rx_last_packet_valid = 1U;
                remote_rx_valid_packet_count++;
            }
            else
            {
                remote_rx_last_packet_valid = 0U;
                remote_rx_invalid_packet_count++;
            }
        }
    }

    if ((remote_link_active != 0U) &&
        ((uint32_t)(now - remote_last_packet_ms) >= APP_REMOTE_LINK_TIMEOUT_MS))
    {
        remote_link_active = 0U;
        remote_command_flags = 0U;
        remote_command = REMOTE_COMMAND_OFF;

        remote_rx_link_active = 0U;
        remote_rx_command_flags = 0U;
        remote_rx_switch2_command = 0U;
        remote_rx_command = REMOTE_COMMAND_OFF;
        remote_rx_candidate_command = REMOTE_COMMAND_OFF;
        remote_rx_candidate_age_ms = 0UL;
        remote_rx_timeout_count++;
    }

    if (remote_rx_valid_packet_count > 0UL)
    {
        remote_rx_last_packet_age_ms =
            (uint32_t)(now - remote_last_packet_ms);
    }
    else
    {
        remote_rx_last_packet_age_ms = 0xFFFFFFFFUL;
    }

#if (APP_NRF_FLIGHT_COMMAND_PRIORITY_RX_ONLY != 0U)
    /* R8R35R3R10R3: flight safety favors command reception over rocket-side
     * downlink. NRFTelemetry stops creating new TX jobs in flight. If the
     * command heartbeat nevertheless disappears, periodically re-assert PRX
     * (CE high / 4-byte pipe) once the async TX state is idle. This is a short
     * register operation, not a delay/re-init loop. The 1 Hz NRF24_Update()
     * configuration verifier remains as the deeper recovery layer. */
    if ((PreflightTrigger_IsFlightActive() != 0U) &&
        (remote_link_active == 0U) &&
        (NRF24_AsyncTxIsBusy() == 0U) &&
        ((uint32_t)(now - remote_last_rx_rearm_ms) >=
         APP_NRF_FLIGHT_RX_REARM_PERIOD_MS))
    {
        remote_last_rx_rearm_ms = now;
        NRF24_SetRxMode();
        remote_rx_flight_rearm_count++;
    }
#endif

    RemoteControl_ApplyLed(now);
    NRF24_Update();
}

void RemoteControl_OnIrq(void)
{
    remote_irq_pending = 1U;
    remote_rx_irq_count++;
}

uint8_t RemoteControl_GetCommand(void)
{
    return remote_command;
}

uint8_t RemoteControl_IsLinkActive(void)
{
    return remote_link_active;
}

uint8_t RemoteControl_GetCommandFlags(void)
{
    return remote_command_flags;
}

uint8_t RemoteControl_GetSwitch2Command(void)
{
    return ((remote_command_flags & REMOTE_FLAG_SWITCH2) != 0U) ? 1U : 0U;
}
