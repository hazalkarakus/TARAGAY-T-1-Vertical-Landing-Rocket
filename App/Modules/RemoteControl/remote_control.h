#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>

void RemoteControl_Init(void);
void RemoteControl_Update(void);
void RemoteControl_OnIrq(void);

uint8_t RemoteControl_GetCommand(void);
uint8_t RemoteControl_IsLinkActive(void);
uint8_t RemoteControl_GetCommandFlags(void);
uint8_t RemoteControl_GetSwitch2Command(void);

extern volatile uint32_t remote_rx_bad_magic_count;
extern volatile uint32_t remote_rx_bad_flags_count;
extern volatile uint32_t remote_rx_bad_checksum_count;


/* V8.9 monitor-only Live Expressions. */
extern volatile uint8_t remote_rx_command;
extern volatile uint8_t remote_rx_command_flags;
extern volatile uint8_t remote_rx_switch2_command;
extern volatile uint8_t remote_rx_link_active;
extern volatile uint8_t remote_rx_last_sequence;
extern volatile uint8_t remote_rx_last_packet_valid;
extern volatile uint32_t remote_rx_valid_packet_count;
extern volatile uint32_t remote_rx_invalid_packet_count;
extern volatile uint32_t remote_rx_timeout_count;
extern volatile uint32_t remote_rx_irq_count;
extern volatile uint32_t remote_rx_last_packet_age_ms;
extern volatile uint32_t remote_rx_flight_rearm_count;


/* V8.14 command deglitch diagnostics. */
extern volatile uint8_t remote_rx_candidate_command;
extern volatile uint32_t remote_rx_candidate_age_ms;
extern volatile uint32_t remote_rx_command_change_count;
extern volatile uint32_t remote_rx_command_chatter_count;

#endif
