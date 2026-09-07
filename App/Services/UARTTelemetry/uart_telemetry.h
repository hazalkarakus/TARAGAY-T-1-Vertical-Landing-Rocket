#ifndef APP_SERVICES_UART_TELEMETRY_H
#define APP_SERVICES_UART_TELEMETRY_H

#include <stdint.h>

void UARTTelemetry_Init(void);
void UARTTelemetry_Update10Hz(void);

extern volatile uint8_t uart_telemetry_initialized;
extern volatile uint8_t uart_telemetry_streaming;
extern volatile uint32_t uart_telemetry_send_count;
extern volatile uint32_t uart_telemetry_busy_skip_count;
extern volatile uint32_t uart_telemetry_format_error_count;
extern volatile uint32_t uart_telemetry_frame_sequence;
extern volatile uint32_t uart_telemetry_last_service_ms;
extern volatile uint16_t uart_telemetry_last_message_length;
extern volatile uint16_t uart_telemetry_last_crc16;

#endif
