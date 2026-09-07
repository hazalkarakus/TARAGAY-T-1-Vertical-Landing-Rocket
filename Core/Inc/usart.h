#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define USART2_TELEMETRY_BAUD_RATE 115200UL

void MX_USART2_UART_Init(void);

uint8_t USART2_TxDMA_Start(const uint8_t *data, uint16_t length);
uint8_t USART2_TxDMA_IsBusy(void);
void USART2_TxDMA_IRQHandler(void);

extern volatile uint8_t usart2_initialized;
extern volatile uint8_t usart2_tx_dma_busy;
extern volatile uint32_t usart2_tx_dma_start_count;
extern volatile uint32_t usart2_tx_dma_complete_count;
extern volatile uint32_t usart2_tx_dma_error_count;
extern volatile uint32_t usart2_tx_dma_busy_reject_count;
extern volatile uint16_t usart2_tx_dma_last_length;
extern volatile uint32_t usart2_actual_baud_rate;

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
