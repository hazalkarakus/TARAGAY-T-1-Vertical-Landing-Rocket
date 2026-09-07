#include "usart.h"

#include <stdint.h>

/*
 * USART2 telemetry transport
 * ---------------------------
 * TX  : PA2 / AF7
 * RX  : PA3 / AF7 (configured for future use; telemetry is TX-only today)
 * Link: 115200 baud, 8 data bits, no parity, 1 stop bit
 * DMA : DMA1 Stream 6, Channel 4, memory-to-peripheral, normal mode
 *
 * This transport deliberately uses the STM32 registers directly. The project
 * did not previously include the HAL UART module, so this keeps the existing
 * sensor/SD build untouched while still providing non-blocking UART DMA TX.
 */

volatile uint8_t usart2_initialized = 0U;
volatile uint8_t usart2_tx_dma_busy = 0U;
volatile uint32_t usart2_tx_dma_start_count = 0UL;
volatile uint32_t usart2_tx_dma_complete_count = 0UL;
volatile uint32_t usart2_tx_dma_error_count = 0UL;
volatile uint32_t usart2_tx_dma_busy_reject_count = 0UL;
volatile uint16_t usart2_tx_dma_last_length = 0U;
volatile uint32_t usart2_actual_baud_rate = 0UL;

#define USART2_DMA_CLEAR_FLAGS \
    (DMA_HIFCR_CFEIF6  | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CTEIF6 | \
     DMA_HIFCR_CHTIF6  | DMA_HIFCR_CTCIF6)

#define USART2_DMA_ERROR_FLAGS \
    (DMA_HISR_DMEIF6 | DMA_HISR_TEIF6)

static void USART2_DMA_DisableAndWait(void)
{
    DMA1_Stream6->CR &= ~DMA_SxCR_EN;

    while ((DMA1_Stream6->CR & DMA_SxCR_EN) != 0U)
    {
        /* Hardware clears EN after the current AHB transaction completes. */
    }
}

void MX_USART2_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint32_t peripheral_clock_hz;
    uint32_t brr_value;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    /* PA2 = USART2_TX, PA3 = USART2_RX. */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Put UART and DMA into a known disabled state before reconfiguration. */
    USART2->CR1 = 0U;
    USART2->CR2 = 0U;
    USART2->CR3 = 0U;

    USART2_DMA_DisableAndWait();
    DMA1->HIFCR = USART2_DMA_CLEAR_FLAGS;

    peripheral_clock_hz = HAL_RCC_GetPCLK1Freq();

    /* OVER8=0. BRR numerical value is round(PCLK / baud). */
    brr_value =
        (peripheral_clock_hz + (USART2_TELEMETRY_BAUD_RATE / 2UL)) /
        USART2_TELEMETRY_BAUD_RATE;

    if (brr_value == 0UL)
    {
        brr_value = 1UL;
    }

    USART2->BRR = brr_value;

    /* 8-N-1, transmitter and receiver enabled, oversampling by 16. */
    USART2->CR2 = 0U;
    USART2->CR3 = USART_CR3_DMAT;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    /* DMA1 Stream6 Channel4 = USART2_TX. Byte-wide, normal, memory increment. */
    DMA1_Stream6->PAR = (uint32_t)&USART2->DR;
    DMA1_Stream6->M0AR = 0U;
    DMA1_Stream6->NDTR = 0U;
    DMA1_Stream6->FCR = 0U;
    DMA1_Stream6->CR =
        DMA_SxCR_CHSEL_2 |
        DMA_SxCR_DIR_0 |
        DMA_SxCR_MINC |
        DMA_SxCR_TCIE |
        DMA_SxCR_TEIE |
        DMA_SxCR_DMEIE;

    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 10U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    usart2_tx_dma_busy = 0U;
    usart2_tx_dma_start_count = 0UL;
    usart2_tx_dma_complete_count = 0UL;
    usart2_tx_dma_error_count = 0UL;
    usart2_tx_dma_busy_reject_count = 0UL;
    usart2_tx_dma_last_length = 0U;

    usart2_actual_baud_rate = peripheral_clock_hz / brr_value;
    usart2_initialized = 1U;
}

uint8_t USART2_TxDMA_Start(const uint8_t *data, uint16_t length)
{
    uint32_t primask;

    if ((usart2_initialized == 0U) || (data == 0) || (length == 0U))
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if ((usart2_tx_dma_busy != 0U) ||
        ((DMA1_Stream6->CR & DMA_SxCR_EN) != 0U))
    {
        usart2_tx_dma_busy_reject_count++;

        if (primask == 0U)
        {
            __enable_irq();
        }

        return 0U;
    }

    usart2_tx_dma_busy = 1U;
    usart2_tx_dma_last_length = length;

    DMA1->HIFCR = USART2_DMA_CLEAR_FLAGS;
    DMA1_Stream6->M0AR = (uint32_t)data;
    DMA1_Stream6->NDTR = (uint32_t)length;

    __DMB();
    DMA1_Stream6->CR |= DMA_SxCR_EN;

    usart2_tx_dma_start_count++;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return 1U;
}

uint8_t USART2_TxDMA_IsBusy(void)
{
    return usart2_tx_dma_busy;
}

void USART2_TxDMA_IRQHandler(void)
{
    uint32_t status = DMA1->HISR;

    /* Stream6 runs in direct mode. FEIF6 may accompany a completed direct
     * transfer on this STM32F4 configuration and is therefore cleared but is
     * not counted as a failed UART frame. Only TEIF/DMEIF are fatal. */

    if ((status & USART2_DMA_ERROR_FLAGS) != 0U)
    {
        USART2_DMA_DisableAndWait();
        DMA1->HIFCR = USART2_DMA_CLEAR_FLAGS;

        usart2_tx_dma_busy = 0U;
        usart2_tx_dma_error_count++;
        return;
    }

    if ((status & DMA_HISR_TCIF6) != 0U)
    {
        USART2_DMA_DisableAndWait();
        DMA1->HIFCR = USART2_DMA_CLEAR_FLAGS;

        usart2_tx_dma_busy = 0U;
        usart2_tx_dma_complete_count++;
        return;
    }

    /* Clear any unexpected half-transfer/FIFO flag without stalling the link. */
    DMA1->HIFCR = USART2_DMA_CLEAR_FLAGS;
}
