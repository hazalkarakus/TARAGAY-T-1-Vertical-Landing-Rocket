#ifndef NRF24_H
#define NRF24_H

#include <stdint.h>

typedef enum
{
    NRF24_MODE_POWER_DOWN = 0,
    NRF24_MODE_STANDBY,
    NRF24_MODE_RX,
    NRF24_MODE_TX
} NRF24_Mode_t;

typedef enum
{
    NRF24_ASYNC_TX_RESULT_NONE = 0,
    NRF24_ASYNC_TX_RESULT_SUCCESS = 1,
    NRF24_ASYNC_TX_RESULT_FAIL = 2
} NRF24_AsyncTxResult_t;

typedef struct
{
    uint8_t initialized;
    uint8_t connected; /* SPI/register connection, not RF-link status */
    NRF24_Mode_t mode;

    uint8_t status_reg;
    uint8_t config_reg;
    uint8_t rf_ch_reg;
    uint8_t rf_setup_reg;
    uint8_t fifo_status_reg;

    uint32_t tx_count;
    uint32_t tx_fail_count;
    uint32_t rx_count;
    uint32_t error_count;
} NRF24_Data_t;

void NRF24_Init(void);
void NRF24_Update(void);
void NRF24_SetRxMode(void);
void NRF24_SetTxMode(void);

uint8_t NRF24_SendPayload(const uint8_t *payload, uint8_t length);
uint8_t NRF24_IsPayloadAvailable(void);
uint8_t NRF24_ReadPayload(uint8_t *payload, uint8_t length);

/* P64 explicit TDD downlink. These APIs leave the proven P60 4-byte command
 * receiver/static-payload configuration untouched. Runtime mode transitions
 * are non-blocking and return to PRX before reporting completion. */
uint8_t NRF24_AsyncTxStart(const uint8_t *payload, uint8_t length);
void NRF24_AsyncTxService(void);
uint8_t NRF24_AsyncTxIsBusy(void);
NRF24_AsyncTxResult_t NRF24_AsyncTxTakeResult(void);

uint8_t NRF24_IsConnected(void);
NRF24_Data_t NRF24_GetData(void);

#endif
