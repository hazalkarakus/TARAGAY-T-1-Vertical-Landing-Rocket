#include "Modules/NRF24/nrf24.h"

#include "Common/app_config.h"
#include "Platform/board_handles.h"
#include "Platform/board_pins.h"
#include "Services/Timebase/timebase.h"

#include <stddef.h>
#include <string.h>

#define NRF24_CMD_R_REGISTER       0x00U
#define NRF24_CMD_W_REGISTER       0x20U
#define NRF24_CMD_R_RX_PAYLOAD     0x61U
#define NRF24_CMD_W_TX_PAYLOAD     0xA0U
#define NRF24_MAX_PAYLOAD_SIZE      32U
#define NRF24_CMD_FLUSH_TX         0xE1U
#define NRF24_CMD_FLUSH_RX         0xE2U
#define NRF24_CMD_NOP              0xFFU

#define NRF24_REG_CONFIG           0x00U
#define NRF24_REG_EN_AA            0x01U
#define NRF24_REG_EN_RXADDR        0x02U
#define NRF24_REG_SETUP_AW         0x03U
#define NRF24_REG_SETUP_RETR       0x04U
#define NRF24_REG_RF_CH            0x05U
#define NRF24_REG_RF_SETUP         0x06U
#define NRF24_REG_STATUS           0x07U
#define NRF24_REG_RX_ADDR_P0       0x0AU
#define NRF24_REG_TX_ADDR          0x10U
#define NRF24_REG_RX_PW_P0         0x11U
#define NRF24_REG_FIFO_STATUS      0x17U
#define NRF24_REG_DYNPD            0x1CU
#define NRF24_REG_FEATURE          0x1DU

#define NRF24_STATUS_RX_DR         0x40U
#define NRF24_STATUS_TX_DS         0x20U
#define NRF24_STATUS_MAX_RT        0x10U
#define NRF24_FIFO_RX_EMPTY        0x01U

/* Exactly matches the uploaded F103 transmitter. */
#define NRF24_CONFIG_POWERDOWN     0x0CU
#define NRF24_CONFIG_TX            0x0EU
#define NRF24_CONFIG_RX            0x0FU
#define NRF24_RF_SETUP_250KBPS_0DBM 0x26U /* RF_DR_LOW=1, RF_DR_HIGH=0, RF_PWR=0 dBm */
#define NRF24_SETUP_RETR_VALUE     0x2FU
#define NRF24_SPI_TIMEOUT_MS       5U
#define NRF24_RECHECK_MS           1000UL
#define NRF24_ASYNC_SETTLE_US       250UL
#define NRF24_ASYNC_TIMEOUT_MS      20UL

#define NRF24_CE_LOW()   HAL_GPIO_WritePin(BOARD_NRF24_CE_PORT, BOARD_NRF24_CE_PIN, GPIO_PIN_RESET)
#define NRF24_CE_HIGH()  HAL_GPIO_WritePin(BOARD_NRF24_CE_PORT, BOARD_NRF24_CE_PIN, GPIO_PIN_SET)
#define NRF24_CSN_LOW()  HAL_GPIO_WritePin(BOARD_NRF24_CSN_PORT, BOARD_NRF24_CSN_PIN, GPIO_PIN_RESET)
#define NRF24_CSN_HIGH() HAL_GPIO_WritePin(BOARD_NRF24_CSN_PORT, BOARD_NRF24_CSN_PIN, GPIO_PIN_SET)

static NRF24_Data_t nrf24_data;
static const uint8_t nrf24_address[5] = {'T', 'G', 'Y', '0', '1'};
static uint32_t nrf24_last_recheck_ms = 0UL;

typedef enum
{
    NRF24_ASYNC_IDLE = 0,
    NRF24_ASYNC_TX_SETTLE,
    NRF24_ASYNC_TX_WAIT,
    NRF24_ASYNC_RX_SETTLE
} NRF24_AsyncState_t;

static NRF24_AsyncState_t nrf24_async_state = NRF24_ASYNC_IDLE;
static uint8_t nrf24_async_payload[NRF24_MAX_PAYLOAD_SIZE];
static uint8_t nrf24_async_length = 0U;
static uint32_t nrf24_async_state_us = 0UL;
static uint32_t nrf24_async_tx_start_ms = 0UL;
static NRF24_AsyncTxResult_t nrf24_async_pending_result = NRF24_ASYNC_TX_RESULT_NONE;
static NRF24_AsyncTxResult_t nrf24_async_finish_result = NRF24_ASYNC_TX_RESULT_NONE;

/* CubeIDE Live Expressions: P64 explicit downlink diagnostics. */
volatile uint8_t nrf24_async_state_live = 0U;
volatile uint8_t nrf24_async_length_live = 0U;
volatile uint32_t nrf24_async_start_count = 0UL;
volatile uint32_t nrf24_async_success_count = 0UL;
volatile uint32_t nrf24_async_fail_count = 0UL;
volatile uint32_t nrf24_async_busy_reject_count = 0UL;

/* CubeIDE Live Expressions */
volatile uint8_t nrf24_initialized = 0U;
volatile uint8_t nrf24_connected = 0U;
volatile uint8_t nrf24_mode = 0U;
volatile uint8_t nrf24_status_reg = 0U;
volatile uint8_t nrf24_config_reg = 0U;
volatile uint8_t nrf24_rf_ch_reg = 0U;
volatile uint8_t nrf24_rf_setup_reg = 0U;
volatile uint8_t nrf24_fifo_status_reg = 0U;
volatile uint32_t nrf24_tx_count = 0UL;
volatile uint32_t nrf24_tx_fail_count = 0UL;
volatile uint32_t nrf24_rx_count = 0UL;
volatile uint32_t nrf24_error_count = 0UL;
volatile uint8_t nrf24_last_rx_0 = 0U;
volatile uint8_t nrf24_last_rx_1 = 0U;
volatile uint8_t nrf24_last_rx_2 = 0U;
volatile uint8_t nrf24_last_rx_3 = 0U;
volatile uint8_t nrf24_addr_readback_0 = 0U;
volatile uint8_t nrf24_addr_readback_1 = 0U;
volatile uint8_t nrf24_addr_readback_2 = 0U;
volatile uint8_t nrf24_addr_readback_3 = 0U;
volatile uint8_t nrf24_addr_readback_4 = 0U;
volatile uint32_t nrf24_reinit_count = 0UL;

static void NRF24_UpdateLiveDebug(void)
{
    nrf24_initialized = nrf24_data.initialized;
    nrf24_connected = nrf24_data.connected;
    nrf24_mode = (uint8_t)nrf24_data.mode;
    nrf24_status_reg = nrf24_data.status_reg;
    nrf24_config_reg = nrf24_data.config_reg;
    nrf24_rf_ch_reg = nrf24_data.rf_ch_reg;
    nrf24_rf_setup_reg = nrf24_data.rf_setup_reg;
    nrf24_fifo_status_reg = nrf24_data.fifo_status_reg;
    nrf24_tx_count = nrf24_data.tx_count;
    nrf24_tx_fail_count = nrf24_data.tx_fail_count;
    nrf24_rx_count = nrf24_data.rx_count;
    nrf24_error_count = nrf24_data.error_count;
}

static HAL_StatusTypeDef NRF24_SPITransfer(const uint8_t *tx, uint8_t *rx, uint16_t size)
{
    HAL_StatusTypeDef st;

    if ((tx == NULL) || (rx == NULL) || (size == 0U))
    {
        nrf24_data.error_count++;
        return HAL_ERROR;
    }

    NRF24_CSN_LOW();
    st = HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)(uintptr_t)tx, rx, size, NRF24_SPI_TIMEOUT_MS);
    NRF24_CSN_HIGH();

    if (st != HAL_OK)
    {
        nrf24_data.error_count++;
    }

    return st;
}

static uint8_t NRF24_ReadReg(uint8_t reg)
{
    uint8_t tx[2] = {(uint8_t)(NRF24_CMD_R_REGISTER | (reg & 0x1FU)), NRF24_CMD_NOP};
    uint8_t rx[2] = {0U, 0U};

    if (NRF24_SPITransfer(tx, rx, 2U) != HAL_OK)
    {
        return 0U;
    }

    nrf24_data.status_reg = rx[0];
    return rx[1];
}

static void NRF24_ReadRegMulti(uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t tx[6] = {0U};
    uint8_t rx[6] = {0U};

    if ((data == NULL) || (size == 0U) || (size > 5U))
    {
        nrf24_data.error_count++;
        return;
    }

    tx[0] = (uint8_t)(NRF24_CMD_R_REGISTER | (reg & 0x1FU));
    for (uint8_t i = 1U; i <= size; i++)
    {
        tx[i] = NRF24_CMD_NOP;
    }

    if (NRF24_SPITransfer(tx, rx, (uint16_t)(size + 1U)) != HAL_OK)
    {
        return;
    }

    nrf24_data.status_reg = rx[0];
    memcpy(data, &rx[1], size);
}

static void NRF24_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(NRF24_CMD_W_REGISTER | (reg & 0x1FU)), value};
    uint8_t rx[2] = {0U, 0U};

    if (NRF24_SPITransfer(tx, rx, 2U) == HAL_OK)
    {
        nrf24_data.status_reg = rx[0];
    }
}

static void NRF24_WriteRegMulti(uint8_t reg, const uint8_t *data, uint8_t size)
{
    uint8_t tx[6] = {0U};
    uint8_t rx[6] = {0U};

    if ((data == NULL) || (size == 0U) || (size > 5U))
    {
        nrf24_data.error_count++;
        return;
    }

    tx[0] = (uint8_t)(NRF24_CMD_W_REGISTER | (reg & 0x1FU));
    memcpy(&tx[1], data, size);

    if (NRF24_SPITransfer(tx, rx, (uint16_t)(size + 1U)) == HAL_OK)
    {
        nrf24_data.status_reg = rx[0];
    }
}

static void NRF24_SendCmd(uint8_t cmd)
{
    uint8_t tx[1] = {cmd};
    uint8_t rx[1] = {0U};

    if (NRF24_SPITransfer(tx, rx, 1U) == HAL_OK)
    {
        nrf24_data.status_reg = rx[0];
    }
}

static uint8_t NRF24_GetStatus(void)
{
    uint8_t tx[1] = {NRF24_CMD_NOP};
    uint8_t rx[1] = {0U};

    if (NRF24_SPITransfer(tx, rx, 1U) == HAL_OK)
    {
        nrf24_data.status_reg = rx[0];
    }

    return nrf24_data.status_reg;
}

static void NRF24_ClearIRQ(void)
{
    NRF24_WriteReg(NRF24_REG_STATUS,
                   (uint8_t)(NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT));
}

static uint8_t NRF24_VerifyConfiguration(void)
{
    uint8_t address[5] = {0U};
    uint8_t ok = 1U;

    nrf24_data.status_reg = NRF24_GetStatus();
    nrf24_data.config_reg = NRF24_ReadReg(NRF24_REG_CONFIG);
    nrf24_data.rf_ch_reg = NRF24_ReadReg(NRF24_REG_RF_CH);
    nrf24_data.rf_setup_reg = NRF24_ReadReg(NRF24_REG_RF_SETUP);
    nrf24_data.fifo_status_reg = NRF24_ReadReg(NRF24_REG_FIFO_STATUS);
    NRF24_ReadRegMulti(NRF24_REG_RX_ADDR_P0, address, 5U);

    nrf24_addr_readback_0 = address[0];
    nrf24_addr_readback_1 = address[1];
    nrf24_addr_readback_2 = address[2];
    nrf24_addr_readback_3 = address[3];
    nrf24_addr_readback_4 = address[4];

    if (nrf24_data.rf_ch_reg != APP_NRF24_CHANNEL) ok = 0U;
    if (nrf24_data.rf_setup_reg != NRF24_RF_SETUP_250KBPS_0DBM) ok = 0U;
    if (NRF24_ReadReg(NRF24_REG_SETUP_AW) != 0x03U) ok = 0U;
    if (NRF24_ReadReg(NRF24_REG_RX_PW_P0) != APP_NRF24_PAYLOAD_SIZE) ok = 0U;
    if (NRF24_ReadReg(NRF24_REG_EN_AA) != 0x00U) ok = 0U;
    if (NRF24_ReadReg(NRF24_REG_EN_RXADDR) != 0x01U) ok = 0U;
    if (memcmp(address, nrf24_address, 5U) != 0) ok = 0U;

    nrf24_data.connected = ok;
    NRF24_UpdateLiveDebug();
    return ok;
}

static void NRF24_ConfigureReceiver(void)
{
    NRF24_CE_LOW();
    NRF24_CSN_HIGH();

    NRF24_WriteReg(NRF24_REG_CONFIG, NRF24_CONFIG_POWERDOWN);
    NRF24_WriteReg(NRF24_REG_EN_AA, 0x00U); /* P64: explicit TDD, no hardware Auto-ACK */
    NRF24_WriteReg(NRF24_REG_EN_RXADDR, 0x01U);
    NRF24_WriteReg(NRF24_REG_SETUP_AW, 0x03U);
    NRF24_WriteReg(NRF24_REG_SETUP_RETR, 0x00U); /* no hardware retransmit; app heartbeat retries */
    NRF24_WriteReg(NRF24_REG_RF_CH, APP_NRF24_CHANNEL);
    NRF24_WriteReg(NRF24_REG_RF_SETUP, NRF24_RF_SETUP_250KBPS_0DBM);
    NRF24_WriteReg(NRF24_REG_RX_PW_P0, APP_NRF24_PAYLOAD_SIZE);
    NRF24_WriteReg(NRF24_REG_DYNPD, 0x00U);
    NRF24_WriteReg(NRF24_REG_FEATURE, 0x00U);
    NRF24_WriteRegMulti(NRF24_REG_RX_ADDR_P0, nrf24_address, 5U);
    NRF24_WriteRegMulti(NRF24_REG_TX_ADDR, nrf24_address, 5U);

    NRF24_ClearIRQ();
    NRF24_SendCmd(NRF24_CMD_FLUSH_RX);
    NRF24_SendCmd(NRF24_CMD_FLUSH_TX);
}

static void NRF24_AsyncBeginReturnToRx(NRF24_AsyncTxResult_t result)
{
    NRF24_CE_LOW();
    NRF24_WriteRegMulti(NRF24_REG_RX_ADDR_P0, nrf24_address, 5U);
    NRF24_WriteReg(NRF24_REG_RX_PW_P0, APP_NRF24_PAYLOAD_SIZE);
    NRF24_ClearIRQ();
    NRF24_WriteReg(NRF24_REG_CONFIG, NRF24_CONFIG_RX);
    nrf24_data.mode = NRF24_MODE_STANDBY;
    nrf24_async_pending_result = result;
    nrf24_async_state = NRF24_ASYNC_RX_SETTLE;
    nrf24_async_state_us = micros();
    nrf24_async_state_live = (uint8_t)nrf24_async_state;
    NRF24_UpdateLiveDebug();
}

static uint8_t NRF24_AsyncLoadAndPulse(void)
{
    uint8_t tx[NRF24_MAX_PAYLOAD_SIZE + 1U] = {0U};
    uint8_t rx[NRF24_MAX_PAYLOAD_SIZE + 1U] = {0U};

    NRF24_SendCmd(NRF24_CMD_FLUSH_TX);
    NRF24_WriteReg(NRF24_REG_STATUS,
                   (uint8_t)(NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT));

    tx[0] = NRF24_CMD_W_TX_PAYLOAD;
    memcpy(&tx[1], nrf24_async_payload, nrf24_async_length);

    if (NRF24_SPITransfer(tx, rx, (uint16_t)(nrf24_async_length + 1U)) != HAL_OK)
    {
        return 0U;
    }

    nrf24_data.status_reg = rx[0];
    NRF24_CE_HIGH();
    /* >10 us CE pulse without scheduler-scale delay. */
    for (volatile uint32_t i = 0UL; i < 2200UL; i++) { __NOP(); }
    NRF24_CE_LOW();
    return 1U;
}

void NRF24_Init(void)
{
    memset(&nrf24_data, 0, sizeof(nrf24_data));
    nrf24_data.mode = NRF24_MODE_POWER_DOWN;
    nrf24_async_state = NRF24_ASYNC_IDLE;
    nrf24_async_length = 0U;
    nrf24_async_pending_result = NRF24_ASYNC_TX_RESULT_NONE;
    nrf24_async_finish_result = NRF24_ASYNC_TX_RESULT_NONE;
    nrf24_async_state_live = 0U;
    nrf24_async_length_live = 0U;
    nrf24_async_start_count = 0UL;
    nrf24_async_success_count = 0UL;
    nrf24_async_fail_count = 0UL;
    nrf24_async_busy_reject_count = 0UL;

    NRF24_CE_LOW();
    NRF24_CSN_HIGH();

    /* Same power-up margin used by the proven F103 transmitter. */
    HAL_Delay(100U);

    NRF24_ConfigureReceiver();
    nrf24_data.initialized = 1U;
    (void)NRF24_VerifyConfiguration();
    NRF24_SetRxMode();

    nrf24_last_recheck_ms = millis();
    NRF24_UpdateLiveDebug();
}

void NRF24_SetRxMode(void)
{
    if (nrf24_data.initialized == 0U)
    {
        return;
    }

    NRF24_CE_LOW();
    NRF24_WriteRegMulti(NRF24_REG_RX_ADDR_P0, nrf24_address, 5U);
    NRF24_WriteReg(NRF24_REG_RX_PW_P0, APP_NRF24_PAYLOAD_SIZE);
    NRF24_ClearIRQ();
    NRF24_WriteReg(NRF24_REG_CONFIG, NRF24_CONFIG_RX);
    /* R8R35R3R7R1: no scheduler-scale blocking delay. RX settle is
     * handled naturally by the radio while CE is asserted; async TX/RX
     * transitions use NRF24_ASYNC_SETTLE_US deadlines. */
    NRF24_CE_HIGH();
    nrf24_data.mode = NRF24_MODE_RX;
    nrf24_data.config_reg = NRF24_ReadReg(NRF24_REG_CONFIG);
    NRF24_UpdateLiveDebug();
}

void NRF24_SetTxMode(void)
{
    if (nrf24_data.initialized == 0U)
    {
        return;
    }

    NRF24_CE_LOW();
    NRF24_WriteRegMulti(NRF24_REG_TX_ADDR, nrf24_address, 5U);
    NRF24_WriteRegMulti(NRF24_REG_RX_ADDR_P0, nrf24_address, 5U);
    NRF24_ClearIRQ();
    NRF24_WriteReg(NRF24_REG_CONFIG, NRF24_CONFIG_TX);
    /* R8R35R3R7R1: no HAL_Delay here. The active telemetry path uses
     * NRF24_AsyncTxStart/Service and its explicit non-blocking settle state. */
    nrf24_data.mode = NRF24_MODE_TX;
    NRF24_UpdateLiveDebug();
}

uint8_t NRF24_IsPayloadAvailable(void)
{
    uint8_t status;
    uint8_t fifo;

    if ((nrf24_data.initialized == 0U) || (nrf24_data.mode != NRF24_MODE_RX))
    {
        return 0U;
    }

    status = NRF24_GetStatus();
    fifo = NRF24_ReadReg(NRF24_REG_FIFO_STATUS);
    nrf24_data.fifo_status_reg = fifo;

    return (((status & NRF24_STATUS_RX_DR) != 0U) ||
            ((fifo & NRF24_FIFO_RX_EMPTY) == 0U)) ? 1U : 0U;
}

uint8_t NRF24_ReadPayload(uint8_t *payload, uint8_t length)
{
    uint8_t tx[APP_NRF24_PAYLOAD_SIZE + 1U] = {0U};
    uint8_t rx[APP_NRF24_PAYLOAD_SIZE + 1U] = {0U};

    if ((payload == NULL) || (length != APP_NRF24_PAYLOAD_SIZE))
    {
        nrf24_data.error_count++;
        return 0U;
    }

    tx[0] = NRF24_CMD_R_RX_PAYLOAD;
    for (uint8_t i = 1U; i <= APP_NRF24_PAYLOAD_SIZE; i++)
    {
        tx[i] = NRF24_CMD_NOP;
    }

    if (NRF24_SPITransfer(tx, rx, (uint16_t)(APP_NRF24_PAYLOAD_SIZE + 1U)) != HAL_OK)
    {
        return 0U;
    }

    memcpy(payload, &rx[1], APP_NRF24_PAYLOAD_SIZE);
    nrf24_data.status_reg = rx[0];
    NRF24_WriteReg(NRF24_REG_STATUS, NRF24_STATUS_RX_DR);
    nrf24_data.rx_count++;

    nrf24_last_rx_0 = payload[0];
    nrf24_last_rx_1 = payload[1];
    nrf24_last_rx_2 = payload[2];
    nrf24_last_rx_3 = payload[3];

    NRF24_UpdateLiveDebug();
    return 1U;
}

uint8_t NRF24_SendPayload(const uint8_t *payload, uint8_t length)
{
    /* R8R35R3R7R1: legacy synchronous TX used a millisecond polling loop and
     * could block App_Run(). Keep the public symbol for source compatibility,
     * but make it a non-blocking enqueue into the already-proven async TX
     * state machine. Return 1 means accepted, not RF delivery confirmation.
     * Delivery result remains available through NRF24_AsyncTxTakeResult(). */
    return NRF24_AsyncTxStart(payload, length);
}

uint8_t NRF24_AsyncTxStart(const uint8_t *payload, uint8_t length)
{
    if ((payload == NULL) || (length == 0U) || (length > NRF24_MAX_PAYLOAD_SIZE) ||
        (nrf24_data.initialized == 0U))
    {
        nrf24_data.error_count++;
        return 0U;
    }

    if (nrf24_async_state != NRF24_ASYNC_IDLE)
    {
        nrf24_async_busy_reject_count++;
        return 0U;
    }

    memcpy(nrf24_async_payload, payload, length);
    nrf24_async_length = length;
    nrf24_async_length_live = length;
    nrf24_async_pending_result = NRF24_ASYNC_TX_RESULT_NONE;
    nrf24_async_finish_result = NRF24_ASYNC_TX_RESULT_NONE;

    /* Leave the static P60 radio setup intact. Only switch role. */
    NRF24_CE_LOW();
    NRF24_WriteRegMulti(NRF24_REG_TX_ADDR, nrf24_address, 5U);
    NRF24_WriteRegMulti(NRF24_REG_RX_ADDR_P0, nrf24_address, 5U);
    NRF24_ClearIRQ();
    NRF24_WriteReg(NRF24_REG_CONFIG, NRF24_CONFIG_TX);
    nrf24_data.mode = NRF24_MODE_TX;
    nrf24_async_state = NRF24_ASYNC_TX_SETTLE;
    nrf24_async_state_us = micros();
    nrf24_async_state_live = (uint8_t)nrf24_async_state;
    nrf24_async_start_count++;
    NRF24_UpdateLiveDebug();
    return 1U;
}

void NRF24_AsyncTxService(void)
{
    uint32_t now_us;

    if (nrf24_async_state == NRF24_ASYNC_IDLE)
    {
        return;
    }

    now_us = micros();

    if (nrf24_async_state == NRF24_ASYNC_TX_SETTLE)
    {
        if ((uint32_t)(now_us - nrf24_async_state_us) < NRF24_ASYNC_SETTLE_US)
        {
            return;
        }

        if (NRF24_AsyncLoadAndPulse() == 0U)
        {
            nrf24_data.tx_fail_count++;
            nrf24_async_fail_count++;
            NRF24_AsyncBeginReturnToRx(NRF24_ASYNC_TX_RESULT_FAIL);
            return;
        }

        nrf24_async_tx_start_ms = millis();
        nrf24_async_state = NRF24_ASYNC_TX_WAIT;
        nrf24_async_state_live = (uint8_t)nrf24_async_state;
        return;
    }

    if (nrf24_async_state == NRF24_ASYNC_TX_WAIT)
    {
        uint8_t status = NRF24_GetStatus();

        if ((status & NRF24_STATUS_TX_DS) != 0U)
        {
            NRF24_WriteReg(NRF24_REG_STATUS, NRF24_STATUS_TX_DS);
            nrf24_data.tx_count++;
            nrf24_async_success_count++;
            NRF24_AsyncBeginReturnToRx(NRF24_ASYNC_TX_RESULT_SUCCESS);
            return;
        }

        if ((status & NRF24_STATUS_MAX_RT) != 0U)
        {
            NRF24_WriteReg(NRF24_REG_STATUS, NRF24_STATUS_MAX_RT);
            NRF24_SendCmd(NRF24_CMD_FLUSH_TX);
            nrf24_data.tx_fail_count++;
            nrf24_async_fail_count++;
            NRF24_AsyncBeginReturnToRx(NRF24_ASYNC_TX_RESULT_FAIL);
            return;
        }

        if ((uint32_t)(millis() - nrf24_async_tx_start_ms) >= NRF24_ASYNC_TIMEOUT_MS)
        {
            NRF24_SendCmd(NRF24_CMD_FLUSH_TX);
            nrf24_data.tx_fail_count++;
            nrf24_async_fail_count++;
            NRF24_AsyncBeginReturnToRx(NRF24_ASYNC_TX_RESULT_FAIL);
        }
        return;
    }

    /* RX settle: no HAL_Delay in the runtime downlink path. */
    if (nrf24_async_state == NRF24_ASYNC_RX_SETTLE)
    {
        if ((uint32_t)(now_us - nrf24_async_state_us) < NRF24_ASYNC_SETTLE_US)
        {
            return;
        }

        NRF24_CE_HIGH();
        nrf24_data.mode = NRF24_MODE_RX;
        nrf24_data.config_reg = NRF24_ReadReg(NRF24_REG_CONFIG);
        nrf24_async_finish_result = nrf24_async_pending_result;
        nrf24_async_pending_result = NRF24_ASYNC_TX_RESULT_NONE;
        nrf24_async_state = NRF24_ASYNC_IDLE;
        nrf24_async_state_live = 0U;
        NRF24_UpdateLiveDebug();
    }
}

uint8_t NRF24_AsyncTxIsBusy(void)
{
    return (nrf24_async_state != NRF24_ASYNC_IDLE) ? 1U : 0U;
}

NRF24_AsyncTxResult_t NRF24_AsyncTxTakeResult(void)
{
    NRF24_AsyncTxResult_t result = nrf24_async_finish_result;
    nrf24_async_finish_result = NRF24_ASYNC_TX_RESULT_NONE;
    return result;
}

void NRF24_Update(void)
{
    uint32_t now = millis();

    if (nrf24_data.initialized == 0U)
    {
        return;
    }

    if (nrf24_async_state != NRF24_ASYNC_IDLE)
    {
        return;
    }

    if ((uint32_t)(now - nrf24_last_recheck_ms) >= NRF24_RECHECK_MS)
    {
        nrf24_last_recheck_ms = now;

        if (NRF24_VerifyConfiguration() == 0U)
        {
            nrf24_reinit_count++;
            NRF24_CE_LOW();
            NRF24_ConfigureReceiver();
            (void)NRF24_VerifyConfiguration();
            NRF24_SetRxMode();
        }
        else if (nrf24_data.mode != NRF24_MODE_RX)
        {
            NRF24_SetRxMode();
        }
    }
}

uint8_t NRF24_IsConnected(void)
{
    return nrf24_data.connected;
}

NRF24_Data_t NRF24_GetData(void)
{
    return nrf24_data;
}
