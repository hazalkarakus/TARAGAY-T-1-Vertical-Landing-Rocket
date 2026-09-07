/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    bsp_driver_sd.c for F4 (based on stm324x9i_eval_sd.c)
 * @brief   This file includes a generic uSD card driver.
 *          To be completed by the user according to the board used for the project.
 * @note    Some functions generated as weak: they can be overridden by
 *          - code in user files
 *          - or BSP code from the FW pack files
 *          if such files are added to the generated project (by the user).
 ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
 ******************************************************************************
 */
/* USER CODE END Header */

#ifdef OLD_API
/* kept to avoid issue when migrating old projects. */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
#else
/* USER CODE BEGIN FirstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END FirstSection */
/* Includes ------------------------------------------------------------------*/
#include "bsp_driver_sd.h"
#include "Common/app_config.h"

/* Extern variables ---------------------------------------------------------*/

extern SD_HandleTypeDef hsd;

/* -------------------------------------------------------------------------- */
/* SDIO low-level diagnostics - Live Expressions                              */
/* -------------------------------------------------------------------------- */

volatile uint32_t sd_bsp_init_call_count = 0UL;
volatile uint8_t  sd_bsp_detect_value = 0U;
volatile uint8_t  sd_bsp_hal_init_status = 0xFFU;
volatile uint8_t  sd_bsp_wide_bus_status = 0xFFU;

volatile uint32_t sd_bsp_hal_error_code = 0UL;
volatile uint32_t sd_bsp_hal_state = 0UL;
volatile uint32_t sd_bsp_hal_card_state = 0xFFFFFFFFUL;

volatile uint32_t sd_bsp_card_type = 0UL;
volatile uint32_t sd_bsp_card_version = 0UL;
volatile uint32_t sd_bsp_card_rca = 0UL;
volatile uint32_t sd_bsp_card_class = 0UL;

volatile uint32_t sd_bsp_sdio_power = 0UL;
volatile uint32_t sd_bsp_sdio_clkcr = 0UL;
volatile uint32_t sd_bsp_sdio_sta = 0UL;
volatile uint32_t sd_bsp_sdio_resp1 = 0UL;

/* V47 SD command-path recovery diagnostics */
volatile uint32_t sd_v47_init_attempt_count = 0UL;
volatile uint32_t sd_v47_host_reset_count = 0UL;
volatile uint8_t  sd_v47_cmd_idle_level = 0U;
volatile uint8_t  sd_v47_d0_idle_level = 0U;
volatile uint8_t  sd_v47_d1_idle_level = 0U;
volatile uint8_t  sd_v47_d2_idle_level = 0U;
volatile uint8_t  sd_v47_d3_idle_level = 0U;
volatile uint32_t sd_v47_last_attempt_error = 0UL;
volatile uint8_t  sd_v47_recovered = 0U;
volatile uint32_t sd_p37_soft_recovery_count = 0UL;
volatile uint32_t sd_p37_runtime_reinit_count = 0UL;
volatile uint32_t sd_p37_runtime_reinit_success_count = 0UL;
volatile uint32_t sd_p37_runtime_reinit_failure_count = 0UL;
volatile uint32_t sd_p37_last_runtime_error = 0UL;

static void BSP_SD_V47_SampleIdleLines(void)
{
  sd_v47_cmd_idle_level = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_SET) ? 1U : 0U;
  sd_v47_d0_idle_level  = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET) ? 1U : 0U;
  sd_v47_d1_idle_level  = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET) ? 1U : 0U;
  sd_v47_d2_idle_level  = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_SET) ? 1U : 0U;
  sd_v47_d3_idle_level  = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_SET) ? 1U : 0U;
}

static void BSP_SD_UpdateDiagnostics(uint8_t read_card_state);

static HAL_StatusTypeDef BSP_SD_V47_ResetAndInitHost(void)
{
  HAL_StatusTypeDef st;

  BSP_SD_V47_SampleIdleLines();

  /* A debugger/firmware reload does not remove power from the SD breakout.
   * Fully de-initialize the STM32 SDIO host before retrying the card sequence. */
  (void)HAL_SD_DeInit(&hsd);
  sd_v47_host_reset_count++;

  __HAL_RCC_SDIO_FORCE_RESET();
  __NOP();
  __NOP();
  __HAL_RCC_SDIO_RELEASE_RESET();

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = APP_SDIO_RUNTIME_CLOCK_DIV;
  hsd.ErrorCode = HAL_SD_ERROR_NONE;
  hsd.State = HAL_SD_STATE_RESET;

  HAL_Delay(50U);
  sd_v47_init_attempt_count++;
  st = HAL_SD_Init(&hsd);
  sd_v47_last_attempt_error = hsd.ErrorCode;
  BSP_SD_UpdateDiagnostics(0U);

  return st;
}

/* -------------------------------------------------------------------------- */
/* SDIO DMA transfer diagnostics - Live Expressions                           */
/* -------------------------------------------------------------------------- */

volatile uint8_t sd_dma_rx_complete = 0U;
volatile uint8_t sd_dma_tx_complete = 0U;
volatile uint8_t sd_dma_transfer_error = 0U;
volatile uint8_t sd_dma_transfer_active = 0U;

volatile uint32_t sd_dma_read_start_count = 0UL;
volatile uint32_t sd_dma_write_start_count = 0UL;
volatile uint32_t sd_dma_read_complete_count = 0UL;
volatile uint32_t sd_dma_write_complete_count = 0UL;

volatile uint32_t sd_dma_start_error_count = 0UL;
volatile uint32_t sd_dma_transfer_error_count = 0UL;
volatile uint32_t sd_dma_timeout_count = 0UL;
volatile uint32_t sd_dma_abort_count = 0UL;

volatile uint32_t sd_dma_transfer_start_ms = 0UL;
volatile uint32_t sd_dma_last_transfer_ms = 0UL;
volatile uint32_t sd_dma_max_transfer_ms = 0UL;
volatile uint32_t sd_dma_last_hal_error = 0UL;


/* R8R35R3R7 SDIO motor-EMI start-failure snapshot diagnostics.
 * Read-only telemetry only: no transfer/retry policy is changed here. */
volatile uint32_t sd_r7_write_start_fail_count = 0UL;
volatile uint32_t sd_r7_first_fail_time_ms = 0UL;
volatile uint32_t sd_r7_first_fail_hal_status = 0xFFFFFFFFUL;
volatile uint32_t sd_r7_first_fail_hsd_state = 0xFFFFFFFFUL;
volatile uint32_t sd_r7_first_fail_hsd_error = 0UL;
volatile uint32_t sd_r7_first_fail_hsd_context = 0UL;
volatile uint32_t sd_r7_first_fail_dma_state = 0xFFFFFFFFUL;
volatile uint32_t sd_r7_first_fail_dma_error = 0UL;
volatile uint32_t sd_r7_first_fail_sdio_sta = 0UL;
volatile uint32_t sd_r7_first_fail_sdio_dctrl = 0UL;
volatile uint32_t sd_r7_first_fail_sdio_dcount = 0UL;
volatile uint32_t sd_r7_last_fail_time_ms = 0UL;
volatile uint32_t sd_r7_last_fail_hal_status = 0xFFFFFFFFUL;
volatile uint32_t sd_r7_last_fail_hsd_state = 0xFFFFFFFFUL;
volatile uint32_t sd_r7_last_fail_hsd_error = 0UL;
volatile uint32_t sd_r7_last_fail_hsd_context = 0UL;
volatile uint32_t sd_r7_last_fail_dma_state = 0xFFFFFFFFUL;
volatile uint32_t sd_r7_last_fail_dma_error = 0UL;
volatile uint32_t sd_r7_last_fail_sdio_sta = 0UL;
volatile uint32_t sd_r7_last_fail_sdio_dctrl = 0UL;
volatile uint32_t sd_r7_last_fail_sdio_dcount = 0UL;

static void BSP_SD_R7_CaptureWriteStartFailure(HAL_StatusTypeDef hal_status)
{
  uint32_t dma_state = 0xFFFFFFFFUL;
  uint32_t dma_error = 0xFFFFFFFFUL;
  uint32_t now_ms = HAL_GetTick();
  uint32_t hsd_state = (uint32_t)hsd.State;
  uint32_t hsd_error = hsd.ErrorCode;
  uint32_t hsd_context = hsd.Context;
  uint32_t sta = SDIO->STA;
  uint32_t dctrl = SDIO->DCTRL;
  uint32_t dcount = SDIO->DCOUNT;

  if (hsd.hdmatx != NULL)
  {
    dma_state = (uint32_t)hsd.hdmatx->State;
    dma_error = hsd.hdmatx->ErrorCode;
  }

  sd_r7_write_start_fail_count++;

  sd_r7_last_fail_time_ms = now_ms;
  sd_r7_last_fail_hal_status = (uint32_t)hal_status;
  sd_r7_last_fail_hsd_state = hsd_state;
  sd_r7_last_fail_hsd_error = hsd_error;
  sd_r7_last_fail_hsd_context = hsd_context;
  sd_r7_last_fail_dma_state = dma_state;
  sd_r7_last_fail_dma_error = dma_error;
  sd_r7_last_fail_sdio_sta = sta;
  sd_r7_last_fail_sdio_dctrl = dctrl;
  sd_r7_last_fail_sdio_dcount = dcount;

  if (sd_r7_write_start_fail_count == 1UL)
  {
    sd_r7_first_fail_time_ms = now_ms;
    sd_r7_first_fail_hal_status = (uint32_t)hal_status;
    sd_r7_first_fail_hsd_state = hsd_state;
    sd_r7_first_fail_hsd_error = hsd_error;
    sd_r7_first_fail_hsd_context = hsd_context;
    sd_r7_first_fail_dma_state = dma_state;
    sd_r7_first_fail_dma_error = dma_error;
    sd_r7_first_fail_sdio_sta = sta;
    sd_r7_first_fail_sdio_dctrl = dctrl;
    sd_r7_first_fail_sdio_dcount = dcount;
  }
}

static void BSP_SD_DMA_BeginTransfer(void)
{
  sd_dma_rx_complete = 0U;
  sd_dma_tx_complete = 0U;
  sd_dma_transfer_error = 0U;
  sd_dma_transfer_active = 1U;
  sd_dma_transfer_start_ms = HAL_GetTick();
  sd_dma_last_hal_error = 0UL;
}

static void BSP_SD_DMA_EndTransfer(void)
{
  uint32_t duration_ms = HAL_GetTick() - sd_dma_transfer_start_ms;

  sd_dma_transfer_active = 0U;
  sd_dma_last_transfer_ms = duration_ms;

  if (duration_ms > sd_dma_max_transfer_ms)
  {
    sd_dma_max_transfer_ms = duration_ms;
  }

  sd_dma_last_hal_error = hsd.ErrorCode;
}

static void BSP_SD_UpdateDiagnostics(uint8_t read_card_state)
{
  sd_bsp_hal_error_code = hsd.ErrorCode;
  sd_bsp_hal_state = (uint32_t)hsd.State;

  if (read_card_state != 0U)
  {
    sd_bsp_hal_card_state = (uint32_t)HAL_SD_GetCardState(&hsd);
    sd_bsp_hal_error_code = hsd.ErrorCode;
  }

  sd_bsp_card_type = hsd.SdCard.CardType;
  sd_bsp_card_version = hsd.SdCard.CardVersion;
  sd_bsp_card_rca = hsd.SdCard.RelCardAdd;
  sd_bsp_card_class = hsd.SdCard.Class;

  sd_bsp_sdio_power = SDIO->POWER;
  sd_bsp_sdio_clkcr = SDIO->CLKCR;
  sd_bsp_sdio_sta = SDIO->STA;
  sd_bsp_sdio_resp1 = SDIO->RESP1;
}

/* USER CODE BEGIN BeforeInitSection */
/* can be used to modify / undefine following code or add code */
/* USER CODE END BeforeInitSection */
/**
  * @brief  Initializes the SD card device.
  * @retval SD status
  */
__weak uint8_t BSP_SD_Init(void)
{
  HAL_StatusTypeDef hal_status = HAL_ERROR;
  uint32_t attempt;

  sd_bsp_init_call_count++;
  sd_bsp_detect_value = BSP_SD_IsDetected();
  sd_bsp_hal_init_status = 0xFFU;
  sd_bsp_wide_bus_status = 0xFFU;
  sd_bsp_hal_card_state = 0xFFFFFFFFUL;
  sd_v47_recovered = 0U;

  BSP_SD_UpdateDiagnostics(0U);

  if (sd_bsp_detect_value != SD_PRESENT)
  {
    return MSD_ERROR;
  }

  /* V47: the V46 trace failed inside HAL_SD_Init with command-response
   * timeout (before FatFS and before 4-bit mode).  Retry only the SDIO/card
   * bring-up after a complete host deinit + peripheral reset. */
  for (attempt = 0U; attempt < 3U; attempt++)
  {
    hal_status = BSP_SD_V47_ResetAndInitHost();
    sd_bsp_hal_init_status = (uint8_t)hal_status;

    if (hal_status == HAL_OK)
    {
      if (attempt != 0U)
      {
        sd_v47_recovered = 1U;
      }
      break;
    }

    /* Leave enough time for a still-powered card to return to idle before the
     * next CMD0/CMD8/ACMD41 sequence. */
    HAL_Delay(100U);
  }

  if (hal_status != HAL_OK)
  {
    BSP_SD_UpdateDiagnostics(0U);
    return MSD_ERROR;
  }

  /* P33: keep the card and host permanently in 1-bit SDIO mode.
   *
   * P32 diagnostics show HAL_SD_Init succeeds and the 4-bit command transition
   * reports HAL_OK, but the first FatFs sector read fails with FR_DISK_ERR.
   * ACMD6 can succeed even when D1-D3 are not electrically reliable, because
   * the command itself travels on CMD. The logger bandwidth is far below what
   * 1-bit SDIO can sustain, so prefer the robust D0-only data path.
   */
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.ErrorCode = HAL_SD_ERROR_NONE;
  sd_bsp_wide_bus_status = 0xFEU; /* forced/reliable 1-bit mode */

  BSP_SD_UpdateDiagnostics(1U);
  return MSD_OK;
}
/* USER CODE BEGIN AfterInitSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END AfterInitSection */

/* USER CODE BEGIN InterruptMode */
/**
  * @brief  Configures Interrupt mode for SD detection pin.
  * @retval Returns 0
  */
__weak uint8_t BSP_SD_ITConfig(void)
{
  /* Code to be updated by the user or replaced by one from the FW pack (in a stmxxxx_sd.c file) */

  return (uint8_t)0;
}

/** @brief  SD detect IT treatment
  */
__weak void BSP_SD_DetectIT(void)
{
  /* Code to be updated by the user or replaced by one from the FW pack (in a stmxxxx_sd.c file) */
}
/* USER CODE END InterruptMode */

/* USER CODE BEGIN BeforeReadBlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeReadBlocksSection */
/**
  * @brief  Reads block(s) from a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @param  Timeout: Timeout for read operation
  * @retval SD status
  */
__weak uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  uint8_t sd_state = MSD_OK;

  if (HAL_SD_ReadBlocks(&hsd, (uint8_t *)pData, ReadAddr, NumOfBlocks, Timeout) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }

  return sd_state;
}

/* USER CODE BEGIN BeforeWriteBlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeWriteBlocksSection */
/**
  * @brief  Writes block(s) to a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write
  * @param  Timeout: Timeout for write operation
  * @retval SD status
  */
__weak uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  uint8_t sd_state = MSD_OK;

  if (HAL_SD_WriteBlocks(&hsd, (uint8_t *)pData, WriteAddr, NumOfBlocks, Timeout) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }

  return sd_state;
}

/* USER CODE BEGIN BeforeReadDMABlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeReadDMABlocksSection */
/**
  * @brief  Reads block(s) from a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @retval SD status
  */
__weak uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
  BSP_SD_DMA_BeginTransfer();
  sd_dma_read_start_count++;

  if (HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t *)pData, ReadAddr, NumOfBlocks) != HAL_OK)
  {
    sd_dma_start_error_count++;
    sd_dma_transfer_error = 1U;
    BSP_SD_DMA_EndTransfer();
    BSP_SD_UpdateDiagnostics(0U);
    return MSD_ERROR;
  }

  return MSD_OK;
}

/* USER CODE BEGIN BeforeWriteDMABlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeWriteDMABlocksSection */
/**
  * @brief  Writes block(s) to a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write
  * @retval SD status
  */
__weak uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
  HAL_StatusTypeDef hal_status;

  BSP_SD_DMA_BeginTransfer();
  sd_dma_write_start_count++;

  hal_status = HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t *)pData, WriteAddr, NumOfBlocks);
  if (hal_status != HAL_OK)
  {
    /* R8R35R3R7: snapshot the exact launch failure before any BSP cleanup
     * changes the observable state. This is diagnostic-only. */
    BSP_SD_R7_CaptureWriteStartFailure(hal_status);
    sd_dma_start_error_count++;
    sd_dma_transfer_error = 1U;
    BSP_SD_DMA_EndTransfer();
    BSP_SD_UpdateDiagnostics(0U);
    return MSD_ERROR;
  }

  return MSD_OK;
}

/* USER CODE BEGIN BeforeEraseSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeEraseSection */
/**
  * @brief  Erases the specified memory area of the given SD card.
  * @param  StartAddr: Start byte address
  * @param  EndAddr: End byte address
  * @retval SD status
  */
__weak uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr)
{
  uint8_t sd_state = MSD_OK;

  if (HAL_SD_Erase(&hsd, StartAddr, EndAddr) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }

  return sd_state;
}

/**
  * @brief  Gets the current SD card data status.
  * @param  None
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  */
__weak uint8_t BSP_SD_GetCardState(void)
{
  return ((HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER ) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}

/**
  * @brief  Get SD information about specific SD card.
  * @param  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
  * @retval None
  */
__weak void BSP_SD_GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo)
{
  /* Get SD card Information */
  HAL_SD_GetCardInfo(&hsd, CardInfo);
}

/* USER CODE BEGIN BeforeCallBacksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeCallBacksSection */
/**
  * @brief SD Abort callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
  BSP_SD_AbortCallback();
}

/**
  * @brief Tx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  BSP_SD_WriteCpltCallback();
}

/**
  * @brief Rx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  BSP_SD_ReadCpltCallback();
}

/**
  * @brief SD error callback
  * @param hsd_handle: SD handle
  * @retval None
  */
void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd_handle)
{
  (void)hsd_handle;

  sd_dma_transfer_error = 1U;
  sd_dma_transfer_error_count++;
  BSP_SD_DMA_EndTransfer();
  BSP_SD_UpdateDiagnostics(0U);
}

/* USER CODE BEGIN CallBacksSection_C */
/**
  * @brief BSP SD Abort callback
  * @retval None
  * @note empty (up to the user to fill it in or to remove it if useless)
  */
__weak void BSP_SD_AbortCallback(void)
{
  sd_dma_abort_count++;
  sd_dma_transfer_error = 1U;
  BSP_SD_DMA_EndTransfer();
  BSP_SD_UpdateDiagnostics(0U);
}

/**
  * @brief BSP Tx Transfer completed callback
  * @retval None
  * @note empty (up to the user to fill it in or to remove it if useless)
  */
__weak void BSP_SD_WriteCpltCallback(void)
{
  sd_dma_tx_complete = 1U;
  sd_dma_write_complete_count++;
  BSP_SD_DMA_EndTransfer();
  BSP_SD_UpdateDiagnostics(0U);
}

/**
  * @brief BSP Rx Transfer completed callback
  * @retval None
  * @note empty (up to the user to fill it in or to remove it if useless)
  */
__weak void BSP_SD_ReadCpltCallback(void)
{
  sd_dma_rx_complete = 1U;
  sd_dma_read_complete_count++;
  BSP_SD_DMA_EndTransfer();
  BSP_SD_UpdateDiagnostics(0U);
}
/* USER CODE END CallBacksSection_C */
#endif

/**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @param  None
 * @retval Returns if SD is detected or not
 */
uint8_t BSP_SD_RuntimeQuiesceNonBlocking(void)
{
  sd_p37_last_runtime_error = hsd.ErrorCode;

  __HAL_SD_DISABLE_IT(&hsd, SDIO_IT_DATAEND | SDIO_IT_DCRCFAIL |
                            SDIO_IT_DTIMEOUT | SDIO_IT_TXUNDERR |
                            SDIO_IT_RXOVERR);
  CLEAR_BIT(hsd.Instance->DCTRL, SDIO_DCTRL_DTEN | SDIO_DCTRL_DMAEN);
  __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_FLAGS);

  if ((hsd.hdmatx != NULL) && (hsd.hdmatx->State == HAL_DMA_STATE_BUSY))
  {
    __HAL_DMA_DISABLE(hsd.hdmatx);
  }
  if ((hsd.hdmarx != NULL) && (hsd.hdmarx->State == HAL_DMA_STATE_BUSY))
  {
    __HAL_DMA_DISABLE(hsd.hdmarx);
  }

  /* Logger will stay stopped after this path, so no HAL-state normalization
   * or polling abort is needed in flight. */
  sd_dma_transfer_active = 0U;
  sd_dma_transfer_error = 0U;
  sd_dma_tx_complete = 0U;
  sd_dma_rx_complete = 0U;
  BSP_SD_UpdateDiagnostics(0U);
  return MSD_OK;
}

uint8_t BSP_SD_RuntimeSoftRecover(void)
{
  sd_p37_last_runtime_error = hsd.ErrorCode;

  __HAL_SD_DISABLE_IT(&hsd, SDIO_IT_DATAEND | SDIO_IT_DCRCFAIL |
                            SDIO_IT_DTIMEOUT | SDIO_IT_TXUNDERR |
                            SDIO_IT_RXOVERR);
  CLEAR_BIT(hsd.Instance->DCTRL, SDIO_DCTRL_DTEN | SDIO_DCTRL_DMAEN);
  __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_FLAGS);

  if ((hsd.hdmatx != NULL) && (hsd.hdmatx->State == HAL_DMA_STATE_BUSY))
  {
    (void)HAL_DMA_Abort(hsd.hdmatx);
  }
  if ((hsd.hdmarx != NULL) && (hsd.hdmarx->State == HAL_DMA_STATE_BUSY))
  {
    (void)HAL_DMA_Abort(hsd.hdmarx);
  }

  hsd.State = HAL_SD_STATE_READY;
  hsd.Context = SD_CONTEXT_NONE;
  hsd.ErrorCode = HAL_SD_ERROR_NONE;
  sd_dma_transfer_active = 0U;
  sd_dma_transfer_error = 0U;
  sd_dma_tx_complete = 0U;
  sd_dma_rx_complete = 0U;
  sd_p37_soft_recovery_count++;
  BSP_SD_UpdateDiagnostics(0U);
  return MSD_OK;
}

uint8_t BSP_SD_RuntimeReinit(void)
{
  uint8_t result;
  sd_p37_runtime_reinit_count++;
  sd_p37_last_runtime_error = hsd.ErrorCode;
  result = BSP_SD_Init();
  if (result == MSD_OK)
  {
    sd_p37_runtime_reinit_success_count++;
  }
  else
  {
    sd_p37_runtime_reinit_failure_count++;
  }
  return result;
}

__weak uint8_t BSP_SD_IsDetected(void)
{
    return SD_PRESENT;
}

/* USER CODE BEGIN AdditionalCode */
/* user code can be inserted here */
/* USER CODE END AdditionalCode */
