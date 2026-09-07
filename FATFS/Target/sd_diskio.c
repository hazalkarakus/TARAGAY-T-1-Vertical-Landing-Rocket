/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @brief   SD Disk I/O driver - SDIO DMA version
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "sdio.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/

#define SD_DEFAULT_BLOCK_SIZE             512U
#define SD_DMA_WAIT_TIMEOUT_MS             5000UL
#define SD_DMA_ALIGNMENT_MASK              0x3UL

/*
 * FatFS can sometimes pass a buffer that is not 32-bit aligned.
 * STM32F4 SDIO DMA is configured as WORD/WORD, so an aligned one-sector
 * scratch buffer is used for those accesses.
 */
static uint32_t sd_dma_scratch[SD_DEFAULT_BLOCK_SIZE / sizeof(uint32_t)]
    __attribute__((aligned(4)));

/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* DMA completion/error flags from bsp_driver_sd.c */
extern volatile uint8_t sd_dma_rx_complete;
extern volatile uint8_t sd_dma_tx_complete;
extern volatile uint8_t sd_dma_transfer_error;
extern volatile uint8_t sd_dma_transfer_active;

extern volatile uint32_t sd_dma_timeout_count;
extern volatile uint32_t sd_dma_last_hal_error;

/* Live Expressions: FatFS DMA layer */
volatile uint32_t sd_diskio_dma_read_call_count = 0UL;
volatile uint32_t sd_diskio_dma_write_call_count = 0UL;
volatile uint32_t sd_diskio_dma_read_sector_count = 0UL;
volatile uint32_t sd_diskio_dma_write_sector_count = 0UL;

volatile uint32_t sd_diskio_unaligned_read_count = 0UL;
volatile uint32_t sd_diskio_unaligned_write_count = 0UL;

volatile uint32_t sd_diskio_dma_wait_timeout_count = 0UL;
volatile uint32_t sd_diskio_dma_result_error_count = 0UL;
volatile uint32_t sd_diskio_card_busy_timeout_count = 0UL;

volatile uint32_t sd_diskio_last_read_duration_ms = 0UL;
volatile uint32_t sd_diskio_last_write_duration_ms = 0UL;
volatile uint32_t sd_diskio_max_read_duration_ms = 0UL;
volatile uint32_t sd_diskio_max_write_duration_ms = 0UL;

/* Private function prototypes -----------------------------------------------*/
static DSTATUS SD_CheckStatus(BYTE lun);
static uint8_t SD_WaitForDMA(volatile uint8_t *complete_flag);
static uint8_t SD_WaitForCardTransfer(void);
static uint8_t SD_ReadAligned(BYTE *buff, DWORD sector, UINT count);
static uint8_t SD_WriteAligned(const BYTE *buff, DWORD sector, UINT count);

DSTATUS SD_initialize(BYTE lun);
DSTATUS SD_status(BYTE lun);
DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count);

#if _USE_WRITE == 1
DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count);
#endif

#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff);
#endif

const Diskio_drvTypeDef SD_Driver =
{
  SD_initialize,
  SD_status,
  SD_read,
#if _USE_WRITE == 1
  SD_write,
#endif
#if _USE_IOCTL == 1
  SD_ioctl,
#endif
};

/* -------------------------------------------------------------------------- */

static DSTATUS SD_CheckStatus(BYTE lun)
{
  (void)lun;

  Stat = STA_NOINIT;

  if (BSP_SD_GetCardState() == MSD_OK)
  {
    Stat &= (DSTATUS)~STA_NOINIT;
  }

  return Stat;
}

/* -------------------------------------------------------------------------- */

static uint8_t SD_WaitForDMA(volatile uint8_t *complete_flag)
{
  uint32_t start_ms = HAL_GetTick();

  while ((*complete_flag == 0U) && (sd_dma_transfer_error == 0U))
  {
    if ((uint32_t)(HAL_GetTick() - start_ms) >= SD_DMA_WAIT_TIMEOUT_MS)
    {
      sd_dma_timeout_count++;
      sd_diskio_dma_wait_timeout_count++;

      (void)HAL_SD_Abort(&hsd);

      sd_dma_transfer_error = 1U;
      sd_dma_last_hal_error = hsd.ErrorCode;
      return 0U;
    }

    /*
     * Interrupts remain enabled. DMA, SDIO, sensor and SysTick interrupts
     * continue running while FatFS waits synchronously.
     */
    __NOP();
  }

  if (sd_dma_transfer_error != 0U)
  {
    sd_diskio_dma_result_error_count++;
    sd_dma_last_hal_error = hsd.ErrorCode;
    return 0U;
  }

  return 1U;
}

/* -------------------------------------------------------------------------- */

static uint8_t SD_WaitForCardTransfer(void)
{
  uint32_t start_ms = HAL_GetTick();

  while (BSP_SD_GetCardState() != MSD_OK)
  {
    if ((uint32_t)(HAL_GetTick() - start_ms) >= SD_DMA_WAIT_TIMEOUT_MS)
    {
      sd_dma_timeout_count++;
      sd_diskio_card_busy_timeout_count++;

      (void)HAL_SD_Abort(&hsd);

      sd_dma_transfer_error = 1U;
      sd_dma_last_hal_error = hsd.ErrorCode;
      return 0U;
    }

    __NOP();
  }

  return 1U;
}

/* -------------------------------------------------------------------------- */

static uint8_t SD_ReadAligned(BYTE *buff, DWORD sector, UINT count)
{
  /*
   * V5 diagnosis/reliability path:
   * FatFs mount and normal file I/O use the blocking BSP call.  This removes
   * the shared SDIO-DMA state machine from the filesystem path while we prove
   * the card/partition/filesystem itself.  The raw-DMA logger is disabled in
   * app_config.h for this build.
   */
  if (BSP_SD_ReadBlocks(
          (uint32_t *)buff,
          (uint32_t)sector,
          (uint32_t)count,
          SD_DMA_WAIT_TIMEOUT_MS) != MSD_OK)
  {
    sd_diskio_dma_result_error_count++;
    return 0U;
  }

  if (SD_WaitForCardTransfer() == 0U)
  {
    return 0U;
  }

  return 1U;
}

/* -------------------------------------------------------------------------- */

static uint8_t SD_WriteAligned(const BYTE *buff, DWORD sector, UINT count)
{
  if (BSP_SD_WriteBlocks(
          (uint32_t *)(uintptr_t)buff,
          (uint32_t)sector,
          (uint32_t)count,
          SD_DMA_WAIT_TIMEOUT_MS) != MSD_OK)
  {
    sd_diskio_dma_result_error_count++;
    return 0U;
  }

  if (SD_WaitForCardTransfer() == 0U)
  {
    return 0U;
  }

  return 1U;
}

/* -------------------------------------------------------------------------- */

DSTATUS SD_initialize(BYTE lun)
{
  Stat = STA_NOINIT;

  if (BSP_SD_Init() == MSD_OK)
  {
    Stat = SD_CheckStatus(lun);
  }

  return Stat;
}

/* -------------------------------------------------------------------------- */

DSTATUS SD_status(BYTE lun)
{
  return SD_CheckStatus(lun);
}

/* -------------------------------------------------------------------------- */

DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
  uint32_t start_ms;
  uint32_t duration_ms;

  (void)lun;

  if ((buff == NULL) || (count == 0U))
  {
    return RES_PARERR;
  }

  if ((Stat & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  sd_diskio_dma_read_call_count++;
  sd_diskio_dma_read_sector_count += count;
  start_ms = HAL_GetTick();

  if ((((uintptr_t)buff) & SD_DMA_ALIGNMENT_MASK) == 0U)
  {
    if (SD_ReadAligned(buff, sector, count) == 0U)
    {
      return RES_ERROR;
    }
  }
  else
  {
    sd_diskio_unaligned_read_count++;

    for (UINT i = 0U; i < count; i++)
    {
      if (SD_ReadAligned(
              (BYTE *)sd_dma_scratch,
              sector + (DWORD)i,
              1U) == 0U)
      {
        return RES_ERROR;
      }

      memcpy(
          &buff[(uint32_t)i * SD_DEFAULT_BLOCK_SIZE],
          sd_dma_scratch,
          SD_DEFAULT_BLOCK_SIZE
      );
    }
  }

  duration_ms = HAL_GetTick() - start_ms;
  sd_diskio_last_read_duration_ms = duration_ms;

  if (duration_ms > sd_diskio_max_read_duration_ms)
  {
    sd_diskio_max_read_duration_ms = duration_ms;
  }

  return RES_OK;
}

/* -------------------------------------------------------------------------- */

#if _USE_WRITE == 1
DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
  uint32_t start_ms;
  uint32_t duration_ms;

  (void)lun;

  if ((buff == NULL) || (count == 0U))
  {
    return RES_PARERR;
  }

  if ((Stat & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  sd_diskio_dma_write_call_count++;
  sd_diskio_dma_write_sector_count += count;
  start_ms = HAL_GetTick();

  if ((((uintptr_t)buff) & SD_DMA_ALIGNMENT_MASK) == 0U)
  {
    if (SD_WriteAligned(buff, sector, count) == 0U)
    {
      return RES_ERROR;
    }
  }
  else
  {
    sd_diskio_unaligned_write_count++;

    for (UINT i = 0U; i < count; i++)
    {
      memcpy(
          sd_dma_scratch,
          &buff[(uint32_t)i * SD_DEFAULT_BLOCK_SIZE],
          SD_DEFAULT_BLOCK_SIZE
      );

      if (SD_WriteAligned(
              (const BYTE *)sd_dma_scratch,
              sector + (DWORD)i,
              1U) == 0U)
      {
        return RES_ERROR;
      }
    }
  }

  duration_ms = HAL_GetTick() - start_ms;
  sd_diskio_last_write_duration_ms = duration_ms;

  if (duration_ms > sd_diskio_max_write_duration_ms)
  {
    sd_diskio_max_write_duration_ms = duration_ms;
  }

  return RES_OK;
}
#endif

/* -------------------------------------------------------------------------- */

#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  BSP_SD_CardInfo CardInfo;

  (void)lun;

  if ((Stat & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  switch (cmd)
  {
    case CTRL_SYNC:
    {
      res = (SD_WaitForCardTransfer() != 0U) ? RES_OK : RES_ERROR;
      break;
    }

    case GET_SECTOR_COUNT:
    {
      BSP_SD_GetCardInfo(&CardInfo);
      *(DWORD *)buff = CardInfo.LogBlockNbr;
      res = RES_OK;
      break;
    }

    case GET_SECTOR_SIZE:
    {
      BSP_SD_GetCardInfo(&CardInfo);
      *(WORD *)buff = CardInfo.LogBlockSize;
      res = RES_OK;
      break;
    }

    case GET_BLOCK_SIZE:
    {
      BSP_SD_GetCardInfo(&CardInfo);
      *(DWORD *)buff =
          CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
      res = RES_OK;
      break;
    }

    default:
    {
      res = RES_PARERR;
      break;
    }
  }

  return res;
}
#endif
