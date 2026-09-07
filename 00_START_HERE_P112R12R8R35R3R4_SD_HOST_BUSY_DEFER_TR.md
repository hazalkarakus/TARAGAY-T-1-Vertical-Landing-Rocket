# P112R12R8R35R3R4 — SD HOST-BUSY DEFER

Bu revizyon R8R35R3R3'ün SD 200 Hz throughput düzeltmesini korur ve yalnızca
SDIO DMA **başlatma anındaki kısa host-busy yarışını** ele alır.

## Neden
R8R35R3R3 donanım logunda:
- 200 Hz capture/push/pop dengeliydi,
- ring 0–1 frame civarında kaldı,
- drop/overrun 0 idi,
- fakat bir sonraki writer buffer DMA başlatılırken `FR_DISK_ERR` oluştu.

Bu revizyonda yeni DMA başlatılmadan önce şu dört koşul birlikte doğrulanır:

1. `sd_dma_transfer_active == 0`
2. `hsd.State == HAL_SD_STATE_READY`
3. `hsd.Context == SD_CONTEXT_NONE`
4. TX DMA handle `HAL_DMA_STATE_READY`

Bu koşullardan biri hazır değilse **hata üretilmez**. Writer buffer değiştirilmez,
retry bütçesi tüketilmez, HAL abort/reinit çağrılmaz. Bir sonraki main-loop turunda
yeniden denenir.

Gerçek SDIO CRC/timeout/command/DMA hataları için mevcut fail-visible / flight-safe
politika aynen korunur.

## Değişmeyen uçuş/kontrol kaynakları
- TaragayFlightLogic / Hover V19.6: değişmedi
- RCS V7.13.4 / SolenoidOutput: değişmedi
- Full-State ESKF: değişmedi
- Needle autonomous controller: değişmedi
- Scheduler: değişmedi
- PE9 / authorization / STOP mantığı: değişmedi

## Yeni UART SD diagnostikleri
TGY73 sonuna üç alan eklendi:
- `sd_host_busy_defers_r4`
- `sd_bsp_hal_state_r4` (`1 = HAL_SD_STATE_READY`)
- `sd_dma_start_errors_r4`

## Test
**Basınçsız / depressurized** test yap.

```powershell
python monitor_uart_p112r12r8r35r3r4_sd_host_busy_defer_LIVE.py --port COM21 --duration 35
```

Beklenen:
- `SD=1/1` test sonuna kadar
- `ring` düşük (0–birkaç frame)
- `drop=0`
- `sd_dma_start_errors_r4=0`
- `sd_host_busy_defers_r4` 0 veya pozitif olabilir; pozitifse gate gerçekten bir
  kısa host-busy penceresini ertelemiş demektir
- scheduler miss/realign 0

Bu revizyon tek başına "flight qualified" ilanı değildir.
