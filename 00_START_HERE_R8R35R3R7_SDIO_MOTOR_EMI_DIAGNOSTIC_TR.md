# TARAGAY-T1 R8R35R3R7 — SDIO Motor-EMI Diagnostic

Bu revizyon R3R6 davranisini degistirmez. R3R6 flight-safe SD DMA START retry aynen korunur.
R3R7 yalnizca `HAL_SD_WriteBlocks_DMA()` baslangic hatasi oldugu anda, BSP temizligi yapilmadan ONCE ilk ve son hata snapshotlarini UART TGY73'e ekler.

Kaydedilen tanilar:
- HAL return: HAL_OK/HAL_ERROR/HAL_BUSY/HAL_TIMEOUT
- `hsd.State`, `hsd.ErrorCode`, `hsd.Context`
- TX DMA `State`, `ErrorCode`
- `SDIO->STA`, `SDIO->DCTRL`, `SDIO->DCOUNT`
- ilk ve son start-failure zamanlari + toplam failure count

Test BASINCISIZ/INERT yapilmalidir. Needle motor guc hatti BAGLI olsun.

```powershell
python monitor_uart_p112r12r8r35r3r7_sdio_motor_emi_diag_LIVE.py --port COM21 --duration 35
```

TXT dosyasini gonder. Ilk/son snapshot ile HAL mi BUSY/ERROR/TIMEOUT veriyor, HSD/DMA state ve SDIO status bitleri ne durumda net ayrilacak.

NOT: Bu paket ARM/CubeIDE target build ve flight qualification yerine gecmez.
