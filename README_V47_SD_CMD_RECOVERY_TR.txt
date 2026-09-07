V47 SD CMD RECOVERY / DIAGNOSTIC

Taban: V46.

Degisen kaynaklar:
- FATFS/Target/bsp_driver_sd.c
- Core/Src/sdio.c
- App/app.c (yalniz UART diagnostik alanlari)

Dokunulmayan kritik kisimlar:
- IMU V44 software-SPI surucusu
- BMP585 V45 fallback
- LiDAR
- nRF
- kontrol / motor / role

V46 logunda HAL=1, HE=4 goruldu. HE=4 HAL_SD_ERROR_CMD_RSP_TIMEOUT'tur;
FatFS mounttan once kart komut cevabi alinamamaktadir.

V47:
1) HAL_SD_DeInit ile hostu tamamen kapatir.
2) SDIO periferik reset uygular.
3) 50 ms bekleyip HAL_SD_Init'i en fazla 3 kez dener.
4) CMD ve DAT GPIO'larinda zayif internal pull-up kullanir; CLK NOPULL kalir.
5) 4-bit gecis basarisiz olsa bile 1-bit SDIO ile devam etmeyi dener.
6) Otomatik format YOKTUR, dosya silmez.

UART yeni alanlar:
ATT=init deneme sayisi, HR=host reset sayisi, CMD=CMD idle seviyesi,
D=D0D1D2D3 idle seviyeleri, LE=son HAL hata kodu, REC=recovery basarisi,
PWR/CLK/STA/R1=SDIO register diagnostigi.
