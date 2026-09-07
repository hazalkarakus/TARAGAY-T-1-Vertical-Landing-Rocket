TGY V8.19M V45 - BMP585 BAROMETRE OKUMA FALLBACK TESTI
======================================================

TABAN:
- V44 IMU SOFTWARE SPI FALLBACK projesi.
- V44'te calisan IMU kodu degistirilmedi.

DEGISEN TEK KAYNAK DOSYA:
- App/Modules/Sensors/Barometer/ms5611_spi.c

DOKUNULMAYANLAR:
- IMU / software-SPI yolu
- LiDAR
- UART / DMA
- nRF
- SD
- scheduler
- RCS / role / STOP / tahliye
- needle / motor / kontrol katmani

V45 BARO DAVRANISI:
1) BMP585 once mevcut 6-byte olcum burst okumasini dener.
2) Burst'ten gecersiz basinc gelirse ayni olcumu 0x1D..0x22 registerlarini
   tek tek okuyarak tekrar dener.
3) Tek-register yolu gecerli olursa bu kart icin fallback aktif kalir.
4) Basinc/temp donusumu ve ust katmana verilen veri formati degistirilmedi.

LIVE EXPRESSIONS (opsiyonel):
- bmp585_single_register_fallback_active
- bmp585_burst_invalid_count
- bmp585_single_register_read_count
- bmp585_single_register_success_count
- bmp585_single_register_error_count

BEKLENEN:
- BARO INIT=1 CON=1 CHIP=0x51 CFG=1
- FRESH ve VSAMP duzenli artmali
- RERR cok hizli artmaktan cikmali
- 400 iyi kalibrasyon orneginden sonra VALID=1 olmali

NOT:
Bu bir tezgah/test duzeltmesidir. Ucus kullanimi icin zamanlama ve veri
kararliligi ayrica test edilmelidir.
