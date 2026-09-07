V46 SD MOUNT FIX

Taban: V45. IMU/BARO/LIDAR/NRF/control degistirilmedi.

SD tarafinda:
- SDIO data bus tekrar 4-bit calisma moduna alindi (eski calisan V12 ile ayni).
- Kart identification her zamanki gibi 1-bit baslar, sonra HAL_SD_ConfigWideBusOperation ile 4-bit'e gecilir.
- SDLogger f_mount oncesi disk_initialize(0) yapar.
- STA_NOINIT ise 20 ms sonra bir kez retry yapar.
- Mount hata verirse bir kez temiz re-init + remount denenir.
- Otomatik format YOK; karttaki dosyalar silinmez.
- UART SD satirina LAST/DISK/RETRY/HAL/WIDE/HE diagnostics eklendi.

Beklenen basari:
SD INIT=1 MOUNT=1 READY=1 LOG=1 ERR=0 LAST=0 DISK=0 HAL=0 WIDE=0 HE=0
