TGY V8.19M - KUCUK ROKET BOARD UYUMLULUK V22
=============================================

TABAN / KORUNAN UYGULAMA:
- TGY_V8_19M_NEEDLE_ENDURANCE_10CYCLE
- Needle valve / ana motor kontrol mantigi korunmustur.
- Servo, kontrol, estimator, SensorQualification ve 200 Hz TIM7 needle kontrolu korunmustur.
- IMU surucusu degistirilmedi (iki projede de zaten byte-byte ayniydi).
- BMP585/barometre ust seviye ve gelismis V8.19M surucusu korunmustur.

KUCUK BOARD'DA CALISAN REFERANSTAN ALINAN DONANIM DAVRANISI:
1) I2C2: 400 kHz yerine 100 kHz.
2) LIDAR: TGY_ALL_SENSORS_V20'deki basit ve yeniden baglanan I2C2/DMA state machine.
   V8.19M'in bekledigi ek diagnostic semboller uyumluluk icin korundu.
3) NRF24: TGY_ALL_SENSORS_V20'deki dogrudan HAL_SPI3 surucusu.
   Adres TGY01, kanal 76, payload 4 byte ayarlari korunur.
4) SDIO host: 1-bit mod.
5) FatFs sd_diskio: mount/dosya islemlerinde blocking BSP read/write yolu.
6) BSP SD: kart 1-bit modda tutulur.

BILEREK KORUNAN HEDEFLER:
- TIM5 200 Hz logger capture ayari V8.19M'den korunur.
- TIM7 needle-valve ISR korunur.
- V8.19M SD frame formati, needle/nRF/servo logger alanlari korunur.
- V8.19M RemoteControl deglitch ve monitor mantigi korunur; sadece alt nRF SPI surucusu degisti.

FLASH SONRASI KONTROL:
- APP_VERSION_STRING = 8.19M-SMALLHW-V22
- IMU: mevcut davranis bozulmamali.
- BARO: mevcut V8.19M davranisi bozulmamali.
- LIDAR: lidar_connected -> 1, lidar_distance_valid -> 1 ve lidar_update_count artmali.
- NRF: nrf24_connected -> 1, remote_rx_valid_packet_count verici acikken artmali.
- SD: sd_logger_mount_ok -> 1, sd_logger_file_open -> 1, sd_logger_total_bytes_written artmali.

NOT:
CubeMX ile .ioc'den tekrar 'Generate Code' yaparsan elle uygulanmis 100 kHz I2C2 ve 1-bit SDIO
ayarlarini geri yazabilir. Bu V22'yi test ederken Generate Code yapma; sadece Clean + Build yap.
