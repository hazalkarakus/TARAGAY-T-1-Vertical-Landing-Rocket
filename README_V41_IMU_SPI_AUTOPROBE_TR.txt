V41 - IMU SPI1 AUTO-PROBE / STARTUP FIX
========================================
Taban: V40_DRIVER_LEVEL_DIAG

Degisen kaynak dosya:
- App/Modules/Sensors/IMU/imu.c

Diger sensor/sistem kaynaklarina dokunulmadi.

Degisiklik:
1) WHO_AM_I ve IMU register konfigurasyonu, sensor henuz dogrulanmadan DMA'ya bagimli degil.
   2-byte register islemleri polling SPI ile yapiliyor.
2) Baslangicta SPI1 otomatik taraniyor:
   - Mode 3 ve Mode 0
   - /256, /128, /64, /32
   En az 2/3 kez dogru WHO_AM_I (0x6A veya 0x6B) gereklidir.
3) Sensor konfigurasyonu dogrulandiktan sonra runtime icin /32 -> /64 -> /128 -> /256
   sirasinda en hizli stabil profil seciliyor.
4) 1 kHz ham IMU veri yolu yine mevcut DMA mimarisini kullanir.
5) V40 UART driver diagnostigi korunur. Hedef:
   IMU INIT=1 CON=1 WHO=0x6B (veya 0x6A)

Yeni Live Expressions degiskenleri:
- imu_spi_profile_found
- imu_spi_mode_selected
- imu_spi_prescaler_selected
- imu_spi_probe_attempt_count
- imu_spi_poll_read_count
- imu_spi_poll_error_count
