TGY V8.19M V44 - IMU SOFTWARE SPI FALLBACK

TABAN:
- V43_IMU_DIRECT_SPI_REGISTER_FIX

V43 TEST SONUCU:
- GPIO bit-bang: WHO_AM_I = 0x6B ve register fingerprint aliniyor.
- STM32 SPI1 peripheral: MODE0/MODE3 WHO_AM_I = 0x00.
Bu nedenle fiziksel IMU/hat cevabi var, ancak SPI1 peripheral yolu bu kartta kullanilabilir veri uretmiyor.

V44 DEGISIKLIGI:
- YALNIZ App/Modules/Sensors/IMU/imu.c degistirildi.
- IMU icin software-SPI Mode 3 fallback eklendi.
- Baslangicta software-SPI ile WHO_AM_I 3 kez okunur, en az 2 gecerli cevapta fallback aktif olur.
- IMU register read/write ve ham 13-byte okumalar software-SPI backend'e yonlendirilir.
- Mevcut validation, calibration, estimator arayuzu korunur.
- Diger sensor, UART, nRF, SD, LiDAR, barometre, scheduler ve kontrol dosyalari degistirilmedi.
- Hardware SPI yolu fallback olarak kodda kalir.

NOT:
- UART'taki eski IMU DMA sayaçlari uyumluluk icin software-transfer tamamlanmalarinda da artar.
  Bu V44'te gercek SPI1 DMA kullanildigi anlamina gelmez.
- V44 once tezgah/test ortaminda dogrulanmalidir; flight-qualified kabul edilmemelidir.

BEKLENEN UART:
IMU INIT=1 CON=1 WHO=0x6B ...
ve ardindan SVALID/VSAMP/VALID degerlerinin ilerlemesi beklenir.

LIVE EXPRESSIONS EK DIAGNOSTIK:
imu_swspi_active       -> 1 beklenir
imu_swspi_whoami       -> 0x6A veya 0x6B
imu_swspi_probe_count
imu_swspi_transfer_count
imu_swspi_error_count  -> 0 beklenir
