TARAGAY-T1 V8.19B - IMU QUALIFICATION + BMP585 SABIT BELLEK TESHISI
===================================================================

BU SURUMUN AMACI
----------------

V8.19B yalniz iki kontrollu degisiklik yapar:

1) IMU qualification penceresi duzeltildi.
2) BMP585'in yaklasik 38 Hz gorunmesinin nedenini ayirmak icin yeni,
   sembollerden bagimsiz sabit bellek blogu eklendi.

Barometre kabul araligi DEGISTIRILMEDI:

    APP_SENSOR_QUAL_BARO_RATE_MIN_HZ = 90.0 Hz
    APP_SENSOR_QUAL_BARO_RATE_MAX_HZ = 111.0 Hz

MATLAB vana kontrolu, ESKF ayarlari, RCS ve fiziksel cikislar degistirilmedi.


1. IMU QUALIFICATION DUZELTMESI
-------------------------------

ISM330DLC 1.666 kHz ODR'de calisirken 1 kHz task icinde uc ardisik SPI burst
okunuyor. Uc ham okumanin birbirinden farkli olmasi yeni sensor verisinin
gelmesi nedeniyle normaldir.

V8.19A'da su iki duzeltilmis/diagnostik olay qualification hatasi sayiliyordu:

    imu_redundant_disagreement_count
    imu_bit8_correction_count

V8.19B'de bunlar izlenmeye devam eder fakat qualification penceresini kirletmez.
Qualification artik gercek hata/reddetme kaynaklarini kullanir:

    imu_dma_error_count
    IMU_GetDmaTimeoutCount()
    IMU_GetStaleCount()
    IMU_GetPatternErrorCount()
    IMU_GetRecoveryCount()
    IMU_GetRegisterErrorCount()
    imu_redundant_reject_count

Beklenen sonuc: gercek hata/reject yoksa sensor_qual_flags bit 7 artik 1 olur.


2. BMP585 SABIT BELLEK BLOGU
----------------------------

CubeIDE Debug Memory gorunumunde su adresi ac:

    0x1000F300

256 byte / 64 adet 32-bit word oku. CubeIDE byte gorunumu little-endian'dir.

Ilk uc 32-bit word:

    [0] = 0xB58519B1   magic
    [1] = 0x0008190B   V8.19B
    [2] = 0x00000100   256 byte

[3] ve [59] ayni, cift sequence olmali. [60] checksum, [61] checksum tersidir.


WORD HARITASI
-------------

 [0]  magic
 [1]  version
 [2]  byte size
 [3]  begin sequence
 [4]  diagnostic flags
 [5]  scheduler BARO task toplam run_count
 [6]  BMP585 driver service toplam cagrisi
 [7]  INT_STATUS toplam poll
 [8]  INT_STATUS okuma hata sayisi
 [9]  DRDY=0 poll sayisi
[10]  DRDY=1 poll sayisi
[11]  measurement read denemesi
[12]  measurement read hata sayisi
[13]  low-level kabul edilen sample
[14]  ust katman kabul edilen sample
[15]  duplicate sample skip

[16]  BARO task rate x10
[17]  driver service rate x10
[18]  INT_STATUS poll rate x10
[19]  DRDY=0 poll rate x10
[20]  DRDY=1 rate x10
[21]  measurement read rate x10

[22]  BARO task last execution us
[23]  BARO task max execution us
[24]  BARO task overrun count
[25]  BARO task deadline miss count

[26]  son service araligi us
[27]  minimum service araligi us
[28]  maksimum service araligi us
[29]  son DRDY araligi us
[30]  minimum DRDY araligi us
[31]  maksimum DRDY araligi us

[32]  CHIP_ID                 beklenen 0x51
[33]  INT_SOURCE              beklenen 0x01
[34]  ODR_CONFIG              beklenen 0xA9
[35]  OSR_CONFIG              beklenen 0x59
[36]  OSR_EFF                 beklenen 0x99
[37]  STATUS                  NVM_RDY=1, NVM_ERR=0
[38]  son INT_STATUS
[39]  periyodik register snapshot count
[40]  register snapshot read error count
[41]  register mismatch count
[42]  son register snapshot timestamp us

[43]  SPI communication error count
[44]  invalid ADC count
[45]  ADC read count
[46]  barometer source update count
[47]  barometer accepted update count
[48]  |task run - driver service|
[49]  |DRDY - low-level accepted|
[50]  qualification baro rate x10
[51]  diagnostic snapshot timestamp us
[52]  son kabul edilen sample araligi us
[53]  minimum sample araligi us
[54]  maksimum sample araligi us
[55]  bmp585_config_ok
[56]  bmp585_drdy_source_ok
[57]  bmp585_register_readback_ok
[58]  diagnostic flags kopyasi
[59]  end sequence
[60]  checksum
[61]  checksum inverse
[62]  0x4241524F = 'BARO'
[63]  0x56313942 = 'V19B'


3. 38 HZ NEDENINI AYIRMA
-------------------------

10 saniye calistirdiktan sonra beklenen 200 Hz servis degerleri:

    [16] ~= 2000
    [17] ~= 2000
    [18] ~= 2000
    [26] ~= 5000 us
    [48] <= 1

Beklenen 100.299 Hz sensor/DRDY degerleri:

    [20] ~= 1000
    [21] ~= 1000
    [29] ~= 10000 us
    [49] <= 1

Temel ayrim:

- [16]/[17]/[18] yaklasik 2000, fakat [20] yaklasik 380 ise scheduler dogru;
  sorun sensor DRDY uretimi veya register konfigurasyonu tarafindadir.
- [16] de dusukse scheduler/task servis frekansi gercekte 200 Hz degildir.
- [16] dogru, [17] dusukse Barometer_Update driveri her taskta cagirmiyordur.
- [17] dogru, [18] dusukse driver polling zinciri erken donuyordur.
- [8] veya [43] artiyorsa SPI/INT_STATUS okuma sorunu vardir.
- [40] veya [41] artiyorsa registerlar calisma sirasinda okunamiyor veya
  beklenen konfigurasyonda kalmiyordur.
- [19] + [20] yaklasik [18] olmalidir.

Bosch BMP585 datasheet'e gore ODR code 0x0A = 100.299 Hz'dir. Pressure x8 ve
temperature x2 kombinasyonunda 100.299 Hz NORMAL mode gecerlidir.


4. TEST ADIMI
-------------

1) Project -> Clean
2) Project -> Build Project
3) Yeni V8.19B .elf dosyasini karta Debug ile yukle
4) Resume ile en az 10 saniye calistir
5) Suspend
6) Memory view -> 0x1000F300
7) 0x1000F300..0x1000F3FF ekran goruntusunu gonder

Ek olarak 0x1000F200 sensor qualification blogunun ilk uc word'u V8.19B'de:

    0x819C19D1
    0x0008190B
    0x00000100
