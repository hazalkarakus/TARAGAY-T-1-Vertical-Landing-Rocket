TARAGAY-T1 V8.18 - SENSOR QUALIFICATION BUILD
=============================================

AMAÇ
----
MATLAB/Simulink'ten üretilecek kontrol kodunu sisteme almadan önce IMU,
BMP585 ve Garmin LIDAR-Lite v3 veri yolunu ölçülebilir PASS/FAIL kriterleriyle
doğrulamak.

Bu sürümde fiziksel kontrol çıkışları bilinçli olarak kapalıdır:
- APP_GNC_ACTIVE_ENABLED = 0
- RCS dry-run = 1
- Needle automatic physical output = 0
- Vent servo = 0

Basınç/gaz hattı bağlı olmadan sensör bench testi için hazırlanmıştır.

V8.18'DE YAPILAN KRİTİK DÜZELTMELER
------------------------------------
1) BMP585 gerçek sample rate düzeltildi.
   Önce 200 Hz task her çağrıda aynı BMP585 data register'ını okuyabiliyor ve
   update_count'u artırabiliyordu. Sensör 50 Hz ODR'de olduğu için aynı fiziksel
   sample yaklaşık 4 kez işlenebiliyordu.

   V8.18'de 200 Hz service/polling korunur fakat yeni sample yalnız BMP585
   INT_STATUS.DRDY = 1 olduğunda kabul edilir. Barometer_Update(), median ve
   Butterworth zinciri böylece gerçek yeni 50 Hz örneklerle çalışır.

2) Barometre filtre tasarım sample rate'i 25 Hz yerine gerçek 50 Hz yapıldı.

3) Barometre startup ground-reference kalibrasyonu 1 sample yerine 100 sample
   yapıldı. 50 Hz'de yaklaşık 2 saniyelik referans oluşturur.

4) LIDAR'daki sabit 9.00 m startup calibration kapatıldı.
   LIDAR artık fiziksel mesafeyi yayınlar. Relative vertical origin/reference
   Full-State ESKF tarafından oluşturulur. Böylece sistem 9 m dışında bir
   yükseklikte başlatıldığında yapay offset üretilmez.

5) Ayrı SensorQualification servisi eklendi.
   Gerçek sample rate, freshness, transport errors, calibration, filter state,
   BMP585 DRDY/sample coherence ve stationary IMU sanity aynı anda ölçülür.

SENSÖR HEDEFLERİ
----------------
IMU accepted data     : 950..1050 Hz (nominal 1000 Hz task)
BMP585 real samples   : 45..55 Hz (configured 50 Hz ODR)
Garmin LIDAR real     : 165..205 Hz

LIDAR aralığı completion-driven'dır; default sensitivity profilinde gerçek
bench rate daha önce yaklaşık 175-185 Hz bandında görülmüştür.

PASS için ardışık 3 temiz yaklaşık-1-saniyelik pencere gerekir.

IMU STARTUP
-----------
Kart/aviyonik ilk açılışta SABİT tutulmalıdır.

IMU startup calibration:
- 1200 accepted sample (~1.2 s)
- beklenen poz: body +Z ~= +1 g
- gyro bias ve accelerometer offset hesaplanır
- stddev sınırları geçmeden calibration complete olmaz

Qualification sırasında ayrıca kart sabitken:
- accel norm 0.90..1.10 g
- |gyro X/Y/Z| <= 1.0 dps
şartları aranır.

BMP585
------
- normal mode
- pressure OSR x16
- temperature OSR x2
- IIR bypass (üst katmanda Butterworth var)
- ODR = 50 Hz
- 200 Hz service sadece DRDY poll eder
- accepted update yalnız DRDY sonrası
- pressure Butterworth = 3 Hz @ 50 Hz
- vertical-speed Butterworth = 3 Hz @ 50 Hz
- startup ground pressure reference = 100 valid sample

GARMIN LIDAR-LITE V3
--------------------
Korunan sensitivity profile:
- SIG_COUNT_VAL = 0x80
- ACQ_CONFIG_REG = 0x08
- THRESHOLD_BYPASS = 0x00
- minimum trigger interval = 5 ms
- bias correction = her 100 ölçümde bir
- median3 + 15 Hz Butterworth

Hard-coded 9 m calibration V8.18'de KAPALI.

RAW MEMORY DIAGNOSTIC
---------------------
Live Expressions yerine raw Memory kullan.

ESKF block   : 0x1000F000
GNC block    : 0x1000F100
MATLAB port  : 0x1000F180
SENSOR V8.18 : 0x1000F200

V8.18 sensor block boyutu = 0x100 byte = 64 x 32-bit word.

İlk word'ler:
+0x00 magic        0x818C18D1
+0x04 version      0x00081800
+0x08 size         0x00000100
+0x0C seq_begin

MAP
---
+0x10 flags
+0x14 qualification_state   0 warmup / 1 qualifying / 2 PASS / 3 fail
+0x18 good_windows
+0x1C completed_windows

+0x20 IMU rate x10
+0x24 BARO rate x10
+0x28 LIDAR rate x10
+0x2C IMU device ID
+0x30 IMU sample age us
+0x34 accel norm mg
+0x38 gyro X mdps
+0x3C gyro Y mdps
+0x40 gyro Z mdps
+0x44 IMU calibration sample count
+0x48 gyro cal std X mdps
+0x4C gyro cal std Y mdps
+0x50 gyro cal std Z mdps
+0x54 accel-norm cal std mg

+0x58 IMU DMA error total
+0x5C IMU DMA timeout total
+0x60 IMU stale total
+0x64 IMU pattern error total
+0x68 IMU recovery total
+0x6C IMU register error total
+0x70 IMU redundant disagreement total
+0x74 IMU bit8 correction total

+0x78 BMP585 pressure Pa
+0x7C BMP585 temperature mC
+0x80 baro filtered altitude mm
+0x84 baro vertical speed mm/s
+0x88 baro age us
+0x8C baro accepted update count
+0x90 baro invalid sample total
+0x94 BMP585 communication error total
+0x98 BMP585 chip ID
+0x9C BMP585 config OK
+0xA0 BMP585 DRDY count

+0xA4 LIDAR physical distance mm
+0xA8 LIDAR filtered distance mm
+0xAC LIDAR age us
+0xB0 LIDAR update count
+0xB4 LIDAR error total
+0xB8 LIDAR timeout total
+0xBC LIDAR DMA error total
+0xC0 LIDAR profile config OK
+0xC4 LIDAR pending-overrun total

+0xC8 baro error delta / last 1s window
+0xCC lidar error delta / last 1s window
+0xD0 imu error delta / last 1s window
+0xD4 IMU filter max gap us
+0xD8 baro last sample interval us
+0xDC lidar last sample interval us
+0xE0 baro max sample interval us
+0xE4 lidar max sample interval us
+0xE8 BMP585 low-level accepted update count

+0xEC seq_end
+0xF0 checksum XOR
+0xF4 checksum inverse
+0xF8 0x53454E53 ('SENS')
+0xFC 0x56383138 ('V818')

FLAGS +0x10
-----------
bit0  diagnostic alive
bit1  actuator-safe build
bit2  IMU connected
bit3  IMU last sample valid
bit4  IMU calibration complete
bit5  IMU filter valid/initialized
bit6  IMU rate in 950..1050 Hz
bit7  IMU transport window clean

bit8  baro connected
bit9  baro healthy
bit10 baro calibrated
bit11 baro filter valid/initialized
bit12 baro rate in 45..55 Hz
bit13 baro transport window clean
bit14 BMP585 chip ID == 0x51
bit15 BMP585 configuration OK

bit16 LIDAR connected
bit17 LIDAR distance valid
bit18 LIDAR filter valid/initialized
bit19 LIDAR profile configured
bit20 LIDAR rate in 165..205 Hz
bit21 LIDAR transport window clean
bit22 hard-coded LIDAR startup calibration disabled
bit23 all physical values finite/in range
bit24 sensor timestamps fresh
bit25 instantaneous qualification criteria OK
bit26 FINAL PASS (3 consecutive clean windows)
bit27 recognized IMU WHO_AM_I
bit28 BMP585 DRDY/accepted-sample counters coherent
bit29 stationary IMU sanity OK

PASS KOŞULU
-----------
Qualification state = 2 ve bit26 = 1.

Ayrıca:
- IMU error delta = 0
- baro error delta = 0
- lidar error delta = 0
- BMP585 DRDY ve accepted counts coherent
- rate değerleri hedef aralığında
olmalı.

ÖNEMLİ: IMU redundant disagreement veya bit8 correction sayacı yeni bir
1-saniyelik pencerede artarsa o pencere FAIL sayılır. V8.18 bu tip bus/veri
problemlerini gizlemek yerine özellikle görünür hale getirir.

İLK BENCH TESTİ
---------------
1) Basınç/gaz hattını tamamen ayır.
2) Clean Project -> Build Project -> Debug.
3) Kartı açılıştan itibaren düz ve sabit tut.
4) Resume et ve yaklaşık 6-8 saniye elleme.
5) Pause/Suspend.
6) Memory view -> 0x1000F200.
7) 0x1000F200..0x1000F2FF ekran görüntüsünü al.
8) İlk hedef: state=2/PASS ve 3 temiz window.

Bu sürüm sensör verilerini 'mükemmel' ilan etmez; bunu ölçülebilir hale getirir.
İlk snapshot/log sonucuna göre IMU noise, baro drift, LIDAR jitter ve hata
sayaçlarını ayrı ayrı sıkılaştıracağız. MATLAB kontrol koduna ancak bu katman
stabil PASS verdikten sonra geçilmelidir.
