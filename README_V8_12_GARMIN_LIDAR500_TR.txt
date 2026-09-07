TGY V8.12 - GARMIN LIDAR-LITE v3 / 500 Hz
================================================

KAYNAK
------
Bu sürüm Garmin LIDAR-Lite v3 Operation Manual + Garmin'ın resmi
LIDARLite Arduino Library davranışına göre ayarlanmıştır.

GARMIN SINIRLARI
----------------
LIDAR-Lite v3:
- I2C Fast Mode = 400 kbit/s
- default address = 0x62
- update rate:
    270 Hz typical
    650 Hz fast mode (reduced sensitivity)
    >1000 Hz short-range only
- repetition rate:
    ~50 Hz default
    500 Hz MAX

Bu yüzden uçuş için hedef REAL SAMPLE RATE:
500 Hz

650 / >1000 rakamları cihazın ölçüm/update kabiliyetidir.
Garmin'ın belirttiği continuous repetition max 500 Hz olduğu için
driver 2.000 ms'den daha sık yeni measurement trigger etmez.

NEDEN 0x1D?
-----------
Garmin resmi library preset:
Short range / high speed:
SIG_COUNT_VAL      0x02 = 0x1D
ACQ_CONFIG_REG     0x04 = 0x08
THRESHOLD_BYPASS   0x1C = 0x00

Garmin'ın daha agresif ShortRangeHighSpeed örneğinde:
0x02 = 0x0D
0x04 = 0x04
0x12 = 0x03

kullanılıyor.

Biz bunu SEÇMEDİK çünkü roket yaklaşık 9 m'den başlıyor.
500 Hz product repetition limitine ulaşmak için 0x1D fast preset yeterli
olmalı ve 0x0D ayarına göre daha fazla range/sensitivity marjı bırakır.

BIAS CORRECTION
---------------
Garmin high-rate kullanımında receiver bias correction'ın her 100
measurement command'ın başında yapılmasını öneriyor.

V8.12:
measurement 0   -> command 0x04 (bias)
measurement 1-99-> command 0x03 (no bias)
measurement 100 -> command 0x04
...

500 Hz'de yaklaşık her 200 ms'de bir bias correction.

STATE MACHINE
-------------
Eski driver:
fixed 5 ms wait
-> gerçek rate sınırlanıyordu

V8.12:
measurement command
   ->
STATUS 0x01 busy poll
   ->
busy=0
   ->
distance 0x8F read
   ->
minimum 2.000 ms trigger interval dolmuşsa next measurement

State-machine service:
2000 Hz (500 us)

Gerçek range sample hedef:
500 Hz (2000 us)

FILTER
------
Butterworth filter sample rate:
200 Hz -> 500 Hz

LPF cutoff:
15 Hz korunur.

MEDIAN:
3 sample korunur.

I2C:
I2C2 400 kHz korunur.

INVALID READINGS
----------------
Garmin:
1 cm = no-return
5 cm = invalid measurement condition

Bu nedenle V8.12:
valid minimum = 6 cm

LIVE EXPRESSIONS - FAST CONFIG
------------------------------
lidar_fast_config_ok
lidar_fast_sig_count_value
lidar_fast_acq_config_value
lidar_fast_threshold_value

Beklenen:
fast_config_ok          = 1
sig_count               = 29 decimal = 0x1D
acq_config              = 8  = 0x08
threshold               = 0

BIAS:
lidar_bias_command_count
lidar_no_bias_command_count

Yaklaşık oran:
1 bias / 99 no-bias

STATUS:
lidar_status_poll_start_count
lidar_status_poll_complete_count
lidar_status_busy_count
lidar_last_status_reg

TRIGGER:
lidar_last_trigger_interval_us
lidar_min_trigger_interval_us
lidar_max_trigger_interval_us

Normal steady state:
last trigger interval >= 2000 us
yaklaşık 2000 us civarı

SAMPLE:
lidar_update_count
lidar_last_sample_interval_us
lidar_min_sample_interval_us
lidar_max_sample_interval_us

V8.12:
v87_lidar_task_delta_100ms
v812_lidar_sample_delta_100ms
v812_lidar_sample_rate_hz
v812_lidar_fast_config_ok
v812_lidar_500hz_ok
v812_integration_ok

BEKLENEN
--------
Service task:
v87_lidar_task_delta_100ms ~200

Real samples:
v812_lidar_sample_delta_100ms ~50
v812_lidar_sample_rate_hz     ~500

last_sample_interval_us       ~2000 us

fast_config_ok = 1
500hz_ok       = 1

ERROR:
lidar_timeout_count            = 0
lidar_dma_error_count          = 0
lidar_sample_pending_overrun_count = 0 ideal

TEST
----
İlk test motor PSU kapalı yapılabilir.
SD kart boot öncesi takılı olsun.

Clean -> Build -> Debug -> Resume

Önce yalnız:
v812_lidar_sample_rate_hz
v812_lidar_500hz_ok
lidar_last_sample_interval_us
lidar_fast_config_ok
lidar_timeout_count
lidar_dma_error_count
cpu_load_percent

bak.

9 m TESTİ ÇOK ÖNEMLİ
--------------------
0x1D fast mode reduced range/sensitivity trade-off içerir.

Bench'te 2-3 m görmek yeterli değil.
Roketin gerçek geometrisine benzer şekilde:
- 9 m
- 7 m
- 5 m
- 3 m
- 1 m

mesafelerde güvenilir valid sample testi yap.

Her mesafede kontrol:
distance_valid = 1
error/timeout artmıyor
sample rate mümkün olduğunca ~500 Hz
ani 1 cm / 5 cm okumalar oluşmuyor

Eğer 9 m yüzeyinde 500 Hz fast mode güvenilir değilse,
bir sonraki sürümde dinamik profile geçebiliriz:
yüksek irtifada daha fazla acquisition/range,
yakında 500 Hz fast mode.

ŞU ANKİ TERCİH
--------------
Önce tek ve deterministic profile:
Garmin preset 1 / 0x1D
max repetition 500 Hz

Bu test geçerse bu ayarı dondururuz.


V8.12A BUILD FIX
----------------
V8.12 ilk paketinde V13 NRF/servo alanında uint32_t hizalaması nedeniyle
C derleyicisi frame'e otomatik padding ekliyordu.

Sonuç:
beklenen sizeof = 288
gerçek sizeof   = 292
beklenen CRC    = 286
gerçek CRC      = 288

V8.12A'da yalnız V13 NRF/servo alan sırası yeniden düzenlendi:
- V12 needle alanlarının offsetleri DEĞİŞMEDİ
- uint32 packet_count doğal 4-byte hizaya taşındı
- frame yeniden tam 288 byte
- CRC offset yeniden 286

Ayrıca app_tasks.c'nin kullandığı Garmin fast-mode debug değişkenlerinin
extern bildirimleri Lidar.h içine eklendi.
