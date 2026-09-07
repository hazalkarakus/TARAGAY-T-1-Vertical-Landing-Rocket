TGY V8.13 - LIDAR 200 Hz FREEZE + ROBUST SD RETRY
======================================================

NEDEN V8.13?
------------
Son testte Garmin LIDAR-Lite v3 default sensitivity profile:
SIG_COUNT = 0x80
ACQ_CONFIG = 0x08
THRESHOLD = 0x00

ile gerçek sample cadence sürekli:
5000 us = 200 Hz

çıktı.

Aynı testte SD:
frame_count            = 7424
total_bytes_written    = 2119680 byte

V13 frame = 288 byte olduğuna göre:
2119680 / 288 = 7360 frame

Yani:
7424 - 7360 = 64 frame

tam olarak BİR eski 64-frame writer buffer'ı SD'ye tamamlanmadan kaldı.

Ayrıca:
async data start    = 116
async data complete = 115

de tam olarak 1 transfer farkı gösterdi.

Bu yüzden V8.13 iki şeyi dondurur/düzeltir.

1) LIDAR = GERÇEK 200 Hz
------------------------
Garmin range/sensitivity ayarları korunur:
SIG_COUNT_VAL    = 0x80
ACQ_CONFIG       = 0x08
THRESHOLD        = 0x00

I2C2 = 400 kHz
state-machine service = 1 kHz

Host trigger cap:
5000 us = 200 Hz

Filter sample rate:
200 Hz

Bu artık kabul edilen ayardır.
Ekstra 50 Hz uğruna sensitivity azaltılmıyor.

Live:
v813_lidar_sample_delta_100ms
v813_lidar_sample_rate_hz
v813_lidar_200hz_ok
v813_lidar_profile_config_ok

Beklenen:
sample delta /100ms ~= 20
sample rate ~= 200 Hz
200hz_ok = 1

2) SD DMA TRANSIENT RETRY
-------------------------
Eski writer:
64 frame x 288 = 18432 byte
= 36 sector / DMA

Yeni writer:
32 frame x 288 = 9216 byte
= 18 sector / DMA

Transferler yarıya kısaldı.

Normal 50 Hz capture için:
32 frame = 0.64 s data / write

Bu hâlâ çok düşük SD write frekansıdır fakat uzun multi-sector DMA burst
süresini azaltır.

DMA timeout:
250 ms -> 500 ms

Daha önemlisi:
DATA veya GUARD DMA'da geçici:
- start failure
- HAL transfer error
- timeout

olursa logger hemen kapanmaz.

Aynı sektör adresine maksimum 2 kez yeniden yazar.
Data sector offset yalnız başarıdan sonra ilerlediği için retry sektör atlamaz.

Live:
sd_logger_dma_retry_attempt_count
sd_logger_dma_retry_success_count
sd_logger_dma_retry_exhausted_count
sd_logger_dma_retry_card_busy_count
sd_logger_dma_retry_abort_count
sd_logger_dma_retry_pending
sd_logger_dma_retry_current

İdeal:
attempt = 0
success = 0
exhausted = 0

Ama kart tek transient hata verirse kabul edilebilir:
attempt = 1
success = 1
exhausted = 0

ve logging DEVAM ETMELİ.

FAIL:
exhausted > 0
write_error > 0
dropped frame > 0
ring overrun > 0

SD MUHASEBE TESTİ
-----------------
V13 frame = 288 byte.

Finalden sonra:
total_bytes_written == frame_count * 288

olmalı.

Live:
v813_sd_accounting_ok

Finalde = 1 istiyoruz.

ÖRNEK:
5000 frame -> 1,440,000 byte

BARO TAKILI DEĞİLKEN
--------------------
v87_baro_ok = 0
ve full integration = 0 normaldir.

Bunun için ayrı:
v813_core_without_baro_ok

eklendi.

Baro yokken bile:
IMU + LIDAR + valve + NRF + servo + scheduler

sağlıklıysa:
v813_core_without_baro_ok = 1

Uçuş/full-system kriteri:
v813_integration_ok

barometreyi yine zorunlu tutar.

TEST
----
SD kart boot öncesi takılı.
Baro takılı olmak zorunda değil.
Motor PSU başlangıçta kapalı olabilir.

Önce 60-120 saniye yalnız logging testi.

Live:
v813_lidar_sample_rate_hz
v813_lidar_200hz_ok
v813_core_without_baro_ok

sd_logger_frame_count
sd_logger_total_bytes_written
sd_logger_async_data_start_count
sd_logger_async_data_complete_count

sd_logger_dma_retry_attempt_count
sd_logger_dma_retry_success_count
sd_logger_dma_retry_exhausted_count

sd_logger_error_count
sd_logger_write_error_count
sd_logger_dropped_frame_count
sd_logger_ring_overrun_count

Sonra actuator/NRF/servo dinamik testi.

Profil PASS + auto finalize sonrasında:
v813_sd_accounting_ok = 1

ve:
frame_count * 288 == total_bytes_written

olmalı.

Bu geçerse SD + LIDAR altyapısını dondurup estimator/ESKF'ye geçebiliriz.
