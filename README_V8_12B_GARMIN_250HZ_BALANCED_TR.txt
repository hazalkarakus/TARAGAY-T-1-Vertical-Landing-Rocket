TGY V8.12B - GARMIN LIDAR-LITE v3 / BALANCED 250 Hz MAX
================================================================

KARAR
-----
500 Hz reduced-sensitivity fast profile kullanılmıyor.
Range ve sensitivity öncelikli.

Garmin documented default values:
SIG_COUNT_VAL    0x80
ACQ_CONFIG_REG   0x08
THRESHOLD_BYPASS 0x00

Bu değerler reconnect sırasında açıkça tekrar yazılır. Böylece LIDAR beslemesi
kesilmeden yalnız STM32 resetlenirse eski 0x1D fast ayarı sensörde kalmaz.

HIZ
---
LIDAR state-machine service = 1000 Hz
Host trigger ceiling         = 250 Hz
Minimum trigger interval     = 4000 us
Filter nominal sample rate   = 250 Hz
I2C2                         = 400 kHz

ÖNEMLİ: 250 Hz zorlanmaz. STATUS busy biti düşmeden mesafe okunmaz. Düşük
yansıtıcılık / uzun mesafe acquisition süresini uzatırsa gerçek rate 250 Hz'nin
altına iner. Bu FAIL değildir; güvenilir ölçüm hızdan önceliklidir.

BIAS
----
Her 100 measurement command'ın ilkinde 0x04 bias correction, kalanlarında 0x03.

LIVE EXPRESSIONS
----------------
lidar_profile_config_ok
lidar_profile_sig_count_value
lidar_profile_acq_config_value
lidar_profile_threshold_value

v87_lidar_task_delta_100ms
v812b_lidar_sample_delta_100ms
v812b_lidar_sample_rate_hz
v812b_lidar_250hz_ok
v812b_lidar_profile_config_ok
v812b_integration_ok

lidar_last_trigger_interval_us
lidar_last_sample_interval_us
lidar_timeout_count
lidar_dma_error_count
lidar_sample_pending_overrun_count

Beklenen iyi hedefte:
service /100ms ~100
real samples /100ms ~25
rate ~250 Hz
trigger interval >=4000 us
profile sig_count = 128 (0x80)
profile config = 8
threshold = 0

9m/7m/5m/3m/1m testinde exact 250 Hz yerine valid ve stabil range önceliklidir.
V13 SD + NRF/servo + needle logging ve Scheduler_Rebase cleanup aynen korunur.
