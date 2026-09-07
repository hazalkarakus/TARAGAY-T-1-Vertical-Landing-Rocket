TGY FULL INTEGRATION - SD + IMU + BMP585 + LIDAR + NEEDLE V8.7
===================================================================

V8.6 DİNAMİK SONUCU
-------------------
Gerçek actuator 30/50/80/100/0 profili LIDAR + BMP585 + IMU açıkken PASS:
- v83_test_profile_index = 5
- v83_test_result = 2
- IMU OK = 1
- BARO OK = 1
- LIDAR OK = 1
- valve exact 200 Hz OK = 1
- integration OK = 1
- LIDAR DMA error = 0
- LIDAR timeout = 0
- CPU load yaklaşık %29

V8.7'DE EKLENEN
---------------
Mevcut SDLogger geri açıldı.

SD donanımı / yazılım yolu:
- SDIO 4-bit
- DMA2 Stream6
- FATFS
- flight.bin
- 256-byte V11 binary frame
- TIM5 exact 50 Hz capture
- 256-frame RAM ring
- double writer buffers
- raw multi-sector SDIO DMA
- 32 MB file preallocation

ÖNEMLİ
------
SD kartı BOOT'TAN ÖNCE tak.

SDLogger_Init() boot sırasında:
- yaklaşık 500 ms bekleme
- FATFS mount
- flight.bin CREATE_ALWAYS
- preallocation
yapar.

Kart yoksa sensör/actuator sistemi yine çalışır ama:
v87_sd_ok = 0
v87_integration_ok = 0
olur.

ZAMANLAMA
---------
IMU                 1000 Hz
BMP585 service       200 Hz
LIDAR service        200 Hz
Needle TIM7          exact 200 Hz
SD TIM5 capture       50 Hz

SDLogger_Update(), cooperative scheduler'ın task listesine eklenmedi.
Main loop içinde asenkron DMA servis edilir.

SD kaynak snapshot'ı IMU 1 kHz taskında publish edilir.

LIVE EXPRESSIONS - SD
---------------------
sd_logger_initialized
sd_logger_ready
sd_logger_mount_ok
sd_logger_test_passed
sd_logger_file_open
sd_logger_logging_active

sd_logger_frame_count
sd_logger_total_bytes_written
sd_logger_dropped_frame_count
sd_logger_missed_period_count

sd_logger_ring_count
sd_logger_ring_high_watermark
sd_logger_ring_overrun_count
sd_logger_buffer_overrun_count

sd_logger_async_data_start_count
sd_logger_async_data_complete_count
sd_logger_async_timeout_count

sd_logger_error_count
sd_logger_write_error_count

sd_logger_capture_timer_started
sd_logger_timer_irq_count
sd_logger_timer_last_interval_us
sd_logger_timer_min_interval_us
sd_logger_timer_max_interval_us
sd_logger_timer_max_isr_duration_us

sd_logger_source_publish_count
sd_logger_preallocate_ok
sd_logger_preallocated_bytes
sd_logger_preallocation_duration_ms

V8.7:
v87_sd_ok
v87_sd_progress_ok
v87_sd_frame_delta_100ms
v87_sd_bytes_delta_100ms

v87_sd_update_count
v87_sd_update_last_us
v87_sd_update_max_us
v87_sd_auto_stop_done
v87_sd_auto_stop_duration_ms

COEXISTENCE
-----------
v87_scheduler_ok
v87_imu_ok
v87_baro_ok
v87_lidar_ok
v87_sd_ok
v87_valve_tick_ok
v87_adc_ok
v87_integration_ok

v87_imu_task_delta_100ms
v87_baro_task_delta_100ms
v87_lidar_task_delta_100ms
v87_valve_tick_delta_100ms

v87_imu_task_max_exec_us
v87_imu_task_overrun_count
v87_imu_task_deadline_miss_count

v87_baro_task_max_exec_us
v87_baro_task_overrun_count
v87_baro_task_deadline_miss_count

v87_lidar_task_max_exec_us
v87_lidar_task_overrun_count
v87_lidar_task_deadline_miss_count

cpu_load_percent
cpu_idle_percent

STATİK BEKLENTİ
---------------
SD:
initialized = 1
ready = 1
mount_ok = 1
test_passed = 1
file_open = 1
logging_active = 1
capture_timer_started = 1

v87_sd_frame_delta_100ms ≈ 5
v87_sd_progress_ok = 1
v87_sd_ok = 1

sd_logger_frame_count sürekli artmalı.
sd_logger_source_publish_count çok hızlı artmalı.
dropped/ring_overrun/buffer_overrun = 0
async_timeout = 0
write_error = 0

TIM5:
last interval yaklaşık 20000 us.
Debugger nedeniyle min/max tekil sapabilir; sürekli sapmamalı.

Sistem:
IMU delta /100ms   ≈ 100
BARO delta /100ms  ≈ 20
LIDAR delta /100ms ≈ 20
VALVE /100ms       ≈ 20

v87_integration_ok = 1

DİNAMİK ACTUATOR + SD TESTİ
---------------------------
Statik test PASS ise:

Motor PSU başlangıçta kapalı.

1. USER -> ZERO
2. USER -> ENABLE
motor PSU düşük akım limitinde aç
3. USER -> 30 -> 50 -> 80 -> 100 -> 0

Profil boyunca SD logger açık kalır.

PASS sonrasında V8.7 otomatik:
SDLogger_Stop()
yapar ve flight.bin'i finalize eder.

Final:
v83_test_profile_index = 5
v83_test_result = 2

v87_sd_auto_stop_done = 1
sd_logger_logging_active = 0
sd_logger_file_open = 0
needle_valve_fault = 0
RAW yaklaşık ZERO

Bu noktadan sonra SD kartı güvenle sökebilirsin.

DİKKAT
------
SDLogger_Stop() finalde dosyayı truncate/sync/close eder ve kısa süre bloklayabilir.
Bu yalnız actuator profili tamamen bittikten sonra yapılır.

V8.7 PASS KRİTERİ
-----------------
Profil çalışırken:
- IMU / BARO / LIDAR sağlıklı
- valve exact 200 Hz
- SD 50 Hz capture
- frame drop = 0
- ring overrun = 0
- buffer overrun = 0
- SD async timeout = 0
- sensor task overrun/deadline miss = 0
- needle fault = 0

SONRAKİ AŞAMA
-------------
V8.7 geçerse:
- SD flight.bin dosyasını PC'de doğrula
- sonra estimator/ESKF katmanlarını kademeli geri aç
- NRF/RCS/servo en son eklenir
