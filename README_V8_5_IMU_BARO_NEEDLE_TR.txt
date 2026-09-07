TGY FULL INTEGRATION - IMU + BAROMETER + NEEDLE V8.5
=====================================================

V8.4 SONUCU
-----------
IMU + gerçek actuator profili birlikte PASS:
- IMU 1 kHz = 100 task / 100 ms
- Valve TIM7 = 20 tick / 100 ms = 200 Hz
- IMU max exec ~362 us
- overrun = 0
- deadline miss = 0
- CPU load ~24%
- actuator 30/50/80/100/0 PASS
- valve fault = 0

V8.5'TE EKLENEN
---------------
Mevcut projedeki BAROMETER katmanı SPI2 üzerinde geri açıldı.

ÖNEMLİ:
Dosya/üst API isimleri hâlâ MS5611 uyumluluğu taşıyor fakat yüklediğin
güncel projenin düşük seviye sürücüsü gerçekte Bosch BMP585/BMP581 içindir.
Kodda chip ID:
  BMP585 = 0x51
  BMP581 = 0x50
beklenir.

Bu aşamada barometre sürücüsünün algoritmasını DEĞİŞTİRMEDİM.
Sadece mevcut sürücüyü 200 Hz scheduler slotunda yeniden aktifleştirip
IMU + actuator coexistence ölçümleri ekledim.

AKTİF
-----
- IMU 1 kHz
- Barometer service 200 Hz
- Needle valve exact TIM7 200 Hz
- CPU monitor
- USER-button actuator profile

HALA KAPALI
-----------
- LIDAR
- NRF / RemoteControl
- SD logger
- attitude estimator
- EKF / ESKF
- servo
- active RCS

İLK TEST
--------
Motor PSU KAPALI.

Import -> Clean -> Build -> Debug -> Resume.

Live Expressions:

IMU:
v85_imu_connected
v85_imu_device_id
v85_imu_driver_sample_valid
v85_imu_progress_ok
v85_imu_sample_age_us
v85_imu_task_delta_100ms
v85_imu_task_max_exec_us
v85_imu_task_overrun_count
v85_imu_task_deadline_miss_count

BAROMETER:
v85_baro_connected
v85_baro_data_ready
v85_baro_pressure_valid
v85_baro_calibrated
v85_baro_healthy
v85_baro_sensor_manager_valid
v85_baro_progress_ok

v85_baro_temperature_c
v85_baro_pressure_pa
v85_baro_filtered_pressure_pa
v85_baro_altitude_m
v85_baro_filtered_altitude_m
v85_baro_vertical_speed_mps

v85_baro_valid_sample_count
v85_baro_invalid_sample_count
v85_baro_source_update_count
v85_baro_source_delta_100ms

v85_baro_task_delta_100ms
v85_baro_task_last_exec_us
v85_baro_task_max_exec_us
v85_baro_task_overrun_count
v85_baro_task_deadline_miss_count
v85_baro_max_measure_duration_us

COEXISTENCE:
v85_scheduler_ok
v85_imu_ok
v85_baro_ok
v85_valve_tick_ok
v85_adc_ok
v85_integration_ok
v85_valve_tick_delta_100ms

NEEDLE:
needle_valve_raw_adc
needle_valve_zero_adc
needle_valve_fault
needle_valve_control_tick_count
needle_valve_hw_adc_timeout_count

CPU:
cpu_load_percent
cpu_idle_percent
cpu_task0_load_percent
cpu_task1_load_percent

BEKLENEN
--------
IMU:
imu_ok = 1
imu task delta /100ms ~100
overrun = 0
deadline miss = 0

BARO:
baro_connected = 1
data_ready = 1
pressure_valid = 1
calibrated = 1
healthy = 1
sensor_manager_valid = 1
progress_ok = 1
baro_ok = 1

Basınç:
yaklaşık bulunduğun ortamın atmosfer basıncı.
Genel sanity range:
30000 .. 120000 Pa.

BARO TASK:
v85_baro_task_delta_100ms ~20
overrun = 0
deadline miss = 0

VALVE:
v85_valve_tick_delta_100ms ~20
fault = 0
ADC timeout = 0 veya çok düşük

FINAL:
v85_scheduler_ok = 1
v85_imu_ok = 1
v85_baro_ok = 1
v85_valve_tick_ok = 1
v85_adc_ok = 1
v85_integration_ok = 1

CPU:
V8.4'te yaklaşık %24 idi.
Barometre eklendiğinde yeni değeri ölç; şimdilik sabit bir limit dayatmıyoruz.
Önemli olan overrun/deadline miss olmaması ve valve 200 Hz'in korunması.

BARO SÜRÜCÜ NOTU
----------------
Mevcut BMP585 düşük seviye sürücü normal modda DATA register'larını
200 Hz servis çağrısında okuyor. ODR ve üst filtre örnekleme oranını
bu entegrasyon testinden SONRA ayrıca doğrulayacağız.

Yani V8.5'in amacı:
"barometre + IMU + actuator aynı anda stabil mi?"
sorusunu cevaplamak.

ACTUATOR YÜK TESTİ
------------------
Statik coexistence PASS olursa mevcut USER-button profili:

1. ZERO
2. ENABLE
motor PSU düşük akım limitiyle aç
3. 30 -> 50 -> 80 -> 100 -> 0

Live Expressions:
v83_test_auto_active
v83_test_profile_index
v83_test_result

PASS:
v83_test_result = 2
v83_test_profile_index = 5
v85_integration_ok = 1
needle_valve_fault = 0
RAW ~= ZERO

SONRAKİ
-------
V8.5 geçerse:
1) BMP585 gerçek ODR/filter timing kontrolü
2) sonra LIDAR entegrasyonu
