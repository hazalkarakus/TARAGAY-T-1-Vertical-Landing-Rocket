TGY FULL INTEGRATION - IMU + NEEDLE VALVE V8.4
================================================

BU AŞAMADA AKTİF
----------------
- Orijinal full-project main/peripheral init
- Board / Timebase / BusManager
- Cooperative scheduler
- IMU + SPI1 DMA (mevcut proje sürücüsü)
- SensorManager IMU startup calibration + Butterworth filtering
- Needle Valve PC1 ADC
- TIM7 exact 200 Hz actuator control
- TIM3 PB0/PB1 BTS7960 PWM
- USER-button 30/50/80/100/0 actuator profile
- CPU load monitor

HALA KAPALI
-----------
- Barometer
- LIDAR
- NRF / RemoteControl
- SD logging
- attitude estimator
- Vertical EKF / FullState ESKF
- servo
- active RCS controller

RCS relay outputs safe OFF tutulur.

ÖNEMLİ IMU NOTU
---------------
Bu V8.4, yüklediğin güncel TGY projesindeki mevcut IMU sürücüsünü kullanır.
Bu sürücü App/Modules/Sensors/IMU/imu.c içinde ISM330DLC uyumlu
WHO_AM_I 0x6A / 0x6B kontrolü yapmaktadır.

Bu aşamada sürücünün modelini veya ayarlarını değiştirmedim; yalnızca
mevcut, proje içindeki IMU yolunu scheduler'a geri ekledim.

PROJE
-----
CubeIDE adı:
TGY_FULL_INTEGRATION_IMU_NEEDLE_VALVE_V8_4

BAŞLANGIÇ TESTİ
---------------
Motor PSU KAPALI.

1) Import
2) Clean
3) Build
4) Debug
5) Resume

İlk Live Expressions:
v84_imu_connected
v84_imu_device_id
v84_imu_driver_sample_valid
v84_sensor_imu_valid
v84_imu_progress_ok
v84_imu_sample_age_us

v84_imu_task_delta_100ms
v84_valve_health_delta_100ms
v84_valve_tick_delta_100ms

v84_imu_task_last_exec_us
v84_imu_task_max_exec_us
v84_imu_task_overrun_count
v84_imu_task_deadline_miss_count

v84_scheduler_ok
v84_imu_ok
v84_valve_tick_ok
v84_adc_ok
v84_integration_ok

needle_valve_raw_adc
needle_valve_zero_adc
needle_valve_enabled
needle_valve_zero_valid
needle_valve_fault
needle_valve_control_tick_count
needle_valve_hw_adc_raw12
needle_valve_hw_adc_timeout_count

cpu_load_percent
cpu_idle_percent

IMU veri kontrolü:
v84_gyro_x_raw
v84_gyro_y_raw
v84_gyro_z_raw
v84_accel_x_raw
v84_accel_y_raw
v84_accel_z_raw
v84_accel_norm_g
v84_gyro_x_filtered_dps
v84_accel_z_filtered_g

Ayrıca mevcut sürücü Live Expressions:
imu_drv_connected
imu_drv_device_id
imu_dma_start_count
imu_dma_complete_count
imu_dma_error_count
imu_dma_timeout_count
imu_valid_sample_count
imu_invalid_sample_count
sensor_imu_calibration_complete
sensor_imu_calibration_sample_count

BEKLENEN
--------
IMU bağlıysa:
v84_imu_connected            = 1
v84_imu_device_id            = 0x6A veya 0x6B
v84_imu_driver_sample_valid  = 1
v84_imu_progress_ok          = 1
v84_imu_ok                   = 1

Scheduler:
v84_imu_task_delta_100ms       yaklaşık 100
v84_valve_health_delta_100ms   yaklaşık 20
v84_valve_tick_delta_100ms     yaklaşık 20
v84_scheduler_ok               = 1
v84_valve_tick_ok              = 1
v84_adc_ok                     = 1
v84_integration_ok             = 1

Needle valve:
RAW kapalı konumda yaklaşık 1012..1023
FAULT = 0
ADC timeout sabit 0 veya çok düşük
TIM7 tick sürekli artar.

IMU startup calibration:
sensor_imu_calibration_complete ilk anda 0 olabilir.
Kart sabit tutulursa mevcut proje ayarlarına göre kalibrasyon tamamlanınca 1 olur.
v84_sensor_imu_valid bu yüzden startup sırasında geçici 0 olabilir.
Bu, driver-level v84_imu_ok ile ayrı tutulmuştur.

CPU / DEADLINE
--------------
Özellikle:
v84_imu_task_max_exec_us
v84_imu_task_overrun_count
v84_imu_task_deadline_miss_count
cpu_load_percent

değerlerine bak.

IMU 1 kHz taskının periyodu 1000 us, mevcut budget 450 us.
İlk testte budget aşımı görürsek önce ölçüp sonra optimize edeceğiz;
actuator tuning değerlerine dokunmayacağız.

ACTUATOR PROFİL TESTİ
---------------------
Başlangıç coexistence değerleri iyi ise aynı V8.3 USER-button testini çalıştır:

Motor PSU başlangıçta kapalı.

1. basış -> ZERO
2. basış -> ENABLE

Sonra motor PSU düşük akım limitinde aç.

3. basış ->
30% -> 50% -> 80% -> 100% -> 0%

Existing test Live Expressions:
v83_test_auto_active
v83_test_profile_index
v83_test_result

PASS:
v83_test_result = 2
v83_test_profile_index = 5
needle_valve_fault = 0
RAW yaklaşık ZERO
v84_integration_ok = 1

Profil çalışırken herhangi bir USER basışı ABORT + STOP yapar.

SONRAKİ AŞAMA
-------------
V8.4 hem IMU hem actuator ile geçerse V8.5'te barometreyi geri ekleyeceğiz.
