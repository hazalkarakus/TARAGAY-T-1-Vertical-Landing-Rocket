TGY FULL INTEGRATION - IMU + BMP585 + LIDAR + NEEDLE V8.6
================================================================

V8.5 SONUCU
-----------
IMU + BMP585 + gerçek actuator dinamik profili birlikte PASS:
- actuator profile index = 5
- actuator result = 2 PASS
- IMU OK
- BMP585 OK
- valve exact 200 Hz OK
- barometer 200 Hz OK
- overrun = 0
- deadline miss = 0
- integration_ok = 1
- CPU load yaklaşık %28

V8.6'DA EKLENEN
---------------
LIDAR-Lite sürücüsü I2C2 üzerinde DMA ile geri açıldı.

Mevcut proje sürücüsü:
- I2C2 = 400 kHz
- PB10 = SCL
- PB11 = SDA
- address = 0x62
- DMA1 Stream2 RX
- DMA1 Stream7 TX
- 200 Hz Lidar_Update state-machine task
- measurement wait = 5 ms

TIM7 needle-valve IRQ priority = 5.
I2C/DMA interrupts priority = 8.
Dolayısıyla needle-valve 200 Hz IRQ daha yüksek önceliktedir.

AKTİF
-----
Task 0: IMU 1 kHz
Task 1: BMP585 + valve health 200 Hz
Task 2: LIDAR 200 Hz
Task 3: integration monitor 10 Hz

Needle local controller:
TIM7 exact 200 Hz interrupt.

HALA KAPALI
-----------
- NRF / RemoteControl
- SD logger
- attitude estimator
- Vertical EKF / FullState ESKF
- servo
- active RCS control

İLK TEST
--------
Motor PSU KAPALI.

Import -> Clean -> Build -> Debug -> Resume.

LIDAR Live Expressions:
v86_lidar_initialized
v86_lidar_connected
v86_lidar_data_ready
v86_lidar_distance_valid
v86_lidar_progress_ok
v86_lidar_ok

v86_lidar_distance_cm
v86_lidar_distance_m
v86_lidar_median_distance_m
v86_lidar_filtered_distance_m

v86_lidar_update_count
v86_lidar_update_delta_100ms
v86_lidar_read_count
v86_lidar_error_count
v86_lidar_timeout_count

v86_lidar_dma_tx_start_count
v86_lidar_dma_tx_complete_count
v86_lidar_dma_rx_start_count
v86_lidar_dma_rx_complete_count
v86_lidar_dma_error_count
v86_lidar_dma_busy_count

v86_lidar_last_sample_interval_us
v86_lidar_min_sample_interval_us
v86_lidar_max_sample_interval_us
v86_lidar_sample_age_us

v86_lidar_probe_attempt_count
v86_lidar_probe_success_count
v86_lidar_probe_failure_count
v86_lidar_bus_recovery_count
v86_lidar_last_i2c_error_code

v86_lidar_task_delta_100ms
v86_lidar_task_max_exec_us
v86_lidar_task_overrun_count
v86_lidar_task_deadline_miss_count

COEXISTENCE:
v86_scheduler_ok
v86_imu_ok
v86_baro_ok
v86_lidar_ok
v86_valve_tick_ok
v86_adc_ok
v86_integration_ok
v86_valve_tick_delta_100ms

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
cpu_task2_load_percent

BEKLENEN - LIDAR
----------------
İlk 100 ms içinde connected hemen 1 olmayabilir; sürücü startup probe'u
100 ms gecikmeli başlatıyor.

Sonrasında:
initialized          = 1
connected            = 1
data_ready           = 1
distance_valid       = 1
progress_ok          = 1
lidar_ok             = 1

LIDAR task / 100 ms:
v86_lidar_task_delta_100ms ~20

Gerçek sample update_delta_100ms ölçüm zamanına bağlı olarak task delta'dan
daha düşük olabilir. Önemli olan update_count'ın sürekli artması ve
sample_age'in düşük kalmasıdır.

DMA:
TX/RX start ve complete sayaçları sürekli artmalı.
dma_error ve timeout idealde 0.

ÖNEMLİ KALİBRASYON NOTU
-----------------------
app_config içinde LIDAR startup calibration referansı 9.00 m'dir.

Masa üzerinde LIDAR 9 m'ye bakmıyorsa:
v86_lidar_calibration_complete = 0
kalması NORMAL olabilir.

V8.6 integration_ok bunu şart koşmaz.
Sürücü kalibrasyon bitene kadar gerçek sensör mesafesini offset uygulamadan
yayınlamaya devam eder.

ZAMANLAMA HEDEFİ
----------------
IMU task delta /100ms    ~100
Baro task delta /100ms   ~20
LIDAR task delta /100ms  ~20
Valve TIM7 /100ms        ~20

IMU overrun/deadline miss = 0
Baro overrun/deadline miss = 0

LIDAR steady-state task max süresini ölç.
Startup/reconnect sırasında HAL_I2C_IsDeviceReady nedeniyle tekil overrun
oluşabilir; sürekli artmamalıdır.

Final:
v86_scheduler_ok   = 1
v86_imu_ok         = 1
v86_baro_ok        = 1
v86_lidar_ok       = 1
v86_valve_tick_ok  = 1
v86_adc_ok         = 1
v86_integration_ok = 1

ACTUATOR DİNAMİK YÜK TESTİ
--------------------------
Statik coexistence PASS olduktan sonra mevcut USER-button profilini tekrar:

1) ZERO
2) ENABLE
motor PSU düşük akım limitinde aç
3) 30 -> 50 -> 80 -> 100 -> 0

Live:
v83_test_auto_active
v83_test_profile_index
v83_test_result

PASS:
v83_test_profile_index = 5
v83_test_result = 2
v86_integration_ok = 1
v86_lidar_ok = 1
needle_valve_fault = 0
RAW ~= ZERO

SONRAKİ AŞAMA
-------------
V8.6 geçerse:
- LIDAR gerçek sample cadence / filter timing'i değerlendir
- ardından SD logging'i geri ekle
- estimator katmanlarını en son kademeli aç
