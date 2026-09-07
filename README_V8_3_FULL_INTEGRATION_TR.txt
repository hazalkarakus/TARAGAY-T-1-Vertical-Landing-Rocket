TGY FULL INTEGRATION + NEEDLE VALVE V8.3
========================================

AMAÇ
----
V8.2 gerçek actuator otomatik profili PASS olduktan sonra ilk kontrollü
entegrasyon adımıdır. Bu proje ORİJİNAL TGY_FULL_SENSOR_NRF_SERVO_90
projesini taban alır.

BU ADIMDA AKTİF
----------------
- Orijinal Core/main.c ve CubeMX peripheral init
- Board + TIM2 micros() timebase
- BusManager init
- Orijinal cooperative Scheduler + CPU load monitor
- 1 kHz integration background task
- 200 Hz scheduler health task
- 10 Hz integration monitor
- PC1 / ADC1_IN11 needle feedback
- TIM7 deterministic 200 Hz needle local controller
- TIM3 PB0/PB1 BTS7960 PWM
- PC4/PC5 BTS enable
- RCS relay outputs safe OFF
- USER button integration test

BU ADIMDA BİLEREK PASİF
-----------------------
- IMU
- Barometer
- Lidar
- NRF
- SD logger
- ESKF / Vertical EKF / Attitude estimator
- RCS attitude controller
- Servo

Bunlar bir sonraki sürümlerde tek tek geri eklenecek. Böylece hangi modül
actuator zamanlamasını veya belleğini bozarsa anında yakalanacak.

PROJE ADI
----------
TGY_FULL_INTEGRATION_NEEDLE_VALVE_V8_3

KABLO
-----
Pot +     -> 3.3V
Pot GND   -> GND
Pot wiper -> PC1
PB0 -> LPWM OPEN
PB1 -> RPWM CLOSE
PC4 -> L_EN
PC5 -> R_EN
BTS VCC -> 5V
GND ortak

BUILD
-----
1) Eski debug session Terminate.
2) ZIP'i ayrı klasöre çıkar.
3) Import Existing Projects into Workspace.
4) Clean + Build.
5) Debug + Resume.

MOTOR PSU İLK BAŞTA KAPALI.

LIVE EXPRESSIONS - ENTEGRASYON
------------------------------
v83_fast_task_counter
v83_valve_health_task_counter
v83_monitor_task_counter
v83_scheduler_ok
v83_valve_tick_ok
v83_adc_ok
v83_integration_ok
v83_valve_tick_delta_100ms
v83_adc_timeout_count
v83_valve_fault_snapshot

cpu_load_percent
cpu_idle_percent
cpu_task0_load_percent
cpu_task1_load_percent
cpu_task2_load_percent

LIVE EXPRESSIONS - ACTUATOR
---------------------------
needle_valve_raw_adc
needle_valve_zero_adc
needle_valve_target_adc
needle_valve_error_adc
needle_valve_enabled
needle_valve_zero_valid
needle_valve_lock
needle_valve_fault
needle_valve_rpwm
needle_valve_lpwm
needle_valve_control_tick_count
needle_valve_hw_adc_raw12
needle_valve_hw_adc_mv
needle_valve_hw_adc_timeout_count

LIVE EXPRESSIONS - TEST
-----------------------
v83_test_button_stage
v83_test_auto_active
v83_test_profile_index
v83_test_profile_cmd_x10000
v83_test_result
v83_test_leg_final_raw
v83_test_leg_final_error
v83_test_leg_settle_ms

İLK KONTROL
-----------
Motor PSU OFF iken yaklaşık:
RAW          = 1012..1023
RAW12        = 4050..4095
FAULT        = 0
ENABLED      = 0
ZERO_VALID   = 0
TIM7 delta   ~20 / 100 ms
v83_scheduler_ok    = 1
v83_valve_tick_ok   = 1
v83_adc_ok          = 1
v83_integration_ok  = 1

BUTON TESTİ
-----------
Motor PSU OFF:
1. basış -> ZERO
2. basış -> ENABLE

Sonra motor PSU düşük akım limitiyle açılır.
3. basış -> otomatik profil:
30% -> 50% -> 80% -> 100% -> 0%

Profil sırasında herhangi bir USER buton basışı = ABORT + STOP.

PASS:
v83_test_result       = 2
v83_test_profile_index= 5
v83_test_auto_active  = 0
needle_valve_enabled  = 0
needle_valve_fault    = 0
RAW ~= ZERO
v83_integration_ok    = 1

SONRAKİ ADIM
------------
Bu test PASS ise V8.4'te IMU'yu gerçek 1 kHz göreviyle geri ekleyeceğiz.
Needle TIM7 200 Hz yapısı değişmeyecek.

NOT
---
CubeMX'ten bu bring-up aşamasında regenerate yapma. PC1/TIM3/TIM7 actuator
hardware'i uygulama kodundan güvenli şekilde yapılandırılıyor.
