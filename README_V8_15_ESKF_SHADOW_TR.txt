TGY V8.15 - FULL-STATE ESKF SHADOW INTEGRATION
=================================================

AMAÇ
----
Çalışan sensör/aktüatör/SD/NRF altyapısını değiştirmeden Full-State ESKF'yi
gerçek STM32 üzerinde SHADOW observer olarak çalıştırmak.

ÇOK ÖNEMLİ
----------
ESKF hiçbir aktüatörü sürmez.

ESKF -> Needle        YOK
ESKF -> Servo         YOK
ESKF -> RCS solenoid  YOK

Bu sürüm yalnız estimator doğrulamasıdır.

AKTİF ESTIMATOR MİMARİSİ
------------------------
Her yeni IMU sample:
SensorManager_UpdateIMU()
    ->
AttitudeEstimator_Update()
    ->
FullStateESKF_Predict()
    ->
SDLogger_PublishSources()

Nominal propagation:
1000 Hz

Public ESKF output:
APP_FULL_ESKF_EULER_DECIMATION = 5
1000 / 5 = 200 Hz

Correction service:
200 Hz

Covariance propagation:
APP_FULL_ESKF_COVARIANCE_DECIMATION = 20
1000 / 20 = 50 Hz

Gravity correction:
200 / 4 = 50 Hz

Yani daha önce seçilen mimari:
- nominal propagation 1000 Hz
- ESKF output/control-ready state 200 Hz
- covariance 50 Hz

aynen uygulanmıştır.

SHADOW MODE
-----------
APP_FULL_ESKF_SHADOW_MODE = 1

Estimator çıktıları yalnız:
- Live Expressions
- flight.bin

üzerinden incelenir.

SD LOGGER
---------
Mevcut V13 frame zaten FullStateESKF alanlarını içeriyordu.
Bu yüzden frame formatını değiştirmedik.

Estimator aktif olduktan sonra aynı V13 BIN içinde:
- position XYZ
- velocity XYZ
- quaternion
- roll/pitch/yaw
- accel bias XYZ
- gyro bias XYZ
- world linear acceleration
- estimator flags
- predict/public/covariance counters
- DWT execution profiler

gerçek veri olarak kaydolur.

BARO TAKILI DEĞİLSE
-------------------
Full-State ESKF yine IMU + LIDAR ile shadow test edilebilir.

Bu durumda:
v815_eskf_baro_reference_ready = 0
normaldir.

Bench PASS için:
v815_eskf_shadow_core_ok

barometreyi şart koşmaz.

Tam uçuş entegrasyon:
v815_integration_ok

barometre dahil tam sistemi şart koşar.

İLK TEST
--------
Motor PSU kapalı olabilir.
Roket/aviyonik mümkün olduğunca hareketsiz dursun.

Clean -> Build -> Debug -> Resume.

Önce en az 5-10 saniye hiç hareket ettirme.

LIVE EXPRESSIONS - ESTIMATOR
----------------------------
v815_attitude_ok

v815_eskf_initialized
v815_eskf_healthy
v815_eskf_shadow_mode

v815_eskf_lidar_reference_ready
v815_eskf_baro_reference_ready
v815_eskf_stationary_detected

v815_attitude_delta_100ms
v815_eskf_predict_delta_100ms
v815_eskf_public_delta_100ms
v815_eskf_covariance_delta_100ms
v815_eskf_correction_delta_100ms

v815_eskf_rate_ok
v815_eskf_shadow_core_ok

EXECUTION:
v87_imu_task_max_exec_us
v87_imu_task_overrun_count
v87_imu_task_deadline_miss_count

v815_eskf_correction_task_max_exec_us
v815_eskf_correction_task_overrun_count
v815_eskf_correction_task_deadline_miss_count

v815_eskf_predict_max_exec_us
v815_eskf_correction_max_exec_us

HEALTH:
v815_eskf_numerical_error_count
v815_eskf_gap_skip_count
v815_eskf_reset_count

STATE:
v815_eskf_position_x_m
v815_eskf_position_y_m
v815_eskf_position_z_m

v815_eskf_velocity_x_mps
v815_eskf_velocity_y_mps
v815_eskf_velocity_z_mps

v815_eskf_roll_deg
v815_eskf_pitch_deg
v815_eskf_yaw_deg

v815_eskf_gyro_bias_x_dps
v815_eskf_gyro_bias_y_dps
v815_eskf_gyro_bias_z_dps

SYSTEM:
cpu_load_percent
cpu_idle_percent

v87_scheduler_ok
v87_imu_ok
v87_lidar_ok
v89_nrf_hw_ok
v810_servo_ok

BEKLENEN RATE
-------------
100 ms pencerede:

attitude delta       ~= 100
ESKF predict         ~= 100
ESKF public output   ~= 20
ESKF correction      ~= 20
ESKF covariance      ~= 5

v815_eskf_rate_ok = 1

BEKLENEN STARTUP
----------------
İlk 1-3 saniye bootstrap/reference süreci normaldir.

Sonrasında:
attitude_ok                 = 1
eskf_initialized            = 1
eskf_healthy                = 1
eskf_shadow_mode            = 1
eskf_lidar_reference_ready  = 1

Roket hareketsizse birkaç saniye sonra:
eskf_stationary_detected = 1
beklenir.

Gyro bootstrap 200 Hz correction service içinde çalıştığı için startup bias
kilidinin yaklaşık birkaç saniye sürmesi normaldir.

BARO yoksa:
baro_reference_ready = 0 normal.

HAREKETSİZ BENCH STATE
----------------------
Referans alındıktan sonra yaklaşık:

position Z   ~ 0 m
velocity XYZ ~ 0 m/s

roll/pitch sensörün gerçek eğimine yakın.

Yaw:
magnetometer/heading aid kullanılmadığı için relative yaw'dır ve zamanla drift
edebilir. Bu FAIL değildir.

PASS KRİTERİ
------------
v815_eskf_shadow_core_ok = 1

ve:
numerical_error_count = 0
IMU task overrun = 0
ESKF correction task overrun = 0

CPU'nun hâlâ yeterli idle marjı kalmalı.

İLK TESTTE MOTOR ÇALIŞTIRMA
---------------------------
Önce estimator'ın hareketsiz bench testi geçsin.

Ardından ikinci test:
- sistemi elle roll/pitch hareket ettir
- roll/pitch/yaw yönlerini doğrula
- position/velocity davranışını izle

Bunlar da geçerse üçüncü adımda:
Needle + NRF + servo + SD yükü altında ESKF shadow endurance testi yapılır.

SONRAKİ
-------
V8.15 PASS:
1) estimator eksen/yön doğrulaması
2) barometreyi yeniden takıp Baro + LIDAR Z fusion doğrulaması
3) kontrollü hareket logu
4) ancak daha sonra ESKF output -> GNC/control bağlantısı
