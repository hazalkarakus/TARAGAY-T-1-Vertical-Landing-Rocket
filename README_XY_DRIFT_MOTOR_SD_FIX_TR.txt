TARAGAY T1 - XY DRIFT + MOTOR SD LOG FIX
=========================================

TABAN
-----
Bu klasor, kullanicinin orijinal calisan TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ_RF250K(4)
klasoru + onceki XY-only rail guard uzerinden uretilmistir.

FIRMWAREDE DEGISEN SADECE 2 KAYNAK DOSYA
----------------------------------------
1) App/Modules/Estimation/FullStateESKF/full_state_eskf.c
2) App/Services/SDLogger/sd_logger.c

LIDAR, barometre, Z ekseni, IMU surucusu, RCS, motor kontrol algoritmasi,
NRF, PE9 ve gorev akisi degistirilmedi.

XY DUZELTMESI
-------------
- Gercek 15-state ESKF nominal X/Y state ve covariance yapisi korunur.
- horizontal_position_valid yine 0 kalir; yani X/Y mutlak konum olarak GNC'ye
  yetkili hale getirilmez.
- Public/SD icin ayri bir bounded diagnostic X/Y projection kullanilir.
- Cok yavas horizontal ivme bias/gravity leakage high-pass ile bastirilir.
- Velocity/position icin cok yumusak damping ve fizik disi rail korumasi vardir.
- Z, LIDAR, baro, quaternion ve attitude bu projeksiyondan etkilenmez.

MOTOR SD KAYDI
--------------
- Ground/preflight normal V14 kaydi aynen 30 Hz devam eder.
- PE9 sonrasi SDIO yazisi yine durur; motor/RCS EMI guvenlik mimarisi BOZULMADI.
- Flight verisi CCM RAM'e yazilir ve STOP/touchdown sonrasi mevcut quiet gate ile
  SD'ye replay edilir.
- Background flight log mevcut 7.5 Hz decimation ile devam eder.
- Motor/homing/P110 hareketi aktifken V16 motor frame'i HER TIM5 tick'te, yani
  30 Hz yakalanir. Motor durunca background rate'e geri doner.
- Boylece ADC, target, RPWM/LPWM, P83 feedback, P110 state/speed/result ve turn
  bilgisi motor hareketi boyunca 7.5 Hz yerine 30 Hz olur.

OKUYUCU
-------
sd_flight_reader_motor_v16.py guncellendi.
flight_motor.csv icine zero_valid olmasa bile hareket baslangic ADC'sinden:
- motor_motion_reference_adc
- motor_motion_delta_adc
- motor_motion_turns_est
- motor_feedback_adc
alanlari eklenir.

NOT
---
X/Y icin GPS/UWB/optical-flow gibi mutlak yatay sensor yoktur. Bu nedenle yeni
X/Y, uzun sureli mutlak navigasyon degil, drift'i bastirilmis diagnostic veridir.
