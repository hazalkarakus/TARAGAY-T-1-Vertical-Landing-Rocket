# P36 — Non-blocking IMU Recovery + LiDAR 1 kHz + 200 Hz ESKF

P36, P35 SD/CRC yolunu aynen korur. Degisikliklerin merkezi zamanlamadir.

- Runtime IMU recovery artik HAL_Delay kullanmaz. Reset polling ve 40 ms sensor settle, 1 kHz task icinde state-machine deadline'lariyla ilerler.
- Recovery sirasinda IMU gecersiz kalir; flight stack eski IMU'yu valid olarak kullanmaz.
- LiDAR state-machine service 1 kHz'e dondu. Measurement fusion her yeni valid sample'da yapilir.
- ESKF public correction 5 ms / 200 Hz kalir ve scheduler'da LiDAR'dan daha yuksek onceliklidir.
- 25 Hz covariance artik App_Run background slack firsati beklemez; ayri 40 ms scheduler task'idir.
- Freshness ve remote background servisleri IMU deadline'ina cok yakin baslatilmaz.
- SD V14 / 384 byte / 200 Hz ve P35 fast CRC degismemistir.

## UART P36 yeni IMU alanlari

imu_recovery_state, imu_recovery_step, imu_recovery_count,
imu_recovery_attempts, imu_recovery_failures, imu_recovery_last_us,
imu_recovery_max_us, imu_stale_count, imu_pattern_errors,
imu_invalid_samples, imu_redundant_rejects

Toplam UART alan sayisi: 178. monitor_uart_p36.py kullanin.

## Test hedefi

- task_imu_us normalde ~200-350 us; task_imu_max_us artik 40+ ms olmamali.
- scheduler_realigns ve miss_imu cok seyrek/artmiyor olmali.
- eskf_public_count ~2000 / 10 s.
- cov_count ~250 / 10 s.
- baro_updates ~2000 / 10 s.
- LiDAR update fiziksel sensor dongusune bagli olarak ~180-200 Hz hedef.
- sd_dropped=0, sd_ring_overruns=0, sd_frames ~2000 / 10 s korunmali.
