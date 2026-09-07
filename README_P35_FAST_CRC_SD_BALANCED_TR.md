# P35 - Fast CRC + SD Balanced Runtime

P34 UART testinde IMU/scheduler miss davranisi belirgin iyilesti ancak SD ring,
bit-bit CRC16 maliyeti ve asiri background defer nedeniyle tasiyordu.

P35 degisiklikleri:
- UART CRC16-CCITT tablo tabanli 256-entry lookup. CRC sonucu/protokol ayni.
- SD frame CRC16-CCITT tablo tabanli lookup. flight.bin formati ayni.
- SD drain: max 4 frame/pass, 320 us budget.
- SD IMU slack threshold: 220 us.
- UART slack threshold: 300 us.
- Scheduler slow task ayni IMU generation icinde tekrar tekrar kontrol edilmiyor.
- ESKF public 200 Hz, baro 200 Hz, LiDAR hedefi, covariance decimation ve SD 200 Hz capture korunur.

Beklenen UART:
- sd_dropped ve sd_ring_overruns sabit 0
- sd_frames ~ +2000 / 10 s
- eskf_public_count ~ +2000 / 10 s
- baro_updates ~ +2000 / 10 s
- scheduler_realigns ve miss_* cok seyrek / sabit
- bg_uart_us ve sd_drain_us P34'e gore ciddi dusuk
