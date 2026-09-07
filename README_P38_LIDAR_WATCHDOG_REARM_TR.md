# TARAGAY-T1 P38 — LiDAR watchdog re-arm

P38, P37 üzerinde yalnız LiDAR acquisition/recovery regresyonunu düzeltir.

## Değişiklikler

- Recovery sırasında eski `last_sample_timestamp_us` temizlenir.
- Stale watchdog, recovery/startup sonrasında ilk gerçek geçerli mesafe örneği gelene kadar kapalıdır.
- Recovery success artık DMA komutunun `HAL_OK` dönmesi değil, recovery sonrası ilk geçerli LiDAR sample'ın yayınlanmasıdır.
- İlk sample 150 ms içinde gelmezse yeni non-blocking recovery başlatılır.
- `Lidar_TryConnect()` sonrası ikinci kez `Lidar_StartMeasurementDMA()` çağrılması kaldırılmıştır.
- P37 IMU non-blocking recovery, SD 200 Hz fast-CRC/runtime recovery, ESKF divergence inhibit/reacquisition ve scheduler düzeni korunmuştur.
- UART alan sayısı değişmemiştir: 206.

## Hedef

- `lidar_updates` tekrar artmalı.
- `lidar_rec_success` yalnız gerçek recovery sonrası sample geldiğinde artmalı.
- `task_lidar_max_us < 1000 us`, `scheduler_realigns ~= 0`.
- LiDAR fiziksel update oranı mümkünse 150–200 Hz bandına dönmeli.
