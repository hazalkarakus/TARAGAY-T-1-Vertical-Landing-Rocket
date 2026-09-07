# TARAGAY-T1 P32 — UART-only CPU / Scheduler / SD Recovery

Bu sürüm P31 UART kaydında görülen üç ana probleme odaklanır:

- `cpu_x100 ~= 9680..9708` (yaklaşık %97 CPU)
- `scheduler_realigns` sayacının yaklaşık her 100 ms'de +30 artması
- `sd_ready=0`, `sd_logging=0`

## P32 değişiklikleri

1. **1 kHz SD yükü kaldırıldı**
   - P31'de `Task_IMU_1kHz()` her 1 ms'de `SDLogger_PublishSources()` çağırıyordu.
   - Bu fonksiyon yüzlerce alanı float->integer dönüştürüyor ve büyük snapshot hazırlıyordu.
   - P32'de 1 kHz yol yalnız `SDLogger_PushFastIMU()` çağırır ve sadece 5 örneklik ham IMU geçmişini günceller.
   - Ağır/coherent SD snapshot artık `Task_FullESKFCorrection_200Hz()` içinde 200 Hz hazırlanır.
   - SD mount/logging başarısızsa her iki fonksiyon da hemen çıkar; başarısız SD yolu 1 kHz kontrol görevini tüketmez.

2. **ESKF korunuyor**
   - Nominal predict: 1 kHz.
   - Public/correction: gerçek 200 Hz.
   - Ağır covariance: 25 Hz.
   - ESKF frekansı düşürülmedi.

3. **Legacy attitude debug aynası 200 Hz'e indirildi**
   - AttitudeEstimator matematiği 1 kHz kalır.
   - Yalnız eski Live-Expressions volatile kopyaları rutin durumda 200 Hz güncellenir.
   - Hata/reset yolları debug durumunu anında yayımlar.

4. **SD boot recovery**
   - SD init yalnız scheduler başlamadan önce yapılır.
   - En fazla 3 tam `SDLogger_Init()` denemesi vardır.
   - Denemeler arasında 250 ms vardır.
   - Scheduler başladıktan sonra bloklayan mount/re-init retry yoktur.

5. **UART-only teşhis genişletildi**
   - CPU toplam yükü ve IMU/BARO/LIDAR/NRF/ESKF görev yükleri.
   - Her görevin son/maksimum çalışma süresi ve deadline miss sayısı.
   - SD mount/file/result/disk/HAL/retry/drop/overrun/timeout/frame sayaçları.
   - nRF status/config/channel/RF setup/FIFO/error/invalid/IRQ alanları.

6. **Scheduler health LiDAR oranı düzeltildi**
   - Scheduler'da LiDAR görevi 5 ms = 200 Hz.
   - 10 Hz monitor artık 100 ms'de 15..25 çağrı bekler; eski 80..120 beklentisi kaldırıldı.

## İlk UART testi

Kart resetlendikten sonra:

```powershell
py monitor_uart_v55.py --port COMxx --json
```

Önce 30-60 saniye yalnız aviyonik test yap. Motor ve solenoidleri bağlı tutma.

### Ana başarı kriterleri

- `eskf_public_count`: 10 saniyede yaklaşık +2000.
- `eskf_public_age_ms`: tipik 0..5 ms.
- `cpu_x100`: P31'deki ~9700 değerinden belirgin şekilde aşağı inmeli. İlk hedef <6000 (%60).
- `scheduler_realigns`: P31'deki +30/100 ms paterni bitmeli; ideal olarak uzun süre sabit veya çok seyrek artmalı.
- `sd_ready=1`, `sd_logging=1`.
- `sd_frames` sürekli artmalı.
- `sd_write_errors=0`, `sd_dropped=0`, `sd_ring_overruns=0`, `sd_async_timeouts=0`.

SD yine başlamazsa UART'tan özellikle şu alanları gönder:

- `sd_last_result`
- `sd_disk_status`
- `sd_mount_retries`
- `sd_hal_init`
- `sd_wide`
- `sd_host_attempts`
- `sd_host_resets`
- `sd_hal_error`
- `sd_last_hal_error`
- `sd_recovered`

Bunlar kart/SDIO/FatFS problemini ayırmak için yeterlidir.

### CPU kaynağını ayırma

- `cpu_imu_x100`
- `cpu_baro_x100`
- `cpu_lidar_x100`
- `cpu_nrf_x100`
- `cpu_eskf_x100`

ve süreler:

- `task_imu_us`, `task_imu_max_us`
- `task_baro_us`, `task_baro_max_us`
- `task_lidar_us`, `task_lidar_max_us`
- `task_nrf_us`, `task_nrf_max_us`
- `task_eskf_us`, `task_eskf_max_us`

Bu alanlarla ikinci optimizasyon gerekirse doğrudan hangi göreve dokunacağımız belli olur.

## nRF

`nrf_connected=1` yalnız SPI/register yapılandırmasının doğru olduğunu söyler. RF paket alımı için:

- `nrf_link=1`
- `nrf_rx_count` artıyor
- `nrf_errors=0`
- `nrf_invalid` mümkünse 0

beklenir. Paket yoksa UART'taki `nrf_status`, `nrf_config`, `nrf_channel`, `nrf_rf_setup`, `nrf_fifo` alanları kullanılacaktır.
