# TARAGAY-T1 P37 — LiDAR / SDIO / ESKF Recovery

P37, P36 UART + SD testinde görülen üç ana arızaya odaklanır:

1. LiDAR I2C recovery sırasında yaklaşık 35 ms task bloklaması.
2. SDIO DATA/COMMAND CRC hatası sonrası logger'ın `ready=1` görünmesine rağmen yazmanın durması.
3. ESKF dikey durumunun sensörlerden kopunca innovation gate nedeniyle yeniden birleşememesi ve kontrol komutunun güvenli olmaması.

## Korunan yollar

- IMU service: 1 kHz.
- IMU runtime recovery: non-blocking state machine.
- ESKF correction/public scheduler: 200 Hz nominal.
- ESKF covariance scheduler: 25 Hz nominal.
- BMP585 barometre: 200 Hz nominal.
- LiDAR service: 1 kHz.
- SD logger: V14, 384 byte/frame, 200 Hz capture, P35 fast CRC.
- SDIO: güvenli 1-bit mod.
- UART: USART2 PA2/PA3, 115200 8N1, DMA TX, RX komutları kapalı.

## P37 LiDAR recovery

`HAL_I2C_IsDeviceReady()` runtime yolundan kaldırılmıştır. Recovery artık aşamalıdır:

- QUIESCE
- DMA_WAIT
- HOST_RESET
- SETTLE
- RESTART

DMA stream'i spin-loop ile beklenmez. Stream disable istenir; bir sonraki 1 kHz service'te EN biti kontrol edilir. EN düşük olduğunda HAL DMA state/flag temizliği yapılır. Böylece I2C/DMA recovery'nin scheduler'ı onlarca ms tutmaması hedeflenir.

UART yeni alanları:

- `lidar_rec_active`
- `lidar_rec_step`
- `lidar_rec_attempts`
- `lidar_rec_success`
- `lidar_rec_failures`
- `lidar_rec_last_us`
- `lidar_rec_max_us`
- `lidar_rec_step_max_us`

## P37 SDIO runtime hata yönetimi

Preflight sırasında SD DMA/CRC hatasında aynı buffer ve aynı sektör adresi tekrar denenir. Retry sınırı aşılırsa SD host yeniden initialize edilir ve mevcut buffer tekrar yazılır.

Uçuş aktifken SD kart yeniden enumerate edilmez. Bu işlem kontrol döngüsünü bloklayabileceği için SD donanımı non-blocking biçimde quiesce edilir, logging kapatılır ve `SYS_FAULT_SD_LOGGING` yayınlanır. Uçuş kontrol zamanlaması SD kurtarma uğruna bloklanmaz.

UART yeni alanları:

- `sd_soft_recoveries`
- `sd_runtime_reinits`
- `sd_runtime_reinit_success`
- `sd_runtime_reinit_failures`
- `sd_runtime_last_error`
- `sd_runtime_rec_count`
- `sd_runtime_rec_success`
- `sd_runtime_rec_failures`
- `sd_runtime_rec_flight_aborts`
- `sd_runtime_rec_last_us`
- `sd_runtime_rec_max_us`

## P37 ESKF divergence / reacquisition güvenliği

Dikey ESKF, barometre ve LiDAR'ın birbirleriyle tutarlı olduğu halde nominal dikey konumdan büyük ölçüde ayrıldığını doğrularsa:

1. `output_inhibited=1` olur.
2. RCS ESKF kontrol yolu güvenli duruma zorlanır.
3. Ana/needle generated control fiziksel çıkışı güvenli komuta zorlanır.
4. Sadece PZ/VZ dikey alt durumu sensör hedefiyle yeniden hizalanır.
5. Barometre + LiDAR tekrar tutarlı ve küçük innovation ile belirli süre stabil olmadan inhibit kaldırılmaz.

LiDAR mevcut değilse barometre-only hard fallback yalnız filtre >10 m ve baro hatası >5 m olduğunda kullanılabilir; çıkış yine LiDAR geri dönüp doğrulamadan serbest bırakılmaz.

Yeni SystemMonitor fault'ları:

- `16 = SYS_FAULT_ESKF_DIVERGENCE`
- `17 = SYS_FAULT_SD_LOGGING`

UART yeni ESKF alanları:

- `eskf_inhibit`
- `eskf_reacquire`
- `eskf_div_reason`
- `eskf_div_count`
- `eskf_reacq_count`
- `eskf_div_candidate`
- `eskf_reacq_stable`
- `vertical_consistency_mm`
- `vertical_reacq_target_mm`

## IMU güvenilirlik ayarı

P36'daki direct-register software SPI korunmuştur. Edge delay 8 NOP'tan 10 NOP'a çıkarılmıştır. Amaç recovery/redundant reject oranını düşürmek; 1 kHz task bütçesinin korunması UART ile doğrulanmalıdır.

## UART

P37 `$TGY55` frame yapısı korunur ancak alan sayısı **206** olmuştur. Buffer 3072 byte'a çıkarılmıştır. P36 monitor 178 alan beklediği için P37 ile kullanılmamalıdır.

Kullanım:

```powershell
python monitor_uart_p37.py --port COM8 --json > uart_p37.txt
```

## İlk test kriterleri

Gaz / basınç / fiziksel aktüatör bağlantısı olmadan 120–180 saniye test önerilir.

- `task_imu_max_us`: tercihen < 600 us.
- `task_lidar_max_us`: hedef < 1000 us; P36'daki ~35 ms tekrar etmemeli.
- `scheduler_realigns`: sabit veya çok seyrek artmalı.
- `eskf_public_count`: yaklaşık 200 Hz.
- `cov_count`: yaklaşık 25 Hz.
- `baro_updates`: yaklaşık 200 Hz.
- `sd_frames`: yaklaşık 200 Hz.
- `sd_dropped = 0`, `sd_ring_overruns = 0` sağlıklı testte korunmalı.
- SD CRC hatası olursa preflight'ta recovery sayaçları artmalı ve `sd_frames` tekrar ilerlemeli.
- `eskf_inhibit` normal durumda 0 kalmalı. Divergence oluşursa fiziksel kontrol komutları güvenli hale geçmelidir.

## Doğrulama notu

Bu paket host tarafında C syntax ve statik/protokol validator testlerinden geçirilmiştir. Bu ortamda `arm-none-eabi-gcc` bulunmadığından gerçek STM32 ARM link/build yapılmamıştır. CubeIDE'de `Clean Project -> Build Project -> Flash` yapılmalıdır.
